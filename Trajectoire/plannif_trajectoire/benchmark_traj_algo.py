# benchmark_traj_algo.py
import numpy as np
import time
import statistics as stats
import random
from tqdm import tqdm
import logging
from datetime import datetime
import os
import matplotlib
matplotlib.use("Agg")   # backend non interactif, orienté fichiers
import matplotlib.pyplot as plt
import csv
import multiprocessing as mp
import traceback

import config_traj

# ---- Import tes algos (adapte selon tes noms de fichiers) ----
from shortest_path_algorithms import plan_nearest_neighbor, plan_optimal_bruteforce, plan_cheapest_insertion, plan_greedy_then_swap_improve, plan_random_restart_greedy
from shortest_path_algorithms import plan_exact_tsp, plan_heuristic_tsp
from shortest_path_algorithms import plan_bnb_basic, plan_bnb_heuristic


# ---- Import helpers pour scorer (coût total) ----
from other_fct_traj import output_pos_for_color, cost_do_bloc_from

# Pour Windows multiprocessing
mp.freeze_support()

def _algo_worker(q, algo_name, blocs_list, start_pos):
    """
    Worker TOP-LEVEL (picklable). Exécute l'algo demandé.
    """
    try:
        import time
        import shortest_path_algorithms as spa

        # Rebuild blocs en list[tuple]
        blocs = [tuple(b) for b in blocs_list]

        algo_map = {
            "Exact TSP DP": spa.plan_exact_tsp,
            "Heuristic TSP": spa.plan_heuristic_tsp,
            "Branch and Bound Basic": spa.plan_bnb_basic,
            "Branch and Bound Heuristic": spa.plan_bnb_heuristic,
            "Optimal brute-force": spa.plan_optimal_bruteforce,
            "Nearest neighbor O(n^2)": spa.plan_nearest_neighbor,
            "Cheapest insertion": spa.plan_cheapest_insertion,
            "Nearest neighbor + swap": spa.plan_greedy_then_swap_improve,
            "Random Restart Greedy": spa.plan_random_restart_greedy,
            "Regret-3 + VND": spa.plan_regret3_plus_vnd,
            "Lookahead(L=8,k=2) + VND": spa.plan_lookahead_L8_k2_plus_vnd,
            "GRASP(R=200) + VND": lambda blocs, start_pos: spa.plan_grasp_vnd(blocs, start_pos, R=200, rcl_size=8, seed=123, vnd_loops=20),

        }

        fn = algo_map[algo_name]

        t0 = time.perf_counter()
        res = fn(blocs, start_pos)
        t1 = time.perf_counter()
        q.put(("ok", res, t1 - t0, None))

    except Exception as e:
        q.put(("err", None, None, f"{repr(e)}\n{traceback.format_exc()}"))

def run_algo_with_timeout(algo_name, blocs, start_pos, timeout_sec):
    """
    Exécute un ALGO (par nom) dans un process séparé.
    blocs: np.array(dtype=object) ou list
    start_pos: tuple(x,y) ou ce que tu utilises
    """
    ctx = mp.get_context("spawn")   # Windows
    q = ctx.Queue()

    # Convertir blocs en structure picklable
    if hasattr(blocs, "tolist"):
        blocs_list = blocs.tolist()
    else:
        blocs_list = list(blocs)

    p = ctx.Process(target=_algo_worker, args=(q, algo_name, blocs_list, start_pos))
    p.start()
    p.join(timeout_sec)

    if p.is_alive():
        p.terminate()
        p.join()
        return (False, None, timeout_sec, True, "timeout")

    if q.empty():
        return (False, None, 0.0, False, "no_result")

    status, res, elapsed, err = q.get()
    if status == "ok":
        return (True, res, elapsed, False, None)
    return (False, None, 0.0, False, err)

def clean(vals):
    """Garde seulement les floats/ints valides (pas NaN/inf)."""
    out = []
    for v in vals:
        if v is None:
            continue
        try:
            fv = float(v)
        except Exception:
            continue
        if np.isfinite(fv):
            out.append(fv)
    return out

def total_cost_for_order(order, start_pos):
    """Calcule la distance totale selon TON modèle (pos -> bloc -> sortie)."""
    pos = start_pos
    total = 0.0
    for b in order:
        total += cost_do_bloc_from(pos, b)
        pos = output_pos_for_color(b[0])
    return total

def time_one(algo_name, blocs, start_pos, repeats=10, warmup=1, timeout_sec=None):
    """
    Mesure le temps d'exécution d'un algo (par nom) avec watchdog.
    Return: (best_time, mean_time, stdev_time, last_result)
    """
    last_result = None
    times = []

    # warmup
    for _ in range(warmup):
        if timeout_sec is None:
            # pas recommandé sur Windows si tu veux pouvoir kill
            raise ValueError("timeout_sec doit être défini sur Windows pour watchdog")
        ok, res, elapsed, to, err = run_algo_with_timeout(algo_name, blocs, start_pos, timeout_sec)
        if not ok:
            raise TimeoutError(f"Warmup failed: {'timeout' if to else err}")
        last_result = res

    for _ in range(repeats):
        ok, res, elapsed, to, err = run_algo_with_timeout(algo_name, blocs, start_pos, timeout_sec)
        if not ok:
            raise TimeoutError(f"Repeat failed: {'timeout' if to else err}")
        last_result = res
        times.append(elapsed)

    best = min(times) if times else 0.0
    mean = stats.mean(times) if times else 0.0
    stdev = stats.pstdev(times) if len(times) > 1 else 0.0
    return best, mean, stdev, last_result



def generate_random_bloc_list(num_blocs, area_size=100, grid=5):
    """
    Génère num_blocs blocs aléatoires.
    - Pas deux blocs à la même position
    - x et y multiples de 'grid'
    """
    blocs = []
    colors = ["red", "blue", "green", "yellow"]

    used_positions = set()

    max_coord = area_size // grid   # ex: 100//5 = 20

    while len(blocs) < num_blocs:
        color = random.choice(colors)

        x = random.randint(0, max_coord) * grid
        y = random.randint(0, max_coord) * grid

        if (x, y) in used_positions:
            continue   # déjà occupé

        used_positions.add((x, y))
        blocs.append((color, x, y))

    return np.array(blocs, dtype=object)

def generate_bloc_instances(n_instances, m_blocs, seed=None):
    if seed is not None:
        random.seed(seed)

    return [generate_random_bloc_list(m_blocs) for _ in range(n_instances)]

def setup_logger():
    import logging
    import os
    from datetime import datetime

    os.makedirs("logs_algo_path", exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_filename = f"logs_algo_path/benchmark_{timestamp}.log"

    root = logging.getLogger()
    root.setLevel(logging.INFO)

    # Supprime anciens handlers
    for h in root.handlers[:]:
        root.removeHandler(h)

    formatter = logging.Formatter("%(asctime)s | %(levelname)s | %(message)s")

    fh = logging.FileHandler(log_filename)
    fh.setFormatter(formatter)

    #sh = logging.StreamHandler()
    #sh.setFormatter(formatter)

    root.addHandler(fh)
    #root.addHandler(sh)

    logging.info("==== Benchmark started ====")
    logging.info(f"Log file: {log_filename}")

def main():
    setup_logger()
    run_stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    plots_dir = os.path.join("Trajectoire", "plots_algo_path", f"run_{run_stamp}")
    os.makedirs(plots_dir, exist_ok=True)

    start_pos = config_traj.home_position

    # --- Explosion watcher ---
    ABS_TIMEOUT_SEC = 1.45        # si un run dépasse 250ms -> timeout
    EXPLODE_MEAN_SEC = 1.40       # si mean dépasse 200ms -> on stop pour les m suivants
    GROWTH_FACTOR = 62.0           # si mean(m) > 8 * mean(m-1) -> explosion
    disabled_algo = set()         # algos désactivés pour les prochains m
    prev_mean_by_algo = {}        # pour détecter croissance

    # ---- Paramètres benchmark (n = nb d'instances, m = nb de blocs/instance) ----
    vitesse_mvt = 50.0  # cm/s
    n_instances = 2     # nombre d'instances par valeur de m
    m_values = range(2,25)  # nombre de blocs (m)
    seed = 20            # reproductibilité
    repeats_per_instance = 3  # répéter chaque algo sur la même instance pour lisser le bruit
    warmup = 1

    # Stockage global pour plots: sweep_results[algo]["m"], ["t_calc"], ["t_move"], ["t_total"]
    sweep_results = {}

    csv_rows = []

    for m_blocs in tqdm(m_values, desc="Sweeping m values"):
        instances = generate_bloc_instances(n_instances, m_blocs, seed=seed+m_blocs)

        # ---- Définition des algos à comparer ----
        # NB: on va créer les lambdas *à l'intérieur* de la boucle sur "blocs"
        #     pour utiliser l'instance courante.
        algo_specs = [
            "Exact TSP DP",
            "Branch and Bound Basic",
            "Branch and Bound Heuristic",
            # "Heuristic TSP",
            "Optimal brute-force",
        ]

        # Filtrer les algos désactivés
        # Enlève les algos "explosés"
        algo_specs = [name for name in algo_specs if name not in disabled_algo]


        # ---- Accumulateurs ----
        # metrics[name] = {"dists": [...], "means": [...], "bests":[...]}
        metrics = {
            name: {
                "dists": [],
                "ratios": [],
                "mean_t_calculations": [],
                "best_t_calculations": []
            }
            for name in algo_specs
        }


        logging.info(f"\nBenchmark multi-instances")
        logging.info(f"n_instances={n_instances} | m_blocs={m_blocs} | repeats/instance={repeats_per_instance} | seed={seed+m_blocs}")
        logging.info("-" * 90)

        for blocs in instances:
            # ----- baseline exact (une fois par instance) -----
            opt_dist = float("nan")
            try:
                # 1 run, pas de repeats: tu veux juste la valeur de référence
                _, _, _, exact_result = time_one(
                    algo_name="Exact TSP DP",   # ou "Optimal brute-force" si tu veux pour m petit
                    blocs=blocs,
                    start_pos=start_pos,
                    repeats=1,
                    warmup=0,
                    timeout_sec=ABS_TIMEOUT_SEC
                )

                if exact_result is not None:
                    exact_order, exact_dist = exact_result
                    if exact_dist is None:
                        exact_dist = total_cost_for_order(exact_order, start_pos)
                    opt_dist = float(exact_dist)

            except Exception as e:
                logging.warning(f"[BASELINE EXACT FAILED] m={m_blocs}: {e}")
                opt_dist = float("nan")

            for name in algo_specs:
                if name in disabled_algo:
                    continue

                try:
                    best_t, mean_t_calculations, stdev_t, result = time_one(
                        algo_name=name,
                        blocs=blocs,
                        start_pos=start_pos,
                        repeats=repeats_per_instance,
                        warmup=warmup,
                        timeout_sec=ABS_TIMEOUT_SEC
                    )

                    if result is None:
                        raise ValueError("Algo returned None")

                    order, dist = result
                    if dist is None:
                        dist = total_cost_for_order(order, start_pos)

                    metrics[name]["dists"].append(dist)
                    metrics[name]["mean_t_calculations"].append(mean_t_calculations)
                    metrics[name]["best_t_calculations"].append(best_t)

                    # ratio vs optimum exact (si dispo)
                    if np.isfinite(opt_dist) and opt_dist > 0:
                        metrics[name]["ratios"].append(dist / opt_dist)
                    else:
                        metrics[name]["ratios"].append(float("nan"))


                except Exception as e:
                    logging.warning(f"Algo '{name}' failed on one instance: {e}")
                    metrics[name]["dists"].append(float("nan"))
                    metrics[name]["mean_t_calculations"].append(float("nan"))
                    metrics[name]["best_t_calculations"].append(float("nan"))
                    metrics[name]["ratios"].append(float("nan"))


                    if isinstance(e, TimeoutError):
                        logging.warning(f"[TIMEOUT] Disable '{name}' for next m (timeout {ABS_TIMEOUT_SEC}s)")
                        disabled_algo.add(name)

                        


        # ---------------- Explosion watcher (après agrégation) ----------------
        for name in list(metrics.keys()):
            t_calc_clean = clean(metrics[name]["mean_t_calculations"])

            # si aucun data valide
            if len(t_calc_clean) == 0:
                continue

            mean_m = stats.mean(t_calc_clean)

            # Rule A: mean trop grand sur ce m
            if mean_m > EXPLODE_MEAN_SEC:
                logging.warning(f"[EXPLODE] Disable '{name}' for next m: mean_m={mean_m:.4f}s > {EXPLODE_MEAN_SEC:.4f}s")
                disabled_algo.add(name)
                continue

            # Rule B: croissance vs m précédent (sur la moyenne agrégée)
            prev = prev_mean_by_algo.get(name, None)
            if prev is not None and mean_m > GROWTH_FACTOR * prev:
                logging.warning(f"[EXPLODE] Disable '{name}' for next m: growth mean_m={mean_m:.4f}s > {GROWTH_FACTOR}*{prev:.4f}s")
                disabled_algo.add(name)
                continue

            prev_mean_by_algo[name] = mean_m
# ----------------------------------------------------------------------


        logging.info("\n")  # newline après le \r

        # ---- Résumé agrégé ----
        logging.info("Resultats agreges (moyenne sur instances)")
        logging.info("-" * 160)
        logging.info(
            f"{'Algo':28s} | "
            f"{'dist_mean':>10s} | {'dist_std':>10s} | "
            f"{'t_calc(ms)':>12s} | {'t_calc_std':>12s} | "
            f"{'t_move(ms)':>12s} | {'t_move_std':>12s} | "
            f"{'t_total(ms)':>12s} | {'t_total_std':>12s} | "
            f"{'ratio':>8s} | {'r_std':>8s}"
        )

        logging.info("-" * 160)

        summary = []

        for name in metrics:
            dists = clean(metrics[name]["dists"])
            t_calc = clean(metrics[name]["mean_t_calculations"])

            # si un algo a échoué partout sur ce m
            if len(dists) == 0 or len(t_calc) == 0:
                logging.info(f"{name:28s} | {'NO DATA':>10s} | {'':>10s} | {'':>12s} | {'':>12s} | {'':>12s} | {'':>12s} | {'':>12s} | {'':>12s}")
                continue

            d_mean = stats.mean(dists)
            d_std  = stats.pstdev(dists) if len(dists) > 1 else 0.0

            # mouvement
            t_move = [d / vitesse_mvt for d in dists]
            t_mean_mvt = stats.mean(t_move)
            t_std_mvt  = stats.pstdev(t_move) if len(t_move) > 1 else 0.0

            # calcul
            t_mean_calc = stats.mean(t_calc)
            t_std_calc  = stats.pstdev(t_calc) if len(t_calc) > 1 else 0.0

            # total (aligner longueurs: prendre le min)
            L = min(len(t_move), len(t_calc))
            t_total = [t_move[i] + t_calc[i] for i in range(L)]
            t_mean_total = stats.mean(t_total)
            t_std_total  = stats.pstdev(t_total) if len(t_total) > 1 else 0.0
            
            # ratio vs optimal exact
            ratios = clean(metrics[name]["ratios"])
            r_mean = stats.mean(ratios) if len(ratios) else float("nan")
            r_std  = stats.pstdev(ratios) if len(ratios) > 1 else 0.0


            summary.append(
                (name, d_mean, d_std,
                t_mean_calc, t_std_calc,
                t_mean_mvt, t_std_mvt,
                t_mean_total, t_std_total)
            )

            csv_rows.append([
                m_blocs,
                name,
                t_mean_calc,
                t_mean_mvt,
                t_mean_total,
                d_mean,
                r_mean,
                r_std
            ])

            logging.info(
                f"{name:28s} | "
                f"{d_mean:10.2f} | {d_std:10.2f} | "
                f"{t_mean_calc*1000:12.3f} | {t_std_calc*1000:12.3f} | "
                f"{t_mean_mvt*1000:12.3f} | {t_std_mvt*1000:12.3f} | "
                f"{t_mean_total*1000:12.3f} | {t_std_total*1000:12.3f} | "
                f"{r_mean:8.4f} | {r_std:8.4f}"
            )



            # --- Stockage pour plots ---
            if name not in sweep_results:
                sweep_results[name] = {
                    "m": [],
                    "t_calc": [],
                    "t_move": [],
                    "t_total": [],
                    "dist": [],
                    "ratio": []
                }


            sweep_results[name]["m"].append(m_blocs)
            sweep_results[name]["t_calc"].append(t_mean_calc)
            sweep_results[name]["t_move"].append(t_mean_mvt)
            sweep_results[name]["t_total"].append(t_mean_total)
            sweep_results[name]["dist"].append(d_mean)
            sweep_results[name]["ratio"].append(r_mean)


        # ---- Classements ----
        logging.info("\nClassement (distance moyenne croissante):")
        for i, (name, d_mean, d_std, t_mean_calc, t_std_calc, t_mean_mvt, t_std_mvt, t_mean_total, t_std_total) in enumerate(sorted(summary, key=lambda r: r[1]), 1):
            logging.info(f"{i:2d}. {name} -> dist_mean={d_mean:.2f} (±{d_std:.2f}), t_total_mean={t_mean_total*1000:.3f} ms")

        logging.info("\nClassement (temps total moyen croissant):")
        for i, (name, d_mean, d_std, t_mean_calc, t_std_calc, t_mean_mvt, t_std_mvt, t_mean_total, t_std_total) in enumerate(sorted(summary, key=lambda r: r[7]), 1):
            logging.info(f"{i:2d}. {name} -> t_total_mean={t_mean_total*1000:.3f} ms (±{t_std_total*1000:.3f}), dist_mean={d_mean:.2f}")


    def plot_metric(metric_key, title, ylabel, filename):
        plt.figure()
        has_any = False

        for algo_name, series in sweep_results.items():
            ms = series["m"]
            ys = series[metric_key]
            if len(ms) == 0:
                continue
            has_any = True
            plt.plot(ms, ys, marker='o', label=algo_name, markersize=5, linewidth=2)

        plt.xlabel("Nombre de blocs (m)")
        plt.ylabel(ylabel)
        plt.title(title)
        plt.grid(True)

        if has_any:
            plt.legend()

        outpath = os.path.join(plots_dir, filename)
        plt.savefig(outpath, dpi=200, bbox_inches="tight")
        plt.close()
        logging.info(f"Plot saved: {outpath}")



    plot_metric(
        metric_key="t_calc",
        title="Temps de calcul (t_calc) vs nombre de blocs",
        ylabel="Temps de calcul (s)",
        filename="t_calc_vs_m.png"
    )
    plot_metric(
        metric_key="t_move",
        title="Temps de mouvement (t_move) vs nombre de blocs",
        ylabel="Temps de mouvement (s)",
        filename="t_move_vs_m.png"
        )

    plot_metric(
        metric_key="t_total",
        title="Temps total (t_total) vs nombre de blocs",
        ylabel="Temps total (s)",
        filename="t_total_vs_m.png"
    )

    plot_metric(
        metric_key="dist",
        title="Distance moyenne vs nombre de blocs",
        ylabel="Distance moyenne",
        filename="dist_vs_m.png"
    )

    plot_metric(
        metric_key="ratio",
        title="Ratio dist/optimum (plus proche de 1.0 = meilleur)",
        ylabel="dist / dist_opt",
        filename="ratio_vs_m.png"
    )

    # ----- SAVE CSV RESULTS -----
    csv_path = os.path.join(plots_dir, "results.csv")

    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow([
            "m_blocs",
            "algorithm",
            "t_calc_mean_sec",
            "t_move_mean_sec",
            "t_total_mean_sec",
            "dist_mean",
            "ratio_mean",
            "ratio_std"
        ])
        writer.writerows(csv_rows)

    logging.info(f"CSV saved: {csv_path}")


if __name__ == "__main__":
    main()