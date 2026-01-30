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

import config_traj

# ---- Import tes algos (adapte selon tes noms de fichiers) ----
from shortest_path_algorithms import plan_nearest_neighbor, plan_optimal_bruteforce, plan_cheapest_insertion, plan_greedy_then_swap_improve, plan_random_restart_greedy
from shortest_path_algorithms import plan_exact_tsp, plan_heuristic_tsp

# ---- Import helpers pour scorer (coût total) ----
from other_fct_traj import output_pos_for_color, cost_do_bloc_from

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

def time_one(run_fn, repeats=10, warmup=1):
    """
    Mesure le temps d'exécution d'une fonction qui retourne (order, total_cost)
    Return: (best_time, mean_time, stdev_time, last_result)
    """
    # warm-up
    last_result = None
    for _ in range(warmup):
        last_result = run_fn()

    times = []
    for _ in range(repeats):
        t0 = time.perf_counter()
        last_result = run_fn()
        t1 = time.perf_counter()
        times.append(t1 - t0)

    best = min(times)
    mean = stats.mean(times)
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
    max_opt = 7 # max blocs pour algo optimal

    # ---- Paramètres benchmark (n = nb d'instances, m = nb de blocs/instance) ----
    vitesse_mvt = 50.0  # cm/s
    n_instances = 2     # nombre d'instances par valeur de m
    m_values = range(2,17)  # nombre de blocs (m)
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
            #("Nearest neighbor O(n^2)", lambda blocs: (lambda: plan_nearest_neighbor(blocs, start_pos))),
            #("Cheapest insertion", lambda blocs: (lambda: plan_cheapest_insertion(blocs, start_pos))),
            #("Nearest neighbor + swap", lambda blocs: (lambda: plan_greedy_then_swap_improve(blocs, start_pos, max_passes=20))),
            ("Exact TSP DP", lambda blocs: (lambda: plan_exact_tsp(blocs, start_pos))),
            #("Heuristic TSP", lambda blocs: (lambda: plan_heuristic_tsp(blocs, start_pos))),
        ]

        # brute-force seulement si m <= max_opt
        if m_blocs <= max_opt:
            algo_specs.append(("Optimal brute-force", lambda blocs: (lambda: plan_optimal_bruteforce(blocs, start_pos))))

        # ---- Accumulateurs ----
        # metrics[name] = {"dists": [...], "means": [...], "bests":[...]}
        metrics = {name: {"dists": [], "mean_t_calculations": [], "best_t_calculations": []} for name, _ in algo_specs}

        logging.info(f"\nBenchmark multi-instances")
        logging.info(f"n_instances={n_instances} | m_blocs={m_blocs} | repeats/instance={repeats_per_instance} | seed={seed+m_blocs}")
        logging.info("-" * 90)

        for blocs in instances:
            for name, make_runner in algo_specs:

                run_fn = make_runner(blocs)

                try:
                    best_t, mean_t_calculations, stdev_t, result = time_one(
                    run_fn,
                    repeats=repeats_per_instance,
                    warmup=warmup
                    )

                    # result doit être (order, dist) ou (order, None)
                    if result is None:
                        raise ValueError("Algo returned None")

                    order, dist = result

                    # Si dist n'est pas fourni, on le recalcule
                    if dist is None:
                        dist = total_cost_for_order(order, start_pos)

                    metrics[name]["dists"].append(dist)
                    metrics[name]["mean_t_calculations"].append(mean_t_calculations)
                    metrics[name]["best_t_calculations"].append(best_t)

                except Exception as e:
                    # On enregistre l'erreur et on continue, sinon metrics reste vide et ça crash au mean()
                    logging.warning(f"Algo '{name}' failed on one instance: {e}")
                    # Option: append NaN pour garder le même nombre d'échantillons
                    metrics[name]["dists"].append(float("nan"))
                    metrics[name]["mean_t_calculations"].append(float("nan"))
                    metrics[name]["best_t_calculations"].append(float("nan"))


        logging.info("\n")  # newline après le \r

        # ---- Résumé agrégé ----
        logging.info("Resultats agreges (moyenne sur instances)")
        logging.info("-" * 160)
        logging.info(
            f"{'Algo':28s} | "
            f"{'dist_mean':>10s} | {'dist_std':>10s} | "
            f"{'t_calc(ms)':>12s} | {'t_calc_std':>12s} | "
            f"{'t_move(ms)':>12s} | {'t_move_std':>12s} | "
            f"{'t_total(ms)':>12s} | {'t_total_std':>12s}"
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
                d_mean
            ])

            logging.info(
                f"{name:28s} | "
                f"{d_mean:10.2f} | {d_std:10.2f} | "
                f"{t_mean_calc*1000:12.3f} | {t_std_calc*1000:12.3f} | "
                f"{t_mean_mvt*1000:12.3f} | {t_std_mvt*1000:12.3f} | "
                f"{t_mean_total*1000:12.3f} | {t_std_total*1000:12.3f}"
            )

            # --- Stockage pour plots ---
            if name not in sweep_results:
                sweep_results[name] = {
                    "m": [],
                    "t_calc": [],
                    "t_move": [],
                    "t_total": [],
                    "dist": []
                }


            sweep_results[name]["m"].append(m_blocs)
            sweep_results[name]["t_calc"].append(t_mean_calc)
            sweep_results[name]["t_move"].append(t_mean_mvt)
            sweep_results[name]["t_total"].append(t_mean_total)
            sweep_results[name]["dist"].append(d_mean)


        # ---- Classements ----
        logging.info("\nClassement (distance moyenne croissante):")
        for i, (name, d_mean, d_std, t_mean_calc, t_std_calc, t_mean_mvt, t_std_mvt, t_mean_total, t_std_total) in enumerate(sorted(summary, key=lambda r: r[1]), 1):
            logging.info(f"{i:2d}. {name} -> dist_mean={d_mean:.2f} (±{d_std:.2f}), t_total_mean={t_mean_total*1000:.3f} ms")

        logging.info("\nClassement (temps total moyen croissant):")
        for i, (name, d_mean, d_std, t_mean_calc, t_std_calc, t_mean_mvt, t_std_mvt, t_mean_total, t_std_total) in enumerate(sorted(summary, key=lambda r: r[7]), 1):
            logging.info(f"{i:2d}. {name} -> t_total_mean={t_mean_total*1000:.3f} ms (±{t_std_total*1000:.3f}), dist_mean={d_mean:.2f}")


    def plot_metric(metric_key, title, ylabel, filename):
        plt.figure()
        for algo_name, series in sweep_results.items():
            ms = series["m"]
            ys = series[metric_key]

            # Courbe seulement (pas de trend)
            plt.plot(ms, ys, marker='o', label=algo_name, markersize=5, linewidth=2)

        plt.xlabel("Nombre de blocs (m)")
        plt.ylabel(ylabel)
        plt.title(title)
        plt.grid(True)
        plt.legend()
        outpath = os.path.join(plots_dir, filename)
        plt.savefig(outpath, dpi=200, bbox_inches="tight")
        plt.close()   # mieux que plt.show() pour un benchmark

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
            "dist_mean"
        ])
        writer.writerows(csv_rows)

    logging.info(f"CSV saved: {csv_path}")


if __name__ == "__main__":
    main()