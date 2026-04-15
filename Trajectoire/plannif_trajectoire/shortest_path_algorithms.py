"""
shortest_path_algorithms.py

Algorithmes de planification (ordre des blocs) pour minimiser le coût total
selon TON modèle:

- On part de start_pos (home_position).
- Pour chaque bloc b=(color,x,y):
    coût étape = cost_do_bloc_from(position_courante, b)
    puis la "position_courante" devient output_pos_for_color(b.color)

Donc:
- le coût entre deux blocs dépend de la couleur du bloc précédent (coût dirigé / asymétrique),
- on optimise un CHEMIN OUVERT (pas de retour à la maison à la fin).

Toutes les fonctions retournent:
    (order: list[Bloc], total_cost: float)

où Bloc = tuple[str, int, int]  # (color, x, y)
"""

from __future__ import annotations

import itertools
import random
from typing import List, Tuple, Optional

import numpy as np

import config_traj
from other_fct_traj import cost_do_bloc_from, output_pos_for_color

from python_tsp.exact import solve_tsp_dynamic_programming
from python_tsp.heuristics import solve_tsp_local_search


# -------------------- Types & config --------------------

Bloc = Tuple[str, int, int]  # (color, x, y)

# (Optionnel) tu peux garder ces variables là si tu veux les utiliser ailleurs.
home_position = config_traj.home_position
MAX_BLOCS_OPTIMAL = getattr(config_traj, "MAX_BLOCS_OPTIMAL", None)


# -------------------- Helpers de coût (TON modèle) --------------------

def _as_bloc_list(blocs) -> List[Bloc]:
    """Accepte list ou np.array(dtype=object). Retourne list[tuple]."""
    if hasattr(blocs, "tolist"):
        blocs = blocs.tolist()
    return [tuple(b) for b in blocs]


def total_cost_for_order(order: List[Bloc], start_pos) -> float:
    """Coût exact selon TON modèle (chemin ouvert)."""
    pos = start_pos
    total = 0.0
    for b in order:
        total += float(cost_do_bloc_from(pos, b))
        pos = output_pos_for_color(b[0])
    return total


def compute_distance_matrix(blocs, start_pos) -> np.ndarray:
    """
    Matrice de distance (n+1)x(n+1) pour les solveurs TSP de python_tsp.
    Convention:
      - index 0 = "start"
      - index i+1 = bloc i
    Distances:
      - 0 -> i : start_pos -> bloc_i
      - i -> j : output(bloc_i) -> bloc_j
    NOTE: Les solveurs TSP supposent souvent un tour fermé; on RECALCULE ensuite
          le coût réel avec total_cost_for_order() (sans retour).
    """
    B = _as_bloc_list(blocs)
    n = len(B)
    dist = np.zeros((n + 1, n + 1), dtype=float)

    # 0 -> i
    for i in range(n):
        dist[0, i + 1] = float(cost_do_bloc_from(start_pos, B[i]))

    # i -> j (via sortie du bloc i)
    for i in range(n):
        out_i = output_pos_for_color(B[i][0])
        for j in range(n):
            if i == j:
                dist[i + 1, j + 1] = 0.0
            else:
                dist[i + 1, j + 1] = float(cost_do_bloc_from(out_i, B[j]))

    return dist


# ======================================================================
# 1) EXACT / BASELINE
# ======================================================================

def plan_optimal_bruteforce(blocs, start_pos):
    """
    Exact par brute-force: O(n!)
    À garder seulement pour n très petit.
    """
    B = _as_bloc_list(blocs)
    n = len(B)
    if n == 0:
        return [], 0.0
    if n == 1:
        return [B[0]], float(cost_do_bloc_from(start_pos, B[0]))

    best_order = None
    best_cost = float("inf")

    for perm in itertools.permutations(B):
        c = total_cost_for_order(list(perm), start_pos)
        if c < best_cost:
            best_cost = c
            best_order = list(perm)

    return best_order, best_cost


def plan_exact_tsp(blocs, start_pos):
    """
    Exact via DP (Held-Karp): ~O(n^2 * 2^n) (via python_tsp)
    IMPORTANT: on recalcule le coût réel sans retour.
    """
    B = _as_bloc_list(blocs)
    n = len(B)
    if n == 0:
        return [], 0.0
    if n == 1:
        return [B[0]], float(cost_do_bloc_from(start_pos, B[0]))

    dist_matrix = compute_distance_matrix(B, start_pos)

    perm, _ = solve_tsp_dynamic_programming(dist_matrix)

    # perm contient 0 (start) + indices des noeuds (1..n)
    order = [B[i - 1] for i in perm if i != 0]
    true_cost = total_cost_for_order(order, start_pos)
    return order, true_cost


# ======================================================================
# 2) HEURISTIQUES "classiques" (rapides)
# ======================================================================

def plan_heuristic_tsp(blocs, start_pos):
    """
    Heuristique via python_tsp local search (rapide).
    Complexité typique ~O(n^2) à O(n^3) selon la recherche locale.
    """
    B = _as_bloc_list(blocs)
    n = len(B)
    if n == 0:
        return [], 0.0
    if n == 1:
        return [B[0]], float(cost_do_bloc_from(start_pos, B[0]))

    dist_matrix = compute_distance_matrix(B, start_pos)

    perm, _ = solve_tsp_local_search(dist_matrix)

    order = [B[i - 1] for i in perm if i != 0]
    true_cost = total_cost_for_order(order, start_pos)
    return order, true_cost


def plan_nearest_neighbor(blocs, start_pos):
    """
    Nearest Neighbor (greedy): O(n^2)
    """
    remaining = _as_bloc_list(blocs)
    order: List[Bloc] = []
    pos = start_pos
    total = 0.0

    while remaining:
        best_i = None
        best_cost = float("inf")
        for i, b in enumerate(remaining):
            c = float(cost_do_bloc_from(pos, b))
            if c < best_cost:
                best_cost = c
                best_i = i

        b = remaining.pop(best_i)
        order.append(b)
        total += best_cost
        pos = output_pos_for_color(b[0])

    return order, total


def plan_cheapest_insertion(blocs, start_pos):
    """
    Cheapest insertion "naïf": O(n^3) (recalc complet à chaque candidat)
    Qualité souvent meilleure que NN.
    """
    remaining = _as_bloc_list(blocs)
    if not remaining:
        return [], 0.0

    # commencer par le bloc le moins cher depuis start
    first = min(remaining, key=lambda b: float(cost_do_bloc_from(start_pos, b)))
    order = [first]
    remaining.remove(first)

    while remaining:
        best_total = float("inf")
        best_b = None
        best_pos = None

        for b in remaining:
            for k in range(len(order) + 1):
                candidate = order[:k] + [b] + order[k:]
                c = total_cost_for_order(candidate, start_pos)
                if c < best_total:
                    best_total = c
                    best_b = b
                    best_pos = k

        order.insert(best_pos, best_b)
        remaining.remove(best_b)

    return order, total_cost_for_order(order, start_pos)


def plan_greedy_then_swap_improve(blocs, start_pos, max_passes=20):
    """
    NN + amélioration par swap (local search):
      - greedy: O(n^2)
      - swap improve: O(max_passes * n^3) (car recalc complet à chaque swap)
    """
    order, best_cost = plan_nearest_neighbor(blocs, start_pos)
    n = len(order)
    if n <= 1:
        return order, best_cost

    def swapped(o, i, j):
        oo = o[:]
        oo[i], oo[j] = oo[j], oo[i]
        return oo

    passes = 0
    improved = True

    while improved and passes < max_passes:
        improved = False
        passes += 1
        for i in range(n - 1):
            for j in range(i + 1, n):
                cand = swapped(order, i, j)
                c = total_cost_for_order(cand, start_pos)
                if c < best_cost:
                    order = cand
                    best_cost = c
                    improved = True

    return order, best_cost


def plan_random_restart_greedy(blocs, start_pos, restarts=200, k=3, seed=0):
    """
    Random-Restart greedy:
    À chaque étape on choisit aléatoirement parmi les k meilleurs candidats.
    Complexité ~O(restarts * n^2 log n) (tri à chaque étape).
    """
    rng = random.Random(seed)
    B = _as_bloc_list(blocs)
    if not B:
        return [], 0.0

    best_order = None
    best_cost = float("inf")

    for _ in range(restarts):
        remaining = B[:]
        order = []
        pos = start_pos
        total = 0.0

        while remaining:
            scored = [(float(cost_do_bloc_from(pos, b)), idx, b) for idx, b in enumerate(remaining)]
            scored.sort(key=lambda t: t[0])
            kk = min(k, len(scored))
            _, chosen_idx, chosen_bloc = rng.choice(scored[:kk])

            remaining.pop(chosen_idx)
            order.append(chosen_bloc)

            total += float(cost_do_bloc_from(pos, chosen_bloc))
            pos = output_pos_for_color(chosen_bloc[0])

        if total < best_cost:
            best_cost = total
            best_order = order

    return best_order, best_cost


# ======================================================================
# 3) BRANCH & BOUND (exact, chemin ouvert)
# ======================================================================

def _build_cost_tables(blocs, start_pos):
    """
    Pré-calculs pour accélérer BnB:
      cost_start[i] = start_pos -> bloc_i
      cost_ij[i][j] = out(bloc_i) -> bloc_j
    """
    B = _as_bloc_list(blocs)
    n = len(B)

    cost_start = [0.0] * n
    for i in range(n):
        cost_start[i] = float(cost_do_bloc_from(start_pos, B[i]))

    cost_ij = [[0.0] * n for _ in range(n)]
    for i in range(n):
        out_i = output_pos_for_color(B[i][0])
        for j in range(n):
            if i == j:
                cost_ij[i][j] = 0.0
            else:
                cost_ij[i][j] = float(cost_do_bloc_from(out_i, B[j]))

    return B, cost_start, cost_ij


def plan_bnb_basic(blocs, start_pos):
    """
    Branch & Bound EXACT (chemin ouvert).
    Pire cas O(n!), mais prune souvent.

    Borne inf (admissible) utilisée:
      - on doit aller au moins vers un prochain noeud non visité,
      - et chaque noeud non visité devra "sortir" au moins une fois,
      - sauf le dernier (chemin ouvert) => on enlève le plus gros min_out.
    """
    B, cost_start, cost_ij = _build_cost_tables(blocs, start_pos)
    n = len(B)
    if n == 0:
        return [], 0.0
    if n == 1:
        return [B[0]], cost_start[0]

    # Upper bound initial (rapide)
    best_order, best_cost = plan_nearest_neighbor(B, start_pos)

    # min_out[i] = plus petit arc sortant i->j (j!=i)
    min_out = [0.0] * n
    for i in range(n):
        m = float("inf")
        for j in range(n):
            if i != j:
                m = min(m, cost_ij[i][j])
        min_out[i] = m

    def bound(cur_idx, unv, cur_cost):
        if not unv:
            return cur_cost

        # coût minimal pour "connecter" la position courante à un non-visité
        if cur_idx == -1:
            min_from_cur = min(cost_start[j] for j in unv)
        else:
            min_from_cur = min(cost_ij[cur_idx][j] for j in unv)

        outs = [min_out[j] for j in unv]
        # chemin ouvert: le dernier n'a pas besoin de "sortie"
        return cur_cost + min_from_cur + (sum(outs) - max(outs))

    best_idx_order = None

    def dfs(cur_idx, unv, path, cur_cost):
        nonlocal best_cost, best_idx_order

        if bound(cur_idx, unv, cur_cost) >= best_cost:
            return

        if not unv:
            if cur_cost < best_cost:
                best_cost = cur_cost
                best_idx_order = path[:]
            return

        # essayer les branches prometteuses d'abord
        if cur_idx == -1:
            cand = sorted(unv, key=lambda j: cost_start[j])
        else:
            cand = sorted(unv, key=lambda j: cost_ij[cur_idx][j])

        for nxt in cand:
            step = cost_start[nxt] if cur_idx == -1 else cost_ij[cur_idx][nxt]
            new_cost = cur_cost + step
            if new_cost >= best_cost:
                continue
            new_unv = [x for x in unv if x != nxt]
            dfs(nxt, new_unv, path + [nxt], new_cost)

    dfs(-1, list(range(n)), [], 0.0)

    if best_idx_order is None:
        # fallback
        return best_order, best_cost

    order = [B[i] for i in best_idx_order]
    return order, best_cost


def plan_bnb_heuristic(blocs, start_pos, max_passes_swap=20):
    """
    Branch & Bound EXACT, avec UB initial plus serré:
    greedy + swap improve.
    """
    B, cost_start, cost_ij = _build_cost_tables(blocs, start_pos)
    n = len(B)
    if n == 0:
        return [], 0.0
    if n == 1:
        return [B[0]], cost_start[0]

    best_order, best_cost = plan_greedy_then_swap_improve(B, start_pos, max_passes=max_passes_swap)

    min_out = [0.0] * n
    for i in range(n):
        m = float("inf")
        for j in range(n):
            if i != j:
                m = min(m, cost_ij[i][j])
        min_out[i] = m

    def bound(cur_idx, unv, cur_cost):
        if not unv:
            return cur_cost

        if cur_idx == -1:
            min_from_cur = min(cost_start[j] for j in unv)
        else:
            min_from_cur = min(cost_ij[cur_idx][j] for j in unv)

        outs = [min_out[j] for j in unv]
        return cur_cost + min_from_cur + (sum(outs) - max(outs))

    best_idx_order = None

    def dfs(cur_idx, unv, path, cur_cost):
        nonlocal best_cost, best_idx_order

        if bound(cur_idx, unv, cur_cost) >= best_cost:
            return

        if not unv:
            if cur_cost < best_cost:
                best_cost = cur_cost
                best_idx_order = path[:]
            return

        if cur_idx == -1:
            cand = sorted(unv, key=lambda j: cost_start[j])
        else:
            cand = sorted(unv, key=lambda j: cost_ij[cur_idx][j])

        for nxt in cand:
            step = cost_start[nxt] if cur_idx == -1 else cost_ij[cur_idx][nxt]
            new_cost = cur_cost + step
            if new_cost >= best_cost:
                continue
            new_unv = [x for x in unv if x != nxt]
            dfs(nxt, new_unv, path + [nxt], new_cost)

    dfs(-1, list(range(n)), [], 0.0)

    if best_idx_order is None:
        return best_order, best_cost

    order = [B[i] for i in best_idx_order]
    return order, best_cost


# ======================================================================
# 4) HEURISTIQUES "greedy mais pas trop" (plus coûteuses, meilleure qualité)
# ======================================================================

def _dist_from_pos_to_bloc(pos, b: Bloc) -> float:
    return float(cost_do_bloc_from(pos, b))

def _dist_from_bloc_to_bloc(b_from: Bloc, b_to: Bloc) -> float:
    out_pos = output_pos_for_color(b_from[0])
    return float(cost_do_bloc_from(out_pos, b_to))

def _path_cost_from_indices(tour_idx: List[int], B: List[Bloc], start_pos) -> float:
    """Coût d'un chemin (indices) sans retour final."""
    if not tour_idx:
        return 0.0
    total = _dist_from_pos_to_bloc(start_pos, B[tour_idx[0]])
    for i in range(len(tour_idx) - 1):
        total += _dist_from_bloc_to_bloc(B[tour_idx[i]], B[tour_idx[i + 1]])
    return total

def _insertion_delta(tour: List[int], insert_pos: int, node: int, B: List[Bloc], start_pos) -> float:
    """
    Delta coût si on insère node à insert_pos dans tour (chemin ouvert).
    """
    if len(tour) == 0:
        return _dist_from_pos_to_bloc(start_pos, B[node])

    if insert_pos == 0:
        a = tour[0]
        return (
            _dist_from_pos_to_bloc(start_pos, B[node]) +
            _dist_from_bloc_to_bloc(B[node], B[a]) -
            _dist_from_pos_to_bloc(start_pos, B[a])
        )

    if insert_pos == len(tour):
        prev = tour[-1]
        return _dist_from_bloc_to_bloc(B[prev], B[node])

    prev = tour[insert_pos - 1]
    nxt  = tour[insert_pos]
    return (
        _dist_from_bloc_to_bloc(B[prev], B[node]) +
        _dist_from_bloc_to_bloc(B[node], B[nxt]) -
        _dist_from_bloc_to_bloc(B[prev], B[nxt])
    )

def _best_insertion_position(tour: List[int], node: int, B: List[Bloc], start_pos):
    best_pos = 0
    best_delta = float("inf")
    for pos in range(len(tour) + 1):
        d = _insertion_delta(tour, pos, node, B, start_pos)
        if d < best_delta:
            best_delta = d
            best_pos = pos
    return best_pos, best_delta


# ---- 4.1 Regret-k insertion ----

def plan_regret_insertion(blocs, start_pos, k: int = 3, seed: Optional[int] = None):
    """
    Regret-k insertion:
      - on évalue plusieurs positions d'insertion pour chaque node
      - on choisit le node avec le plus grand regret (perte si on ne le place pas maintenant)
    k=2 est classique; k=3,5 = plus "lookahead".
    """
    rng = random.Random(seed)
    B = _as_bloc_list(blocs)
    n = len(B)
    if n == 0:
        return [], 0.0
    if n == 1:
        return [B[0]], total_cost_for_order([B[0]], start_pos)

    idxs = list(range(n))
    rng.shuffle(idxs)

    tour = [idxs[0], idxs[1]]
    remaining = set(idxs[2:])

    while remaining:
        best_node = None
        best_score = -float("inf")

        for node in remaining:
            deltas = []
            for pos in range(len(tour) + 1):
                deltas.append(_insertion_delta(tour, pos, node, B, start_pos))
            deltas.sort()

            best_delta = deltas[0]
            kk = min(k, len(deltas))

            regret = 0.0
            for i in range(1, kk):
                regret += (deltas[i] - best_delta)

            # score: regret prioritaire, tie-break sur best_delta
            score = regret - 1e-9 * best_delta
            if score > best_score:
                best_score = score
                best_node = node

        pos, _ = _best_insertion_position(tour, best_node, B, start_pos)
        tour.insert(pos, best_node)
        remaining.remove(best_node)

    order = [B[i] for i in tour]
    return order, total_cost_for_order(order, start_pos)


# ---- 4.2 Lookahead insertion ----

def plan_lookahead_insertion(blocs, start_pos, L: int = 8, k: int = 2, seed: Optional[int] = None):
    """
    Lookahead insertion:
      - à chaque étape, on prend les L meilleurs candidats (par best_delta)
      - pour chacun, on simule (k-1) insertions futures cheap
      - on choisit l'action qui minimise le coût simulé

    Peut vite monter en coût si tu augmentes L et k.
    """
    rng = random.Random(seed)
    B = _as_bloc_list(blocs)
    n = len(B)
    if n == 0:
        return [], 0.0
    if n == 1:
        return [B[0]], total_cost_for_order([B[0]], start_pos)

    idxs = list(range(n))
    rng.shuffle(idxs)

    tour = [idxs[0], idxs[1]]
    remaining = set(idxs[2:])

    def simulate(tour0: List[int], rem0: set[int], depth: int) -> float:
        tour_sim = list(tour0)
        rem_sim = set(rem0)
        for _ in range(depth):
            if not rem_sim:
                break
            best_node = None
            best_pos = None
            best_delta = float("inf")
            for node in rem_sim:
                pos, d = _best_insertion_position(tour_sim, node, B, start_pos)
                if d < best_delta:
                    best_delta = d
                    best_node = node
                    best_pos = pos
            tour_sim.insert(best_pos, best_node)
            rem_sim.remove(best_node)
        return _path_cost_from_indices(tour_sim, B, start_pos)

    while remaining:
        candidates = []
        for node in remaining:
            pos, d = _best_insertion_position(tour, node, B, start_pos)
            candidates.append((d, node, pos))
        candidates.sort(key=lambda x: x[0])
        candidates = candidates[:max(1, min(L, len(candidates)))]

        best_action = None
        best_future_cost = float("inf")

        for _, node, pos in candidates:
            tour2 = list(tour)
            rem2 = set(remaining)
            tour2.insert(pos, node)
            rem2.remove(node)

            future_cost = simulate(tour2, rem2, depth=max(0, k - 1))
            if future_cost < best_future_cost:
                best_future_cost = future_cost
                best_action = (node, pos)

        node, pos = best_action
        tour.insert(pos, node)
        remaining.remove(node)

    order = [B[i] for i in tour]
    return order, total_cost_for_order(order, start_pos)


# ---- 4.3 Local search: VND (relocate/swap/2-opt) ----

def _two_opt_best_improvement(tour: List[int], B: List[Bloc], start_pos) -> bool:
    """
    2-opt best-improvement (sur un chemin).
    Même si coûts asymétriques, ça reste un voisinage exploitable.
    """
    n = len(tour)
    if n < 4:
        return False

    best_gain = 0.0
    best_i = None
    best_j = None

    for i in range(0, n - 2):
        for j in range(i + 1, n - 1):
            a = tour[i - 1] if i > 0 else None
            b = tour[i]
            c = tour[j]
            d = tour[j + 1] if j + 1 < n else None

            old = 0.0
            if a is None:
                old += _dist_from_pos_to_bloc(start_pos, B[b])
            else:
                old += _dist_from_bloc_to_bloc(B[a], B[b])
            if d is not None:
                old += _dist_from_bloc_to_bloc(B[c], B[d])

            new = 0.0
            if a is None:
                new += _dist_from_pos_to_bloc(start_pos, B[c])
            else:
                new += _dist_from_bloc_to_bloc(B[a], B[c])
            if d is not None:
                new += _dist_from_bloc_to_bloc(B[b], B[d])

            gain = old - new
            if gain > best_gain:
                best_gain = gain
                best_i, best_j = i, j

    if best_gain > 1e-12:
        tour[best_i:best_j + 1] = reversed(tour[best_i:best_j + 1])
        return True

    return False


def _relocate_best_improvement(tour: List[int], B: List[Bloc], start_pos, block_len: int = 1) -> bool:
    """
    Déplace un segment (len=1 ou 2) ailleurs.
    Implémentation simple (recalc coût complet) => cher mais robuste.
    """
    n = len(tour)
    if n < block_len + 2:
        return False

    base_cost = _path_cost_from_indices(tour, B, start_pos)
    best_gain = 0.0
    best_move = None  # (cut, ins, seg)

    for cut in range(0, n - block_len + 1):
        seg = tour[cut:cut + block_len]
        rest = tour[:cut] + tour[cut + block_len:]

        for ins in range(0, len(rest) + 1):
            cand = rest[:ins] + seg + rest[ins:]
            c = _path_cost_from_indices(cand, B, start_pos)
            gain = base_cost - c
            if gain > best_gain:
                best_gain = gain
                best_move = (cut, ins, seg)

    if best_move is not None and best_gain > 1e-12:
        cut, ins, seg = best_move
        rest = tour[:cut] + tour[cut + block_len:]
        tour[:] = rest[:ins] + seg + rest[ins:]
        return True

    return False


def _swap_best_improvement(tour: List[int], B: List[Bloc], start_pos) -> bool:
    """
    Swap de deux noeuds. Simple (recalc complet) => O(n^3).
    """
    n = len(tour)
    if n < 2:
        return False

    base_cost = _path_cost_from_indices(tour, B, start_pos)
    best_gain = 0.0
    best_pair = None

    for i in range(n - 1):
        for j in range(i + 1, n):
            cand = list(tour)
            cand[i], cand[j] = cand[j], cand[i]
            c = _path_cost_from_indices(cand, B, start_pos)
            gain = base_cost - c
            if gain > best_gain:
                best_gain = gain
                best_pair = (i, j)

    if best_pair is not None and best_gain > 1e-12:
        i, j = best_pair
        tour[i], tour[j] = tour[j], tour[i]
        return True

    return False


def improve_vnd(order: List[Bloc], start_pos, max_outer_loops: int = 30):
    """
    Variable Neighborhood Descent:
      relocate(1) -> relocate(2) -> swap -> 2-opt
    jusqu'à stagnation.
    """
    B = list(order)
    n = len(B)
    if n <= 1:
        return B, total_cost_for_order(B, start_pos)

    tour = list(range(n))

    for _ in range(max_outer_loops):
        improved = False

        if _relocate_best_improvement(tour, B, start_pos, block_len=1):
            improved = True
        if _relocate_best_improvement(tour, B, start_pos, block_len=2):
            improved = True
        if _swap_best_improvement(tour, B, start_pos):
            improved = True
        if _two_opt_best_improvement(tour, B, start_pos):
            improved = True

        if not improved:
            break

    out = [B[i] for i in tour]
    return out, total_cost_for_order(out, start_pos)


# ---- 4.4 GRASP + VND ----

def plan_grasp_vnd(blocs, start_pos, R: int = 100, rcl_size: int = 8, seed: Optional[int] = None, vnd_loops: int = 20):
    """
    GRASP:
      - Construction: cheapest-insertion mais choix random dans RCL (restricted candidate list)
      - Amélioration: VND
    """
    rng = random.Random(seed)
    B = _as_bloc_list(blocs)
    n = len(B)
    if n == 0:
        return [], 0.0
    if n == 1:
        return [B[0]], total_cost_for_order([B[0]], start_pos)

    best_order = None
    best_cost = float("inf")

    idxs_all = list(range(n))

    for _ in range(R):
        idxs = idxs_all[:]
        rng.shuffle(idxs)

        tour = [idxs[0], idxs[1]]
        remaining = set(idxs[2:])

        while remaining:
            candidates = []
            for node in remaining:
                pos, d = _best_insertion_position(tour, node, B, start_pos)
                candidates.append((d, node, pos))
            candidates.sort(key=lambda x: x[0])

            kk = min(rcl_size, len(candidates))
            _, node, pos = candidates[rng.randrange(kk)]

            tour.insert(pos, node)
            remaining.remove(node)

        order0 = [B[i] for i in tour]
        order1, cost1 = improve_vnd(order0, start_pos, max_outer_loops=vnd_loops)

        if cost1 < best_cost:
            best_cost = cost1
            best_order = order1

    return best_order, best_cost


# ---- 4.5 Wrappers pratiques (sans lambda, friendly multiprocessing spawn) ----

def plan_regret3_plus_vnd(blocs, start_pos, seed: Optional[int] = None):
    order, _ = plan_regret_insertion(blocs, start_pos, k=3, seed=seed)
    return improve_vnd(order, start_pos, max_outer_loops=25)

def plan_lookahead_L8_k2_plus_vnd(blocs, start_pos, seed: Optional[int] = None):
    order, _ = plan_lookahead_insertion(blocs, start_pos, L=8, k=2, seed=seed)
    return improve_vnd(order, start_pos, max_outer_loops=25)

def plan_grasp_R200_vnd(blocs, start_pos):
    return plan_grasp_vnd(blocs, start_pos, R=200, rcl_size=8, seed=123, vnd_loops=20)


# -------------------- Exports (optionnel) --------------------

__all__ = [
    "compute_distance_matrix",
    "total_cost_for_order",
    # exact / baseline
    "plan_optimal_bruteforce",
    "plan_exact_tsp",
    # heuristics classiques
    "plan_heuristic_tsp",
    "plan_nearest_neighbor",
    "plan_cheapest_insertion",
    "plan_greedy_then_swap_improve",
    "plan_random_restart_greedy",
    # exact branch & bound
    "plan_bnb_basic",
    "plan_bnb_heuristic",
    # heuristics lourdes
    "plan_regret_insertion",
    "plan_lookahead_insertion",
    "improve_vnd",
    "plan_grasp_vnd",
    "plan_regret3_plus_vnd",
    "plan_lookahead_L8_k2_plus_vnd",
    "plan_grasp_R200_vnd",
]
