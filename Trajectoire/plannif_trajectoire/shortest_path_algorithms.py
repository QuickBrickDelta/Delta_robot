import numpy as np
import itertools
import random
import config_traj  # Pour les variables globales
from other_fct_traj import cost_do_bloc_from, output_pos_for_color, distance_between_3_points

from python_tsp.exact import solve_tsp_dynamic_programming
from python_tsp.heuristics import solve_tsp_local_search

# importer les variables globales depuis config_traj
blocs = config_traj.blocs
home_position = config_traj.home_position
MAX_BLOCS_OPTIMAL = config_traj.MAX_BLOCS_OPTIMAL

'''
These are all TSP (planning) algorithms to order a set of "blocs" to minimize total cost.
'''

### Distance matrix computation for TSP solver
def compute_distance_matrix(blocs, start_pos):
    n = len(blocs)
    dist = np.zeros((n + 1, n + 1), dtype=float)

    # 0 -> i : home -> bloc_i -> output(bloc_i)
    for i in range(n):
        dist[0, i+1] = cost_do_bloc_from(start_pos, blocs[i])

    # i -> j : output(bloc_i) -> bloc_j -> output(bloc_j)
    for i in range(n):
        pos_i_out = output_pos_for_color(blocs[i][0])
        for j in range(n):
            if i == j:
                dist[i+1, j+1] = 0.0
            else:
                dist[i+1, j+1] = cost_do_bloc_from(pos_i_out, blocs[j])

    return dist


### -------------- Optimal planning function ----------------

def plan_optimal_bruteforce(blocs, start_pos):
    # O(n!) !!!
    # Traveling Salesman Problem
    best_order = None
    best_total = float("inf")

    for perm in itertools.permutations(blocs):
        pos = start_pos
        total = 0.0
        for b in perm:
            total += cost_do_bloc_from(pos, b)
            pos = output_pos_for_color(b[0])
        if total < best_total:
            best_total = total
            best_order = perm

    return list(best_order), best_total

def plan_exact_tsp(blocs, start_pos):
    ## O(n^2 * 2^n) avec solve_tsp_dynamic_programming
    if len(blocs) == 0:
        return [], 0.0

    dist_matrix = compute_distance_matrix(blocs, start_pos)

    permutation, _ = solve_tsp_dynamic_programming(dist_matrix)

    # enlever 0
    order = [blocs[i - 1] for i in permutation if i != 0]

    # IMPORTANT: recalculer avec TON modèle (sans retour implicite)
    true_cost = total_cost_for_order(order, start_pos)
    return order, true_cost


### -------------- Low computing planning function ----------------

def plan_heuristic_tsp(blocs, start_pos):
    # O(n^2) avec solve_tsp_local_search
    n = len(blocs)
    if n == 0:
        return [], 0.0

    dist_matrix = compute_distance_matrix(blocs, start_pos)

    # solution heuristique
    permutation, _ = solve_tsp_local_search(dist_matrix)

    # convertir la permutation (indices) en ordre de blocs
    order = [blocs[i - 1] for i in permutation if i != 0]

    # IMPORTANT: recalculer avec TON modèle (sans retour implicite)
    true_cost = total_cost_for_order(order, start_pos)
    return order, true_cost

def total_cost_for_order(order, start_pos):
    pos = start_pos
    total = 0.0
    for b in order:
        total += cost_do_bloc_from(pos, b)
        pos = output_pos_for_color(b[0])
    return total

def plan_cheapest_insertion(blocs, start_pos):
    #O(n^3)
    blocs = [tuple(b) for b in blocs]
    remaining = list(blocs)
    if not remaining:
        return [], 0.0

    # Démarre avec 1 bloc: celui le moins cher depuis start
    first = min(remaining, key=lambda b: cost_do_bloc_from(start_pos, b))
    order = [first]
    remaining.remove(first)

    while remaining:
        best_total = float("inf")
        best_b = None
        best_pos = None

        for b in remaining:
            # tester toutes les positions d'insertion
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


def plan_nearest_neighbor(blocs, start_pos):
    #O(n^2)
    remaining = list(blocs)
    order = []
    pos = start_pos
    total = 0.0

    while remaining:
        # choisir le bloc qui minimise (pos -> bloc -> sortie)
        best_i, best_cost = None, float("inf")
        for i, b in enumerate(remaining):
            c = cost_do_bloc_from(pos, b)
            if c < best_cost:
                best_cost = c
                best_i = i

        b = remaining.pop(best_i)
        order.append(b)
        total += best_cost

        # après dépôt, nouvelle position = sortie du bloc
        pos = output_pos_for_color(b[0])

    return order, total

def plan_greedy_then_swap_improve(blocs, start_pos, max_passes=20):
    # mix des deux précédents
    # O(n^2) pour le greedy + O(n^3) pour l'amélioration par swap
    # solution initiale: ton greedy
    order, best_cost = plan_nearest_neighbor(blocs, start_pos)

    def try_swap(order, i, j):
        new_order = order[:]
        new_order[i], new_order[j] = new_order[j], new_order[i]
        return new_order

    improved = True
    passes = 0

    while improved and passes < max_passes:
        improved = False
        passes += 1

        for i in range(len(order) - 1):
            for j in range(i + 1, len(order)):
                candidate = try_swap(order, i, j)
                c = total_cost_for_order(candidate, start_pos)
                if c < best_cost:
                    order = candidate
                    best_cost = c
                    improved = True

    return order, best_cost

def plan_random_restart_greedy(blocs, start_pos, restarts=200, k=3, seed=0):
    """
    Heuristique: relance un greedy plusieurs fois.
    À chaque étape, au lieu de choisir STRICTEMENT le meilleur bloc,
    on choisit aléatoirement parmi les k meilleurs (k>=1).

    restarts: nb d'essais
    k: taille du "top-k" candidates pour randomiser (k=1 => greedy pur)
    seed: reproductibilité
    """
    rng = random.Random(seed)

    best_order = None
    best_total = float("inf")

    blocs_list = list(blocs)

    for _ in range(restarts):
        remaining = blocs_list[:]
        order = []
        pos = start_pos
        total = 0.0

        while remaining:
            # calcule les coûts depuis la position courante
            scored = [(cost_do_bloc_from(pos, b), idx, b) for idx, b in enumerate(remaining)]
            scored.sort(key=lambda t: t[0])

            # choisir au hasard dans les k meilleurs
            kk = min(k, len(scored))
            _, chosen_idx, chosen_bloc = rng.choice(scored[:kk])

            # appliquer le choix
            remaining.pop(chosen_idx)
            order.append(chosen_bloc)

            step_cost = cost_do_bloc_from(pos, chosen_bloc)
            total += step_cost
            pos = output_pos_for_color(chosen_bloc[0])

        if total < best_total:
            best_total = total
            best_order = order

    return best_order, best_total


# ===================== Branch & Bound (chemin ouvert, exact) =====================

def _build_cost_tables(blocs, start_pos):
    """
    Pour indices i,j sur blocs:
      cost_start[i] = coût start_pos -> bloc_i -> out(bloc_i)
      cost_ij[i][j] = coût out(bloc_i) -> bloc_j -> out(bloc_j)
    """
    blocs_list = [tuple(b) for b in blocs]
    n = len(blocs_list)

    cost_start = [0.0] * n
    for i in range(n):
        cost_start[i] = cost_do_bloc_from(start_pos, blocs_list[i])

    cost_ij = [[0.0] * n for _ in range(n)]
    for i in range(n):
        out_i = output_pos_for_color(blocs_list[i][0])
        for j in range(n):
            if i == j:
                cost_ij[i][j] = 0.0
            else:
                cost_ij[i][j] = cost_do_bloc_from(out_i, blocs_list[j])

    return blocs_list, cost_start, cost_ij


def plan_bnb_basic(blocs, start_pos):
    """
    Branch & Bound EXACT pour ton modèle (chemin ouvert).
    Complexité pire cas O(n!), mais prune beaucoup.
    """
    blocs_list, cost_start, cost_ij = _build_cost_tables(blocs, start_pos)
    n = len(blocs_list)
    if n == 0:
        return [], 0.0
    if n == 1:
        return [blocs_list[0]], cost_start[0]

    # Upper bound initial : ton greedy (rapide)
    best_order, best_cost = plan_nearest_neighbor(blocs_list, start_pos)

    # Lower bound admissible :
    # - on devra quitter chaque noeud non-visité au moins une fois,
    # - sauf le dernier (chemin ouvert) => on soustrait la plus grosse "min_out"
    min_out = [0.0] * n
    for i in range(n):
        m = float("inf")
        for j in range(n):
            if i != j:
                m = min(m, cost_ij[i][j])
        # si n==1, pas le cas; ici n>=2
        min_out[i] = m

    def bound(cur_idx, unv, cur_cost):
        # cur_idx = -1 si on est encore au start
        if not unv:
            return cur_cost

        if cur_idx == -1:
            min_from_cur = min(cost_start[j] for j in unv)
        else:
            min_from_cur = min(cost_ij[cur_idx][j] for j in unv)

        # optimistic outgoing for remaining nodes
        outs = [min_out[j] for j in unv]
        # dernier noeud n’a pas besoin de sortie dans un chemin ouvert
        return cur_cost + min_from_cur + (sum(outs) - max(outs))

    best_idx_order = None
    nodes_expanded = 0

    def dfs(cur_idx, unv, idx_path, cur_cost):
        nonlocal best_cost, best_idx_order, nodes_expanded
        nodes_expanded += 1

        if bound(cur_idx, unv, cur_cost) >= best_cost:
            return

        if not unv:
            if cur_cost < best_cost:
                best_cost = cur_cost
                best_idx_order = idx_path[:]
            return

        # Branching: essayer les prochains les moins chers d’abord (prune +)
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
            dfs(nxt, new_unv, idx_path + [nxt], new_cost)

    dfs(-1, list(range(n)), [], 0.0)

    if best_idx_order is None:
        # fallback (ne devrait pas arriver)
        return best_order, best_cost

    order = [blocs_list[i] for i in best_idx_order]
    return order, best_cost


def plan_bnb_heuristic(blocs, start_pos, max_passes_swap=20):
    """
    Branch & Bound EXACT + heuristiques de pruning :
      - Upper bound initial plus serré: greedy + swap improve
      - Branching order: plus prometteur d'abord
      - Même borne admissible que basic (simple & safe)
    """
    blocs_list, cost_start, cost_ij = _build_cost_tables(blocs, start_pos)
    n = len(blocs_list)
    if n == 0:
        return [], 0.0
    if n == 1:
        return [blocs_list[0]], cost_start[0]

    # Upper bound initial meilleur
    best_order, best_cost = plan_greedy_then_swap_improve(blocs_list, start_pos, max_passes=max_passes_swap)

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
    nodes_expanded = 0

    def dfs(cur_idx, unv, idx_path, cur_cost):
        nonlocal best_cost, best_idx_order, nodes_expanded
        nodes_expanded += 1

        if bound(cur_idx, unv, cur_cost) >= best_cost:
            return

        if not unv:
            if cur_cost < best_cost:
                best_cost = cur_cost
                best_idx_order = idx_path[:]
            return

        # Branching: ordre "meilleur prochain coup"
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
            dfs(nxt, new_unv, idx_path + [nxt], new_cost)

    dfs(-1, list(range(n)), [], 0.0)

    if best_idx_order is None:
        return best_order, best_cost

    order = [blocs_list[i] for i in best_idx_order]
    return order, best_cost


