#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
One-shot main:
1) Récupère un tuple stable depuis la Vision (imprimé + retourné)
2) Planifie la trajectoire (BnB / TSP / Cheapest Insertion)
3) Anime la trajectoire 3D
"""

from __future__ import annotations
import sys
from pathlib import Path
import numpy as np

# ---------------------------------------------------------------------
# 1) Ajouter VisionNumerique/ et Trajectoire/plannif_trajectoire/ au sys.path
# ---------------------------------------------------------------------
THIS_FILE = Path(__file__).resolve()
PROJECT_ROOT = THIS_FILE.parent.parent  # .../Delta_robot

VISION_DIR = PROJECT_ROOT / "VisionNumerique"
TRAJ_SUBDIR = PROJECT_ROOT / "Trajectoire" / "plannif_trajectoire"

for p in (VISION_DIR, TRAJ_SUBDIR):
    p_str = str(p)
    if p_str not in sys.path:
        sys.path.insert(0, p_str)

# ---------------------------------------------------------------------
# 2) Imports après injection du path
# ---------------------------------------------------------------------
# Vision (fichier: VisionNumerique/stable_block_output.py)
from stable_block_output import Context, print_stable_blocks_once

# Trajectoire (dossier: Trajectoire/plannif_trajectoire/)
from config_traj import home_position
from shortest_path_algorithms import plan_bnb_basic, plan_exact_tsp, plan_cheapest_insertion
from plannif_trajectoire import plan_full_trajectory
from animation_and_plot_traj import animate_full_trajectory_3D

# ---------------------------------------------------------------------

def plan_and_animate_from(blocs_obj) -> None:
    """
    blocs_obj : list[tuple] -> [(color, "2x4", Xcm, Ycm, angle_deg), ...]
    """
    # Sécurisation du type
    if not blocs_obj or not isinstance(blocs_obj, (list, tuple)):
        print("Aucun bloc détecté : rien à planifier.")
        return

    # Déjà une LISTE de TUPLES : on l'utilise directement
    blocs_list = list(blocs_obj)
    n = len(blocs_list)


    # Choix de l’algorithme (même logique que ta pipeline Trajectoire)
    if n < 11:
        blocs_sorted, total_distance = plan_bnb_basic(blocs_list, home_position)
        print(f"[Planner] Branch-and-Bound | distance totale = {total_distance:.2f}")
    elif n < 14:
        blocs_sorted, total_distance = plan_exact_tsp(blocs_list, home_position)
        print(f"[Planner] TSP exact | distance totale = {total_distance:.2f}")
    else:
        blocs_sorted, total_distance = plan_cheapest_insertion(blocs_list, home_position)
        print(f"[Planner] Cheapest Insertion | distance (approx) = {total_distance:.2f}")

    print("Ordre retenu :", blocs_sorted)

    # Trajectoire complète (pick → place → home)
    full_path = plan_full_trajectory(blocs_sorted)

    print("Trajectoire complète (résumé) :")
    for step in full_path:
        print(step)

    # Animation 3D (bloquante tant que la fenêtre est ouverte)
    animate_full_trajectory_3D(
        full_path,
        blocs=blocs_sorted,
        home_position=home_position,
        dt=0.05,
        show_trace=True
    )

# ---------------------------------------------------------------------
def main() -> None:
    # 1) Init Vision une seule fois (ouvre la caméra + charge l'homographie)
    ctx = Context()
    try:
        # 2) One-shot : obtenir un tuple stable depuis la Vision
        blocs = print_stable_blocks_once(ctx)  # blocs est une LISTE de TUPLES
        if blocs:
            plan_and_animate_from(blocs)
        else:
            print("Tuple vide / invalide, fin.")
    finally:
        # 4) Libérer proprement la caméra
        ctx.close()

# ---------------------------------------------------------------------
if __name__ == "__main__":
    main()