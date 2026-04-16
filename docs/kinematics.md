# Cinématique du Robot Delta

Ce document explique le modèle mathématique utilisé pour contrôler le Robot Delta à 3 bras.

## Système de coordonnées

- **Origine (0,0,0)** : Centre géométrique de la plaque de base supérieure.
- **Axe X** : Pointe vers le premier bras.
- **Axe Z** : Pointe vers le bas (les valeurs négatives augmentent à mesure que le robot descend).

## Cinématique Inverse (IGM)

La cinématique inverse convertit une position `(P_x, P_y, P_z)` souhaitée de l'effecteur en 3 angles moteurs `(theta_1, theta_2, theta_3)`.

Le robot delta se compose de 3 chaînes identiques. Pour chaque chaîne, nous trouvons l'intersection entre :
1.  Le cercle tracé par le bras supérieur.
2.  La sphère tracée par les tiges parallèles inférieures autour du point de fixation de l'effecteur.

La solution est calculée analytiquement dans `CinématiqueRobot/Cinematique_delta3bras.py`.

## Cinématique Directe (DGM)

La cinématique directe convertit 3 angles moteurs en une position `(x, y, z)`. Elle est utilisée pour la simulation et la visualisation en temps réel dans l'interface utilisateur.

Puisque les mathématiques du robot delta impliquent un système d'équations quadratiques (intersection de 3 sphères), nous utilisons une approche géométrique optimisée pour trouver l'unique intersection valide.

## Planification de Trajectoire

Le robot utilise une **interpolation linéaire** dans l'espace cartésien pour des mouvements fluides.
- **Profil de vitesse en S (S-Curve)** : Utilisé pour minimiser les secousses lors du démarrage et de l'arrêt.
- **Recherche de chemin** : Un algorithme **Branch & Bound** trouve la séquence la plus courte pour visiter tous les blocs détectés.

![Schéma Cinématique](../docs/img/schema_kinematic.png)
*(Prochainement : Diagrammes détaillés des longueurs de bras f, e, rf et re)*
