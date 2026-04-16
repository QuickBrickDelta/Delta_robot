# Guide de Contribution

Merci de l'intérêt que vous portez au projet Delta Robot ! Nous accueillons avec plaisir les contributions de la communauté pour améliorer les performances, le système de vision et la documentation du robot.

## Comment contribuer

1.  **Signaler des bugs** : Ouvrez une "Issue" décrivant le bug, incluant les étapes pour le reproduire et votre configuration matérielle.
2.  **Suggérer des fonctionnalités** : Ouvrez une "Issue" avec le tag [Feature Request].
3.  **Pull Requests** :
    *   Faites un "Fork" du dépôt.
    *   Créez une nouvelle branche (`feature/ma-super-fonctionnalite` ou `fix/bug-critique`).
    *   Assurez-vous que votre code suit les standards PEP8.
    *   Ajoutez des docstrings aux nouvelles fonctions.
    *   Soumettez une PR vers la branche `main`.

## Configuration de développement

1.  Clonez le dépôt : `git clone https://github.com/votre-utilisateur/Delta_robot.git`
2.  Créez un environnement virtuel : `python -m venv venv`
3.  Installez les dépendances : `pip install -r requirements.txt`
4.  Lancez l'interface de simulation : `python UI/main.py`

## Standards de codage

-   Utilisez Python 3.10+
-   Suivez la convention PEP8.
-   Documentez les changements majeurs de cinématique ou de vision dans le dossier `docs/`.
