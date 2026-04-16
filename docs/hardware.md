# Documentation Matérielle

Ce guide couvre l'assemblage physique, l'électronique et la liste des composants (BOM) du Robot Delta.

## Liste des composants (BOM)

| Composant | Description | Quantité |
|-----------|-------------|----------|
| **Moteurs** | Dynamixel XC430-W240 (ou équivalent) | 3 |
| **Contrôleur** | OpenRB-150 (compatible Arduino) | 1 |
| **Alimentation** | Bloc DC 12V 5A | 1 |
| **Bras** | Tiges en fibre de carbone ou bras imprimés 3D | 3 ensembles |
| **Effecteur** | Effecteur imprimé 3D avec pince/aimant | 1 |
| **Caméra** | Raspberry Pi Camera Module 3 (ou Webcam USB) | 1 |

## Schéma de câblage

1.  **Moteurs Dynamixel** : Connectez les 3 moteurs en "daisy-chain" et reliez-les à l'un des ports Dynamixel de l'OpenRB-150.
2.  **Alimentation** : Connectez le bloc 12V au bornier de l'OpenRB-150. **ATTENTION** : Vérifiez la polarité.
3.  **Communication** : Reliez l'OpenRB-150 au Raspberry Pi (ou PC) via un câble USB-C.

![Schéma Électrique](../Schéma_Électrique_Delta.drawio)
*(Exportez ce diagramme en PNG pour une meilleure visibilité dans le README)*

## Étapes d'assemblage

1.  **Base** : Montez les 3 moteurs Dynamixel à 120° sur le châssis principal.
2.  **Parallélogrammes** : Assemblez les 6 tiges parallèles à l'aide des joints à rotule.
3.  **Effecteur** : Montez l'effecteur final (pince) et connectez le signal du servo aux broches de l'OpenRB-150.
4.  **Calibration** : Assurez-vous que les moteurs sont à 0° (horizontaux) avant de fixer les bras principaux.
