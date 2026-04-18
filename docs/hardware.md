# Documentation Matérielle

Ce guide couvre l'assemblage physique, l'électronique et la liste des composants (BOM) du Robot Delta.

## Liste des composants à acheter (BOM)
| Image | Composant | Description | Quantité |
|-------|-----------|-------------|----------|
| <img width="112" alt="image" src="https://github.com/user-attachments/assets/79218fda-4ba6-41e4-b102-d2806592d881" /> | **Moteurs** | Dynamixel XC430-W240 (ou équivalent) | 3 |
| <img width="112" alt="image" src="https://github.com/user-attachments/assets/48a50fdb-edff-48ba-a590-18e0012d24dc" /> | **Contrôleur** | OpenRB-150 (compatible Arduino) | 1 |
| <img width="112" alt="image" src="https://github.com/user-attachments/assets/aa68dc4d-1664-4c22-bcb0-4b5a6fb88da6" /> | **Alimentation** | Bloc DC 12V 5A | 1 |
| <img width="112" alt="image" src="https://github.com/user-attachments/assets/1d09d913-0b13-4f0c-ba2d-909a5c9281eb" /> | **Joint à rotule** | Joint à rotule fileté M4 | 12 |
| <img width="112" alt="image" src="https://github.com/user-attachments/assets/5d0b0c94-f7e8-4888-9cf5-b6ac15f7f698" /> | **Tiges** | Tige filetée M4-30mm | 6 |
| <img width="112" alt="image" src="https://github.com/user-attachments/assets/81c8e887-966b-428b-b957-b3c6305357cd" /> | **Caméra ** | Caméra module 3 Raspberry Pi | 1 |
| <img width="112" alt="image" src="https://github.com/user-attachments/assets/6e59d47c-f8be-48bd-be10-786d8f84be9a" /> | **Servo moteur SG90** | Servo moteur SG90 | 2 |
| <img width="112" alt="image" src="https://github.com/user-attachments/assets/f57d85bb-15fc-4947-af7a-a8d3ca943967" /> | **Bande lumineuse de DEL** | À connexion USB (5V), intensité variable | 1 |
| <img width="112" alt="image" src="https://github.com/user-attachments/assets/343a47da-f884-4245-a464-d9bcfb0623e6" /> | **Attache à tiroir** | Attache à tiroir Richelieu | 1 |
| <img width="112" alt="image" src="https://github.com/user-attachments/assets/2fa09ca0-90e4-4fcf-a44d-8b80e94b3f76" /> | **Fil caméra** | Fil 22 pin à 15 pin pour caméra Raspberry Pi | 1 |
| <img width="112" alt="image" src="https://github.com/user-attachments/assets/654351da-64af-43c1-a66e-e706d72b5a5e" /> | **Extrudé 9 pouces** | Extrudé 9 pouces | 3 |
| <img width="112" alt="image" src="https://github.com/user-attachments/assets/c434e5ce-8826-448e-9ba6-6d3346a1e637" /> | **Extrudé 12 pouces** | Extrudé 12 pouces | 3 |
| <img width="112" alt="image" src="https://github.com/user-attachments/assets/afae6ed6-e6a3-47b0-b2a9-2a09f5db7c9f" /> | **Extrudé 24 pouces** | Extrudé 24 pouces | 3 |
| <img width="112" alt="image" src="https://github.com/user-attachments/assets/e49eb2e4-39da-49b2-9273-a03a61c4f0f3" /> | **Plaque pour extrudés** | Plaque connecteur pour extrudés | 12 |
| <img width="112" alt="image" src="https://github.com/user-attachments/assets/caf67060-d961-4b98-938b-468eeb22e1d3" /> | **Convertisseur buck** | Convertisseur buck 12V à 5V | 1 |
| <img width="112" alt="image" src="https://github.com/user-attachments/assets/7de1da41-5414-4fda-9424-fd76470347f2" /> | **Feuille d'acrylique** | Plaque d'acrylique 3mm | 1 |
| <img width="112" alt="image" src="https://github.com/user-attachments/assets/10237d91-6230-498f-b80f-ebaee7c5f138" /> | **Plaques de bois (découpe laser)** | Plaques de bois 4.3mm — utiliser les retailles lorsque possible | 8 |

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
