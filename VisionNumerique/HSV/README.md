## VisionNumerique/HSV

Objectif: regrouper les scripts dont la stratégie couleur est **majoritairement HSV**
(plus robuste aux variations de luminosité que des seuils RGB).

### Calibration HSV (ROI)

Lance:

```bash
python VisionNumerique/HSV/calibrate_hsv_ranges.py
```

Usage:
- **Drag** une ROI sur un bloc
- **1..9** assigne la dernière ROI à une couleur (ex: `1=red`, `2=green`, ...)
- **e** exporte `hsv_ranges.json` dans ce dossier
- **r** reset, **q/ESC** quitter

Le fichier `hsv_ranges.json` contient des plages HSV OpenCV \(H 0..179\) sous forme
de liste d'intervalles `[[loHSV],[hiHSV]]`. Le rouge peut être exporté en **2 intervalles** (wrap autour de 0).

