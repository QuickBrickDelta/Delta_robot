import numpy as np
from pathlib import Path

CALIB_PATH = Path(__file__).resolve().parent / "homography_plane.npz"
data = np.load(CALIB_PATH)

print(data.files)
print(data["H"])
print(data["imgW"], data["imgH"])
print(data["cam_center_world"])
