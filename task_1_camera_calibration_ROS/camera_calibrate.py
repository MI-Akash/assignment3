from machinevisiontoolbox import Image, CentralCamera, ImageCollection
import matplotlib.pyplot as plt
import numpy as np
import yaml
import cv2  

# Load checkerboard images 
raw_images = ImageCollection("images/*.png")  
print(f"Loaded {len(raw_images)} calibration images")


resized_images = []
for img in raw_images:
    resized_array = cv2.resize(img.image, (1920, 1080), interpolation=cv2.INTER_AREA)
    resized_images.append(Image(resized_array))

# Calibrate camera using checkerboard

K, distortion, frames = CentralCamera.images2C(
    resized_images,
    gridshape=(7, 10),
    squaresize=0.020  
)


print("\nIntrinsic Matrix (K):\n", K)
print("\nDistortion Coefficients [k1, k2, p1, p2, k3]:\n", distortion)


for frame in frames:
    CentralCamera.plot(pose=frame.pose, scale=0.05)
plt.show()


camera_config = {
    "Camera": {
        "name": "MyCalibratedCamera",
        "setup": "monocular",
        "model": "perspective",
        "fx": float(K[0, 0]),
        "fy": float(K[1, 1]),
        "cx": float(K[0, 2]),
        "cy": float(K[1, 2]),
        "k1": float(distortion[0]),
        "k2": float(distortion[1]),
        "p1": float(distortion[2]),
        "p2": float(distortion[3]),
        "k3": float(distortion[4]),
        "fps": 30.0,
        "cols": 1920,
        "rows": 1080,
        "color_order": "RGB"
    }
}

with open("camera_congif.yaml", "w") as f:
    yaml.dump(camera_config, f, sort_keys=False)

print("Camera calibration file saved as camera_config.yaml")
