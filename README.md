<h1>Assignment 3</h1>

## All the codes are in the following folders 
```bash
task_1_camera_calibration_ROS
task_2_apply_camera_calibration_ROS
task_4_understandVsalam
```

<h2>Task 1: Camera Calibration (ROS)</h2>

<figure>
    <figcaption><strong>Capturing checkerboard using in ros2 and publishing as a topic</figcaption>
    <img src="images/camcal.png" alt="Photo 1" width="720">
    
</figure>

<b> Step1: Download to task_1_camera_calibration_ROS.</b>
<b> Step2: source ros2 then run ros2_cam_strm_saver.py.</b>
<b> This python script will save camera frames from the stream to \``images\'' folder.</b>
<b> Step3: run camera_calibrate.py; it will calibrate the camera configuration from the \``images\'' in the images folder and save \``camera_config.yaml\'' file.</b>


<h2>Task 2: Apply Camera Calibration (ROS)</h2>

<figure>
    <figcaption><strong>Estimating camera position in the turtle bot meze environment</figcaption>
    <img src="images/campos.png" alt="Photo 1" width="720">
    
</figure>

## Build and Launch the Docker Environment

```bash
# In the cloned turtlebot3_behavior_demos directory:
docker compose build

# Launch the demo world:
docker compose up demo-world

# Launch the development container:
docker compose up dev

# Access the development container:
docker compose exec -it dev bash

# Navigate to the source folder:
cd src

# Create a new ROS2 Python package:
ros2 pkg create --build-type ament_python ecp_pkg

# Create and edit the main Python script:
nano ecp_pkg/ecp_pkg/ecp.py
# Paste the code from the task_2_apply_camera_calibration_ROS folder into ecp.py.
# Make the script executable:
chmod +x ecp_pkg/ecp_pkg/ecp.py
# Edit the setup.py file:
nano ecp_pkg/setup.py
# Paste the setup.py content from the task_2_apply_camera_calibration_ROS folder into this file.
# Build the specific package:
colcon build --packages-select ecp_pkg

# Source the install setup (only if not already sourced):
source install/setup.bash

# Run the node:
ros2 run ecp_pkg ecp
```


<h2>Task 3: Showcase StellaVSLAM</h2>
## Pleas click and watch the video

[![Watch the video](https://img.youtube.com/vi/gKrJTZa282E/maxresdefault.jpg)](https://youtu.be/gKrJTZa282E)


<h2>Task 4: Understand VSLAM</h2>
<figure>
    <figcaption><strong>VSLAM with python code runs on KITTI dataset</figcaption>
    <img src="images/Vslampython.png" alt="Photo 1" width="720">
    
</figure>

## Python is prepared for VSLAM and tested on KITTI dataset 
```bash
# please download the dataset from KITTI 
# in task_4_understandVSalam folder, open the vslam3.py 
# edit the following with your desired path 
def main():

    # path to the calibration file of the camera
    calib_file = r"E:\Storage\Academic_study\CS_AI_for_Robotics\Assignment3\data_odometry_gray\dataset\sequences\00\calib.txt"
    # path to the images from the datatset
    image_folder = r"E:\Storage\Academic_study\CS_AI_for_Robotics\Assignment3\data_odometry_gray\dataset\sequences\00\image_0"
    # path to saving output trajectory
    output_traj_file = r"E:\Storage\Academic_study\CS_AI_for_Robotics\Assignment3\trajectory.txt"

```
<figure>
    <figcaption><strong>Trajectory of VSLAM with python code runs on KITTI dataset</figcaption>
    <img src="images/trajectory.png" alt="Photo 1" width="720">
    
</figure>
