# Trashbot: Environmentally Friendly Robot
Trashbot Project for COMP150: Probabilistic Robotics

NOTE: I used Using Baymax 7. All software are installed on Baymax 7 Turtlebot for testing.

## Environments (Tested only on Ubuntu)

- Non-GPU environment
  - While GPU is essential for YOLO detection to work smoothly, it is not required. Baymax 7 turtlebot didn't have GPU available, but it worked nonetheless. However, I strongly recommend GPU environment as it will increase the performance by 100 folds.
- GPU Environment
  - While I didn't have a GPU testing environment, GPU is highly recommended due to computation required for YOLO to run smoothly. If you have GPU, I recommend using it. To use GPU, install CUDA and CUDNN.
- ROS-kinetic
  - This package depends on ROS. Please install ros-kinetic-desktop-full.
  - These packages may be required to install: sudo apt install ros-kinetic-turtlebot ros-kinetic-turtlebot-rviz-launchers ros-kinetic-turtlebot-navigation ros-kinetic-move-base
- Python 2
  - Python 2 is required as it is standard for ROS.
  - Numpy is required for dependency.
  - I had issue where em couldn't be found. In this case, run "pip install empy".
  - Although not required, it may be good to install "pip install catkin_tools".
- OpenCV
  - OpenCV is required to run this package. Please refer to OpenCV section below if installation is an issue.
- Darknet
  - Darknet is provided in this package, with few modifications. Please refer to Darknet section below.
  
Once all packages above are installed, please follow the Package Installation section below.

## Installing OpenCV 3.4.x on Ubuntu
1. Install following packages using "sudo apt install"
  - build-essential cmake unzip pkg-config liblapacke-dev libjpeg-dev libpng-dev libtiff-dev libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libxvidcore-dev libx264-dev libgtk-3-dev libatlas-base-dev gfortran libtiff5-dev libjasper-dev libpng12-dev libhdf5-serial-dev graphviz libopenblas-dev python-tk python3-tk python-imaging-tk
2. Turn on the following options when running ccmake
  - BUILD_EXAMPLES, INSTALL_PYTHON_EXAMPLES, OPENCV_ENABLE_NONFREE, OPENCV_EXTRA_MODULES_PATH, with_QT, CMAKE_BUILD_TYPE=Release, and CUDA if cuda is available.
  - cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local -D BUILD_EXAMPLES=ON -D INSTALL_PYTHON_EXAMPLES=ON -D OPENCV_ENABLE_NONFREE=ON -D OPENCV_EXTRA_MODULES_PATH=/home/js0823/Downloads/opencv_contrib/modules/ -D WITH_QT=ON -D WITH_CUDA=ON
3. Finally, make and install.

More information are here. Please note I didn't use python's virtual environment as it seems to conflict with ROS environment.
https://www.pyimagesearch.com/2016/10/24/ubuntu-16-04-how-to-install-opencv/

## Darknet

Darknet package is located in the Darknet directory. Depending on whether you have GPU or not, the installation is a bit different.

1. Make sure OpenCV is built and is linked.
2. If you have GPU, make sure CUDA and CUDNN is installed.
3. If you are working on GPU environment, open Makefile and edit GPU=0 to GPU=1, and CUDNN=0 to CUDNN=1. It is off by default.
4. Run "make -j #" where # is the number of CPU.

If you want to train your own model, you can refer to https://towardsdatascience.com/tutorial-build-an-object-detection-system-using-yolo-9a930513643a. After setting up your training data, you only need to follow from section 4 on the link.

Note: If error Out of memory occurs then in .cfg-file you should increase subdivisions=16, 32 or 64". It's due to lack of memory for your GPU.

## Package Installation
1. Build darknet by going into darknet and typing "make -j #" where # is the number of CPU.
2. Run catkin_make from the ROS workspace to build Trashbot.
3. Make sure devel.bash is sourced.
4. Run "roslaunch trashbot_launch trashbot.launch"
5. Run "roslaunch amcl_navigation.launch"
6. Run "roslaunch yolo_detector.launch"
7. Go to the RVIZ window and wait until YOLO shows video streaming at the bottom left.
8. Run "roslaunch trashbot_roamer.launch"

## Other comments
- amcl_navigation.launch reports warning about Costmap2DROS transform timeout. This looks like time synchronization issue, and it plagues the navigation. It seems to be laptop performance issue.
