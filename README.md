# Trashbot: Environmentally Friendly Robot
Trashbot Project for COMP150: Probabilistic Robotics

## TODO
1. Get YOLO working and setup training data for training and testing.
2. Get video file from turtlebot which contains trashes around the floor.
3. Test on the video file and tune YOLO as needed.
4. Write code for the turtlebot to move towards the trash when detected and voice out.
5. Fix as needed and might have to deal with performance.

## Environments
Deep learning Environment
  - Anaconda Python 3.6
  - Package dependencies:
    - tensorflow 1.12.0
    - opencv 3.4.2
    - numpy
    - matplotlib
    - seaborn
    - pillow

ROS
  - Kinetic

Building opencv 3.4.2 and contrib on Ubuntu's Anaconda environment
1. Install following packages using "sudo apt install"
  - build-essential cmake unzip pkg-config liblapacke-dev libjpeg-dev libpng-dev libtiff-dev libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libxvidcore-dev libx264-dev libgtk-3-dev libatlas-base-dev gfortran libtiff5-dev libjasper-dev libpng12-dev libhdf5-serial-dev graphviz libopenblas-dev python-tk python3-tk python-imaging-tk
2. Turn on the following options when running ccmake
  - BUILD_EXAMPLES, INSTALL_PYTHON_EXAMPLES, OPENCV_ENABLE_NONFREE, OPENCV_EXTRA_MODULES_PATH
3. Finally, make and install.

More information are here.
https://dev.widemeadows.de/2017/08/23/building-opencv-for-anaconda-python-3/

## Training Dataset
Dataset from trashbot.
I will be testing plastic, paper, and metal first, then expand if I have time.
