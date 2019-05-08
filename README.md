# Trashbot: Environmentally Friendly Robot
Trashbot Project for COMP150: Probabilistic Robotics

Using Baymax 7

## TODO
1. YOLO is very slow on turtlebot laptops (can't do much about it?)
2. Moving close to trash using pointcloud/depthcloud.

## Environments
Deep learning Environment
  - Package dependencies:
    - tensorflow 1.12.0
    - opencv 3.4.2
    - numpy
    - matplotlib
    - seaborn
    - pillow

Building opencv 3.4.6 using Python 3 and virtual environment
1. Install following packages using "sudo apt install"
  - build-essential cmake unzip pkg-config liblapacke-dev libjpeg-dev libpng-dev libtiff-dev libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libxvidcore-dev libx264-dev libgtk-3-dev libatlas-base-dev gfortran libtiff5-dev libjasper-dev libpng12-dev libhdf5-serial-dev graphviz libopenblas-dev python-tk python3-tk python-imaging-tk
2. Turn on the following options when running ccmake
  - BUILD_EXAMPLES, INSTALL_PYTHON_EXAMPLES, OPENCV_ENABLE_NONFREE, OPENCV_EXTRA_MODULES_PATH, with_QT, CMAKE_BUILD_TYPE=Release, and CUDA if cuda is available.
  - cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local -D BUILD_EXAMPLES=ON -D INSTALL_PYTHON_EXAMPLES=ON -D OPENCV_ENABLE_NONFREE=ON -D OPENCV_EXTRA_MODULES_PATH=/home/js0823/Downloads/opencv_contrib/modules/ -D WITH_QT=ON -D WITH_CUDA=ON
3. Finally, make and install.

More information are here.
https://www.pyimagesearch.com/2016/10/24/ubuntu-16-04-how-to-install-opencv/

## Training Dataset
Dataset from trashbot.
I will be testing plastic, paper, and metal first, then expand if I have time.

## darknet
- On Geforce 1060 6GB, darknet fails when using, for example, yolov3.weights with COCO dataset pretrained on dog.jpg. The author of darknet says "Note: if error Out of memory occurs then in .cfg-file you should increase subdivisions=16, 32 or 64". This works well.

Training command ex: ./darknet detector train trashnet/trashnet.data trashnet/yolov3.cfg trashnet/darknet53.conv.74

Testing command ex: ./darknet detector test trashnet/trashnet.data trashnet/yolov3.cfg backup/yolov3_6000.weights /home/js0823/github-repo/darknet/trashnet/data/images/metal146.jpg

To make this work nicely on ubuntu, check the following.
1. Run "sudo apt install libomp-dev" to make OpenMP work on darknet.
2. Make sure opencv is built.
3. Make sure cuda and cudann is installed.
4. Change Makefile so that GPU=1, CUDNN=1, OPENCV=1, and OPENMP=1.

## Running the whole thing
1. Build darknet.
2. catkin_make the trashbot.
3. Make sure devel.bash is sourced.
4. Run "roslaunch trashbot_launch trashbot_lidar.launch"
5. Run "roslaunch amcl_navigation.launch"
6. Run "roslaunch trashbot_roamer.launch"
7. Run "roslaunch yolo_detector.launch"

Note: I should merge trashbot_roamer and yolo_detector into one file.

## Other comments
1. When building Tufts turtlebot from source, it gets error message saying "move_base_msgs" is not found. Run ros-kinetic-move-base in this case.
2. When building Tufts turtlebot from source, it gets error message "could not find em". Run "pip install empy".
3. Install catkin command by "pip install catkin_tools"
4. To record video, follow https://learn.turtlebot.com/2015/02/04/4/
5. Must manually install turtlebot via "sudo apt install ros-kinetic-turtlebot"
6. Must manually install turtlebot rviz launchers via "sudo apt install ros-kinetic-turtlebot-rviz-launchers"
7. Must manually install turtlebot navigation via "sudo apt install ros-kinetic-turtlebot-navigation"
