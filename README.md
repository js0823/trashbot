# Trashbot: Environmentally Friendly Robot
Trashbot Project for COMP150: Probabilistic Robotics

Using Baymax 7

## Done
1. ROS and OpenCV 3.4.6 installed on both desktop and vmware.
2. Darknet tested on both desktop and vmware.
3. ROS catkin_ws working as intended.
4. All trash images annotated.
5. Trashes trained on Darknet (Retrained to one label: Trash)

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

Installing ROS Kinetic using Python 3
  - Install ros-kinetic-desktop
  - When using virtualenv, catkin_ws may run into error. If that is the case, do "pip install rospkg catkin_pkg" on virtualenv.

More information here: https://ai-mrkogao.github.io/ros/ROSvirtualenv/

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

## Other comments
1. When building Tufts turtlebot from source, it gets error message saying "move_base_msgs" is not found. Run ros-kinetic-move-base in this case.
2. When building Tufts turtlebot from source, it gets error message "could not find em". Run "pip install empy".
3. Install catkin command by "pip install catkin_tools"
