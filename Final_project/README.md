#Final project for ROVI1

#Directory structure
Code for vision is in the VIS folder.
Code to detect keypoints in marker1 is in the Vision class
Code to detect keypoints in marker2b is in the LineFinding class

Code for Robotics is in the ROB folder
It is split into two files, SamplePlugin.cpp and Visualservoing.cpp
Visualservoing handles computations with relation to kinematics, SamplePlugin handles the rest.


#Requirements
RobWork Studio (good luck with compiling it...)
OpenCV 2 or 3.
stdc++11 compatible compiler

#How to compile
mkdir build
cd build
cmake ..
make
You can use the defines in SamplePlugin.hpp to change how the tracking is performed.
You can use the defines in Vision.cpp and Linefinding.cpp to control if they should work for ROBWORK or the marker series.


We compile 3 files
libSamplePlugin.so -- The sampleplugin which can be loaded into robworks
marker1_test -- Test runs on marker1
marker2_test --Test runs on marker2b

For using the marker tests, run the relevant executable with the paths to the images you want to run analysis on as the argument. The program will loop through them and for each on print how many keypoints was found and how long it took (in nanoseconds)

When running the sample plugin, press start to load the background and marker images.
Press stop to start the tracking (Yes, very intuiative)

You need to have SamplePluginPA10 located in /home/student/Downloads/ as well as PA10WorkCell
(yes, nice hardcoded paths...)
