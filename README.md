# aruco_board_detect
ROS package to detect aruco boards from images

Set the environment variable `OpenCV_DIR` pointing to a custom openCV build, since this package doesn't work with OpenCV 3.2 bundled with Ubuntu Bionic + ROS Melodic. Use
 ```
catkin build -DOpenCV_DIR=${OpenCV_DIR}
 ```
or
```
catkin_make -DOpenCV_DIR=${OpenCV_DIR}
```
to build the workspace. On Ubuntu Bionic I had to recompile dependencies of this package that also rely on OpenCV:
```
common_msgs (jade-devel)
image_common (hydro-devel)
vision_opencv (melodic)
```


#TODO list
write launchfile and remap topics
make debug windows optional with cmd argument
broadcast tf transform of the bottom right corner of the board too

