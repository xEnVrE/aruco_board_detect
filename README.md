# aruco_board_detect

ROS package to detect aruco boards and markers in images, estimating their pose. Written in C++ and Python.

## Prerequisites (assuming Ubuntu 20.04)

1. Install `OpenCV`
  ```console
  sudo apt install libopencv-dev
  ```

## Installation instructions

1. Clone within a working ROS workspace $WS
  ```console
  cd $WS/src
  git clone https://github.com/xenvre/aruco_board_detect
  ```

1. Build
  ```console
  cd $WS
  catkin build
  ```

## Usage

Once a camera is plugged in your rig and its node is running, simply use the `roslaunch` file provided. Some `roslaunch` parameters you might find useful:

| Parameter | Effect |
| --- | --- |
| `camera_info_topic`               | The node will subscribe to this topic to source camera parameters |
| `camera_image_topic`              | The node will subscribe to this topic to source input images |
| `show_debug_img`                  | Shows the output image in a window |
| `publish_single_markers`          | Enable single marker detection |
| `detection_rate`                  | Time (second) between marker detections |
| `board_config_file`               | Config file for the marker board |
| `single_markers_config_file`      | Config file for the single markers |

You can change the configuration of the marker board being sought in the `cfg/board_config.yaml` file. Same goes for individual markers.

The package also contains a script to generate fiducial markers.

## Expected output

The node will broadcast a tf frame named `aruco_board` (the board reference frame) and one named `graspa_board` (the same frame, shifted on the bottom right of the board).

### Published topics

| Topic | Explanation |
| - | - |
| `/aruco_board_detector/board_pose` | The marker board pose (stamped with the camera reference frame) |
| `/aruco_board_detector/debug_image` | The output image, i.e. the input image with markers drawn on it |
| `/aruco_board_detector/marker_data` | Stamped 6D pose of every single marker detected, with ID |

### Subscribed topics

| Topic | Explanation |
| - | - |
| `camera_info_topic` | Camera parameters topic |
| `camera_image_topic` | Camera image topic |

### Example command

```
roscore
roslaunch realsense2_camera rs_rgbd.launch
roslaunch aruco_board_detect aruco_board_detect.launch publish_single_markers:=true show_debug_img:=true
```

<img src=assets/output.jpeg width=1000/>

### Script example command

The following command can be used to generate a 400x400 px marker (ID 44) with a 100 px white border (quiet zone) and a 50 px black border. `DICT_4X4_50` is the dictionary, as defined by the OpenCV headers.

```
python `rospack find aruco_board_detect`/scripts/generate_aruco.py -o marker_44.png -w 100 -b 50 -s 400 -i 44 -t DICT_4X4_50

```

<img src=assets/marker_44.png width=400/>
