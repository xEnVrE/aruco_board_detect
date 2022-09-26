# aruco_board_detect

ROS package to detect aruco boards and markers in images, estimating their pose.

## Prerequisites

- Install `OpenCV`
  ```console
  sudo apt install libopencv-dev
  ```

## Installation

1. Clone within a working ROS workspace $WS
  ```console
  cd $WS/src
  git clone https://github.com/xenvre/aruco_board_detect
  ```
2. Build
  ```console
  cd $WS
  catkin build
  ```

## Usage

Once a camera is plugged in your rig and its node is running, simply use the `roslaunch` file provided:

```console
roslaunch aruco_board_detect aruco_board_detect.launch publish_single_markers:=true show_debug_img:=true
```

Some `roslaunch` parameters you might find useful:

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

## Inputs/outputs

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

## Marker generator

The following command shows how to generate an ArUco marker. In the example, we consider a 4.0 x 4.0 cm (`-s 4.0`) marker with ID = 0 (`-i 0`) from the dictonary `DICT_5X5_50` (`-t DICT_5X5_50`).

```console
python `rospack find aruco_board_detect`/scripts/generate_aruco.py -o marker.png -s 4.0 -i 0 -t DICT_5X5_50

```

For futher options, run the above command with `--help`:

```console
python `rospack find aruco_board_detect`/scripts/generate_aruco.py --help
```
