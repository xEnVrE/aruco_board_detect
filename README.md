# aruco_detector

ROS package to detect ArUco markers in images and estimate their pose.

## Prerequisites

- `OpenCV`
- `Eigen3`
  > e.g., in Ubuntu
  >```console
  >sudo apt install libopencv-dev libeigen3-dev
  >```

## Installation

1. Clone within a working ROS workspace $WS
  ```console
  cd $WS/src
  git clone https://github.com/xenvre/aruco_detector
  ```
2. Build
  ```console
  cd $WS
  catkin build
  ```

## Usage

Once a camera is plugged in your rig and its node is running, simply use the `roslaunch` file provided:

```console
roslaunch aruco_board_detect aruco_board_detect.launch [show_debug_image:=true]
```

Some `roslaunch` parameters you might find useful:

| Parameter | Description |
| --- | --- |
| `camera_info_topic`               | The node will subscribe to this topic to source camera parameters |
| `camera_image_topic`              | The node will subscribe to this topic to source input images |
| `show_debug_image`                | Shows the output image in a OpenCV window |
| `detection_period`                | Time between marker detections (seconds)|
| `markers_config_file`             | Config file for marker settings |

Please check the launch file for defaults. Specifically, the file `cfg/markers_config.yaml` is used as default configuration file.

## Published topics

| Topic | Explanation |
| - | - |
| `/aruco_board_detector/marker_pose` | The marker poses (stamped with the camera reference frame) |
| `/aruco_board_detector/debug_image` | The output image, i.e. the input image with markers drawn on it |

The node will also broadcast several tf frames named `marker_<id>` where `<id>` is the ID of the marker.


## Marker generator

The following command shows how to generate an ArUco marker. In the example, we consider a 4.0 x 4.0 cm (`-s 4.0`) marker with ID = 0 (`-i 0`) from the dictonary `DICT_5X5_50` (`-t DICT_5X5_50`).

```console
python `rospack find aruco_detector`/scripts/generate_aruco.py -o marker.png -s 4.0 -i 0 -t DICT_5X5_50

```

For futher options, run the above command with `--help`:

```console
python `rospack find aruco_detector`/scripts/generate_aruco.py --help
```
