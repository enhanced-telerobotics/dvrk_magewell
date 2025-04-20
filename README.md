# dvrk_magewell

This ROS 2 package provides functionality for publishing and displaying video streams from stereo cameras. It includes launch files and nodes for configuring and running the video streams with various options.

## Quick Start
### Publishing and Displaying Video Streams
- On dVRK HRSV:
    ```bash
    ros2 launch dvrk_magewell show_hrsv.py
    ```
- On Goovis:
    ```bash
    ros2 launch dvrk_magewell show_goovis.py
    ```

### Directly Displaying Video Streams (No Publishing)
- On Goovis:
    ```bash
    ros2 launch dvrk_magewell local_show_goovis.py
    ```
- Flexible display:
    ```bash
    ros2 run dvrk_magewell local_display_video -d -m -h 720 -w 1280 --left-offset 0
    ```

## Usage
### Launching Stereo Video Streams
The `publish_stereo.py` launch file is used to publish video streams from two cameras (left and right). You can specify the height of the video stream using the `height` argument. By default, the full resolution of the cameras is used.

#### Examples:
- Launch with default resolution (1920x1080):
  ```bash
  ros2 launch dvrk_magewell publish_stereo.py
  ```
- Launch with 1280x720 pixels:
  ```bash
  ros2 launch dvrk_magewell publish_stereo.py height:=720
  ```
- Launch with 960x540 pixels:
  ```bash
  ros2 launch dvrk_magewell publish_stereo.py height:=540
  ```

### Displaying Video Streams
The `display_video` node can be used to display the video streams with various configurations. Below are some examples of how to run the `display_video` node:

#### Examples:
- Display a single video stream in mono mode with a height of 720 pixels and a width of 1280 pixels:
  ```bash
  ros2 run dvrk_magewell display_video -d -m -h 720 -w 1280 --left-offset 0
  ```
  Set `-d` to enable debug mode, `-m` to display in mono mode, and `--left-offset` to 0 to display on the main display.
- Display concatenated stereo video streams with a height of 720 pixels and a width of 1280 pixels:
  ```bash
  ros2 run dvrk_magewell display_video -d -c -h 720 -w 1280 --left-offset 0
  ```
  Remove `-m` to display stereo images. Use `-c` to concatenate stereo images into a single window.
- Display concatenated stereo video streams optimized for Goovis with a height of 1080 pixels and a width of 960 pixels (squeezed stereo to 1920 pixels):
  ```bash
  ros2 run dvrk_magewell display_video -c -h 1080 -w 960 --left-offset 5120
  ```
  Add `--left-offset` because the Goovis is placed on the right side of two WQHD displays. No `--crop` flag is added since the frame ratio needs to be changed.
- Display cropped stereo video streams for HRSV with a height of 768 pixels and a width of 1024 pixels:
  ```bash
  ros2 run dvrk_magewell display_video -h 768 -w 1024 --crop --left-offset 5120
  ```
  Add `--crop` to enable center cropping of the video stream while maintaining the frame ratio.

## Nodes

### `publish_video`
Publishes video streams from a camera to a specified ROS 2 topic.

#### Arguments:
- `--topic`: The topic name to publish the video stream (e.g., `davinci_endo/left/image_raw`).
- `--device`: The camera device ID (e.g., `0` or `2`).
- `--height`: The target height of the video stream (default is 0 for full resolution).

### `display_video`
Displays video streams from ROS 2 topics with options for mono, concatenated, or cropped views.

### `local_display_video`
Displays video streams from a specified camera device without publishing to a ROS 2 topic. This is useful for local testing and debugging.

#### Arguments:
- `-d` or `--disable-window-settings`: Enable debug mode.
- `-m` or `--mono`: Display in mono mode.
- `-c` or `--concat`: Concatenate stereo images.
- `-h` or `--height`: Set the height of the display window.
- `-w` or `--width`: Set the width of the display window.
- `--crop`: Enable cropping of the video stream.
- `--left-offset`: Set the left offset for the display window.
- `--right-offset`: Set the right offset for the display window (default is computed as `left_offset + width`).

## Launch Files

### `publish_stereo.py`
Launches two `publish_video` nodes to publish video streams from left and right cameras.

### `show_goovis.py`
Launches nodes to publish and display stereo video streams optimized for Goovis.

### `show_hrsv.py`
Launches nodes to publish and display stereo video streams optimized for HRSV.

### `local_show_goovis.py`
Launches nodes to display stereo video streams optimized for Goovis without publishing to a ROS 2 topic.

## Dependencies
- ROS 2
- OpenCV

## Installation
1. Clone the repository into your ROS 2 workspace:
   ```bash
   git clone <repository_url> ~/ros2_ws/src/dvrk_magewell
   ```
2. Build the workspace:
   ```bash
   cd ~/ros2_ws
   colcon build
   ```
3. Source the workspace:
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```
