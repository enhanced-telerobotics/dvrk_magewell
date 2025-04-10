import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import time
import argparse  # Add argparse for CLI commands

# Initialize CvBridge
bridge = CvBridge()

# Define global variables to store the images
right_image = None
left_image = None

# Callback function to process the right image message
def right_image_callback(msg):
    global right_image
    try:
        # Convert the ROS 2 Image message to an OpenCV image
        right_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(f"Failed to convert right image: {e}")

# Callback function to process the left image message
def left_image_callback(msg):
    global left_image
    try:
        # Convert the ROS 2 Image message to an OpenCV image
        left_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(f"Failed to convert left image: {e}")

# Function to resize image while maintaining aspect ratio to both height and width constraints
def resize_image(image, target_height, target_width):
    h, w = image.shape[:2]
    scaling_factor = max(target_width / float(w), target_height / float(h))
    new_width = int(w * scaling_factor)
    new_height = int(h * scaling_factor)
    return cv2.resize(image, (new_width, new_height))

# Function to center-crop the image to the target width
def center_crop(image, target_width):
    h, w = image.shape[:2]
    start_x = (w - target_width) // 2
    if start_x < 0:
        start_x = 0
    return image[:, start_x:start_x + target_width]

# Function to display images in two separate windows
def display_images(node, display_mode):
    global right_image, left_image

    # Target height and width for resized images
    target_height = 768
    target_width = 1024

    # Create windows based on display mode
    if display_mode in ['left', 'stereo']:
        cv2.namedWindow("Left Image", cv2.WINDOW_NORMAL)
    if display_mode in ['right', 'stereo']:
        cv2.namedWindow("Right Image", cv2.WINDOW_NORMAL)

    # Initialize the FPS calculation
    prev_time = time.time()

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)

        if right_image is not None and left_image is not None:
            # Resize both images to the target height and width
            left_image_resized = resize_image(left_image, target_height, target_width)
            right_image_resized = resize_image(right_image, target_height, target_width)

            # Center crop the images to the target width
            left_image_cropped = center_crop(left_image_resized, target_width)
            right_image_cropped = center_crop(right_image_resized, target_width)

            # Calculate FPS
            curr_time = time.time()
            fps = 1 / (curr_time - prev_time)
            prev_time = curr_time

            # Overlay FPS on both images
            if display_mode in ['left', 'stereo']:
                cv2.putText(left_image_cropped, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            if display_mode in ['right', 'stereo']:
                cv2.putText(right_image_cropped, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # Display the images based on display mode
            if display_mode in ['left', 'stereo']:
                cv2.imshow("Left Image", left_image_cropped)
            if display_mode in ['right', 'stereo']:
                cv2.imshow("Right Image", right_image_cropped)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    cv2.destroyAllWindows()

def main(args=None):
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Display left, right, or stereo images.')
    parser.add_argument('--display', type=str, choices=['left', 'right', 'stereo'], default='stereo',
                        help='Select which images to display: left, right, or stereo')
    cli_args = parser.parse_args()

    # Initialize rclpy
    rclpy.init(args=args)

    # Create a node
    node = rclpy.create_node('ros2_continuous_image_node')

    # Subscribe to the right image topic
    node.create_subscription(Image, '/davinci_endo/right/image_raw', right_image_callback, 10)

    # Subscribe to the left image topic
    node.create_subscription(Image, '/davinci_endo/left/image_raw', left_image_callback, 10)

    # Run the display_images function
    try:
        display_images(node, cli_args.display)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly (optional)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
