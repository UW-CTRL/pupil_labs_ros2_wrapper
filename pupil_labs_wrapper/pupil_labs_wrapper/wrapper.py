#!/home/kysh/venv/pupil_labs/bin/python

# import sys
# print(sys.version)

import rclpy
# import pupil_labs
from rclpy.node import Node
from pupil_labs.realtime_api.simple import discover_one_device
from egocentric_msg.msg import GazeData
from sensor_msgs.msg import Image

def populate_image_message(pl_image_msg, timestamp):
    ros_img = Image()
    ros_img.header.stamp.sec = timestamp.sec
    ros_img.header.stamp.nanosec = timestamp.nanosec
    ros_img.height = pl_image_msg.bgr_pixels.shape[0]
    ros_img.width = pl_image_msg.bgr_pixels.shape[1]
    ros_img.data = pl_image_msg.bgr_pixels.tobytes()
    # Set the encoding (e.g., "bgr8" for OpenCV BGR images)
    ros_img.encoding = "bgr8"  # or "rgb8" depending on your image format
    return ros_img

def populate_sensor_message(pl_gaze_msg, timestamp):
    msg = GazeData()
    # msg.header.stamp.sec = timestamp.sec
    # msg.header.stamp.nanosec = timestamp.nanosec
    msg.x = pl_gaze_msg.x
    msg.y = pl_gaze_msg.y
    msg.worn = pl_gaze_msg.worn
    msg.pupil_diameter_left = pl_gaze_msg.pupil_diameter_left
    msg.eyeball_center_left_x = pl_gaze_msg.eyeball_center_left_x
    msg.eyeball_center_left_y = pl_gaze_msg.eyeball_center_left_y
    msg.eyeball_center_left_z = pl_gaze_msg.eyeball_center_left_z
    msg.optical_axis_left_x = pl_gaze_msg.optical_axis_left_x
    msg.optical_axis_left_y = pl_gaze_msg.optical_axis_left_y
    msg.optical_axis_left_z = pl_gaze_msg.optical_axis_left_z
    msg.pupil_diameter_right = pl_gaze_msg.pupil_diameter_right
    msg.eyeball_center_right_x = pl_gaze_msg.eyeball_center_right_x
    msg.eyeball_center_right_y = pl_gaze_msg.eyeball_center_right_y
    msg.eyeball_center_right_z = pl_gaze_msg.eyeball_center_right_z
    msg.optical_axis_right_x = pl_gaze_msg.optical_axis_right_x
    msg.optical_axis_right_y = pl_gaze_msg.optical_axis_right_y
    msg.optical_axis_right_z = pl_gaze_msg.optical_axis_right_z
    msg.timestamp_unix_seconds = pl_gaze_msg.timestamp_unix_seconds
    # print(msg)
    return msg

class PupilLabsWrapper(Node):
    def __init__(self):
        super().__init__('pupil_labs_wrapper')
        self.get_logger().info("Looking for the next best device...")
        self.device = discover_one_device(max_search_duration_seconds=10)
        if self.device is None:
            self.get_logger().error("No device found.")
            raise SystemExit(-1)
        
        self.get_logger().info(f"Connecting to {self.device}...")
        
        # Setup publishers
        self.pub_gaze = self.create_publisher(GazeData, 'pupil_labs/gaze', 10)
        self.pub_rgb = self.create_publisher(Image, 'pupil_labs/scene_img', 10)
        self.pub_eyes = self.create_publisher(Image, 'pupil_labs/eye_img', 10)
        
        self.timer = self.create_timer(1.0 / 30.0, self.publish_pupil_labs_data)

    def publish_pupil_labs_data(self):
        try:
            # Get data from device
            pupil_labs_msg = self.device.receive_matched_scene_and_eyes_video_frames_and_gaze()
            # Get current time for timestamp
            current_time = self.get_clock().now().to_msg()
            # Populate and publish messages
            self.pub_gaze.publish(populate_sensor_message(pupil_labs_msg.gaze, current_time))
            self.pub_rgb.publish(populate_image_message(pupil_labs_msg.scene, current_time))
            self.pub_eyes.publish(populate_image_message(pupil_labs_msg.eyes, current_time))
        except Exception as e:
            self.get_logger().error(f"Error receiving or publishing data: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PupilLabsWrapper()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


