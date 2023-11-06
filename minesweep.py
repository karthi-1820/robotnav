import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import time
import cv2
import numpy as np

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Twist, 'blinky/cmd_vel', 10)
        self.flags = {
            '1st_green_ball_detected': False,
            '1st_green_ball_touched': False,
            '2nd_green_ball_detected': False,
            '2nd_green_ball_touched': False,
            'blue_ball_detected': False,
            'blue_ball_touched': False,
            'ball aligned': False,
        }
        self.search_for_2nd_green_ball = False
        self.image_processing_active = True
        self.state = "Forward"
        

    def calculate_distance(self, object_width_in_pixels):
        KNOWN_WIDTH = 0.1  # Actual width of the object (in meters)
        FOCAL_LENGTH = 400  # Focal length of the camera (in pixels)
        distance = (KNOWN_WIDTH * FOCAL_LENGTH) / object_width_in_pixels
        return distance

    def detect_objects(self, cv2_img, color):
        if not self.image_processing_active:
            return
        self.state = "search"
        # Define Region of Interest (ROI)
        y_max, x_max, _ = cv2_img.shape
        roi = cv2_img[60:y_max, 60:x_max]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        if color == "green":
            lower = np.array([25, 80, 4])
            upper = np.array([64, 255, 255])
            color_bgr = (0, 255, 0)
        elif color == "blue":
            lower = np.array([100, 150, 0])
            upper = np.array([140, 255, 255])
            color_bgr = (255, 0, 0)
        else:
            self.get_logger().error(f"Unsupported color: {color}")
            return

        mask = cv2.inRange(hsv, lower, upper)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        center_x = None
        bottom_threshold = 20
        for contour in contours:
            if cv2.contourArea(contour) > 100:
                x, y, w, h = cv2.boundingRect(contour)
                center_x = x + w // 2
                center_y = y + h // 2
                distance = self.calculate_distance(w)
                cv2.rectangle(roi, (x, y), (x+w, y+h), color_bgr, 2)
               
                if color == "blue":
                    kernel = np.ones((5, 5), np.uint8)
                    mask = cv2.erode(mask, kernel, iterations=1)
                    area = cv2.contourArea(contour)
                    perimeter = cv2.arcLength(contour, True)
                    circularity = 4 * np.pi * (area / (perimeter ** 2)) if perimeter > 0 else 0
                    if 0.7 <= circularity <= 1.2:
                        self.flags['blue_ball_detected'] = True
                        self.get_logger().info(f"Blue ball detected at {center_x}, distance: {distance:.2f} m")
                        cv2.rectangle(roi, (x-2, y-2), (x+w+2, y+h+2), color_bgr, 2)
                else:
                    if not self.flags['1st_green_ball_touched']:
                        self.flags['1st_green_ball_detected'] = True
                        self.get_logger().info(f"1st Green ball detected at {center_x}, distance: {distance:.2f} m")
                    elif self.search_for_2nd_green_ball and not self.flags['2nd_green_ball_touched']:
                        self.flags['2nd_green_ball_detected'] = True
                        self.get_logger().info(f"2nd Green ball detected at {center_x}, distance: {distance:.2f} m")
                    cv2.rectangle(roi, (x, y), (x+w, y+h), color_bgr, 2)
        

        if color == "green":
            if len(contours) == 0:
                if not self.flags['1st_green_ball_touched'] or (self.search_for_2nd_green_ball and not self.flags['2nd_green_ball_touched']):

                   # Check if the center of the ball is within the threshold from the bottom of the ROI
                    if center_y > (y_max - bottom_threshold):
                        if not self.flags['1st_green_ball_touched']:
                            self.flags['1st_green_ball_detected'] = True
                            self.get_logger().info("1st Green ball detected as touched (center close to bottom frame)")
                            self.flags['1st_green_ball_touched'] = True
                            self.search_for_2nd_green_ball = True
                        elif self.search_for_2nd_green_ball and not self.flags['2nd_green_ball_touched']:
                            self.flags['2nd_green_ball_detected'] = True
                            self.get_logger().info("2nd Green ball detected as touched (center close to bottom frame)")
                            self.flags['2nd_green_ball_touched'] = True

        elif color == "blue" and self.flags['blue_ball_detected'] and not self.flags['blue_ball_touched']:
            if center_y > (y_max - bottom_threshold):
                self.get_logger().info("Blue ball detected as touched (center close to bottom frame)")
                self.flags['blue_ball_touched'] = True
                self.image_processing_active = False 
        
        cv2.imshow("roi",roi)
        return center_x
    
    def ball_tracking(self, center_x):
        if center_x is None:
            return
        current_position = center_x
        desired_position = 264//2
        error = center_x - desired_position
        max_error_threshold = 20
        min_error_threshold = -20
        print('Current Position', current_position)
        print('Error:', error)

        clockwise_rotation_speed = -0.1  
        counterclockwise_rotation_speed = 0.1

        if error < min_error_threshold:
            self.rotate_robot(counterclockwise_rotation_speed)
        elif abs(error) > max_error_threshold:
            self.rotate_robot(clockwise_rotation_speed)
        elif min_error_threshold<abs(error)<max_error_threshold:
            # self.stop_robot()
            self.flags['ball aligned'] = True
            self.move_to_ball()




    def image_callback(self, msg):
        if self.image_processing_active:
        
            cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            if not self.flags['1st_green_ball_touched']:
                center_x = self.detect_objects(cv2_img, "green")
                if self.flags['1st_green_ball_detected']:
                    self.ball_tracking(center_x)
            elif self.search_for_2nd_green_ball and not self.flags['2nd_green_ball_touched']:
                center_x = self.detect_objects(cv2_img, "green")
                if self.flags['2nd_green_ball_detected']:
                    self.ball_tracking(center_x)
            elif not self.flags['blue_ball_touched']:
                self.detect_objects(cv2_img, "blue")

            cv2.imshow("Image", cv2_img)
            cv2.waitKey(1)


    def rotate_robot(self, angular_z):
        #print('Turn')
        msg = Twist()
        msg.angular.z = angular_z
        self.publisher.publish(msg)

    def stop_robot(self):
        print('Stop')
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.publisher.publish(twist_msg)

    def forward_robot(self):
        print('Forward')
        twist_msg = Twist()
        twist_msg.linear.x = 0.1 
        self.publisher.publish(twist_msg)

    def move_to_ball(self):
        print('Forward')
        twist_msg = Twist()
        twist_msg.linear.x = 0.1 
        self.publisher.publish(twist_msg)


    # def execute_state(self):
    #     if self.flags['1st_green_ball_detected'] and not self.flags['1st_green_ball_touched']:
    #         #  if self.flags['ball aligned'] == True:
    #         #     self.move_to_ball()
    #         #     print("moving towards ball")
    #         # self.state = "Ball Detected"
    #         # self.stop_robot()
    #         print("1")
       
    #     else:
    #         self.forward_robot()
        
        
def main(args=None):
    rclpy.init(args=args)
    image_processor = ImageProcessor()
    rclpy.spin(image_processor)
    image_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




