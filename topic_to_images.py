import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.subscription1 = self.create_subscription(
            Image,
            '/oak/right/image_raw',
            self.image_callback1,
            10)
        self.subscription2 = self.create_subscription(
            Image,
            '/oak/left/image_raw',
            self.image_callback2,
            10)
        
        self.bridge = CvBridge()

        self.image_count1 = 0
        self.image_count2 = 0
        self.max_images = 500  # Change this to the number of images you want to save
        self.image_dir_left = 'images/left'  # Directory to save images
        self.image_dir_right = 'images/right'  # Directory to save images
        os.makedirs(self.image_dir_left, exist_ok=True)
        os.makedirs(self.image_dir_right, exist_ok=True)

    def image_callback1(self, msg):
        if self.image_count1 < self.max_images:
            if self.image_count1 % 100 == 0: self.save_image(msg, 'image_topic1')
            self.image_count1 += 1
            if self.image_count1 >= self.max_images:
                self.get_logger().info('Saved %d images from image_topic1' % self.max_images)
                self.subscription1.destroy()

    def image_callback2(self, msg):
        if self.image_count2 < self.max_images:
            if self.image_count2 % 100 == 0: self.save_image(msg, 'image_topic2')
            self.image_count2 += 1
            if self.image_count2 >= self.max_images:
                self.get_logger().info('Saved %d images from image_topic2' % self.max_images)
                self.subscription2.destroy()

    def save_image(self, msg, topic):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            filename = f'image_{str(int((self.image_count1 if topic == "image_topic1" else self.image_count2)/100))}.jpg'
            cv2.imwrite(os.path.join((self.image_dir_right if topic == "image_topic1" else self.image_dir_left), filename), cv_image)
        except Exception as e:
            self.get_logger().error(f'Error saving image: {e}')

def main(args=None):
    rclpy.init(args=args)
    image_saver = ImageSaver()
    rclpy.spin(image_saver)
    image_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
