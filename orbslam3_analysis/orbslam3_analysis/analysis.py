import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Import the desired message type
from orbslam3_msgs.msg import OrbFeatures, KeyPoint, Descriptors
from sensor_msgs.msg import Image
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import cv2
from dataclasses import dataclass
from typing import List

from pathlib import Path
import argparse


@dataclass
class ImageWithFeatures:
    img: np.ndarray
    keypoints: KeyPoint
    descriptors: Descriptors


SAMPLE_RATE = 1 # Hz
MAX_SAMPLES = 10
IMAGES_WITH_FEATURES = []




def format_topic_path(*args) -> str:
    if len(args) == 0:
        # raise
        pass
    elif len(args) == 1:
        return args[0]
    else:
        return "/".join(args)

def pascal_to_snake(pascal_str: str) -> str:
    snake_str = ""
    for i, c in enumerate(pascal_str):
        if c.isupper() and i > 0:
            snake_str += "_"
        snake_str += c.lower()
    return snake_str




class AnalysisNode(Node):

    def __init__(self):
        self.node_name = pascal_to_snake(self.__class__.__name__)
        super().__init__(node_name=self.node_name)
        self.get_logger().info(f"Initialize {self.node_name}")


        self.orb_features_subscription = self.create_subscription(
            msg_type=OrbFeatures,
            topic=format_topic_path("orbslam3", "orb_features"),
            callback=self.orb_features_callback,
            qos_profile=10
        )

        self.image_subscription = self.create_subscription(
            msg_type=Image,
            topic=format_topic_path("camera", "left"),
            callback=self.image_callback,
            qos_profile=10
        )

        self.latest_image = None
        self.latest_features = None


        # self.fig = plt.figure()
        # self.ax1 = self.fig.add_subplot(1, 1, 1)


        # self.animation_timer = self.create_timer(
        #     timer_period_sec=1,
        #     callback=self.update_plot
        # )

        self.sample_timer = self.create_timer(
            timer_period_sec=1/SAMPLE_RATE,
            callback=self.sample_callback
        )

    def sample_callback(self) -> None:
        if self.latest_features is None or self.latest_image is None:
            return
        IMAGES_WITH_FEATURES.append(ImageWithFeatures(
            img=self.latest_image,
            keypoints=self.latest_features['keypoints'],
            descriptors=self.latest_features['descriptors'],
        ))
        self.latest_image = None
        self.latest_features = None
        self.get_logger().info(f"Collected {len(IMAGES_WITH_FEATURES)}/{MAX_SAMPLES} samples")
        if len(IMAGES_WITH_FEATURES) > MAX_SAMPLES:
            raise SystemExit

    def orb_features_callback(self, msg: OrbFeatures) -> None:
        keypoints: KeyPoint = msg.keypoints
        descriptors: Descriptors = msg.descriptors
        r: int = descriptors.rows
        c: int = descriptors.columns

        # Convert the descriptors to a numpy array
        descriptors = np.array(descriptors.data)
        try:
            descriptors = descriptors.reshape(r, c)
        except ValueError:
            return
        self.latest_features = {
            "keypoints": keypoints,
            "descriptors": descriptors
        }
        # self.get_logger().info(f"Received a message: #keypoints = {len(keypoints)}, #descriptors = ({r}, {c})")


    def image_callback(self, msg: Image) -> None:
        h = msg.height
        w = msg.width
        c = msg.step // w
        self.latest_image = np.array(msg.data).reshape(h, w, c).squeeze()
        # self.get_logger().info(f"Received a message: {h} x {w} x {c}")



    # def analyze_callback(self, msg):
    #     # Extract features from the msg
    #     data = self.extract_features(msg)

    #     # Update the plot with new data
    #     self.update_plot(data)

    # def extract_features(self, msg):
    #     # Your feature extraction logic here
    #     pass

    # def update_plot(self):
    #     def animate(i):
    #         # self.ax1.clear()
    #         # Draw the features on the image
    #         if self.latest_image is not None:
    #             return [self.ax1.imshow(self.latest_image, cmap='gray')]
    #             # self.ax1.imshow(self.img)
    #         # self.ax1.imshow(data)

    #     ani = animation.FuncAnimation(self.fig, animate, interval=1000)
    #     plt.show()

def main(args=None):
    prog = os.path.basename(__file__).replace('.py', '')
    parser = argparse.ArgumentParser(prog=prog)
    parser.add_argument('-r', '--rate', type=float, default=1.0, help='Sample rate in Hz')
    parser.add_argument('-m', '--max-samples', type=int, default=10, help='Maximum number of samples to collect')
    parser.add_argument('-o', '--output', type=str, default='orb_features.mp4', help='Output file name')

    rclpy.init(args=args)

    args = parser.parse_args()
    SAMPLE_RATE = args.rate
    MAX_SAMPLES = args.max_samples



    node = AnalysisNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        # Create an animation sequence of the features

        fig, ax = plt.subplots()

        def update(frame_number: int):
            # Clear the current plot
            ax.clear()
            im = IMAGES_WITH_FEATURES[frame_number].img
            ax.imshow(im, cmap='gray')

            # Draw the features on the image
            keypoints = IMAGES_WITH_FEATURES[frame_number].keypoints
            for kp in keypoints:
                x = kp.x
                y = kp.y
                ax.scatter(x, y, s=1, c='r', marker='o')
            height, width = im.shape
            ax.set_ylim(height, 0)
            ax.set_xlim(0, width)
            ax.set_title(f"Frame {frame_number}")
            ax.grid(False)


        ani = animation.FuncAnimation(fig, update, frames=range(len(IMAGES_WITH_FEATURES)), interval=1000)
        output_file = Path(args.output)
        # ani.save(output_file)
        plt.show()


    node.destroy_node()
    print("Shutting down...")

    rclpy.shutdown()

if __name__ == '__main__':
    main()
