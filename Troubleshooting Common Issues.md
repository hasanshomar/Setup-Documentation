# Possible Python script to try for CSV file export: 

```bash
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
import csv

class DepthToCSV(Node):
    def __init__(self):
        super().__init__('depth_to_csv')
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.listener_callback,
            10)
        self.csv_file = open('depth.csv', 'w')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['depth'])

    def listener_callback(self, msg):
        depth_image = np.frombuffer(msg.data, dtype=np.uint16)
        depth_image = np.reshape(depth_image, (msg.height, msg.width))
        depth_image = cv2.normalize(depth_image, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        depth_image = cv2.applyColorMap(depth_image, cv2.COLORMAP_JET)
        depth_image = cv2.cvtColor(depth_image, cv2.COLOR_BGR2RGB)
        depth_image = cv2.flip(depth_image, 1)
        depth_image = cv2.resize(depth_image, (640, 480))
        depth_image = np.reshape(depth_image, (480, 640, 3))
        depth_image = depth_image.flatten()
        self.csv_writer.writerow(depth_image)
        self.csv_file.flush()

def main(args=None):
    rclpy.init(args=args)
    node = DepthToCSV()
    rclpy.spin(node)
    node.csv_file.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

```bash
import csv
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class DepthToCSV(Node):
    def __init__(self):
        super().__init__('depth_to_csv')
        self.csv_file = open('depth.csv', 'w')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['depth'])
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.csv_writer.writerow([msg.data])
        self.csv_file.flush()

def main(args=None):
    rclpy.init(args=args)
    node = DepthToCSV()
    rclpy.spin(node)
    node.csv_file.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```



```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
import csv

class DepthToCSV(Node):
    def __init__(self):
        super().__init__('depth_to_csv')
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.listener_callback,
            10)
        self.csv_file = open('depth.csv', 'a')
        self.csv_writer = csv.writer(self.csv_file)
        if self.csv_file.tell() == 0:
            self.csv_writer.writerow(['x', 'y', 'depth'])

    def listener_callback(self, msg):
        depth_image = np.frombuffer(msg.data, dtype=np.uint16)
        depth_image = np.reshape(depth_image, (msg.height, msg.width))
        depth_image = cv2.normalize(depth_image, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        depth_image = cv2.applyColorMap(depth_image, cv2.COLORMAP_JET)
        depth_image = cv2.cvtColor(depth_image, cv2.COLOR_BGR2RGB)
        depth_image = cv2.flip(depth_image, 1)
        depth_image = cv2.resize(depth_image, (640, 480))
        depth_image = np.reshape(depth_image, (480, 640, 3))
        for i in range(480):
            for j in range(640):
                self.csv_writer.writerow([j, i, depth_image[i, j, 0]])
        self.csv_file.flush()

def main(args=None):
    rclpy.init(args=args)
    node = DepthToCSV()
    rclpy.spin(node)
    node.csv_file.close()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
