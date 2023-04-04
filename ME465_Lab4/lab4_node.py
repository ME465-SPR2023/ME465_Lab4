import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.srv import ManageLifecycleNodes
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from PIL import Image
from tqdm import tqdm


class Lab4(Node):
    def __init__(self):
        super().__init__("lab4_node")

        self.map = np.ndarray((0, 0))
        self.map_timer = self.create_timer(0.5, self.publish_map)
        self.manage_nodes = self.create_client(ManageLifecycleNodes, "/lifecycle_manager/manage_nodes")
        self.create_subscription(
            OccupancyGrid,
            "/map",
            self.map_callback,
            5,
        )
        self.create_subscription(
            PointStamped,
            "/clicked_point",
            self.goal_callback,
            5,
        )
        self.vectors = self.create_publisher(
            MarkerArray,
            "/vectors",
            5,
        )
        self.map_publisher = self.create_publisher(
            OccupancyGrid,
            "/map_decimated",
            5,
        )
        self.activate_map()

    def activate_map(self):
        if self.manage_nodes.wait_for_service(5):
            self.manage_nodes.call_async(ManageLifecycleNodes.Request(command=0))
        else:
            raise rclpy.executors.TimeoutException("/lifecycle_manager/manage_nodes service still not available after 5 seconds")

    def goal_callback(self, msg):
        self.goal = np.array([msg.point.x, msg.point.y]) - self.resolution / 2
        self.clear_vectorfield()
        self.MDP()

    def r(self, x, y):
        if np.all(np.abs((x, y) - self.goal) < self.resolution):
            return 1e15
        else:
            return -1

    def motion_model(self, distance, u):
        dt = 2
        alpha1 = 8
        std = alpha1 * u**2
        return (2 * np.pi * std**2)**(-0.5) * np.exp(-0.5 * (distance - u * dt)**2 / std**2)

    def clear_vectorfield(self):
        self.vectors.publish(MarkerArray(markers=[Marker(ns="vec", action=3)]))

    def publish_vectorfield(self, V, color="blue"):
        desired_length = 0.4
        u, v = np.gradient(V)
        length = np.linalg.norm(np.stack((u, v), axis=2), axis=2) / desired_length
        u /= length
        v /= length
        vectors = MarkerArray()
        id = 1
        for i in np.ndindex(self.X.shape):
            vector = Marker()
            vector.id = id
            id += 1
            vector.ns = "vec"
            vector.type = 0
            vector.action = 0
            vector.scale.x = 0.15
            vector.scale.y = 0.3
            vector.scale.z = 0.0
            vector.header.frame_id = "map"
            vector.points = [
                Point(x=self.X[i] + self.resolution / 2, y=self.Y[i] + self.resolution / 2),
                Point(x=self.X[i] + self.resolution / 2 + v[i], y=self.Y[i] + self.resolution / 2 + u[i]),
            ]
            vector.color.a = 1.0
            if color == "blue":
                vector.color.b = 1.0
            elif color == "green":
                vector.color.g = 1.0
            else:
                raise ValueError("unknown color {}".format(color))
            vectors.markers.append(vector)
        self.vectors.publish(vectors)

    def map_callback(self, msg):
        decimate_factor = 8
        map = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        self.map = np.array(Image.fromarray(map).resize(np.array(map.shape[::-1]) // decimate_factor)) > 0
        self.origin = np.array([msg.info.origin.position.x, msg.info.origin.position.y])
        self.resolution = decimate_factor * msg.info.resolution
        self.process_map()
        x = self.origin[0] + np.arange(0, self.width * self.resolution, self.resolution)
        y = self.origin[1] + np.arange(0, self.height * self.resolution, self.resolution)
        self.X, self.Y = np.meshgrid(x, y)

    def process_map(self):
        for i in range(self.width):
            ind = np.argwhere(self.map[:,i])
            self.map[:ind.min(),i] = True
            self.map[ind.max():,i] = True
        old_width = self.width
        old_height = self.height
        old_map = self.map
        border_size = 4
        self.map = np.ones(np.array(old_map.shape) + 2 * border_size, dtype=bool)
        self.map[border_size:border_size + old_height, border_size:border_size + old_width] = old_map
        self.origin -= border_size * self.resolution

    def publish_map(self):
        if self.map.size:
            map = OccupancyGrid()
            map.header.frame_id = "map"
            map.info.resolution = self.resolution
            map.info.width = self.width
            map.info.height = self.height
            map.info.origin.position.x = self.origin[0]
            map.info.origin.position.y = self.origin[1]
            map.data = (100 * self.map.astype(int).flatten()).tolist()
            self.map_publisher.publish(map)

    @property
    def width(self):
        return self.map.shape[1]

    @property
    def height(self):
        return self.map.shape[0]


def main(args=None):
    rclpy.init(args=args)
    node = Lab4()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
