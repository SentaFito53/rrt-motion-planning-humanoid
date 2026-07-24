import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Float32MultiArray
import cv2
import numpy as np

# ==========================================================
# UBAH ANGKA INI sesuai robot yang mau dimonitor.
# Kalau dijalankan langsung dengan "python3 rrt_map.py" tanpa
# argumen --ros-args, nilai INI yang dipakai.
# Isi 0 kalau node target TIDAK memakai namespace (topic polos: /pose).
# ==========================================================
ROBOT_ID = 2


class RRTMonitor(Node):
    def __init__(self):
        super().__init__('rrt_monitor')

        # Parameter robot_id: default dari ROBOT_ID di atas file.
        # Bisa dioverride lewat: --ros-args -p robot_id:=<angka>
        self.declare_parameter('robot_id', ROBOT_ID)
        robot_id = self.get_parameter('robot_id').value
        prefix = f'/robot_{robot_id}' if robot_id > 0 else ''

        self.robotPosX = 0.0
        self.robotPosY = 0.0
        self.rrtPath = []      # list of (x, y) dalam cm
        self.obstacles = []    # list of (x, y, radius) dalam cm

        pose_topic = f'{prefix}/pose'
        path_topic = f'{prefix}/rrt_path'
        obstacle_topic = f'{prefix}/rrt_obstacles'
        self.get_logger().info(
            f'Subscribing ke: {pose_topic}, {path_topic}, {obstacle_topic}')

        self.create_subscription(Odometry, pose_topic, self.pose_callback, 1)
        self.create_subscription(Path, path_topic, self.path_callback, 1)
        self.create_subscription(Float32MultiArray, obstacle_topic, self.obstacle_callback, 1)
        self.mapImage = np.zeros((800, 1100, 3), np.uint8)
        self.create_timer(0.1, self.draw_callback)  # 10 fps cukup untuk keperluan monitoring

    def pose_callback(self, msg):
        self.robotPosX = msg.pose.pose.position.x
        self.robotPosY = msg.pose.pose.position.y
        print(f"[POSE] robotPosX = {self.robotPosX:.2f}, robotPosY = {self.robotPosY:.2f}")

    def path_callback(self, msg):
        self.rrtPath = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        print(f"[PATH] diterima {len(self.rrtPath)} waypoint: {self.rrtPath}")

    def obstacle_callback(self, msg):
        data = msg.data
        self.obstacles = [(data[i], data[i + 1], data[i + 2]) for i in range(0, len(data), 3)]
        print(f"[OBSTACLE] diterima {len(self.obstacles)} obstacle: {self.obstacles}")

    def world_to_image(self, x, y):
        # skema sama seperti node 'map': geser +450 (X) dan +300 (Y) agar pusat lapangan di tengah gambar
        ix = int(x + 450 + 100)
        iy = int(y + 300 + 100)
        return ix, iy

    def draw_callback(self):
        img = self.mapImage
        img[:] = (0, 150, 0)

        # --- Garis lapangan (disederhanakan dari node 'map' yang sudah ada) ---
        cv2.rectangle(img, (100, 100), (1000, 700), (255, 255, 255), 2)   # garis luar
        cv2.line(img, (550, 100), (550, 700), (255, 255, 255), 2)         # garis tengah
        cv2.circle(img, (550, 400), 75, (255, 255, 255), 2)               # lingkaran tengah
        cv2.rectangle(img, (100, 270), (160, 530), (255, 255, 255), 2)    # gawang kiri
        cv2.rectangle(img, (940, 270), (1000, 530), (255, 255, 255), 2)   # gawang kanan

        # --- Obstacle: lingkaran merah = radius aman ---
        for (ox, oy, r) in self.obstacles:
            cx, cy = self.world_to_image(ox, oy)
            cv2.circle(img, (cx, cy), int(r), (0, 0, 255), 2)
            cv2.circle(img, (cx, cy), 4, (0, 0, 255), -1)

        # --- Jalur RRT: garis kuning menghubungkan tiap waypoint ---
        for i in range(len(self.rrtPath) - 1):
            p1 = self.world_to_image(*self.rrtPath[i])
            p2 = self.world_to_image(*self.rrtPath[i + 1])
            cv2.line(img, p1, p2, (0, 255, 255), 2)
        for wp in self.rrtPath:
            wx, wy = self.world_to_image(*wp)
            cv2.circle(img, (wx, wy), 4, (0, 255, 255), -1)

        # --- Posisi robot: titik oranye ---
        rx, ry = self.world_to_image(self.robotPosX, self.robotPosY)
        cv2.circle(img, (rx, ry), 8, (255, 128, 0), -1)

        smallImg = cv2.resize(img, (640, 480), interpolation=cv2.INTER_AREA)
        cv2.imshow("RRT Monitor - BarelangFC", smallImg)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = RRTMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
