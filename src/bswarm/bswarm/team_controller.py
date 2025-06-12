#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry
from px4_msgs.msg import VehicleCommand as VehicleCommandMsg
from px4_msgs.srv import VehicleCommand as VehicleCommandSrv
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from boids import Boid

COUNT = 25

class MultiNamespaceCommandNode(Node):
    def __init__(self, namespaces):
        super().__init__('bswarm')

        self.namespaces = namespaces
        self._pub_map = {}
        self._cli_map = {}
        self._pos_map = {}

        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        for ns in namespaces:
            # create publishers and services
            topic = f'{ns}/fmu/in/vehicle_command'
            svc   = f'{ns}/fmu/vehicle_command'

            self._pub_map[ns] = self.create_publisher(
                VehicleCommandMsg, topic, 10
            )
            self._cli_map[ns] = self.create_client(
                VehicleCommandSrv, svc
            )

            # odo subscriber
            odom_topic = f'{ns}/fmu/out/vehicle_odometry'
            self.create_subscription(
                VehicleOdometry,
                odom_topic,
                lambda msg, ns=ns: self._pos_map.__setitem__(ns, msg),
                odom_qos
            )

            # log
            self.get_logger().info(f'Ready for {ns}: service {svc}, topic {topic}, subscribed to {odom_topic}')

    def call_service(self, namespace, request_params: dict):
        client = self._cli_map[namespace]
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"[{namespace}] service not available")
            return False

        req = VehicleCommandSrv.Request()
        for key, value in request_params.items():
            setattr(req.request, key, value)

        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            res = future.result()
            self.get_logger().info(f"[{namespace}] service response: {res}")
            return res
        else:
            self.get_logger().error(f"[{namespace}] service call failed")
            return None

    def get_position(self, namespace, timeout=5.0):
        start = time.time()
        while self._pos_map[namespace] is None and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        odom = self._pos_map[namespace]
        if odom is None:
            self.get_logger().error(f"[{namespace}] no odometry received within timeout")
            return None, None
        return odom.x, odom.y

    @staticmethod
    def compute_waypoint(self, namespace, x, y):
        # get positions of all boids
        for ns in self.namespaces:
            all_boids.append( Boid(self.get_position(ns)) )
            if namespace == ns:
                b = Boid(self.get_position(ns))

        if b is not None:
            b.update(all_boids, [])
            return b.position


def main():
    count = COUNT
    rclpy.init()

    namespaces = [f'/px4_{i}' for i in range(1, count + 1)]
    node = MultiNamespaceCommandNode(namespaces)

    ## Start sequence
    for idx, ns in enumerate(namespaces, start=1):
        # offboard mode
        params1 = {
            'timestamp': int(time.time()),
            'command': 176,
            'param1': 1.0,
            'param2': 6.0,
            'target_system': idx,
            'target_component': 1,
            'from_external': True
        }
        node.call_service(ns, params1)

        # arm 
        params2 = {
            'timestamp': int(time.time()),
            'command': 400,
            'param1': 1.0,
            'target_system': idx,
            'target_component': 1,
            'from_external': True
        }
        node.call_service(ns, params2)

        # takeoff
        params3 = {
            'timestamp': int(time.time()),
            'command': 22,
            'param7': 5.0,
            'target_system': idx,
            'target_component': 1,
            'from_external': True
        }
        node.call_service(ns, params3)

        node.get_logger().info(f"[{ns}]: READY")

    ## Pause
    time.sleep(10)

    ## Control team boids
    for idx, ns in enumerate(namespaces, start=1):
        # get current position
        x, y = node.get_position(ns)
        if x is None:
            continue

        # compute new waypoint
        new_x, new_y = node.compute_waypoint(x, y)
        node.get_logger().info(f"[{ns}] new waypoint: ({new_x}, {new_y})")

        # move to new waypoint
        params4 = {
            'timestamp': int(time.time()),
            'command': 16,
            'param5': new_x,
            'param6': new_y,
            'target_system': 1,
            'target_component': 1,
            'from_external': True
        }
        node.call_service(ns, params4)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
