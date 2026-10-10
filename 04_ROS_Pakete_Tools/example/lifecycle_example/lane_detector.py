import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from std_msgs.msg import String


class LaneDetector(LifecycleNode):

    def __init__(self):
        super().__init__('lane_detector')

    def on_configure(self, state):
        # Ressourcen anlegen: Publisher, Parameter lesen, Kamera öffnen ...
        self.pub = self.create_lifecycle_publisher(String, 'lane', 10)
        self.timer = self.create_timer(1.0, self.detect)
        self.get_logger().info('konfiguriert')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info('aktiv — sende Ergebnisse')
        return super().on_activate(state)      # aktiviert den Lifecycle-Publisher

    def on_deactivate(self, state):
        self.get_logger().info('inaktiv — pausiert')
        return super().on_deactivate(state)

    def on_cleanup(self, state):
        self.destroy_timer(self.timer)
        self.destroy_lifecycle_publisher(self.pub)
        return TransitionCallbackReturn.SUCCESS

    def detect(self):
        if self.pub.is_activated:
            self.pub.publish(String(data='Spur erkannt'))


def main():
    rclpy.init()
    node = LaneDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
