import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher1_ = self.create_publisher(String, 'xinx', 10)
    #    self.publisher2_ = self.create_publisher(String, 'sss', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i1 = 0
    #    self.i2 = 1000

    def timer_callback(self):
        msg1 = String()
        msg1.data = 'Hello World: %d' % self.i1
     #   msg2 = String()
      #  msg2.data = 'xinx, hi: %d' % self.i2
        self.publisher1_.publish(msg1)
 #       self.publisher2_.publish(msg2)
        self.get_logger().info('Publishing: "%s"' % msg1.data)
  #      self.get_logger().info('Publishing: "%s"' % msg2.data)
        self.i1 += 1
   #     self.i2 -= 1  # 修复：缩进与上一行对齐

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
