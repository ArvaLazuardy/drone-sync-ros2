import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32
from std_srvs.srv import Empty



class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Int32,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.palindromeCounter = 0
        self.detectedCounter = 0

        self.cli = self.create_client(Empty, 'land_drone')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Empty.Request()

    def listener_callback(self, msg):
        if self.checkPalindorme(msg):
            self.palindromeCounter += 1

            if self.palindromeCounter % 67 == 0:
                self.get_logger().info('MAS NF DETECTED: "%s"' % msg.data)
                self.detectedCounter += 1

                if self.detectedCounter == 10:
                    future = self.cli.call_async(self.req)
                    rclpy.shutdown()
                    return

            else:
                self.get_logger().info('PALINDROME recieved: "%s"' % msg.data)
            
        else:
            self.get_logger().info('detection received : "%s"' % msg.data)


    def checkPalindorme(self, msg):
        msg_string = str(msg.data)
        start = 0
        end = len(msg_string) - 1
        
        while start < end:
            if msg_string[start] != msg_string[end]:
                return False
            start += 1
            end -= 1 
        return True

        #or just use msg_string = msg_string[::-1]
    


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
