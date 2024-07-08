import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

import matplotlib.pyplot as plt


class IkSubscriber(Node):

    def __init__(self):

        super().__init__('ik_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            'tau_d_calculated',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.max_len = 1000
        self.len = 0
        self.joint_vals = []
        self.plotted = False


    def listener_callback(self, msg):

        joint_vals = msg.position

        if self.len < self.max_len:
            print(joint_vals)
            self.joint_vals.append(joint_vals[1])
            self.len += 1
        else:
            print("finished recording!")
            if not self.plotted:
                self.plot_result()
                self.plotted = True
            

    def plot_result(self):
        
        x = [i for i in range(1, self.max_len+1)]
        y = self.joint_vals
        plt.plot(x, y)
        plt.show()
        


def main(args=None):

    rclpy.init(args=args)

    ik_sub = IkSubscriber()

    rclpy.spin(ik_sub)

    ik_sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':

    main()