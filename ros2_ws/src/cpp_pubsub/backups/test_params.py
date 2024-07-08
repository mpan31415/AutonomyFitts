#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class TestParams(Node):

    def __init__(self):

        super().__init__('test_params_rclpy')

        # parameter stuff
        self.param_names = ['my_str', 'my_int', 'my_double_array']
        self.declare_parameters(
            namespace='',
            parameters=[
                (self.param_names[0], rclpy.Parameter.Type.STRING),
                (self.param_names[1], rclpy.Parameter.Type.INTEGER),
                (self.param_names[2], rclpy.Parameter.Type.DOUBLE_ARRAY)
            ]
        )
        (param_str, param_int, param_double_array) = self.get_parameters(self.param_names)
        self.p_str = param_str.value
        self.p_int = param_int.value
        self.p_double_array = param_double_array.value

        self.timer_callback()

        # # publisher
        # self.publisher_ = self.create_publisher(String, '/test_param_rclpy', 10)
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):
        print("\n" * 10)
        print("=" * 100)

        print("\n\nThe current parameters are as follows:\n")
        print("The participant_id = %s\n\n" % self.p_str)
        print("The trajectory_id = %d\n\n" % self.p_int)
        print("The autonomy_id = %s\n\n" % self.p_double_array)

        print("=" * 100)
        print("\n" * 10)
        




# The following is just to start the node
def main(args=None):

    rclpy.init(args=args)
    node = TestParams()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()