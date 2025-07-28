import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import os
import time

class TerminalNavigation(Node):
    def __init__(self):
        super().__init__('terminal_navigation')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def run(self):
        while True:
            self.clear_screen()
            print("Enter target coordinates and orientation (x, y, yaw).")
            print("Enter 'exit' to quit.")
            
            try:
                user_input = input("Input (x, y, yaw): ").strip()
                if user_input.lower() == 'exit':
                    print("Exiting...")
                    break

                x, y, yaw = map(float, user_input.split(','))
                orientation = self.yaw_to_quaternion(yaw)
                self.navigate_to_pose(x, y, orientation)

            except ValueError:
                print("Invalid input format. Please use the format: x, y, yaw (e.g., 1.5, 2.3, 0.78).")
                time.sleep(2)

    def navigate_to_pose(self, x, y, orientation):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Set target position
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        # Set target orientation (quaternion)
        goal_msg.pose.pose.orientation.x = orientation[0]
        goal_msg.pose.pose.orientation.y = orientation[1]
        goal_msg.pose.pose.orientation.z = orientation[2]
        goal_msg.pose.pose.orientation.w = orientation[3]

        print(f"Navigating to: x={x}, y={y}, orientation={orientation}")
        self._action_client.wait_for_server()

        # Send the goal and wait for result
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            print('Goal rejected.')
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        if result.status == 3:  # SUCCEEDED
            print("Navigation succeeded!")
        else:
            print(f"Navigation failed with status: {result.status}")

    def yaw_to_quaternion(self, yaw):
        """
        Convert yaw angle (in radians) to quaternion (x, y, z, w).
        """
        import math
        qx = 0.0
        qy = 0.0
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return (qx, qy, qz, qw)

    def clear_screen(self):
        os.system('cls' if os.name == 'nt' else 'clear')

def main(args=None):
    rclpy.init(args=args)
    navigation_node = TerminalNavigation()
    navigation_node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
