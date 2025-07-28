import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import os
import time

class TerminalNavigation(Node):
    def __init__(self):
        super().__init__('terminal_navigation')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._current_pose = None

        # Updated starting x position for shelves 1, 2, and 3, and shelves 4, 5, and 6
        self.x_start_shelves_1_to_3 = 9.5
        self.x_start_shelves_4_to_6 = 21.5  # Updated to start at 21.5
        self.x_spacing = 1.2
        self.num_tables_per_shelf = 8  # 8 tables per shelf

        # Define the y positions for all 6 shelves
        self.shelf_positions = {
            1: 1.75,  # Shelf 1
            2: 4.25,  # Shelf 2
            3: 6.75,  # Shelf 3
            4: 1.75,  # Shelf 4 (same y as Shelf 1)
            5: 4.25,  # Shelf 5 (same y as Shelf 2)
            6: 6.75   # Shelf 6 (same y as Shelf 3)
        }

        # Define working stations and home position
        self.working_station_1 = (12.9, 9.5, 0.0, 0.0, 0.0, 0.0, 1.0)  # Position for Working Station 1
        self.working_station_2 = (23.1, 9.5, 0.0, 0.0, 0.0, 0.0, 1.0)  # Position for Working Station 2
        self.home_position = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)  # Home Position

        # Subscribe to the current robot pose using PoseWithCovarianceStamped
        self._pose_subscriber = self.create_subscription(
            PoseWithCovarianceStamped,
            'amcl_pose',
            self.pose_callback,
            10)

    def pose_callback(self, msg):
        self._current_pose = msg.pose.pose  # Access the pose inside PoseWithCovarianceStamped

    def run(self):
        while True:
            self.print_menu()
            shelf, table = self.get_user_choice()
            if shelf == 0 and table == 0:  # Exit option
                print("Exiting...")
                break
            else:
                self.navigate_to_shelf_table(shelf, table)

                # Navigate to corresponding working station based on shelf
                if shelf in [1, 2, 3]:
                    self.navigate_to_fixed_position("Working Station 1", self.working_station_1)
                elif shelf in [4, 5, 6]:
                    self.navigate_to_fixed_position("Working Station 2", self.working_station_2)

                # Wait for 5 seconds
                print("Waiting for 5 seconds at the working station...")
                time.sleep(5)

                # Navigate back to home
                self.navigate_to_fixed_position("Home Position", self.home_position)

                # Ask user if they want to pick another table or exit
                self.prompt_pick_table()

    def print_menu(self):
        # Updated message format
        print("Select a shelf and table to navigate to:")
        for shelf in self.shelf_positions:
            print(f"Shelf {shelf} with {self.num_tables_per_shelf} tables")
        print("Enter 0,0 to Exit")

    def get_user_choice(self):
        while True:
            try:
                # Updated user prompt message
                choice = input("\nPlease choose a shelf and table: ")
                if choice.strip() == "":
                    print("Invalid input. Please enter a valid shelf and table.")
                    continue

                shelf, table = map(int, choice.split(','))
                if shelf in self.shelf_positions and 1 <= table <= self.num_tables_per_shelf:
                    return shelf, table
                else:
                    print("Invalid choice. Shelf or table out of range. Please try again.")
            except ValueError:
                print("Invalid input format. Please enter in the format Shelf,Table (e.g., 1,3).")

    def navigate_to_shelf_table(self, shelf, table):
        """
        Function to navigate the robot to the desired table on a shelf.
        Takes shelf and table as input, and calculates the x, y position and orientation.
        """
        position = self.get_shelf_table_position(shelf, table)
        
        if position is not None:
            x, y = position
            # Assuming fixed orientation for simplicity (you can modify it as needed)
            orientation = (0.0, 0.0, 0.0, 1.0)
            print(f"Navigating to Shelf {shelf}, Table {table}: (x = {x}, y = {y}, "
                  f"orientation (x: {orientation[0]}, y: {orientation[1]}, "
                  f"z: {orientation[2]}, w: {orientation[3]}))")
            # Send the robot to the calculated position
            self.send_navigation_goal((x, y, 0.0, *orientation))
        else:
            print("Invalid shelf or table selection.")

    def get_shelf_table_position(self, shelf, table):
        """
        Calculate the x, y position of the table on the selected shelf.
        Shelves 1, 2, and 3 start from x = 9.5, while Shelves 4, 5, and 6 start from x = 21.5.
        The y-values are defined by the shelf.
        """
        if shelf in self.shelf_positions and 1 <= table <= self.num_tables_per_shelf:
            y_value = self.shelf_positions[shelf]
            if shelf in [1, 2, 3]:
                x_value = self.x_start_shelves_1_to_3 + (table - 1) * self.x_spacing
            elif shelf in [4, 5, 6]:
                x_value = self.x_start_shelves_4_to_6 + (table - 1) * self.x_spacing
            return (x_value, y_value)
        else:
            print("Shelf or table is out of range.")
            return None

    def navigate_to_fixed_position(self, position_name, target_pose):
        """
        Function to navigate the robot to a specific fixed position (e.g., working station, home).
        """
        print(f"Navigating to {position_name} (x: {target_pose[0]}, y: {target_pose[1]}, z: {target_pose[2]}, "
          f"orientation (x: {target_pose[3]}, y: {target_pose[4]}, z: {target_pose[5]}, w: {target_pose[6]}))")
        self.send_navigation_goal(target_pose)

    def send_navigation_goal(self, target_pose):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Set position from input
        goal_msg.pose.pose.position.x = target_pose[0]
        goal_msg.pose.pose.position.y = target_pose[1]
        goal_msg.pose.pose.position.z = target_pose[2]

        # Set orientation from input (quaternion x, y, z, w)
        goal_msg.pose.pose.orientation.x = target_pose[3]
        goal_msg.pose.pose.orientation.y = target_pose[4]
        goal_msg.pose.pose.orientation.z = target_pose[5]
        goal_msg.pose.pose.orientation.w = target_pose[6]

        self._action_client.wait_for_server()

        # Send goal and wait for result
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            print('Goal rejected :(')
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        # Handling result status
        time.sleep(3)  # Wait for the robot to stabilize before getting the position
        if self._current_pose:
            final_x = self._current_pose.position.x
            final_y = self._current_pose.position.y
            final_z = self._current_pose.position.z
            final_orientation_x = self._current_pose.orientation.x
            final_orientation_y = self._current_pose.orientation.y
            final_orientation_z = self._current_pose.orientation.z
            final_orientation_w = self._current_pose.orientation.w
            print(f"Navigation succeeded!\nFinal position of the robot: (x: {final_x}, y: {final_y}, z: {final_z})")

            print(f"Final orientation of the robot: (x: {final_orientation_x}, y: {final_orientation_y}, z: {final_orientation_z}, w: {final_orientation_w})\n")
        else:
            print("[ERROR] Current pose not received.")

    def prompt_pick_table(self):
        # Ask the user if they want to pick another table or exit
        while True:
            choice = input("Do you want to pick another table? (Y/N): ").strip().upper()
            if choice == 'Y':
                self.clear_screen()
                return  # Go back toHere’s the remainder of the code for the function prompt_pick_table, ensuring it clears the screen and either restarts or exits based on the user's choice:
            elif choice == 'N':
                print("Exiting...")
                exit()  # Exit the program
            else:
                print("Invalid choice, please enter 'Y' or 'N'.")

    def clear_screen(self):
        # Clear the console screen for a fresh start
        os.system('cls' if os.name == 'nt' else 'clear')

def main(args=None):
    rclpy.init(args=args)
    navigation_node = TerminalNavigation()
    navigation_node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()