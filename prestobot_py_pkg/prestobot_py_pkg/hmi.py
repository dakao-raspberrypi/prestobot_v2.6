#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations
import pygame

class HmiNode(Node):
    def __init__(self):
        super().__init__("hmi_node")

        # Pygame Initialization
        pygame.init()
        self.screen_ = pygame.display.set_mode((1200, 700)) # Increased height for clarity
        pygame.display.set_caption('Prestobot HMI - Multi-Hall Navigation with Home')

        # Colors and Font
        self.WHITE_ = (255, 255, 255)
        self.BLACK_ = (0, 0, 0)
        self.BLUE_ = (173, 216, 230)
        self.GREEN_ = (144, 238, 144) # Green for the Home button
        self.font1_ = pygame.font.SysFont('sans', 25)

        # Navigation Setup
        self.navigator_ = BasicNavigator()
        self.navigator_.waitUntilNav2Active()
        self.get_logger().info("Nav2 is active. Ready to send goals.")

        # ===============================================================================
        # == 1. DEFINE YOUR HALLS AND INTERMEDIATE WAYPOINTS HERE ==
        # ===============================================================================
        self.hall_definitions_ = {
            1: set(list(range(1, 15)) + list(range(34, 42))),
            2: set(list(range(15, 23)) + list(range(42, 48))),
            3: set(list(range(23, 34)))
        }

        # EDIT the coordinates for your intermediate points
        intermediate_coord_1 = (8.0, 0.0, 0.0) # Connects Hall 1 & 2
        intermediate_coord_2 = (8.0, 40.0, 0.0) # Connects Hall 2 & 3
        self.intermediate_poses_ = {
            'hall_1_2': self.create_pose_stamped(*intermediate_coord_1),
            'hall_2_3': self.create_pose_stamped(*intermediate_coord_2)
        }

        # Track the robot's current location (assume it starts at Home in Hall 1)
        self.current_hall_ = 1

        # ===============================================================================
        # == 2. PASTE YOUR 46 CUSTOM ROOM POSITIONS HERE ==
        # ===============================================================================
        self.room_coordinates = [
            (65.3, 0.0, -1.57), #1
            (58.4, 0.0, -1.57), #2
            (55.3, 0.0, -1.57), #3
            (48.4, 0.0, -1.57), #4
            (45.3, 0.0, -1.57), #5
            (38.4, 0.0, -1.57), #6
            (35.3, 0.0, -1.57), #7
            (28.4, 0.0, -1.57), #8
            (25.3, 0.0, -1.57), #9
            (18.4, 0.0, -1.57), #10
            (15.3, 0.0, -1.57), #11
            (8.4, 0.0, -1.57),  #12
            # Skip Room 13
            (5.6, 0.0, -1.57), #14
            (8.0, 2.4, 3.14), #15
            (8.0, 9.6, 3.14), #16
            (8.0, 12.4, 3.14), #17
            (8.0, 19.6, 3.14), #18
            (8.0, 22.4, 3.14), #19
            (8.0, 29.6, 3.14), #20
            (8.0, 32.4, 3.14), #21
            (8.0, 39.6, 3.14), #22
            (65.3, 40.0, -1.57), #23
            (58.4, 40.0, -1.57), #24
            (55.3, 40.0, -1.57), #25
            (48.4, 40.0, -1.57), #26
            (45.3, 40.0, -1.57), #27
            (38.4, 40.0, -1.57), #28
            (35.3, 40.0, -1.57), #29
            (28.4, 40.0, -1.57), #30
            (25.3, 40.0, -1.57), #31
            (18.4, 40.0, -1.57), #32
            (15.3, 40.0, -1.57), #33
            (30.3, 0.0, 1.57), #34
            (33.4, 0.0, 1.57), #35
            (40.3, 0.0, 1.57), #36
            (43.4, 0.0, 1.57), #37
            (50.3, 0.0, 1.57), #38
            (53.4, 0.0, 1.57), #39
            (60.3, 0.0, 1.57), #40
            (63.4, 0.0, 1.57), #41
            (8.0, 4.6, 0.0),  #42
            (8.0, 7.4, 0.0),  #43
            (8.0, 14.6, 0.0), #44
            (8.0, 17.4, 0.0), #45
            (8.0, 24.6, 0.0), #46
            (8.0, 27.4, 0.0)  #47
        ]
        # ===============================================================================

        self.buttons_ = []
        self.generate_buttons()
        self.timer_ = self.create_timer(0.02, self.update)
        self.get_logger().info("HMI with multi-hall logic and Home position has been started.")

    def generate_buttons(self):
        # --- Create and add the Home button FIRST ---
        home_pose = self.create_pose_stamped(0.0, 0.0, 0.0)
        home_text = self.font1_.render('Home', True, self.BLACK_)
        home_rect = pygame.Rect(50, 50, 120, 60) # Make it bigger
        self.buttons_.append({
            'text': home_text,
            'rect': home_rect,
            'pose': home_pose,
            'label': 'Home',
            'color': self.GREEN_ # Assign a unique color
        })

        # --- Create and add the Room buttons ---
        cols, button_width, button_height, margin = 10, 80, 50, 20
        start_x, start_y = 50, 150 # Start grid lower to not overlap with Home
        room_label = 1
        for i, coords in enumerate(self.room_coordinates):
            if room_label == 13: room_label += 1
            row, col = i // cols, i % cols
            x_pos, y_pos = start_x + col * (button_width + margin), start_y + row * (button_height + margin)

            text_surface = self.font1_.render(str(room_label), True, self.BLACK_)
            button_rect = pygame.Rect(x_pos, y_pos, button_width, button_height)
            goal_pose = self.create_pose_stamped(*coords)

            self.buttons_.append({
                'text': text_surface,
                'rect': button_rect,
                'pose': goal_pose,
                'label': f'Room {room_label}',
                'color': self.BLUE_
            })
            room_label += 1

    def get_hall_for_room(self, room_label):
        # --- Explicitly assign the Home position to Hall 1 ---
        if room_label == 'Home':
            return 1
        try:
            room_number = int(room_label.split(' ')[1])
            for hall, rooms in self.hall_definitions_.items():
                if room_number in rooms:
                    return hall
        except (ValueError, IndexError):
            return None # Should not happen with our labels
        return None

    def handle_navigation_request(self, destination_label, destination_pose):
        destination_hall = self.get_hall_for_room(destination_label)
        if destination_hall is None:
            self.get_logger().error(f"Could not determine hall for '{destination_label}'")
            return

        self.get_logger().info(f"Request: Current Hall={self.current_hall_}, Destination='{destination_label}' in Hall={destination_hall}")

        if self.current_hall_ == destination_hall:
            self.get_logger().info("Same hall. Navigating directly.")
            self.navigator_.goToPose(destination_pose)
        else:
            self.get_logger().info("Different hall. Planning path with intermediates.")
            waypoints = []
            # Logic to select the correct sequence of intermediate points
            if (self.current_hall_, destination_hall) in [(1, 2), (2, 1)]:
                waypoints.append(self.intermediate_poses_['hall_1_2'])
            elif (self.current_hall_, destination_hall) in [(2, 3), (3, 2)]:
                waypoints.append(self.intermediate_poses_['hall_2_3'])
            elif (self.current_hall_, destination_hall) == (1, 3):
                waypoints.extend([self.intermediate_poses_['hall_1_2'], self.intermediate_poses_['hall_2_3']])
            elif (self.current_hall_, destination_hall) == (3, 1):
                waypoints.extend([self.intermediate_poses_['hall_2_3'], self.intermediate_poses_['hall_1_2']])

            waypoints.append(destination_pose)
            self.get_logger().info(f"Sending {len(waypoints)} waypoints to Nav2.")
            self.navigator_.followWaypoints(waypoints)

        # Update the robot's current hall state to the destination's hall
        self.current_hall_ = destination_hall

    def update(self):
        self.screen_.fill(self.WHITE_)
        mouse_pos = pygame.mouse.get_pos()
        for button in self.buttons_:
            pygame.draw.rect(self.screen_, button['color'], button['rect'])
            text_rect = button['text'].get_rect(center=button['rect'].center)
            self.screen_.blit(button['text'], text_rect)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                rclpy.shutdown()
            if event.type == pygame.MOUSEBUTTONDOWN:
                for button in self.buttons_:
                    if button['rect'].collidepoint(mouse_pos):
                        self.handle_navigation_request(button['label'], button['pose'])
                        break

        pygame.display.flip()

    def create_pose_stamped(self, position_x, position_y, rotation_z):
        q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, rotation_z)
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator_.get_clock().now().to_msg()
        goal_pose.pose.position.x = position_x
        goal_pose.pose.position.y = position_y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = q_x
        goal_pose.pose.orientation.y = q_y
        goal_pose.pose.orientation.z = q_z
        goal_pose.pose.orientation.w = q_w
        return goal_pose

def main(args=None):
    rclpy.init(args=args)
    node = HmiNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()