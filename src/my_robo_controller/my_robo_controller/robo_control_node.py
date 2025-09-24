#!/usr/bin/env python3
import sys
import os

# Add your virtual environment path if needed
venv_path = '/media/bishaldas/Apps/Ros2_4wheel_Control_bot/src/.venv/lib/python3.12/site-packages'
if venv_path not in sys.path:
    sys.path.insert(0, venv_path)

import cv2
import mediapipe as mp
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


# ---------------- ROS2 Publisher ----------------
class CarPublisher(Node):
    def __init__(self):
        super().__init__('car_controller')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def move(self, linear_x=0.1, angular_z=0.0):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.pub.publish(msg)
        self.get_logger().info(f"Publishing: linear={linear_x}, angular={angular_z}")


# ---------------- MediaPipe + Gesture Logic ----------------
def detect_gesture(hand_landmarks):
    lm = hand_landmarks.landmark

    # Forward: Index finger up
    if lm[8].y < lm[6].y:
        return "Forward"  # Changed from "F" to "Forward"
    elif lm[20].y < lm[18].y:
        return "Backward"  # Changed from "B" to "Backward"
    else:
        return "Stop"  # Changed from "S" to "Stop"


# ---------------- Main ----------------
def main():
    rclpy.init()
    car = CarPublisher()

    mp_hands = mp.solutions.hands
    hands = mp_hands.Hands(max_num_hands=1)
    mp_draw = mp.solutions.drawing_utils

    cap = cv2.VideoCapture(0)

    try:
        while rclpy.ok():
            ret, img = cap.read()
            if not ret:
                break

            img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            results = hands.process(img_rgb)

            gesture = "Stop"
            if results.multi_hand_landmarks:
                for handLms in results.multi_hand_landmarks:
                    mp_draw.draw_landmarks(img, handLms, mp_hands.HAND_CONNECTIONS)
                    gesture = detect_gesture(handLms)

            # Map gesture → movement (FIXED - now matches the gesture names!)
            if gesture == "Forward":
                car.move(0.3, 0.0)
            elif gesture == "Backward":
                car.move(-0.3, 0.0)
            else:  # Stop
                car.move(0.0, 0.0)

            # Show camera feed with gesture
            cv2.putText(img, f"Gesture: {gesture}", (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow("Gesture Control", img)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        pass

    cap.release()
    cv2.destroyAllWindows()
    car.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()