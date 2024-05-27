#! /usr/bin/env python
# -*- coding: utf-8 -*-

import curses
import os
import signal
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from rclpy.duration import Duration as rclpyDuration


class TextWindow():

    def __init__(self, stdscr, lines=10):
        self._screen = stdscr
        self._screen.nodelay(True)
        curses.curs_set(0)
        self._num_lines = lines

    def read_key(self):
        keycode = self._screen.getch()
        return keycode if keycode != -1 else None

    def clear(self):
        self._screen.clear()

    def write_line(self, lineno, message):
        if lineno < 0 or lineno >= self._num_lines:
            raise ValueError('lineno out of bounds')
        height, width = self._screen.getmaxyx()
        y = (height / self._num_lines) * lineno
        x = 10
        for text in message.split('\n'):
            text = text.ljust(width)
            self._screen.addstr(int(y), int(x), text)
            y += 1

    def refresh(self):
        self._screen.refresh()

    def beep(self):
        curses.flash()


class ArmTeleop(Node):

    def __init__(self, interface):
        super().__init__('arm_teleop')

        self._interface = interface
        self._pub_cmd_arm = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self._pub_cmd_torso = ActionClient(self, FollowJointTrajectory, '/torso_controller/follow_joint_trajectory')
        self._pub_cmd_gripper = ActionClient(self, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')

        self._hz = 10
        self._joint_positions_arm = [0.2, -1.34, -0.2, 1.94, -1.57, 1.37, 0.0]
        self._joint_position_torso = 0.15
        self._joint_position_gripper = 0.0
        self._selected_joint = None
        self._running = True
        self._goal_sent = False
        self._last_pressed = {}
        self._pressed_duration = 0.4

    movement_bindings = {
        ord('a'): -0.01,
        ord('d'): 0.01,
        ord('w'): 0.001,  # gripper open
        ord('s'): -0.001,  # gripper close
        ord('o'): -1.57,  # fast turning positive
        ord('p'): 1.57  # fast turning negative
    }

    joint_bindings = {
        ord('1'): 0,
        ord('2'): 1,
        ord('3'): 2,
        ord('4'): 3,
        ord('5'): 4,
        ord('6'): 5,
        ord('7'): 6,
        ord('8'): 'torso',
        ord('9'): 'gripper'
    }

    def run(self):
        while self._running:
            keycode = self._interface.read_key()
            if keycode is not None:
                self._key_pressed(keycode)
            self._set_velocity()
            if self._goal_sent:
                self._publish()
                self._goal_sent = False
            time.sleep(1.0 / self._hz)

    def _key_pressed(self, keycode):
        if keycode == ord('q'):
            self._running = False
            os.kill(os.getpid(), signal.SIGINT)
        elif keycode in self.joint_bindings:
            self._selected_joint = self.joint_bindings[keycode]
            if isinstance(self._selected_joint, int):
                self._interface.write_line(1, f'Selected Joint: {self._selected_joint + 1}')
            else:
                self._interface.write_line(1, f'Selected Joint: {self._selected_joint.capitalize()}')
        elif keycode in self.movement_bindings and self._selected_joint is not None:
            self._last_pressed[keycode] = self.get_clock().now()
            if isinstance(self._selected_joint, int):
                self._interface.write_line(2, f'Joint {self._selected_joint + 1} Position: {self._joint_positions_arm[self._selected_joint]:.2f}')
            elif self._selected_joint == 'torso':
                self._interface.write_line(2, f'Torso Position: {self._joint_position_torso:.2f}')
            elif self._selected_joint == 'gripper':
                self._interface.write_line(2, f'Gripper Position: {self._joint_position_gripper:.2f}')
        elif keycode == ord('r'):
            self._selected_joint = None
            self._interface.write_line(1, 'Reselect Joint')

    def _set_velocity(self):
        now = self.get_clock().now()
        keys = []
        for keycode, press_time in self._last_pressed.items():
            if now - press_time < rclpyDuration(seconds=self._pressed_duration):
                keys.append(keycode)
        if keys:
            for k in keys:
                if isinstance(self._selected_joint, int):
                    self._joint_positions_arm[self._selected_joint] += self.movement_bindings[k]
                elif self._selected_joint == 'torso':
                    self._joint_position_torso += self.movement_bindings[k]
                elif self._selected_joint == 'gripper':
                    self._joint_position_gripper += self.movement_bindings[k]
                self._goal_sent = True
            self._last_pressed.clear()

    def _publish(self):
        self._interface.clear()
        self._interface.write_line(0, 'Use keys 1-9 to select joint, A/D/W/S to adjust, W and S is for small turning 0.001, A and D is for 0.01, O and P is for fast turning 1.57, R to reselect, Q to exit.')
        self._interface.refresh()

        if isinstance(self._selected_joint, int):
            self._send_goal_arm()
        elif self._selected_joint == 'torso':
            self._send_goal_torso()
        elif self._selected_joint == 'gripper':
            self._send_goal_gripper()

    def _send_goal_arm(self):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [
            'arm_1_joint', 'arm_2_joint', 'arm_3_joint',
            'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint'
        ]

        point = JointTrajectoryPoint()
        point.positions = self._joint_positions_arm.copy()
        point.time_from_start = Duration(sec=1, nanosec=0)

        goal_msg.trajectory.points = [point]

        self._pub_cmd_arm.wait_for_server()
        self._send_goal_future = self._pub_cmd_arm.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def _send_goal_torso(self):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['torso_lift_joint']

        point = JointTrajectoryPoint()
        point.positions = [self._joint_position_torso]
        point.time_from_start = Duration(sec=1, nanosec=0)

        goal_msg.trajectory.points = [point]

        self._pub_cmd_torso.wait_for_server()
        self._send_goal_future = self._pub_cmd_torso.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def _send_goal_gripper(self):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['gripper_right_finger_joint', 'gripper_left_finger_joint']

        point = JointTrajectoryPoint()
        point.positions = [self._joint_position_gripper, self._joint_position_gripper]
        point.time_from_start = Duration(sec=1, nanosec=0)

        goal_msg.trajectory.points = [point]

        self._pub_cmd_gripper.wait_for_server()
        self._send_goal_future = self._pub_cmd_gripper.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.error_code}')


def execute(stdscr):
    rclpy.init()
    app = ArmTeleop(TextWindow(stdscr))
    app.run()
    app.destroy_node()
    rclpy.shutdown()


def main():
    try:
        curses.wrapper(execute)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()




