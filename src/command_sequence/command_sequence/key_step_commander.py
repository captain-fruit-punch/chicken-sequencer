#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from roarm_msgs.srv import MoveJointCmd
from std_msgs.msg import Float32
import sys
import termios
import tty
import threading

def get_key():
    settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class SequenceCommander(Node):
    def __init__(self):
        super().__init__('sequence_commander')
        
        # Create service client for move joint commands
        self.move_client = self.create_client(MoveJointCmd, '/move_joint_cmd')
        
        # Create publisher and subscriber for gripper commands
        self.gripper_pub = self.create_publisher(Float32, '/gripper_cmd', 10)
        self.gripper_sub = self.create_subscription(
            Float32,
            '/gripper_state',  # Assuming this is the feedback topic
            self.gripper_callback,
            10
        )
        
        # Wait for service to be available
        while not self.move_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Move joint service not available, waiting...')
            
        gripper_open = ('gripper', 1.8)
        gripper_close = ('gripper', 0.2)
        
        ingredient_clearance = ('move', -0.005, 0.28, 0.34, 1.570, -0.05, 0.0)
        grill_clearance = ('move', -0.2, 0.0, 0.33, 1.570, -0.05, 0.0)
        delivery_clearance = ('move', -0.02, -0.27, 0.34, 1.570, -0.05, 0.0)
        
        ingredient_0 = ('move', 0.0, 0.28, 0.3, 1.570, -0.05, 0.0)
        
        grill_0 = ('move', -0.41, 0.0, 0.03, 1.570, -0.05, 0.0)
        grill_0_clearance = ('move', -0.41, 0.0, 0.09, 1.570, -0.05, 0.0)
        grill_0_flip = ('move', -0.41, 0.0, 0.09, -1.570, -0.05, 0.0)
        grill_0_clear_flip = ('move', -0.35, 0.1, -0.02, -1.570, -0.05, 0.0)
        
        delivery_0 = ('move', 0.0, -0.27, 0.3, 1.570, -0.05, 0.0)
        
        # Define sequence of commands
        self.sequence = [
            # Format: ('move', x, y, z, roll, pitch, yaw) or ('gripper', position)
            gripper_open,
            ingredient_clearance,
            ingredient_0,
            gripper_close,
            ingredient_clearance,
            grill_clearance,
            grill_0,
            gripper_open,
            grill_clearance,
            grill_0,
            gripper_close,
            grill_0_clearance,
            grill_0_flip,
            gripper_open,
            grill_0_clear_flip,
            grill_clearance,
            grill_0,
            gripper_close,
            grill_clearance,
            delivery_clearance,
            delivery_0,
            gripper_open,
            delivery_clearance,
            ingredient_clearance,
        ]
        
        self.current_index = 0
        self.get_logger().info('Sequence commander initialized. Press space to execute next command.')
        
        # Start keyboard input thread
        self.running = True
        self.input_thread = threading.Thread(target=self.input_loop)
        self.input_thread.start()
        
    def input_loop(self):
        while self.running:
            key = get_key()
            if key == ' ':  # space key
                self.handle_keypress()
            elif key == '\x03':  # ctrl+c
                self.running = False
                break

    def gripper_callback(self, msg):
        """Callback for gripper state feedback"""
        self.get_logger().info(f'Gripper state feedback: position={msg.data}')
                
    def send_move_command(self, x, y, z, roll, pitch, yaw):
        request = MoveJointCmd.Request()
        request.x = x
        request.y = y
        request.z = z
        request.roll = roll
        request.pitch = pitch
        request.yaw = yaw
        
        self.get_logger().info(f'Sending move command: x={x}, y={y}, z={z}, roll={roll}, pitch={pitch}, yaw={yaw}')
        future = self.move_client.call_async(request)
        
        # Add callback for when the future completes
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f'Move command completed with result: {f.result() if not f.exception() else f"Error: {f.exception()}"}'
            )
        )
        return future
        
    def send_gripper_command(self, position):
        msg = Float32()
        msg.data = position
        self.gripper_pub.publish(msg)
        self.get_logger().info(f'Sending gripper command: position={position}')
        
    def handle_keypress(self):
        if self.current_index >= len(self.sequence):
            self.get_logger().info('Sequence completed!')
            return
            
        command = self.sequence[self.current_index]
        
        if command[0] == 'move':
            future = self.send_move_command(*command[1:])
            # Note: In a real implementation, you might want to wait for the future
            # to complete before allowing the next command
        elif command[0] == 'gripper':
            self.send_gripper_command(command[1])
            
        self.current_index += 1
        self.get_logger().info(f'Command {self.current_index}/{len(self.sequence)} executed. Press space for next command.')

def main(args=None):
    rclpy.init(args=args)
    commander = SequenceCommander()
    
    try:
        rclpy.spin(commander)
    except KeyboardInterrupt:
        commander.running = False
    finally:
        commander.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 