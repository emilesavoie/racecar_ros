#!/usr/bin/env python
import rclpy
import math
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


class SlashController(Node):

    def __init__(self):
        super().__init__("controller")

        # Init subscribers
        self.sub_ref = self.create_subscription(Twist, "ctl_ref", self.read_ref, 1)
        self.sub_prop = self.create_subscription(
            Float32MultiArray, "prop_sensors", self.read_arduino, 1
        )
        self.sub_laser = self.create_subscription(
            Twist, "car_position", self.read_laser, 1
        )

        # Init publishers
        self.pub_cmd = self.create_publisher(Twist, "prop_cmd", 1)

        # Timer
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.timed_controller)

        # Parameters

        # Controller
        self.steering_offset = 0.0  # To adjust according to the vehicle
        
        # K_autopilot is 2x3 matrix: [propulsion_gains; steering_gains]
        self.K_autopilot = np.array([
            [0, 0, 1.0],                      # V control gains [y, theta, v]
            [0.3162277, -0.53827192, 0]       # delta control gains [y, theta, v]
        ])

        self.K_parking = np.array([
          [1, -0.00008306, -0.000166137],                      # V control gains
          [0, 0.2, -0.32]       # delta control gains (K2)
        ])

        self.Rm = 0.64
        self.km = 0.0028
        self.Jm = 0.00005
        self.bm = 0.00001
        self.tau_s = 0.01
        self.N = 15.3
        self.r = 0.04
        self.Cd = 0.5
        self.Cr = 0.01
        self.L = 0.3
        self.m = 5
        self.v_nom = 2
        self.rho = 1.2
        self.Aire = 0.29*0.12

        # Memory

        # References Inputs
        self.propulsion_ref = 0
        self.steering_ref = 0
        self.high_level_mode = 0  # Control mode of this controller node

        # Output commands
        self.propulsion_cmd = 0  # Command sent to propulsion
        self.arduino_mode = 0  # Control mode
        self.steering_cmd = 0  # Command sent to the steering servo

        # Sensing inputs
        self.laser_y = 0
        self.laser_theta = 0
        self.velocity = 0
        self.position = 0

        # Filters
        self.laser_y_old = 0
        self.laser_dy_fill = 0

    #######################################
    def timed_controller(self):

        # Computation of dy / dt with filtering
        self.laser_dy_fill = (
            0.9 * self.laser_dy_fill + 0.1 * (self.laser_y - self.laser_y_old) / self.dt
        )
        self.laser_y_old = self.laser_y

        if self.high_level_mode < 0:
            # Full stop mode
            self.propulsion_cmd = 0  # Command sent to propulsion
            self.arduino_mode = 0  # Control mode
            self.steering_cmd = 0  # Command sent to the steering servo

        else:

            # APP2 (open-loop steering) Controllers Below
            if self.high_level_mode == 1:
                # Open-Loop propulsion and steering
                self.propulsion_cmd = self.propulsion_ref
                self.arduino_mode = 1
                self.steering_cmd = self.steering_ref + self.steering_offset

            # For compatibility mode 0 needs to be closed-loop velocity
            elif self.high_level_mode == 0:
                # Closed-loop velocity on arduino, open-loop steering
                self.propulsion_cmd = self.propulsion_ref
                self.arduino_mode = 1
                self.steering_cmd = self.propulsion_ref + self.steering_offset

            elif self.high_level_mode == 2:
                # Closed-loop position on arduino, open-loop steering
                self.propulsion_cmd = self.propulsion_ref
                self.arduino_mode = 3
                self.steering_cmd = self.steering_ref + self.steering_offset

            # APP4 (closed-loop steering) controllers below
            elif self.high_level_mode == 3 or self.high_level_mode == 5:
                # Closed-loop velocity and steering

                # State vector: [lateral_position, heading_angle, velocity]
                x = [self.laser_y, self.laser_theta, self.velocity]
                # Reference: [desired_y, desired_theta, desired_velocity]
                r = [-0.2, 0, 2] 

                # Get control commands
                u = self.controller1(x, r)

                # Apply commands
                self.steering_cmd = u[1] + self.steering_offset
                self.propulsion_cmd = u[0]
                self.arduino_mode = 2  # Open-loop mode on arduino

            elif self.high_level_mode == 4:
                # Closed-loop position and steering
		
		#self.get_logger().debug("In high level mode 4")
                # TODO: Define state and reference for parking controller
                x = [self.position, self.laser_y, self.laser_theta]
                r = [4, -0.2, 0]  # Adjust reference as needed

                u = self.controller2(x, r)

                self.steering_cmd = u[1] + self.steering_offset
                self.propulsion_cmd = u[0]
                self.arduino_mode = 5  # Mode on arduino

            elif self.high_level_mode == 6:
                # Reset encoders
                self.propulsion_cmd = 0
                self.arduino_mode = 4
                self.steering_cmd = 0

            elif self.high_level_mode == 7:
                # Closed-loop velocity open-loop steering
                self.propulsion_cmd = self.propulsion_ref
                self.arduino_mode = 2
                self.steering_cmd = self.steering_ref + self.steering_offset

            elif self.high_level_mode == 8:
                # Template for custom controllers
                self.steering_cmd = 0 + self.steering_offset
                self.propulsion_cmd = 0
                self.arduino_mode = 0

        self.send_arduino()

    #######################################
    def control_law(self, x, K, r):
        """
        Compute control law u = -K(x - r) with saturation
        
        Args:
            x: State vector [y, theta, v]
            K: Control gain matrix (2x3)
            r: Reference vector [y_ref, theta_ref, v_ref]
            
        Returns:
            V: Propulsion voltage command (saturated)
            delta: Steering angle command (saturated)
        """
        V_sat = 8.4
        delta_sat = 0.5

        x_tilde = x - r  # Error from reference
        u = -K @ x_tilde  # Control command
        
        V = float(u[0])
        delta = float(u[1])
        
        # Apply saturations
        V = np.clip(V+1.4, 0, V_sat)
        delta = np.clip(delta, -delta_sat, delta_sat)
        
        return V, delta

    def control_law2(self, x, K, r):
        delta_sat = 0.5

        x_tilde = x - r  # erreur par rapport à la consigne
        u = -K @ x_tilde     # commande continue
        vd = float(u[0])
        delta = float(u[1])
        # saturations
        delta = np.clip(delta, -delta_sat, delta_sat)
        return vd, delta

    #######################################
    def sign_smooth(self, x):
        """Smooth sign function"""
        if x == 0:
            return 0
        elif x > 0:
            return 1
        else:
            return -1

    #######################################
    def controller1(self, y, r):
        """
        Autopilot controller for velocity and steering
        
        Args:
            y: State vector [y_position, theta, velocity]
            r: Reference vector [y_ref, theta_ref, v_ref]
            
        Returns:
            u: Control vector [V_command, delta_command]
        """
        # Convert to numpy arrays
        x = np.array(y)
        r_vec = np.array(r)

        # Compute closed-loop control
        V, delta = self.control_law(x, self.K_autopilot, r_vec)

        u = np.array([V, delta])

        return u

    #######################################
    def controller2(self, y, r):

        x = np.array(y)
        r_vec = np.array(r)

        # commande (fermée)
        vd, delta = self.control_law2(x, self.K_parking, r_vec)

        u = np.array([vd, delta])

        return u
    #######################################
    def read_ref(self, ref_msg):
        """Read reference commands from topic"""
        self.propulsion_ref = ref_msg.linear.x
        self.high_level_mode = ref_msg.linear.z
        self.steering_ref = ref_msg.angular.z

    #######################################
    def read_laser(self, msg):
        """Read laser/position sensor data"""
        self.laser_y = msg.linear.y
        self.laser_theta = msg.angular.z

    #######################################
    def read_arduino(self, msg):
        """Read feedback from arduino (encoders)"""
        self.velocity = msg.data[1]
        self.position = msg.data[0]

    #######################################
    def send_arduino(self):
        """Send commands to arduino"""
        # Init command msg
        cmd_prop = Twist()

        # Package commands
        cmd_prop.linear.x = float(self.propulsion_cmd)  # Propulsion command
        cmd_prop.linear.z = float(self.arduino_mode)  # Control mode
        cmd_prop.angular.z = float(self.steering_cmd)  # Steering command

        # Publish command
        self.pub_cmd.publish(cmd_prop)

    #######################################
    def pub_kinematic(self):
        """Publish kinematic information (unused)"""
        # init msg
        pos = Twist()
        vel = Twist()
        acc = Twist()

        # Set values
        pos.linear.x = 0
        vel.linear.x = 0
        acc.linear.x = 0

        # Publish (note: publishers not initialized for this)
        # self.pub_pos.publish(pos)
        # self.pub_vel.publish(vel)
        # self.pub_acc.publish(acc)


def main(args=None):
    rclpy.init(args=args)
    node = SlashController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
