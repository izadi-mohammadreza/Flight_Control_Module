import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleStatus, VehicleOdometry, VehicleAttitudeSetpoint, ActuatorMotors, VehicleCommand, OffboardControlMode, VehicleThrustSetpoint, VehicleTorqueSetpoint
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import SetParametersResult
import numpy as np
from scipy.spatial.transform import Rotation as R
from numpy.linalg import norm
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from controller import calculate_controller_output






class ControllerNode(Node):
    def __init__(self):
    
    
        super().__init__('controller_node')
        self.current_status=VehicleStatus()
        self.in_sitl_mode = True     # False for real test
        self.control_mode = 2        # default control mode
        self.num_of_arms = 4  
        self.thrust_constant = 5.84e-06  
        self.arm_length = 0.25  
        self.moment_constant = 0.06 
        self.max_rotor_speed=1100 

        self.PWM_MIN = 1075  
        self.PWM_MAX = 1950  
        self.omega_to_pwm_coefficients = np.array([0.001142, 0.2273, 914.2]) 

        
        self.zero_position_armed = 100  # Default PWM value for zero throttle
        self.input_scaling = 1000  # Scaling factor for converting 
        
        
        self.position_W = np.zeros(3)
        self.velocity_W = np.zeros(3)
        self.R_B_W = np.eye(3)
        self.angular_velocity_B = np.zeros(3)


        self.acceleration_W = np.zeros(3)
        self.r_position_W = np.zeros(3)
        self.r_velocity_W = np.zeros(3)
        self.r_acceleration_W = np.zeros(3)
        self.r_yaw = 0.0
        self.r_yaw_rate = 0.0
        

        self.uav_mass = 1.725
        self.gravity = 9.81
        self.position_gain = np.array([7, 7, 6])
        self.velocity_gain = np.array([6, 6, 3])
        self.attitude_gain = np.array([3.5, 3.5, 0.3])
        self.angular_rate_gain = np.array([0.5, 0.5, 0.1])
        #self.inertia_matrix = np.array([0.8612, 0.8962, 0.16088])
        self.inertia_matrix = np.array([[0.029125, 0, 0],
                                 [0, 0.029125, 0],
                                 [0, 0, 0.055225]])

        
        self.torques_and_thrust_to_rotor_velocities = np.zeros((4, 4))
        self.rotor_velocities_to_torques_and_thrust = np.zeros((4, 4))
        self.throttles_to_normalized_torques_and_thrust = np.zeros((4, 4))

        # Configure QoS profile according to PX4
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )


		#Setup subscribers
		
        self.subscription_status = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.status_callback, qos_profile)
        self.subscription_pose = self.create_subscription(PoseStamped, '/command/pose', self.command_pose_callback, 10)
        self.subscription_odometry = self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odometry_callback, qos_profile)
        


        # Setup Publishers
        self.attitude_setpoint_publisher = self.create_publisher(VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint', qos_profile)
        self.actuator_motors_publisher  = self.create_publisher(ActuatorMotors, '/fmu/in/actuator_motors', qos_profile) 
        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.thrust_setpoint_publisher = self.create_publisher(VehicleThrustSetpoint, '/fmu/in/vehicle_thrust_setpoint', qos_profile)
        self.torque_setpoint_publisher = self.create_publisher(VehicleTorqueSetpoint, '/fmu/in/vehicle_torque_setpoint', qos_profile)
            
            

        # Creating timers
        self.offboard_timer = self.create_timer(0.33, self.publish_offboard_control_mode_msg)
        self.controller_timer = self.create_timer(0.01, self.update_controller_output)

        # Setup Timer
        self.timer = self.create_timer(0.1, self.update_controller_output)
        

        self.compute_control_allocation_and_actuator_effect_matrices()
        

    def compute_control_allocation_and_actuator_effect_matrices(self):
        kDegToRad = np.pi / 180
        if self.num_of_arms == 4:
            kS = np.sin(45 * kDegToRad)
            rotor_velocities_to_torques_and_thrust = np.array([
                [-kS, kS, kS, -kS],
                [-kS, kS, -kS, kS],
                [-1, -1, 1, 1],
                [1, 1, 1, 1]
            ])
            mixing_matrix = np.array([
                [-0.495384, -0.707107, -0.765306, 1.0],
                [0.495384, 0.707107, -1.0, 1.0],
                [0.495384, -0.707107, 0.765306, 1.0],
                [-0.495384, 0.707107, 1.0, 1.0]
            ])

            self.throttles_to_normalized_torques_and_thrust = np.array([
                [-0.5718, 0.4376, 0.5718, -0.4376],
                [-0.3536, 0.3536, -0.3536, 0.3536],
                [-0.2832, -0.2832, 0.2832, 0.2832],
                [0.2500, 0.2500, 0.2500, 0.2500]
            ])

            # Control allocation matrix: Wrench to Rotational velocities
            k = np.diag([
                self.thrust_constant * self.arm_length,
                self.thrust_constant * self.arm_length,
                self.moment_constant * self.thrust_constant,
                self.thrust_constant
            ])
            
            
            rotor_velocities_to_torques_and_thrust = k @ rotor_velocities_to_torques_and_thrust
            
            print("rotor_velocities_to_torques_and_thrust2 =", rotor_velocities_to_torques_and_thrust)
            
            
            
            self.torques_and_thrust_to_rotor_velocities = np.linalg.pinv(rotor_velocities_to_torques_and_thrust)


            # Output for debugging purposes
            print("k=", k)
            print("rotor_velocities_to_torques_and_thrust =", rotor_velocities_to_torques_and_thrust)
            print("torques_and_thrust_to_rotor_velocities =", self.torques_and_thrust_to_rotor_velocities)
            print("throttles_to_normalized_torques_and_thrust =", self.throttles_to_normalized_torques_and_thrust)
        else:
            print("[controller] Unknown UAV parameter num_of_arms. Cannot calculate control matrices")
            
            
            
    def px4_inverse(self, wrench):
        if self.num_of_arms == 4:
            omega = np.dot(self.torques_and_thrust_to_rotor_velocities, wrench)
            omega[omega < 0] = 0
            omega = np.sqrt(omega)

            ones_temp = np.ones(4)
            pwm = (self.omega_to_pwm_coefficients[0] * omega**2 +
                   self.omega_to_pwm_coefficients[1] * omega +
                   self.omega_to_pwm_coefficients[2] * ones_temp)

            throttles = (pwm - (self.PWM_MIN * ones_temp)) / (self.PWM_MAX - self.PWM_MIN)

            normalized_torque_thrust = np.dot(self.throttles_to_normalized_torques_and_thrust, throttles)
            
            return normalized_torque_thrust, throttles
        else:
            print("[controller] Unknown UAV parameter num_of_arms. Cannot calculate control matrices")    
   
                      
            
    def px4_inverse_sitl(self, wrench):
    
        normalized_torque_thrust=np.zeros(self.num_of_arms)
        throttles=np.zeros(self.num_of_arms)
        omega = np.zeros(self.num_of_arms)
        ones_temp = np.ones(self.num_of_arms)

        # Control allocation: Wrench to Rotational velocities (omega)
        omega = np.dot(self.torques_and_thrust_to_rotor_velocities, wrench)
        omega = np.sqrt(np.maximum(omega, 0))  # Element-wise max to ensure non-negative under sqrt
        #print(omega)
        throttles = (omega - (self.zero_position_armed * ones_temp))
        throttles /= self.input_scaling
        
        print(throttles)

        # Inverse Mixing: throttles to normalized torques and thrust
        normalized_torque_thrust = np.dot(self.throttles_to_normalized_torques_and_thrust, throttles)
        
        
        #print("omegaaaa =", omega)
        #print("throttlessss =", throttles)
        #print("normalized_torque_thrustttt =", normalized_torque_thrust)
        
        return normalized_torque_thrust, throttles

    
          
        
    def publish_vehicle_command(self, command, param1, param2):
        """
        Publishes a VehicleCommand message with the given parameters.
        """
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.get_clock().now().nanoseconds // 1000  # Convert ns to microseconds
        #self.vehicle_command_publisher.publish(msg)

    def arm(self):

        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0)
        self.get_logger().info("Arm command sent")

    def disarm(self):

        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 0)
        self.get_logger().info("Disarm command sent") 
        
        
        
        
    def publish_offboard_control_mode_msg(self):
        offboard_msg = OffboardControlMode()
        offboard_msg.position = False
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.body_rate = False

        # Setting the mode based on the internal control_mode_
        if self.control_mode == 1:
            offboard_msg.attitude = True
            offboard_msg.thrust_and_torque = False
            offboard_msg.direct_actuator = False
        elif self.control_mode == 2:
            offboard_msg.attitude = False
            offboard_msg.thrust_and_torque = True
            offboard_msg.direct_actuator = False
        elif self.control_mode == 3:
            offboard_msg.attitude = False
            offboard_msg.thrust_and_torque = False
            offboard_msg.direct_actuator = True
        else:
            offboard_msg.attitude = True
            offboard_msg.thrust_and_torque = False
            offboard_msg.direct_actuator = False

        offboard_msg.timestamp = self.get_clock().now().nanoseconds // 1000  # Convert nanoseconds to microseconds
        self.offboard_control_mode_publisher.publish(offboard_msg)
        #self.get_logger().info("Offboard enabled")  
        
        
                     
    def command_pose_callback(self, msg):
        #print('rezacommand')
        # Extract trajectory point from Pose message
        position, orientation = self.eigen_trajectory_point_from_pose_msg(msg)
        #self.get_logger().info("Controller got first command message.")
        # Simulate setting a trajectory point with a fixed velocity and acceleration for demonstration
        self.set_trajectory_point(position, np.zeros(3), np.zeros(3), orientation, np.zeros(3))

    def eigen_trajectory_point_from_pose_msg(self, msg):
        position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        orientation = R.from_quat([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ])
        return position, orientation
        

    def set_trajectory_point(self, position, velocity, acceleration, orientation, angular_velocity):
        self.r_position_W = position
        self.r_velocity_W = velocity
        self.r_acceleration_W = acceleration
        self.r_R_B_W = orientation.as_matrix()
        self.r_yaw = orientation.as_euler('xyz')[-1]
        self.r_yaw_rate = angular_velocity[2]
        #self.get_logger().info(f"Trajectory set: Position {position}, Yaw {self.r_yaw}")
        

    def odometry_callback(self, odom_msg):
        #self.get_logger().info("Controller got first odometry message.")
        position, orientation, velocity, angular_velocity = self.eigen_odometry_from_px4_msg(odom_msg)
        self.set_odometry(position, orientation, velocity, angular_velocity)

    def eigen_odometry_from_px4_msg(self, msg):
    
        position = self.rotate_vector_from_to_enu_ned(np.array([msg.position[0], msg.position[1], msg.position[2]]))
        velocity = self.rotate_vector_from_to_enu_ned(np.array([msg.velocity[0], msg.velocity[1], msg.velocity[2]]))
        
        angular_velocity = self.rotate_vector_from_to_frd_flu(np.array([msg.angular_velocity[0], msg.angular_velocity[1], msg.angular_velocity[2]]))
        
        
        quaternion = np.array([msg.q[1], msg.q[2], msg.q[3], msg.q[0]])    #x,y,z,w
        orientation = self.rotate_quaternion_from_to_enu_ned(quaternion)   #x,y,z,w
        

        return position, orientation, velocity, angular_velocity
        
        
        
    def set_odometry(self, position_W, orientation_B_W, velocity_B, angular_velocity):
    
        rotation = R.from_quat(orientation_B_W)
        self.R_B_W= rotation.as_matrix()
        self.position_W = position_W
        self.velocity_W = self.R_B_W @ velocity_B
        self.angular_velocity_B = angular_velocity
        #self.velocity_W = np.dot(R_B_W, velocity_B)
        
        euler_angles_W = rotation.as_euler('zyx')  # or use another sequence if required
        
        
        
        
            
    def rotate_quaternion_from_to_enu_ned(self, quat_in):                          #*************************************
        # Define the static rotation from NED to ENU using Euler angles
    	# Rotation order is z-y-x, which corresponds to intrinsic rotations around new axes
    	
    	euler_1 = (np.pi, 0.0, np.pi/2)  # Rotation: 180 degrees around Z and 90 degrees around X
    	NED_ENU_Q = R.from_euler('xyz', euler_1).as_quat()

    	# Define the static rotation between aircraft and base_link frames
    	euler_2 = (np.pi, 0.0, 0.0)  # Rotation: 180 degrees around X
    	AIRCRAFT_BASELINK_Q = R.from_euler('xyz', euler_2).as_quat()

    	# Convert input quaternion to scipy's Rotation object

    	rotation_in = R.from_quat(quat_in)

    	# Apply the transformations
    	# Note the order of multiplication (right to left) as per quaternion multiplication rules

    	rotation_out = R.from_quat(NED_ENU_Q) * rotation_in * R.from_quat(AIRCRAFT_BASELINK_Q)
    	
    	
    	qout=np.array([rotation_out.as_quat()[0],rotation_out.as_quat()[1],rotation_out.as_quat()[2],rotation_out.as_quat()[3]])
    	#print(qout)

    	# Return the resulting quaternion
    	return qout  
    	
 



        
        

    def rotate_vector_from_to_enu_ned(self, vec):
        return np.array([vec[1], vec[0], -vec[2]])

    def rotate_vector_from_to_frd_flu(self, vec):
        return np.array([vec[0], -vec[1], -vec[2]])
   


    def status_callback(self, status_msg):
 
        self.current_status = status_msg
        #print(self.current_status.nav_state)
        
        if self.current_status.arming_state == 2:  
            self.get_logger().info("ARMED - vehicle_status_msg.")
        else:
            self.get_logger().info("NOT ARMED - vehicle_status_msg.")

        if self.current_status.nav_state == 14:  
            self.get_logger().info("OFFBOARD - vehicle_status_msg.")
        else:
            self.get_logger().info("NOT OFFBOARD - vehicle_status_msg.")
            


         
        
        
        
    def publish_attitude_setpoint_msg(self, controller_output, desired_quaternion):     #mode 1
        """
        Publishes an attitude setpoint message.

        Args:
        - controller_output (np.ndarray): Array containing torque (x,y,z) and thrust values.
        - desired_quaternion (np.ndarray): Quaternion representing desired orientation.
        """
        msg = VehicleAttitudeSetpoint()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000  # Convert ns to microseconds
        # Rotating quaternion from ENU to NED coordinate system
        rotated_quat = self.rotate_quaternion_from_to_enu_ned(desired_quaternion)
        
        # Setting quaternion components
        msg.q_d = [rotated_quat[3], rotated_quat[0], rotated_quat[1], rotated_quat[2]]  # w, x, y, z order
        
        # Setting thrust
        if controller_output[3] > 0.1:
            msg.thrust_body = [0.0, 0.0, -controller_output[3]]
        else:
            msg.thrust_body = [0.0, 0.0, -0.1]

        self.attitude_setpoint_publisher.publish(msg)
        #print(controller_output)
        #self.get_logger().info('Published attitude setpoint message.') 
 
    def publish_thrust_torque_msg(self, controller_output):                             #mode 2
        """
        Publishes thrust and torque setpoint messages.

        Args:
        - controller_output (np.ndarray): Array containing torque (x,y,z) and thrust values.
        """
        thrust_sp_msg = VehicleThrustSetpoint()
        torque_sp_msg = VehicleTorqueSetpoint()

        timestamp_now = self.get_clock().now().nanoseconds // 1000
        thrust_sp_msg.timestamp_sample = torque_sp_msg.timestamp_sample = timestamp_now
        thrust_sp_msg.timestamp = torque_sp_msg.timestamp = timestamp_now

        # Setting thrust values
        thrust_sp_msg.xyz = [0.0, 0.0, -max(controller_output[3], 0.1)]  # Ensure non-negative thrust with minimum

        # Setting torque values
        rotated_torque = self.rotate_vector_from_to_frd_flu(controller_output[:3])
        torque_sp_msg.xyz = rotated_torque.tolist()
        
        #print(thrust_sp_msg.xyz)
        #print(torque_sp_msg.xyz)

        # Publish messages
        self.thrust_setpoint_publisher.publish(thrust_sp_msg)
        self.torque_setpoint_publisher.publish(torque_sp_msg)
        
        #print(thrust_sp_msg.xyz,torque_sp_msg.xyz)
        #self.get_logger().info('Published thrust and torque setpoint messages.')    

    def publish_actuator_motors_msg(self, throttles):                                 #mode 3


        #print(throttles)
        actuator_motors_msg = ActuatorMotors()
        actuator_motors_msg.control = list(throttles[:4]) + [float('nan')] * 8  # Set first four and fill the rest with NaN
        actuator_motors_msg.reversible_flags = 0
        actuator_motors_msg.timestamp = (self.get_clock().now().nanoseconds // 1000)
        actuator_motors_msg.timestamp_sample = actuator_motors_msg.timestamp

        self.actuator_motors_publisher.publish(actuator_motors_msg)
        #print(throttles)
        #self.get_logger().info('Published actuator motors message.')    
        

    def update_controller_output(self):
        controller_output, desired_quaternion = calculate_controller_output(
            self.position_W, self.r_position_W, self.velocity_W, self.r_velocity_W,
            self.R_B_W, self.uav_mass, self.gravity, self.r_acceleration_W, self.position_gain,
            self.velocity_gain, self.r_yaw, self.angular_velocity_B, self.inertia_matrix,
            self.attitude_gain, self.angular_rate_gain, self.r_yaw_rate)
        
        #print(controller_output)
        # Normalize the controller output
        if self.in_sitl_mode:
            normalized_torque_thrust, throttles = self.px4_inverse_sitl(controller_output)
            #normalized_torque_thrust, throttles = self.px4_inverse_sitl([1,1,1,1])
        else:
            normalized_torque_thrust, throttles = self.px4_inverse(controller_output)
        
        # Publish the controller output
        if self.current_status.nav_state == 14:

            if self.control_mode == 1:
                self.publish_attitude_setpoint_msg(normalized_torque_thrust, desired_quaternion)
            elif self.control_mode == 2:
                self.publish_thrust_torque_msg(normalized_torque_thrust)
            elif self.control_mode == 3:
                self.publish_actuator_motors_msg(throttles)
            else:
                self.publish_attitude_setpoint_msg(normalized_torque_thrust, desired_quaternion)


    

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

