#!/usr/bin/env python3
...

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.parameter import Parameter
from std_msgs.msg import Float64
# import yaml
from rclpy.qos import QoSProfile

class Arm_Driver(Node):

    def __init__(self):
        super().__init__('arm_driver_node')

        # setup ros parameters
        self.setup_params()

        # ros subscribers and publishers
        self.setup_ros()

        # Start the timer to update PWM gradually
        self.create_timer(self.timer_interval, self.command_pub_callback)


    def setup_params(self):

        # Add motor parameters

        servo_ch_list_descriptor = ParameterDescriptor(description='List of servo channels for docking arm!')
        self.declare_parameter('servo_ch_list',[1], servo_ch_list_descriptor)

        servo_topic_list_descriptor = ParameterDescriptor(description='List of servo topics for docking arm!')
        self.declare_parameter('servo_topic_list',['servo_#'], servo_topic_list_descriptor)

        # angle_hard_min_deg_descriptor = ParameterDescriptor(description='Harware minimum angle of servo in degrees!')
        # self.declare_parameter('angle_hard_min_deg',[1], angle_hard_min_deg_descriptor)

        # angle_hard_max_deg_descriptor = ParameterDescriptor(description='Harware maximum angle of servo in degrees!')
        # self.declare_parameter('angle_hard_max_deg',[1], angle_hard_max_deg_descriptor)

        angle_soft_min_deg_descriptor = ParameterDescriptor(description='Software limited minimum angle of servo in degrees!')
        self.declare_parameter('angle_soft_min_deg',[1], angle_soft_min_deg_descriptor)

        angle_max_deg_descriptor = ParameterDescriptor(description='Software limited maximum angle of servo in degrees!')
        self.declare_parameter('angle_soft_max_deg',[1], angle_max_deg_descriptor)
        
        angle_offset_descriptor = ParameterDescriptor(description='Servo Angle offset (in degrees)!')
        self.declare_parameter('angle_offset',[1], angle_offset_descriptor)

        angle_equation_c1_descriptor = ParameterDescriptor(description='First coefficient of servo angle equation!')
        self.declare_parameter('angle_equation_c1',[1.0], angle_equation_c1_descriptor)

        angle_equation_c2_descriptor = ParameterDescriptor(description='Second coefficient of servo angle equation!')
        self.declare_parameter('angle_equation_c2',[1.0], angle_equation_c2_descriptor)

        self.servo_ch_list = list(self.get_parameter('servo_ch_list').get_parameter_value().integer_array_value)
        self.servo_topic_list = list(self.get_parameter('servo_topic_list').get_parameter_value().string_array_value)
        # self.angle_hard_min_deg = list(self.get_parameter('angle_hard_min_deg').get_parameter_value().integer_array_value)
        # self.angle_hard_max_deg = list(self.get_parameter('angle_hard_max_deg').get_parameter_value().integer_array_value)
        self.angle_soft_min_deg = list(self.get_parameter('angle_soft_min_deg').get_parameter_value().integer_array_value)
        self.angle_soft_max_deg = list(self.get_parameter('angle_soft_max_deg').get_parameter_value().integer_array_value)
        self.angle_offset = list(self.get_parameter('angle_offset').get_parameter_value().integer_array_value)
        self.angle_equation_c1 = list(self.get_parameter('angle_equation_c1').get_parameter_value().double_array_value)
        self.angle_equation_c2 = list(self.get_parameter('angle_equation_c2').get_parameter_value().double_array_value)

        self.get_logger().info(f'servo_ch_list: {self.servo_ch_list}', once = True)
        self.get_logger().info(f'servo_topic_list: {self.servo_topic_list}', once = True) 
        # self.get_logger().info(f'angle_hard_min_deg: {self.angle_hard_min_deg}', once = True)
        # self.get_logger().info(f'angle_hard_max_deg: {self.angle_hard_max_deg}', once = True)
        self.get_logger().info(f'angle_soft_min_deg: {self.angle_soft_min_deg}', once = True)
        self.get_logger().info(f'angle_soft_max_deg: {self.angle_soft_max_deg}', once = True)
        self.get_logger().info(f'angle_offset {self.angle_offset}', once = True)
        self.get_logger().info(f'angle_equation_c1: {self.angle_equation_c1}', once = True)
        self.get_logger().info(f'angle_equation_c2={self.angle_equation_c2}', once = True)


        # Add pwm parameters to step motor slower/faster

        current_pwm_descriptor = ParameterDescriptor(description='Current PWM values for each servo')
        self.declare_parameter('current_pwm', [0.0] * len(self.servo_topic_list), current_pwm_descriptor)

        desired_pwm_descriptor = ParameterDescriptor(description='Desired PWM values for each servo')
        self.declare_parameter('desired_pwm', [0.0] * len(self.servo_topic_list), desired_pwm_descriptor)

        steps_descriptor = ParameterDescriptor(description='Step size for PWM adjustment')
        self.declare_parameter('steps', 0.1, steps_descriptor)

        timer_interval_descriptor = ParameterDescriptor(description='Interval in seconds between timer callbacks')
        self.declare_parameter('timer_interval', 0.1, timer_interval_descriptor)

        self.current_pwm = list(self.get_parameter('current_pwm').get_parameter_value().double_array_value)
        self.desired_pwm = list(self.get_parameter('desired_pwm').get_parameter_value().double_array_value)
        self.steps = self.get_parameter('steps').get_parameter_value().double_value
        self.timer_interval = self.get_parameter('timer_interval').get_parameter_value().double_value

        self.get_logger().info(f'current_pwm={self.current_pwm}', once = True)
        self.get_logger().info(f'desired_pwm={self.desired_pwm}', once = True)
        self.get_logger().info(f'steps={self.steps}', once = True)
        self.get_logger().info(f'timer_interval={self.timer_interval}', once = True)



    def setup_ros(self):

        self.output_cmd_publishers = {}

        # Creates subscription to topics for each servo where user can publish desired angle in degrees.
        for i, servo_topic in enumerate(self.servo_topic_list):
            self.input_angle_sub = self.create_subscription(
            Float64, 
            f"{servo_topic}_input_angle",
            lambda msg, id=i: self.input_angle_callback(msg, id),
            qos_profile = QoSProfile(depth=10),
        )
            
            # Publishes the output_cmd to servo topic after input_angle_callback is executed.
            output_cmd_pub = self.create_publisher(
                Float64, 
                servo_topic, 
                qos_profile=QoSProfile(depth=10)
            )
            self.output_cmd_publishers[i] = output_cmd_pub




    def input_angle_callback(self, msg, id): 
        print("input_angle_callback started")
        if 0 <= id < len(self.servo_topic_list):
            input_angle = msg.data
            min_limit = self.angle_soft_min_deg[id]
            max_limit = self.angle_soft_max_deg[id]
            ang_offset = self.angle_offset[id]
            if input_angle > max_limit:
                input_angle = max_limit
            if input_angle < min_limit:
                input_angle = min_limit
            m = self.angle_equation_c1[id]  
            b = self.angle_equation_c2[id] 
            output_cmd = m * (input_angle + ang_offset) + b
            self.desired_pwm[id] = output_cmd
        else:
            self.get_logger().error(f"Invalid servo ID: {id}")    


        
    def command_pub_callback(self):
        # Update the PWM values gradually for each servo
        for i in range(len(self.servo_topic_list)):
            if self.current_pwm[i] < self.desired_pwm[i]:
                self.current_pwm[i] += self.steps
                if self.current_pwm[i] > self.desired_pwm[i]:
                    self.current_pwm[i] = self.desired_pwm[i]
            elif self.current_pwm[i] > self.desired_pwm[i]:
                self.current_pwm[i] -= self.steps
                if self.current_pwm[i] < self.desired_pwm[i]:
                    self.current_pwm[i] = self.desired_pwm[i]

            # Publish the current PWM value to the respective servo topic
            self.output_cmd_publishers[i].publish(Float64(data=self.current_pwm[i]))
           # self.get_logger().info(f"Servo {i}: Current PWM = {self.current_pwm[i]}")

                                   
def main(args=None):

    rclpy.init(args=args)
    arm_driver = Arm_Driver()

    try:
        rclpy.spin(arm_driver)     #
    except KeyboardInterrupt:
        print("Node interrupted and shutting down.")
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        arm_driver.destroy_node()  #
        try:
            rclpy.shutdown()        #
        except rclpy._rclpy_pybind11.RCLError:
            # Ignore the error if shutdown has already been called
            pass

if __name__ == '__main__':
    main()  