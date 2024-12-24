import math

from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_D, SpeedPercent
from ev3dev2.sensor.lego import UltrasonicSensor
from ev3dev2.sound import Sound

from utils import Vector


class Robot:

    MAX_RANGE_DIS = 20  # centimeters

    def __init__(self, log,
                 wheel_diameter: float, wheel_base: float,
                 base_speed: float, speed_correction: float,
                 initial_theta: float):
        self.log = log
        self.sound = Sound()
        self.ultrasonic_sensor = UltrasonicSensor()
        self.left_motor = LargeMotor(OUTPUT_A)
        self.right_motor = LargeMotor(OUTPUT_D)
        self.wheel_diameter = wheel_diameter
        self.wheel_base = wheel_base
        self.base_speed = base_speed
        self.speed_correction = speed_correction
        self.base_left_speed = base_speed * speed_correction
        self.base_right_speed = base_speed / speed_correction
        self.pos = Vector(0, 0)
        self.look_at = Vector(0, 1)
        self.left_motor_dis = 0.0
        self.right_motor_dis = 0.0
        self.theta = math.radians(initial_theta)

    def beep(self, n=1):
        for _ in range(n):
            self.sound.beep()

    def tachos_to_distance(self, tachos: int) -> float:
        return (tachos / 360) * math.pi * self.wheel_diameter

    def distance_to_tachos(self, distance: float) -> float:
        return (distance / (math.pi * self.wheel_diameter)) * 360

    def rotation_to_wheel_degrees(self, degrees: float) -> float:
        wheel_travel_distance = (degrees / 360) * math.pi * self.wheel_base
        return self.distance_to_tachos(wheel_travel_distance)

    def reset_motors(self):
        self.log.debug("Motor L pos after movement: {}".format(self.left_motor.position))
        self.log.debug("Motor R pos after movement: {}".format(self.right_motor.position))
        self.left_motor.reset()
        self.right_motor.reset()
        self.left_motor_dis = 0
        self.right_motor_dis = 0
        self.log.debug("Motor L pos after reset: {}".format(self.left_motor.position))
        self.log.debug("Motor R pos after reset: {}".format(self.right_motor.position))

    def update_odometry(self):
        self.log.info("Updating odometry")

        # Update robot's orientation (theta)
        current_left_motor_dis = self.tachos_to_distance(self.left_motor.position)
        current_right_motor_dis = self.tachos_to_distance(self.right_motor.position)
        d_left = current_left_motor_dis - self.left_motor_dis
        d_right = current_right_motor_dis - self.right_motor_dis
        self.left_motor_dis = current_left_motor_dis
        self.right_motor_dis = current_right_motor_dis

        d_center = (d_left + d_right) / 2
        delta_theta = (d_right - d_left) / self.wheel_base
        self.theta += delta_theta
        self.theta %= (2 * math.pi)  # Keep theta within [0, 2*pi]

        # Update position
        self.pos.x += d_center * math.sin(self.theta)
        self.pos.y += d_center * math.cos(self.theta)

        # Update look_at vector
        self.look_at.x = math.sin(self.theta)
        self.look_at.y = math.cos(self.theta)

        # Log updated values
        self.log.info("Updated Position: {}".format(self.pos))
        self.log.info("Updated Orientation: {:.4f} radians".format(self.theta))
        self.log.info("Look-at Vector: {}".format(self.look_at))

    def move_straight(self, distance: float):
        self.log.info("||> MOVING {} {:.4f} cm".format("FORWARD" if distance > 0 else "BACKWARD", distance))
        degrees_to_turn = self.distance_to_tachos(distance)
        self.log.info("Target Tachos: {:.4f}".format(degrees_to_turn))
        self.left_motor.on_for_degrees(degrees=degrees_to_turn, speed=SpeedPercent(self.base_left_speed), brake=True,
                                       block=False)
        self.right_motor.on_for_degrees(degrees=degrees_to_turn, speed=SpeedPercent(self.base_right_speed), brake=True,
                                        block=True)
        self.update_odometry()
        self.reset_motors()
        self.log.info("\n")

    def turn_degrees(self, degrees: float, speed=20):
        self.log.info("||> ROTATING {:.4f} deg...".format(degrees))
        wheel_degrees = self.rotation_to_wheel_degrees(degrees)
        self.log.info("Target Tachos: {:.4f}".format(wheel_degrees))
        self.left_motor.on_for_degrees(degrees=-wheel_degrees, speed=SpeedPercent(self.base_left_speed), brake=True,
                                       block=False)
        self.right_motor.on_for_degrees(degrees=wheel_degrees, speed=SpeedPercent(self.base_right_speed), brake=True,
                                        block=True)
        self.update_odometry()
        self.reset_motors()
        self.log.info("\n")

    def run_forever(self):
        self.log.info("||> MOVING IN A STRAIGHT LINE...")
        self.left_motor.speed_sp = int(self.base_left_speed / 100 * self.left_motor.max_speed)
        self.right_motor.speed_sp = int(self.base_right_speed / 100 * self.right_motor.max_speed)
        self.left_motor.run_forever()
        self.right_motor.run_forever()

    def turn_forever(self, center_of_rotation: Vector = None, clockwise: bool = False):
        self.log.info(
            "||> ROTATING {}...".format("AROUND {}".format(center_of_rotation) if center_of_rotation else "IN PLACE")
        )
        if center_of_rotation:
            distance_to_center = len(center_of_rotation - self.pos)
            robot_cor_angle = math.degrees(
                math.acos(center_of_rotation.dot(self.look_at) / len(center_of_rotation) * len(self.look_at))
            )
            if robot_cor_angle != 90:
                self.turn_degrees(90 - robot_cor_angle)
            left_radius = distance_to_center - (self.wheel_base / 2)
            right_radius = distance_to_center + (self.wheel_base / 2)
            speed_ratio = right_radius / left_radius
            self.left_motor.speed_sp = int(self.base_speed * self.speed_correction)
            self.right_motor.speed_sp = int(self.base_speed * speed_ratio / self.speed_correction)
            self.left_motor.speed_sp = self.left_motor.speed_sp if clockwise else -self.left_motor.speed_sp
            self.right_motor.speed_sp = -self.right_motor.speed_sp if clockwise else self.right_motor.speed_sp
        else:
            self.left_motor.speed_sp = int(self.base_left_speed / 100 * self.left_motor.max_speed)
            self.right_motor.speed_sp = int(self.base_right_speed / 100 * self.right_motor.max_speed)
            self.left_motor.speed_sp = self.left_motor.speed_sp if clockwise else -self.left_motor.speed_sp
            self.right_motor.speed_sp = -self.right_motor.speed_sp if clockwise else self.right_motor.speed_sp
        self.left_motor.run_forever()
        self.right_motor.run_forever()

    def stop(self):
        self.log.info("\n")
        self.log.info("||> STOP MOTORS".format(SpeedPercent(self.base_left_speed)))
        self.left_motor.stop()
        self.right_motor.stop()
        self.update_odometry()
        self.reset_motors()
        self.log.info("\n")
