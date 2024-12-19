import math

from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_D, SpeedPercent

from utils import Vector


class WheelPlatform:
    
    def __init__(self, log, wheel_diameter=5.6, wheel_base=11.36, base_speed=20, speed_correction: float = 0.02):
        self.log = log
        self.left_motor = LargeMotor(OUTPUT_A)
        self.right_motor = LargeMotor(OUTPUT_D)
        self.wheel_diameter = wheel_diameter
        self.wheel_base = wheel_base
        self.left_speed = base_speed - speed_correction
        self.right_speed = base_speed + speed_correction
        self.pos = Vector(0, 0)
        self.look_at = Vector(0, 1)
        self.theta = math.pi / 2  # radians

    def tachos_to_distance(self, tachos: int) -> float:
        return (tachos / 360) * math.pi * self.wheel_diameter

    def distance_to_tachos(self, distance: float) -> float:
        return (distance / (math.pi * self.wheel_diameter)) * 360

    def rotation_to_wheel_degrees(self, degrees: float) -> float:
        wheel_travel_distance = (degrees / 360) * math.pi * self.wheel_base
        return self.distance_to_tachos(wheel_travel_distance)

    def reset_motors(self):
        self.log.info("Motor L pos after movement: {}".format(self.left_motor.position))
        self.log.info("Motor R pos after movement: {}".format(self.right_motor.position))
        self.left_motor.reset()
        self.right_motor.reset()
        self.log.info("Motor L pos after reset: {}".format(self.left_motor.position))
        self.log.info("Motor R pos after reset: {}".format(self.right_motor.position))

    def update_odometry(self):
        self.log.info("Updating odometry")

        # Update robot's orientation (theta)
        d_left = self.tachos_to_distance(self.left_motor.position)
        d_right = self.tachos_to_distance(self.right_motor.position)
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
        self.log.info("Updated Orientation: {:.8f} radians".format(self.theta))
        self.log.info("Look-at Vector: {}".format(self.look_at))
        self.reset_motors()

    def move_straight(self, distance: float):
        self.log.info("** Moving forward {}cm **".format(distance))
        degrees_to_turn = self.distance_to_tachos(distance)
        self.log.info("Target Tachos: {}".format(degrees_to_turn))
        self.left_motor.on_for_degrees(degrees=degrees_to_turn, speed=SpeedPercent(self.left_speed), brake=True, block=False)
        self.right_motor.on_for_degrees(degrees=degrees_to_turn, speed=SpeedPercent(self.right_speed), brake=True, block=True)
        self.update_odometry()
        self.log.info("\n")

    def turn_degrees(self, degrees: float, speed=20):
        self.log.info("** Rotating {}deg **".format(degrees))
        wheel_degrees = self.rotation_to_wheel_degrees(degrees)
        self.log.info("Target Tachos: {}".format(wheel_degrees))
        self.left_motor.on_for_degrees(degrees=-wheel_degrees, speed=SpeedPercent(self.left_speed), brake=True, block=False)
        self.right_motor.on_for_degrees(degrees=wheel_degrees, speed=SpeedPercent(self.right_speed), brake=True, block=True)
        self.update_odometry()
        self.log.info("\n")

    def run_forever(self):
        self.log.info("Running {}".format(SpeedPercent(self.left_speed)))
        self.left_motor.speed_sp = int(self.left_speed/100 * self.left_motor.max_speed)
        self.right_motor.speed_sp = int(self.right_speed/100 * self.right_motor.max_speed)
        self.left_motor.run_forever()
        self.right_motor.run_forever()

    def turn_forever(self, clockwise: bool = False):
        self.log.info("Running {} rotation".format("clockwise" if clockwise else "counter-clockwise"))
        self.left_motor.speed_sp = int(self.left_speed/100 * self.left_motor.max_speed)
        self.right_motor.speed_sp = int(self.right_speed/100 * self.right_motor.max_speed)
        self.left_motor.speed_sp = self.left_motor.speed_sp if clockwise else -self.left_motor.speed_sp
        self.right_motor.speed_sp = -self.right_motor.speed_sp if clockwise else self.right_motor.speed_sp
        self.left_motor.run_forever()
        self.right_motor.run_forever()

    def stop(self):
        self.left_motor.stop()
        self.right_motor.stop()
        self.update_odometry()
