#!/usr/bin/env python3
import logging
import math
import time

from robot import RobotController
from utils import Vector


class ParkingController(RobotController):
    
    def __init__(self, log_name: str, log_level: int = logging.DEBUG):
        super().__init__(log_name, log_level)
        self.obstacle_pos_1 = None
        self.obstacle_pos_2 = None
        self.initial_theta = None

    def search_for_first_obstacle(self):
        self.initial_theta = self.robot.theta
        search_cone_angle = math.radians(60)
        search_max_theta = self.initial_theta + search_cone_angle
        search_min_theta = self.initial_theta - search_cone_angle

        self.log.info("||> SEARCHING FOR FIRST OBSTACLE [SEARCH CONE: {:.4f} deg]".format(
            2 * math.degrees(search_cone_angle))
        )
        self.robot.turn_degrees(math.degrees(search_cone_angle))
        min_dis_angle = self.robot.theta
        min_dis = self.robot.ultrasonic_sensor.distance_centimeters
        self.log.debug("Initial distance: {:.4f} {:.4f}".format(min_dis, min_dis_angle))

        self.robot.turn_forever(clockwise=True)
        while 0.7*search_min_theta <= self.robot.theta <= 1.3*search_max_theta:
            self.robot.update_odometry()
            self.log.debug("\nCurrent theta: {:.4f}  Target theta {:.4f}".format(
                math.degrees(self.robot.theta), math.degrees(search_min_theta))
            )
            current_dis = self.robot.ultrasonic_sensor.distance_centimeters
            current_theta = self.robot.theta
            self.log.debug("Current obs dis: {:.2f}\n".format(current_dis))
            if current_dis < min_dis:
                min_dis = current_dis
                min_dis_angle = current_theta
        self.robot.stop()

        self.log.info("Detected closest obstacle at {:.2f} cm and {:.4f} deg ".format(
            min_dis, math.degrees(min_dis_angle))
        )
        self.log.debug("Robot angle: {:.2f} deg".format(math.degrees(self.robot.theta)))
        self.robot.turn_degrees(math.degrees(min_dis_angle - self.robot.theta))
        # self.obstacle_pos_1 = self.scan_obstacle(min_dis)
        self.obstacle_pos_1 = Vector(0, min_dis)
        self.robot.pos = Vector(0, 0)
        self.robot.look_at = Vector(0, 1)
        self.robot.theta = math.pi / 2

    def search_for_second_obstacle(self):
        # distance_to_center = (self.obstacle_pos_1 - self.robot.pos).length
        # security_dis = 15
        # self.robot.move_straight(distance_to_center - security_dis)
        # self.robot.turn_forever(center_of_rotation=self.obstacle_pos_1, clockwise=True)
        # time.sleep(20)
        sec_dis = 20
        target_scan_pos = Vector(sec_dis, self.obstacle_pos_1.y)
        target_scan_angle = self.robot.pos.angle_with(target_scan_pos)
        self.robot.turn_degrees(-target_scan_angle)
        self.robot.move_straight(math.sqrt(sec_dis ** 2 + self.obstacle_pos_1.y ** 2))
        self.robot.turn_degrees(target_scan_angle + math.pi / 2)
        right_cone_limit = self._scan_until_not_detected(self.robot.theta, sec_dis, clockwise=True, restore=False)

    def xd(self):
        sign = self.robot.theta - self.initial_theta

        # Determina el sentido del giro
        clockwise = sign >= 0

        security_dis = 20
        min_dis = 15  # Distancia mínima segura
        target_distance = 30  # Distancia objetivo para iniciar ajustes
        max_rotation_speed = 50  # Velocidad máxima de rotación
        max_rotation_angle = 45  # Máximos grados de giro por iteración

        while True:
            self.robot.update_odometry()
            current_dis = self.robot.ultrasonic_sensor.distance_centimeters

            if current_dis > target_distance:
                self.robot.move_straight((current_dis - target_distance) + 1)

            # Si el obstáculo sigue en rango, ajusta el giro
            if current_dis <= target_distance:
                rotation_factor = max(0.2, (target_distance - current_dis) / target_distance)
                rotation_angle = int(max_rotation_angle * rotation_factor)

                # Aumentar grados sin perder el obstáculo de vista
                if clockwise:
                    self.robot.left_motor.speed_sp = rotation_angle
                    self.robot.right_motor.speed_sp = -rotation_angle
                else:
                    self.robot.left_motor.speed_sp = -rotation_angle
                    self.robot.right_motor.speed_sp = rotation_angle

                self.log.debug("Rotating with angle: {} degrees (factor: {:.2f})".format(rotation_angle, rotation_factor))
                self.robot.left_motor.run_forever()
                self.robot.right_motor.run_forever()

            # Detiene el movimiento si se alcanza la distancia mínima segura
            if current_dis < min_dis:
                self.robot.stop()
                self.log.info("Stopped at distance: {:.2f} cm".format(current_dis))
                break

    def _scan_until_not_detected(self, initial_theta: float, obstacle_dis: float, clockwise: bool, restore: bool = True) -> float:
        current_dis = obstacle_dis

        # Scan until obstacle is no longer detected
        self.robot.turn_forever(clockwise=clockwise)
        while current_dis <= obstacle_dis * 1.1:
            current_dis = self.robot.ultrasonic_sensor.distance_centimeters
            self.log.debug(
                "Scanning obstacle to the left: {} cm [Initial distance: {}]".format(current_dis, obstacle_dis))
        self.robot.stop()
        theta_limit = self.robot.theta

        self.log.debug("Found obstacle, turned {}".format(theta_limit))
        # Restore theta & return
        if restore:
            self.log.info("Turning back to initial theta...")
            self.robot.turn_degrees(math.degrees(initial_theta - theta_limit))
        return theta_limit

    def scan_obstacle(self, obstacle_dis) -> Vector:
        initial_theta = self.robot.theta
        mediatriz_izq = self._scan_until_not_detected(initial_theta, obstacle_dis, clockwise=False)
        mediatriz_der = self._scan_until_not_detected(initial_theta, obstacle_dis, clockwise=True)
        self.robot.pos = Vector(0, 0)
        beta = mediatriz_izq - (mediatriz_izq - mediatriz_der) / 2
        obstacle_pos = Vector(obstacle_dis * math.cos(beta), obstacle_dis * math.sin(beta))
        self.log.info("Found obstacle at {}".format(obstacle_pos))
        return obstacle_pos

    def move(self):
        self.search_for_first_obstacle()
        self.search_for_second_obstacle()


if __name__ == '__main__':
    robot_controller = ParkingController(log_name="sra_grupo4_trabajo_final", log_level=logging.DEBUG)
    robot_controller.robot.base_left_speed = 10 * 1.0023746690616273
    robot_controller.robot.base_right_speed = 10 * 0.9976309566323637
    robot_controller.run()
