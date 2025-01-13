#!/usr/bin/env python3
import logging
import math
import time

from robot import RobotController
from utils import Vector


class ParkingController(RobotController):
    
    def __init__(self, log_name: str, log_level: int = logging.DEBUG):
        super().__init__(log_name, log_level)
        self.first_obstacle_pos = None
        self.second_obstacle_pos = None
        self.color_band_dir = None
        self.parking_pos = None

    def search_for_first_obstacle(self):
        self.log.info("||> SEARCHING FOR FIRST OBSTACLE")
        min_dis, min_dis_angle = self.robot.scan_for_closest_obstacle()
        self.robot.rotate_to_match(min_dis_angle)
        self.robot.pos = Vector(0, 0)
        self.first_obstacle_pos = self.robot.get_obstacle_pos()

    def search_for_second_obstacle(self):
        self.log.info("||> SEARCHING FOR SECOND OBSTACLE")

        # Position robot
        self.robot.approach_obstacle(security_distance=20)
        self.robot.store_position()
        self.robot.rotate_to_avoid_obstacle(clockwise=False)

        # Look for second obstacle
        self.robot.run_forever()
        while self.second_obstacle_pos is None:
            self.robot.update_odometry()
            if self.robot.pos.y > self.first_obstacle_pos.y:
                self.robot.stop()
                detected_obstacle = self.robot.scan_for_closest_obstacle(search_cone_degrees=120, max_range=40)
                if detected_obstacle:
                    dis, detection_theta = detected_obstacle
                    self.robot.rotate_to_match(detection_theta)
                    self.second_obstacle_pos = self.robot.get_obstacle_pos()
                    self.color_band_dir = self.second_obstacle_pos - self.first_obstacle_pos
            if self.robot.is_detecting_black():
                self.robot.stop()
                self.robot.move_to_stored_position()
                self.robot.rotate_to_avoid_obstacle(clockwise=True)
                self.robot.run_forever()
        self.robot.stop()

    def park_robot(self):
        self.parking_pos = Vector.middle_of(self.second_obstacle_pos, self.first_obstacle_pos)
        self.log.info("||> PARKING AT: {}".format(self.parking_pos))
        self.robot.turn_degrees(self.robot.look_at.angle_with(self.color_band_dir, in_degrees=True))
        robot_to_parking = self.parking_pos - self.robot.pos
        angle_with_parking_pos = 180 - 90 - robot_to_parking.angle_with(self.robot.look_at)
        self.robot.move_straight(robot_to_parking.length * math.sin(angle_with_parking_pos))
        self.robot.turn_degrees(self.robot.look_at.angle_with(self.parking_pos, in_degrees=True))
        self.robot.run_forever()
        while True:
            self.robot.update_odometry()
            if self.robot.is_detecting_black():
                self.robot.stop()
                break
        self.robot.turn_degrees(90)
        self.log.info("APARCAO ;^)")

    def xd(self):
        sign = self.robot.theta - self.robot.stored_theta

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

    def move(self):
        self.search_for_first_obstacle()
        self.search_for_second_obstacle()
        self.park_robot()


if __name__ == '__main__':
    robot_controller = ParkingController(log_name="sra_grupo4_trabajo_final", log_level=logging.DEBUG)
    robot_controller.robot.base_left_speed = 10 * 1.0023746690616273
    robot_controller.robot.base_right_speed = 10 * 0.9976309566323637
    robot_controller.run()
