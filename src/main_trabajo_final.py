#!/usr/bin/env python3
import logging
import math

from robot import RobotController
from utils import Vector


class ParkingController(RobotController):
    
    def __init__(self, log_name: str, log_level: int = logging.DEBUG):
        super().__init__(log_name, log_level)
        self.obstacle_pos_1 = None
        self.obstacle_pos_2 = None
    
    def search_for_first_obstacle(self):
        initial_theta = self.robot.theta
        search_cone_angle = math.radians(60)
        search_max_theta = initial_theta + search_cone_angle
        search_min_theta = initial_theta - search_cone_angle

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
        self.obstacle_pos_1 = self.scan_obstacle(min_dis)

    def search_for_second_obstacle(self):
        self.robot.turn_forever(center_of_rotation=self.obstacle_pos_1, clockwise=True)

    def scan_obstacle(self, obstacle_dis) -> Vector:
        initial_theta = self.robot.theta
        current_dis = obstacle_dis
        self.robot.turn_forever()
        while current_dis <= obstacle_dis * 1.1:
            current_dis = self.robot.ultrasonic_sensor.distance_centimeters
            self.log.debug("Scanning obstacle to the left: {} cm [Initial distance: {}]".format(current_dis, obstacle_dis))
        self.robot.stop()
        mediatriz_izq = self.robot.theta
        self.robot.turn_degrees(math.degrees(-(mediatriz_izq-initial_theta)))
        self.log.debug("Found obstacle, turned {} and turning back to initial theta".format(mediatriz_izq))

        current_dis = obstacle_dis
        self.robot.turn_forever(clockwise=True)
        while current_dis <= obstacle_dis * 1.1:
            self.log.debug("Scanning obstacle to the right: {} cm [Initial distance: {}]".format(current_dis, obstacle_dis))
            current_dis = self.robot.ultrasonic_sensor.distance_centimeters
            self.log.debug("Scanning obstacle to the right: {} cm [Initial distance: {}]".format(current_dis, obstacle_dis))
        self.robot.stop()
        mediatriz_der = self.robot.theta
        self.robot.turn_degrees(math.degrees(initial_theta-mediatriz_der))
        self.robot.pos = Vector(0, 0)

        beta = mediatriz_izq - (mediatriz_izq - mediatriz_der) / 2
        obstacle_pos = Vector(obstacle_dis * math.cos(beta), obstacle_dis * math.sin(beta))
        self.log.info("Found obstacle at {}".format(obstacle_pos))
        return obstacle_pos

    def move(self):
        self.search_for_first_obstacle()
        self.search_for_second_obstacle()


if __name__ == '__main__':
    robot_controller = ParkingController(log_name="sra_grupo4_trabajo_final")
    robot_controller.run()
