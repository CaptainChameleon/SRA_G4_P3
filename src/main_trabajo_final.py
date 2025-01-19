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
        self.second_obstacle_dis = None
        self.second_obstacle_theta = None
        self.turned_to_left = True

    def search_for_first_obstacle(self):
        self.log.info("||> SEARCHING FOR FIRST OBSTACLE")
        dis_to_first_obs, _ = self.robot.scan_for_closest_obstacle(search_cone_degrees=160)
        security_dis = 25
        self.robot.move_straight(dis_to_first_obs - security_dis)
        self.robot.rotate_to_avoid_obstacle(security_dis, clockwise=True, safety_theta=10)
        self.robot.scan_side_for_closest_obstacle(clockwise=False, search_cone_degrees=90)
        self.robot.pos = Vector(0, 0)
        self.robot.look_at = Vector(0, 1)
        self.robot.theta = math.pi/2
        self.first_obstacle_pos = Vector(0, security_dis)
        self.log.info("||> FIRST OBSTACLE AT: {}".format(self.first_obstacle_pos))

    def search_for_second_obstacle(self):
        self.log.info("||> SEARCHING FOR SECOND OBSTACLE")
        self.robot.store_position()
        distance_to_first_obs = (self.first_obstacle_pos - self.robot.pos).length
        self.log.info("Distance to first obstacle: {}".format(distance_to_first_obs))
        self.robot.rotate_to_avoid_obstacle(distance_to_first_obs, clockwise=False, safety_theta=15)
        self.robot.run_forever()
        first_black_detection = False
        while self.second_obstacle_pos is None:
            self.robot.update_odometry()
            if not first_black_detection and self.robot.is_detecting_black():
                self.log.info("||> DETECTED BLACK LINE")
                first_black_detection = True
                self.robot.stop()
                self.robot.move_to_stored_position()
                distance_to_first_obs = (self.first_obstacle_pos - self.robot.pos).length
                self.robot.rotate_to_avoid_obstacle(distance_to_first_obs, clockwise=True, safety_theta=10)
                self.turned_to_left = False
                self.robot.run_forever()
            if self.robot.pos.y > ((self.first_obstacle_pos.y + self.robot.wheel_base/3) if self.turned_to_left else (self.first_obstacle_pos.y + self.robot.wheel_base/2)):
                self.log.info("||> PASSED FIRST OBSTACLE")
                self.robot.stop()
                detected_obstacle = self.robot.scan_for_closest_obstacle(search_cone_degrees=145, max_range=40)
                if detected_obstacle:
                    self.log.info("||> DETECTED SECOND OBSTACLE")
                    dis, detection_theta = detected_obstacle
                    self.second_obstacle_dis = dis
                    self.second_obstacle_theta = detection_theta
                    self.second_obstacle_pos = Vector(self.robot.pos.x + dis * math.cos(detection_theta),
                                                      self.robot.pos.y + dis * math.sin(detection_theta))
                    self.log.info("||> SECOND OBSTACLE AT: {}".format(self.second_obstacle_pos))
                    break

    def park_robot(self):

        security_dis = 20
        self.robot.move_straight(self.second_obstacle_dis - security_dis)
        self.robot.theta = 0
        self.second_obstacle_theta = 0
        # self.robot.scan_until_not_detected(self.second_obstacle_dis, clockwise=self.turned_to_left, restore=False)
        self.robot.rotate_to_avoid_obstacle(self.second_obstacle_dis, clockwise=self.turned_to_left, safety_theta=5)
        first_obstacle_dis, first_obstacle_theta = self.robot.scan_side_for_closest_obstacle(clockwise=self.turned_to_left, search_cone_degrees=120)

        if self.turned_to_left:
            theta_diff = abs(self.second_obstacle_theta - first_obstacle_theta)
            if theta_diff <= math.pi:
                self.log.info("Entro aqui 1")
                parking_theta = (self.second_obstacle_theta + first_obstacle_theta) / 2
            else:
                self.log.info("Entro aqui 2")
                lower_theta = min(self.second_obstacle_theta, first_obstacle_theta)
                bigger_theta = max(self.second_obstacle_theta, first_obstacle_theta)
                lower_theta += 2*math.pi
                parking_theta = (lower_theta + bigger_theta)/2 - 2*math.pi
        else:
            self.log.info("Entro aqui 3")
            parking_theta = (self.second_obstacle_theta + first_obstacle_theta) / 2

        self.log.info("First obstacle dis: {}".format(first_obstacle_dis))
        self.log.info("First obstacle theta: {}".format(first_obstacle_theta))
        self.log.info("Second obstacle theta: {}".format(self.second_obstacle_theta))
        self.log.info("Parking theta: {}".format(parking_theta))
        self.log.info("Current theta: {}".format(self.robot.theta))


        self.robot.rotate_to_match(self.robot.normalize_theta(parking_theta))
        self.robot.run_forever()
        self.log.info("Moving to parking spot")
        while not self.robot.is_detecting_black():
            self.robot.update_odometry()
        self.robot.stop()

        # COLOR SENSOR SIDEWAYS
        self.robot.move_straight(-1.5)
        self.robot.scan_side_for_closest_obstacle(clockwise=False, search_cone_degrees=270)

        # COLOR SENSOR UPFRONT
        #self.robot.move_straight(5.5)
        #self.robot.turn_forever()
        #while not self.robot.is_detecting_black():
        #    self.robot.update_odometry()
        #self.robot.stop()

    def move(self):
        self.search_for_first_obstacle()
        self.search_for_second_obstacle()
        self.park_robot()


if __name__ == '__main__':
    robot_controller = ParkingController(log_name="sra_grupo4_trabajo_final", log_level=logging.INFO)
    robot_controller.robot.speed = 10
    robot_controller.run()
