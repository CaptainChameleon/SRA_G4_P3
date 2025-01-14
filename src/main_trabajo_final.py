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
        self.black_ribbon_dir = None
        self.parking_pos = None
        self.second_obstacle_dis = None
        self.second_obstacle_theta = None
        self.turned_to_left = True

    def search_for_first_obstacle(self):
        self.log.info("||> SEARCHING FOR FIRST OBSTACLE")
        dis_to_first_obs, _ = self.robot.scan_for_closest_obstacle(restore=False)
        self.robot.pos = Vector(0, 0)
        self.robot.look_at = Vector(0, 1)
        self.robot.theta = math.pi/2
        self.first_obstacle_pos = Vector(0, dis_to_first_obs)
        self.log.info("||> FIRST OBSTACLE AT: {}".format(self.first_obstacle_pos))
        #self.first_obstacle_pos = self.robot.get_obstacle_pos()

    def search_for_second_obstacle(self):
        self.log.info("||> SEARCHING FOR SECOND OBSTACLE")
        # security_dis = 20
        # self.robot.move_straight((self.first_obstacle_pos - self.robot.pos).length - security_dis)
        self.robot.store_position()
        distance_to_first_obs = (self.first_obstacle_pos - self.robot.pos).length
        self.robot.rotate_to_avoid_obstacle(distance_to_first_obs, clockwise=False)
        self.robot.run_forever()
        first_black_detection = False
        while self.second_obstacle_pos is None:
            self.robot.update_odometry()
            if not first_black_detection and self.robot.is_detecting_black():
                self.log.info("||> DETECTED BLACK LINE")
                first_black_detection = True
                self.robot.stop()
                self.robot.move_to_stored_position()
                # dis_to_first_obs, _ = self.robot.scan_for_closest_obstacle(search_cone_degrees=90, restore=False)
                # if not self.robot.is_detecting_obstacle_within(dis_to_first_obs):
                #     self.robot.look_towards(self.first_obstacle_pos)
                self.robot.rotate_to_avoid_obstacle(distance_to_first_obs, clockwise=True)
                self.turned_to_left = False
                self.robot.run_forever()
            if self.robot.pos.y > self.first_obstacle_pos.y + 10:
                self.log.info("||> PASSED FIRST OBSTACLE")
                self.robot.stop()
                detected_obstacle = self.robot.scan_for_closest_obstacle(search_cone_degrees=120, max_range=40)
                if detected_obstacle:
                    self.log.info("||> DETECTED SECOND OBSTACLE")
                    dis, detection_theta = detected_obstacle
                    self.second_obstacle_dis = dis
                    self.robot.rotate_to_match(detection_theta)
                    self.second_obstacle_pos = Vector(self.robot.pos.x + dis*math.cos(detection_theta), self.robot.pos.y + dis*math.sin(detection_theta))
                    self.log.info("||> SECOND OBSTACLE AT: {}".format(self.second_obstacle_pos))
                    #self.second_obstacle_pos = self.robot.get_obstacle_pos()
                    self.black_ribbon_dir = self.second_obstacle_pos - self.first_obstacle_pos
                    break

    def park_robot(self):
        self.parking_pos = Vector.middle_of(self.second_obstacle_pos, self.first_obstacle_pos)
        self.log.info("||> PARKING AT: {}".format(self.parking_pos))
        self.log.info("First obs: {}".format(self.first_obstacle_pos))
        self.log.info("Second obs: {}".format(self.second_obstacle_pos))
        self.log.info("Black ribbon dir: {}".format(self.black_ribbon_dir))
        self.log.info("Robot pos: {}".format(self.robot.pos))
        self.log.info("Looking at: {}".format(self.robot.look_at))

        second_obstacle_theta = 0
        self.robot.theta = 0
        self.robot.scan_until_not_detected(self.second_obstacle_dis, clockwise=self.turned_to_left, restore=False)
        first_obstacle_dis, first_obstacle_theta = self.robot.scan_for_second_closest_obstacle(clockwise=self.turned_to_left, search_cone_degrees=180)
        parking_theta = first_obstacle_theta / 2
        self.robot.rotate_to_match(parking_theta)
        self.robot.run_forever()
        while not self.robot.is_detecting_black():
            self.robot.update_odometry()
        self.robot.stop()

        



        #self.robot.look_towards(self.second_obstacle_pos)
        #self.robot.move_straight((self.parking_pos - self.robot.pos).length + self.robot.wheel_base / 2)
        #self.robot.turn_forever()
        #while not self.robot.is_detecting_black():
        #    self.robot.update_odometry()
        #self.robot.stop()
        #self.robot.turn_degrees(90)

        # self.robot.turn_degrees(self.robot.look_at.angle_with(self.black_ribbon_dir, in_degrees=True))
        # robot_to_parking = self.parking_pos - self.robot.pos
        # angle_with_parking_pos = 180 - 90 - self.robot.look_at.angle_with(robot_to_parking)
        # self.robot.move_straight(robot_to_parking.length * math.sin(angle_with_parking_pos))
        # self.robot.turn_degrees(self.robot.look_at.angle_with(self.parking_pos, in_degrees=True))
        # self.robot.run_forever()
        # while True:
        #     self.robot.update_odometry()
        #     if self.robot.is_detecting_black():
        #         self.robot.stop()
        #         self.log.info("Angle with first obstacle {}".format(self.robot.look_at.angle_with(self.robot.pos - self.first_obstacle_pos)))
        #         break
        # #self.robot.turn_degrees(90)
        # self.log.info("||> APARCAO ;^]")


    def move(self):
        self.search_for_first_obstacle()
        self.search_for_second_obstacle()
        self.park_robot()


if __name__ == '__main__':
    robot_controller = ParkingController(log_name="sra_grupo4_trabajo_final", log_level=logging.INFO)
    robot_controller.robot.speed = 10
    robot_controller.run()
