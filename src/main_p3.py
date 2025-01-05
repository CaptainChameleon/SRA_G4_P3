#!/usr/bin/env python3

import time

from robot import RobotController


class SquareTestController(RobotController):

    def wait(self, duration: int = 5):
        self.robot.beep(2)
        time.sleep(duration)

    def make_square(self, side_len: float = 50, clockwise: bool = True):
        degrees = -90 if clockwise else 90
        for _ in range(4):
            self.robot.move_straight(side_len)
            self.robot.turn_degrees(degrees)

    def bidirectional_square_test(self, square_size=50, wait_time=10):
        self.make_square(side_len=square_size)
        self.wait(duration=wait_time)
        self.make_square(side_len=square_size, clockwise=False)

    def ultrasonic_test(self, stop_distance: float = 10):
        # MOVE UNTIL DISTANCE IS REACHED TEST
        self.robot.run_forever()
        while self.robot.ultrasonic_sensor.distance_centimeters > stop_distance:
            self.log.info("Detected obstacle at {} cm".format(self.robot.ultrasonic_sensor.distance_centimeters))
        self.robot.stop()
        self.log.info("Stopped at {} cm from obstacle".format(self.robot.ultrasonic_sensor.distance_centimeters))

        # MOVE TO CERTAIN DISTANCE TEST
        # distance_to_obs = self.robot.ultrasonic_sensor.distance_centimeters
        # self.log.info("\n||> ULTRASONIC TEST: Obstacle at {} cm...".format(distance_to_obs))
        # self.robot.move_straight(distance_to_obs - stop_distance)
        # self.log.info("Stopped at {} cm from obstacle".format(self.robot.ultrasonic_sensor.distance_centimeters))

        # SNAPSHOT TEST
        # self.robot.beep(n_times=3, delay=2)
        # self.robot.beep(n_times=2)
        # self.log.info("Detected obstacle at {} cm".format(self.robot.ultrasonic_sensor.distance_centimeters))

    def gyroscope_test(self, degrees: float = 90):
        self.robot.gyroscope.reset()
        initial_theta = self.robot.gyroscope.angle
        self.robot.turn_degrees(degrees)
        self.log.info("Rotated from {} deg to {} deg".format(initial_theta, self.robot.gyroscope.angle))

    def move(self):
        # A) Make a square
        # self.robot.move_straight(50)
        # B)
        # for _ in range(10):
        #     self.bidirectional_square_test(wait_time=20)
        # C) Determinar si existe error sistemático.
        # D) Forzar la aparición de error sistemático y comprobar si el test aplica las correcciones adecuadas.
        # E) Construir una plataforma diferencial con una geometría diferente y comparar y justificar los resultados.
        self.make_square(clockwise=True)

        # F) Caracterización del error de otros sensores: ultrasonidos, orientación
        self.ultrasonic_test(stop_distance=10)
        self.gyroscope_test(degrees=90)


if __name__ == '__main__':
    robot_controller = SquareTestController(log_name="sra_grupo4_p3")
    robot_controller.run()
