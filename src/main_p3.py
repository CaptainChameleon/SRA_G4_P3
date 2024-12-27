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

    def move(self):
        # A)
        self.make_square()

        # B)
        # for _ in range(10):
        #     self.bidirectional_square_test(wait_time=20)

        # C) Determinar si existe error sistemático.
        # D) Forzar la aparición de error sistemático y comprobar si el test aplica las correcciones adecuadas.
        # E) Construir una plataforma diferencial con una geometría diferente y comparar y justificar los resultados.
        # F) Caracterización del error de otros sensores: ultrasonidos, orientación


if __name__ == '__main__':
    robot_controller = SquareTestController(log_name="sra_grupo4_p3")
    robot_controller.run()
