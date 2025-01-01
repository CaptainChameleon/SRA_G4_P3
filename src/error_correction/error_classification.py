import configparser
import csv
import math
import os
import statistics


class OdometricErrorClassifier:
    """ Clase para clasificar errores sistemáticos en el comportamiento odométrico de un robot. """
    
    WHEEL_DIAMETER = 5.6
    INITIAL_WHEEL_BASE = 13.5
    BASE_SPEED = 20

    def __init__(self, square_side_length):
        """
        Inicializa con las mediciones en direcciones horaria y antihoraria y la longitud del lado del cuadrado.

        Args:
        clockwise_distance (float): Distancia promedio desde el punto inicial en dirección horaria.
        counter_clockwise_distance (float): Distancia promedio desde el punto inicial en dirección antihoraria.
        square_side_length (float): Longitud de cada lado del cuadrado en centímetros.
        """
        self.side_length = square_side_length
        self.meassures = self.load_meassures()
        
    @staticmethod
    def load_meassures():
        with open(os.path.join(os.path.dirname(__file__), "square_test.csv")) as meassurements:
            data = list(csv.reader(meassurements))[1:]
            return [(float(cw_meassure), float(ccw_meassure)) for cw_meassure, ccw_meassure in data]

    def calculate_type_a(self, cw_dist, ccw_dist):
        """ Evalúa el error tipo A relacionado con la base incierta de distancia. """
        error_present = False
        angular_discrepancy = 0
        error_ratio = 0
        if (cw_dist < 0 and ccw_dist < 0) or (cw_dist > 0 and ccw_dist > 0):
            angular_discrepancy = ((cw_dist + ccw_dist) / (-4 * self.side_length)) * (180 / math.pi)
            if angular_discrepancy != 0:
                error_ratio = 90 / (90 - angular_discrepancy)
                error_present = True
        return error_present, angular_discrepancy, error_ratio

    # def calculate_type_a_v2(self, cw_dist, ccw_dist):
    #     """ Evalúa el error tipo A relacionado con la base incierta de distancia. """
    #     error_present = False
    #     angular_discrepancy = 0
    #     error_ratio = 0
    #     if cw_dist > 0 and ccw_dist > 0:
    #         angular_discrepancy = ((cw_dist + ccw_dist) / (-4 * self.side_length)) * (180 / math.pi)
    #         if angular_discrepancy < 0:
    #             error_ratio = 90 / (90 - angular_discrepancy)
    #             error_present = True
    #     return error_present, angular_discrepancy, error_ratio

    def calculate_type_b(self, cw_dist, ccw_dist):
        """ Evalúa el error tipo B causado por diámetros de rueda desiguales. """
        error_present = False
        angular_discrepancy = 0
        error_ratio = 0
        if (cw_dist < 0 < ccw_dist) or (cw_dist > 0 > ccw_dist):
            angular_diff = cw_dist - ccw_dist if cw_dist < 0 < ccw_dist else ccw_dist - cw_dist
            angular_discrepancy = (angular_diff/(-4 * self.side_length)) * (180 / math.pi)
            if angular_discrepancy != 0:
                semi_diameter = (self.side_length / 2) / math.sin(math.radians(angular_discrepancy / 2))
                error_ratio = (semi_diameter + angular_discrepancy / 2) / (semi_diameter - angular_discrepancy / 2)
                error_present = True
        return error_present, angular_discrepancy, error_ratio

    def report_errors_from_last_meassure(self):
        """ Determina y reporta los tipos de error encontrados en los movimientos del robot. """
        cw_dist, ccw_dist = self.meassures[-1]
        a_present, alpha, eb = self.calculate_type_a(cw_dist, ccw_dist)
        b_present, beta, ed = self.calculate_type_b(cw_dist, ccw_dist)

        print('==================================')
        print('Informe de Análisis de Errores Odometricos')
        print('==================================')
        print('Análisis de Error Tipo A:')
        print(f'  Discrepancia Angular (Alpha): {alpha:.4f} grados')
        print(f'  Magnitud del Error (EB): {eb:.4f}')
        print(f'  Presencia: {"Detectado" if a_present else "No detectado"}')
        print('----------------------------------')
        print('Análisis de Error Tipo B:')
        print(f'  Discrepancia Angular (Beta): {beta:.4f} grados')
        print(f'  Magnitud del Error (ED): {ed:.4f}')
        print(f'  Presencia: {"Detectado" if b_present else "No detectado"}')
        print('==================================\n')
        
    def generate_config(self):
        wheel_diameter = self.WHEEL_DIAMETER
        wheel_base = self.INITIAL_WHEEL_BASE
        base_left_speed = self.BASE_SPEED
        base_right_speed = self.BASE_SPEED
        print('==================================')
        print('Generacion de configuracion')
        print('==================================')
        for meassure_num, meassure in enumerate(self.meassures):
            cw_meassure, ccw_meassure = meassure
            type_a_is_present, type_a_ang_discrepancy, type_a_ratio = self.calculate_type_a(cw_meassure, ccw_meassure)
            type_b_is_present, type_b_ang_discrepancy, type_b_ratio = self.calculate_type_b(cw_meassure, ccw_meassure)
            if type_a_is_present:
                wheel_base *= type_a_ratio
                print('----------------------------------')
                print(f'Meassure nº{meassure_num+1}: Corrected wheel base to {wheel_base:.4f} from {wheel_base/type_a_ratio:.4f} with ratio {type_a_ratio:.4f}')
            if type_b_is_present:
                base_left_speed *= type_b_ratio
                base_right_speed /= type_b_ratio
                print('----------------------------------')
                print(f'Meassure nº{meassure_num + 1}: Corrected left_speed to {base_left_speed:.4f} from {base_left_speed / type_b_ratio:.4f} with ratio {type_b_ratio}:.4f')
        print('==================================')
        config = configparser.ConfigParser()
        config.read("../../config.ini")
        config["Base"]["wheel_diameter"] = str(wheel_diameter)
        config["Base"]["wheel_base"] = str(wheel_base)
        config["Base"]["base_left_speed"] = str(base_left_speed)
        config["Base"]["base_right_speed"] = str(base_right_speed)
        with open("../../config.ini", "w") as configfile:
            config.write(configfile)


if __name__ == '__main__':
    error_classifier = OdometricErrorClassifier(square_side_length=50)
    error_classifier.report_errors_from_last_meassure()
    error_classifier.generate_config()
