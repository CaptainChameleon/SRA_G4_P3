import configparser
import csv
import json
import math
import os


class OdometricErrorClassifier:
    """ Clase para clasificar errores sistemáticos en el comportamiento odométrico de un robot. """
    
    WHEEL_DIAMETER = 5.6
    INITIAL_WHEEL_BASE = 13.5
    BASE_SPEED = 20

    def __init__(self, path_to_measures: str):
        measure_data = self.load_measure_data(path_to_measures)
        self.square_length = measure_data["square_length"]
        self.wheel_diameter = measure_data["wheel_diameter"]
        self.initial_wheel_base = measure_data["initial_wheel_base"]
        self.initial_left_speed = measure_data["initial_left_speed"]
        self.initial_right_speed = measure_data["initial_right_speed"]
        self.measurements = [
            (cw_measure, ccw_measure)
            for cw_measure, ccw_measure
            in zip(measure_data["measurements"]["clockwise"], measure_data["measurements"]["counter_clockwise"])
        ]
        
    @staticmethod
    def load_measure_data(path_to_measures: str):
        with open(path_to_measures) as measurements:
            return json.load(measurements)

    def calculate_type_a(self, cw_dist, ccw_dist):
        """ Evalúa el error tipo A relacionado con la base incierta de distancia. """
        error_present = False
        angular_discrepancy = 0
        error_ratio = 0
        if (cw_dist < 0 and ccw_dist < 0) or (cw_dist > 0 and ccw_dist > 0):
            angular_discrepancy = ((cw_dist + ccw_dist) / (-4 * self.square_length)) * (180 / math.pi)
            if angular_discrepancy != 0:
                error_ratio = 90 / (90 - angular_discrepancy)
                error_present = True
        return error_present, angular_discrepancy, error_ratio

    def calculate_type_b(self, cw_dist, ccw_dist):
        """ Evalúa el error tipo B causado por diámetros de rueda desiguales. """
        error_present = False
        angular_discrepancy = 0
        error_ratio = 0
        if (cw_dist < 0 < ccw_dist) or (cw_dist > 0 > ccw_dist):
            angular_diff = cw_dist - ccw_dist if cw_dist < 0 < ccw_dist else ccw_dist - cw_dist
            angular_discrepancy = (angular_diff / (-4 * self.square_length)) * (180 / math.pi)
            if angular_discrepancy != 0:
                semi_diameter = (self.square_length / 2) / math.sin(math.radians(angular_discrepancy / 2))
                error_ratio = (semi_diameter + angular_discrepancy / 2) / (semi_diameter - angular_discrepancy / 2)
                error_present = True
        return error_present, angular_discrepancy, error_ratio

    def report_errors_from_last_measure(self):
        """ Determina y reporta los tipos de error encontrados en los movimientos del robot. """
        cw_dist, ccw_dist = self.measurements[-1]
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

    def _apply_corrections(self):
        wheel_base = self.initial_wheel_base
        base_left_speed = self.initial_left_speed
        base_right_speed = self.initial_right_speed
        for measure_num, meassure in enumerate(self.measurements):
            cw_measure, ccw_measure = meassure
            type_a_is_present, type_a_ang_discrepancy, type_a_ratio = self.calculate_type_a(cw_measure, ccw_measure)
            type_b_is_present, type_b_ang_discrepancy, type_b_ratio = self.calculate_type_b(cw_measure, ccw_measure)
            if type_a_is_present:
                wheel_base *= type_a_ratio
                print(f'Measure nº{measure_num + 1}:')
                print(f'\tCorrected wheel base from {wheel_base / type_a_ratio:.4f} to {wheel_base:.4f} ')
                print(f'\tError Ratio {type_a_ratio:.4f}')
            elif type_b_is_present:
                base_left_speed *= type_b_ratio
                base_right_speed /= type_b_ratio
                print(f'Measure nº{measure_num + 1}: ')
                print(f'\tCorrected left_speed from {base_left_speed / type_b_ratio:.4f} to {base_left_speed:.4f}')
                print(f'\tCorrected right_speed from {base_right_speed * type_b_ratio:.4f} to {base_right_speed:.4f}')
                print(f'\tError Ratio {type_b_ratio:.4f}')
            if measure_num < len(self.measurements)-1:
                print('----------------------------------')
        print('==================================')
        return wheel_base, base_left_speed, base_right_speed
        
    def generate_config(self):
        print('==================================')
        print('Generating config file...')
        print('==================================')
        wheel_base, base_left_speed, base_right_speed = self._apply_corrections()
        config = configparser.ConfigParser()
        config.read("../../config.ini")
        config["Base"]["wheel_diameter"] = str(self.wheel_diameter)
        config["Base"]["wheel_base"] = str(wheel_base)
        config["Base"]["base_left_speed"] = str(base_left_speed)
        config["Base"]["base_right_speed"] = str(base_right_speed)
        with open("../../config.ini", "w") as configfile:
            config.write(configfile)


if __name__ == '__main__':
    # measures = os.path.join(os.path.dirname(__file__), "square_test.json")
    # measures = os.path.join(os.path.dirname(__file__), "square_test_with_error.json")
    measures = os.path.join(os.path.dirname(__file__), "square_test_with_diff_base.json")
    error_classifier = OdometricErrorClassifier(measures)
    error_classifier.report_errors_from_last_measure()
    error_classifier.generate_config()
