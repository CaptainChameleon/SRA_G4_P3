#!/usr/bin/env python3
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_D, SpeedPercent
from ev3dev2.sound import Sound
from time import sleep
import math as Math

# Configuración de motores y sonido
motor_izquierdo = LargeMotor(OUTPUT_A)
motor_derecho = LargeMotor(OUTPUT_D)
sonido = Sound()

# Parámetros para movimiento recto y giros
distancia_grados = (50/(Math.pi*5.5))*360 #5.6 es el diametro de las ruedas (Verificar)
giro_grados = 193      # 180 Grados es para 12cm de distancia entre las ruedas

# Función para avanzar 40 cm
def move_forward(distancia_grados, speed=20):
    motor_izquierdo.on_for_degrees(speed=SpeedPercent(speed), degrees=distancia_grados, brake=True, block=False)
    motor_derecho.on_for_degrees(speed=SpeedPercent(speed), degrees=distancia_grados, brake=True, block=True)
    motor_izquierdo.reset()
    motor_derecho.reset()

# Función para girar 90°
def turn_90_degrees(direction, giro_grados, speed=20):
    if direction == "right":
        motor_izquierdo.on_for_degrees(speed=SpeedPercent(speed), degrees=giro_grados, brake=True, block=False)
        motor_derecho.on_for_degrees(speed=SpeedPercent(-speed), degrees=giro_grados, brake=True, block=True)
    else:
        motor_izquierdo.on_for_degrees(speed=SpeedPercent(-speed), degrees=giro_grados, brake=True, block=False)
        motor_derecho.on_for_degrees(speed=SpeedPercent(speed), degrees=giro_grados, brake=True, block=True)


# Realizar el recorrido en cuadrado (Apartado A)
for _ in range(4):
    move_forward(distancia_grados)
    turn_90_degrees("right", giro_grados)


''' APARTADO B
# 10 recorridos en sentido horario
for _ in range(10):
    for _ in range(4):
        move_forward(distancia_grados)
        turn_90_degrees("right", giro_grados)

# 10 recorridos en sentido anti-horario
for _ in range(10):
    for _ in range(4):
        move_forward(distancia_grados)
        turn_90_degrees("left", giro_grados)
'''

''' APARTADO D Cambiar la función move_forward

def move_forward_with_error(speed_left=20, speed_right=30):
    motor_izquierdo.on_for_degrees(speed=SpeedPercent(speed_left), degrees=distancia_grados, brake=True, block=True)
    motor_derecho.on_for_degrees(speed=SpeedPercent(speed_right), degrees=distancia_grados, brake=True, block=True)

'''

''' APARTADO F (Poner Objeto delante del robot para medir la distancia a partir del ultrasonido y realizar prueba giroscopio)
us = UltrasonicSensor()
with open('test.txt', 'w') as f:
    for  i in range(3):
        
        f.write(str(i) + ': ' + str(us.distance_centimeters) + ' cm')

        sound.beep()
        sleep(5)

'''

# Indicar finalización
sonido.beep()
print("Recorrido cuadrado completado.")

