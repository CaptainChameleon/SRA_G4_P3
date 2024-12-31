import math

def calculo_base(cw, ccw, L):
    primerTermino = (cw-ccw)/(-4*L)
    segundoTermino = 180/math.pi
    return primerTermino*segundoTermino

def calculo_radio(cw, ccw, L):
    beta_grados = calculo_base(cw, ccw, L)  # Obtener beta en grados
    beta_radianes = math.radians(beta_grados)  # Convertir grados a radianes para cálculos trigonométricos
    seno_beta_medio = abs(math.sin(beta_radianes / 2))  # Usar seno de la mitad de beta en radianes
    if seno_beta_medio == 0:  # Comprobar división por cero
        return float('inf')  # Devolver infinito para evitar división por cero
    solucion = (L / 2) / seno_beta_medio
    return solucion

def ratio(cw, ccw, L):
    radio =  calculo_radio(cw, ccw, L)
    base = calculo_base(cw, ccw, L)
    base_media = base/2
    solucion = (radio-base_media)/(radio+base_media)
    return solucion


if __name__ == "__main__":


    L = 50
    cw = 4.37
    ccw = 3.81
    bnominal = 11.5

    base = calculo_base(cw, ccw, L)

    radio = calculo_radio(cw, ccw, L)

    ratioValue = ratio(cw, ccw, L)

    base = calculo_base(cw, ccw, L)
    Bact = bnominal * (90/(90-base))

    vl = 0.2 * ratioValue
    vr = 0.2 / ratioValue

    print("Base Corregida: ", Bact)
    print("Speed Correction: ", ratioValue)
    print("VL: ", vl)
    print("VR: ", vr)
    

