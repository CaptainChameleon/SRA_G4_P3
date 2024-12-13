import math


class Vector:
    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def __add__(self, other):
        return Vector(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Vector(self.x - other.x, self.y - other.y)

    def dot(self, other):
        if isinstance(other, Vector):
            return self.x * other.x + self.y * other.y
        else:
            raise ValueError("Ambos operandos deben ser vectores.")

    def rotate(self, angle):
        radians = math.radians(angle)
        cos_theta = math.cos(radians)
        sin_theta = math.sin(radians)
        new_x = self.x * cos_theta - self.y * sin_theta
        new_y = self.x * sin_theta + self.y * cos_theta

        new_x = 0 if abs(new_x) < 1e-10 else new_x
        new_y = 0 if abs(new_y) < 1e-10 else new_y
        return Vector(new_x, new_y)

    def __repr__(self):
        return "Vector({}, {})".format(self.x, self.y)


if __name__ == "__main__":
    vector1 = Vector(0, 1)
    rotated_vector = vector1.rotate(90)
    print("Vector original:", vector1)
    print("Vector rotado:", rotated_vector)
