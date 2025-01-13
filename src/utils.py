import math


class Vector:

    @staticmethod
    def middle_of(vector_a, vector_b):
        middle = vector_b - vector_a
        middle = middle.to_length(middle.length / 2)
        return vector_a + middle

    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def __add__(self, other):
        return Vector(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Vector(self.x - other.x, self.y - other.y)

    @property
    def length(self):
        return math.sqrt(self.x**2 + self.y**2)

    def to_length(self, new_length):
        b = math.sqrt(new_length**2 / (self.x**2 + self.y**2))
        return Vector(b * self.x, b * self.y)

    def dot(self, other):
        if isinstance(other, Vector):
            return self.x * other.x + self.y * other.y
        else:
            raise ValueError("Ambos operandos deben ser vectores.")

    def cross(self, other):
        return self.x * other.y - self.y * other.x

    def angle_with(self, other, in_degrees: bool = False):
        angle = math.asin(self.cross(other) / (self.length * other.length))
        if in_degrees:
            return math.degrees(angle)
        return angle

    def rotate_degrees(self, angle):
        return self.rotate_radians(math.radians(angle))

    def rotate_radians(self, angle):
        cos_theta = math.cos(angle)
        sin_theta = math.sin(angle)
        new_x = self.x * cos_theta - self.y * sin_theta
        new_y = self.x * sin_theta + self.y * cos_theta

        new_x = 0 if abs(new_x) < 1e-10 else new_x
        new_y = 0 if abs(new_y) < 1e-10 else new_y
        return Vector(new_x, new_y)

    def __repr__(self):
        return "Vector({:.4f}, {:.4f})".format(self.x, self.y)


if __name__ == "__main__":
    vector1 = Vector(0, 1)
    rotated_vector = vector1.rotate_degrees(90)
    print("Vector original:", vector1)
    print("Vector rotado:", rotated_vector)
