"""matrix -- 4x4-Matrizen als Liste von Zeilen. Kein Tkinter, kein ROS, kein
NumPy.

Seit urdf_gui2_1 ein eigenes Modul. Der Inhalt ist zeilengleich aus
urdf_gui3_7.py verschoben (dort Zeilen 1237-1340); neu sind
nur dieser Kopf, die Importe und STAND.
"""

import math
import konvention

# Muss zu MODUL_STAENDE in urdf_gui2_1.py passen.
STAND = 1


# ==========================================================================
# Matrizen: 4x4 als Liste von Zeilen
# ==========================================================================

def identity():
    return [[1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]]


def multiply(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)]
            for i in range(4)]


def from_rpy_xyz(rpy, xyz):
    """URDF-origin als 4x4. Feste Achsen: R = Rz(yaw) @ Ry(pitch) @ Rx(roll).

    Ausdruecklich nicht ueber die Anzeigekonvention: die URDF fuehrt rpy und
    nichts anderes. Wer hier die gewaehlte Konvention einsetzte, schriebe je
    nach Einstellung verschiedene Dateien aus demselben Modell.
    """
    return konvention.from_rpy_xyz(rpy, xyz)


def from_axis_angle(axis, angle):
    """Rodrigues-Formel als 4x4. axis muss normiert sein."""
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    k = 1.0 - c
    return [
        [x * x * k + c,     x * y * k - z * s, x * z * k + y * s, 0.0],
        [y * x * k + z * s, y * y * k + c,     y * z * k - x * s, 0.0],
        [z * x * k - y * s, z * y * k + x * s, z * z * k + c,     0.0],
        [0.0,               0.0,               0.0,               1.0],
    ]


def from_translation(xyz):
    matrix = identity()
    matrix[0][3], matrix[1][3], matrix[2][3] = xyz
    return matrix


def invert(matrix):
    """Kehrmatrix einer homogenen Transformation.

    Nicht der allgemeine Fall: der Drehanteil ist orthonormal, seine
    Kehrmatrix ist die Transponierte. Damit reicht Umsortieren plus drei
    Skalarprodukte. Eine Gauss-Elimination waere hier nicht nur Aufwand,
    sondern auch ungenauer, weil sie die Orthonormalitaet nicht ausnutzt.

        R^-1 = R^T        t^-1 = -R^T * t
    """
    result = identity()
    for row in range(3):
        for column in range(3):
            result[row][column] = matrix[column][row]
    for row in range(3):
        result[row][3] = -sum(matrix[k][row] * matrix[k][3] for k in range(3))
    return result


def to_xyz_rpy(matrix):
    """4x4 -> ((x, y, z), (roll, pitch, yaw)). Umkehrung von from_rpy_xyz.

    Der Weg in die URDF und in die Bearbeitungsfelder. Fuer alles, was eine
    Lage anzeigt, steht ANZEIGE.zerlege daneben -- das ist die Stelle, die
    der Umschalter erreicht.
    """
    return konvention.to_xyz_rpy(matrix)


def to_quaternion(matrix):
    """4x4 -> (x, y, z, w), Shepperd-Methode."""
    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (matrix[2][1] - matrix[1][2]) / s
        y = (matrix[0][2] - matrix[2][0]) / s
        z = (matrix[1][0] - matrix[0][1]) / s
    elif matrix[0][0] > matrix[1][1] and matrix[0][0] > matrix[2][2]:
        s = math.sqrt(1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]) * 2.0
        w = (matrix[2][1] - matrix[1][2]) / s
        x = 0.25 * s
        y = (matrix[0][1] + matrix[1][0]) / s
        z = (matrix[0][2] + matrix[2][0]) / s
    elif matrix[1][1] > matrix[2][2]:
        s = math.sqrt(1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]) * 2.0
        w = (matrix[0][2] - matrix[2][0]) / s
        x = (matrix[0][1] + matrix[1][0]) / s
        y = 0.25 * s
        z = (matrix[1][2] + matrix[2][1]) / s
    else:
        s = math.sqrt(1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]) * 2.0
        w = (matrix[1][0] - matrix[0][1]) / s
        x = (matrix[0][2] + matrix[2][0]) / s
        y = (matrix[1][2] + matrix[2][1]) / s
        z = 0.25 * s
    return x, y, z, w
