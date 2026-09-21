"""einstellungen -- Pfade, Takte und Grenzen, die man beim Einrichten anfasst.

Seit urdf_gui2_1 ein eigenes Modul. Der Inhalt ist zeilengleich aus
urdf_gui3_7.py verschoben (dort Zeilen 1140-1176); neu sind
nur dieser Kopf, die Importe und STAND.
"""

# Muss zu MODUL_STAENDE in urdf_gui2_1.py passen.
STAND = 1


# ---------- hier URDF eintragen ----------
URDF_PATH = "/home/fwohlfeil/simple_robot.urdf"
# ---------- Ablage fuer bearbeitete Staende ----------
# Absoluter Pfad oder relativ zur geladenen URDF. Wird bei Bedarf angelegt.
BACKUP_DIR = "/home/fwohlfeil/Display/URDF_Speicher"
# -----------------------------------------
# Absoluter Pfad zu TCP_Fahrten
BAHN_DIR = "/home/fwohlfeil/Display/TCP_Fahrten"
# ----------------------------------------- 

TF_RATE_HZ = 30.0
FOOTER_TICK_MS = 50
# Zeilen, die das Meldungsfenster haelt. Alles darueber faellt vorne heraus.
MAX_LOG_LINES = 600
MAX_HISTORY = 25                                       # Tiefe von Rueckgaengig
# Beim Ziehen am Schieber nicht bei jedem Pixel die Tabellen neu bauen.
# Die TF-Ausgabe bekommt den Wert trotzdem sofort.
SLIDER_REFRESH_MS = 60
# Spanne der Schieber, wenn die URDF keinen Anschlag vorgibt.
FREE_ROTATION_DEG = 180.0
FREE_TRAVEL_M = 1.0
# Angestrebte Zahl der Stufen ueber die ganze Spanne. Die Schrittweite ergibt
# sich daraus und wird auf einen glatten Wert gerundet.
SLIDER_STEPS = 1000

# ---------- Bahnrekorder ----------
# Eigener Takt, bewusst nicht an SLIDER_REFRESH_MS gekoppelt: die Abtastung
# soll gleichmaessig bleiben, auch wenn die Tabellen gerade viel zu tun haben.
RECORD_RATE_HZ = 25.0
RECORD_TICK_MS = int(round(1000.0 / RECORD_RATE_HZ))
# Steht der TCP still, waere jeder Takt ein weiterer Punkt auf derselben
# Stelle. Erst ab diesem Weg bzw. dieser Drehung wird abgelegt.
RECORD_MIN_DIST_M = 0.0005
RECORD_MIN_ANGLE_DEG = 0.5
# Deckel gegen unbegrenzten Speicherverbrauch. Bei 25 Hz sind das gut
# 13 Minuten durchgehender Bewegung, danach faellt vorne der aelteste Punkt weg.
RECORD_MAX_POINTS = 20000

RECORD_MIN_JOINT_DEG = 0.5
