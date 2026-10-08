"""einstellungen -- Pfade, Takte und Grenzen, die man beim Einrichten anfasst.

Seit urdf_gui2_1 ein eigenes Modul. Der Inhalt ist zeilengleich aus
urdf_gui3_7.py verschoben (dort Zeilen 1140-1176); neu sind
nur dieser Kopf, die Importe und STAND.

Seit Stand 2 (urdf_gui 2.3) stehen keine absoluten Pfade mehr hier. Alle
Ablagen liegen neben dieser Datei, also dort, wo auch das Launch-Skript
seine URDF sucht -- die Oberflaeche laeuft damit in jedem Ordner und auf
jedem Rechner, ohne dass jemand diese Datei anpasst. Wer etwas anderes
will, setzt eine Umgebungsvariable statt hier einen Pfad einzutragen:

    URDF_GUI_URDF        URDF, wenn beim Start keine angegeben ist
    URDF_GUI_SICHERUNG   Ablage fuer Sicherungen der URDF
    URDF_GUI_BAHNEN      Ablage fuer CSV-Bahnen
"""

import os

# Muss zu MODUL_STAENDE in urdf_gui.py passen.
# 2: Pfade relativ zu dieser Datei, Umgebungsvariablen, BAHN_VIEWER.
STAND = 2

# Ordner dieser Datei. Alle Standardpfade haengen daran.
HIER = os.path.dirname(os.path.abspath(__file__))


def _pfad(variable, standard):
    """Pfad aus der Umgebung oder den Standard. Leer zaehlt als nicht gesetzt."""
    wert = os.environ.get(variable, "").strip()
    return os.path.expanduser(wert) if wert else standard


# ---------- URDF ohne Startargument ----------
# Dieselbe Datei, die display_launch4.py als Standard nimmt.
URDF_PATH = _pfad("URDF_GUI_URDF", os.path.join(HIER, "bmKinematic.urdf"))
# ---------- Ablage fuer bearbeitete Staende ----------
# Absoluter Pfad oder relativ zur geladenen URDF. Wird bei Bedarf angelegt.
BACKUP_DIR = _pfad("URDF_GUI_SICHERUNG", os.path.join(HIER, "URDF_Speicher"))
# ---------- Ablage fuer Bahnen (CSV) ----------
# Absoluter Pfad oder relativ zur geladenen URDF. Wird bei Bedarf angelegt.
BAHN_DIR = _pfad("URDF_GUI_BAHNEN", os.path.join(HIER, "TCP_Fahrten"))
# ---------- Bahnansicht ----------
# Liegt neben dieser Datei. Die Oberflaeche nennt sie nach jedem Export.
# Stand bis 2.2 in totg_prozess.py und zeigte dort auf bahn_viewer4.py,
# eine Datei, die es nie gab.
BAHN_VIEWER = "bahn_viewer3.py"
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
