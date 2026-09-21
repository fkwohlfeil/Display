"""stil -- Farbpalette und Masse der Baumansicht. Reine Konstanten.

Seit urdf_gui2_1 ein eigenes Modul. Der Inhalt ist zeilengleich aus
urdf_gui3_7.py verschoben (dort Zeilen 1177-1236); neu sind
nur dieser Kopf, die Importe und STAND.
"""

# Muss zu MODUL_STAENDE in urdf_gui2_1.py passen.
STAND = 1


# ---------- Farben ----------
# Eine Palette fuer alles: Flaechen hell, Text dunkelgrau statt schwarz,
# Signalfarben nur dort, wo sie wirklich etwas bedeuten.
BG = "#eef1f5"                                         # Fensterhintergrund
SURFACE = "#ffffff"                                    # Karten und Tabellen
BORDER = "#d5dbe3"
TEXT = "#1f2933"
MUTED = "#6b7480"
FAINT = "#9aa3ad"
DISABLED = "#a8b0ba"
DISABLED_BG = "#c3c9d1"

ACCENT = "#2563c9"
OK_GREEN = "#1a7f37"
WARN = "#9a6700"

BUTTON = "#e3e8ef"
BUTTON_HOVER = "#d5dce6"
BUTTON_DOWN = "#c7cfdb"
HEADING = "#eaeef3"                                    # Tabellenkopf
SELECT = "#d3e3fb"
STRIPE = "#f7f9fb"                                     # jede zweite Zeile
SCROLL = "#ccd3dc"
TROUGH = "#dde3ea"                                     # Schienenfarbe der Schieber
LOG_BG = "#fbfcfd"

TINT_OK = "#e6f4ea"                                    # TCP-Zeile
TINT_BAD = "#fdeceb"                                   # nicht erreichbarer Frame
TINT_WARN = "#fdf3e2"

COLOR_STOP = "#d13438"
COLOR_STOP_HOVER = "#b32b2f"
COLOR_GO = "#2e7d32"
COLOR_GO_HOVER = "#256628"
# Fahren ist nicht dasselbe wie einen Sollwert stellen. Eigene Farbe,
# damit der Unterschied nicht nur in der Beschriftung steht.
COLOR_DRIVE = "#1565c0"
COLOR_DRIVE_HOVER = "#0f4c8f"
# Der Halte-Taster ist weder das eine noch das andere: er plant und faehrt.
# Eigene Farbe, damit niemand ihn fuer eine zweite Starttaste haelt.
COLOR_HOLD = "#00695c"
COLOR_HOLD_HOVER = "#004d40"

BG_OK = SURFACE                                        # Eingabefeld in Ordnung
BG_CLAMPED = "#fdf3e2"                                 # auf das Limit begrenzt
BG_INVALID = "#fbe0e0"                                 # nicht lesbar

# ---------- Masse der Baumansicht ----------
# Alles in Bildpunkten auf der Zeichenflaeche. Die Werte stehen hier oben,
# weil das Layout sie zweimal braucht: beim Rechnen der Breiten und beim
# Zeichnen. Zwei getrennte Zahlensaetze waeren die sichere Quelle fuer
# Kaesten, die neben ihren Kanten stehen.
TREE_NODE_W = 128                                      # Breite eines Linkkastens
TREE_NODE_H = 24                                       # Hoehe eines Linkkastens
TREE_GAP_X = 22                                        # Luft zwischen Geschwistern
TREE_GAP_Y = 36                                        # Hoehe einer Gelenkkante
TREE_MARGIN = 18                                       # Rand der Zeichenflaeche
TREE_GRAB = 12                                         # Trefferzone neben der Kante
