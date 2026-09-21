"""anzeige -- Anzeigekonvention und das Singleton ANZEIGE. Es gibt genau eine
Instanz;
wer sie braucht, importiert sie von hier.

Seit urdf_gui2_1 ein eigenes Modul. Der Inhalt ist zeilengleich aus
urdf_gui3_7.py verschoben (dort Zeilen 1341-1435); neu sind
nur dieser Kopf, die Importe und STAND.
"""

import konvention

# Muss zu MODUL_STAENDE in urdf_gui2_1.py passen.
STAND = 1


# ==========================================================================
# Anzeigekonvention
# ==========================================================================

class Anzeigekonvention:
    """Die eine Stelle, an der steht, wie Winkel angezeigt werden.

    Ein Modulzustand, kein Fensterzustand. Der Umschalter sitzt im
    Parameterfenster, wirkt aber auf Frametabelle, TCP-Zeile, kartesischen
    Reiter, Fahrfenster und Jogtasten gleichzeitig. Zwei Fenster, die
    dieselben drei Zahlen nach verschiedenen Vorschriften zeigen, waeren
    genau die Doppelbedeutung, die diese Oberflaeche sonst vermeidet.

    Wer etwas anzeigt, ruft zerlege. Wer etwas gelesenes weiterreicht, ruft
    baue. Dazwischen liegt immer eine Matrix, nie ein zweiter Winkelsatz.
    """

    def __init__(self):
        self._name = konvention.STANDARD
        self._horcher = []                             # Rueckrufe beim Wechsel
        self._sperre = None                            # Grund, wenn gesperrt

    # -- Zustand -----------------------------------------------------------

    @property
    def name(self):
        return self._name

    @property
    def achsen(self):
        return konvention.ACHSEN[self._name]

    @property
    def titel(self):
        return konvention.TITEL[self._name]

    @property
    def beschreibung(self):
        return konvention.BESCHREIBUNG[self._name]

    def sperren(self, grund):
        """Umschalten voruebergehend verbieten, etwa waehrend einer Aufnahme."""
        self._sperre = grund

    def freigeben(self):
        self._sperre = None

    @property
    def gesperrt(self):
        return self._sperre

    def setzen(self, name):
        """Konvention wechseln. Gibt  (geaendert, grund) zurueck."""
        konvention.pruefe(name)
        if name == self._name:
            return False, "unveraendert"
        if self._sperre is not None:
            return False, self._sperre
        self._name = name
        for rueckruf in list(self._horcher):
            rueckruf(name)
        return True, ""

    def melden(self, rueckruf):
        """Rueckruf beim Wechsel eintragen. Bekommt den neuen Namen."""
        self._horcher.append(rueckruf)

    # -- Rechnen -----------------------------------------------------------

    def zerlege(self, matrix):
        """4x4 -> ((x, y, z), (A, B, C)) in der gewaehlten Konvention."""
        return konvention.to_xyz_abc(matrix, self._name)

    def baue(self, abc, xyz):
        """(A, B, C) und (x, y, z) -> 4x4 in der gewaehlten Konvention."""
        return konvention.from_abc_xyz(abc, xyz, self._name)

    def entartet(self, abc):
        """True, wenn B nahe an einer Stellung steht, in der A und C zerfallen."""
        return konvention.ist_singulaer(abc, self._name,
                                        ENTARTUNG_WARNUNG_DEG)

    def spaltennamen(self, praefix=""):
        return konvention.spaltennamen(praefix, self._name)


# Ab hier meldet die TCP-Zeile die Entartung. Ein Grad ist grosszuegig, aber
# die Anzeige wird schon deutlich frueher unruhig: bei einem halben Grad
# Abstand aendert sich A um mehrere Grad, wenn der TCP um einen Millimeter
# wandert.
ENTARTUNG_WARNUNG_DEG = 1.0

ANZEIGE = Anzeigekonvention()
