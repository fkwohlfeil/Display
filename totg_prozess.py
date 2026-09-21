"""totg_prozess -- totg_rechner.py als kurzer Kindprozess. Kein Tk, kein ROS.

Seit urdf_gui2_1 ein eigenes Modul. Der Inhalt ist zeilengleich aus
urdf_gui3_7.py verschoben (dort Zeilen 3970-4111); neu sind
nur dieser Kopf, die Importe und STAND.
"""

import os
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import totg_folge

# Muss zu MODUL_STAENDE in urdf_gui2_1.py passen.
STAND = 1


# ==========================================================================
# TOTG-Hilfsprozess (seit 3.6)
# ==========================================================================

# Liegt neben dieser Datei, wie totg_folge.py.
TOTG_RECHNER = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "totg_rechner.py")

# Eigene Vorsilbe, nicht WORKDIR_PREFIX: sweep_workdirs raeumt beim
# Neustart von move_group die alten Ordner ab und darf dabei keinen
# erwischen, in dem gerade gerechnet wird.
TOTG_PREFIX = "urdf_gui_totg_"

# Die Bahnansicht, die beide CSVs nebeneinander zeigt.
BAHN_VIEWER = "bahn_viewer4.py"


class TotgProzess:
    """totg_rechner.py als kurzer Kindprozess. Kein Tk, kein ROS.

    Gestartet wird je Auftrag, abgefragt im Fusszeilentakt ueber
    abfragen(). Gewartet wird nirgends: der GUI-Thread sieht nach, ob das
    Kind fertig ist, und geht sonst weiter -- dieselbe Regel wie bei der
    Bruecke, nur ohne ROS dazwischen.

    Die Ausgabe des Kindes geht in eine Datei. Ohne sie ist ein
    Fehlschlag nicht aufzuklaeren: TOTG meldet den Grund, etwa eine
    fehlende Beschleunigungsgrenze, ueber das rclcpp-Protokoll und nicht
    ueber den Rueckgabewert. Der Ordner des letzten Auftrags bleibt
    deshalb liegen, bis der naechste beginnt oder die GUI endet.
    """

    def __init__(self, rechner=TOTG_RECHNER):
        self.rechner = rechner
        self.process = None
        self.ordner = None
        self.tag = None
        self.gestartet = None
        self.frist = 0.0

    @property
    def laeuft(self):
        return self.process is not None

    def starten(self, auftrag, tag, frist=totg_folge.FRIST_S):
        """Auftrag schreiben und das Kind starten. None oder eine Meldung."""
        self.aufraeumen()
        if not os.path.isfile(self.rechner):
            return f"{self.rechner} fehlt. Er gehoert neben diese Datei."
        try:
            self.ordner = tempfile.mkdtemp(prefix=TOTG_PREFIX)
            pfad = os.path.join(self.ordner, "auftrag.json")
            with open(pfad, "w", encoding="utf-8") as handle:
                handle.write(totg_folge.auftrag_text(auftrag))
            protokoll = open(os.path.join(self.ordner, "totg.log"), "wb")
        except OSError as exc:
            return f"Arbeitsordner fuer TOTG nicht schreibbar: {exc}"
        try:
            self.process = subprocess.Popen(
                [sys.executable, self.rechner, pfad,
                 os.path.join(self.ordner, "antwort.json")],
                stdout=protokoll,
                stderr=subprocess.STDOUT,
                start_new_session=True)
        except (OSError, ValueError) as exc:
            self.process = None
            return f"totg_rechner nicht startbar: {exc}"
        finally:
            protokoll.close()
        self.tag = tag
        self.gestartet = time.monotonic()
        self.frist = float(frist)
        return None

    def abfragen(self):
        """None, solange es laeuft. Sonst ein Ergebnis mit tag.

        Entweder error, oder text mit der Antwortdatei. Gelesen wird die
        Antwort drueben mit totg_folge.antwort_lesen -- hier wird nur
        abgeholt.
        """
        if self.process is None:
            return None
        tag = self.tag
        dauer = time.monotonic() - self.gestartet
        code = self.process.poll()
        if code is None:
            if dauer < self.frist:
                return None
            self.abbrechen()
            return {"tag": tag,
                    "error": f"totg_rechner antwortet seit "
                             f"{self.frist:.0f} s nicht"}
        self.process = None
        try:
            with open(os.path.join(self.ordner, "antwort.json"),
                      encoding="utf-8") as handle:
                text = handle.read()
        except OSError:
            return {"tag": tag,
                    "error": f"totg_rechner endete mit Code {code} "
                             f"ohne Antwort"}
        return {"tag": tag, "text": text, "dauer": dauer}

    def protokoll(self, lines=12):
        """Die letzten Zeilen der Kindausgabe."""
        if not self.ordner:
            return []
        try:
            with open(os.path.join(self.ordner, "totg.log"), "rb") as handle:
                handle.seek(0, os.SEEK_END)
                groesse = handle.tell()
                handle.seek(max(0, groesse - 32768))
                text = handle.read().decode("utf-8", "replace")
        except OSError:
            return []
        return [zeile for zeile in text.splitlines() if zeile.strip()][-lines:]

    def abbrechen(self):
        """Kind beenden, falls es laeuft. Der Ordner bleibt."""
        process, self.process = self.process, None
        if process is None:
            return
        for sig, frist in ((signal.SIGTERM, 2.0), (signal.SIGKILL, 1.0)):
            try:
                os.killpg(process.pid, sig)
            except OSError:
                pass
            try:
                process.wait(timeout=frist)
                return
            except subprocess.TimeoutExpired:
                continue

    def aufraeumen(self):
        """Kind beenden und den Ordner entfernen."""
        self.abbrechen()
        if self.ordner and os.path.isdir(self.ordner):
            shutil.rmtree(self.ordner, ignore_errors=True)
        self.ordner = None
