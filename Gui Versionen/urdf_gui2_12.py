#!/usr/bin/env python3
"""URDF anzeigen, Achsen vorgeben, Vorwaertskinematik rechnen, TF senden.

2.12 ist ein Wort. In _read_plan stand joint_name, richtig ist joint_names:

    AttributeError: 'JointTrajectory' object has no attribute 'joint_name'

Die Verwechslung hat System. JointConstraint.joint_name ist Einzahl, weil
eine Bedingung fuer ein Gelenk gilt; JointTrajectory.joint_names ist Mehrzahl,
weil eine Bahn alle fuehrt. Beide stehen in demselben Auftrag wenige Zeilen
auseinander.

Bemerkenswert ist nicht der Tippfehler, sondern wie lange er sich halten
konnte. Er lag auf dem Erfolgsweg: der Fehlerweg kehrt vor der Bahn um und
fasst das Feld nie an. Solange die Kollisionspruefung jeden Plan verwarf,
wurde die Zeile nie ausgefuehrt. Sie wurde erst erreicht, als 2.9 die
Pruefung abschaltbar machte und zum ersten Mal ein Plan durchkam -- und dann
nahm sie den ROS-Takt mit, weil eine Ausnahme in einem Rueckruf von rclpy
geradewegs in den Executor faellt. Sichtbar wurde davon nichts als eine Zahl
in der Fusszeile, die hochlief.

Drei Fassungen hat es gebraucht, und keine davon war ueberfluessig: 2.5
schrieb die Ausgabe von move_group ueberhaupt erst in eine Datei, 2.8 holte
sie bei einem Fehlschlag herein, 2.11 hielt den Takt am Leben und legte die
Ausnahme ins Meldungsfenster. Erst dann stand der Fehler da, wo jemand ihn
lesen konnte. Das ist der Ertrag: nicht die Korrektur, sondern der Weg, auf
dem ein Fehler sich zeigen kann, statt sich als Stillstand zu aeussern.

Aus 2.11 unveraendert uebernommen: die Absicherung des ROS-Takts.

2.11 haelt den ROS-Takt am Leben. Das Fehlerbild war: nach dem Planen kommt
kein /tf mehr, und Last Refresh zaehlt nur noch hoch. Beides ist dieselbe
Beobachtung, und sie sagt genau eine Sache -- der Executor-Thread ist tot.

Er stand in einer Zeile:

    self._thread = threading.Thread(target=self._executor.spin, daemon=True)

spin() unmittelbar als Ziel eines Threads zu uebergeben hat einen Fehler ohne
Warnung. Faellt in irgendeinem Callback eine Ausnahme, verlaesst sie spin(),
der Thread endet, und mit ihm enden alle Timer: TF, Probe, Auftragstakt. Der
Traceback geht nach stderr, und unter einem Launchfile liest den niemand. Was
bleibt, ist eine Zahl in der Fusszeile, die immer weiter hochlaeuft -- und
eine Zahl, die hochlaeuft, liest man als "es dauert", nicht als "es ist
vorbei". Der Unterschied ist der ganze Punkt.

Vier Aenderungen:

    Eigene Schleife um spin_once statt spin() als Ziel. Eine Ausnahme wird
    gefangen, gezaehlt und dem GUI-Thread hingelegt; der naechste Takt kommt
    trotzdem. Erst nach SPIN_ERROR_LIMIT Fehlern gibt die Schleife auf, und
    dann sagt sie es.

    Jeder eigene Callback laeuft durch _guard. Auch _on_plan_done, denn was
    dort geworfen wird, faellt sonst in rclpys Zustellung der Antwort -- und
    von dort geradewegs in den Executor. Genau der Weg, den das Fehlerbild
    beschreibt: die Ausgabe endet mit dem ersten Planergebnis.

    Der Dienstclient wird nicht mehr aus einem Callback heraus zerstoert,
    sondern zwischen zwei Durchlaeufen. Eine Entitaet wegzuwerfen, waehrend
    der Executor ueber seine Entitaeten laeuft, ist ein Griff in das eigene
    Wartefeld.

    Ein Waechter auf die TF-Ausgabe. Aelter als TF_STALL_MS heisst Meldung im
    Klartext, einmal, mit der Unterscheidung ob der Takt noch laeuft oder
    schon beendet ist. Ein Bediener soll nicht aus einer Zahl schliessen
    muessen, dass RViz einen Stand zeigt, den es nicht mehr gibt.

Die eigentliche Ausnahme ist damit noch nicht behoben -- sie steht jetzt nur
im Meldungsfenster statt in einem Thread, dessen Ausgabe niemand liest. Das
ist der Unterschied zwischen einem Fehler und einem Raetsel.

Aus 2.10 unveraendert uebernommen: die beiden Bremsen.

2.10 nimmt zwei Bremsen heraus, die beide dieselbe Bauart haben: etwas
waechst unbegrenzt, und eine Stelle liest es jedes Mal ganz.

Die erste steckte in log_tail. Seit 2.5 landet die Ausgabe von move_group in
einer Datei, seit 2.8 holt die GUI daraus die letzten Zeilen, sobald ein
Auftrag scheitert. ValidateSolution schreibt zu jedem gefundenen Kontakt zwei
Zeilen; ein gescheiterter Auftrag kommt so auf mehrere hundert, und nach
einer Viertelstunde Fehlersuche liegen dort zweistellige Megabyte.
readlines() zog die komplett in den Speicher, um zehn Zeilen abzuschneiden --
im GUI-Thread, also genau dann sichtbar stehend, wenn das Fenster erklaeren
sollte, was schiefging. Gelesen wird jetzt nur das letzte Stueck der Datei.
Bei 27 MB sind das 0.2 statt 150 ms.

Die zweite ist das Meldungsfenster selbst. Ein Textwidget haelt alles, was je
hineingeschrieben wurde, und jede weitere Einfuegung samt see("end") wird
dabei langsamer. Ueber eine lange Sitzung mit vielen Diagnosen zu je vierzehn
Zeilen ist das eine Oberflaeche, die ohne erkennbaren Grund zaeher wird. Es
haelt jetzt MAX_LOG_LINES Zeilen, vorne faellt heraus was darueber liegt.

Beides ist dieselbe Lehre, und sie ist es wert, im Versuch ausgesprochen zu
werden: eine Datenstruktur, die nur waechst, ist eine Zeitbombe mit langer
Zuendschnur. Sie faellt in der Entwicklung nie auf, weil dort niemand eine
Stunde am Stueck arbeitet.

Dazu zwei Kleinigkeiten am Warten. Waehrend PLANEN laeuft im Fahrfenster eine
Uhr mit -- ein wartendes Fenster ist von einem stehenden sonst nicht zu
unterscheiden, und der Bediener haelt es nach drei Sekunden fuer haengend, zu
Recht. Und die Frist fuer eine Anfrage haengt jetzt am Auftrag statt an einer
festen Zahl: wer die Planungszeit hochsetzt, meint es so.

Aus 2.9 unveraendert uebernommen: der Schalter fuer die Kollisionspruefung.

2.9 macht die Kollisionspruefung schaltbar. Der Anlass steht in 2.8: eine
fertig gerechnete PTP-Bahn wurde vom Antwort-Adapter ValidateSolution
verworfen, weil base_link und link_2 sich beruehrten -- an neun von elf
Stuetzpunkten. Der Kontakt war echt, die Kollision nicht: die
Kollisionsmatrix in der erzeugten SRDF kennt nur benachbarte Paare, und
base_link und link_2 liegen zwei Schritte auseinander. Bei Ersatzkoerpern aus
Quadern und Zylindern beruehren sich solche Paare regelmaessig, ohne dass die
Mechanik es taete.

Der Schalter sitzt in der Karteikarte Pipeline und nimmt zwei Adapter
gleichzeitig heraus, je einen auf jeder Seite:

    CheckStartStateCollision   lehnt den Auftrag vorher ab
    ValidateSolution           wirft die fertige Bahn nachher weg

Nur einen davon zu schalten waere halb geschaltet, und halb geprueft ist
schwerer zu erklaeren als gar nicht geprueft. Beide werden beim Hochfahren
gelesen, der Schalter steht deshalb bei den Werten mit Neustart.

Was er nicht ist: eine Loesung. Aus heisst nicht, dass nichts kollidiert,
sondern dass niemand hinsieht -- das steht so an der Karteikarte, im
Protokoll beim Umschalten und in der Zustandszeile des Fahrfensters. Die
Loesung waere eine gemessene Kollisionsmatrix: Stichproben im Achsraum ueber
/check_state_validity, Paare die nie oder immer beruehren heraus, wie es der
Setup Assistant tut. Das bleibt fuer eine spaetere Fassung.

Aus 2.8 unveraendert uebernommen: die Diagnose gescheiterter Auftraege.

2.8 macht aus einem Fehlercode eine Auskunft. Anlass war die Meldung

    Kein Plan: Unbekannt (99999)

und die ist als Auskunft wertlos. 99999 ist UNDEFINED, der Vorbelegungswert
des Feldes: niemand hat hineingeschrieben. Das bedeutet nicht, dass die
Planung gescheitert ist, sondern dass der Auftrag bei keinem Planer angekommen
ist -- eine Aussage nicht ueber das Ergebnis, sondern ueber sein Ausbleiben.
Wer sie fuer ein Planungsproblem haelt, sucht am Ziel, an den Achsgrenzen und
an der Toleranz und findet dort nichts, weil dort nichts ist.

Der Grund steht in einem solchen Fall grundsaetzlich woanders, naemlich in
der Ausgabe von move_group. Deshalb holt die GUI sie jetzt bei jedem
gescheiterten Auftrag von selbst herein und stellt daneben, womit gefragt
wurde: Gruppe, Planer, Pipeline, die Achsen im Ziel und die Kette aus der
SRDF. Was gefragt wurde, weiss nur die GUI; warum nicht geantwortet wurde,
weiss nur move_group. Nebeneinander ergeben beide meist schon die Antwort.

Aus 2.7 unveraendert uebernommen: die Trennung von Stellen und Fahren.

2.7 trennt Stellen und Fahren. Bis 2.6 lagen beide in denselben Feldern: wer
einen Achswert eintippte oder den Schieber zog, stellte die Achse in
demselben Augenblick -- und wer danach "PTP fahren" drueckte, liess eine Bahn
von einem Punkt zu genau diesem Punkt planen. Pilz lieferte gehorsam einen
einzelnen Stuetzpunkt zurueck, die GUI meldete "Ziel bereits erreicht", und
das war voellig richtig. Die Fahrt war nie moeglich.

Der Fehler steckte nicht in der Rechnung, sondern in der Bedeutung eines
Feldes. Ein Sollwert und ein Fahrziel sehen gleich aus, beide sind eine Zahl
mit Einheit neben einem Gelenknamen, und sie sagen Entgegengesetztes: der
eine, wo die Achse jetzt steht, das andere, wo sie noch nicht ist. Wer beides
in dasselbe Feld legt, hat kein Feld mit zwei Bedeutungen, sondern eins ohne.

Seit 2.7 sind es drei Handlungen in einer Richtung, und jede hat ihren Ort:

    Sollwertfelder links   stellen sofort, ohne MoveIt, wie immer
    Fahrfenster, Ziel      schreibt nur dorthin, nichts bewegt sich
    Fahrfenster, Planen    fragt move_group, das Ergebnis wird gehalten
    Fahrfenster, Fahren    taktet die gehaltene Bahn ab

Die Taste "Fahrt starten" ist stumpf, solange kein Plan gehalten wird. Das
ist die ganze Aussage dieser Fassung: zwischen dem Rechnen einer Bahn und dem
Bewegen der Maschine liegt eine bewusste Handlung des Bedieners. Eine
Steuerung trennt an derselben Stelle zwischen Programmwahl und Startfreigabe,
und aus demselben Grund.

Ein gehaltener Plan gehoert zu dem Achsstand, aus dem heraus er gerechnet
wurde. Wird ein Sollwert gestellt, das Modell geaendert oder ein neues
geladen, faellt er weg -- ein Plan mit falschem Startpunkt ist kein halber
Plan, sondern keiner. Die Pruefung sitzt in _push_positions, also an dem
einen Weg, den alle Sollwerte nehmen, und in _drive_start noch einmal.

Waehrend PLANEN, FAHREN und BREMSEN sind sowohl die Sollwertfelder als auch
die Zielfelder gesperrt. Beides aus demselben Grund wie schon in 2.4: die
Bahn wurde mit den Werten gerechnet, die beim Planen galten.

Aus 2.6 unveraendert uebernommen: Kindprozess, Signale, Zahlenformat.

2.6 setzt einen Punkt. Woertlich: yaml_scalar schrieb Fliesskommazahlen mit
"%.10g", und das liefert fuer 2.0 die Zeichenfolge 2. YAML liest darin eine
Ganzzahl, rclcpp deklariert den Parameter daraufhin als integer, MoveIt
erwartet an dieser Stelle ein double, und die Ausnahme daraus wird nirgends
gefangen. Der Prozess bricht mit terminate() ab, bevor er einen Dienst
angeboten hat.

    parameter 'robot_description_planning.joint_limits.joint1.max_velocity'
    has invalid type: expected [double] got [integer]

Der Fehler ist alt -- er steckt genauso in 2.1 bis 2.5 und trifft jede Achse,
deren Geschwindigkeit rund ist, also in der Praxis fast jede. Sichtbar wird er
erst jetzt, weil die Ausgabe von move_group seit 2.5 in einer Datei landet und
nicht mehr in DEVNULL. Vorher war die Beobachtung "move_group startet und ist
gleich wieder weg", und dazu passt beliebig viel.

Zwei Dinge sind daran Lehrstoff. Erstens ist eine Zahl nicht ihre
Schreibweise: 2 und 2.0 sind derselbe Wert und zwei verschiedene Typen,
sobald ein Textformat dazwischen liegt. Zweitens ist der Weg vom Wert bis zum
Absturz lang und geht durch vier Zustaendigkeiten -- Python, YAML, rclcpp,
MoveIt -- von denen keine falsch gerechnet hat.

Aus 2.5 unveraendert uebernommen: Kindprozess, Signale, Protokolldatei.

2.5 raeumt den Kindprozess auf. Der Fehler davor war einer, den man nicht
sieht, sondern nur an seinen Folgen bemerkt: 2.4 startet move_group ueber
"ros2 run" und beendet ihn mit terminate(). "ros2 run" ist aber nur ein
Python-Wrapper, der die eigentliche Binary als eigenes Kind startet. Das
Signal erreicht den Wrapper, der stirbt, und weil start_new_session=True den
Baum in eine eigene Prozessgruppe legt, wird das Enkelkind nicht mitgenommen.
Es laeuft weiter -- mit dem alten Robotermodell im Speicher.

Nach zwei Neustarts laufen also drei move_group. Alle bedienen dieselben
absoluten Dienste /plan_kinematic_path und /compute_ik. Welcher antwortet,
entscheidet die Zuordnung in DDS, in aller Regel der zuerst gefundene, also
der aelteste. Die GUI startet einen neuen Knoten, schreibt eine neue
Parameterdatei -- und redet danach mit dem Vorgaenger. Das ist keine
Eigenart von MoveIt, sondern eine von POSIX, und sie ist genau deshalb
lehrreich: ein Signal geht an einen Prozess, nicht an eine Absicht.

Vier Aenderungen folgen daraus:

    Signale gehen an die Prozessgruppe, nicht an den Prozess, und in der
    Reihenfolge SIGINT, SIGTERM, SIGKILL. Erst freundlich, dann bestimmt.

    Der Wrapper faellt weg. Die Binary wird ueber AMENT_PREFIX_PATH gesucht
    und unmittelbar gestartet. Dann gibt es einen Prozess statt zwei, das
    Signal kommt an, und der Rueckgabewert bedeutet etwas.

    Jeder Start bekommt einen eigenen Knotennamen, move_group_guiPID_N. Die
    Dienstnamen sind absolut und haengen nicht daran, und die Parameterdatei
    steht unter /**:, es bricht also nichts. Gewonnen ist die Frage, die
    vorher nicht zu stellen war: ist der Knoten im Graphen meiner? Die Probe
    prueft seit 2.5 den Knoten und nicht mehr den Dienstnamen -- ein
    Dienstname im Graphen sagt nur, dass irgendwer ihn bedient.

    Die Ausgabe von move_group landet im Arbeitsordner statt in DEVNULL, und
    der Ordner des letzten Laufs bleibt liegen. Ein Fehlstart war vorher
    nicht aufklaerbar: der Prozess war nach zwei Sekunden weg und die einzige
    Spur wurde beim Aufraeumen gleich mitgeloescht.

Dazu zwei Dinge auf der ROS-Seite, die dasselbe Bild vervollstaendigen: der
Dienstclient wird bei jedem Neustart verworfen -- ein Client, dessen Server
verschwunden ist, bleibt gelegentlich als verbunden stehen und der Aufruf
geht ins Leere -- und jede Anfrage bekommt eine Frist. Ohne die kommt bei
ausbleibender Antwort nichts zurueck, und die Ablaufsteuerung bleibt in
PLANEN stehen, bis jemand das Fenster schliesst.

Aus 2.4 unveraendert uebernommen: Fahren, Zustandsmaschine, Bremsrampe.

2.4 faehrt zum ersten Mal. Der Weg dorthin ist bewusst der kuerzeste:
ein Ziel im Achsraum. PTP braucht keine Rueckwaertskinematik, keine Lage-
und Drehbedingungen und keine Bahngrenzen -- nur den Auftrag und das
Abtasten. Kartesische Ziele, LIN und die Punkteliste kommen danach.

Der Ablauf: die Taste "PTP fahren" liest die Zielfelder, baut einen Auftrag
aus reinen Zahlen, gibt ihn an RosBridge, und die stellt ihn im
Executor-Thread an /plan_kinematic_path. Die Antwort geht ueber dieselbe
Warteschlange zurueck, aus der schon die Zustandsmeldungen kommen, und wird
in _tick abgeholt. Der TrajectoryPlayer tastet die Bahn danach im festen
Takt ab und schreibt ueber _push_positions -- denselben Weg, den auch der
Schieberegler nimmt. Es gibt weiterhin genau einen.

Zwei Entscheidungen darin sind Lehrstoff und keine Nebensache. Erstens
entsteht die Bahnzeit aus der Summe der Takte und nicht aus der Wanduhr:
Tk-after ist nicht echtzeitfaehig, und wer die Wanduhr in die Rechnung
nimmt, nimmt ihr Zittern gleich mit. Derselbe Auftrag liefert so zweimal
denselben Auszug, und der Abstand zu echter Zeit wird angezeigt statt
versteckt. Zweitens haelt STOP nicht an, sondern bremst: die Zeitrate faehrt
ueber eine Rampe auf null, der Roboter bleibt auf seiner Bahn. Ein Sprung
auf null waere die Verletzung genau der Grenzen, um die es hier geht.

Aus 2.1 unveraendert uebernommen: MoveIt an der Oberflaeche, Kindprozess und
Sperren.

Der Kern des Umbaus ist eine Einsicht, die man aussprechen muss: 1.26
verspricht, dass sich das Modell im laufenden Betrieb aendern laesst und
sofort wirkt. MoveIt verspricht das Gegenteil -- Robotermodell, SRDF,
Achsgrenzen und Solver werden beim Hochfahren von move_group gelesen und
danach nie wieder. Statt zu versuchen, den Knoten am Leben zu halten, macht
diese Fassung den Neustart zur normalen, sichtbaren Handlung. Die GUI
schreibt die Parameterdatei selbst, startet move_group als Kindprozess und
merkt sich, ob beide noch zueinander passen.

Daraus folgt die Aufteilung im neuen Fenster "Parameter...": vorne steht,
was pro Auftrag wirkt, dahinter, was erst beim naechsten Start gelesen wird.
Jede Karteikarte sagt, zu welcher Sorte sie gehoert.

Waehrend einer Fahrt und waehrend des Hochlaufens von move_group ist die
Oberflaeche gesperrt -- und zwar beides: die Bewegungsparameter und das
Modell. Der Grund ist derselbe. Die geplante Bahn wurde mit den Werten
gerechnet, die beim Planen galten; aendert sich waehrenddessen eine
Achsgrenze oder faellt ein Gelenk weg, faehrt der Roboter eine Bahn, die zu
nichts mehr passt. Die Sperre haengt an genau einer Stelle, _update_locks,
und _commit weist jede Modellaenderung ab, solange sie steht. Seit 2.4
haengt sie am Zustand: PLANEN, FAHREN und BREMSEN sperren, BEREIT und FEHLER
geben frei.

Neu an der ROS-Seite: /joint_states geht mit heraus. Ohne das meldet der
Zustandsmonitor von move_group, er bekomme keinen aktuellen Roboter, und
verweigert die Planung -- die haeufigste Ursache dafuer, dass MoveIt laeuft,
aber nichts plant. Ein robot_state_publisher wird bewusst NICHT mitgestartet:
der legt feste Gelenke auf /tf_static und braechte die Geisterframes zurueck,
die weiter unten beschrieben sind. Die GUI bleibt alleinige TF-Quelle.

Oberflaeche nach Skizze 1.4:

    Kopf    STOP-Taste, Robotername, Meldungsfenster
    Mitte   links die Gelenke (rad und grad), in der Mitte die Frames
            (X Y Z RPY), rechts die Sollwertvorgabe kartesisch oder
            achsspezifisch samt Jerk, Beschleunigung und Geschwindigkeit
    Fuss    Sitzungsdauer und Alter der letzten TF-Ausgabe

Alle Frames gehen zyklisch auf /tf, auch die fester Gelenke. Das ist Absicht:
ueber /tf_static abgelegte Frames verfallen nie und blieben nach einem
Modellwechsel als Geisterframes in RViz stehen.
Zusaetzlich wird robot_description latched gesendet, damit RViz das Modell findet.
Die Frames stehen als Baum entlang der kinematischen Kette und lassen sich
zweigweise einklappen; die ganze Karte laesst sich wegklappen, dann bekommen
Gelenkliste und Sollwerte den Platz.
Das Bearbeiten von Laengen, Winkeln, Gelenktyp, Achse und Anschlaegen liegt in
einem eigenen Fenster; uebernommene Aenderungen wirken sofort, geschrieben wird
erst per Speichern. parent und child bleiben gesperrt, sie legen die Topologie
fest. Neue Glieder entstehen immer als Link samt zufuehrendem Gelenk,
geloescht wird ein Gelenk stets mit allem, was daran haengt. Jede Aenderung
laeuft ueber dieselbe Pruefung und laesst sich rueckgaengig machen.

Neu gegenueber 1.16: im Bearbeiten-Fenster steht links die Struktur als
Zeichnung. Links sind Kaesten, Gelenke sind die Kanten dazwischen;
durchgezogen heisst beweglich, gestrichelt fest. Ein Klick waehlt aus und
fuellt zugleich die Felder rechts. Die Schaltflaechen am Zeigerort fuehren die
drei Strukturbefehle aus: am Kasten anhaengen, an der Kante einfuegen oder
herausnehmen. Dieselben Befehle liegen als Tasten unter STRUKTUR.

Damit gibt es drei Arten, den Baum zu aendern, und sie unterscheiden sich
genau in dem, was mit dem Rest der Kette geschieht:

    anhaengen       neues Blatt am Ende, sonst aendert sich nichts
    einfuegen       neues Glied zwischen zwei bestehende, der Teilbaum
                    dahinter bleibt stehen oder wandert mit
    herausnehmen    ein einzelnes Glied faellt weg, was daran hing,
                    rueckt eine Stufe hoch
    Teilbaum        Gelenk und alles dahinter verschwinden
    loeschen

1.25 behebt einen Fehler in 1.24: die Auswahl des Fuehrungsgelenks liess
sich nach dem Anwaehlen eines festen Gelenks nicht mehr oeffnen.

1.24 ergaenzt mechanische Kopplungen. In der Zeile KOPPLUNG folgt ein Gelenk
einem anderen nach q = Faktor * q(Fuehrungsgelenk) + Versatz. Das ist kein
Zusatz der Oberflaeche: es wird als <mimic> in die URDF geschrieben und gilt
damit auch fuer robot_state_publisher und MoveIt. Ein gekoppeltes Gelenk
verliert seinen Freiheitsgrad und bekommt statt des Schiebereglers die
Rechenvorschrift angezeigt. Ketten von Kopplungen sind bewusst verboten.

1.23 macht Namen aenderbar. Auf beiden Karteikarten steht eine Zeile NAME mit
eigener Taste; sie zieht alle Verweise mit, die sonst ins Leere zeigen wuerden
-- bei einem Link parent und child jedes Gelenks, bei einem Gelenk mimic,
transmission und gazebo. Der Wurzel-Link bleibt gesperrt: sein Name steht
ausserhalb der URDF noch einmal als fixed frame in display2.rviz.

1.22 holt das Aussehen ins Bearbeiten-Fenster: rechts stehen zwei
Karteikarten, "Gelenk" mit den bisherigen Feldern und "Koerper" mit Form,
Groesse, Ursprung und Farbe. Ein eigenes Fenster gibt es dafuer nicht mehr.
Die Koerperseite hat eine eigene Linkauswahl, in der auch der Wurzel-Link
steht -- zu ihm fuehrt kein Gelenk, ueber die Gelenkliste waere er nie
erreichbar.

1.21 ergaenzte den Ursprung des Koerpers, also Versatz und
Drehung gegen den Linkframe, samt einer Taste fuer den haeufigsten Fall: Z auf
die halbe Bauhoehe, damit ein Segment am Gelenk beginnt statt mittig darauf zu
sitzen. Ist der Haken fuer die Kollision gesetzt, wandert der Ursprung dort
mit.

1.20 ergaenzt ein Fenster fuer das Aussehen eines Links: Form, Groesse und
Farbe des ersten <visual>. Erreichbar ueber die Schaltflaeche am Linkkasten im
Baum oder ueber die Taste "Aussehen..." unter STRUKTUR. Die Linktabelle zeigt
Form und Farbe als eigene Spalten, im Baum steht ein Farbfeld im Kasten;
durchgestrichen heisst, dass der Link kein <visual> hat und in RViz unsichtbar
bleibt. Das betrifft jeden neu angehaengten Link, denn append_limb legt keines
an.

1.19 behebt einen Fensterfehler: das Bearbeiten-Fenster war transient zum
Hauptfenster und bekam deshalb keinen Eintrag in der Fensterleiste. Einmal
minimiert, war es nicht mehr erreichbar, denn lift() holt ein eingeklapptes
Fenster nicht zurueck. Da dort Speichern, Baum und alle Strukturbefehle
liegen, wirkte die Anwendung danach eingefroren.

Aufruf:
    ./urdf_gui2_4.py                      # Standardpfad aus URDF_PATH
    ./urdf_gui2_4.py ~/Display/bmKinematic.urdf
"""
import csv
import collections
import argparse
import copy
import math
import os
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import traceback
import time
import tkinter as tk
import tkinter.font as tkfont
import xml.etree.ElementTree as ET
from datetime import datetime
from tkinter import ttk, filedialog, messagebox

# ---------- hier URDF eintragen ----------
URDF_PATH = "/home/fwohlfeil/simple_robot.urdf"
# ---------- Ablage fuer bearbeitete Staende ----------
# Absoluter Pfad oder relativ zur geladenen URDF. Wird bei Bedarf angelegt.
BACKUP_DIR = "/home/fwohlfeil/Display/URDF_Speicher"
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
    """URDF-origin als 4x4. Feste Achsen: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, xyz[0]],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, xyz[1]],
        [-sp,     cp * sr,                cp * cr,                xyz[2]],
        [0.0,     0.0,                    0.0,                    1.0],
    ]


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
    """4x4 -> ((x, y, z), (roll, pitch, yaw)). Umkehrung von from_rpy_xyz."""
    xyz = (matrix[0][3], matrix[1][3], matrix[2][3])
    sp = -matrix[2][0]
    if abs(sp) >= 1.0 - 1e-9:                          # Gimbal Lock
        pitch = math.copysign(math.pi / 2.0, sp)
        roll = 0.0
        yaw = math.atan2(-matrix[0][1], matrix[1][1])
    else:
        pitch = math.asin(sp)
        roll = math.atan2(matrix[2][1], matrix[2][2])
        yaw = math.atan2(matrix[1][0], matrix[0][0])
    return xyz, (roll, pitch, yaw)


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


# ==========================================================================
# URDF einlesen
# ==========================================================================

# Reihenfolge wie in der Auswahlliste. fixed steht hinten, weil der Wechsel
# dorthin Achse und Anschlaege verwirft.
JOINT_TYPES = ("revolute", "continuous", "prismatic", "fixed")
NEEDS_AXIS = ("revolute", "continuous", "prismatic")
NEEDS_LIMIT = ("revolute", "prismatic")

# Namen werden zu TF-Frames. Leerzeichen, Schraegstriche und Umlaute machen
# dort Aerger, deshalb bewusst eng gefasst.
NAME_PATTERN = re.compile(r"^[A-Za-z_][A-Za-z0-9_.\-]*$")

# Stellen, mit denen ein <origin> geschrieben wird. Sechs waren es bis 1.17,
# und fuer getippte Werte reicht das auch. Einfuegen und Herausnehmen rechnen
# den Ursprung aber ueber eine Matrizenkette neu; dort schlaegt die Rundung
# als Lageabweichung durch, rund drei Mikrometer je Schritt und ueber mehrere
# Schritte summiert. Zehn Stellen druecken das auf Maschinengenauigkeit,
# ohne dass die Datei unleserlich wird: 0.3 bleibt 0.3 und wird nicht zu
# 0.30000000000000004. Achse und Anschlaege bleiben bei sechs Stellen, sie
# werden nie nachgerechnet.
ORIGIN_DIGITS = 10
# Eintrag in der Kopplungsliste fuer "keine Kopplung".
NO_COUPLING = "-- frei --"

# ---------- Aussehen der Links ----------
# Die vier Formen, die URDF in <geometry> kennt. Ein Netz laesst sich nur
# skalieren, nicht umformen: die Datei bestimmt seine Gestalt.
SHAPES = ("box", "cylinder", "sphere", "mesh")
SHAPE_LABELS = {"box": "Quader", "cylinder": "Zylinder",
                "sphere": "Kugel", "mesh": "Netz"}
SHAPE_FIELDS = {
    "box":      [("sx", "X", 0.100), ("sy", "Y", 0.100), ("sz", "Z", 0.100)],
    "cylinder": [("radius", "Radius", 0.030), ("length", "Hoehe", 0.200)],
    "sphere":   [("radius", "Radius", 0.050)],
    "mesh":     [("scale", "Massstab", 1.000)],
}
DEFAULT_RGBA = (0.60, 0.60, 0.62, 1.00)
# Felder im Aussehen-Fenster, die beim Umschalten der Form stehen bleiben.
VISUAL_KEEP = ("hex", "alpha", "ox", "oy", "oz", "oroll", "opitch", "oyaw")
# Formen mit einer Ausdehnung entlang z, die sich halbieren laesst.
HAS_HEIGHT = {"box": "sz", "cylinder": "length"}

DEFAULT_EFFORT = "100"
DEFAULT_VELOCITY = "1.0"
DEFAULT_AXIS = "0 0 1"


def num(elem, name, default=0.0):
    if elem is None or elem.get(name) is None:
        return default
    return float(elem.get(name))


def vec(elem, name, default=(0.0, 0.0, 0.0)):
    if elem is None or elem.get(name) is None:
        return list(default)
    return [float(v) for v in elem.get(name).split()]


def rgba_to_hex(rgba):
    """(r, g, b, a) mit 0..1 in #rrggbb. Die Deckung bleibt aussen vor."""
    return "#" + "".join(f"{max(0, min(255, round(c * 255))):02x}"
                         for c in rgba[:3])


def hex_to_rgb(text):
    """#rrggbb in (r, g, b) mit 0..1. Wirft ValueError bei Unsinn."""
    text = text.strip().lstrip("#")
    if len(text) != 6:
        raise ValueError("Farbe braucht sechs Stellen, zum Beispiel #a0a0a2")
    return tuple(int(text[i:i + 2], 16) / 255.0 for i in (0, 2, 4))


def fmt(values, digits=4):
    return ", ".join(f"{v:.{digits}f}" for v in values)


def fmt_origin(values):
    """Zahlentripel fuer ein <origin>-Attribut. Einzige Schreibstelle."""
    return " ".join(f"{v:.{ORIGIN_DIGITS}g}" for v in values)


def is_rotational(joint):
    return joint["type"] in ("revolute", "continuous")


def slider_range(joint):
    """Spanne des Schiebers in Anzeigeeinheit, also grad bzw. m.

    Ohne Anschlag in der URDF (continuous) braucht der Schieber trotzdem
    zwei Enden. Die Eingabe im Feld bleibt davon unbegrenzt.
    """
    if joint["has_limit"]:
        if is_rotational(joint):
            return math.degrees(joint["lower"]), math.degrees(joint["upper"])
        return joint["lower"], joint["upper"]
    if is_rotational(joint):
        return -FREE_ROTATION_DEG, FREE_ROTATION_DEG
    return -FREE_TRAVEL_M, FREE_TRAVEL_M


def slider_step(joint):
    """Schrittweite des Schiebers, passend zur Spanne des Gelenks.

    Eine feste Schrittweite taugt nicht: dieselbe Zahl, die bei einer Achse
    mit 340 Grad sinnvoll ist, waere bei einem Hub von 40 mm die ganze
    Spanne. Deshalb aus der Spanne abgeleitet und auf 1, 2, 2.5 oder 5 mal
    Zehnerpotenz gerundet, damit runde Werte auf dem Raster liegen.
    """
    low, high = slider_range(joint)
    span = abs(high - low)
    if span < 1e-12:
        return 1e-3
    raw = span / SLIDER_STEPS
    exponent = math.floor(math.log10(raw))
    for nice in (1.0, 2.0, 2.5, 5.0):
        step = nice * 10.0 ** exponent
        if step >= raw:
            return step
    return 10.0 ** (exponent + 1)


def value_digits(step):
    """Nachkommastellen, die zur Schrittweite passen.

    Mindestens zwei, damit Winkel weiter wie gewohnt mit zwei Stellen
    dastehen; bei feinen Schritten entsprechend mehr.
    """
    if step >= 1.0:
        return 2
    return max(2, min(6, int(math.ceil(-math.log10(step)))))


def limit_digits(joint):
    """Stellen fuer die Anzeige der Anschlaege.

    Gedeckelt, damit die Spalte nicht ueberlaeuft; die Werte im Eingabefeld
    duerfen feiner sein als diese Kurzanzeige.
    """
    return min(4, value_digits(slider_step(joint)))


def snap(value, low, high, step):
    """Auf das Raster legen und in der Spanne halten.

    Verankert bei null, nicht am unteren Anschlag: sonst laege die
    Nullstellung zwischen zwei Stufen und waere mit dem Schieber nicht
    genau zu treffen. Die Anschlaege liegen meist neben dem Raster, weil sie
    aus gerundeten Radiant stammen; die letzte halbe Stufe faengt deshalb
    auf den Anschlag, damit er auch am Schieber genau erreichbar bleibt.
    """
    if value >= high - step / 2.0:
        return high
    if value <= low + step / 2.0:
        return low
    return min(max(round(value / step) * step, low), high)


def read_visual(root, link):
    """Form, Groesse und Farbe des ersten <visual> eines Links oder None.

    URDF erlaubt mehrere <visual> je Link, etwa um einen Koerper aus zwei
    Grundformen zusammenzusetzen. Gelesen und geschrieben wird nur das erste;
    alles andere waere ein eigener Editor. Die Anzahl wird mitgegeben, damit
    das Fenster warnen kann, statt stillschweigend nur die Haelfte zu zeigen.
    """
    visuals = link.findall("visual")
    if not visuals:
        return None
    visual = visuals[0]
    geometry = visual.find("geometry")
    origin = visual.find("origin")

    shape, size, filename = None, {}, ""
    for kind in SHAPES:
        node = geometry.find(kind) if geometry is not None else None
        if node is None:
            continue
        shape = kind
        if kind == "box":
            values = vec(node, "size", (0.1, 0.1, 0.1))
            size = dict(zip(("sx", "sy", "sz"), values))
        elif kind == "cylinder":
            size = {"radius": num(node, "radius", 0.03),
                    "length": num(node, "length", 0.2)}
        elif kind == "sphere":
            size = {"radius": num(node, "radius", 0.05)}
        else:
            filename = node.get("filename", "")
            # Ein Netz skaliert dreiachsig. Der Editor bietet einen Faktor an;
            # ungleiche Achsen bleiben erhalten, solange niemand ihn anfasst.
            size = {"scale": vec(node, "scale", (1.0, 1.0, 1.0))[0]}
        break

    rgba, material = colour_of(root, visual)
    return {"shape": shape, "size": size, "filename": filename,
            "xyz": vec(origin, "xyz"), "rpy": vec(origin, "rpy"),
            "rgba": rgba, "material": material, "count": len(visuals),
            "shared": material is not None and rgba is not None
                      and visual.find("material") is not None
                      and visual.find("material").find("color") is None}


def colour_of(root, visual):
    """Farbe eines <visual> als ((r, g, b, a), Materialname).

    Ein Material kann die Farbe selbst tragen oder nur auf eine oben im
    Dokument definierte verweisen. Der Verweis ist der Grund, warum diese
    Funktion den ganzen Baum braucht und nicht nur das <visual>.
    """
    material = visual.find("material")
    if material is None:
        return None, None
    name = material.get("name")
    colour = material.find("color")
    if colour is None and name:
        for shared in root.findall("material"):
            if shared.get("name") == name:
                colour = shared.find("color")
                break
    if colour is None or colour.get("rgba") is None:
        return None, name
    values = [float(v) for v in colour.get("rgba").split()]
    while len(values) < 4:
        values.append(1.0)
    return tuple(values[:4]), name


def set_visual(root, link_name, shape, size, rgba, filename,
               xyz, rpy, with_collision):
    """Form, Groesse und Farbe des ersten <visual> setzen. Legt es bei Bedarf an.

    Zur Farbe: geschrieben wird immer ein eigenes Material je Link mit
    eingebauter <color>. Der Grund ist Vorhersagbarkeit. Verweist ein Link auf
    ein gemeinsames Material, faerbt eine Aenderung sonst alle mit, die
    denselben Namen nennen -- wer die Farbe eines Links einstellt, erwartet
    das nicht. Der Preis ist ein Materialeintrag mehr je Link.

    Der Ursprung verschiebt und dreht den Koerper gegen den Linkframe. Er ist
    der Grund, warum ein Quader ohne Zutun mittig auf dem Frame sitzt und
    damit zur Haelfte nach hinten ragt: <origin> fehlt, und dann gilt die
    Null. Fuer ein Armsegment entlang z gehoert dort meist die halbe Hoehe.
    Mit der Kinematik hat das nichts zu tun -- der <origin> eines Gelenks
    bewegt die Kette, der eines <visual> nur das Bild.

    `with_collision` fuehrt dieselbe Form und denselben Ursprung in
    <collision> nach. Ohne das driften
    Anzeige und Kollisionskoerper auseinander, was spaetestens bei einer
    Bahnplanung auffaellt. Es ueberschreibt aber eine womoeglich bewusst
    groebere Kollisionsform, deshalb entscheidet der Aufrufer.
    """
    link = next((l for l in root.findall("link")
                 if l.get("name") == link_name), None)
    if link is None:
        raise ValueError(f"Link {link_name} ist nicht mehr im Baum")

    visual = link.find("visual")
    if visual is None:
        visual = ET.SubElement(link, "visual")

    geometry = visual.find("geometry")
    if geometry is None:
        geometry = ET.SubElement(visual, "geometry")
    write_shape(geometry, shape, size, filename)
    write_origin(visual, xyz, rpy)

    material = visual.find("material")
    if material is None:
        material = ET.SubElement(visual, "material")
    material.set("name", f"{link_name}_material")
    colour = material.find("color")
    if colour is None:
        colour = ET.SubElement(material, "color")
    colour.set("rgba", " ".join(f"{v:.4g}" for v in rgba))

    if with_collision:
        collision = link.find("collision")
        if collision is None:
            collision = ET.SubElement(link, "collision")
        target = collision.find("geometry")
        if target is None:
            target = ET.SubElement(collision, "geometry")
        write_shape(target, shape, size, filename)
        write_origin(collision, xyz, rpy)
    return visual


def write_origin(parent, xyz, rpy):
    """<origin> setzen oder anlegen. Dieselbe Genauigkeit wie bei Gelenken."""
    origin = parent.find("origin")
    if origin is None:
        origin = ET.SubElement(parent, "origin")
    origin.set("xyz", fmt_origin(xyz))
    origin.set("rpy", fmt_origin(rpy))
    return origin


def write_shape(geometry, shape, size, filename):
    """<geometry> auf genau eine Form setzen. Alte Formen fallen weg."""
    for kind in SHAPES:
        for old in geometry.findall(kind):
            geometry.remove(old)
    node = ET.SubElement(geometry, shape)
    if shape == "box":
        node.set("size", " ".join(f"{size[k]:.6g}"
                                  for k in ("sx", "sy", "sz")))
    elif shape == "cylinder":
        node.set("radius", f"{size['radius']:.6g}")
        node.set("length", f"{size['length']:.6g}")
    elif shape == "sphere":
        node.set("radius", f"{size['radius']:.6g}")
    else:
        node.set("filename", filename)
        node.set("scale", " ".join([f"{size['scale']:.6g}"] * 3))
    return node


def read_mimic(joint):
    """<mimic> eines Gelenks als dict oder None.

    Die mechanische Kopplung ist kein Zusatz der Oberflaeche, sondern ein
    Element der URDF:

        <mimic joint="joint5" multiplier="-0.6" offset="0"/>

    Damit gilt  q = multiplier * q(joint5) + offset. Der Wert wird also nicht
    mehr gestellt, sondern gerechnet -- das gekoppelte Gelenk verliert seinen
    Freiheitsgrad. Weil es in der Datei steht, halten sich auch
    robot_state_publisher und MoveIt daran und nicht nur diese Oberflaeche.
    """
    mimic = joint.find("mimic")
    if mimic is None or not mimic.get("joint"):
        return None
    return {"joint": mimic.get("joint"),
            "multiplier": num(mimic, "multiplier", 1.0),
            "offset": num(mimic, "offset", 0.0)}


def set_mimic(root, joint_name, source, multiplier, offset):
    """Kopplung setzen oder aendern."""
    element = next((j for j in root.findall("joint")
                    if j.get("name") == joint_name), None)
    if element is None:
        raise ValueError(f"Gelenk {joint_name} ist nicht mehr im Baum")
    mimic = element.find("mimic")
    if mimic is None:
        mimic = ET.SubElement(element, "mimic")
    mimic.set("joint", source)
    mimic.set("multiplier", f"{multiplier:.6g}")
    mimic.set("offset", fmt_origin([offset]))
    return mimic


def clear_mimic(root, joint_name):
    """Kopplung entfernen. Das Gelenk ist danach wieder frei stellbar."""
    element = next((j for j in root.findall("joint")
                    if j.get("name") == joint_name), None)
    if element is None:
        raise ValueError(f"Gelenk {joint_name} ist nicht mehr im Baum")
    for mimic in element.findall("mimic"):
        element.remove(mimic)


def drop_stale_mimics(root):
    """Kopplungen entfernen, die ins Leere zeigen oder sinnlos geworden sind.

    Wird nach jedem Loeschen gerufen. Ohne das bliebe nach dem Entfernen des
    Fuehrungsgelenks ein <mimic> stehen, das auf nichts mehr verweist -- die
    Pruefung faengt es zwar, aber dann waere das Loeschen selbst blockiert,
    obwohl der Bediener nichts falsch gemacht hat.
    """
    known = {j.get("name"): j.get("type") for j in root.findall("joint")}
    for joint in root.findall("joint"):
        mimic = joint.find("mimic")
        if mimic is None:
            continue
        source = mimic.get("joint")
        if (source not in known or source == joint.get("name")
                or known[source] == "fixed" or joint.get("type") == "fixed"):
            joint.remove(mimic)


def coupling_problem(joints, name, source):
    """Meldung zur geplanten Kopplung oder None.

    Ketten sind bewusst verboten: koppelte man A an B und B an C, muesste die
    Auswertung eine Reihenfolge finden, und tf2, KDL und MoveIt loesen solche
    Ketten unterschiedlich oder gar nicht auf. Eine Ebene deckt den Zweck ab
    -- ein Gelenk folgt einem angetriebenen Gelenk.
    """
    follower = next((j for j in joints if j["name"] == name), None)
    leader = next((j for j in joints if j["name"] == source), None)
    if follower is None:
        return f"Gelenk {name} gibt es nicht"
    if leader is None:
        return f"Gelenk {source} gibt es nicht"
    if name == source:
        return "Ein Gelenk kann nicht sich selbst folgen"
    if follower["type"] == "fixed" or leader["type"] == "fixed":
        return "Feste Gelenke lassen sich nicht koppeln"
    if leader["mimic"] is not None:
        return (f"{source} folgt selbst schon {leader['mimic']['joint']}. "
                f"Ketten von Kopplungen sind nicht vorgesehen.")
    if any(j["mimic"] is not None and j["mimic"]["joint"] == name
           for j in joints):
        return (f"{name} ist selbst Fuehrungsgelenk. Erst die Kopplung dort "
                f"loesen.")
    return None


def apply_couplings(joints, positions):
    """Gerechnete Werte der gekoppelten Gelenke eintragen. Aendert das dict.

    Ein Durchlauf genuegt, weil Ketten verboten sind: jedes Fuehrungsgelenk
    ist frei stellbar und steht bereits im dict.
    """
    for joint in joints:
        mimic = joint["mimic"]
        if mimic is None or joint["type"] == "fixed":
            continue
        driver = positions.get(mimic["joint"], 0.0)
        value = mimic["multiplier"] * driver + mimic["offset"]
        if joint["has_limit"]:
            value = min(max(value, joint["lower"]), joint["upper"])
        positions[joint["name"]] = value
    return positions


def parse_root(root):
    """Aus einem ElementTree-Baum lesen. Gibt (name, links, joints) zurueck."""
    links = []
    for link in root.findall("link"):
        inertial = link.find("inertial")
        links.append({
            "name": link.get("name"),
            "mass": num(inertial.find("mass") if inertial is not None else None, "value"),
            "com": vec(inertial.find("origin") if inertial is not None else None, "xyz"),
            "has_inertial": inertial is not None,
            "visual": read_visual(root, link),
        })

    joints = []
    for joint in root.findall("joint"):
        origin = joint.find("origin")
        limit = joint.find("limit")
        rpy = vec(origin, "rpy")
        xyz = vec(origin, "xyz")

        axis = vec(joint.find("axis"), "xyz", (1.0, 0.0, 0.0))
        length = math.sqrt(sum(a * a for a in axis))
        axis = [a / length for a in axis] if length > 1e-12 else [1.0, 0.0, 0.0]

        jtype = joint.get("type")
        joints.append({
            "name": joint.get("name"),
            "type": jtype,
            "parent": joint.find("parent").get("link") if joint.find("parent") is not None else None,
            "child": joint.find("child").get("link") if joint.find("child") is not None else None,
            "xyz": xyz,
            "rpy": rpy,
            "origin": from_rpy_xyz(rpy, xyz),          # konstant, vorab gerechnet
            "quat": to_quaternion(from_rpy_xyz(rpy, xyz)),
            "axis": axis,
            "lower": num(limit, "lower"),
            "upper": num(limit, "upper"),
            "effort": num(limit, "effort"),
            "velocity": num(limit, "velocity"),
            # continuous hat definitionsgemaess keinen Anschlag
            "has_limit": limit is not None and jtype != "continuous",
            "mimic": read_mimic(joint),
        })

    return root.get("name"), links, joints


def parse(path):
    """URDF-Datei einlesen. Gibt (root, name, links, joints, xml) zurueck."""
    with open(path, "r") as handle:
        urdf_xml = handle.read()
    root = ET.fromstring(urdf_xml)
    name, links, joints = parse_root(root)
    return root, name, links, joints, urdf_xml


def serialize(root):
    """Baum als eingerueckten XML-Text. ET.indent gibt es ab Python 3.9."""
    copied = copy.deepcopy(root)
    try:
        ET.indent(copied, space="  ")
    except AttributeError:
        pass
    return ET.tostring(copied, encoding="unicode")


def set_axis(element, axis):
    """<axis> setzen oder entfernen. axis None heisst: Typ braucht keine."""
    existing = element.find("axis")
    if axis is None:
        if existing is not None:
            element.remove(existing)
        return
    if existing is None:
        existing = ET.SubElement(element, "axis")
    existing.set("xyz", " ".join(f"{v:.6g}" for v in axis))


def set_limit(element, lower, upper, keep=False):
    """<limit> setzen oder entfernen.

    Bei `keep` bleibt das Element mit effort und velocity erhalten, nur die
    Anschlaege verschwinden. Das ist der Fall continuous: die Achse dreht
    durch, die Grenzwerte fuer Moment und Drehzahl gelten weiter.
    """
    existing = element.find("limit")
    if lower is None:
        if existing is None:
            if not keep:
                return
            # continuous ohne <limit>: Moment und Drehzahl gehoeren trotzdem
            # hinein, sonst fehlen sie nachgelagerten Werkzeugen.
            existing = ET.SubElement(element, "limit")
            existing.set("effort", DEFAULT_EFFORT)
            existing.set("velocity", DEFAULT_VELOCITY)
            return
        if not keep:
            element.remove(existing)
            return
        for attribute in ("lower", "upper"):
            existing.attrib.pop(attribute, None)
        return
    if existing is None:
        existing = ET.SubElement(element, "limit")
    # In der URDF Pflichtangaben, vorhandene Werte nicht ueberschreiben.
    existing.set("effort", existing.get("effort") or DEFAULT_EFFORT)
    existing.set("velocity", existing.get("velocity") or DEFAULT_VELOCITY)
    existing.set("lower", f"{lower:.6g}")
    existing.set("upper", f"{upper:.6g}")


def name_problem(name, kind, taken):
    """Meldung zum Namen oder None, wenn er brauchbar ist."""
    name = (name or "").strip()
    if not name:
        return f"{kind}: Name fehlt"
    if not NAME_PATTERN.match(name):
        return (f'{kind} "{name}": erlaubt sind Buchstabe oder Unterstrich am '
                f"Anfang, danach Buchstaben, Ziffern, _ . -")
    if name in taken:
        return f'{kind} "{name}" ist schon vergeben'
    return None


def subtree_links(joints, start):
    """`start` und alles, was daran haengt. Kreisfest."""
    kids = {}
    for joint in joints:
        kids.setdefault(joint["parent"], []).append(joint["child"])
    found, stack = [], [start]
    while stack:
        name = stack.pop()
        if name in found:                              # Kreis, nicht zweimal
            continue
        found.append(name)
        stack.extend(kids.get(name, []))
    return found


def append_limb(root, link_name, joint_name, jtype, parent,
                xyz, rpy, axis, lower, upper):
    """Neuen Link samt zufuehrendem Gelenk anhaengen.

    Beides zusammen, nie einzeln: ein Link ohne Gelenk waere ein zweiter
    Wurzel-Link, ein Gelenk ohne Link haette kein Ziel. Der Baum bleibt so in
    jedem Zwischenschritt gueltig.
    """
    link = ET.SubElement(root, "link")
    link.set("name", link_name)

    joint = ET.SubElement(root, "joint")
    joint.set("name", joint_name)
    joint.set("type", jtype)
    ET.SubElement(joint, "parent").set("link", parent)
    ET.SubElement(joint, "child").set("link", link_name)
    origin = ET.SubElement(joint, "origin")
    origin.set("xyz", fmt_origin(xyz))
    origin.set("rpy", fmt_origin(rpy))
    set_axis(joint, axis)
    set_limit(joint, lower, upper, keep=(jtype == "continuous"))
    return joint


def insert_limb(root, link_name, joint_name, jtype, below,
                xyz, rpy, axis, lower, upper, keep_pose):
    """Neues Glied zwischen zwei bestehende haengen.

    Aus   A --below--> B   wird   A --joint_name--> N --below--> B.

    Drei Eingriffe, die nur zusammen einen gueltigen Baum ergeben und deshalb
    im selben Entwurf laufen muessen:

        1. Link N und das zufuehrende Gelenk von A nach N anlegen,
        2. bei `below` den Elternteil von A auf N umschreiben,
        3. den Ursprung von `below` umrechnen, falls gewuenscht.

    Punkt 2 ist der Unterschied zu append_limb: dort entsteht immer ein Blatt,
    weil kein bestehendes Gelenk angefasst wird. Hier wandert ein vorhandener
    Teilbaum an eine neue Stelle, und genau deshalb bleibt die Kette eine
    Kette statt sich zu verzweigen.

    Zu Punkt 3: der Ursprung von `below` ist in A angeschrieben. Sitzt N mit
    T_neu unter A, so beschreibt derselbe Zahlensatz jetzt eine Lage relativ
    zu N und der ganze Teilbaum hinter B wandert um T_neu mit. Soll er
    stehenbleiben, bekommt `below` neu

        T_rest = T_neu^-1 * T_alt

    denn dann gilt wieder T_neu * T_rest = T_alt. Beides kann gewollt sein --
    ein Sensorframe zieht man nachtraeglich ein, ohne dass sich der Roboter
    bewegt, ein echtes Zwischenstueck verlaengert die Kette absichtlich --
    deshalb entscheidet der Aufrufer und nicht diese Funktion.
    """
    existing = next((j for j in root.findall("joint")
                     if j.get("name") == below), None)
    if existing is None:
        raise ValueError(f"Gelenk {below} ist nicht mehr im Baum")
    parent_element = existing.find("parent")
    if parent_element is None:
        raise ValueError(f"Gelenk {below} hat kein <parent>")

    # Erst das neue Glied, dann umhaengen. Andere Reihenfolge ginge auch, so
    # ist der Entwurf aber nie in einem Zustand ohne Ziel fuer <parent>.
    joint = append_limb(root, link_name, joint_name, jtype,
                        parent_element.get("link"),
                        xyz, rpy, axis, lower, upper)

    if keep_pose:
        origin = existing.find("origin")
        old = from_rpy_xyz(vec(origin, "rpy"), vec(origin, "xyz"))
        rest = multiply(invert(from_rpy_xyz(rpy, xyz)), old)
        rest_xyz, rest_rpy = to_xyz_rpy(rest)
        if origin is None:
            origin = ET.SubElement(existing, "origin")
        origin.set("xyz", fmt_origin(rest_xyz))
        origin.set("rpy", fmt_origin(rest_rpy))

    parent_element.set("link", link_name)
    return joint


def rename_link(root, old, new):
    """Einen Link umbenennen und alle Verweise darauf mitziehen.

    Ein Name steht in URDF an mehr Stellen als der einen, an der er vergeben
    wird. Wird nur <link name> geaendert, zeigen die Gelenke ins Leere und die
    Pruefung meldet einen zweiten Wurzel-Link -- der Baum zerfaellt.

    Mitgezogen werden:
        <link name>                    die Vergabe selbst
        <joint><parent link>           Verweis von oben
        <joint><child link>            Verweis von unten
        <gazebo reference>             Verweis aus der Simulation
        <material name>                nur das eigene, siehe unten

    Das Material wird nur umbenannt, wenn es der Regel {link}_material folgt,
    also von set_visual angelegt wurde und niemandem sonst gehoert. Ein
    gemeinsames Material bleibt unangetastet, sonst faerbte ein Umbenennen
    fremde Links um.
    """
    element = next((l for l in root.findall("link")
                    if l.get("name") == old), None)
    if element is None:
        raise ValueError(f"Link {old} ist nicht mehr im Baum")
    element.set("name", new)

    for joint in root.findall("joint"):
        for side in ("parent", "child"):
            hook = joint.find(side)
            if hook is not None and hook.get("link") == old:
                hook.set("link", new)

    for tag in root.findall("gazebo"):
        if tag.get("reference") == old:
            tag.set("reference", new)

    for material in element.iter("material"):
        if material.get("name") == f"{old}_material":
            material.set("name", f"{new}_material")
    return element


def rename_joint(root, old, new):
    """Ein Gelenk umbenennen und alle Verweise darauf mitziehen.

    Anders als beim Link haengt an einem Gelenknamen keine Topologie: parent
    und child stehen im Gelenk selbst. Verweise gibt es trotzdem, und sie sind
    leichter zu uebersehen, weil sie ausserhalb des Gelenks stehen:

        <joint name>                   die Vergabe selbst
        <mimic joint>                  ein Gelenk, das diesem folgt
        <transmission><joint name>     Zuordnung zum Antrieb
        <gazebo reference>             Verweis aus der Simulation
    """
    element = next((j for j in root.findall("joint")
                    if j.get("name") == old), None)
    if element is None:
        raise ValueError(f"Gelenk {old} ist nicht mehr im Baum")
    element.set("name", new)

    for mimic in root.iter("mimic"):
        if mimic.get("joint") == old:
            mimic.set("joint", new)

    for transmission in root.findall("transmission"):
        for joint in transmission.findall("joint"):
            if joint.get("name") == old:
                joint.set("name", new)

    for tag in root.findall("gazebo"):
        if tag.get("reference") == old:
            tag.set("reference", new)
    return element


def remove_limb(root, joint_name, keep_pose):
    """Ein einzelnes Glied aus der Kette nehmen, ohne den Rest zu verlieren.

    Aus   A --joint_name--> B --k--> C   wird   A --k--> C.

    Die Umkehrung von insert_limb und der Gegenpol zu remove_branch: dort
    faellt alles hinter dem Gelenk weg, hier nur das eine Glied. Was daran
    hing, wandert eine Stufe hoch.

    Link und Gelenk verschwinden zwangslaeufig gemeinsam, so wie sie in
    append_limb gemeinsam entstehen. Bliebe das Gelenk ohne Kindlink, haette
    es kein Ziel; bliebe der Link ohne zufuehrendes Gelenk, waere er ein
    zweiter Wurzel-Link. Beides waere keine gueltige URDF.

    Zu `keep_pose`: die Urspruenge der Kindgelenke sind in B angeschrieben.
    Faellt B weg, muessen sie in A stehen, also

        T_neu = T_joint_name * T_alt

    denn genau diese Verkettung lief vorher ueber B. Ist das Gelenk beweglich,
    gilt die Rechnung fuer Stellung null: mit dem Gelenk verschwindet auch
    sein Freiheitsgrad, eine andere Stellung liesse sich gar nicht erhalten.
    """
    element = next((j for j in root.findall("joint")
                    if j.get("name") == joint_name), None)
    if element is None:
        raise ValueError(f"Gelenk {joint_name} ist nicht mehr im Baum")
    parent_element = element.find("parent")
    child_element = element.find("child")
    if parent_element is None or child_element is None:
        raise ValueError(f"Gelenk {joint_name} hat kein <parent> oder <child>")

    above = parent_element.get("link")
    gone = child_element.get("link")
    origin = element.find("origin")
    lift = from_rpy_xyz(vec(origin, "rpy"), vec(origin, "xyz"))

    for other in root.findall("joint"):
        hook = other.find("parent")
        if hook is None or hook.get("link") != gone:
            continue
        if keep_pose:
            child_origin = other.find("origin")
            rest = multiply(lift, from_rpy_xyz(vec(child_origin, "rpy"),
                                               vec(child_origin, "xyz")))
            rest_xyz, rest_rpy = to_xyz_rpy(rest)
            if child_origin is None:
                child_origin = ET.SubElement(other, "origin")
            child_origin.set("xyz", fmt_origin(rest_xyz))
            child_origin.set("rpy", fmt_origin(rest_rpy))
        hook.set("link", above)

    root.remove(element)
    for link in list(root.findall("link")):
        if link.get("name") == gone:
            root.remove(link)
    drop_stale_mimics(root)
    return gone


def remove_branch(root, link_names):
    """Links und alle Gelenke entfernen, die dorthin fuehren."""
    names = set(link_names)
    for link in list(root.findall("link")):
        if link.get("name") in names:
            root.remove(link)
    for joint in list(root.findall("joint")):
        child = joint.find("child")
        if child is not None and child.get("link") in names:
            root.remove(joint)
    drop_stale_mimics(root)


def backup_dir_for(path):
    """Ablageordner, wird bei Bedarf angelegt."""
    if os.path.isabs(BACKUP_DIR):
        folder = BACKUP_DIR
    else:
        folder = os.path.join(os.path.dirname(os.path.abspath(path)), BACKUP_DIR)
    os.makedirs(folder, exist_ok=True)
    return folder


STAMP = re.compile(r"_\d{8}_\d{6}$")


def timestamped_name(path):
    """Dateiname mit Zeitstempel, Endung bleibt erhalten.

    Ein bereits vorhandener Stempel wird ersetzt, sonst haengt sich beim
    Speichern einer Ablagedatei ein zweiter an den Namen.
    """
    stem, suffix = os.path.splitext(os.path.basename(path))
    return f"{STAMP.sub('', stem)}_{datetime.now():%Y%m%d_%H%M%S}{suffix or '.urdf'}"


def save_urdf(path, xml_text):
    """Atomar schreiben. Wird eine vorhandene Datei ueberschrieben, kommt
    vorher eine Sicherungskopie in den Ablageordner."""
    backup = None
    if os.path.exists(path):
        backup = os.path.join(backup_dir_for(path), timestamped_name(path))
        shutil.copy2(path, backup)

    # Die temporaere Datei muss im Zielordner liegen. Ueber Dateisystemgrenzen
    # hinweg kopiert os.replace und ist dann nicht mehr atomar.
    handle_fd, tmp = tempfile.mkstemp(dir=os.path.dirname(os.path.abspath(path)),
                                      suffix=".tmp")
    try:
        with os.fdopen(handle_fd, "w") as handle:
            handle.write(xml_text)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(tmp, path)                          # atomar im selben Dateisystem
    except BaseException:
        if os.path.exists(tmp):
            os.unlink(tmp)
        raise
    return backup


def duplicates(names):
    """Namen, die mehr als einmal vorkommen."""
    seen, twice = set(), set()
    for name in names:
        if name in seen:
            twice.add(name)
        seen.add(name)
    return sorted(twice)


def validate_root(root):
    """Prueft den Baum. Gibt eine Liste von Fehlermeldungen zurueck.

    Die Pruefung ist die einzige Instanz, die den Bediener vor einem kaputten
    Modell schuetzt: robot_state_publisher rechnet nur Vorwaertskinematik und
    beanstandet nichts.
    """
    errors = []
    joints = root.findall("joint")
    link_elements = root.findall("link")
    link_names = [l.get("name") for l in link_elements]
    links = {n for n in link_names if n}
    children = []

    # -- Namen ------------------------------------------------------------
    if sum(1 for n in link_names if not (n or "").strip()):
        errors.append("Ein Link hat keinen Namen")
    for name in duplicates([n for n in link_names if n]):
        errors.append(f'Linkname "{name}" ist mehrfach vergeben')
    for name in duplicates([j.get("name") for j in joints if j.get("name")]):
        errors.append(f'Gelenkname "{name}" ist mehrfach vergeben')

    # -- Kopplungen -------------------------------------------------------
    joint_names = {j.get("name") for j in joints}
    followers = {}
    for joint in joints:
        mimic = joint.find("mimic")
        if mimic is None:
            continue
        name, source = joint.get("name"), mimic.get("joint")
        if not source:
            errors.append(f"{name}: <mimic> ohne Gelenkangabe")
        elif source not in joint_names:
            errors.append(f'{name}: folgt "{source}", das es nicht gibt')
        elif source == name:
            errors.append(f"{name}: folgt sich selbst")
        else:
            followers[name] = source
    for name, source in followers.items():
        if source in followers:
            errors.append(f"{name}: folgt {source}, das selbst "
                          f"{followers[source]} folgt. Ketten sind nicht "
                          f"vorgesehen.")

    for joint in joints:
        name = joint.get("name")
        if not (name or "").strip():
            errors.append("Ein Gelenk hat keinen Namen")
            continue

        # -- Typ ----------------------------------------------------------
        jtype = joint.get("type")
        if jtype not in JOINT_TYPES:
            errors.append(f'{name}: Typ "{jtype}" ist keiner der erlaubten '
                          f'({", ".join(JOINT_TYPES)})')

        # -- Verknuepfung -------------------------------------------------
        for role in ("parent", "child"):
            element = joint.find(role)
            if element is None:
                errors.append(f"{name}: <{role}> fehlt")
                continue
            link = element.get("link")
            if link not in links:
                errors.append(f'{name}: Link "{link}" ist nicht deklariert')
            if role == "child":
                children.append(link)

        # -- Anschlaege ---------------------------------------------------
        limit = joint.find("limit")
        if jtype in NEEDS_LIMIT:
            if limit is None:
                errors.append(f"{name}: {jtype} braucht <limit>, sonst ist die "
                              f"Achse unbegrenzt")
            else:
                for required in ("effort", "velocity"):
                    if limit.get(required) is None:
                        errors.append(f"{name}: <limit> ohne {required}")
                try:
                    lower = float(limit.get("lower", 0.0))
                    upper = float(limit.get("upper", 0.0))
                except ValueError:
                    errors.append(f"{name}: Grenzwert ist keine Zahl")
                else:
                    if lower >= upper:
                        errors.append(f"{name}: untere Grenze muss kleiner als "
                                      f"die obere sein")

        # -- Achse --------------------------------------------------------
        if jtype in NEEDS_AXIS:
            axis = joint.find("axis")
            if axis is not None:
                try:
                    values = [float(v) for v in (axis.get("xyz") or "").split()]
                except ValueError:
                    values = []
                if len(values) != 3:
                    errors.append(f"{name}: <axis> braucht drei Zahlen")
                elif math.sqrt(sum(v * v for v in values)) < 1e-9:
                    errors.append(f"{name}: <axis> ist der Nullvektor")

    if len(set(children)) != len(children):
        errors.append(f"Mehrere Eltern fuer: {', '.join(duplicates(children))}")

    roots = links - set(children)
    if len(roots) != 1:
        errors.append(f"Genau ein Wurzel-Link erwartet, gefunden: {len(roots)}")

    # -- Kreise -----------------------------------------------------------
    # Ein Kreis faellt sonst durch: jeder seiner Links hat genau einen Vater
    # und ist damit kein Wurzel-Link. Die FK laesst ihn stillschweigend weg.
    reachable = set(roots)
    pending = [j for j in joints
               if j.find("parent") is not None and j.find("child") is not None]
    while pending:
        progressed = False
        for joint in list(pending):
            if joint.find("parent").get("link") in reachable:
                reachable.add(joint.find("child").get("link"))
                pending.remove(joint)
                progressed = True
        if not progressed:
            break
    unreachable = sorted(links - reachable)
    if unreachable:
        errors.append(f"Nicht von der Wurzel erreichbar, vermutlich ein Kreis: "
                      f"{', '.join(unreachable)}")

    return errors


# ==========================================================================
# Vorwaertskinematik
# ==========================================================================

def local_transform(joint, value):
    """Parent -> Child bei Gelenkwert `value` (rad bzw. m)."""
    if joint["type"] in ("revolute", "continuous"):
        return multiply(joint["origin"], from_axis_angle(joint["axis"], value))
    if joint["type"] == "prismatic":
        return multiply(joint["origin"],
                        from_translation([a * value for a in joint["axis"]]))
    return joint["origin"]


def root_links(links, joints):
    """Links, die in keinem <child> vorkommen."""
    children = {j["child"] for j in joints}
    return [l["name"] for l in links if l["name"] not in children]


def leaf_links(links, joints):
    """Links, die in keinem <parent> vorkommen. Kandidaten fuer den TCP."""
    parents = {j["parent"] for j in joints}
    return [l["name"] for l in links if l["name"] not in parents]


def tree_layout(links, joints):
    """Mitte und Oberkante jedes Linkkastens. Gibt {linkname: (x, y)} zurueck.

    Zweipass, wie beim Setzen einer Tabelle:

        1. von unten nach oben: wie breit ist der Teilbaum unter einem Link,
        2. von oben nach unten: diese Breiten nebeneinander legen und jeden
           Link mittig ueber seine Kinder setzen.

    Ohne den ersten Durchlauf wuesste man beim Platzieren des Vaters noch
    nicht, wie viel Platz die Kinder brauchen; man muesste raten und spaeter
    verschieben. Die Tiefe bestimmt allein die Zeile, deshalb liegen alle
    Kinder eines Links garantiert auf einer Hoehe.

    `seen` bricht Kreise ab. Ein Kreis ist keine gueltige URDF, aber die
    Ansicht wird auch fuer eine geladene Datei gezeichnet, die die Pruefung
    nur bemaengelt hat. Links, die von keiner Wurzel aus erreichbar sind,
    tauchen nicht auf; der Aufrufer sieht das am Vergleich der Anzahlen.
    """
    kids = {}
    for joint in joints:
        kids.setdefault(joint["parent"], []).append(joint)

    width = {}

    def measure(link, seen):
        if link in seen:
            return TREE_NODE_W
        seen = seen | {link}
        spans = [measure(j["child"], seen) for j in kids.get(link, ())]
        total = sum(spans) + TREE_GAP_X * (len(spans) - 1) if spans else 0.0
        width[link] = max(TREE_NODE_W, total)
        return width[link]

    def place(link, left, depth, seen):
        if link in seen:
            return
        seen = seen | {link}
        nodes[link] = (left + width[link] / 2.0,
                       TREE_MARGIN + depth * (TREE_NODE_H + TREE_GAP_Y))
        offset = left
        for joint in kids.get(link, ()):
            child = joint["child"]
            if child in seen:
                continue
            place(child, offset, depth + 1, seen)
            offset += width[child] + TREE_GAP_X

    nodes = {}
    left = TREE_MARGIN
    # Mehrere Wurzeln sind ungueltig, kommen beim Laden einer fremden Datei
    # aber vor. Dann stehen die Baeume nebeneinander, statt dass die Ansicht
    # leer bleibt und der Fehler unsichtbar wird.
    for root in root_links(links, joints):
        measure(root, frozenset())
        place(root, left, 0, frozenset())
        left += width[root] + 3 * TREE_GAP_X
    return nodes


def forward_kinematics(links, joints, positions):
    """{linkname: 4x4} relativ zum Wurzel-Link, in einem Durchlauf."""
    roots = root_links(links, joints)
    if not roots:
        return {}                                      # Zyklus, nichts berechenbar

    poses = {roots[0]: identity()}
    pending = list(joints)
    while pending:
        progressed = False
        for joint in list(pending):
            if joint["parent"] in poses:
                poses[joint["child"]] = multiply(
                    poses[joint["parent"]],
                    local_transform(joint, positions.get(joint["name"], 0.0)))
                pending.remove(joint)
                progressed = True
        if not progressed:
            break                                      # abgehaengter Teilbaum
    return poses

# ==========================================================================
# Bahnrekorder
# ==========================================================================

def angle_delta(a, b):
    """Kleinster Winkelabstand in rad, ueber +-pi hinweg."""
    return abs((a - b + math.pi) % (2.0 * math.pi) - math.pi)


# ==========================================================================
# MoveIt: Konfiguration, SRDF und Kindprozess
# ==========================================================================

# Name der Planungsgruppe in der erzeugten SRDF. Er muss zu dem passen, was
# spaeter im MotionPlanRequest steht, deshalb steht er hier einmal.
MOVEIT_GROUP = "manipulator"

# Unendlich, einmal benannt. Eine Achsgrenze kann so gemeint sein.
INF = float("inf")

# Wie lange nach dem Start auf move_group gewartet wird, bevor die Anzeige
# von "startet" auf "keine Antwort" wechselt.
MOVEIT_WAIT_S = 25.0

# Frist fuer eine Planungsanfrage. Laeuft sie ab, kommt ein Fehler in die
# Warteschlange und die Ablaufsteuerung verlaesst PLANEN. Ohne diese Frist
# haengt sie dort, wenn niemand antwortet: das Future wird dann nie fertig,
# und der Rueckweg in die GUI fuehrt allein ueber dessen Callback.
# Zuschlag auf die Planungszeit des Auftrags. Die Frist ergibt sich daraus,
# damit eine hochgesetzte Planungszeit nicht gegen eine feste Grenze laeuft.
PLAN_TIMEOUT_MARGIN_S = 10.0

# Vorsilbe der Arbeitsordner. Steht hier, weil beim Start auch die Reste
# frueherer Sitzungen damit gesucht werden.
WORKDIR_PREFIX = "urdf_gui_moveit_"

# Beschleunigung und Ruck stehen nicht in der URDF. Fehlen sie, werden sie
# aus der Geschwindigkeit abgeleitet. Bewusst grosszuegig: die Werte sind
# Startwerte zum Ueberschreiben, keine Behauptung ueber die Mechanik.
ACC_FROM_VEL = 2.0
JERK_FROM_ACC = 10.0
DEFAULT_JOINT_VELOCITY = 1.0                           # rad/s bzw. m/s

# Felder der Karteikarte Achsgrenzen. Ein leeres Feld heisst: Grenze aus,
# also has_*_limits false. Das spart je Achse vier Kaestchen.
LIMIT_FIELDS = [
    ("velocity",     "v",     "rad/s",  70),
    ("acceleration", "a",     "rad/s2", 70),
    ("deceleration", "dec",   "rad/s2", 70),
    ("jerk",         "jerk",  "rad/s3", 70),
]

# Kartesische Grenzen fuer Pilz. Eine eigene Rotationsbeschleunigung gibt es
# nicht, sie wird aus dem translatorischen Verhaeltnis abgeleitet.
CARTESIAN_FIELDS = [
    ("max_trans_vel", "Bahngeschwindigkeit", "m/s",   1.000),
    ("max_trans_acc", "Bahnbeschleunigung",  "m/s2",  2.250),
    ("max_trans_dec", "Bahnverzoegerung",    "m/s2", -5.000),
    ("max_rot_vel",   "Drehgeschwindigkeit", "rad/s", 1.570),
]

KINEMATICS_SOLVERS = (
    "kdl_kinematics_plugin/KDLKinematicsPlugin",
    "trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin",
    "lma_kinematics_plugin/LMAKinematicsPlugin",
)

KINEMATICS_FIELDS = [
    ("kinematics_solver_timeout",           "Zeitgrenze je Aufruf", "s", 0.050),
    ("kinematics_solver_search_resolution", "Suchschrittweite",   "rad", 0.005),
]

# Werte, die pro Auftrag in den MotionPlanRequest wandern. Sie brauchen
# keinen Neustart, deshalb stehen sie getrennt von allem anderen.
PLAN_FIELDS = [
    ("velocity_scaling",     "Override Geschwindigkeit", "%",   100.0),
    ("acceleration_scaling", "Override Beschleunigung",  "%",   100.0),
    ("planning_time",        "Planungszeit",             "s",     5.0),
    ("planning_attempts",    "Versuche",                 "",      1.0),
    ("goal_position_tol",    "Zieltoleranz Lage",        "m",   0.001),
    ("goal_orientation_tol", "Zieltoleranz Drehung",     "deg",  0.100),
    ("max_step",             "Stuetzpunktabstand LIN",   "m",   0.010),
    ("jump_threshold",       "Sprungschwelle",           "",      2.0),
    ("blend_radius",         "Ueberschleifradius",       "m",   0.000),
]

PLANNER_IDS = ("PTP", "LIN", "CIRC")


def yaml_scalar(value):
    """Einen Wert als YAML schreiben. Nur die Faelle, die hier vorkommen."""
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, float):
        if value != value:                             # NaN ist ungleich sich selbst
            return ".nan"
        if value == INF:
            return ".inf"
        if value == -INF:
            return "-.inf"
        text = f"{value:.10g}"
        # Der Punkt muss stehen bleiben, auch wenn nichts dahinter kommt.
        # YAML liest 2 als Ganzzahl; rclcpp deklariert den Parameter dann als
        # integer, MoveIt erwartet dort ein double, und die Ausnahme daraus
        # wird nicht gefangen -- der Prozess bricht mit terminate() ab. Eine
        # ganze Zahl an dieser Stelle beendet also den Knoten. Das ist der
        # Grund, warum eine Zahl und ihre Schreibweise nicht dasselbe sind.
        if not any(c in text for c in ".eEn"):
            text += ".0"
        return text
    if isinstance(value, int):
        return str(value)
    text = str(value)
    if text == "" or any(c in text for c in ":#{}[],&*?|-<>=!%@`\"'\n"):
        return '"' + text.replace("\\", "\\\\").replace('"', '\\"') + '"'
    return text


def yaml_block(text, indent):
    """Mehrzeiligen Text als Literalblock einruecken.

    Die URDF geht so unveraendert in die Parameterdatei. Sie ueber die
    Kommandozeile zu uebergeben scheitert an den Zeilenumbruechen, deshalb
    fuehrt der Weg immer ueber eine Datei.
    """
    pad = " " * indent
    lines = text.replace("\r\n", "\n").split("\n")
    while lines and not lines[-1].strip():
        lines.pop()
    return "|-\n" + "\n".join(pad + line for line in lines)


def build_srdf(robot_name, links, joints, group_name=MOVEIT_GROUP, tip=None):
    """SRDF aus dem geladenen Modell erzeugen.

    Von Hand gepflegt waere sie nach dem dritten Strukturbefehl kaputt: die
    GUI haengt Glieder an und nimmt sie heraus, und jeder dieser Schritte
    aendert Gruppe und Kollisionspaare. Deshalb entsteht sie hier neu, jedes
    Mal wenn das Modell sich aendert.

    Die Gruppe ist die Kette von der Wurzel zum Tip-Link. Benachbarte Links
    koennen sich nicht treffen, ihr Paar wird abgeschaltet; alles andere
    prueft MoveIt selbst.
    """
    roots = root_links(links, joints)
    base = roots[0] if roots else (links[0]["name"] if links else "base_link")

    if tip is None:
        leaves = leaf_links(links, joints)
        tip = leaves[0] if leaves else base

    lines = ['<?xml version="1.0"?>',
             f'<robot name="{robot_name or "robot"}">',
             f'  <group name="{group_name}">',
             f'    <chain base_link="{base}" tip_link="{tip}"/>',
             '  </group>']

    movable = [j for j in joints if j["type"] != "fixed" and j["mimic"] is None]
    if movable:
        lines.append(f'  <group_state name="null" group="{group_name}">')
        for joint in movable:
            lines.append(f'    <joint name="{joint["name"]}" value="0"/>')
        lines.append('  </group_state>')

    for joint in joints:
        if not joint["parent"] or not joint["child"]:
            continue
        lines.append(f'  <disable_collisions link1="{joint["parent"]}" '
                     f'link2="{joint["child"]}" reason="Adjacent"/>')

    lines.append('</robot>')
    return "\n".join(lines) + "\n"


class MoveItConfig:
    """Alle Werte, die move_group beim Hochfahren liest, plus die pro Auftrag.

    Bewusst ohne Tk und ohne ROS: die Klasse laesst sich ohne Fenster und
    ohne Roboter pruefen. Sie kennt drei Sorten von Werten und haelt sie
    getrennt, weil sie sich unterschiedlich verhalten:

        plan        wandert in den MotionPlanRequest, wirkt sofort
        limits      Achsgrenzen, werden beim Start gelesen
        cartesian   Bahngrenzen fuer Pilz, werden beim Start gelesen
        kinematics  Solver und dessen Parameter, wird beim Start gelesen

    Nur die erste Gruppe kommt ohne Neustart aus. Genau diese Unterscheidung
    zeigt die Oberflaeche dem Bediener an.
    """

    def __init__(self):
        self.plan = {key: default for key, _l, _u, default in PLAN_FIELDS}
        self.planner_id = "PTP"
        self.group = MOVEIT_GROUP
        self.tip = ""                                  # leer: Blatt automatisch
        self.limits = {}                               # {gelenk: {feld: wert|None}}
        self.cartesian = {key: default
                          for key, _l, _u, default in CARTESIAN_FIELDS}
        self.kinematics = {key: default
                           for key, _l, _u, default in KINEMATICS_FIELDS}
        self.solver = KINEMATICS_SOLVERS[0]
        self.use_ruckig = True
        # Kollisionspruefung der Bahn. An heisst: MoveIt prueft Startzustand
        # und jeden Stuetzpunkt gegen die Planungsszene und wirft die ganze
        # Bahn weg, sobald ein Paar sich beruehrt, das die SRDF nicht
        # freigegeben hat. Aus heisst nicht "es kollidiert nicht", sondern
        # "es wird nicht geprueft". Solange die Ersatzkoerper grob sind und
        # die Kollisionsmatrix nur benachbarte Paare kennt, ist das der
        # ehrlichere Zustand -- eine Pruefung, die falsche Treffer meldet,
        # ist keine Sicherheit, sondern nur ein Hindernis.
        self.check_collisions = True
        self.allow_execution = False                   # die GUI faehrt selbst

    # -- Achsgrenzen -------------------------------------------------------

    @staticmethod
    def defaults_for(joint):
        """Startwerte aus der URDF ableiten.

        Geschwindigkeit steht in <limit velocity=...>, Beschleunigung und Ruck
        stehen dort nicht und koennen es auch nicht: die URDF kennt sie nicht.
        Sie werden deshalb abgeleitet und sind zum Ueberschreiben gedacht.
        """
        velocity = joint.get("velocity")
        try:
            velocity = float(velocity)
        except (TypeError, ValueError):
            velocity = 0.0
        if velocity <= 0.0:
            velocity = DEFAULT_JOINT_VELOCITY
        acceleration = velocity * ACC_FROM_VEL
        return {"velocity": velocity,
                "acceleration": acceleration,
                "deceleration": -acceleration,
                "jerk": acceleration * JERK_FROM_ACC}

    def sync_joints(self, joints):
        """Eintraege an das Modell anpassen.

        Neue Gelenke bekommen Startwerte, verschwundene fallen heraus.
        Bestehende bleiben unveraendert -- ein Umbenennen des Gelenks setzt
        seine Grenzen also zurueck, und das ist richtig so: es ist aus Sicht
        der Konfiguration ein anderes Gelenk.
        """
        wanted = [j for j in joints
                  if j["type"] != "fixed" and j["mimic"] is None]
        names = {j["name"] for j in wanted}
        for name in list(self.limits):
            if name not in names:
                del self.limits[name]
        for joint in wanted:
            if joint["name"] not in self.limits:
                self.limits[joint["name"]] = self.defaults_for(joint)
        return [j["name"] for j in wanted]

    # -- Parameterdatei ----------------------------------------------------

    def limits_yaml(self, indent):
        pad = " " * indent
        out = []
        for name in sorted(self.limits):
            values = self.limits[name]
            out.append(f"{pad}{name}:")
            for key, has in (("velocity", "has_velocity_limits"),
                             ("acceleration", "has_acceleration_limits"),
                             ("jerk", "has_jerk_limits")):
                value = values.get(key)
                out.append(f"{pad}  {has}: {yaml_scalar(value is not None)}")
                if value is not None:
                    out.append(f"{pad}  max_{key}: {yaml_scalar(float(value))}")
            # Pilz-Erweiterung. Der Wert ist negativ, das ist dort so gewollt.
            value = values.get("deceleration")
            out.append(f"{pad}  has_deceleration_limits: "
                       f"{yaml_scalar(value is not None)}")
            if value is not None:
                out.append(f"{pad}  max_deceleration: {yaml_scalar(float(value))}")
        return "\n".join(out)

    def params_yaml(self, urdf_xml, srdf_xml):
        """Die vollstaendige Parameterdatei fuer move_group.

        Die Pfade der Parameter haengen an der MoveIt-Fassung. Sie stehen
        deshalb hier an einer Stelle und nicht verteilt im Programm.
        """
        # Zwei Adapter tragen die Kollisionspruefung, je einer auf jeder
        # Seite: CheckStartStateCollision lehnt den Auftrag vorher ab,
        # ValidateSolution wirft die fertige Bahn nachher weg. Ein Schalter
        # fuer beide, sonst ist "aus" nur halb aus.
        requests = ["default_planning_request_adapters/ResolveConstraintFrames",
                    "default_planning_request_adapters/ValidateWorkspaceBounds",
                    "default_planning_request_adapters/CheckStartStateBounds"]
        if self.check_collisions:
            requests.append(
                "default_planning_request_adapters/CheckStartStateCollision")

        adapters = ["default_planning_response_adapters/DisplayMotionPath"]
        if self.check_collisions:
            adapters.insert(
                0, "default_planning_response_adapters/ValidateSolution")
        if self.use_ruckig:
            adapters.insert(
                0, "default_planning_response_adapters/AddRuckigTrajectorySmoothing")

        lines = [
            "/**:",
            "  ros__parameters:",
            "    robot_description: " + yaml_block(urdf_xml, 6),
            "    robot_description_semantic: " + yaml_block(srdf_xml, 6),
            "    robot_description_kinematics:",
            f"      {self.group}:",
            f"        kinematics_solver: {yaml_scalar(self.solver)}",
        ]
        for key, _label, _unit, _default in KINEMATICS_FIELDS:
            lines.append(f"        {key}: "
                         f"{yaml_scalar(float(self.kinematics[key]))}")

        lines += [
            "    robot_description_planning:",
            "      joint_limits:",
            self.limits_yaml(8),
            "      cartesian_limits:",
        ]
        for key, _label, _unit, _default in CARTESIAN_FIELDS:
            lines.append(f"        {key}: "
                         f"{yaml_scalar(float(self.cartesian[key]))}")

        lines += [
            "    planning_pipelines: [\"pilz_industrial_motion_planner\"]",
            "    default_planning_pipeline: pilz_industrial_motion_planner",
            "    pilz_industrial_motion_planner:",
            "      planning_plugins:",
            "        - pilz_industrial_motion_planner/CommandPlanner",
            "      request_adapters:",
        ]
        for adapter in requests:
            lines.append(f"        - {adapter}")
        lines.append("      response_adapters:")
        for adapter in adapters:
            lines.append(f"        - {adapter}")

        lines += [
            "    capabilities: \"pilz_industrial_motion_planner/MoveGroupSequenceAction "
            "pilz_industrial_motion_planner/MoveGroupSequenceService\"",
            # Ohne Controller waere jede Ausfuehrung ein Fehler. Die GUI
            # tastet die geplante Bahn selbst ab, deshalb bleibt das aus.
            f"    allow_trajectory_execution: {yaml_scalar(self.allow_execution)}",
            "    moveit_manage_controllers: false",
            "    publish_planning_scene: true",
            "    publish_geometry_updates: true",
            "    publish_state_updates: true",
            "    publish_transforms_updates: true",
        ]
        return "\n".join(lines) + "\n"

    # -- Vergleich ---------------------------------------------------------

    def startup_signature(self):
        """Alles, was einen Neustart erfordert, als vergleichbarer Text."""
        parts = [self.group, self.tip, self.solver, str(self.use_ruckig),
                 str(self.check_collisions)]
        for name in sorted(self.limits):
            values = self.limits[name]
            parts.append(name + "=" + ",".join(
                f"{key}:{values.get(key)}" for key, _l, _u, _w in LIMIT_FIELDS))
        for key, _l, _u, _d in CARTESIAN_FIELDS:
            parts.append(f"{key}:{self.cartesian[key]}")
        for key, _l, _u, _d in KINEMATICS_FIELDS:
            parts.append(f"{key}:{self.kinematics[key]}")
        return "|".join(parts)


def find_move_group():
    """Pfad der move_group-Binary aus AMENT_PREFIX_PATH.

    Der Umweg ueber "ros2 run" kostet einen zusaetzlichen Prozess, und dieser
    eine Prozess ist der Grund fuer den Fehler, den 2.5 behebt: der Wrapper
    nimmt das Signal entgegen, das Kind ueberlebt ihn. Wer den Pfad kennt,
    braucht ihn nicht. Gefunden wird er ueber dieselbe Umgebungsvariable, die
    auch "ros2 run" durchsucht -- wer die GUI ohne gesourcetes Setup startet,
    findet hier nichts, und dann findet "ros2 run" auch nichts.
    """
    for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
        if not prefix:
            continue
        path = os.path.join(prefix, "lib", "moveit_ros_move_group", "move_group")
        if os.access(path, os.X_OK):
            return path
    return None


def move_group_pids():
    """PIDs aller laufenden move_group-Prozesse, ueber /proc.

    Bewusst ohne pgrep: das ist ein weiterer Prozess, dessen Ausgabe geparst
    werden muesste, und unter WSL ist es nicht in jeder Ausbaustufe da. /proc
    ist da, sobald Linux laeuft.
    """
    found = []
    try:
        entries = os.listdir("/proc")
    except OSError:
        return found
    for entry in entries:
        if not entry.isdigit():
            continue
        try:
            with open(f"/proc/{entry}/cmdline", "rb") as handle:
                parts = [p for p in handle.read().split(b"\0") if p]
        except OSError:                                # Prozess ist gerade weg
            continue
        if parts and parts[0].rsplit(b"/", 1)[-1] == b"move_group":
            found.append(int(entry))
    return found


class MoveItProcess:
    """move_group als Kindprozess der GUI.

    MoveIt liest Robotermodell, SRDF, Achsgrenzen und Solver beim Hochfahren
    und danach nicht mehr. Ein Modellwechsel zur Laufzeit ist dort nicht
    vorgesehen. Statt zu versuchen, den Knoten am Leben zu halten, macht die
    GUI den Neustart zur normalen Handlung: sie schreibt die Parameterdatei
    selbst und faehrt den Prozess damit neu hoch.

    Damit das trage, muss der alte Prozess auch wirklich weg sein. Er ist es
    seit 2.5: das Signal geht an die Prozessgruppe, und gestartet wird ohne
    Wrapper. Zusaetzlich traegt jeder Start einen eigenen Knotennamen, damit
    im Graphen zu erkennen ist, welcher Knoten zu dieser GUI gehoert.

    Zustaende: aus, startet, laeuft, beendet, fehler.
    """

    def __init__(self, log=None):
        self.log = log or (lambda text, level="info": None)
        self.process = None
        self.workdir = None
        self.started_at = None
        self.signature = None                          # Stand, mit dem gestartet wurde
        self.last_error = ""
        self.node_name = ""                            # Knoten des laufenden Starts
        self.logfile = None                            # Ausgabe von move_group
        self.command = []                              # zum Anzeigen im Protokoll
        self.pgid = None                               # Prozessgruppe des Starts
        self._serial = 0                               # zaehlt die Starts

    # -- Zustand -----------------------------------------------------------

    @property
    def alive(self):
        return self.process is not None and self.process.poll() is None

    def state(self):
        if self.process is None:
            return "aus"
        code = self.process.poll()
        if code is None:
            if self.started_at is not None and \
                    time.monotonic() - self.started_at < 3.0:
                return "startet"
            return "laeuft"
        return "beendet" if code == 0 else "fehler"

    def uptime(self):
        if not self.alive or self.started_at is None:
            return None
        return time.monotonic() - self.started_at

    def strays(self):
        """Fremde move_group-Prozesse: alle ausser dem eigenen.

        Der eigene ist nach dem Umbau derselbe Prozess, den Popen zurueckgibt.
        Bleibt hier trotzdem etwas uebrig, dann laeuft ein move_group, den
        diese GUI nicht gestartet hat -- aus einer frueheren Sitzung, aus
        einem Launchfile oder von Hand.
        """
        mine = self.process.pid if self.alive else None
        return [pid for pid in move_group_pids() if pid != mine]

    def log_tail(self, lines=12, window=65536):
        """Die letzten Zeilen der Ausgabe. Leer, solange nichts geschrieben ist.

        Gelesen wird nur das letzte Stueck der Datei, nicht die ganze. Der
        Unterschied ist nicht theoretisch: ValidateSolution schreibt zu jedem
        gefundenen Kontakt zwei Zeilen, ein gescheiterter Auftrag kommt so
        auf mehrere hundert, und nach einer Viertelstunde Fehlersuche liegt
        dort ein zweistelliger Megabytebetrag. readlines() zog den komplett
        in den Speicher, und zwar im GUI-Thread -- das Fenster stand still,
        genau in dem Augenblick, in dem es erklaeren sollte, was schiefging.

        Ein Fenster von 64 kB traegt bei diesen Zeilenlaengen rund 400
        Zeilen. Die erste davon ist womoeglich angeschnitten, deshalb faellt
        sie heraus, wenn nicht vom Dateianfang gelesen wurde.
        """
        if not self.logfile or not os.path.isfile(self.logfile):
            return []
        try:
            size = os.path.getsize(self.logfile)
            with open(self.logfile, "rb") as handle:
                start = max(0, size - window)
                handle.seek(start)
                raw = handle.read().decode("utf-8", errors="replace")
        except OSError:
            return []
        parts = raw.split("\n")
        if parts and parts[-1] == "":
            parts.pop()                                # Umbruch am Dateiende
        if start > 0 and parts:
            parts = parts[1:]                          # angeschnittene Zeile
        return [line.rstrip() for line in parts[-lines:] if line.strip()]

    def log_size(self):
        """Groesse der Protokolldatei in Byte, 0 wenn es sie nicht gibt."""
        if not self.logfile:
            return 0
        try:
            return os.path.getsize(self.logfile)
        except OSError:
            return 0

    # -- Lebenszyklus ------------------------------------------------------

    def start(self, config, urdf_xml, srdf_xml):
        """Parameterdatei schreiben und move_group starten.

        Gibt None zurueck oder eine Meldung. Ein laufender Prozess wird
        vorher beendet -- zwei move_group auf denselben Diensten waeren eine
        Fehlersuche ohne Ende, und genau die war es bis 2.4.
        """
        self.stop()

        self._serial += 1
        # Der Name muss ueber die Lebensdauer der GUI eindeutig sein, nicht
        # global. PID und laufende Nummer reichen dafuer und sind lesbar.
        self.node_name = f"move_group_gui{os.getpid()}_{self._serial}"

        try:
            self.workdir = tempfile.mkdtemp(prefix=WORKDIR_PREFIX)
            params = os.path.join(self.workdir, "move_group.yaml")
            with open(params, "w", encoding="utf-8") as handle:
                handle.write(config.params_yaml(urdf_xml, srdf_xml))
            # Zum Nachsehen, wenn etwas nicht passt. Kostet nichts.
            with open(os.path.join(self.workdir, "robot.urdf"), "w",
                      encoding="utf-8") as handle:
                handle.write(urdf_xml)
            with open(os.path.join(self.workdir, "robot.srdf"), "w",
                      encoding="utf-8") as handle:
                handle.write(srdf_xml)
        except OSError as exc:
            self.last_error = f"Parameterdatei nicht schreibbar: {exc}"
            return self.last_error

        arguments = ["--ros-args",
                     "--params-file", params,
                     "-r", f"__node:={self.node_name}"]
        binary = find_move_group()
        if binary:
            self.command = [binary] + arguments
        else:
            # Rueckfallweg. Er funktioniert, aber der Wrapper haengt wieder
            # dazwischen; deshalb steht es im Protokoll.
            self.command = ["ros2", "run", "moveit_ros_move_group",
                            "move_group"] + arguments
            self.log("move_group-Binary nicht in AMENT_PREFIX_PATH gefunden, "
                     "starte ueber ros2 run", "warn")

        # Die Ausgabe geht in eine Datei und nicht nach DEVNULL. Ohne sie ist
        # ein Fehlstart nicht aufklaerbar: der Prozess ist nach zwei Sekunden
        # weg, und die Meldung, die es erklaert, steht in seiner letzten Zeile.
        self.logfile = os.path.join(self.workdir, "move_group.log")
        try:
            handle = open(self.logfile, "wb")
        except OSError as exc:
            self.last_error = f"Protokolldatei nicht schreibbar: {exc}"
            return self.last_error

        try:
            self.process = subprocess.Popen(
                self.command,
                stdout=handle,
                stderr=subprocess.STDOUT,
                # Eigene Sitzung, damit ein Strg-C im Terminal der GUI nicht
                # gleich mit durchschlaegt -- und damit killpg unten eine
                # Gruppe hat, die nur diesen Baum enthaelt.
                start_new_session=True)
        except (OSError, ValueError) as exc:
            self.process = None
            self.last_error = f"move_group nicht startbar: {exc}"
            return self.last_error
        finally:
            handle.close()                             # Popen hat eine eigene Kopie

        # Wegen start_new_session ist die Gruppennummer gleich der PID. Sie
        # wird gemerkt, weil sie nach dem Einsammeln des Prozesses nicht mehr
        # zu erfragen ist -- die Gruppe kann dann noch Mitglieder haben.
        try:
            self.pgid = os.getpgid(self.process.pid)
        except OSError:
            self.pgid = self.process.pid

        self.started_at = time.monotonic()
        self.signature = config.startup_signature()
        self.last_error = ""
        self.log(f"move_group gestartet als {self.node_name} "
                 f"(PID {self.process.pid}), Parameter in {self.workdir}", "ok")
        return None

    def _signal_group(self, sig):
        """Ein Signal an die ganze Prozessgruppe.

        Der Kern der Sache. Wird ueber den Rueckfallweg "ros2 run" gestartet,
        ist move_group ein Enkelkind; ein Signal an self.process erreicht nur
        den Wrapper, und das Enkelkind laeuft weiter. start_new_session=True
        legt den ganzen Baum in eine eigene Gruppe, und diese Gruppe wird hier
        adressiert. Ohne Wrapper trifft es denselben einen Prozess wie zuvor.

        Die Gruppennummer stammt aus dem Start und nicht aus getpgid: sobald
        der Wrapper eingesammelt ist, gibt es seine PID nicht mehr, seine
        Gruppe aber schon.
        """
        if self.pgid is None:
            return
        try:
            os.killpg(self.pgid, sig)
        except OSError:
            if self.process is not None:
                try:
                    self.process.send_signal(sig)
                except OSError:
                    pass

    def _group_alive(self):
        """Lebt in der Prozessgruppe noch etwas?

        process.poll() beantwortet diese Frage nicht -- es sagt nur, ob das
        unmittelbare Kind weg ist. Genau dieser Unterschied ist der Fehler
        aus 2.4, und deshalb darf auch die Eskalation der Signale nicht an
        ihm haengen. Signal 0 stellt nichts zu, es prueft nur, ob es etwas
        zuzustellen gaebe.

        poll() davor sammelt den Wrapper ein: ein nicht abgeholter Zombie
        gehoert noch zur Gruppe und wuerde sie als lebendig ausweisen.
        """
        if self.pgid is None:
            return False
        if self.process is not None:
            self.process.poll()
        try:
            os.killpg(self.pgid, 0)
        except OSError:
            return False
        return True

    def _wait(self, timeout):
        """Rueckgabewert oder None, wenn die Zeit nicht gereicht hat."""
        if self.process is None:
            return None
        try:
            return self.process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            return None

    def _wait_group(self, timeout):
        """Warten, bis die Gruppe leer ist. Gibt zurueck, ob sie es wurde."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if not self._group_alive():
                return True
            time.sleep(0.05)
        return not self._group_alive()

    def stop(self, keep_workdir=True):
        """Beenden und aufraeumen. Erst freundlich, dann bestimmt.

        SIGINT zuerst: daran haengt rclcpp seinen geordneten Shutdown, der
        Knoten meldet sich beim Graphen ab und verschwindet sofort aus
        ros2 node list. SIGTERM danach, SIGKILL zuletzt.

        Weitergeschaltet wird, solange die Gruppe nicht leer ist. Am Wrapper
        allein liesse sich das nicht festmachen: der geht schon beim ersten
        Signal, und ein im Hintergrund gestartetes Kind ignoriert SIGINT
        ohnehin -- eine Shell setzt es dort auf ignorieren. Wer nur den
        Wrapper beobachtet, haelt die Sache fuer erledigt und laesst die
        Waise stehen.
        """
        for sig, grace in ((signal.SIGINT, 3.0),
                           (signal.SIGTERM, 3.0),
                           (signal.SIGKILL, 2.0)):
            if not self._group_alive():
                break
            self._signal_group(sig)
            self._wait_group(grace)
        if self._group_alive():
            self.last_error = ("move_group liess sich nicht beenden "
                               f"(Prozessgruppe {self.pgid})")
            self.log(self.last_error, "error")
        self._wait(0.5)                                # Zombie einsammeln
        self.process = None
        self.pgid = None
        self.started_at = None
        self.node_name = ""
        # Der Arbeitsordner bleibt liegen: er ist die einzige Spur, wenn der
        # Start misslungen ist. Aufgeraeumt wird beim naechsten Start der GUI,
        # siehe sweep_workdirs.
        if not keep_workdir and self.workdir and os.path.isdir(self.workdir):
            shutil.rmtree(self.workdir, ignore_errors=True)
            self.workdir = None
            self.logfile = None

    def kill_strays(self):
        """Fremde move_group-Prozesse beenden. Gibt die Zahl zurueck.

        Sie stammen aus einer Sitzung, die nicht sauber beendet wurde, oder
        aus einem Launchfile. Solange sie laufen, bedienen sie dieselben
        Dienste wie der eigene Knoten, und welcher antwortet, entscheidet
        nicht die GUI.
        """
        pids = self.strays()
        for pid in pids:
            for sig in (signal.SIGINT, signal.SIGTERM):
                try:
                    os.kill(pid, sig)
                except OSError:
                    break
                time.sleep(0.3)
                try:
                    os.kill(pid, 0)                    # lebt er noch?
                except OSError:
                    break
            else:
                try:
                    os.kill(pid, signal.SIGKILL)
                except OSError:
                    pass
        return len(pids)

    @staticmethod
    def sweep_workdirs(keep=None, max_age_s=0.0):
        """Liegengebliebene Arbeitsordner entfernen. Gibt die Zahl zurueck.

        Wird beim Start der GUI gerufen. Ordner, die juenger sind als
        max_age_s, bleiben stehen: es koennte eine zweite GUI daneben laufen.
        """
        base = tempfile.gettempdir()
        removed = 0
        now = time.time()
        try:
            entries = os.listdir(base)
        except OSError:
            return 0
        for name in entries:
            if not name.startswith(WORKDIR_PREFIX):
                continue
            path = os.path.join(base, name)
            if keep and os.path.abspath(path) == os.path.abspath(keep):
                continue
            try:
                if now - os.path.getmtime(path) < max_age_s:
                    continue
            except OSError:
                continue
            shutil.rmtree(path, ignore_errors=True)
            removed += 1
        return removed


# ==========================================================================
# Bahn abfahren
# ==========================================================================

# So viele Ausnahmen vertraegt der ROS-Takt, bevor er aufgibt. Einer allein
# darf ihn nicht beenden, endloses Wiederholen aber auch nicht.
SPIN_ERROR_LIMIT = 20

# Ab diesem Alter der letzten TF-Ausgabe meldet der Waechter. Beim Takt von
# TF_RATE_HZ ist das etwa das Dreissigfache des Erwarteten.
TF_STALL_MS = 1000.0

# Takt, in dem der Executor-Thread nach neuen Auftraegen sieht.
JOB_RATE_HZ = 20.0

# Interpolationstakt der GUI. Reale Steuerungen liegen bei 4 bis 12 ms; hier
# waeren die schoen, aber Tk-after, der Python-GIL, DDS ueber Sockets und
# unter WSL der Hypervisor liegen dazwischen. 20 ms sind unter diesen
# Bedingungen stabil zu halten -- und der Abstand zwischen Soll- und Ist-Takt
# ist genau das, was in der Fusszeile steht und im Unterricht etwas wert ist.
IPO_TICK_MS = 20
IPO_DT = IPO_TICK_MS / 1000.0

# Zeit, in der die Bahn beim Anhalten von voller Rate auf null gefuehrt wird.
# Ein Sprung auf null waere die Verletzung genau der Grenzen, um die es hier
# geht. Die Rampe ist linear in der Zeitrate: einfach, erklaerbar, und sie
# begrenzt die Achsbeschleunigung auf einen bekannten Faktor. Sie ist keine
# ruckbegrenzte Bremsung -- das waere ein eigener Schritt.
BRAKE_TIME_S = 0.40

# Zustaende der Ablaufsteuerung.
STATE_READY = "BEREIT"
STATE_PLANNING = "PLANEN"
STATE_MOVING = "FAHREN"
STATE_BRAKING = "BREMSEN"
STATE_ERROR = "FEHLER"

# Zustaende, in denen nichts veraendert werden darf.
STATES_BUSY = (STATE_PLANNING, STATE_MOVING, STATE_BRAKING)

# moveit_msgs/MoveItErrorCodes. Die nackte Zahl hilft niemandem, und der
# Umgang mit diesen Faellen ist ein guter Teil des Versuchs -- deshalb steht
# neben der Uebersetzung auch, woran es meistens liegt.
MOVEIT_ERRORS = {
    1: ("Erfolg", ""),
    # UNDEFINED, der Vorbelegungswert des Feldes. Er bedeutet nicht "es ist
    # schiefgegangen", sondern "niemand hat geantwortet": der Auftrag ist bei
    # keinem Planer angekommen. Der Grund steht dann nicht in der Antwort,
    # sondern in der Ausgabe von move_group -- deshalb holt _on_plan_result
    # sie in diesem Fall von selbst herein.
    99999: ("Unbestimmt", "Kein Planer hat geantwortet. Pipeline geladen? "
                          "Pilz installiert?"),
    999999: ("Unbestimmt", "Kein Planer hat geantwortet. Pipeline geladen? "
                           "Pilz installiert?"),
    -1: ("Fehlgeschlagen", "Sammelfehler ohne naehere Angabe."),
    -2: ("Planung fehlgeschlagen",
         "Pilz weicht nicht aus. Ziel erreichbar? Achsgrenzen zu eng?"),
    -3: ("Bahn ungueltig",
         "Die gerechnete Bahn hat die Pruefung nicht bestanden."),
    -4: ("Bahn durch Umgebungsaenderung ungueltig", ""),
    -5: ("Ausfuehrung fehlgeschlagen",
         "Betrifft nur die Ausfuehrung durch move_group, nicht diese GUI."),
    -6: ("Keine Sensordaten", ""),
    -7: ("Zeitgrenze ueberschritten",
         "Planungszeit im Parameterfenster erhoehen."),
    -8: ("Abgebrochen", ""),
    -10: ("Startzustand in Kollision",
          "Der Roboter steht schon in sich selbst. SRDF-Paare pruefen."),
    -11: ("Startzustand verletzt Bahnbedingung", ""),
    -12: ("Ziel in Kollision", ""),
    -13: ("Ziel verletzt Bahnbedingung", ""),
    -14: ("Zielbedingungen verletzt",
          "Erreicht, aber ausserhalb der Toleranz. Zieltoleranz pruefen."),
    -15: ("Gruppenname ungueltig",
          "Die Planungsgruppe steht so nicht in der SRDF."),
    -16: ("Zielbedingungen ungueltig",
          "Gelenknamen im Ziel passen nicht zur Gruppe."),
    -17: ("Roboterzustand ungueltig",
          "Meist fehlt /joint_states oder es ist zu alt."),
    -18: ("Linkname ungueltig", ""),
    -19: ("Objektname ungueltig", ""),
    -21: ("Frame-Transformation fehlgeschlagen", ""),
    -22: ("Kollisionspruefung nicht verfuegbar", ""),
    -23: ("Roboterzustand veraltet",
          "/joint_states kommt nicht oder zu langsam."),
    -24: ("Sensordaten veraltet", ""),
    -25: ("Verbindungsfehler", ""),
    -31: ("Keine IK-Loesung",
          "Nur bei kartesischen Zielen. Lage ausserhalb des Arbeitsraums?"),
}


def moveit_error_text(code):
    """Fehlercode als Satz. Unbekannte Codes bleiben als Zahl sichtbar."""
    label, hint = MOVEIT_ERRORS.get(int(code), (None, ""))
    if label is None:
        return f"Unbekannter Fehlercode {int(code)}"
    if hint:
        return f"{label} ({int(code)}). {hint}"
    return f"{label} ({int(code)})"


def hermite(p0, v0, p1, v1, span, s):
    """Kubische Hermite-Interpolation zwischen zwei Stuetzpunkten.

    Pilz gibt zu jedem Punkt auch die Geschwindigkeit heraus. Wer die
    wegwirft und geradlinig interpoliert, bekommt an jedem Stuetzpunkt einen
    Knick in der Geschwindigkeit -- bei dichter Abtastung kaum sichtbar, im
    Geschwindigkeitsdiagramm aber sofort. Die kubische Form nimmt beide
    Randgeschwindigkeiten mit und ist damit stetig differenzierbar.

    span ist der Zeitabstand der Stuetzpunkte, s der Anteil dazwischen.
    """
    s2 = s * s
    s3 = s2 * s
    h00 = 2.0 * s3 - 3.0 * s2 + 1.0
    h10 = s3 - 2.0 * s2 + s
    h01 = -2.0 * s3 + 3.0 * s2
    h11 = s3 - s2
    return h00 * p0 + h10 * span * v0 + h01 * p1 + h11 * span * v1


class TrajectoryPlayer:
    """Tastet eine geplante Bahn im festen Takt ab.

    Bewusst ohne Tk und ohne ROS: die Klasse bekommt Stuetzpunkte als reine
    Zahlen und gibt Sollwerte als reine Zahlen zurueck. Damit laesst sie sich
    ohne Fenster und ohne Roboter pruefen, und im Unterricht kann man sie in
    zehn Zeilen selbst aufrufen.

    Die Bahnzeit entsteht aus der Summe der Takte, nicht aus der Wanduhr.
    Das ist der Grund, warum derselbe Auftrag zweimal denselben CSV-Auszug
    liefert: Tk-after ist nicht echtzeitfaehig, und wer die Wanduhr in die
    Rechnung nimmt, nimmt deren Zittern gleich mit. Der Abstand zwischen
    Bahnzeit und Wanduhrzeit ist dadurch messbar -- er wird angezeigt statt
    versteckt.

    points: [(zeit, {name: pos}, {name: vel}), ...], nach Zeit sortiert.
    """

    def __init__(self, points, dt=IPO_DT):
        if len(points) < 2:
            raise ValueError("Eine Bahn braucht mindestens zwei Stuetzpunkte")
        self.points = points
        self.dt = dt
        self.names = list(points[0][1])
        self.duration = points[-1][0]
        self.t = 0.0                                   # Bahnzeit
        self.ticks = 0                                 # gezaehlte Takte
        self.rate = 1.0                                # Zeitrate, 1.0 = voll
        self.braking = False
        self._index = 0                                # Suchzeiger, laeuft vor

    # -- Zustand -----------------------------------------------------------

    @property
    def done(self):
        return self.t >= self.duration and not self.braking

    @property
    def stopped(self):
        return self.braking and self.rate <= 0.0

    @property
    def progress(self):
        if self.duration <= 0.0:
            return 1.0
        return min(1.0, self.t / self.duration)

    def wall_drift(self, elapsed):
        """Abstand zwischen gezaehlter Bahnzeit und vergangener Wanduhrzeit."""
        return elapsed - self.ticks * self.dt

    # -- Abtasten ----------------------------------------------------------

    def sample(self, t=None):
        """Sollwerte zum Zeitpunkt t. Ohne Angabe zur aktuellen Bahnzeit."""
        if t is None:
            t = self.t
        if t <= self.points[0][0]:
            return dict(self.points[0][1])
        if t >= self.points[-1][0]:
            return dict(self.points[-1][1])

        # Der Zeiger laeuft nur vorwaerts, solange die Zeit vorwaerts laeuft.
        # Beim Bremsen steht sie hoechstens still, sie geht nie zurueck.
        while (self._index + 1 < len(self.points) - 1
               and self.points[self._index + 1][0] <= t):
            self._index += 1
        while self._index > 0 and self.points[self._index][0] > t:
            self._index -= 1

        t0, p0, v0 = self.points[self._index]
        t1, p1, v1 = self.points[self._index + 1]
        span = t1 - t0
        if span <= 0.0:
            return dict(p1)
        s = (t - t0) / span
        return {name: hermite(p0.get(name, 0.0), v0.get(name, 0.0),
                              p1.get(name, 0.0), v1.get(name, 0.0), span, s)
                for name in self.names}

    def step(self):
        """Einen Takt weiter. Liefert die Sollwerte danach."""
        self.ticks += 1
        if self.braking and self.rate > 0.0:
            self.rate = max(0.0, self.rate - self.dt / BRAKE_TIME_S)
        self.t = min(self.duration, self.t + self.dt * self.rate)
        return self.sample()

    def brake(self):
        """Anhalten einleiten. Die Bahn wird nicht verlassen, nur langsamer."""
        self.braking = True


class PathRecorder:
    """Ringpuffer fuer die TCP-Bahn.

    Kein ROS, kein Tk: die Klasse haelt nur Zahlen. Die Abtastung stoesst die
    GUI ueber `sample` an, gefiltert wird hier drin.
    """

    def __init__(self, max_points=RECORD_MAX_POINTS,
                 min_dist=RECORD_MIN_DIST_M,
                 min_angle=math.radians(RECORD_MIN_ANGLE_DEG)):
        self.points = collections.deque(maxlen=max_points)
        self.min_dist = min_dist
        self.min_angle = min_angle
        self.joint_names = []                          # Reihenfolge der Achsspalten
        self.active = False
        self.start_time = None                         # time.monotonic()
        self.length = 0.0                              # gefahrener Weg des TCP
        self.overflow = False                          # aeltester Punkt verworfen

    # -- Steuerung ---------------------------------------------------------

    def start(self, joint_names):
        """Aufzeichnung beginnen oder fortsetzen.

        Wechselt der Achssatz, sind die alten Punkte nicht mehr spaltenrichtig
        zu schreiben. Dann wird verworfen statt vermischt.
        """
        names = list(joint_names)
        if names != self.joint_names:
            self.clear()
            self.joint_names = names
        self.active = True
        return len(self.points)

    def stop(self):
        self.active = False

    def clear(self):
        self.points.clear()
        self.start_time = None
        self.length = 0.0
        self.overflow = False

    # -- Abtastung ---------------------------------------------------------

    def sample(self, xyz, rpy, values, now=None):
        """Einen Punkt anbieten. Gibt zurueck, ob er abgelegt wurde."""
        if not self.active:
            return False
        now = time.monotonic() if now is None else now
        if self.start_time is None:
            self.start_time = now

        step = 0.0
        if self.points:
            _t, last_xyz, last_rpy, _v = self.points[-1]
            step = math.sqrt(sum((a - b) ** 2 for a, b in zip(xyz, last_xyz)))
            # RPY reicht als Ruhekriterium. Nahe der Gimbal-Lage springen die
            # Werte, das erzeugt hoechstens einen Punkt zu viel, nie zu wenig.
            turn = max(angle_delta(a, b) for a, b in zip(rpy, last_rpy))
            if step < self.min_dist and turn < self.min_angle:
                return False

        if len(self.points) == self.points.maxlen:
            self.overflow = True
        self.points.append((now - self.start_time, tuple(xyz), tuple(rpy),
                            tuple(values)))
        self.length += step                            # zaehlt weiter, auch wenn
        return True                                    # vorne Punkte herausfallen

    # -- Auswertung --------------------------------------------------------

    @property
    def count(self):
        return len(self.points)

    @property
    def duration(self):
        """Zeit vom ersten bis zum letzten abgelegten Punkt."""
        if len(self.points) < 2:
            return 0.0
        return self.points[-1][0] - self.points[0][0]

    def write_csv(self, path):
        """Punkte als CSV ablegen. Winkel in grad, Laengen in m."""
        with open(path, "w", newline="") as handle:
            writer = csv.writer(handle, delimiter=";")
            writer.writerow(["t_s", "x_m", "y_m", "z_m",
                             "roll_deg", "pitch_deg", "yaw_deg"]
                            + list(self.joint_names))
            for stamp, xyz, rpy, values in self.points:
                writer.writerow([f"{stamp:.4f}"]
                                + [f"{v:.6f}" for v in xyz]
                                + [f"{math.degrees(v):.4f}" for v in rpy]
                                + [f"{v:.6f}" for v in values])
        return len(self.points)

    
# ==========================================================================
# ROS-Anbindung
# ==========================================================================

class RosBridge:
    """Node, Spin-Thread und TF-Ausgabe. Alle ROS-Importe liegen hier drin."""

    def __init__(self):
        self.node = None
        self.error = None
        self.tick_count = 0
        self.enabled = True

        # Vom Executor-Thread geschrieben, vom GUI-Thread gelesen. Einzelne
        # float-Zuweisungen sind unter dem GIL atomar, ein Lock ist unnoetig.
        self.last_publish = None                       # time.monotonic()
        self.cycle_ms = None                           # geglaetteter Zyklusabstand
        self.moveit_ready = False                      # der eigene Knoten antwortet

        # Vom GUI-Thread gesetzt, vom Probe-Takt gelesen: der Knotenname, den
        # der zuletzt gestartete Kindprozess traegt. Eine einzelne Zuweisung
        # einer Zeichenkette ist unter dem GIL atomar, ein Lock waere hier
        # eine Behauptung ueber Gefahr, die es nicht gibt.
        self.moveit_node = ""
        self.moveit_nodes = []                         # alle move_group im Graphen

        # Ergebnisse aus dem Executor-Thread an den GUI-Thread. Der Tk-Thread
        # darf ROS nicht anfassen und schon gar nicht auf eine Antwort warten
        # -- das Fenster stuende still, bis move_group fertig ist. Deshalb
        # legt der Executor hier ab und _tick holt ab.
        self.results = collections.deque(maxlen=200)

        # Auftraege in die Gegenrichtung. Der GUI-Thread legt hier ab, der
        # Auftragstakt im Executor-Thread holt und stellt den Dienstaufruf.
        # call_async aus dem Tk-Thread heraus waere kuerzer, wuerde aber die
        # Regel brechen, dass der GUI-Thread ROS nicht anfasst.
        self._jobs = collections.deque(maxlen=20)

        self._lock = threading.Lock()
        self._broadcast = []                           # alle Gelenke, die auf /tf gehen
        self._positions = {}
        self._executor = None
        self._thread = None
        self._timer = None
        self._broadcaster = None
        self._description_pub = None
        self._joint_pub = None
        self._joint_names = []
        self._probe_timer = None
        self._job_timer = None
        self._plan_client = None
        self._pending = None                           # (future, Ablauf, Frist)
        self._stop = threading.Event()
        self._reset_wanted = False                     # zwischen zwei Durchlaeufen
        self._spin_errors = 0

    @property
    def running(self):
        return self.node is not None

    def thread_alive(self):
        """Laeuft der Executor-Thread noch? Die Frage war bis 2.11 nicht zu stellen."""
        return self._thread is not None and self._thread.is_alive()

    # -- Lebenszyklus ------------------------------------------------------

    def start(self):
        """Node hochfahren. Gibt None zurueck oder eine Fehlermeldung."""
        if self.running:
            return None
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.qos import (QoSProfile, DurabilityPolicy,
                                   ReliabilityPolicy, HistoryPolicy)
            from sensor_msgs.msg import JointState
            from std_msgs.msg import String
            from tf2_ros import TransformBroadcaster
        except ImportError as exc:
            self.error = f"ROS 2 nicht verfuegbar ({exc.name}). Nur Anzeige und FK."
            return self.error

        rclpy.init()
        self.node = Node("urdf_gui_tf")
        self._broadcaster = TransformBroadcaster(self.node)

        qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._description_pub = self.node.create_publisher(String, "robot_description", qos)

        # /joint_states ist fuer MoveIt keine Zugabe, sondern Voraussetzung:
        # ohne aktuellen Zustand verweigert der Planungsszenen-Monitor die
        # Arbeit. Es laeuft im TF-Takt mit, damit beide dieselbe Stempelzeit
        # tragen und nicht gegeneinander laufen.
        self._joint_pub = self.node.create_publisher(JointState, "joint_states", 10)

        self._timer = self.node.create_timer(
            1.0 / TF_RATE_HZ, lambda: self._guard("TF", self._on_timer))
        self._probe_timer = self.node.create_timer(
            1.0, lambda: self._guard("Probe", self._on_probe))
        self._job_timer = self.node.create_timer(
            1.0 / JOB_RATE_HZ, lambda: self._guard("Auftrag", self._on_jobs))

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._stop.clear()
        self._spin_errors = 0
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()
        return None

    def _spin(self):
        """Eigene Schleife statt executor.spin() als Thread-Ziel.

        spin() direkt als Ziel zu uebergeben hat einen Fehler ohne Warnung:
        faellt in irgendeinem Callback eine Ausnahme, verlaesst sie spin(),
        der Thread endet -- und mit ihm enden alle Timer. Nach aussen sieht
        das aus wie "seit dem Planen kommt kein /tf mehr, und Last Refresh
        zaehlt nur noch hoch". Eine Wirkung ohne sichtbare Ursache, weil der
        Traceback in einem Thread landet, dessen Ausgabe unter einem
        Launchfile niemand liest.

        Ein Takt, der aus einem einzelnen schlechten Callback aussteigt, ist
        fuer eine Bedienoberflaeche die falsche Bauart. Ein Fehler gehoert
        gemeldet, nicht mit dem Takt zusammen begraben. Deshalb faengt die
        Schleife hier und legt die Ausnahme dem GUI-Thread hin.
        """
        import rclpy
        while not self._stop.is_set():
            # Aufraeumen zwischen zwei Durchlaeufen und nie waehrend eines
            # Callbacks: einen Client zu zerstoeren, waehrend der Executor
            # ueber seine Entitaeten laeuft, heisst in das eigene Wartefeld
            # greifen.
            if self._reset_wanted:
                self._reset_wanted = False
                self._drop_client("move_group neu gestartet")
            try:
                self._executor.spin_once(timeout_sec=0.05)
            except Exception as exc:                   # auch rclpy-eigene
                if type(exc).__name__ == "ExternalShutdownException":
                    return
                self._spin_errors += 1
                lines = traceback.format_exc().strip().split("\n")
                self.results.append(("ros_error", " | ".join(lines[-3:])))
                if self._spin_errors >= SPIN_ERROR_LIMIT:
                    self.results.append(
                        ("ros_error", f"{self._spin_errors} Fehler im "
                                      "ROS-Takt, er wird beendet"))
                    return
                time.sleep(0.05)                       # nicht heisslaufen

    def shutdown(self):
        if not self.running:
            return
        import rclpy
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self._executor.shutdown()
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self.node = None

    # -- Modell und Werte --------------------------------------------------

    def set_model(self, joints, urdf_xml):
        """Neues Modell uebernehmen. Alle Frames laufen zyklisch ueber /tf.

        Feste Gelenke bewusst NICHT auf /tf_static: dort abgelegte Frames
        landen in einem Cache, der nie verfaellt. Nach einem Modellwechsel
        blieben sie als Geisterframes in RViz stehen, und loeschen laesst sich
        in tf2 nichts. Ueber /tf verfallen sie nach der Cache-Zeit von selbst.
        """
        if not self.running:
            return self.error or "Kein ROS-Node aktiv"

        from std_msgs.msg import String

        usable = [j for j in joints if j["parent"] and j["child"]]
        movable = [j for j in usable if j["type"] != "fixed"]

        self._description_pub.publish(String(data=urdf_xml))

        with self._lock:
            self._broadcast = usable
            self._positions = {j["name"]: 0.0 for j in movable}
            # Gekoppelte Gelenke gehen mit heraus. Sie haben in der URDF ein
            # <mimic>, aber MoveIt erwartet sie trotzdem im Zustand.
            self._joint_names = [j["name"] for j in movable]
        self.tick_count = 0
        self.last_publish = None
        self.cycle_ms = None

        self._publish_once()                           # nicht auf den Timer warten

        return (f"{len(usable)} Frames auf /tf mit {TF_RATE_HZ:.0f} Hz "
                f"({len(movable)} beweglich), robot_description und "
                f"/joint_states gesendet")

    def set_positions(self, positions):
        """Sollwerte uebernehmen. Wird aus dem GUI-Thread aufgerufen."""
        with self._lock:
            self._positions = dict(positions)

    # -- Ausgabe -----------------------------------------------------------

    def _message(self, joint, matrix, stamp):
        from geometry_msgs.msg import TransformStamped
        msg = TransformStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = joint["parent"]
        msg.child_frame_id = joint["child"]
        msg.transform.translation.x = matrix[0][3]
        msg.transform.translation.y = matrix[1][3]
        msg.transform.translation.z = matrix[2][3]
        qx, qy, qz, qw = to_quaternion(matrix)
        msg.transform.rotation.x = qx
        msg.transform.rotation.y = qy
        msg.transform.rotation.z = qz
        msg.transform.rotation.w = qw
        return msg

    def _publish_once(self):
        """Einen Zyklus senden. Feste Gelenke liefern dabei stets ihr origin."""
        with self._lock:
            frames = list(self._broadcast)
            positions = dict(self._positions)
        if not frames:
            return

        stamp = self.node.get_clock().now().to_msg()
        self._broadcaster.sendTransform([
            self._message(j, local_transform(j, positions.get(j["name"], 0.0)), stamp)
            for j in frames
        ])
        self._publish_joint_state(stamp, positions)

        now = time.monotonic()
        if self.last_publish is not None:
            delta_ms = (now - self.last_publish) * 1000.0
            self.cycle_ms = (delta_ms if self.cycle_ms is None
                             else 0.9 * self.cycle_ms + 0.1 * delta_ms)
        self.last_publish = now
        self.tick_count += 1

    def _publish_joint_state(self, stamp, positions):
        """Denselben Stand als JointState herausgeben.

        Geschwindigkeit und Kraft bleiben leer: solange die Sollwerte nur
        gestellt werden, gibt es keine. Sobald die naechste Version eine Bahn
        abtastet, kommt velocity aus dem Profil dazu.
        """
        if self._joint_pub is None:
            return
        with self._lock:
            names = list(self._joint_names)
        if not names:
            return
        from sensor_msgs.msg import JointState
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = names
        msg.position = [float(positions.get(name, 0.0)) for name in names]
        self._joint_pub.publish(msg)

    # -- Auftraege ---------------------------------------------------------

    def request_plan(self, payload):
        """Aus dem GUI-Thread. Legt den Auftrag ab und kehrt sofort zurueck.

        payload enthaelt nur Zahlen und Zeichenketten. Die ROS-Nachricht
        entsteht erst drueben im Executor-Thread -- so bleibt der GUI-Thread
        frei von moveit_msgs und laesst sich ohne ROS pruefen.
        """
        self._jobs.append(("plan", dict(payload)))

    def _guard(self, name, function):
        """Einen Callback so aufrufen, dass er den Takt nicht mitreissen kann.

        Jeder eigene Callback laeuft hierdurch. Was eine Ausnahme wirft, wird
        gemeldet und uebersprungen; der naechste Takt kommt trotzdem. Ohne
        das reicht ein einziger Fehler in der Antwortverarbeitung, um die
        TF-Ausgabe stillzulegen.
        """
        try:
            function()
        except Exception:
            lines = traceback.format_exc().strip().split("\n")
            self.results.append(("ros_error", f"{name}: " + " | ".join(lines[-2:])))

    def reset_moveit(self, node_name=""):
        """Aus dem GUI-Thread. Nach jedem Start und Stopp von move_group.

        Der Dienstclient wird drueben im Executor-Thread weggeworfen. Ein
        Client, dessen Server verschwunden ist, bleibt unter Fast DDS
        gelegentlich als verbunden stehen: service_is_ready sagt ja, der
        Aufruf geht ins Leere, und das Future wird nie fertig. Ihn neu zu
        bauen kostet nichts und nimmt diese Klasse von Fehlern heraus.
        """
        self.moveit_node = node_name
        self.moveit_ready = False
        self._reset_wanted = True

    def _on_jobs(self):
        """Laeuft im Executor-Thread. Holt einen Auftrag und stellt ihn."""
        self._check_pending()
        try:
            kind, payload = self._jobs.popleft()
        except IndexError:
            return
        if kind != "plan":
            return
        try:
            self._send_plan(payload)
        except Exception as exc:                       # nichts darf den Takt toeten
            self._pending = None
            self.results.append(("plan", {"error": f"Auftrag fehlgeschlagen: {exc}"}))

    def _drop_client(self, reason):
        """Client wegwerfen und eine offene Anfrage als Fehler abschliessen."""
        if self._pending is not None:
            future = self._pending[0]
            self._pending = None
            try:
                future.cancel()
            except Exception:
                pass
            self.results.append(("plan", {"error": f"Anfrage verworfen: {reason}"}))
        if self._plan_client is not None:
            try:
                self.node.destroy_client(self._plan_client)
            except Exception:
                pass
            self._plan_client = None

    def _check_pending(self):
        """Frist einer offenen Anfrage pruefen. Laeuft im Auftragstakt mit.

        Der Rueckweg in die GUI fuehrt allein ueber den Callback des Futures.
        Kommt keine Antwort, wird er nie gerufen, und die Ablaufsteuerung
        bleibt in PLANEN stehen. Deshalb endet hier jede Anfrage, spaetestens
        nach der Frist, die der Auftrag mitgebracht hat.
        """
        if self._pending is None:
            return
        future, deadline, timeout = self._pending
        if future.done():
            self._pending = None
            return
        if time.monotonic() < deadline:
            return
        self._pending = None
        try:
            future.cancel()
        except Exception:
            pass
        self.results.append(("plan", {
            "error": f"move_group antwortet seit {timeout:.0f} s nicht. "
                     "Laeuft ein zweiter move_group?"}))

    def _send_plan(self, payload):
        from moveit_msgs.srv import GetMotionPlan
        from moveit_msgs.msg import Constraints, JointConstraint

        if self._plan_client is None:
            self._plan_client = self.node.create_client(
                GetMotionPlan, "/plan_kinematic_path")
        if not self._plan_client.service_is_ready():
            self.results.append(("plan", {
                "error": "/plan_kinematic_path antwortet nicht"}))
            return

        request = GetMotionPlan.Request()
        plan = request.motion_plan_request
        plan.group_name = payload["group"]
        plan.planner_id = payload["planner_id"]
        plan.pipeline_id = "pilz_industrial_motion_planner"
        plan.num_planning_attempts = int(payload["attempts"])
        plan.allowed_planning_time = float(payload["planning_time"])
        plan.max_velocity_scaling_factor = float(payload["velocity_scaling"])
        plan.max_acceleration_scaling_factor = float(
            payload["acceleration_scaling"])

        # Startzustand ausdruecklich mitgeben statt ihn aus /joint_states
        # holen zu lassen. Beides ginge; so ist aber sicher, dass geplant
        # wird, was die GUI anzeigt, und nicht ein Stand von vor drei Takten.
        plan.start_state.joint_state.name = list(payload["start"])
        plan.start_state.joint_state.position = [
            float(value) for value in payload["start"].values()]
        plan.start_state.is_diff = False

        tolerance = float(payload["tolerance"])
        goal = Constraints()
        for name, value in payload["goal"].items():
            constraint = JointConstraint()
            constraint.joint_name = name
            constraint.position = float(value)
            constraint.tolerance_above = tolerance
            constraint.tolerance_below = tolerance
            constraint.weight = 1.0
            goal.joint_constraints.append(constraint)
        plan.goal_constraints.append(goal)

        timeout = float(payload.get("timeout") or PLAN_TIMEOUT_MARGIN_S)
        future = self._plan_client.call_async(request)
        self._pending = (future, time.monotonic() + timeout, timeout)
        future.add_done_callback(self._on_plan_done)

    def _on_plan_done(self, future):
        """Rueckruf aus rclpy. Faengt alles, sonst faellt es in den Executor."""
        self._guard("Planantwort", lambda: self._read_plan(future))

    def _read_plan(self, future):
        """Antwort in reine Zahlen umsetzen und dem GUI-Thread hinlegen."""
        self._pending = None
        try:
            response = future.result()
        except Exception as exc:
            self.results.append(("plan", {"error": str(exc)}))
            return

        result = response.motion_plan_response
        code = int(result.error_code.val)
        payload = {"code": code, "points": []}
        if code != 1:
            self.results.append(("plan", payload))
            return

        trajectory = result.trajectory.joint_trajectory
        # Mehrzahl. JointConstraint.joint_name ist Einzahl -- eine Bedingung
        # gilt fuer ein Gelenk -- und JointTrajectory.joint_names ist Mehrzahl,
        # weil eine Bahn alle fuehrt. Beide stehen in demselben Auftrag
        # wenige Zeilen auseinander, und die Verwechslung faellt nirgends auf:
        # bis 2.10 nahm sie den ROS-Takt mit, und zwar erst beim ersten
        # gelungenen Plan, weil der Fehlerweg die Bahn gar nicht anfasst.
        names = list(trajectory.joint_names)
        points = []
        for point in trajectory.points:
            seconds = (point.time_from_start.sec
                       + point.time_from_start.nanosec * 1e-9)
            positions = {name: float(value)
                         for name, value in zip(names, point.positions)}
            raw = point.velocities
            velocities = ({name: float(value)
                           for name, value in zip(names, raw)}
                          if len(raw) == len(names)
                          else {name: 0.0 for name in names})
            points.append((seconds, positions, velocities))
        payload["points"] = points
        payload["planning_time"] = float(result.planning_time)
        self.results.append(("plan", payload))

    def _on_probe(self):
        """Einmal je Sekunde nachsehen, ob der eigene move_group da ist.

        Bis 2.4 stand hier allein die Frage, ob /plan_kinematic_path im
        Graphen steht. Die Frage ist zu schwach: ein Dienstname im Graphen
        sagt nur, dass irgendwer ihn bedient. Lief noch ein alter Knoten,
        meldete die Anzeige "bereit", waehrend der neue Prozess noch hochfuhr
        oder schon gestorben war -- und geplant wurde mit dem alten Modell.
        Seit 2.5 traegt jeder Start einen eigenen Knotennamen, und hier wird
        genau der gesucht.

        Laeuft im Executor-Thread, deshalb geht die Meldung ueber die
        Warteschlange an den GUI-Thread und nicht direkt ins Protokoll.
        """
        try:
            services = dict(self.node.get_service_names_and_types())
            nodes = [name for name, _ns in self.node.get_node_names_and_namespaces()]
        except Exception:                              # Knoten faehrt gerade herunter
            return

        found = sorted(name for name in nodes if name.startswith("move_group"))
        wanted = self.moveit_node
        # Ohne erwarteten Namen -- etwa bei einem von aussen gestarteten
        # move_group -- bleibt es bei der alten, schwaecheren Frage.
        mine = (wanted in found) if wanted else bool(found)
        ready = (mine
                 and "/plan_kinematic_path" in services
                 and "/compute_ik" in services)

        if found != self.moveit_nodes:
            self.moveit_nodes = found
            if len(found) > 1:
                self.results.append(("moveit_nodes", found))
        if ready != self.moveit_ready:
            self.moveit_ready = ready
            self.results.append(("moveit", ready))

    def take_results(self):
        """Aus dem GUI-Thread. Leert die Warteschlange in einem Zug."""
        out = []
        while True:
            try:
                out.append(self.results.popleft())
            except IndexError:
                break
        return out

    def _on_timer(self):
        """Laeuft im Executor-Thread. Nur lesen, was unter dem Lock geholt wurde."""
        if not self.enabled:
            return
        self._publish_once()


# ==========================================================================
# GUI
# ==========================================================================

# Spalten: Schluessel, Ueberschrift, Breite, Ausrichtung.
# Zahlen stehen rechts, damit die Kommastellen untereinander liegen.
# Der Framename steht in der Baumspalte #0, damit die Einrueckung dort
# sichtbar wird. Deshalb taucht er hier nicht mehr auf.
FRAME_TREE = ("Frame", 150)                            # Mindestbreite, waechst mit
TREE_INDENT = 14                                       # Einrueckung je Ebene
TREE_MAX_WIDTH = 330                                   # sonst ufert die Spalte aus
FRAME_COLUMNS = [
    ("x",     "X [m]",    64, "e"),
    ("y",     "Y [m]",    64, "e"),
    ("z",     "Z [m]",    64, "e"),
    ("roll",  "R [deg]",  66, "e"),
    ("pitch", "P [deg]",  66, "e"),
    ("yaw",   "Y [deg]",  66, "e"),
]

JOINT_COLUMNS = [
    ("name",  "Gelenk",       90, "w"),
    ("type",  "Typ",          78, "w"),
    ("rad",   "rad/m",        74, "e"),
    ("grad",  "grad/mm",      76, "e"),
    ("limit", "Grenzen",     132, "e"),
    ("chain", "Kette",       117, "w"),
]

LINK_COLUMNS = [
    ("name",  "Link",        160, "w"),
    ("mass",  "Masse [kg]",   95, "e"),
    ("com",   "COM xyz [m]", 175, "w"),
    ("shape", "Form",         80, "w"),
    ("color", "Farbe",        80, "w"),
]

CART_FIELDS = [("x", "X", "m"), ("y", "Y", "m"), ("z", "Z", "m"),
               ("roll", "Roll", "deg"), ("pitch", "Pitch", "deg"),
               ("yaw", "Yaw", "deg")]

MOTION_FIELDS = [("jerk", "Jerk", "m/s3", 10.0),
                 ("acceleration", "Acceleration", "m/s2", 2.0),
                 ("speed", "Speed", "m/s", 0.5)]

PAD = 10                                               # ein Rastermass fuer alles


class UrdfViewer(tk.Tk):

    def __init__(self, urdf_path):
        super().__init__()
        self.title("URDF Viewer")
        self.geometry("1560x900")
        self.minsize(1220, 700)
        self.configure(background=BG)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

        self.urdf_path = urdf_path
        self.robot_name = ""
        self.xml_root = None                           # bearbeiteter Baum, die Wahrheit
        self.links = []
        self.joints = []
        self.urdf_xml = ""
        self.dirty = False                             # geaendert, aber nicht gespeichert
        self.stopped = False                           # STOP gedrueckt
        self.positions = {}                            # {gelenkname: rad bzw. m}
        self.entries = {}                              # {gelenkname: tk.Entry}
        self.sliders = {}                              # {gelenkname: ttk.Scale}
        self.syncing = False                           # Schieber wird gerade gesetzt
        self.refresh_job = None                        # laufender after-Aufruf
        self.cart_entries = {}                         # Zielvorgabe kartesisch
        self.motion_entries = {}                       # Jerk, Acceleration, Speed
        self.motion = {key: default for key, _l, _u, default in MOTION_FIELDS}
        self.frames_shown = True                       # Frames-Karte sichtbar
        self.frame_items = {}                          # {linkname: iid im Baum}
        self.frame_layout = None                       # {linkname: elternname}
        self.editor = None                             # Bearbeiten-Fenster oder None
        self.edit_entries = {}
        self.add_window = None                         # Anhaengen-Fenster oder None
        self.add_entries = {}
        self.insert_window = None                      # Einfuegen-Fenster oder None
        self.insert_entries = {}
        self.insert_below = None                       # Gelenk, vor das eingefuegt wird
        self.tree_canvas = None                        # Zeichenflaeche der Karte Baum
        self.tree_boxes = {}                           # {linkname: (x0, y0, x1, y1)}
        self.tree_zones = {}                           # {gelenkname: (x0, y0, x1, y1)}
        self.tree_selected = None                      # ausgewaehltes Gelenk oder None
        self.tree_hover = None                         # ("node"/"edge", name) oder None
        self.tree_pills = []                           # [(Rechteck, Befehl)]
        self.remove_window = None                      # Herausnehmen-Fenster oder None
        self.remove_joint = None                       # Gelenk, das herausfaellt
        self.edit_pages = None                         # Karteikarten im Editor
        self.visual_entries = {}
        self.visual_combo = None                       # Linkauswahl der Koerperseite
        self.visual_shape_combo = None
        self.visual_filename = ""                      # Netzdatei, unveraendert
        self.visual_swatch = None                      # Farbfeld zur Vorschau
        self.visual_half = None                        # Taste "halbe Hoehe"
        self.edit_name_entry = None                    # Namensfeld des Gelenks
        self.edit_rename_button = None
        self.edit_mimic_combo = None                   # Auswahl des Fuehrungsgelenks
        self.visual_name_entry = None                  # Namensfeld des Links
        self.visual_rename_button = None
        self.history = []                              # (Text, xml_root-Kopie)
        self.saved_signature = None                    # Stand der Datei zum Vergleich
        self.undo_button = None
        self.go_buttons = []
        self.recorder = PathRecorder()
        self.record_warned = False                     # Hinweis auf fehlenden TCP
        self.model_buttons = []                        # sperren waehrend der Fahrt

        # ---------- MoveIt ----------
        self.moveit_config = MoveItConfig()
        self.moveit = MoveItProcess(log=self._log)
        self.moveit_srdf = ""                          # zuletzt erzeugte SRDF
        self.moveit_stale = False                      # Modell/Parameter neuer
        self.tf_stalled = False                        # Waechter auf /tf
        self.moveit_wait_since = None                  # wartet auf den Hochlauf
        self.lock_reason = None                        # None, "Fahrt", "Neustart..."

        # ---------- Fahren ----------
        self.state = STATE_READY
        self.player = None                             # TrajectoryPlayer oder None
        self.motion_job = None                         # laufender after-Aufruf
        self.play_started = None                       # time.monotonic()
        self.plan_started = None                       # fuer die Planungsdauer
        self.drive_buttons = []                        # brauchen move_group
        self.start_buttons = []                        # brauchen zusaetzlich einen Plan
        self.params_window = None

        # ---------- Fahren ----------
        # Das Fahrfenster hat eigene Zielfelder. Sie sind nicht dieselben wie
        # die Sollwertfelder links: dort stellt ein Wert die Achse sofort,
        # hier beschreibt er ein Ziel, das erst auf Tastendruck angefahren
        # wird. Beides in ein Feld zu legen war der Fehler in 2.4 bis 2.6 --
        # der Roboter stand beim Fahrbefehl schon am Ziel, und die Bahn hatte
        # die Laenge null.
        self.drive_window = None
        self.drive_entries = {}                        # Gelenk -> Zielfeld
        self.drive_widgets = []                        # sperren waehrend der Fahrt
        self.drive_status_var = tk.StringVar(value="")
        self.drive_plan_var = tk.StringVar(value="kein Plan")
        self.plan_hold = None                          # geplant, noch nicht gefahren
        self.plan_hold_start = None                    # Achsstand beim Planen
        self.plan_hold_goal = None
        self.params_status_label = None
        self.param_entries = {}
        self.param_widgets = []
        self.limit_entries = {}

        self.robot_var = tk.StringVar(value="kein Modell")
        self.info_var = tk.StringVar(value="")
        self.path_var = tk.StringVar(value=os.path.basename(urdf_path))
        self.tcp_var = tk.StringVar(value="")
        self.session_var = tk.StringVar(value="Session  00:00:00:000")
        self.refresh_var = tk.StringVar(value="Last Refresh  --")
        self.edit_joint_var = tk.StringVar(value="")
        self.edit_info_var = tk.StringVar(value="")
        self.edit_unit_var = tk.StringVar(value="")
        self.edit_type_var = tk.StringVar(value="")
        self.add_type_var = tk.StringVar(value="revolute")
        self.add_parent_var = tk.StringVar(value="")
        self.add_unit_var = tk.StringVar(value="")
        self.insert_type_var = tk.StringVar(value="fixed")
        self.insert_unit_var = tk.StringVar(value="")
        self.insert_info_var = tk.StringVar(value="")
        self.insert_keep_var = tk.BooleanVar(value=True)
        self.remove_keep_var = tk.BooleanVar(value=True)
        self.visual_link_var = tk.StringVar(value="")
        self.visual_shape_var = tk.StringVar(value="box")
        self.visual_hex_var = tk.StringVar(value="#a0a0a2")
        self.visual_collision_var = tk.BooleanVar(value=True)
        self.visual_note_var = tk.StringVar(value="")
        self.visual_rename_hint_var = tk.StringVar(value="")
        self.edit_mimic_var = tk.StringVar(value=NO_COUPLING)
        self.edit_mimic_unit_var = tk.StringVar(value="")
        self.remove_info_var = tk.StringVar(value="")
        self.tree_hint_var = tk.StringVar(value="")
        self.send_tf = tk.BooleanVar(value=True)
        self.record_var = tk.StringVar(value="0 Punkte")
        self.moveit_var = tk.StringVar(value="move_group  aus")
        self.state_var = tk.StringVar(value=STATE_READY)
        self.planner_var = tk.StringVar(value=PLANNER_IDS[0])
        self.solver_var = tk.StringVar(value=KINEMATICS_SOLVERS[0])
        self.ruckig_var = tk.BooleanVar(value=True)
        self.collision_var = tk.BooleanVar(value=True)
        self.execution_var = tk.BooleanVar(value=False)

        self.session_start = time.monotonic()
        self.ros = RosBridge()

        self._setup_fonts()
        self._setup_style()
        self._build_ui()

        # Reste frueherer Sitzungen. Bis 2.4 loeschte stop() den Ordner
        # sofort; seit 2.5 bleibt der letzte liegen, damit ein Fehlstart
        # nachlesbar ist. Aufgeraeumt wird dafuer hier, und nur was aelter
        # ist als eine Stunde -- daneben koennte eine zweite GUI laufen.
        removed = MoveItProcess.sweep_workdirs(max_age_s=3600.0)
        if removed:
            self._log(f"{removed} alte Arbeitsordner in "
                      f"{tempfile.gettempdir()} entfernt", "info")
        strays = self.moveit.strays()
        if strays:
            self._log(f"{len(strays)} move_group laufen bereits "
                      f"(PID {', '.join(str(p) for p in strays)}). Stammen sie "
                      "aus einer frueheren Sitzung, im Parameterfenster "
                      "beenden -- sonst bedienen sie die Dienste mit.", "warn")

        self._start_ros()
        self._load(urdf_path, initial=True)
        self._tick()
        self._record_tick()                            # eigener Takt, siehe oben


    # ----------------------------------------------------------------------
    # Aussehen
    # ----------------------------------------------------------------------

    def _setup_fonts(self):
        """Die benannten Tk-Schriften einmal zentral setzen.

        Der Standard ist auf vielen Linux-Systemen ein sehr kleines Bitmapfont.
        Es wird die erste Schrift genommen, die tatsaechlich vorhanden ist.
        """
        available = set(tkfont.families())

        base = tkfont.nametofont("TkDefaultFont")
        for family in ("Ubuntu", "Noto Sans", "DejaVu Sans", "Segoe UI", "Cantarell"):
            if family in available:
                base.configure(family=family)
                break
        base.configure(size=10)
        tkfont.nametofont("TkTextFont").configure(**base.actual())

        mono = tkfont.nametofont("TkFixedFont")
        for family in ("Ubuntu Mono", "DejaVu Sans Mono", "Noto Sans Mono",
                       "Liberation Mono", "Consolas"):
            if family in available:
                mono.configure(family=family)
                break
        mono.configure(size=10)

    def _setup_style(self):
        """Ein Farbschema fuer alle ttk-Elemente. clam laesst sich als
        einziges mitgeliefertes Thema durchgaengig umfaerben."""
        style = ttk.Style(self)
        style.theme_use("clam")

        style.configure(".", background=BG, foreground=TEXT,
                        fieldbackground=SURFACE, bordercolor=BORDER,
                        focuscolor=ACCENT)
        style.configure("TFrame", background=BG)
        style.configure("TLabel", background=BG, foreground=TEXT)
        style.configure("Card.TFrame", background=SURFACE)
        style.configure("Card.TLabel", background=SURFACE)
        style.configure("Title.TLabel", background=SURFACE, foreground=TEXT,
                        font=("TkDefaultFont", 17, "bold"))
        style.configure("Muted.TLabel", background=SURFACE, foreground=MUTED)
        style.configure("Section.TLabel", background=SURFACE, foreground=MUTED,
                        font=("TkDefaultFont", 9, "bold"))
        style.configure("Foot.TLabel", background=BG, foreground=MUTED,
                        font=("TkFixedFont", 10))
        # Wurde an fuenf Stellen benutzt und nie angelegt. Ein ttk-Stil ohne
        # eigene Angaben erbt von TLabel, und dessen Hintergrund ist BG --
        # auf einer Karte also ein leicht falscher Ton, und die Festbreite
        # fehlte ganz. Faellt erst auf, wenn Zahlen untereinander stehen.
        style.configure("Mono.TLabel", background=SURFACE, foreground=TEXT,
                        font=("TkFixedFont", 10))
        style.configure("Tcp.TLabel", background=SURFACE, foreground=OK_GREEN,
                        font=("TkFixedFont", 9))

        style.configure("TButton", background=BUTTON, foreground=TEXT,
                        bordercolor=BORDER, lightcolor=BUTTON, darkcolor=BUTTON,
                        relief="flat", padding=(12, 7), focusthickness=0)
        style.map("TButton",
                  background=[("pressed", BUTTON_DOWN), ("active", BUTTON_HOVER),
                              ("disabled", BG)],
                  foreground=[("disabled", DISABLED)])

        style.configure("Mini.TButton", background=SURFACE, foreground=MUTED,
                        bordercolor=BORDER, lightcolor=SURFACE, darkcolor=SURFACE,
                        relief="flat", padding=(8, 2), focusthickness=0,
                        font=("TkDefaultFont", 9))
        style.map("Mini.TButton",
                  background=[("pressed", BUTTON_DOWN), ("active", BUTTON)],
                  foreground=[("active", TEXT)])

        style.configure("TCheckbutton", background=SURFACE, foreground=TEXT,
                        indicatorcolor=SURFACE, focuscolor=SURFACE)
        style.map("TCheckbutton", background=[("active", SURFACE)],
                  indicatorcolor=[("selected", ACCENT)])

        style.configure("TCombobox", fieldbackground=SURFACE, background=BUTTON,
                        arrowcolor=TEXT, bordercolor=BORDER, padding=4)

        style.configure("TNotebook", background=BG, borderwidth=0, tabmargins=(0, 0, 0, 0))
        style.configure("TNotebook.Tab", background=BG, foreground=MUTED,
                        bordercolor=BORDER, padding=(16, 8), font=("TkDefaultFont", 10))
        style.map("TNotebook.Tab",
                  background=[("selected", SURFACE)],
                  foreground=[("selected", TEXT)],
                  expand=[("selected", (0, 0, 0, 0))])

        style.configure("Treeview", background=SURFACE, fieldbackground=SURFACE,
                        foreground=TEXT, rowheight=26, borderwidth=0, relief="flat")
        self.tree_indent = 20                          # Vorgabe aelterer Tk-Fassungen
        try:
            # Eine serielle Kette staffelt sich sonst quer aus der Spalte.
            style.configure("Treeview", indent=TREE_INDENT)   # erst ab Tk 8.6.11
            self.tree_indent = TREE_INDENT
        except tk.TclError:
            pass
        style.configure("Treeview.Heading", background=HEADING, foreground=MUTED,
                        relief="flat", padding=(6, 7),
                        font=("TkDefaultFont", 9, "bold"))
        style.map("Treeview.Heading", background=[("active", BUTTON_HOVER)])
        style.map("Treeview", background=[("selected", SELECT)],
                  foreground=[("selected", TEXT)])

        for orient in ("Vertical", "Horizontal"):
            style.configure(f"{orient}.TScrollbar", background=SCROLL,
                            troughcolor=SURFACE, bordercolor=SURFACE,
                            lightcolor=SCROLL, darkcolor=SCROLL,
                            arrowcolor=MUTED, borderwidth=0)
            style.map(f"{orient}.TScrollbar", background=[("active", MUTED)])

        style.configure("Joint.Horizontal.TScale", background=SURFACE,
                        troughcolor=TROUGH, bordercolor=BORDER,
                        lightcolor=ACCENT, darkcolor=ACCENT, borderwidth=0)
        style.map("Joint.Horizontal.TScale",
                  background=[("disabled", SURFACE)],
                  troughcolor=[("disabled", DISABLED_BG)],
                  lightcolor=[("disabled", DISABLED)],
                  darkcolor=[("disabled", DISABLED)])

        style.configure("TSeparator", background=BORDER)

    # -- kleine Bausteine --------------------------------------------------

    def _card(self, parent, **grid):
        """Weisse Flaeche mit duennem Rand. Traegt alle Inhaltsbereiche."""
        frame = tk.Frame(parent, background=SURFACE, bd=0,
                         highlightthickness=1, highlightbackground=BORDER,
                         highlightcolor=BORDER)
        if grid:
            frame.grid(**grid)
        return frame

    def _entry(self, parent, width=10, value="", font=None):
        """Eingabefeld mit Rand in der Themenfarbe und blauem Fokusrahmen."""
        entry = tk.Entry(parent, width=width, justify="right",
                         font=font or ("TkFixedFont", 10),
                         background=SURFACE, foreground=TEXT,
                         disabledbackground=BG, disabledforeground=DISABLED,
                         insertbackground=TEXT, relief="flat",
                         highlightthickness=1, highlightbackground=BORDER,
                         highlightcolor=ACCENT)
        if value:
            entry.insert(0, value)
        return entry

    def _flat_button(self, parent, text, command, colour, hover, width=9,
                     height=1, size=11):
        button = tk.Button(parent, text=text, command=command,
                           background=colour, foreground="white",
                           activebackground=hover, activeforeground="white",
                           font=("TkDefaultFont", size, "bold"),
                           width=width, height=height, relief="flat",
                           bd=0, highlightthickness=0, cursor="hand2")
        button.bind("<Enter>", lambda _e: button.configure(
            background=button.hover_colour))
        button.bind("<Leave>", lambda _e: button.configure(
            background=button.base_colour))
        button.base_colour, button.hover_colour = colour, hover
        return button

    # ----------------------------------------------------------------------
    # Aufbau
    # ----------------------------------------------------------------------

    def _build_ui(self):
        self.columnconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)
        self._build_head()
        self._build_body()
        self._build_foot()

    def _build_head(self):
        """STOP, Robotername, Meldungsfenster und die Dateitasten."""
        outer = ttk.Frame(self, padding=(PAD, PAD, PAD, 0))
        outer.grid(row=0, column=0, sticky="ew")
        outer.columnconfigure(0, weight=1)

        card = self._card(outer, row=0, column=0, sticky="ew")
        card.columnconfigure(1, weight=1)
        inner = ttk.Frame(card, style="Card.TFrame", padding=(14, 12))
        inner.grid(row=0, column=0, sticky="nsew")
        card.columnconfigure(0, weight=1)
        inner.columnconfigure(1, weight=1)

        self.stop_button = self._flat_button(
            inner, "STOP", self._toggle_stop, COLOR_STOP, COLOR_STOP_HOVER,
            width=8, height=4, size=14)
        self.stop_button.grid(row=0, column=0, rowspan=2, sticky="ns", padx=(0, 16))

        title = ttk.Frame(inner, style="Card.TFrame")
        title.grid(row=0, column=1, sticky="ew")
        ttk.Label(title, textvariable=self.robot_var, style="Title.TLabel").pack(
            side="left")
        ttk.Label(title, textvariable=self.info_var, style="Muted.TLabel").pack(
            side="left", padx=(16, 0), pady=(6, 0))

        log = tk.Frame(inner, background=LOG_BG, bd=0, highlightthickness=1,
                       highlightbackground=BORDER)
        log.grid(row=1, column=1, sticky="nsew", pady=(10, 0))
        log.columnconfigure(0, weight=1)
        self.messages = tk.Text(log, height=7, wrap="none", state="disabled",
                                background=LOG_BG, foreground=TEXT,
                                relief="flat", highlightthickness=0,
                                padx=10, pady=8, font=("TkFixedFont", 10),
                                spacing1=1, spacing3=1)
        self.messages.grid(row=0, column=0, sticky="nsew")
        scroll = ttk.Scrollbar(log, orient="vertical", command=self.messages.yview)
        scroll.grid(row=0, column=1, sticky="ns")
        self.messages.configure(yscrollcommand=scroll.set)
        self.messages.tag_configure("time", foreground=FAINT)
        for tag, colour in (("info", TEXT), ("ok", OK_GREEN),
                            ("warn", WARN), ("error", COLOR_STOP)):
            self.messages.tag_configure(tag, foreground=colour)

        tools = ttk.Frame(inner, style="Card.TFrame")
        tools.grid(row=0, column=2, rowspan=2, sticky="ne", padx=(16, 0))
        load = ttk.Button(tools, text="Laden...", command=self._choose, width=14)
        load.pack(fill="x")
        self.model_buttons.append(load)
        edit = ttk.Button(tools, text="Bearbeiten...", command=self._open_editor,
                          width=14)
        edit.pack(fill="x", pady=(6, 0))
        self.model_buttons.append(edit)
        drive = ttk.Button(tools, text="Fahren...", command=self._open_drive,
                           width=14)
        drive.pack(fill="x", pady=(6, 0))
        params = ttk.Button(tools, text="Parameter...", command=self._open_params,
                            width=14)
        params.pack(fill="x", pady=(6, 0))
        self.save_button = ttk.Button(tools, text="Speichern", command=self._save,
                                      width=14, state="disabled")
        self.save_button.pack(fill="x", pady=(6, 0))
        ttk.Button(tools, text="Speichern unter...", command=self._save_as,
                   width=14).pack(fill="x", pady=(6, 0))
        ttk.Checkbutton(tools, text="TF senden", variable=self.send_tf,
                        command=self._toggle_tf).pack(anchor="w", pady=(10, 0))

    def _build_body(self):
        """Gelenke links, Frames in der Mitte, Sollwerte rechts."""
        body = ttk.Frame(self, padding=(PAD, PAD, PAD, 0))
        body.grid(row=1, column=0, sticky="nsew")
        body.rowconfigure(0, weight=1)
        body.columnconfigure(0, weight=1, minsize=420)
        body.columnconfigure(1, weight=2, minsize=430)   # Baum braucht Breite
        self.body = body

        left = ttk.Notebook(body)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, PAD))
        self.joint_table = self._make_table(left, JOINT_COLUMNS)
        self.link_table = self._make_table(left, LINK_COLUMNS)
        left.add(self.joint_table.master, text="Gelenke")
        left.add(self.link_table.master, text="Links")

        middle = self._card(body, row=0, column=1, sticky="nsew", padx=(0, PAD))
        middle.rowconfigure(1, weight=1)
        middle.columnconfigure(0, weight=1)
        self.frames_card = middle
        header = ttk.Frame(middle, style="Card.TFrame", padding=(12, 8, 10, 6))
        header.grid(row=0, column=0, sticky="ew")
        ttk.Label(header, text="FRAMES", style="Section.TLabel").pack(side="left")
        ttk.Button(header, text="Ausblenden", style="Mini.TButton",
                   command=self._toggle_frames).pack(side="right")
        ttk.Separator(header, orient="vertical").pack(
            side="right", fill="y", padx=8, pady=2)
        ttk.Button(header, text="Einklappen", style="Mini.TButton",
                   command=lambda: self._frames_open(False)).pack(side="right")
        ttk.Button(header, text="Aufklappen", style="Mini.TButton",
                   command=lambda: self._frames_open(True)).pack(
            side="right", padx=(0, 6))

        self.frame_table = self._make_table(middle, FRAME_COLUMNS, padding=(1, 0),
                                            tree=FRAME_TREE)
        self.frame_table.master.grid(row=1, column=0, sticky="nsew")
        self.frame_table.tag_configure("tcp", background=TINT_OK)
        self.frame_table.tag_configure("orphan", background=TINT_BAD)
        self.frame_table.tag_configure("stripe", background=STRIPE)

        # Schmaler Streifen, der die Karte zurueckholt. Liegt in derselben
        # Spalte und wird nur im ausgeblendeten Zustand eingeblendet.
        self.frames_strip = self._card(body)
        self.frames_strip.columnconfigure(0, weight=1)
        strip_button = tk.Button(
            self.frames_strip, text="\n".join("FRAMES"), command=self._toggle_frames,
            background=SURFACE, foreground=MUTED, activebackground=BUTTON,
            activeforeground=TEXT, font=("TkDefaultFont", 9, "bold"),
            relief="flat", bd=0, highlightthickness=0, cursor="hand2",
            padx=4, pady=10)
        strip_button.grid(row=0, column=0, sticky="new")

        # Die TCP-Lage steht bewusst ausserhalb der Karte: sie soll auch dann
        # sichtbar bleiben, wenn die Frames weggeklappt sind.
        tcp = self._card(body, row=1, column=0, columnspan=2, sticky="ew",
                         pady=(PAD, 0))
        tcp_inner = ttk.Frame(tcp, style="Card.TFrame", padding=(12, 7))
        tcp_inner.pack(fill="x")
        ttk.Label(tcp_inner, textvariable=self.tcp_var, style="Tcp.TLabel").pack(
            side="left")

        right = ttk.Frame(body, width=370)
        right.grid(row=0, column=2, rowspan=2, sticky="ns")
        right.grid_propagate(False)
        right.rowconfigure(0, weight=1)
        right.columnconfigure(0, weight=1)

        self.target_book = ttk.Notebook(right)
        self.target_book.grid(row=0, column=0, sticky="nsew")
        self.cart_tab = tk.Frame(self.target_book, background=SURFACE,
                                 padx=16, pady=14)
        self.joint_tab = tk.Frame(self.target_book, background=SURFACE,
                                  padx=16, pady=14)
        self.target_book.add(self.cart_tab, text="Kartesisch")
        self.target_book.add(self.joint_tab, text="Joint")
        self.target_book.select(self.joint_tab)
        self._build_cart_tab()

        motion = self._card(right, row=1, column=0, sticky="ew", pady=(PAD, 0))
        motion.columnconfigure(0, weight=1)
        inner = ttk.Frame(motion, style="Card.TFrame", padding=(16, 12))
        inner.grid(row=0, column=0, sticky="ew")
        inner.columnconfigure(1, weight=1)
        ttk.Label(inner, text="BEWEGUNG", style="Section.TLabel").grid(
            row=0, column=0, columnspan=3, sticky="w", pady=(0, 8))
        for row, (key, label, unit, default) in enumerate(MOTION_FIELDS, start=1):
            ttk.Label(inner, text=label, style="Card.TLabel").grid(
                row=row, column=0, sticky="w", pady=3)
            entry = self._entry(inner, width=9, value=f"{default:g}")
            entry.grid(row=row, column=1, sticky="e", padx=(10, 8), pady=3)
            ttk.Label(inner, text=unit, style="Muted.TLabel", width=5).grid(
                row=row, column=2, sticky="w")
            self.motion_entries[key] = entry
        ttk.Button(inner, text="Uebernehmen", command=self._apply_motion).grid(
            row=len(MOTION_FIELDS) + 1, column=0, columnspan=3, sticky="w",
            pady=(10, 0))
        self._build_record_card(right)

    def _build_record_card(self, parent):
        """Bahnrekorder: aufzeichnen, verwerfen, ausgeben."""
        card = self._card(
        parent, row=2, column=0, sticky="ew", pady=(PAD, 0)
        )
        card.columnconfigure(0, weight=1)

        inner = ttk.Frame(
            card, style="Card.TFrame", padding=(16, 12)
        )
        inner.grid(row=0, column=0, sticky="ew")
        inner.columnconfigure(1, weight=1)

        ttk.Label(
            inner, text="BAHN", style="Section.TLabel"
        ).grid(
            row=0, column=0, sticky="w", pady=(0, 8)
        )

        ttk.Label(
            inner,
            text=f"{RECORD_RATE_HZ:.0f} Hz",
            style="Muted.TLabel"
        ).grid(
            row=0, column=1, sticky="e", pady=(0, 8)
        )

        self.record_button = self._flat_button(
            inner, "Aufzeichnen", self._toggle_record,
            COLOR_GO, COLOR_GO_HOVER, width=12
        )
        self.record_button.grid(
            row=1, column=0, columnspan=2, sticky="w"
        )

        buttons = ttk.Frame(inner, style="Card.TFrame")
        buttons.grid(
            row=2, column=0, columnspan=2,
            sticky="w", pady=(8, 0)
        )

        ttk.Button(
            buttons, text="Verwerfen",
            command=self._clear_record
        ).pack(side="left")

        ttk.Button(
            buttons, text="Export...",
            command=self._export_record
        ).pack(side="left", padx=(8, 0))

        self.record_label = ttk.Label(
            inner,
            textvariable=self.record_var,
            style="Mono.TLabel"
        )
        self.record_label.grid(
            row=3, column=0, columnspan=2,
            sticky="w", pady=(10, 0)
        )


    def _build_cart_tab(self):
        """Zielvorgabe im Basiskoordinatensystem."""
        tab = self.cart_tab
        tab.columnconfigure(1, weight=1)

        ttk.Label(tab, text="ZIEL IM BASISSYSTEM", style="Section.TLabel").grid(
            row=0, column=0, columnspan=3, sticky="w", pady=(0, 10))

        for row, (key, label, unit) in enumerate(CART_FIELDS, start=1):
            pad = (10, 3) if key == "roll" else (0, 3)
            ttk.Label(tab, text=label, style="Card.TLabel").grid(
                row=row, column=0, sticky="w", pady=pad)
            entry = self._entry(tab, width=12, value="0.000")
            entry.grid(row=row, column=1, sticky="e", padx=(10, 8), pady=pad)
            ttk.Label(tab, text=unit, style="Muted.TLabel", width=4).grid(
                row=row, column=2, sticky="w", pady=pad)
            self.cart_entries[key] = entry

        buttons = tk.Frame(tab, background=SURFACE)
        buttons.grid(row=len(CART_FIELDS) + 1, column=0, columnspan=3,
                     sticky="w", pady=(16, 0))
        go = self._flat_button(buttons, "Go", self._cart_go, COLOR_GO,
                               COLOR_GO_HOVER)
        go.pack(side="left")
        self.go_buttons.append(go)
        ttk.Button(buttons, text="TCP uebernehmen",
                   command=self._cart_from_tcp).pack(side="left", padx=(8, 0))

        note = tk.Label(tab, background=TINT_WARN, foreground=WARN, justify="left",
                        text="Rueckwaertskinematik ist noch nicht hinterlegt. "
                             "Go prueft die Eingabe und meldet das Ziel.",
                        wraplength=290, padx=10, pady=8, anchor="w")
        note.grid(row=len(CART_FIELDS) + 2, column=0, columnspan=3,
                  sticky="ew", pady=(16, 0))

    def _build_joint_inputs(self):
        """Eingabefelder passend zur geladenen URDF neu aufbauen."""
        for child in self.joint_tab.winfo_children():
            child.destroy()
        # Die Tasten dieses Reiters sind damit weg, ihre Eintraege in den
        # Listen aber nicht. Bisher blieben sie liegen und wurden bei jedem
        # Sperrvorgang uebersprungen -- die Listen wuchsen mit jedem Laden.
        # Die Go-Taste der kartesischen Seite lebt weiter, deshalb wird
        # gefiltert und nicht geleert.
        self.go_buttons = [b for b in self.go_buttons if b.winfo_exists()]
        self.drive_buttons = [b for b in self.drive_buttons if b.winfo_exists()]
        self.entries.clear()
        self.sliders.clear()
        # Die Go-Taste dieses Tabs wurde eben mit zerstoert, aus der Liste werfen.
        self.go_buttons = [b for b in self.go_buttons if b.winfo_exists()]

        movable = [j for j in self.joints if j["type"] != "fixed"]
        if not movable:
            ttk.Label(self.joint_tab, text="keine beweglichen Gelenke",
                      style="Muted.TLabel").grid(row=0, column=0, sticky="w")
            return

        # Bei vielen Achsen laeuft die Liste sonst unten aus dem Fenster und
        # die Go-Taste waere nicht mehr erreichbar. Liste scrollt, Tasten nicht.
        self.joint_tab.columnconfigure(0, weight=1)
        self.joint_tab.rowconfigure(1, weight=1)
        ttk.Label(self.joint_tab, text="ACHSWERTE", style="Section.TLabel").grid(
            row=0, column=0, sticky="w", pady=(0, 10))

        area = tk.Frame(self.joint_tab, background=SURFACE)
        area.grid(row=1, column=0, sticky="nsew")
        area.rowconfigure(0, weight=1)
        area.columnconfigure(0, weight=1)

        canvas = tk.Canvas(area, background=SURFACE, highlightthickness=0, bd=0)
        canvas.grid(row=0, column=0, sticky="nsew")
        bar = ttk.Scrollbar(area, orient="vertical", command=canvas.yview)
        bar.grid(row=0, column=1, sticky="ns")
        canvas.configure(yscrollcommand=bar.set)

        rows = tk.Frame(canvas, background=SURFACE)
        window = canvas.create_window((0, 0), window=rows, anchor="nw")

        def resized(_event=None):
            canvas.configure(scrollregion=canvas.bbox("all"))
            canvas.itemconfigure(window, width=canvas.winfo_width())
            needed = rows.winfo_reqheight()
            if needed > canvas.winfo_height() + 2:
                bar.grid()
            else:
                bar.grid_remove()
        rows.bind("<Configure>", resized)
        canvas.bind("<Configure>", resized)

        def wheel(event):
            step = -1 if getattr(event, "num", 0) == 4 or event.delta > 0 else 1
            canvas.yview_scroll(step, "units")
        for widget in (canvas, rows):
            widget.bind("<MouseWheel>", wheel)         # Windows und macOS
            widget.bind("<Button-4>", wheel)           # X11
            widget.bind("<Button-5>", wheel)

        rows.columnconfigure(1, weight=1)

        for index, joint in enumerate(movable):
            name = joint["name"]
            row = 2 * index                            # Zeile Wert, darunter Schieber

            ttk.Label(rows, text=name, style="Card.TLabel").grid(
                row=row, column=0, sticky="w")

            low, high = slider_range(joint)
            step = slider_step(joint)
            digits = value_digits(step)

            entry = self._entry(rows, width=10, value=f"{0.0:.{digits}f}")
            entry.grid(row=row, column=1, sticky="e", padx=(10, 8))
            entry.bind("<Return>", lambda _event: self._apply())
            entry.bind("<FocusOut>", lambda _event: self._apply())
            self.entries[name] = entry
            if joint["mimic"] is not None:
                entry.configure(state="disabled")      # wird gerechnet

            ttk.Label(rows, text="deg" if is_rotational(joint) else "m",
                      style="Muted.TLabel", width=4).grid(
                row=row, column=2, sticky="w")

            places = limit_digits(joint)
            ttk.Label(rows, text=f"{low:.{places}f} .. {high:.{places}f}",
                      style="Muted.TLabel").grid(row=row, column=3, sticky="e",
                                                 padx=(8, 0))

            if joint["mimic"] is not None:
                # Kein Regler: das Gelenk hat keinen eigenen Freiheitsgrad
                # mehr. Statt eines gesperrten Griffs, den man vergeblich
                # zieht, steht dort die Rechenvorschrift.
                mimic = joint["mimic"]
                offset = (math.degrees(mimic["offset"]) if is_rotational(joint)
                          else mimic["offset"])
                ttk.Label(rows, text=f"gekoppelt:  {mimic['multiplier']:.4g} * "
                                     f"{mimic['joint']}"
                                     f"{offset:+.4g}".rstrip("0").rstrip(".")
                                     + ("  deg" if is_rotational(joint) else "  m"),
                          style="Muted.TLabel").grid(
                    row=row + 1, column=0, columnspan=4, sticky="w",
                    pady=(2, 10))
                continue

            slider = ttk.Scale(rows, from_=low, to=high,
                               orient="horizontal", style="Joint.Horizontal.TScale",
                               command=lambda value, n=name: self._on_slider(n, value))
            slider.grid(row=row + 1, column=0, columnspan=4, sticky="ew",
                        pady=(2, 10), padx=(0, 4))
            # Erst beim Loslassen ins Protokoll, sonst laeuft es beim Ziehen voll.
            slider.bind("<ButtonRelease-1>", lambda _e: self._slider_done())
            # Pfeiltasten gehen um genau eine Stufe. Bewusst kein Mausrad:
            # beim Rollen durch die Achsliste wuerde sonst ungewollt ein
            # Sollwert verstellt.
            for key, direction in (("<Left>", -1), ("<Down>", -1),
                                   ("<Right>", 1), ("<Up>", 1)):
                slider.bind(key, lambda _e, n=name, d=direction: self._nudge(n, d))
            self.sliders[name] = slider

        buttons = tk.Frame(self.joint_tab, background=SURFACE)
        buttons.grid(row=2, column=0, sticky="w", pady=(10, 0))
        # "Go" bleibt, was es war: den Sollwert stellen, sofort, ohne Bahn.
        # Das ist der Weg ohne MoveIt und muss weiter funktionieren.
        go = self._flat_button(buttons, "Go", self._apply, COLOR_GO, COLOR_GO_HOVER)
        go.pack(side="left")
        self.go_buttons.append(go)
        ttk.Button(buttons, text="Nullstellung", command=self._reset).pack(
            side="left", padx=(8, 0))
        # Seit 2.7 steht hier keine Fahrtaste mehr. Dieser Reiter stellt, er
        # faehrt nicht -- das Ziel einer Fahrt gehoert in das Fahrfenster.
        ttk.Button(buttons, text="Fahren...", command=self._open_drive).pack(
            side="left", padx=(8, 0))
        self._sync_sliders()                           # sonst stehen sie am Anschlag
        # Das Fahrfenster kennt dieselben Gelenke und muss mitgehen. Ein
        # gehaltener Plan gehoert zu einem Modell, das es nicht mehr gibt.
        self._drop_plan()
        self._build_drive_targets()
        self._update_locks()

    def _build_foot(self):
        foot = ttk.Frame(self, padding=(PAD + 4, 8))
        foot.grid(row=2, column=0, sticky="ew")
        foot.columnconfigure(1, weight=1)
        ttk.Label(foot, textvariable=self.session_var, style="Foot.TLabel").grid(
            row=0, column=0, sticky="w")
        ttk.Label(foot, textvariable=self.path_var, style="Foot.TLabel").grid(
            row=0, column=1)
        ttk.Label(foot, textvariable=self.state_var, style="Foot.TLabel").grid(
            row=0, column=2, sticky="e", padx=(0, 18))
        ttk.Label(foot, textvariable=self.moveit_var, style="Foot.TLabel").grid(
            row=0, column=3, sticky="e", padx=(0, 18))
        ttk.Label(foot, textvariable=self.refresh_var, style="Foot.TLabel").grid(
            row=0, column=4, sticky="e")

    def _make_table(self, parent, columns, padding=(0, 0), tree=None):
        """Treeview mit Karte drumherum. Der waagerechte Balken erscheint nur,
        wenn die Spalten wirklich breiter sind als das Fenster.

        Mit `tree` als (Ueberschrift, Breite) bekommt die Tabelle die Spalte #0
        und damit Einrueckung samt Klapppfeilen.
        """
        frame = tk.Frame(parent, background=SURFACE, bd=0,
                         highlightthickness=0)
        keys = [c[0] for c in columns]
        table = ttk.Treeview(frame, columns=keys, selectmode="browse",
                             show="tree headings" if tree else "headings")
        if tree:
            table.heading("#0", text=tree[0], anchor="w")
            table.column("#0", width=tree[1], minwidth=110, stretch=False)
        for key, heading, width, anchor in columns:
            table.heading(key, text=heading, anchor=anchor)
            table.column(key, width=width, minwidth=60, anchor=anchor,
                         stretch=(key == keys[-1]))

        yscroll = ttk.Scrollbar(frame, orient="vertical", command=table.yview)
        xscroll = ttk.Scrollbar(frame, orient="horizontal", command=table.xview)
        table.configure(yscrollcommand=yscroll.set, xscrollcommand=xscroll.set)

        table.grid(row=0, column=0, sticky="nsew", padx=padding[0], pady=padding[1])
        yscroll.grid(row=0, column=1, sticky="ns")
        xscroll.grid(row=1, column=0, sticky="ew")
        frame.rowconfigure(0, weight=1)
        frame.columnconfigure(0, weight=1)

        def fit(_event=None):
            needed = sum(table.column(k, "width") for k in keys)
            if tree:
                needed += table.column("#0", "width")
            if needed > table.winfo_width() + 4:
                xscroll.grid()
            else:
                xscroll.grid_remove()
            if table.yview() == (0.0, 1.0):
                yscroll.grid_remove()
            else:
                yscroll.grid()
        table.bind("<Configure>", fit)
        table.fit = fit
        table.tag_configure("stripe", background=STRIPE)
        return table

    def _insert_row(self, table, index, values, tags=()):
        """Jede zweite Zeile leicht getoent, aber nur ohne eigene Farbe."""
        if not tags:
            tags = ("stripe",) if index % 2 else ()
        table.insert("", "end", tags=tags, values=values)

    # ----------------------------------------------------------------------
    # Meldungsfenster
    # ----------------------------------------------------------------------

    def _log(self, text, level="info"):
        """Zeile ins Meldungsfenster schreiben. Ersetzt die alte Statuszeile.

        Vorne faellt heraus, was ueber MAX_LOG_LINES hinausgeht. Ein
        Textwidget haelt alles, was je hineingeschrieben wurde, und jede
        weitere Einfuegung samt see("end") wird dabei langsamer. Ueber eine
        lange Sitzung mit vielen Diagnosen ist das eine Oberflaeche, die
        ohne erkennbaren Grund zaeher wird.
        """
        self.messages.configure(state="normal")
        self.messages.insert("end", f"{datetime.now():%H:%M:%S}  ", "time")
        self.messages.insert("end", f"{text}\n", level)
        count = int(self.messages.index("end-1c").split(".")[0])
        if count > MAX_LOG_LINES:
            self.messages.delete("1.0", f"{count - MAX_LOG_LINES}.0")
        self.messages.see("end")
        self.messages.configure(state="disabled")

    # ----------------------------------------------------------------------
    # STOP
    # ----------------------------------------------------------------------

    def _toggle_stop(self):
        """STOP sperrt die Sollwertvorgabe. TF laeuft weiter, das Bild bleibt stehen."""
        self.stopped = not self.stopped
        if self.stopped:
            colour, hover, text = COLOR_GO, COLOR_GO_HOVER, "FREI"
            self._log("STOP ausgeloest, Sollwerte werden nicht mehr uebernommen",
                      "error")
            # Waehrend einer Fahrt bedeutet STOP: bremsen. Nicht anhalten --
            # der Sprung auf null waere genau die Grenzverletzung, um die es
            # in dieser Version geht.
            self._brake("STOP")
        else:
            colour, hover, text = COLOR_STOP, COLOR_STOP_HOVER, "STOP"
            self._log("STOP aufgehoben", "ok")
        self.stop_button.base_colour, self.stop_button.hover_colour = colour, hover
        self.stop_button.configure(text=text, background=colour,
                                   activebackground=hover)
        self._update_locks()


    # ----------------------------------------------------------------------
    # Fahrfenster
    # ----------------------------------------------------------------------

    def _open_drive(self):
        """Ziel vorgeben, planen, fahren -- getrennt und in dieser Reihenfolge.

        Der Grund fuer ein eigenes Fenster ist kein Platzproblem. Die
        Sollwertfelder links stellen die Achse in dem Augenblick, in dem der
        Wert steht: sie sind der Weg ohne MoveIt, und er muss unmittelbar
        bleiben. Ein Fahrziel ist das Gegenteil davon -- es beschreibt einen
        Punkt, an dem der Roboter noch nicht ist. Zwei Bedeutungen in einem
        Feld ergeben keine, und genau das war der Fehler bis 2.6.

        Drei Handlungen, drei Tasten, in einer Richtung:

            Ziel setzen   schreibt nur in dieses Fenster, nichts bewegt sich
            Planen        fragt move_group, das Ergebnis wird gehalten
            Fahrt starten taktet die gehaltene Bahn ab
        """
        if self._show_window(self.drive_window):
            return

        window = tk.Toplevel(self)
        window.title("Fahren")
        window.configure(background=BG)
        window.protocol("WM_DELETE_WINDOW", self._close_drive)
        window.geometry("460x620")
        window.minsize(420, 520)
        self.drive_window = window
        self.drive_entries = {}
        self.drive_widgets = []

        window.columnconfigure(0, weight=1)
        window.rowconfigure(0, weight=1)

        card = self._card(window, row=0, column=0, sticky="nsew",
                          padx=PAD, pady=(PAD, 0))
        card.columnconfigure(0, weight=1)
        card.rowconfigure(0, weight=1)
        self.drive_target_frame = tk.Frame(card, background=SURFACE,
                                           padx=14, pady=12)
        self.drive_target_frame.grid(row=0, column=0, sticky="nsew")
        self._build_drive_targets()

        foot = self._card(window, row=1, column=0, sticky="ew",
                          padx=PAD, pady=PAD)
        foot.columnconfigure(0, weight=1)
        inner = tk.Frame(foot, background=SURFACE, padx=14, pady=12)
        inner.grid(row=0, column=0, sticky="ew")
        inner.columnconfigure(0, weight=1)

        ttk.Label(inner, text="AUFTRAG", style="Section.TLabel").grid(
            row=0, column=0, sticky="w", pady=(0, 8))
        ttk.Label(inner, textvariable=self.drive_status_var,
                  style="Mono.TLabel").grid(row=1, column=0, sticky="w")
        ttk.Label(inner, textvariable=self.drive_plan_var,
                  style="Mono.TLabel").grid(row=2, column=0, sticky="w",
                                            pady=(2, 0))

        buttons = tk.Frame(inner, background=SURFACE)
        buttons.grid(row=3, column=0, sticky="w", pady=(12, 0))

        plan = self._flat_button(buttons, "Planen", self._drive_plan,
                                 COLOR_DRIVE, COLOR_DRIVE_HOVER, width=11)
        plan.pack(side="left")
        self.drive_buttons.append(plan)

        start = self._flat_button(buttons, "Fahrt starten", self._drive_start,
                                  COLOR_GO, COLOR_GO_HOVER, width=13)
        start.pack(side="left", padx=(8, 0))
        self.start_buttons.append(start)

        halt = self._flat_button(buttons, "Bremsen",
                                 lambda: self._brake("Fahrfenster"),
                                 COLOR_STOP, COLOR_STOP_HOVER, width=10)
        halt.pack(side="left", padx=(8, 0))

        note = tk.Label(inner, background=TINT_OK, foreground=OK_GREEN,
                        justify="left", anchor="w", padx=10, pady=8,
                        wraplength=380,
                        text="Ein Ziel hier bewegt nichts. Erst Planen, dann "
                             "Fahrt starten. Waehrend der Fahrt sind die "
                             "Sollwertfelder gesperrt.")
        note.grid(row=4, column=0, sticky="ew", pady=(12, 0))

        self._drive_status()
        self._update_locks()
        # Wie das Parameterfenster bewusst ohne transient() und ohne
        # grab_set: man will nebenher am Baum sehen, wohin es geht.

    def _close_drive(self):
        if self.drive_window is not None and self.drive_window.winfo_exists():
            self.drive_window.destroy()
        self.drive_window = None
        self.drive_entries = {}
        self.drive_widgets = []
        self.drive_target_frame = None
        # Die Tasten sind mit zerstoert, ihre Eintraege nicht.
        self.drive_buttons = [b for b in self.drive_buttons if b.winfo_exists()]
        self.start_buttons = [b for b in self.start_buttons if b.winfo_exists()]

    def _build_drive_targets(self):
        """Zielfelder passend zum Modell aufbauen. Auch nach einem Ladevorgang.

        Gekoppelte Gelenke stehen nicht dabei: sie haben keinen eigenen
        Freiheitsgrad, ein Ziel fuer sie waere keins.
        """
        frame = getattr(self, "drive_target_frame", None)
        if frame is None or not frame.winfo_exists():
            return
        for child in frame.winfo_children():
            child.destroy()
        self.drive_entries = {}

        frame.columnconfigure(1, weight=1)
        ttk.Label(frame, text="ZIEL IM ACHSRAUM", style="Section.TLabel").grid(
            row=0, column=0, columnspan=4, sticky="w", pady=(0, 10))

        movable = [j for j in self.joints
                   if j["type"] != "fixed" and j["mimic"] is None]
        if not movable:
            ttk.Label(frame, text="keine beweglichen Gelenke",
                      style="Muted.TLabel").grid(row=1, column=0, sticky="w")
            return

        for index, joint in enumerate(movable, start=1):
            name = joint["name"]
            step = slider_step(joint)
            digits = value_digits(step)
            value = self.positions.get(name, 0.0)
            shown = math.degrees(value) if is_rotational(joint) else value

            ttk.Label(frame, text=name, style="Card.TLabel").grid(
                row=index, column=0, sticky="w", pady=3)
            entry = self._entry(frame, width=10, value=f"{shown:.{digits}f}")
            entry.grid(row=index, column=1, sticky="e", padx=(10, 8), pady=3)
            # Eine Zahl im Feld ist noch kein Auftrag: Return plant, es faehrt
            # weiterhin nichts von allein.
            entry.bind("<Return>", lambda _e: self._drive_plan())
            self.drive_entries[name] = entry

            ttk.Label(frame, text="deg" if is_rotational(joint) else "m",
                      style="Muted.TLabel", width=4).grid(
                row=index, column=2, sticky="w", pady=3)

            low, high = slider_range(joint)
            places = limit_digits(joint)
            ttk.Label(frame, text=f"{low:.{places}f} .. {high:.{places}f}",
                      style="Muted.TLabel").grid(row=index, column=3,
                                                 sticky="e", padx=(8, 0), pady=3)

        row = len(movable) + 1
        buttons = tk.Frame(frame, background=SURFACE)
        buttons.grid(row=row, column=0, columnspan=4, sticky="w", pady=(14, 0))
        for text, command in (("Ist-Werte", self._drive_from_current),
                              ("Nullstellung", self._drive_zero)):
            button = ttk.Button(buttons, text=text, command=command, width=13)
            button.pack(side="left", padx=(0, 8))
            self.drive_widgets.append(button)

    def _drive_from_current(self):
        """Zielfelder auf den aktuellen Achsstand setzen. Bewegt nichts."""
        for joint in self.joints:
            entry = self.drive_entries.get(joint["name"])
            if entry is None:
                continue
            value = self.positions.get(joint["name"], 0.0)
            shown = math.degrees(value) if is_rotational(joint) else value
            entry.configure(background=BG_OK)
            entry.delete(0, "end")
            entry.insert(0, f"{shown:.{value_digits(slider_step(joint))}f}")

    def _drive_zero(self):
        """Zielfelder auf null setzen. Bewegt nichts -- anders als _reset."""
        for joint in self.joints:
            entry = self.drive_entries.get(joint["name"])
            if entry is None:
                continue
            entry.configure(background=BG_OK)
            entry.delete(0, "end")
            entry.insert(0, f"{0.0:.{value_digits(slider_step(joint))}f}")

    def _drive_status(self):
        """Eine Zeile Zustand im Fahrfenster."""
        if not self.ros.moveit_ready:
            hint = "move_group antwortet nicht"
        elif self.moveit_stale:
            hint = "move_group ist veraltet"
        elif self.stopped:
            hint = "STOP steht an"
        else:
            hint = self.moveit_config.planner_id
            if not self.moveit_config.check_collisions:
                hint += ", ohne Kollisionspruefung"

        # Waehrend PLANEN laeuft die Uhr mit. Ohne sie ist ein wartendes
        # Fenster von einem stehenden nicht zu unterscheiden, und der
        # Bediener haelt nach drei Sekunden das Programm fuer haengend --
        # zu Recht, denn er hat keinen Grund fuer etwas anderes.
        if self.state == STATE_PLANNING and self.plan_started is not None:
            waited = time.monotonic() - self.plan_started
            hint = f"{hint}, wartet {waited:.1f} s"
        self.drive_status_var.set(f"Zustand  {self.state}   {hint}")

    # ----------------------------------------------------------------------
    # Sperren
    # ----------------------------------------------------------------------

    def _busy(self):
        """True, solange nichts veraendert werden darf."""
        return self.lock_reason is not None

    def _lock(self, reason):
        """Oberflaeche sperren. Ein Grund, eine Stelle, eine Anzeige.

        Waehrend einer Fahrt darf weder ein Bewegungsparameter noch das
        Modell angefasst werden. Beides aus demselben Grund: die geplante
        Bahn wurde mit den Werten gerechnet, die beim Planen galten. Aendert
        sich waehrenddessen eine Achsgrenze oder faellt ein Gelenk weg, faehrt
        der Roboter eine Bahn, die zu nichts mehr passt.
        """
        self.lock_reason = reason
        self._update_locks()
        self._log(f"Gesperrt: {reason}", "warn")

    def _unlock(self):
        if self.lock_reason is None:
            return
        reason = self.lock_reason
        self.lock_reason = None
        self._update_locks()
        self._log(f"Freigegeben nach {reason}", "ok")

    def begin_motion(self, what="Fahrt"):
        """Haken fuer die Bahnausfuehrung der naechsten Version."""
        self._lock(what)

    def end_motion(self):
        self._unlock()

    def _model_locked(self, what="Das Modell"):
        """Meldet und liefert True, wenn gerade nichts geaendert werden darf."""
        if not self._busy():
            return False
        self._log(f"{what} laesst sich waehrend '{self.lock_reason}' "
                  f"nicht aendern", "error")
        return True

    def _update_locks(self):
        """Einzige Stelle, an der Bedienelemente scharf oder stumpf werden.

        Zwei voneinander unabhaengige Gruende: STOP sperrt die Sollwerte,
        eine laufende Fahrt sperrt zusaetzlich alles, was die Rechnung unter
        der Bahn wegziehen wuerde.
        """
        busy = self._busy()
        # Waehrend einer Fahrt sind die Sollwertfelder gesperrt. Sie stellen
        # die Achse unmittelbar; wer waehrend des Abtastens hineinschreibt,
        # kaempft gegen den Interpolationstakt und gewinnt fuer einen Takt.
        setpoints = "disabled" if (self.stopped or busy) else "normal"
        model = "disabled" if busy else "normal"

        for entry in self.entries.values():
            entry.configure(state=setpoints)
        for entry in self.cart_entries.values():
            entry.configure(state=setpoints)
        for slider in self.sliders.values():
            slider.configure(state=setpoints)
        for entry in self.motion_entries.values():
            entry.configure(state=setpoints)
        for button in self.go_buttons:
            if button.winfo_exists():
                button.configure(
                    state=setpoints,
                    background=DISABLED_BG if setpoints == "disabled"
                    else button.base_colour)

        # Die Zielfelder gehoeren zum Auftrag und sind waehrend seiner
        # Ausfuehrung ebenso gesperrt: die Bahn ist gerechnet, das Ziel
        # nachtraeglich zu aendern hiesse, sie waere es nicht.
        for entry in self.drive_entries.values():
            if entry.winfo_exists():
                entry.configure(state=setpoints)
        for widget in self.drive_widgets:
            if widget.winfo_exists():
                widget.configure(state=setpoints)

        # Geplant wird nur aus BEREIT oder FEHLER heraus, und nur wenn
        # move_group antwortet und dasselbe Modell liest.
        plannable = (not busy and not self.stopped and self.ros.moveit_ready
                     and not self.moveit_stale)
        for button in self.drive_buttons:
            if button.winfo_exists():
                button.configure(
                    state="normal" if plannable else "disabled",
                    background=button.base_colour if plannable else DISABLED_BG)

        # Gefahren wird nur mit einem gehaltenen Plan. Ohne ihn ist die Taste
        # stumpf, und das ist die ganze Aussage dieser Version: zwischen
        # Rechnen und Bewegen liegt eine Handlung.
        startable = plannable and self.plan_hold is not None
        for button in self.start_buttons:
            if button.winfo_exists():
                button.configure(
                    state="normal" if startable else "disabled",
                    background=button.base_colour if startable else DISABLED_BG)

        for button in self.model_buttons:
            if button.winfo_exists():
                button.configure(state=model)
        if self.undo_button is not None and self.undo_button.winfo_exists():
            self.undo_button.configure(
                state="normal" if (self.history and not busy) else "disabled")

        for widget in self.param_widgets:
            if widget.winfo_exists():
                widget.configure(state=model)
        self._params_status()
        self._drive_status()

    # ----------------------------------------------------------------------
    # Ablaufsteuerung
    # ----------------------------------------------------------------------

    def _set_state(self, state, why=""):
        """Einziger Weg, den Zustand zu wechseln.

        Der Zustand bestimmt die Sperre: in PLANEN, FAHREN und BREMSEN darf
        weder ein Bewegungsparameter noch das Modell angefasst werden. Beides
        aus demselben Grund -- die Bahn wurde mit den Werten gerechnet, die
        beim Planen galten.
        """
        if state == self.state:
            return
        self.state = state
        text = state if not why else f"{state}  {why}"
        self.state_var.set(text)

        if state in STATES_BUSY:
            self.lock_reason = state
        else:
            self.lock_reason = None
        self._update_locks()

        level = {"FEHLER": "error", "BREMSEN": "warn"}.get(state, "info")
        self._log(f"Zustand {text}", level)
        self._drive_status()

    # ----------------------------------------------------------------------
    # PTP fahren
    # ----------------------------------------------------------------------

    def _drive_plan(self):
        """Achsziel aus dem Fahrfenster planen lassen. Es faehrt noch nichts.

        Bis 2.6 hing beides an einer Taste, und die Zielwerte standen in
        denselben Feldern, die die Achse sofort stellen. Damit war der Start
        immer gleich dem Ziel und die Bahn leer. Seit 2.7 sind es drei
        getrennte Handlungen: stellen, planen, fahren.

        Ein Ziel im Achsraum braucht keine IK, keine Lage- und
        Drehbedingungen und keine Bahngrenzen. Nur den Auftrag.
        """
        if self.state in STATES_BUSY:
            self._log(f"Es laeuft bereits: {self.state}", "error")
            return
        if self.stopped:
            self._log("STOP steht an, es wird nicht gefahren", "error")
            return
        if not self.ros.moveit_ready:
            self._log("move_group antwortet nicht. Im Parameterfenster "
                      "starten oder extern hochfahren.", "error")
            return
        if self.moveit_stale:
            self._log("move_group liest ein anderes Modell als dieses. "
                      "Erst neu starten.", "error")
            return

        if not self.drive_entries:
            self._log("Das Fahrfenster ist nicht offen, kein Ziel vorhanden",
                      "error")
            return

        target = self._read_joint_targets()
        if target is None:
            return
        if not target:
            self._log("Kein bewegliches Gelenk vorhanden", "error")
            return

        start = {name: float(value) for name, value in self.positions.items()}
        if all(abs(target[name] - start.get(name, 0.0)) < 1e-9 for name in target):
            self._drop_plan()
            self._log("Ziel entspricht dem aktuellen Stand, nichts zu planen",
                      "warn")
            return

        # Wandert unveraendert bis in den MotionPlanRequest. Reine Zahlen und
        # Zeichenketten: der GUI-Thread baut keine ROS-Nachrichten.
        plan = self.moveit_config.plan
        payload = {
            "group": self.moveit_config.group,
            "planner_id": self.moveit_config.planner_id,
            "start": start,
            "goal": target,
            "tolerance": math.radians(float(plan["goal_orientation_tol"])),
            "velocity_scaling": float(plan["velocity_scaling"]) / 100.0,
            "acceleration_scaling": float(plan["acceleration_scaling"]) / 100.0,
            "planning_time": float(plan["planning_time"]),
            "attempts": max(1, int(plan["planning_attempts"])),
        }
        # Die Frist gehoert zum Auftrag und nicht in eine feste Zahl: wer die
        # Planungszeit hochsetzt, meint es so, und eine starre Frist von 20 s
        # wuerde ihm dazwischenfahren. Der Zuschlag deckt Hin- und Rueckweg
        # und das, was MoveIt vor dem eigentlichen Planen tut.
        payload["timeout"] = (payload["planning_time"] * payload["attempts"]
                              + PLAN_TIMEOUT_MARGIN_S)

        self._drop_plan()
        self.plan_hold_start = start
        self.plan_hold_goal = dict(target)
        self.plan_started = time.monotonic()
        self._set_state(STATE_PLANNING, payload["planner_id"])
        self.ros.request_plan(payload)

    def _read_joint_targets(self):
        """Zielwerte lesen, begrenzen und faerben. None heisst: unlesbar.

        Dieselbe Pruefung wie in _apply, nur ohne die Werte zu stellen -- das
        macht die Bahn. Gelesen wird seit 2.7 aus den Zielfeldern des
        Fahrfensters und nicht mehr aus den Sollwertfeldern. Gekoppelte
        Gelenke bleiben draussen, sie folgen ihrem Fuehrungsgelenk.
        """
        target = {}
        invalid = 0
        clamped = 0
        for joint in self.joints:
            if joint["type"] == "fixed" or joint["mimic"] is not None:
                continue
            entry = self.drive_entries.get(joint["name"])
            if entry is None:
                continue
            try:
                typed = float(entry.get().replace(",", "."))
            except ValueError:
                entry.configure(background=BG_INVALID)
                invalid += 1
                continue

            value = math.radians(typed) if is_rotational(joint) else typed
            if joint["has_limit"]:
                limited = min(max(value, joint["lower"]), joint["upper"])
            else:
                limited = value
            if abs(limited - value) > 1e-12:
                clamped += 1
                entry.configure(background=BG_CLAMPED)
                shown = math.degrees(limited) if is_rotational(joint) else limited
                entry.delete(0, "end")
                entry.insert(0, f"{shown:.{value_digits(slider_step(joint))}f}")
            else:
                entry.configure(background=BG_OK)
            target[joint["name"]] = limited

        if invalid:
            self._log(f"{invalid} Eingabe(n) nicht lesbar, es wird nicht "
                      "gefahren", "error")
            return None
        if clamped:
            self._log(f"{clamped} Ziel(e) auf das URDF-Limit begrenzt", "warn")
        return target

    # -- Antwort ------------------------------------------------------------

    def _on_plan_result(self, result):
        """Ergebnis des Planers. Kommt ueber _tick aus dem Executor-Thread."""
        if self.state != STATE_PLANNING:
            return                                     # abgebrochen, verworfen

        took = (time.monotonic() - self.plan_started
                if self.plan_started else 0.0)
        self.plan_started = None

        if result.get("error"):
            self._set_state(STATE_ERROR, "Planung")
            self._log(f"Kein Plan: {result['error']}", "error")
            self._plan_diagnosis()
            return

        code = int(result.get("code", 0))
        if code != 1:
            self._set_state(STATE_ERROR, "Planung")
            self._log(f"Kein Plan: {moveit_error_text(code)}", "error")
            self._plan_diagnosis()
            return

        points = result.get("points") or []
        if len(points) < 2:
            # Steht der Roboter schon am Ziel, liefert Pilz einen einzelnen
            # Punkt. Das ist kein Fehler, es gibt nur nichts zu fahren.
            self._set_state(STATE_READY)
            self._log("Ziel bereits erreicht, keine Fahrt noetig", "ok")
            return

        try:
            player = TrajectoryPlayer(points)
        except ValueError as exc:
            self._set_state(STATE_ERROR, "Bahn")
            self._log(str(exc), "error")
            return

        # Der Plan wird gehalten, nicht gefahren. Zwischen Rechnen und
        # Bewegen liegt seit 2.7 eine bewusste Handlung des Bedieners --
        # dieselbe Trennung, die eine Steuerung zwischen Programmwahl und
        # Startfreigabe kennt.
        self.plan_hold = player
        self._set_state(STATE_READY)
        self._log(f"Plan in {took * 1000:.0f} ms, {len(points)} Stuetzpunkte, "
                  f"Bahndauer {player.duration:.3f} s. Zum Fahren "
                  f"'Fahrt starten'.", "ok")
        self.drive_plan_var.set(f"Plan bereit  {len(points)} Punkte, "
                                f"{player.duration:.2f} s")
        self._update_locks()

    def _plan_diagnosis(self):
        """Nach einem gescheiterten Auftrag zeigen, womit gefragt wurde.

        Ein Fehlercode allein sagt selten genug. Was gefragt wurde, weiss die
        GUI; warum nicht geantwortet wurde, weiss move_group. Beides gehoert
        nebeneinander, sonst sucht man an der falschen Stelle -- und bei
        einem unbestimmten Code sucht man garantiert falsch, weil dieser Code
        keine Aussage ueber die Planung ist, sondern eine ueber ihr
        Ausbleiben.
        """
        goal = self.plan_hold_goal or {}
        self._log(f"Auftrag war: Gruppe {self.moveit_config.group}, "
                  f"Planer {self.moveit_config.planner_id}, Pipeline "
                  f"pilz_industrial_motion_planner, {len(goal)} Achsziele "
                  f"({', '.join(sorted(goal)) or 'keine'})", "warn")

        chain = self._srdf_chain()
        if chain:
            base, tip = chain
            self._log(f"SRDF-Kette: {base} .. {tip}", "warn")

        self._moveit_log_tail(10)

    def _srdf_chain(self):
        """base_link und tip_link aus der zuletzt erzeugten SRDF lesen.

        Stimmen die Achsen im Ziel nicht mit der Kette ueberein, ist das der
        haeufigste Grund fuer eine abgelehnte Zielbedingung. Ein Blick darauf
        kostet nichts und beantwortet die Frage sofort.
        """
        match = re.search(r'<chain\s+base_link="([^"]*)"\s+tip_link="([^"]*)"',
                          self.moveit_srdf or "")
        return match.groups() if match else None

    def _drive_start(self):
        """Die gehaltene Bahn abfahren. Der einzige Weg in eine Bewegung.

        Geprueft wird nicht nur, ob ein Plan da ist, sondern auch, ob der
        Roboter noch dort steht, wo er beim Planen stand. Sonst faehrt er von
        einem anderen Punkt aus eine Bahn, die dafuer nicht gerechnet wurde --
        der erste Stuetzpunkt waere ein Sprung.
        """
        if self.state in STATES_BUSY:
            self._log(f"Es laeuft bereits: {self.state}", "error")
            return
        if self.stopped:
            self._log("STOP steht an, es wird nicht gefahren", "error")
            return
        if self.plan_hold is None:
            self._log("Kein Plan vorhanden. Erst 'Planen'.", "error")
            return

        start = self.plan_hold_start or {}
        drift = max((abs(self.positions.get(name, 0.0) - value)
                     for name, value in start.items()), default=0.0)
        if drift > 1e-9:
            self._drop_plan("Achsstand hat sich seit dem Planen geaendert")
            return

        player = self.plan_hold
        self.plan_hold = None
        self._start_playing(player)

    def _drop_plan(self, reason=""):
        """Gehaltenen Plan verwerfen. Eine Stelle, damit keine Bahn liegenbleibt.

        Gerufen aus _push_positions, also von jeder Aenderung eines
        Sollwerts. Ein Plan, dessen Startpunkt nicht mehr gilt, ist kein
        halber Plan, sondern keiner.
        """
        if self.plan_hold is None:
            self.plan_hold_start = None
            self.plan_hold_goal = None
            self.drive_plan_var.set("kein Plan")
            return
        self.plan_hold = None
        self.plan_hold_start = None
        self.plan_hold_goal = None
        self.drive_plan_var.set("kein Plan")
        if reason:
            self._log(f"Plan verworfen: {reason}", "warn")
        self._update_locks()

    # -- Abfahren -----------------------------------------------------------

    def _start_playing(self, player):
        self.plan_hold = None                          # er wird jetzt gefahren
        self.plan_hold_start = None
        self.drive_plan_var.set("wird gefahren")
        self.player = player
        self.play_started = time.monotonic()
        self._set_state(STATE_MOVING)
        self._motion_tick()

    def _motion_tick(self):
        """Ein Interpolationstakt. Der eine Ort, an dem die Bahn vorrueckt."""
        self.motion_job = None
        player = self.player
        if player is None:
            return

        values = player.step()
        for name, value in values.items():
            if name in self.positions:
                self.positions[name] = value

        # Derselbe Sendeweg wie beim Schieber. Es gibt weiterhin genau einen.
        self._push_positions()
        self._sync_sliders()
        self._sync_joint_entries()

        if player.braking and player.stopped:
            self._finish_playing("angehalten", "warn")
            return
        if player.done:
            self._finish_playing("Ziel erreicht", "ok")
            return

        self.motion_job = self.after(IPO_TICK_MS, self._motion_tick)

    def _finish_playing(self, reason, level="ok"):
        player = self.player
        if player is not None and self.play_started is not None:
            elapsed = time.monotonic() - self.play_started
            drift = player.wall_drift(elapsed)
            self._log(f"{reason}: {player.ticks} Takte, "
                      f"Bahnzeit {player.t:.3f} s, Wanduhr {elapsed:.3f} s, "
                      f"Abweichung {drift * 1000:+.0f} ms", level)
        self.player = None
        self.play_started = None
        if self.motion_job is not None:
            self.after_cancel(self.motion_job)
            self.motion_job = None
        self._update_kinematics()
        self._set_state(STATE_READY)

    def _brake(self, why):
        """Anhalten einleiten. Die Bahn wird nicht verlassen, nur langsamer.

        Ein Sprung auf null waere genau die Grenzverletzung, um die es in
        dieser Version geht. Deshalb faehrt die Zeitrate ueber BRAKE_TIME_S
        auf null, und der Roboter bleibt dabei auf seiner Bahn stehen.
        """
        if self.state == STATE_PLANNING:
            # Der Auftrag laeuft schon. Die Antwort wird verworfen, weil der
            # Zustand nicht mehr PLANEN ist.
            self.plan_started = None
            self.plan_hold_start = None
            self.plan_hold_goal = None
            self.drive_plan_var.set("kein Plan")
            self._set_state(STATE_READY, "Planung verworfen")
            self._log(f"Planung abgebrochen ({why})", "warn")
            return
        if self.player is None or self.state != STATE_MOVING:
            return
        self.player.brake()
        self._set_state(STATE_BRAKING, why)

    # ----------------------------------------------------------------------
    # MoveIt: Zustand und Kindprozess
    # ----------------------------------------------------------------------

    def _moveit_stale(self, reason):
        """Konfiguration passt nicht mehr zu dem, was move_group gelesen hat.

        Wird aus _commit heraus gesetzt, also an genau der Stelle, an der das
        Modell sich aendert, und aus dem Uebernehmen der Parameter.
        """
        if not self.moveit.alive:
            return
        if self.moveit_stale:
            return
        self.moveit_stale = True
        self._log(f"move_group ist veraltet ({reason}), Neustart noetig", "warn")
        self._params_status()

    def _moveit_start(self):
        """move_group mit dem aktuellen Stand hochfahren.

        Waehrend des Hochlaufens ist die Oberflaeche gesperrt: der Prozess
        liest gerade die Datei, die aus dem Modell entstanden ist.
        """
        if self._busy():
            self._log("Neustart nicht moeglich, es laeuft bereits etwas", "error")
            return
        if self.xml_root is None:
            self._log("Kein Modell geladen", "error")
            return

        srdf = build_srdf(self.robot_name, self.links, self.joints,
                          self.moveit_config.group,
                          self.moveit_config.tip or None)
        self.moveit_srdf = srdf

        # Vor dem Start: der alte Client gehoert zu einem Server, den es
        # gleich nicht mehr gibt. Der leere Name sperrt die Probe solange.
        self.ros.reset_moveit("")

        self._lock("Neustart move_group")
        error = self.moveit.start(self.moveit_config, self.urdf_xml, srdf)
        if error:
            self._log(error, "error")
            self._unlock()
            self._params_status()
            return

        # Ab hier weiss die Probe, welcher Knoten der eigene ist.
        self.ros.reset_moveit(self.moveit.node_name)

        strays = self.moveit.strays()
        if strays:
            self._log(f"Achtung: {len(strays)} weitere move_group laufen "
                      f"(PID {', '.join(str(p) for p in strays)}). Sie bedienen "
                      "dieselben Dienste. Im Parameterfenster beenden.", "warn")

        self.moveit_stale = False
        self.moveit_wait_since = time.monotonic()
        self._params_status()
        # Der Hochlauf braucht ein paar Sekunden. Solange bleibt gesperrt,
        # der Rest laeuft weiter -- die Freigabe kommt aus _tick, sobald der
        # Dienst antwortet oder die Wartezeit abgelaufen ist.

    def _moveit_stop(self):
        if self.moveit.alive:
            self.moveit.stop()
            self._log("move_group beendet", "ok")
            rest = self.moveit.strays()
            if rest:
                self._log(f"Es laufen weiter {len(rest)} fremde move_group "
                          f"(PID {', '.join(str(p) for p in rest)})", "warn")
        self.ros.reset_moveit("")
        self.moveit_stale = False
        self.moveit_wait_since = None
        if self.lock_reason == "Neustart move_group":
            self._unlock()
        self._params_status()

    def _moveit_kill_strays(self):
        """Fremde move_group beenden.

        Sie sind der haeufigste Grund dafuer, dass ein Neustart nichts
        aendert: der neue Knoten faehrt hoch, aber die Anfrage landet beim
        alten, weil beide denselben Dienst bedienen und die Zuordnung in DDS
        beim zuerst gefundenen bleibt.
        """
        pids = self.moveit.strays()
        if not pids:
            self._log("Keine fremden move_group gefunden", "ok")
            return
        if not messagebox.askyesno(
                "Fremde move_group beenden",
                f"{len(pids)} move_group laufen ausserhalb dieser GUI "
                f"(PID {', '.join(str(p) for p in pids)}).\n\n"
                "Sie bedienen dieselben Dienste wie der eigene Knoten. "
                "Beenden?"):
            return
        count = self.moveit.kill_strays()
        self._log(f"{count} fremde move_group beendet", "ok")
        if self.moveit.alive:
            # Der Client kann an einen der beendeten Server gebunden sein.
            self.ros.reset_moveit(self.moveit.node_name)
        self._params_status()

    def _tf_watchdog(self, age_ms):
        """Melden, wenn die TF-Ausgabe stehenbleibt.

        Der Takt laeuft bei TF_RATE_HZ, ein Alter von ueber einer Sekunde ist
        also das Dreissigfache des Erwarteten und kein Zufall mehr. Bis 2.10
        stand das nur in einer Zahl in der Fusszeile, die immer weiter
        hochlief -- und eine Zahl, die hochlaeuft, liest man als "es dauert",
        nicht als "es ist vorbei". Der Unterschied ist der ganze Punkt.

        Gemeldet wird einmal, nicht in jedem Durchlauf. Eine Warnung, die
        zwanzigmal je Sekunde kommt, ist Rauschen.
        """
        broken = age_ms > TF_STALL_MS
        if broken and not self.tf_stalled:
            self.tf_stalled = True
            alive = self.ros.thread_alive()
            self._log(f"Keine TF-Ausgabe seit {age_ms / 1000.0:.1f} s. "
                      + ("Der ROS-Takt laeuft, aber nichts wird gesendet."
                         if alive else
                         "Der ROS-Takt ist beendet. RViz zeigt ab jetzt einen "
                         "Stand, den es nicht mehr gibt."), "error")
        elif not broken and self.tf_stalled:
            self.tf_stalled = False
            self._log("TF-Ausgabe laeuft wieder", "ok")

    def _moveit_watch(self):
        """Aus _tick heraus. Wartet auf move_group und gibt dann frei."""
        if self.moveit_wait_since is None:
            return
        if not self.moveit.alive:
            self._log("move_group ist nicht mehr da: "
                      + (self.moveit.last_error or "Prozess beendet"), "error")
            self._moveit_log_tail()
            self.moveit_wait_since = None
            self.ros.reset_moveit("")
            if self.lock_reason == "Neustart move_group":
                self._unlock()
            return
        if self.ros.moveit_ready:
            self.moveit_wait_since = None
            self._log("move_group antwortet, Planung steht bereit", "ok")
            if self.lock_reason == "Neustart move_group":
                self._unlock()
            return
        if time.monotonic() - self.moveit_wait_since > MOVEIT_WAIT_S:
            self.moveit_wait_since = None
            self._log(f"move_group meldet sich seit {MOVEIT_WAIT_S:.0f} s nicht. "
                      "Haeufigste Ursache: /joint_states fehlt oder die "
                      "Planungsgruppe passt nicht zur SRDF.", "error")
            self._moveit_log_tail()
            if self.lock_reason == "Neustart move_group":
                self._unlock()

    def _moveit_log_tail(self, lines=8):
        """Die letzten Zeilen von move_group ins Protokoll holen.

        Bis 2.4 gingen sie nach DEVNULL. Damit war ein Fehlstart nicht
        aufklaerbar: der Prozess war weg, der Arbeitsordner geloescht, und
        uebrig blieb die Zahl in state(). Die Meldung, die erklaert, woran es
        lag, steht fast immer in der letzten Zeile.
        """
        tail = self.moveit.log_tail(lines)
        if not tail:
            self._log("Keine Ausgabe von move_group vorhanden", "warn")
            return
        self._log(f"Letzte Zeilen von move_group ({self.moveit.logfile}):", "warn")
        for line in tail:
            self._log("  " + line[:200], "warn")

    def _moveit_text(self):
        """Eine Zeile Zustand, gleich lautend in Fusszeile und Fenster."""
        state = self.moveit.state()
        if state == "aus":
            return "move_group  aus"
        if state in ("beendet", "fehler"):
            return f"move_group  {state}"
        if self.moveit_wait_since is not None:
            return "move_group  startet, wartet auf Antwort"
        if self.moveit_stale:
            return "move_group  laeuft, aber veraltet"
        uptime = self.moveit.uptime()
        ready = "bereit" if self.ros.moveit_ready else "keine Antwort"
        return f"move_group  {ready}, seit {uptime:.0f} s"

    # ----------------------------------------------------------------------
    # Parameterfenster
    # ----------------------------------------------------------------------

    def _open_params(self):
        """Alle MoveIt-Werte an einer Stelle, nach Wirkung getrennt.

        Die Aufteilung folgt nicht der Ordnung von MoveIt, sondern der Frage,
        was ein Wert kostet. Was pro Auftrag wirkt, steht vorne. Was erst
        beim naechsten Start von move_group gelesen wird, steht dahinter und
        ist als solches gekennzeichnet.
        """
        if self._show_window(self.params_window):
            return

        window = tk.Toplevel(self)
        window.title("Bewegungsparameter")
        window.configure(background=BG)
        window.protocol("WM_DELETE_WINDOW", self._close_params)
        window.geometry("620x620")
        window.minsize(560, 520)
        self.params_window = window
        self.param_entries = {}
        self.param_widgets = []
        self.limit_entries = {}

        window.columnconfigure(0, weight=1)
        window.rowconfigure(0, weight=1)

        book = ttk.Notebook(window)
        book.grid(row=0, column=0, sticky="nsew", padx=PAD, pady=(PAD, 0))

        pages = [("Fahrt", self._build_param_plan),
                 ("Achsgrenzen", self._build_param_limits),
                 ("Kartesisch", self._build_param_cartesian),
                 ("Kinematik", self._build_param_kinematics),
                 ("Pipeline", self._build_param_pipeline)]
        for title, builder in pages:
            page = tk.Frame(book, background=SURFACE, padx=16, pady=14)
            book.add(page, text=title)
            builder(page)

        foot = self._card(window, row=1, column=0, sticky="ew",
                          padx=PAD, pady=PAD)
        foot.columnconfigure(0, weight=1)
        inner = ttk.Frame(foot, style="Card.TFrame", padding=(14, 10))
        inner.grid(row=0, column=0, sticky="ew")
        inner.columnconfigure(0, weight=1)

        self.params_status_label = ttk.Label(
            inner, textvariable=self.moveit_var, style="Mono.TLabel")
        self.params_status_label.grid(row=0, column=0, sticky="w")

        buttons = ttk.Frame(inner, style="Card.TFrame")
        buttons.grid(row=1, column=0, sticky="w", pady=(10, 0))
        for text, command in (("Uebernehmen", self._params_apply),
                              ("Neu starten", self._params_restart),
                              ("Stoppen", self._moveit_stop),
                              ("Fremde beenden", self._moveit_kill_strays)):
            button = ttk.Button(buttons, text=text, command=command, width=14)
            button.pack(side="left", padx=(0, 8))
            self.param_widgets.append(button)

        self._params_fill()
        self._params_status()
        # Bewusst kein _hold_grab und kein transient(self): das Fenster soll
        # neben dem Hauptfenster benutzbar bleiben -- man will einen Wert
        # aendern und gleich danach am Schieber sehen, was er bewirkt. Aus
        # demselben Grund wie beim Bearbeiten-Fenster bekommt es so auch
        # einen eigenen Eintrag in der Fensterleiste und ist nach dem
        # Minimieren wieder erreichbar.

    def _close_params(self):
        if self.params_window is not None and self.params_window.winfo_exists():
            self.params_window.destroy()
        self.params_window = None
        self.param_entries = {}
        self.param_widgets = []
        self.limit_entries = {}
        self.params_status_label = None

    def _param_row(self, parent, row, key, label, unit, width=12):
        """Eine Zeile Beschriftung, Feld, Einheit. Merkt sich das Feld."""
        ttk.Label(parent, text=label, style="Card.TLabel").grid(
            row=row, column=0, sticky="w", pady=3)
        entry = self._entry(parent, width=width)
        entry.grid(row=row, column=1, sticky="e", padx=(10, 8), pady=3)
        ttk.Label(parent, text=unit, style="Muted.TLabel", width=6).grid(
            row=row, column=2, sticky="w")
        self.param_entries[key] = entry
        self.param_widgets.append(entry)
        return entry

    @staticmethod
    def _param_note(parent, row, text, tint=TINT_WARN, colour=WARN):
        note = tk.Label(parent, background=tint, foreground=colour,
                        justify="left", text=text, wraplength=520,
                        padx=10, pady=8, anchor="w")
        note.grid(row=row, column=0, columnspan=3, sticky="ew", pady=(14, 0))
        return note

    def _build_param_plan(self, page):
        page.columnconfigure(1, weight=1)
        ttk.Label(page, text="PRO AUFTRAG", style="Section.TLabel").grid(
            row=0, column=0, columnspan=3, sticky="w", pady=(0, 10))

        ttk.Label(page, text="Fahrart", style="Card.TLabel").grid(
            row=1, column=0, sticky="w", pady=3)
        combo = ttk.Combobox(page, textvariable=self.planner_var,
                             values=list(PLANNER_IDS), state="readonly",
                             width=10)
        combo.grid(row=1, column=1, sticky="e", padx=(10, 8), pady=3)
        self.param_widgets.append(combo)

        for row, (key, label, unit, _default) in enumerate(PLAN_FIELDS, start=2):
            self._param_row(page, row, key, label, unit)

        self._param_note(
            page, len(PLAN_FIELDS) + 2,
            "Diese Werte wirken beim naechsten Auftrag, ohne Neustart. "
            "Achtung beim Override: 0 bedeutet in MoveIt nicht Halt, sondern "
            "wird intern auf 100 Prozent gesetzt. Die Oberflaeche laesst 0 "
            "deshalb nicht zu.", TINT_OK, OK_GREEN)

    def _build_param_limits(self, page):
        """Achsgrenzen als Tabelle, eine Zeile je bewegliches Gelenk."""
        page.columnconfigure(0, weight=1)
        page.rowconfigure(1, weight=1)

        ttk.Label(page, text="ACHSGRENZEN   (Neustart noetig)",
                  style="Section.TLabel").grid(row=0, column=0, sticky="w",
                                               pady=(0, 10))

        holder = tk.Frame(page, background=SURFACE)
        holder.grid(row=1, column=0, sticky="nsew")
        holder.columnconfigure(0, weight=1)
        holder.rowconfigure(0, weight=1)

        canvas = tk.Canvas(holder, background=SURFACE, highlightthickness=0)
        canvas.grid(row=0, column=0, sticky="nsew")
        scroll = ttk.Scrollbar(holder, orient="vertical", command=canvas.yview)
        scroll.grid(row=0, column=1, sticky="ns")
        canvas.configure(yscrollcommand=scroll.set)

        table = tk.Frame(canvas, background=SURFACE)
        canvas.create_window((0, 0), window=table, anchor="nw")
        table.bind("<Configure>", lambda _e: canvas.configure(
            scrollregion=canvas.bbox("all")))

        ttk.Label(table, text="Gelenk", style="Card.TLabel").grid(
            row=0, column=0, sticky="w", padx=(0, 10), pady=(0, 6))
        for column, (_key, label, unit, _width) in enumerate(LIMIT_FIELDS,
                                                             start=1):
            ttk.Label(table, text=f"{label} [{unit}]",
                      style="Muted.TLabel").grid(row=0, column=column,
                                                 padx=4, pady=(0, 6))

        names = self.moveit_config.sync_joints(self.joints)
        for row, name in enumerate(names, start=1):
            ttk.Label(table, text=name, style="Mono.TLabel").grid(
                row=row, column=0, sticky="w", padx=(0, 10), pady=2)
            self.limit_entries[name] = {}
            for column, (key, _label, _unit, width) in enumerate(LIMIT_FIELDS,
                                                                 start=1):
                entry = self._entry(table, width=9)
                entry.grid(row=row, column=column, padx=4, pady=2)
                self.limit_entries[name][key] = entry
                self.param_widgets.append(entry)

        if not names:
            ttk.Label(table, text="kein bewegliches Gelenk",
                      style="Muted.TLabel").grid(row=1, column=0,
                                                 columnspan=5, sticky="w")

        buttons = tk.Frame(page, background=SURFACE)
        buttons.grid(row=2, column=0, sticky="w", pady=(12, 0))
        reset = ttk.Button(buttons, text="Aus URDF fuellen",
                           command=self._params_limits_from_urdf, width=18)
        reset.pack(side="left")
        self.param_widgets.append(reset)

        self._param_note(
            page, 3,
            "Leeres Feld heisst: Grenze aus. Geschwindigkeit kommt aus der "
            "URDF, Beschleunigung und Ruck stehen dort nicht und sind "
            "abgeleitet. Die Verzoegerung ist negativ, so erwartet Pilz sie. "
            "Pilz rechnet ausserdem mit der strengsten Achse fuer alle.")

    def _build_param_cartesian(self, page):
        page.columnconfigure(1, weight=1)
        ttk.Label(page, text="BAHNGRENZEN   (Neustart noetig)",
                  style="Section.TLabel").grid(row=0, column=0, columnspan=3,
                                               sticky="w", pady=(0, 10))
        for row, (key, label, unit, _default) in enumerate(CARTESIAN_FIELDS,
                                                           start=1):
            self._param_row(page, row, key, label, unit)
        self._param_note(
            page, len(CARTESIAN_FIELDS) + 1,
            "Gilt fuer LIN und CIRC. Eine eigene Drehbeschleunigung gibt es "
            "nicht: sie wird als Bahnbeschleunigung durch Bahngeschwindigkeit "
            "mal Drehgeschwindigkeit gerechnet.")

    def _build_param_kinematics(self, page):
        page.columnconfigure(1, weight=1)
        ttk.Label(page, text="INVERSE KINEMATIK   (Neustart noetig)",
                  style="Section.TLabel").grid(row=0, column=0, columnspan=3,
                                               sticky="w", pady=(0, 10))

        ttk.Label(page, text="Solver", style="Card.TLabel").grid(
            row=1, column=0, sticky="w", pady=3)
        combo = ttk.Combobox(page, textvariable=self.solver_var,
                             values=list(KINEMATICS_SOLVERS), state="readonly",
                             width=42)
        combo.grid(row=1, column=1, columnspan=2, sticky="e", padx=(10, 0),
                   pady=3)
        self.param_widgets.append(combo)

        for row, (key, label, unit, _default) in enumerate(KINEMATICS_FIELDS,
                                                           start=2):
            self._param_row(page, row, key, label, unit)

        ttk.Label(page, text="Planungsgruppe", style="Card.TLabel").grid(
            row=len(KINEMATICS_FIELDS) + 2, column=0, sticky="w", pady=3)
        group = self._entry(page, width=18)
        group.grid(row=len(KINEMATICS_FIELDS) + 2, column=1, sticky="e",
                   padx=(10, 8), pady=3)
        self.param_entries["group"] = group
        self.param_widgets.append(group)

        ttk.Label(page, text="Tip-Link", style="Card.TLabel").grid(
            row=len(KINEMATICS_FIELDS) + 3, column=0, sticky="w", pady=3)
        tip = self._entry(page, width=18)
        tip.grid(row=len(KINEMATICS_FIELDS) + 3, column=1, sticky="e",
                 padx=(10, 8), pady=3)
        self.param_entries["tip"] = tip
        self.param_widgets.append(tip)

        self._param_note(
            page, len(KINEMATICS_FIELDS) + 4,
            "Die Zeitgrenze gilt je Aufruf. Bei LIN wird sie mit der Zahl der "
            "Stuetzpunkte multipliziert -- das ist die haeufigste Ursache "
            "dafuer, dass eine Gerade ewig braucht. Bleibt der Tip-Link leer, "
            "nimmt die SRDF das erste Blatt der Kette.")

    def _build_param_pipeline(self, page):
        page.columnconfigure(1, weight=1)
        ttk.Label(page, text="PIPELINE   (Neustart noetig)",
                  style="Section.TLabel").grid(row=0, column=0, columnspan=3,
                                               sticky="w", pady=(0, 10))

        check = ttk.Checkbutton(page, text="Ruckig-Glaettung anhaengen",
                                variable=self.ruckig_var)
        check.grid(row=1, column=0, columnspan=3, sticky="w", pady=4)
        self.param_widgets.append(check)

        check2 = ttk.Checkbutton(
            page, text="Bahn auf Kollisionen pruefen",
            variable=self.collision_var, command=self._collision_hint)
        check2.grid(row=2, column=0, columnspan=3, sticky="w", pady=4)
        self.param_widgets.append(check2)

        check3 = ttk.Checkbutton(
            page, text="Ausfuehrung durch move_group zulassen",
            variable=self.execution_var)
        check3.grid(row=3, column=0, columnspan=3, sticky="w", pady=4)
        self.param_widgets.append(check3)

        self._param_note(
            page, 4,
            "Geplant wird mit Pilz: PTP, LIN und CIRC, deterministisch und "
            "mit Trapezprofil. Ruckig glaettet danach den Ruck, ist aber "
            "keine Zeitparametrierung mit sichtbaren Phasen.\n\n"
            "Die Kollisionspruefung haengt an zwei Adaptern: einer lehnt den "
            "Auftrag vorher ab, einer wirft die fertige Bahn nachher weg. "
            "Geprueft wird gegen die Kollisionsmatrix der SRDF, und die kennt "
            "hier nur benachbarte Paare -- grobe Ersatzkoerper melden dann "
            "Treffer, die die Mechanik nicht hat. Aus heisst nicht, dass "
            "nichts kollidiert, sondern dass niemand hinsieht.\n\n"
            "Die Ausfuehrung bleibt aus: es gibt keine Controller, und die "
            "GUI tastet die geplante Bahn selbst ab. Einschalten nur, wenn "
            "ros2_control laeuft.")

    def _collision_hint(self):
        """Beim Abschalten einmal deutlich sagen, was das heisst.

        Ein Haken, der eine Pruefung ausschaltet, sollte nicht lautlos
        umgehen. Die Meldung steht im Protokoll und ist damit auch spaeter
        noch da -- wenn jemand sich wundert, warum eine Bahn durch den Sockel
        geht.
        """
        if self.collision_var.get():
            self._log("Kollisionspruefung wird beim naechsten Start von "
                      "move_group wieder eingeschaltet", "ok")
        else:
            self._log("Kollisionspruefung wird beim naechsten Start "
                      "abgeschaltet. Bahnen werden dann nicht mehr geprueft, "
                      "auch nicht auf echte Kollisionen.", "warn")

    # -- Fuellen und Lesen -------------------------------------------------

    def _params_fill(self):
        """Felder aus der Konfiguration setzen."""
        if self.params_window is None:
            return
        config = self.moveit_config

        self.planner_var.set(config.planner_id)
        self.solver_var.set(config.solver)
        self.ruckig_var.set(config.use_ruckig)
        self.collision_var.set(config.check_collisions)
        self.execution_var.set(config.allow_execution)

        def put(key, value, digits=4):
            entry = self.param_entries.get(key)
            if entry is None:
                return
            entry.configure(state="normal", background=BG_OK)
            entry.delete(0, "end")
            if value is not None:
                entry.insert(0, f"{value:.{digits}g}"
                             if isinstance(value, float) else str(value))

        for key, _label, _unit, _default in PLAN_FIELDS:
            put(key, config.plan[key])
        for key, _label, _unit, _default in CARTESIAN_FIELDS:
            put(key, config.cartesian[key])
        for key, _label, _unit, _default in KINEMATICS_FIELDS:
            put(key, config.kinematics[key], digits=6)
        put("group", config.group)
        put("tip", config.tip)

        for name, fields in self.limit_entries.items():
            values = config.limits.get(name, {})
            for key, entry in fields.items():
                entry.configure(state="normal", background=BG_OK)
                entry.delete(0, "end")
                value = values.get(key)
                if value is not None:
                    entry.insert(0, f"{float(value):.6g}")

        self._update_locks()

    def _params_limits_from_urdf(self):
        """Achsgrenzen verwerfen und neu aus dem Modell ableiten."""
        if self._busy():
            return
        for joint in self.joints:
            if joint["type"] == "fixed" or joint["mimic"] is not None:
                continue
            self.moveit_config.limits[joint["name"]] = \
                MoveItConfig.defaults_for(joint)
        self._params_fill()
        self._log("Achsgrenzen aus der URDF uebernommen", "ok")

    def _read_param(self, key, label, allow_empty=False, sign=None):
        """Ein Feld lesen und faerben. None heisst: nicht lesbar.

        Rueckgabe ist ein Paar (ok, wert). Ein leeres Feld liefert bei
        allow_empty (True, None) -- das ist die abgeschaltete Grenze.
        """
        entry = self.param_entries.get(key)
        if entry is None:
            return True, None
        text = entry.get().strip().replace(",", ".")
        if not text:
            if allow_empty:
                entry.configure(background=BG_OK)
                return True, None
            entry.configure(background=BG_INVALID)
            self._log(f"{label} fehlt", "error")
            return False, None
        try:
            value = float(text)
        except ValueError:
            entry.configure(background=BG_INVALID)
            self._log(f"{label} ist nicht lesbar", "error")
            return False, None
        if sign == "positive" and value <= 0.0:
            entry.configure(background=BG_INVALID)
            self._log(f"{label} muss groesser als null sein", "error")
            return False, None
        if sign == "negative" and value >= 0.0:
            entry.configure(background=BG_INVALID)
            self._log(f"{label} muss kleiner als null sein", "error")
            return False, None
        if sign == "notneg" and value < 0.0:
            entry.configure(background=BG_INVALID)
            self._log(f"{label} darf nicht negativ sein", "error")
            return False, None
        entry.configure(background=BG_OK)
        return True, value

    def _params_read(self):
        """Alle Felder pruefen und in eine Kopie der Konfiguration schreiben.

        Erst wenn nichts beanstandet wurde, wird uebernommen. Ein halb
        uebernommener Parametersatz waere schlimmer als gar keiner.
        """
        draft = copy.deepcopy(self.moveit_config)
        ok = True

        rules = {"velocity_scaling": "positive",
                 "acceleration_scaling": "positive",
                 "planning_time": "positive",
                 "planning_attempts": "positive",
                 "goal_position_tol": "positive",
                 "goal_orientation_tol": "positive",
                 "max_step": "positive",
                 "jump_threshold": "notneg",
                 "blend_radius": "notneg"}
        for key, label, _unit, _default in PLAN_FIELDS:
            good, value = self._read_param(key, label, sign=rules.get(key))
            ok = ok and good
            if good:
                draft.plan[key] = value
        for key in ("velocity_scaling", "acceleration_scaling"):
            if draft.plan[key] > 100.0:
                entry = self.param_entries.get(key)
                if entry is not None:
                    entry.configure(background=BG_CLAMPED)
                draft.plan[key] = 100.0
                self._log("Override auf 100 Prozent begrenzt", "warn")

        for key, label, _unit, _default in CARTESIAN_FIELDS:
            sign = "negative" if key == "max_trans_dec" else "positive"
            good, value = self._read_param(key, label, sign=sign)
            ok = ok and good
            if good:
                draft.cartesian[key] = value

        for key, label, _unit, _default in KINEMATICS_FIELDS:
            good, value = self._read_param(key, label, sign="positive")
            ok = ok and good
            if good:
                draft.kinematics[key] = value

        group = self.param_entries["group"].get().strip()
        if not NAME_PATTERN.match(group or ""):
            self.param_entries["group"].configure(background=BG_INVALID)
            self._log("Planungsgruppe ist kein gueltiger Name", "error")
            ok = False
        else:
            self.param_entries["group"].configure(background=BG_OK)
            draft.group = group

        tip = self.param_entries["tip"].get().strip()
        known = {link["name"] for link in self.links}
        if tip and tip not in known:
            self.param_entries["tip"].configure(background=BG_INVALID)
            self._log(f"Tip-Link '{tip}' steht nicht im Modell", "error")
            ok = False
        else:
            self.param_entries["tip"].configure(background=BG_OK)
            draft.tip = tip

        for name, fields in self.limit_entries.items():
            values = dict(draft.limits.get(name, {}))
            for key, entry in fields.items():
                text = entry.get().strip().replace(",", ".")
                if not text:
                    entry.configure(background=BG_OK)
                    values[key] = None
                    continue
                try:
                    value = float(text)
                except ValueError:
                    entry.configure(background=BG_INVALID)
                    self._log(f"{name}: {key} ist nicht lesbar", "error")
                    ok = False
                    continue
                if key == "deceleration" and value >= 0.0:
                    entry.configure(background=BG_INVALID)
                    self._log(f"{name}: Verzoegerung muss negativ sein", "error")
                    ok = False
                    continue
                if key != "deceleration" and value <= 0.0:
                    entry.configure(background=BG_INVALID)
                    self._log(f"{name}: {key} muss groesser als null sein",
                              "error")
                    ok = False
                    continue
                entry.configure(background=BG_OK)
                values[key] = value
            draft.limits[name] = values

        draft.planner_id = self.planner_var.get()
        draft.solver = self.solver_var.get()
        draft.use_ruckig = bool(self.ruckig_var.get())
        draft.check_collisions = bool(self.collision_var.get())
        draft.allow_execution = bool(self.execution_var.get())

        return draft if ok else None

    def _params_apply(self):
        """Uebernehmen. Meldet, was davon erst nach einem Neustart wirkt."""
        if self._busy():
            self._log("Parameter sind waehrend "
                      f"'{self.lock_reason}' gesperrt", "error")
            return False

        draft = self._params_read()
        if draft is None:
            return False

        before = self.moveit_config.startup_signature()
        self.moveit_config = draft
        self._sync_motion_from_config()
        after = draft.startup_signature()

        if before != after and self.moveit.alive:
            self._moveit_stale("Parameter geaendert")
            self._log("Uebernommen. Werte pro Auftrag wirken sofort, "
                      "die uebrigen erst nach Neu starten", "ok")
        else:
            self._log("Parameter uebernommen", "ok")
        self._params_status()
        return True

    def _params_restart(self):
        """Uebernehmen und move_group damit neu hochfahren."""
        if not self._params_apply():
            return
        self._moveit_start()

    def _params_status(self):
        self.moveit_var.set(self._moveit_text())

    def _sync_motion_from_config(self):
        """Die drei Felder im Hauptfenster auf die Bahngrenzen spiegeln.

        Sie standen bisher fuer sich und wurden nur protokolliert. Jetzt sind
        sie dieselbe Groesse wie auf der Karteikarte Kartesisch, in denselben
        Einheiten. Zwei Anzeigen, ein Wert.
        """
        self.motion["speed"] = self.moveit_config.cartesian["max_trans_vel"]
        self.motion["acceleration"] = self.moveit_config.cartesian["max_trans_acc"]
        for key in ("speed", "acceleration"):
            entry = self.motion_entries.get(key)
            if entry is None or not entry.winfo_exists():
                continue
            state = entry.cget("state")
            entry.configure(state="normal", background=BG_OK)
            entry.delete(0, "end")
            entry.insert(0, f"{self.motion[key]:g}")
            entry.configure(state=state)

    # ----------------------------------------------------------------------
    # Daten laden und speichern
    # ----------------------------------------------------------------------

    def _choose(self):
        path = filedialog.askopenfilename(
            title="URDF waehlen",
            initialdir=os.path.dirname(self.urdf_path) or ".",
            filetypes=[("URDF", "*.urdf *.xml"), ("Alle Dateien", "*.*")])
        if path:
            self._load(path)

    def _load(self, path, initial=False):
        if not initial and self._model_locked("Ein anderes Modell"):
            return
        try:
            root, name, links, joints, xml_text = parse(path)
        except FileNotFoundError:
            messagebox.showerror("URDF", f"Datei nicht gefunden:\n{path}")
            self._log(f"Datei nicht gefunden: {path}", "error")
            if initial:
                self._choose()
            return
        except ET.ParseError as exc:
            messagebox.showerror("URDF", f"XML fehlerhaft:\n{exc}")
            self._log(f"XML fehlerhaft: {exc}", "error")
            return

        self.urdf_path = path
        self.xml_root = root
        self.robot_name, self.links, self.joints, self.urdf_xml = name, links, joints, xml_text
        self.path_var.set(os.path.basename(path))
        self.positions = {j["name"]: 0.0 for j in joints if j["type"] != "fixed"}
        self.history.clear()
        self._update_undo()
        self.saved_signature = serialize(root)
        self._set_dirty(False)

        # Zweiter Weg neben _rebuild_from_root, deshalb muss die Anpassung
        # der Achsgrenzen hier ebenfalls stehen. Beim ersten Laden gibt es
        # das Parameterfenster noch nicht, spaeter wird es neu aufgebaut.
        self.moveit_config.sync_joints(joints)
        if self.params_window is not None and self.params_window.winfo_exists():
            self._close_params()
            self._open_params()
        self._moveit_stale("anderes Modell geladen")

        self._update_header()
        self._build_joint_inputs()
        self._fill_link_table()
        self._update_kinematics()
        self.tree_selected = None                      # Auswahl gehoert zum alten Modell
        self._draw_tree()
        if self.editor is not None:
            self._edit_fill_combo()

        self._log(f"Roboter {name} initialisiert, {len(links)} Links, "
                  f"{len(joints)} Gelenke", "ok")

        problems = validate_root(root)
        if problems:
            for problem in problems:
                self._log(problem, "warn")
            messagebox.showwarning("URDF", "Hinweise:\n\n" + "\n".join(problems))

        message = self.ros.set_model(joints, xml_text)
        if self.ros.running:                           # sonst steht der Hinweis
            self._log(message, "ok")                   # vom Start schon da
        self._push_positions()

    def _save(self):
        if not self.dirty:
            return
        try:
            backup = save_urdf(self.urdf_path, self.urdf_xml)
        except OSError as exc:
            messagebox.showerror("Speichern", f"Schreiben fehlgeschlagen:\n{exc}")
            self._log(f"Speichern fehlgeschlagen: {exc}", "error")
            return
        self.saved_signature = serialize(self.xml_root)
        self._set_dirty(False)
        self._log(f"Gespeichert. Sicherung: {backup}" if backup
                  else "Gespeichert.", "ok")

    def _save_as(self):
        """Aktuellen Stand als neue Datei ablegen, Original bleibt unberuehrt.

        Danach zeigt die GUI auf die neue Datei: weitere Aenderungen landen
        dort und nicht mehr im Ausgangsmodell.
        """
        folder = backup_dir_for(self.urdf_path)
        path = filedialog.asksaveasfilename(
            title="URDF speichern unter",
            initialdir=folder,
            initialfile=timestamped_name(self.urdf_path),
            defaultextension=".urdf",
            filetypes=[("URDF", "*.urdf *.xml"), ("Alle Dateien", "*.*")])
        if not path:
            return

        try:
            save_urdf(path, self.urdf_xml)
        except OSError as exc:
            messagebox.showerror("Speichern", f"Schreiben fehlgeschlagen:\n{exc}")
            self._log(f"Speichern fehlgeschlagen: {exc}", "error")
            return

        self.urdf_path = path                          # ab jetzt gilt diese Datei
        self.path_var.set(os.path.basename(path))
        self.saved_signature = serialize(self.xml_root)
        self._set_dirty(False)
        self._log(f"Gespeichert unter {path}", "ok")

    def _discard(self):
        if self.dirty and not messagebox.askyesno(
                "Neu laden", "Nicht gespeicherte Aenderungen verwerfen?"):
            return
        self._load(self.urdf_path)

    # ----------------------------------------------------------------------
    # Verlauf
    # ----------------------------------------------------------------------

    def _push_history(self, description):
        """Stand vor einer Aenderung sichern. Aeltestes faellt heraus."""
        self.history.append((description, copy.deepcopy(self.xml_root)))
        del self.history[:-MAX_HISTORY]
        self._update_undo()

    def _undo(self):
        if not self.history:
            return
        if self._model_locked("Rueckgaengig"):
            return
        description, root = self.history.pop()
        self.xml_root = root
        self._rebuild_from_root()
        self._update_undo()
        self._log(f"Rueckgaengig: {description}", "ok")

    def _update_undo(self):
        if self.undo_button is not None and self.undo_button.winfo_exists():
            self.undo_button.configure(
                state="normal" if self.history else "disabled")

    def _commit(self, description, mutate):
        """Aenderung auf einer Kopie ausfuehren, pruefen, dann uebernehmen.

        Einziger Weg, auf dem der Baum veraendert wird. Schlaegt die Pruefung
        fehl, bleibt das Modell unberuehrt und der Verlauf unveraendert.
        """
        if self.xml_root is None:
            return False
        if self._model_locked("Das Modell"):
            return False
        draft = copy.deepcopy(self.xml_root)
        try:
            mutate(draft)
        except ValueError as exc:
            messagebox.showerror("URDF", str(exc))
            self._log(str(exc), "error")
            return False

        problems = validate_root(draft)
        if problems:
            messagebox.showerror("URDF", "Nicht uebernommen:\n\n"
                                 + "\n".join(problems))
            for problem in problems:
                self._log(problem, "error")
            return False

        self._push_history(description)
        self.xml_root = draft
        self._rebuild_from_root()
        self._log(f"{description}, noch nicht gespeichert", "warn")
        # Modellaenderung heisst: move_group liest gerade etwas anderes als
        # das, was hier steht. Genau eine Stelle, an der das auffaellt.
        self._moveit_stale(description)
        self._update_locks()                           # Fahrtaste nachziehen
        return True

    def _set_dirty(self, dirty):
        self.dirty = dirty
        self.save_button.configure(state="normal" if dirty else "disabled")
        self.title(f"URDF Viewer{'  *' if dirty else ''}")

    def _update_header(self):
        roots = root_links(self.links, self.joints)
        movable = sum(1 for j in self.joints if j["type"] != "fixed")
        self.robot_var.set(self.robot_name or "kein Modell")
        self.info_var.set(
            f"{len(self.links)} Links   |   {len(self.joints)} Gelenke, "
            f"{movable} beweglich   |   Wurzel "
            f"{roots[0] if len(roots) == 1 else roots}")

    # ----------------------------------------------------------------------
    # Tabellen
    # ----------------------------------------------------------------------

    def _fill_joint_table(self):
        """Gelenke mit dem aktuellen Sollwert in rad und grad."""
        self.joint_table.delete(*self.joint_table.get_children())
        for index, joint in enumerate(self.joints):
            value = self.positions.get(joint["name"], 0.0)
            if joint["type"] == "fixed":
                primary = secondary = "-"
            elif is_rotational(joint):
                primary = f"{value:.4f}"
                secondary = f"{math.degrees(value):.2f}"
            else:
                # Bei feinem Hub reichen vier Stellen in Meter nicht aus, die
                # Tabelle waere dann groeber als das Raster des Schiebers.
                digits = value_digits(slider_step(joint))
                primary = f"{value:.{max(4, digits)}f}"
                secondary = f"{value * 1000.0:.{max(1, digits - 3)}f}"

            if joint["has_limit"]:
                lo, up = joint["lower"], joint["upper"]
                places = limit_digits(joint)
                if is_rotational(joint):
                    lo, up = math.degrees(lo), math.degrees(up)
                limit = f"{lo:.{places}f} .. {up:.{places}f}"
            elif joint["type"] == "continuous":
                limit = "unbegrenzt"
            else:
                limit = "-"

            self._insert_row(self.joint_table, index, (
                joint["name"], joint["type"], primary, secondary, limit,
                f"{joint['parent']} > {joint['child']}"))
        self.after_idle(self.joint_table.fit)

    def _fill_link_table(self):
        self.link_table.delete(*self.link_table.get_children())
        for index, link in enumerate(self.links):
            if link["has_inertial"]:
                mass, com = f"{link['mass']:.4f}", fmt(link["com"])
            else:
                mass = com = "-"
            visual = link["visual"]
            if visual is None:
                shape = colour = "-"                   # in RViz unsichtbar
            else:
                shape = SHAPE_LABELS.get(visual["shape"], "?")
                colour = rgba_to_hex(visual["rgba"]) if visual["rgba"] else "-"
            self._insert_row(self.link_table, index,
                             (link["name"], mass, com, shape, colour))
        self.after_idle(self.link_table.fit)

    # ----------------------------------------------------------------------
    # Sollwerte
    # ----------------------------------------------------------------------

    def _apply(self):
        """Eingaben lesen, pruefen, begrenzen und uebernehmen."""
        if self.stopped:
            return
        invalid = 0
        clamped = 0
        for joint in self.joints:
            entry = self.entries.get(joint["name"])
            if entry is None:
                continue

            try:
                typed = float(entry.get().replace(",", "."))
            except ValueError:
                entry.configure(background=BG_INVALID)
                invalid += 1
                continue

            value = math.radians(typed) if is_rotational(joint) else typed
            if joint["has_limit"]:
                limited = min(max(value, joint["lower"]), joint["upper"])
            else:
                limited = value

            if abs(limited - value) > 1e-12:
                clamped += 1
                entry.configure(background=BG_CLAMPED)
                shown = math.degrees(limited) if is_rotational(joint) else limited
                entry.delete(0, "end")
                entry.insert(0, f"{shown:.{value_digits(slider_step(joint))}f}")
            else:
                entry.configure(background=BG_OK)

            self.positions[joint["name"]] = limited

        self._sync_sliders()
        self._update_kinematics()
        self._push_positions()

        if invalid:
            self._log(f"{invalid} Eingabe(n) nicht lesbar, Wert unveraendert", "error")
        elif clamped:
            self._log(f"{clamped} Wert(e) auf das URDF-Limit begrenzt", "warn")
        else:
            self._log("Moved to  " + self._pose_text())

    def _joint_by_name(self, name):
        for joint in self.joints:
            if joint["name"] == name:
                return joint
        return None

    def _on_slider(self, name, value):
        """Laeuft bei jedem Pixel Zugweg. Muss deshalb billig bleiben."""
        if self.syncing or self.stopped:
            return
        joint = self._joint_by_name(name)
        if joint is None:
            return

        low, high = slider_range(joint)
        step = slider_step(joint)
        # Auf das Raster legen. Der Griff selbst bleibt beim Ziehen frei, sonst
        # zappelt er gegen die Maus; er springt beim Loslassen auf die Stufe.
        shown = snap(float(value), low, high, step)

        entry = self.entries[name]
        entry.configure(background=BG_OK)
        entry.delete(0, "end")
        entry.insert(0, f"{shown:.{value_digits(step)}f}")

        self.positions[name] = (math.radians(shown) if is_rotational(joint)
                                else shown)
        self._push_positions()                        # TF sofort, ohne Umweg
        self._schedule_refresh()

    def _nudge(self, name, direction):
        """Eine Stufe weiter. Haengt an den Pfeiltasten des Schiebers."""
        if self.stopped:
            return "break"
        joint = self._joint_by_name(name)
        slider = self.sliders.get(name)
        if joint is None or slider is None:
            return "break"
        low, high = slider_range(joint)
        step = slider_step(joint)
        shown = snap(slider.get() + direction * step, low, high, step)
        self.syncing = True
        try:
            slider.set(shown)
        finally:
            self.syncing = False
        self._on_slider(name, shown)
        self._slider_done()
        return "break"                                 # Tk-Vorgabe ausschalten

    def _schedule_refresh(self):
        """Tabellen hoechstens alle SLIDER_REFRESH_MS neu rechnen."""
        if self.refresh_job is None:
            self.refresh_job = self.after(SLIDER_REFRESH_MS, self._refresh_now)

    def _refresh_now(self):
        self.refresh_job = None
        self._update_kinematics()

    def _slider_done(self):
        """Maustaste losgelassen: einmal sauber nachziehen und melden."""
        if self.stopped:
            return
        if self.refresh_job is not None:
            self.after_cancel(self.refresh_job)
            self.refresh_job = None
        self._sync_sliders()                           # Griff auf die Stufe
        self._update_kinematics()
        self._log("Moved to  " + self._pose_text())

    def _sync_sliders(self):
        """Schieber auf die aktuellen Sollwerte setzen, ohne sie auszuloesen."""
        self.syncing = True
        try:
            for joint in self.joints:
                slider = self.sliders.get(joint["name"])
                if slider is None:
                    continue
                value = self.positions.get(joint["name"], 0.0)
                shown = math.degrees(value) if is_rotational(joint) else value
                low, high = slider_range(joint)
                slider.set(snap(shown, low, high, slider_step(joint)))
        finally:
            self.syncing = False

    def _reset(self):
        for joint in self.joints:
            entry = self.entries.get(joint["name"])
            if entry is None:
                continue
            entry.configure(background=BG_OK)
            entry.delete(0, "end")
            entry.insert(0, f"{0.0:.{value_digits(slider_step(joint))}f}")
        self._apply()

    def _push_positions(self):
        """Kopplungen auswerten, Anzeige nachziehen, dann senden.

        Einziger Weg zu ros.set_positions. Damit gibt es genau eine Stelle,
        an der aus gestellten Werten gerechnete werden, und kein Aufrufer
        kann sie versehentlich umgehen.
        """
        apply_couplings(self.joints, self.positions)
        for joint in self.joints:
            if joint["mimic"] is None:
                continue
            value = self.positions.get(joint["name"], 0.0)
            shown = math.degrees(value) if is_rotational(joint) else value
            self._set_entry(self.entries, joint["name"],
                            f"{shown:.{value_digits(slider_step(joint))}f}",
                            enabled=False)
        # Der Stand hat sich geaendert, also gilt der Startpunkt eines
        # gehaltenen Plans nicht mehr. Waehrend einer Fahrt ist nichts
        # gehalten -- _start_playing hat den Plan uebernommen --, deshalb
        # trifft die Zeile den eigenen Takt nicht.
        if self.plan_hold is not None:
            self._drop_plan("Sollwert wurde gestellt")
        self.ros.set_positions(self.positions)

    def _sync_joint_entries(self):
        """Eingabefelder auf die aktuellen Sollwerte setzen."""
        apply_couplings(self.joints, self.positions)
        for joint in self.joints:
            if self.entries.get(joint["name"]) is None:
                continue
            value = self.positions.get(joint["name"], 0.0)
            shown = math.degrees(value) if is_rotational(joint) else value
            self._set_entry(self.entries, joint["name"],
                            f"{shown:.{value_digits(slider_step(joint))}f}",
                            enabled=joint["mimic"] is None)
        self._sync_sliders()

    def _cart_from_tcp(self):
        """Zielfelder mit der aktuellen TCP-Lage vorbelegen."""
        pose = self._tcp_pose()
        if pose is None:
            self._log("Kein eindeutiger Endeffektor, nichts uebernommen", "warn")
            return
        xyz, rpy = pose
        for key, value in zip(("x", "y", "z"), xyz):
            self.cart_entries[key].delete(0, "end")
            self.cart_entries[key].insert(0, f"{value:.4f}")
        for key, value in zip(("roll", "pitch", "yaw"), rpy):
            self.cart_entries[key].delete(0, "end")
            self.cart_entries[key].insert(0, f"{math.degrees(value):.2f}")

    def _cart_go(self):
        """Kartesisches Ziel pruefen. Die Rueckwaertskinematik fehlt noch."""
        if self.stopped:
            return
        target = {}
        for key, label, _unit in CART_FIELDS:
            entry = self.cart_entries[key]
            try:
                target[key] = float(entry.get().replace(",", "."))
            except ValueError:
                entry.configure(background=BG_INVALID)
                self._log(f"{label} ist nicht lesbar", "error")
                return
            entry.configure(background=BG_OK)

        self._log("Ziel kartesisch   X {x:.4f}  Y {y:.4f}  Z {z:.4f} m   "
                  "R {roll:.2f}  P {pitch:.2f}  Y {yaw:.2f} deg".format(**target))
        self._log("Rueckwaertskinematik nicht hinterlegt, keine Achswerte berechnet",
                  "warn")

    def _apply_motion(self):
        """Jerk, Beschleunigung und Geschwindigkeit uebernehmen."""
        values = {}
        for key, label, unit, _default in MOTION_FIELDS:
            entry = self.motion_entries[key]
            try:
                value = float(entry.get().replace(",", "."))
            except ValueError:
                entry.configure(background=BG_INVALID)
                self._log(f"{label} ist nicht lesbar", "error")
                return
            if value <= 0.0:
                entry.configure(background=BG_INVALID)
                self._log(f"{label} muss groesser als null sein", "error")
                return
            entry.configure(background=BG_OK)
            values[key] = value

        self.motion = values
        # Bisher endete die Funktion hier und der Wert lag im Nichts. Speed
        # und Acceleration sind dieselbe Groesse wie die Bahngrenzen auf der
        # Karteikarte Kartesisch, in denselben Einheiten -- also wandern sie
        # dorthin. Jerk hat bei Pilz keine Entsprechung: dort wird trapezoid
        # gerechnet, den Ruck begrenzt erst Ruckig ueber die Achsgrenzen.
        self.moveit_config.cartesian["max_trans_vel"] = values["speed"]
        self.moveit_config.cartesian["max_trans_acc"] = values["acceleration"]
        self.moveit_config.cartesian["max_trans_dec"] = -values["acceleration"]
        if self.params_window is not None and self.params_window.winfo_exists():
            self._params_fill()
        self._moveit_stale("Bahngrenzen geaendert")
        self._log("Bewegung: Jerk {jerk:g}, Acceleration {acceleration:g}, "
                  "Speed {speed:g}".format(**values), "ok")
        self._log("Speed und Acceleration wirken als Bahngrenzen fuer LIN, "
                  "Jerk erst ueber die Achsgrenzen und Ruckig", "info")

    # ----------------------------------------------------------------------
    # Bahnrekorder
    # ----------------------------------------------------------------------

    def _movable_joints(self):
        return [j for j in self.joints if j["type"] != "fixed"]

    def _record_columns(self):
        """Spaltennamen der Achsen samt Einheit, wie sie ins CSV gehen."""
        return [f"{j['name']}_{'deg' if is_rotational(j) else 'm'}"
                for j in self._movable_joints()]

    def _joint_snapshot(self):
        """Achswerte in Anzeigeeinheit, Reihenfolge wie _movable_joints."""
        return [math.degrees(self.positions.get(j["name"], 0.0))
                if is_rotational(j) else self.positions.get(j["name"], 0.0)
                for j in self._movable_joints()]

    def _toggle_record(self):
        if self.recorder.active:
            self.recorder.stop()
            self._log(f"Aufzeichnung angehalten, {self.recorder.count} Punkte "
                      f"in {self.recorder.duration:.1f} s", "ok")
        else:
            kept = self.recorder.start(self._record_columns())
            self.record_warned = False
            self._log(f"Aufzeichnung laeuft, {kept} Punkte im Puffer" if kept
                      else "Aufzeichnung laeuft", "ok")
        self._update_record_button()
        self._update_record_status()

    def _update_record_button(self):
        active = self.recorder.active
        colour = COLOR_STOP if active else COLOR_GO
        hover = COLOR_STOP_HOVER if active else COLOR_GO_HOVER
        self.record_button.base_colour, self.record_button.hover_colour = colour, hover
        self.record_button.configure(text="Anhalten" if active else "Aufzeichnen",
                                     background=colour, activebackground=hover)
        self.record_label.configure(style="Tcp.TLabel" if active else "Mono.TLabel")

    def _clear_record(self):
        """Punkte verwerfen. Laeuft die Aufzeichnung, sammelt sie weiter."""
        if not self.recorder.count:
            return
        count = self.recorder.count
        self.recorder.clear()
        self._update_record_status()
        self._log(f"{count} Bahnpunkte verworfen", "warn")

    def _drop_record(self, reason):
        """Nach einem Modellwechsel passen die alten Punkte nicht mehr."""
        if not self.recorder.count:
            return
        count = self.recorder.count
        self.recorder.clear()
        self.recorder.stop()
        self._update_record_button()
        self._update_record_status()
        self._log(f"{count} Bahnpunkte verworfen: {reason}", "warn")

    def _record_tick(self):
        """Eigener Takt. Nimmt genau einen Punkt entgegen und filtert ihn."""
        if self.recorder.active:
            pose = self._tcp_pose()
            if pose is None:
                if not self.record_warned:
                    self._log("Kein eindeutiger Endeffektor, es wird nichts "
                              "aufgezeichnet", "warn")
                    self.record_warned = True
            else:
                xyz, rpy = pose
                if self.recorder.sample(xyz, rpy, self._joint_snapshot()):
                    self._update_record_status()
        self.after(RECORD_TICK_MS, self._record_tick)

    def _update_record_status(self):
        text = (f"{self.recorder.count:5d} Punkte  {self.recorder.duration:6.1f} s"
                f"  {self.recorder.length * 1000.0:8.1f} mm")
        if self.recorder.overflow:
            text += "  (Puffer voll)"
        self.record_var.set(text)

    def _export_record(self):
        if not self.recorder.count:
            self._log("Keine Bahnpunkte vorhanden", "warn")
            return
        path = filedialog.asksaveasfilename(
            title="Bahn exportieren",
            initialdir=backup_dir_for(self.urdf_path),
            initialfile=f"bahn_{datetime.now():%Y%m%d_%H%M%S}.csv",
            defaultextension=".csv",
            filetypes=[("CSV", "*.csv"), ("Alle Dateien", "*.*")])
        if not path:
            return
        try:
            written = self.recorder.write_csv(path)
        except OSError as exc:
            messagebox.showerror("Export", f"Schreiben fehlgeschlagen:\n{exc}")
            self._log(f"Export fehlgeschlagen: {exc}", "error")
            return
        self._log(f"{written} Bahnpunkte nach {path} geschrieben", "ok")
   

    # ----------------------------------------------------------------------
    # Kinematik
    # ----------------------------------------------------------------------

    def _tcp_pose(self):
        """Lage des einzigen Endeffektors oder None."""
        poses = forward_kinematics(self.links, self.joints, self.positions)
        roots = root_links(self.links, self.joints)
        base = roots[0] if roots else "?"
        tcps = [n for n in leaf_links(self.links, self.joints) if n != base]
        if len(tcps) != 1 or tcps[0] not in poses:
            return None
        return to_xyz_rpy(poses[tcps[0]])

    def _pose_text(self, pad=True):
        """TCP-Lage als Text. Mit pad stehen die Spalten im Protokoll untereinander."""
        pose = self._tcp_pose()
        if pose is None:
            return "Achswerte uebernommen"
        xyz, rpy = pose
        width, digits = (8, 2) if pad else (0, 1)
        return (f"X {xyz[0]:{width}.4f} Y {xyz[1]:{width}.4f} Z {xyz[2]:{width}.4f} m   "
                f"R {math.degrees(rpy[0]):{width}.{digits}f} "
                f"P {math.degrees(rpy[1]):{width}.{digits}f} "
                f"Y {math.degrees(rpy[2]):{width}.{digits}f} deg")

    def _frame_parents(self):
        """{linkname: elternname}. Leer, wenn der Link keinen Vater hat."""
        parents = {l["name"]: "" for l in self.links}
        for joint in self.joints:
            if joint["child"] in parents and joint["parent"]:
                parents[joint["child"]] = joint["parent"]
        return parents

    def _sync_frame_tree(self):
        """Baumgeruest nur neu bauen, wenn sich die Topologie geaendert hat.

        Beim Ziehen am Schieber laeuft das hier bis zu 16 Mal in der Sekunde.
        Wuerde jedes Mal geloescht und neu eingefuegt, klappten alle Zweige
        wieder auf und die Auswahl ginge verloren.
        """
        parents = self._frame_parents()
        if parents == self.frame_layout:
            return

        self.frame_table.delete(*self.frame_table.get_children())
        self.frame_items = {}
        depth = {}

        # Eltern muessen vor ihren Kindern eingefuegt sein.
        pending = [l["name"] for l in self.links]
        while pending:
            progressed = False
            for name in list(pending):
                parent = parents.get(name, "")
                if parent and parent in pending:
                    continue                           # Vater kommt noch
                target = self.frame_items.get(parent, "") if parent else ""
                depth[name] = depth.get(parent, -1) + 1 if target else 0
                self.frame_items[name] = self.frame_table.insert(
                    target, "end", text=name, open=True)
                pending.remove(name)
                progressed = True
            if not progressed:                         # Zyklus, Rest flach anhaengen
                for name in pending:
                    depth[name] = 0
                    self.frame_items[name] = self.frame_table.insert(
                        "", "end", text=name, open=True)
                break

        self._fit_tree_column(depth)
        self.frame_layout = parents

    def _fit_tree_column(self, depth):
        """Baumspalte so breit, dass auch der tiefste Name lesbar bleibt.

        Eine lange serielle Kette rueckt weit ein; mit fester Breite waeren
        die letzten Glieder nur noch als Stummel zu sehen.
        """
        font = tkfont.nametofont("TkDefaultFont")
        needed = max((28 + level * self.tree_indent + font.measure(name)
                      for name, level in depth.items()), default=0) + 14
        self.frame_table.column(
            "#0", width=max(FRAME_TREE[1], min(needed, TREE_MAX_WIDTH)))

    def _toggle_frames(self):
        """Frames-Karte weg- oder wieder einblenden.

        Die Spalte behaelt einen schmalen Streifen zum Zurueckholen. Ihr
        Gewicht faellt auf null, damit der frei werdende Platz an die
        Gelenkliste geht und nicht als Luecke stehen bleibt.
        """
        self.frames_shown = not self.frames_shown
        if self.frames_shown:
            self.frames_strip.grid_remove()
            self.body.columnconfigure(1, weight=2, minsize=430)
            self.frames_card.grid()
            self.after_idle(self.frame_table.fit)
        else:
            self.frames_card.grid_remove()
            self.body.columnconfigure(1, weight=0, minsize=0)
            self.frames_strip.grid(row=0, column=1, sticky="ns", padx=(0, PAD))

    def _frames_open(self, opened):
        """Alle Zweige auf- oder zuklappen."""
        for item in self.frame_items.values():
            self.frame_table.item(item, open=opened)

    def _update_kinematics(self):
        """FK fuer alle Frames neu rechnen und die Tabellen fuellen."""
        poses = forward_kinematics(self.links, self.joints, self.positions)
        leaves = set(leaf_links(self.links, self.joints))
        roots = root_links(self.links, self.joints)
        base = roots[0] if roots else "?"

        self._sync_frame_tree()
        for link in self.links:
            name = link["name"]
            item = self.frame_items.get(name)
            if item is None:
                continue
            matrix = poses.get(name)
            if matrix is None:                         # nicht mit der Basis verbunden
                self.frame_table.item(item, values=["-"] * 6, tags=("orphan",))
                continue

            xyz, rpy = to_xyz_rpy(matrix)
            tags = ("tcp",) if name in leaves and name != base else ()
            self.frame_table.item(item, tags=tags, values=(
                f"{xyz[0]:.4f}", f"{xyz[1]:.4f}", f"{xyz[2]:.4f}",
                f"{math.degrees(rpy[0]):.2f}",
                f"{math.degrees(rpy[1]):.2f}",
                f"{math.degrees(rpy[2]):.2f}"))

        self._fill_joint_table()
        self.after_idle(self.frame_table.fit)

        tcps = [n for n in leaves if n != base]
        if len(tcps) == 1 and tcps[0] in poses:
            self.tcp_var.set(f"TCP {tcps[0]}   {self._pose_text(pad=False)}")
        elif tcps:
            self.tcp_var.set(f"Mehrere Endeffektoren: {', '.join(sorted(tcps))}")
        else:
            self.tcp_var.set("Kein Endeffektor gefunden")

    # ----------------------------------------------------------------------
    # Bearbeiten in einem eigenen Fenster
    # ----------------------------------------------------------------------

    @staticmethod
    def _show_window(window):
        """Fenster nach vorne holen, auch aus dem Minimierten heraus.

        lift() allein genuegt nicht: es ordnet nur die Stapelfolge und laesst
        ein eingeklapptes Fenster eingeklappt. Erst deiconify() macht es
        wieder sichtbar. Ohne diesen Aufruf war das Bearbeiten-Fenster nach
        dem Minimieren nicht mehr erreichbar, und weil dort das Speichern und
        der Baum liegen, wirkte die ganze Anwendung tot.
        """
        if window is None or not window.winfo_exists():
            return False
        window.deiconify()
        window.lift()
        try:
            window.focus_force()
        except tk.TclError:
            pass                                       # nicht jeder Fenstermanager erlaubt das
        return True

    def _hold_grab(self, window):
        """Eingabegriff nur halten, solange das Fenster sichtbar ist.

        Ein Fenster mit grab_set nimmt die gesamte Eingabe an sich. Wird es
        minimiert, nimmt es sie mit: die Anwendung reagiert auf nichts mehr,
        obwohl sie laeuft. Deshalb faellt der Griff beim Verschwinden weg und
        wird beim Auftauchen neu gesetzt.

        Die Pruefung auf event.widget ist noetig, weil ein Toplevel in den
        bindtags seiner Kinder steht und deren Ereignisse mitbekommt.
        """
        def hold(event=None):
            if event is not None and event.widget is not window:
                return
            if window.winfo_exists() and window.winfo_viewable():
                try:
                    window.grab_set()
                except tk.TclError:
                    pass
        def release(event=None):
            if event is not None and event.widget is not window:
                return
            if window.winfo_exists():
                try:
                    window.grab_release()
                except tk.TclError:
                    pass
        window.bind("<Map>", hold)
        window.bind("<Unmap>", release)
        hold()

    def _open_editor(self):
        """Aenderbar sind Laengen, Winkel, Gelenktyp, Achse und Anschlaege.

        parent/child bleiben gesperrt: sie legen die Topologie fest. Der Typ
        laesst sich umstellen; die Auswahl schaltet nur die Felder um,
        geschrieben wird erst mit Uebernehmen.
        """
        if self._show_window(self.editor):
            return

        window = tk.Toplevel(self)
        window.title("URDF bearbeiten")
        window.configure(background=BG)
        # Bewusst nicht transient(self): ein transientes Fenster bekommt bei
        # den meisten Fenstermanagern keinen eigenen Eintrag in der
        # Fensterleiste. Minimiert man es, gibt es keinen Weg zurueck. Als
        # eigenstaendiges Fenster laesst es sich normal ein- und ausklappen.
        window.resizable(True, True)
        # Auf die Bildschirmgroesse begrenzt: unter WSL ist die Flaeche oft
        # kleiner als angenommen, und ein Fenster, dessen Titelleiste
        # ausserhalb liegt, laesst sich nicht mehr verschieben.
        width = min(1360, window.winfo_screenwidth() - 80)
        height = min(800, window.winfo_screenheight() - 80)
        window.geometry(f"{width}x{height}+40+40")
        window.minsize(min(900, width), min(620, height))
        window.protocol("WM_DELETE_WINDOW", self._close_editor)
        self.editor = window

        window.rowconfigure(0, weight=1)
        window.columnconfigure(0, weight=1)            # nur der Baum waechst

        # Links die Struktur zum Anfassen, rechts die Zahlen zum Gelenk.
        # Die Auswahl ist dieselbe: was im Baum markiert ist, steht rechts.
        tree_card = self._card(window, row=0, column=0, sticky="nsew",
                               padx=(PAD, 0), pady=PAD)
        tree_card.rowconfigure(0, weight=1)
        tree_card.columnconfigure(0, weight=1)
        self._build_tree_panel(tree_card).grid(row=0, column=0, sticky="nsew")

        # Rechts zwei Karteikarten statt einer Karte. Untereinander gestapelt
        # braeuchte der Inhalt rund 750 Bildpunkte Hoehe; auf einem Schirm mit
        # 768 Zeilen bliebe nichts uebrig, und eine Karte ohne Bildlauf
        # schneidet still ab. Nebeneinander waere das Fenster ueber 1300
        # Punkte breit. Karteikarten kosten einen Klick und keinen Platz.
        pages = ttk.Notebook(window)
        pages.grid(row=0, column=1, sticky="ns", padx=PAD, pady=PAD)
        self.edit_pages = pages

        tab = ttk.Frame(pages, style="Card.TFrame", padding=(18, 16))
        pages.add(tab, text="Gelenk")
        self.edit_entries = {}

        head = ttk.Frame(tab, style="Card.TFrame")
        head.grid(row=0, column=0, columnspan=4, sticky="w", pady=(0, 16))
        ttk.Label(head, text="Gelenk", style="Card.TLabel").pack(side="left")
        self.edit_combo = ttk.Combobox(head, textvariable=self.edit_joint_var,
                                       state="readonly", width=24)
        self.edit_combo.pack(side="left", padx=(10, 16))
        self.edit_combo.bind("<<ComboboxSelected>>", lambda _e: self._edit_load_joint())

        ttk.Label(head, text="Typ", style="Card.TLabel").pack(side="left")
        self.edit_type_combo = ttk.Combobox(head, textvariable=self.edit_type_var,
                                            state="readonly", width=12,
                                            values=list(JOINT_TYPES))
        self.edit_type_combo.pack(side="left", padx=(10, 16))
        # Nur die Felder umschalten. Der Baum bleibt bis Uebernehmen unberuehrt.
        self.edit_type_combo.bind("<<ComboboxSelected>>",
                                  lambda _e: self._edit_type_changed())
        ttk.Label(head, textvariable=self.edit_info_var, style="Muted.TLabel").pack(
            side="left")

        def field_row(row, title, specs):
            ttk.Label(tab, text=title, style="Section.TLabel").grid(
                row=row, column=0, sticky="w", pady=(8, 4))
            holder = ttk.Frame(tab, style="Card.TFrame")
            holder.grid(row=row, column=1, columnspan=3, sticky="w", pady=(8, 4))
            for key, label in specs:
                ttk.Label(holder, text=label, style="Muted.TLabel").pack(
                    side="left", padx=(0, 6))
                entry = self._entry(holder, width=11)
                entry.pack(side="left", padx=(0, 16))
                self.edit_entries[key] = entry

        # Namenszeile direkt unter der Auswahl: der Name gehoert zum Gelenk
        # wie seine Laenge, nur wirkt er nicht auf die Geometrie sondern auf
        # alles, was ihn nennt. Deshalb eine eigene Taste und nicht
        # "Uebernehmen" -- ein Vertipper im Namen soll nicht zusammen mit
        # einer Laengenaenderung durchrutschen.
        name_row = ttk.Frame(tab, style="Card.TFrame")
        name_row.grid(row=1, column=0, columnspan=4, sticky="w", pady=(0, 4))
        ttk.Label(name_row, text="NAME", style="Section.TLabel").pack(side="left")
        self.edit_name_entry = self._entry(name_row, width=22)
        self.edit_name_entry.configure(justify="left")
        self.edit_name_entry.pack(side="left", padx=(18, 8))
        self.edit_rename_button = ttk.Button(name_row, text="Umbenennen",
                                             command=self._rename_joint)
        self.edit_rename_button.pack(side="left")

        field_row(2, "LAENGE  [m]", [("x", "X"), ("y", "Y"), ("z", "Z")])
        field_row(3, "WINKEL  [deg]", [("roll", "Roll"), ("pitch", "Pitch"),
                                       ("yaw", "Yaw")])
        field_row(4, "ACHSE", [("ax", "X"), ("ay", "Y"), ("az", "Z")])
        field_row(5, "AUSLENKUNG", [("lower", "min"), ("upper", "max")])
        ttk.Label(tab, textvariable=self.edit_unit_var, style="Muted.TLabel").grid(
            row=5, column=1, columnspan=3, sticky="e", padx=(0, 20))

        # Mechanische Kopplung. Steht als <mimic> wirklich in der URDF, wird
        # also mitgespeichert und gilt auch fuer robot_state_publisher und
        # MoveIt -- nicht nur fuer diese Oberflaeche.
        ttk.Label(tab, text="KOPPLUNG", style="Section.TLabel").grid(
            row=6, column=0, sticky="w", pady=(10, 4))
        coupling = ttk.Frame(tab, style="Card.TFrame")
        coupling.grid(row=6, column=1, columnspan=3, sticky="w", pady=(10, 4))
        ttk.Label(coupling, text="folgt", style="Muted.TLabel").pack(
            side="left", padx=(0, 6))
        self.edit_mimic_combo = ttk.Combobox(
            coupling, textvariable=self.edit_mimic_var, state="readonly",
            width=14)
        self.edit_mimic_combo.pack(side="left", padx=(0, 16))
        ttk.Label(coupling, text="Faktor", style="Muted.TLabel").pack(
            side="left", padx=(0, 6))
        self.edit_entries["factor"] = self._entry(coupling, width=10,
                                                  value="1.0000")
        self.edit_entries["factor"].pack(side="left", padx=(0, 16))
        ttk.Label(coupling, text="Versatz", style="Muted.TLabel").pack(
            side="left", padx=(0, 6))
        self.edit_entries["mimic_offset"] = self._entry(coupling, width=10,
                                                        value="0.000")
        self.edit_entries["mimic_offset"].pack(side="left", padx=(0, 8))
        ttk.Label(coupling, textvariable=self.edit_mimic_unit_var,
                  style="Muted.TLabel", width=4).pack(side="left")
        ttk.Button(coupling, text="Kopplung setzen",
                   command=self._apply_coupling).pack(side="left", padx=(10, 0))

        actions = ttk.Frame(tab, style="Card.TFrame")
        actions.grid(row=7, column=0, columnspan=4, sticky="w", pady=(20, 0))
        ttk.Button(actions, text="Uebernehmen", command=self._edit_apply).pack(
            side="left")
        ttk.Button(actions, text="Feld zuruecksetzen",
                   command=self._edit_load_joint).pack(side="left", padx=(8, 0))

        ttk.Separator(tab, orient="horizontal").grid(
            row=8, column=0, columnspan=4, sticky="ew", pady=18)

        ttk.Label(tab, text="STRUKTUR", style="Section.TLabel").grid(
            row=9, column=0, sticky="w")
        # Dieselben Befehle wie die Schaltflaechen im Baum, nur ueber die
        # Auswahlliste statt ueber den Zeiger. Wer lieber tippt als klickt,
        # kommt so an dieselben Aenderungen.
        structure = ttk.Frame(tab, style="Card.TFrame")
        structure.grid(row=10, column=0, columnspan=4, sticky="w", pady=(6, 0))
        ttk.Button(structure, text="Glied anhaengen...",
                   command=self._open_add_window).pack(side="left")
        ttk.Button(structure, text="Glied einfuegen...",
                   command=lambda: self._open_insert_window(
                       self.edit_joint_var.get())).pack(side="left", padx=(8, 0))
        ttk.Button(structure, text="Glied herausnehmen...",
                   command=lambda: self._open_remove_window(
                       self.edit_joint_var.get())).pack(side="left", padx=(8, 0))
        # Wirkt auf den Kindlink des gewaehlten Gelenks. Fuer den Wurzel-Link,
        # zu dem kein Gelenk fuehrt, geht es ueber den Baum.
        ttk.Button(structure, text="Aussehen...",
                   command=self._open_visual_for_selection).pack(
            side="left", padx=(8, 0))

        structure2 = ttk.Frame(tab, style="Card.TFrame")
        structure2.grid(row=11, column=0, columnspan=4, sticky="w", pady=(6, 0))
        ttk.Button(structure2, text="Gelenk und Teilbaum loeschen",
                   command=self._delete_branch).pack(side="left")
        self.undo_button = ttk.Button(structure2, text="Rueckgaengig",
                                      command=self._undo, state="disabled")
        self.undo_button.pack(side="left", padx=(8, 0))
        self._update_undo()

        ttk.Separator(tab, orient="horizontal").grid(
            row=12, column=0, columnspan=4, sticky="ew", pady=18)

        files = ttk.Frame(tab, style="Card.TFrame")
        files.grid(row=13, column=0, columnspan=4, sticky="w")
        ttk.Button(files, text="Speichern", command=self._save).pack(side="left")
        ttk.Button(files, text="Speichern unter...",
                   command=self._save_as).pack(side="left", padx=(8, 0))
        ttk.Button(files, text="Datei neu laden",
                   command=self._discard).pack(side="left", padx=(8, 0))
        ttk.Label(files, text=f"Ablage: {BACKUP_DIR}",
                  style="Muted.TLabel").pack(side="left", padx=(16, 0))

        pages.add(self._build_visual_tab(pages), text="Koerper")
        self._edit_fill_combo()

    def _close_editor(self):
        self._close_add_window()
        self._close_insert_window()
        self._close_remove_window()
        if self.editor is not None:
            self.editor.destroy()
        self.editor = None
        self.edit_entries = {}
        self.visual_entries = {}
        self.visual_swatch = None
        self.visual_half = None
        self.edit_pages = None
        self.undo_button = None
        # Die Zeichenflaeche lag im Fenster und ist mit ihm weg. Ohne das
        # Abmelden zeichnete _draw_tree spaeter auf ein totes Widget.
        self.tree_canvas = None
        self.tree_boxes = {}
        self.tree_zones = {}
        self.tree_pills = []
        self.tree_hover = None

    def _edit_fill_combo(self):
        names = [j["name"] for j in self.joints]
        self.edit_combo.configure(values=names)
        if names:
            if self.edit_joint_var.get() not in names:
                self.edit_joint_var.set(names[0])
            self._edit_load_joint()

        # Die Koerperseite haengt an einem Link, nicht an einem Gelenk. Sie
        # bekommt deshalb eine eigene Liste, in der auch der Wurzel-Link
        # steht -- zu ihm fuehrt kein Gelenk, ueber die Gelenkliste waere er
        # nie erreichbar.
        links = [l["name"] for l in self.links]
        if self.visual_combo is not None and self.visual_combo.winfo_exists():
            self.visual_combo.configure(values=links)
            if links and self.visual_link_var.get() not in links:
                self.visual_link_var.set(links[0])
            self._visual_load_link()

    def _edit_current_joint(self):
        name = self.edit_joint_var.get()
        for joint in self.joints:
            if joint["name"] == name:
                return joint
        return None

    @staticmethod
    def _set_entry(entries, key, value, enabled=True):
        entry = entries.get(key)
        if entry is None:
            return
        entry.configure(state="normal", background=BG_OK)
        entry.delete(0, "end")
        entry.insert(0, value)
        if not enabled:
            entry.configure(state="disabled")

    def _edit_set(self, key, value, enabled=True):
        self._set_entry(self.edit_entries, key, value, enabled)

    def _type_fields(self, entries, unit_var, jtype, source=None):
        """Achs- und Anschlagfelder auf einen Gelenktyp einstellen.

        Wird vom Bearbeiten-Fenster und vom Anhaengen-Fenster benutzt, damit
        beide sich gleich verhalten. `source` ist das vorhandene Gelenk oder
        None fuer ein neues.
        """
        rotational = jtype in ("revolute", "continuous")

        if jtype in NEEDS_AXIS:
            if source is not None and source["type"] in NEEDS_AXIS:
                axis = source["axis"]
            else:
                axis = (0.0, 0.0, 1.0)
            for key, value in zip(("ax", "ay", "az"), axis):
                self._set_entry(entries, key, f"{value:.5g}")
        else:
            for key in ("ax", "ay", "az"):
                self._set_entry(entries, key, "-", enabled=False)

        if jtype in NEEDS_LIMIT:
            factor = math.degrees if rotational else (lambda v: v)
            # Alte Grenzen nur uebernehmen, wenn die Einheit dieselbe bleibt.
            # Sonst wuerden aus 2.09 rad stillschweigend 2.09 m Hub.
            if (source is not None and source["has_limit"]
                    and is_rotational(source) == rotational):
                lower, upper = factor(source["lower"]), factor(source["upper"])
            elif rotational:
                lower, upper = -180.0, 180.0
            else:
                lower, upper = -0.5, 0.5
            self._set_entry(entries, "lower", f"{lower:.3f}")
            self._set_entry(entries, "upper", f"{upper:.3f}")
            unit_var.set("deg" if rotational else "m")
        else:
            self._set_entry(entries, "lower", "-", enabled=False)
            self._set_entry(entries, "upper", "-", enabled=False)
            unit_var.set("continuous: kein Anschlag" if rotational
                         else "fest: keine Auslenkung")

    @staticmethod
    def _read_entry(entries, key):
        """Feldwert als float. Faerbt bei Fehler und wirft ValueError."""
        entry = entries[key]
        try:
            value = float(entry.get().replace(",", "."))
        except ValueError:
            entry.configure(background=BG_INVALID)
            raise
        entry.configure(background=BG_OK)
        return value

    def _edit_load_joint(self):
        """Felder aus dem aktuellen Stand des Modells fuellen."""
        if not self.edit_entries:
            return
        joint = self._edit_current_joint()
        if joint is None:
            return

        for axis_key, index in (("x", 0), ("y", 1), ("z", 2)):
            self._edit_set(axis_key, f"{joint['xyz'][index]:.5f}")
        for angle_key, index in (("roll", 0), ("pitch", 1), ("yaw", 2)):
            self._edit_set(angle_key, f"{math.degrees(joint['rpy'][index]):.3f}")

        self.edit_type_var.set(joint["type"])
        self.edit_info_var.set(f"{joint['parent']} -> {joint['child']}")
        self._edit_type_changed(joint)
        if self.edit_name_entry is not None and self.edit_name_entry.winfo_exists():
            self._set_entry({"n": self.edit_name_entry}, "n", joint["name"])
        self._edit_load_coupling(joint)

        # Liste und Zeichnung zeigen immer dasselbe Gelenk. Der Abgleich steht
        # hier, weil beide Wege hier vorbeikommen: Auswahl in der Liste und
        # Klick in den Baum.
        if self.tree_selected != joint["name"]:
            self.tree_selected = joint["name"]
        self._draw_tree()

    def _edit_type_changed(self, joint=None):
        """Achse und Anschlaege auf den gewaehlten Typ umschalten.

        Aendert nichts am Modell. Beim Wechsel auf einen Typ, der etwas
        braucht, was das Gelenk bisher nicht hat, wird ein Vorschlag
        eingetragen, den der Bediener vor dem Uebernehmen anpassen kann.
        """
        if not self.edit_entries:
            return
        if joint is None:
            joint = self._edit_current_joint()
        if joint is None:
            return
        jtype = self.edit_type_var.get() or joint["type"]
        self._type_fields(self.edit_entries, self.edit_unit_var, jtype, joint)

    def _edit_read(self, key):
        return self._read_entry(self.edit_entries, key)

    def _edit_apply(self):
        """Werte in den XML-Baum schreiben, pruefen und live uebernehmen."""
        joint = self._edit_current_joint()
        if joint is None or self.xml_root is None:
            return

        jtype = self.edit_type_var.get() or joint["type"]
        rotational = jtype in ("revolute", "continuous")

        try:
            xyz = [self._edit_read(k) for k in ("x", "y", "z")]
            rpy = [math.radians(self._edit_read(k)) for k in ("roll", "pitch", "yaw")]
            axis = ([self._edit_read(k) for k in ("ax", "ay", "az")]
                    if jtype in NEEDS_AXIS else None)
            if jtype in NEEDS_LIMIT:
                factor = math.radians if rotational else (lambda v: v)
                lower = factor(self._edit_read("lower"))
                upper = factor(self._edit_read("upper"))
            else:
                lower = upper = None
        except ValueError:
            self._log("Eingabe nicht lesbar, nichts geaendert", "error")
            return

        def mutate(draft):
            element = next((j for j in draft.findall("joint")
                            if j.get("name") == joint["name"]), None)
            if element is None:
                raise ValueError(f"Gelenk {joint['name']} nicht mehr im Baum")

            element.set("type", jtype)
            origin = element.find("origin")
            if origin is None:
                origin = ET.SubElement(element, "origin")
            origin.set("xyz", fmt_origin(xyz))
            origin.set("rpy", fmt_origin(rpy))
            set_axis(element, axis)
            set_limit(element, lower, upper, keep=(jtype == "continuous"))

        if jtype != joint["type"]:
            text = f"{joint['name']}: Typ {joint['type']} -> {jtype}"
        else:
            text = f"{joint['name']} geaendert"
        self._commit(text, mutate)

    # ----------------------------------------------------------------------
    # Baumansicht
    # ----------------------------------------------------------------------

    def _build_tree_panel(self, parent):
        """Zeichenflaeche fuer die Topologie. Gibt den Rahmen zurueck.

        Eine schlichte tk.Canvas, kein OpenGL und keine Fremdbibliothek. Der
        Baum hat selten mehr als ein paar Dutzend Elemente, da ist Neuzeichnen
        billiger als jede Buchfuehrung ueber Aenderungen.

        Die Flaeche liegt im Bearbeiten-Fenster, weil sie dort hingehoert: sie
        zeigt nicht nur die Struktur, sie ist der Weg, sie zu aendern. Im
        Hauptfenster waere sie eine Anzeige ohne Wirkung gewesen.
        """
        frame = tk.Frame(parent, background=SURFACE, bd=0, highlightthickness=0)
        frame.rowconfigure(1, weight=1)
        frame.columnconfigure(0, weight=1)

        head = ttk.Frame(frame, style="Card.TFrame", padding=(12, 8, 10, 6))
        head.grid(row=0, column=0, columnspan=2, sticky="ew")
        ttk.Label(head, text="STRUKTUR", style="Section.TLabel").pack(side="left")
        ttk.Label(head, textvariable=self.tree_hint_var,
                  style="Muted.TLabel").pack(side="left", padx=(12, 0))

        canvas = tk.Canvas(frame, background=SURFACE, bd=0, highlightthickness=0)
        canvas.grid(row=1, column=0, sticky="nsew")
        self.tree_canvas = canvas

        yscroll = ttk.Scrollbar(frame, orient="vertical", command=canvas.yview)
        xscroll = ttk.Scrollbar(frame, orient="horizontal", command=canvas.xview)
        canvas.configure(yscrollcommand=yscroll.set, xscrollcommand=xscroll.set)
        yscroll.grid(row=1, column=1, sticky="ns")
        xscroll.grid(row=2, column=0, sticky="ew")

        canvas.bind("<Button-1>", self._tree_click)
        canvas.bind("<Motion>", self._tree_motion)
        canvas.bind("<Leave>", self._tree_leave)
        # Rad unter X11. Windows meldet <MouseWheel>, Linux zwei Tasten.
        canvas.bind("<Button-4>", lambda _e: canvas.yview_scroll(-2, "units"))
        canvas.bind("<Button-5>", lambda _e: canvas.yview_scroll(2, "units"))
        return frame

    def _tree_point(self, event):
        """Zeigerort in Zeichenkoordinaten. Beruecksichtigt den Bildlauf."""
        canvas = self.tree_canvas
        return canvas.canvasx(event.x), canvas.canvasy(event.y)

    @staticmethod
    def _inside(box, x, y):
        return box is not None and box[0] <= x <= box[2] and box[1] <= y <= box[3]

    @staticmethod
    def _tree_label(name, limit=17):
        """Langer Name gekuerzt, damit er im Kasten bleibt."""
        return name if len(name) <= limit else name[:limit - 3] + "..."

    def _draw_tree(self):
        """Baum neu zeichnen. Einziger Weg, auf dem die Flaeche Inhalt bekommt.

        Erst die Kanten, dann die Kaesten: so verschwinden die Linienenden
        unter den Kaesten und muessen nicht auf den Rand gerechnet werden.
        """
        canvas = self.tree_canvas
        if canvas is None or not canvas.winfo_exists():
            return

        canvas.delete("all")
        self.tree_boxes = {}
        self.tree_zones = {}
        self.tree_pill = None
        self.tree_action = None

        nodes = tree_layout(self.links, self.joints)
        if not nodes:
            canvas.create_text(TREE_MARGIN, TREE_MARGIN, anchor="nw",
                               fill=WARN, font=("TkDefaultFont", 10),
                               text="Kein Wurzel-Link, deshalb kein Baum.")
            canvas.configure(scrollregion=(0, 0, 1, 1))
            self.tree_hint_var.set("")
            return

        # Der gewaehlte Teilbaum wird getoent. Das ist zugleich die Vorschau
        # aufs Loeschen: markiert ist genau das, was dabei verschwaende.
        selected = self._joint_by_name(self.tree_selected) if self.tree_selected else None
        marked = set(subtree_links(self.joints, selected["child"])) if selected else set()
        roots = set(root_links(self.links, self.joints))

        for joint in self.joints:
            above = nodes.get(joint["parent"])
            below = nodes.get(joint["child"])
            if above is None or below is None:
                continue                               # abgehaengt, nicht gezeichnet
            px, py = above
            cx, cy = below
            top = py + TREE_NODE_H
            mid = top + TREE_GAP_Y / 2.0

            chosen = joint["name"] == self.tree_selected
            colour = ACCENT if chosen else MUTED
            thick = 3 if chosen else 1
            # Gestrichelt heisst fest. Der Typ steht damit in der Linie und
            # braucht keine zweite Beschriftung.
            style = {"dash": (4, 3)} if joint["type"] == "fixed" else {}

            canvas.create_line(px, top, px, mid, fill=colour, width=thick, **style)
            if abs(cx - px) > 0.5:                     # Knick nur bei Verzweigung
                canvas.create_line(px, mid, cx, mid, fill=colour, width=thick, **style)
            canvas.create_line(cx, mid, cx, cy, fill=colour, width=thick, **style)

            label = canvas.create_text(
                cx + TREE_GRAB, (mid + cy) / 2.0, anchor="w",
                text=joint["name"], fill=ACCENT if chosen else TEXT,
                font=("TkDefaultFont", 9, "bold" if chosen else "normal"))
            lx0, ly0, lx1, ly1 = canvas.bbox(label)
            # Trefferzone: das senkrechte Stueck ueber dem Kindkasten samt
            # Beschriftung. Der waagerechte Teil des Knicks gehoert bewusst
            # nicht dazu, er liegt bei Verzweigungen quer ueber der Flaeche.
            self.tree_zones[joint["name"]] = (
                min(cx - TREE_GRAB, lx0), min(mid, ly0),
                max(cx + TREE_GRAB, lx1), max(cy, ly1))

        for link in self.links:
            position = nodes.get(link["name"])
            if position is None:
                continue
            cx, top = position
            box = (cx - TREE_NODE_W / 2.0, top,
                   cx + TREE_NODE_W / 2.0, top + TREE_NODE_H)
            if link["name"] in roots:
                fill, edge = TINT_OK, OK_GREEN
            elif link["name"] in marked:
                fill, edge = SELECT, ACCENT
            else:
                fill, edge = SURFACE, BORDER
            canvas.create_rectangle(*box, fill=fill, outline=edge, width=1)
            canvas.create_text((box[0] + box[2]) / 2.0, (box[1] + box[3]) / 2.0,
                               text=self._tree_label(link["name"]), fill=TEXT,
                               font=("TkDefaultFont", 9))
            # Kleines Farbfeld links im Kasten. Ein Link ohne <visual> bleibt
            # in RViz unsichtbar; hier sieht man das, ohne umschalten zu
            # muessen.
            visual = link["visual"]
            swatch = (box[0] + 5, box[1] + 7, box[0] + 15, box[1] + 17)
            if visual is not None and visual["rgba"]:
                canvas.create_rectangle(*swatch, fill=rgba_to_hex(visual["rgba"]),
                                        outline=BORDER, width=1)
            else:
                canvas.create_rectangle(*swatch, fill="", outline=BORDER, width=1)
                canvas.create_line(swatch[0], swatch[3], swatch[2], swatch[1],
                                   fill=MUTED, width=1)
            self.tree_boxes[link["name"]] = box

        area = canvas.bbox("all")
        canvas.configure(scrollregion=(0, 0, area[2] + TREE_MARGIN,
                                       area[3] + TREE_MARGIN))

        hidden = len(self.links) - len(nodes)
        hint = "Kasten: anhaengen   |   Kante: einfuegen"
        if hidden > 0:
            hint += f"   |   {hidden} Link(s) haengen an keiner Wurzel"
        self.tree_hint_var.set(hint)

    def _tree_hit(self, x, y):
        """Was liegt unter dem Zeiger? ("node"/"edge", Name) oder None.

        Von Hand statt ueber find_closest der Canvas: dort haengt das Ergebnis
        an der Zeichenreihenfolge und liefert auch auf leerer Flaeche immer
        irgendein Element. Hier gilt schlicht, ob der Punkt im Rechteck liegt.
        """
        for name, box in self.tree_boxes.items():
            if self._inside(box, x, y):
                return "node", name
        for name, zone in self.tree_zones.items():
            if self._inside(zone, x, y):
                return "edge", name
        return None

    def _tree_draw_pill(self):
        """Schaltflaechen neben dem Element unter dem Zeiger.

        Am Kasten haengt eine, an der Kante zwei: einfuegen und herausnehmen
        sind zueinander die Umkehrung und gehoeren deshalb nebeneinander.
        """
        canvas = self.tree_canvas
        if canvas is None or not canvas.winfo_exists():
            return
        canvas.delete("pill")
        self.tree_pills = []
        if self.tree_hover is None:
            return

        kind, name = self.tree_hover
        if kind == "node":
            anchor = self.tree_boxes.get(name)
            offers = [("Anhaengen", lambda: self._open_add_window(parent=name)),
                      ("Aussehen", lambda: self._focus_visual(name))]
        else:
            anchor = self.tree_zones.get(name)
            offers = [("Einfuegen", lambda: self._open_insert_window(name)),
                      ("Herausnehmen", lambda: self._open_remove_window(name))]
        if anchor is None:                             # Baum wurde zwischendurch neu gebaut
            self.tree_hover = None
            return

        left = anchor[2] + 10
        top = (anchor[1] + anchor[3]) / 2.0 - 10
        for text, action in offers:
            box = (left, top, left + 8 * len(text) + 18, top + 20)
            canvas.create_rectangle(*box, fill=SELECT, outline=ACCENT, width=1,
                                    tags="pill")
            canvas.create_text((box[0] + box[2]) / 2.0, (box[1] + box[3]) / 2.0,
                               text=text, fill=ACCENT, tags="pill",
                               font=("TkDefaultFont", 9, "bold"))
            self.tree_pills.append((box, action))
            left = box[2] + 6

    def _tree_pill_at(self, x, y):
        """Befehl der Schaltflaeche unter dem Punkt oder None."""
        for box, action in self.tree_pills:
            if self._inside(box, x, y):
                return action
        return None

    def _tree_motion(self, event):
        x, y = self._tree_point(event)
        # Steht der Zeiger auf einer Schaltflaeche, bleiben sie stehen. Sonst
        # verschwaenden sie genau auf dem Weg dorthin.
        if self._tree_pill_at(x, y) is not None:
            return
        hit = self._tree_hit(x, y)
        if hit == self.tree_hover:
            return
        self.tree_hover = hit
        self._tree_draw_pill()

    def _tree_leave(self, _event=None):
        if self.tree_hover is not None:
            self.tree_hover = None
            self._tree_draw_pill()

    def _tree_click(self, event):
        x, y = self._tree_point(event)
        action = self._tree_pill_at(x, y)
        if action is not None:
            self.tree_hover = None
            self._tree_draw_pill()
            action()
            return

        hit = self._tree_hit(x, y)
        if hit is None:
            return
        kind, name = hit
        if kind == "node":
            # Ein Kasten hat keinen eigenen Eintrag zum Bearbeiten. Gemeint
            # ist das Gelenk, das zu ihm fuehrt; der Wurzel-Link hat keines.
            joint = next((j for j in self.joints if j["child"] == name), None)
            name = joint["name"] if joint is not None else None
        self._tree_select(name)

    def _tree_select(self, name):
        """Auswahl setzen und mit der Liste im Bearbeiten-Fenster abgleichen."""
        if name and self.editor is not None and self.editor.winfo_exists():
            self.edit_joint_var.set(name)
            self._edit_load_joint()                    # zeichnet den Baum mit
        else:
            self.tree_selected = name
            self._draw_tree()
        self._tree_draw_pill()

    # ----------------------------------------------------------------------
    # Struktur: anhaengen, einfuegen und loeschen
    # ----------------------------------------------------------------------

    def _suggest_name(self, stem, taken):
        """stem1, stem2, ... bis einer frei ist."""
        index = len(taken) + 1
        while f"{stem}{index}" in taken:
            index += 1
        return f"{stem}{index}"

    def _open_add_window(self, parent=None):
        """Neues Glied anlegen: Link und zufuehrendes Gelenk in einem Schritt.

        Getrennt ginge es nicht ohne Zwischenschritt mit ungueltigem Modell:
        ein Link ohne Gelenk waere ein zweiter Wurzel-Link.

        `parent` kommt aus der Baumansicht: dort ist der Anschlusspunkt schon
        angeklickt und muss nicht noch einmal aus einer Liste gesucht werden.
        """
        known = {l["name"] for l in self.links}
        if self.add_window is not None and self.add_window.winfo_exists():
            if parent in known:
                self.add_parent_var.set(parent)
            self._show_window(self.add_window)
            return
        if not self.links:
            self._log("Kein Modell geladen", "warn")
            return

        window = tk.Toplevel(self.editor or self)
        window.title("Glied anhaengen")
        window.configure(background=BG)
        window.transient(self.editor or self)
        window.resizable(False, False)
        window.protocol("WM_DELETE_WINDOW", self._close_add_window)
        self._hold_grab(window)                        # kein zweites Fenster
        self.add_window = window

        card = self._card(window)
        card.pack(fill="both", expand=True, padx=PAD, pady=PAD)
        tab = ttk.Frame(card, style="Card.TFrame", padding=(18, 16))
        tab.pack(fill="both", expand=True)
        self.add_entries = {}

        link_names = [l["name"] for l in self.links]
        joint_names = {j["name"] for j in self.joints}

        head = ttk.Frame(tab, style="Card.TFrame")
        head.grid(row=0, column=0, columnspan=4, sticky="w", pady=(0, 14))
        ttk.Label(head, text="Neuer Link", style="Card.TLabel").pack(side="left")
        entry = self._entry(head, width=16,
                            value=self._suggest_name("link", set(link_names)))
        entry.configure(justify="left")
        entry.pack(side="left", padx=(10, 18))
        self.add_entries["link"] = entry

        ttk.Label(head, text="haengt an", style="Card.TLabel").pack(side="left")
        parent_combo = ttk.Combobox(head, textvariable=self.add_parent_var,
                                    state="readonly", width=18,
                                    values=link_names)
        parent_combo.pack(side="left", padx=(10, 0))
        if parent in known:
            self.add_parent_var.set(parent)            # aus der Baumansicht
        else:
            # Vorschlag: der aktuelle Endeffektor, dort wird meist angebaut.
            leaves = [n for n in leaf_links(self.links, self.joints)]
            self.add_parent_var.set(leaves[-1] if leaves else link_names[0])

        second = ttk.Frame(tab, style="Card.TFrame")
        second.grid(row=1, column=0, columnspan=4, sticky="w", pady=(0, 6))
        ttk.Label(second, text="Gelenkname", style="Card.TLabel").pack(side="left")
        entry = self._entry(second, width=16,
                            value=self._suggest_name("joint", joint_names))
        entry.configure(justify="left")
        entry.pack(side="left", padx=(10, 18))
        self.add_entries["joint"] = entry

        ttk.Label(second, text="Typ", style="Card.TLabel").pack(side="left")
        type_combo = ttk.Combobox(second, textvariable=self.add_type_var,
                                  state="readonly", width=12,
                                  values=list(JOINT_TYPES))
        type_combo.pack(side="left", padx=(10, 0))
        type_combo.bind("<<ComboboxSelected>>", lambda _e: self._add_type_changed())

        def field_row(row, title, specs, defaults):
            ttk.Label(tab, text=title, style="Section.TLabel").grid(
                row=row, column=0, sticky="w", pady=(8, 4))
            holder = ttk.Frame(tab, style="Card.TFrame")
            holder.grid(row=row, column=1, columnspan=3, sticky="w", pady=(8, 4))
            for (key, label), default in zip(specs, defaults):
                ttk.Label(holder, text=label, style="Muted.TLabel").pack(
                    side="left", padx=(0, 6))
                field = self._entry(holder, width=11, value=default)
                field.pack(side="left", padx=(0, 16))
                self.add_entries[key] = field

        field_row(2, "LAENGE  [m]", [("x", "X"), ("y", "Y"), ("z", "Z")],
                  ["0.00000", "0.00000", "0.10000"])
        field_row(3, "WINKEL  [deg]", [("roll", "Roll"), ("pitch", "Pitch"),
                                       ("yaw", "Yaw")], ["0.000"] * 3)
        field_row(4, "ACHSE", [("ax", "X"), ("ay", "Y"), ("az", "Z")],
                  ["0", "0", "1"])
        field_row(5, "AUSLENKUNG", [("lower", "min"), ("upper", "max")],
                  ["-180.000", "180.000"])
        ttk.Label(tab, textvariable=self.add_unit_var, style="Muted.TLabel").grid(
            row=5, column=1, columnspan=3, sticky="e", padx=(0, 20))

        actions = ttk.Frame(tab, style="Card.TFrame")
        actions.grid(row=6, column=0, columnspan=4, sticky="w", pady=(20, 0))
        ttk.Button(actions, text="Anlegen", command=self._add_apply).pack(side="left")
        ttk.Button(actions, text="Abbrechen",
                   command=self._close_add_window).pack(side="left", padx=(8, 0))
        ttk.Label(tab, text="Der neue Link bekommt keine Masse und keine "
                            "Geometrie. Er ist ein Koordinatensystem.",
                  style="Muted.TLabel").grid(row=7, column=0, columnspan=4,
                                             sticky="w", pady=(14, 0))

        self._add_type_changed()

    def _close_add_window(self):
        if self.add_window is not None and self.add_window.winfo_exists():
            self.add_window.grab_release()
            self.add_window.destroy()
        self.add_window = None
        self.add_entries = {}

    def _add_type_changed(self):
        self._type_fields(self.add_entries, self.add_unit_var,
                          self.add_type_var.get())

    def _add_apply(self):
        """Eingaben pruefen und das neue Glied anhaengen."""
        jtype = self.add_type_var.get()
        rotational = jtype in ("revolute", "continuous")
        link_name = self.add_entries["link"].get().strip()
        joint_name = self.add_entries["joint"].get().strip()
        parent = self.add_parent_var.get()

        # -- Namen zuerst, sie sind der haeufigste Fehler -------------------
        problems = [p for p in (
            name_problem(link_name, "Link", {l["name"] for l in self.links}),
            name_problem(joint_name, "Gelenk", {j["name"] for j in self.joints}),
        ) if p]
        if link_name and link_name == joint_name:
            problems.append("Link und Gelenk duerfen nicht gleich heissen")
        if parent not in {l["name"] for l in self.links}:
            problems.append("Kein gueltiger Anschlusspunkt gewaehlt")
        if problems:
            messagebox.showerror("Anhaengen", "\n".join(problems))
            for problem in problems:
                self._log(problem, "error")
            return

        try:
            xyz = [self._read_entry(self.add_entries, k) for k in ("x", "y", "z")]
            rpy = [math.radians(self._read_entry(self.add_entries, k))
                   for k in ("roll", "pitch", "yaw")]
            axis = ([self._read_entry(self.add_entries, k)
                     for k in ("ax", "ay", "az")] if jtype in NEEDS_AXIS else None)
            if jtype in NEEDS_LIMIT:
                factor = math.radians if rotational else (lambda v: v)
                lower = factor(self._read_entry(self.add_entries, "lower"))
                upper = factor(self._read_entry(self.add_entries, "upper"))
            else:
                lower = upper = None
        except ValueError:
            self._log("Eingabe nicht lesbar, nichts angelegt", "error")
            return

        def mutate(draft):
            append_limb(draft, link_name, joint_name, jtype, parent,
                        xyz, rpy, axis, lower, upper)

        if self._commit(f"{link_name} ueber {joint_name} an {parent} angehaengt",
                        mutate):
            self._close_add_window()

    def _open_insert_window(self, below):
        """Neues Glied zwischen zwei bestehende haengen.

        Der Unterschied zum Anhaengen steht in einem einzigen Feld: das
        angeklickte Gelenk wandert unter den neuen Link. Alles andere ist
        dieselbe Eingabe, deshalb sieht das Fenster auch gleich aus.
        """
        if self.add_window is not None and self.add_window.winfo_exists():
            self._show_window(self.add_window)         # erst das andere zuende
            return
        if self._show_window(self.insert_window):
            return
        joint = self._joint_by_name(below)
        if joint is None:
            return
        self.insert_below = below

        window = tk.Toplevel(self)
        window.title("Glied einfuegen")
        window.configure(background=BG)
        window.transient(self)
        window.resizable(False, False)
        window.protocol("WM_DELETE_WINDOW", self._close_insert_window)
        self._hold_grab(window)
        self.insert_window = window

        card = self._card(window)
        card.pack(fill="both", expand=True, padx=PAD, pady=PAD)
        tab = ttk.Frame(card, style="Card.TFrame", padding=(18, 16))
        tab.pack(fill="both", expand=True)
        self.insert_entries = {}

        link_names = {l["name"] for l in self.links}
        joint_names = {j["name"] for j in self.joints}
        new_link = self._suggest_name("link", link_names)

        def show_chain(_event=None):
            """Zeigt die Kette, wie sie nach dem Einfuegen aussieht."""
            entry = self.insert_entries.get("link")
            name = entry.get().strip() if entry is not None else new_link
            self.insert_info_var.set(
                f"{joint['parent']}  ->  {name or '?'}  ->  {joint['child']}"
                f"      bisher: {joint['parent']}  ->  {joint['child']}")
        show_chain()

        head = ttk.Frame(tab, style="Card.TFrame")
        head.grid(row=0, column=0, columnspan=4, sticky="w", pady=(0, 14))
        ttk.Label(head, text="Neuer Link", style="Card.TLabel").pack(side="left")
        entry = self._entry(head, width=16, value=new_link)
        entry.configure(justify="left")
        entry.pack(side="left", padx=(10, 18))
        self.insert_entries["link"] = entry
        entry.bind("<KeyRelease>", show_chain)
        ttk.Label(head, text=f"vor {below}", style="Card.TLabel").pack(side="left")

        second = ttk.Frame(tab, style="Card.TFrame")
        second.grid(row=1, column=0, columnspan=4, sticky="w", pady=(0, 6))
        ttk.Label(second, text="Gelenkname", style="Card.TLabel").pack(side="left")
        entry = self._entry(second, width=16,
                            value=self._suggest_name("joint", joint_names))
        entry.configure(justify="left")
        entry.pack(side="left", padx=(10, 18))
        self.insert_entries["joint"] = entry

        ttk.Label(second, text="Typ", style="Card.TLabel").pack(side="left")
        type_combo = ttk.Combobox(second, textvariable=self.insert_type_var,
                                  state="readonly", width=12,
                                  values=list(JOINT_TYPES))
        type_combo.pack(side="left", padx=(10, 0))
        type_combo.bind("<<ComboboxSelected>>",
                        lambda _e: self._insert_type_changed())

        def field_row(row, title, specs, defaults):
            ttk.Label(tab, text=title, style="Section.TLabel").grid(
                row=row, column=0, sticky="w", pady=(8, 4))
            holder = ttk.Frame(tab, style="Card.TFrame")
            holder.grid(row=row, column=1, columnspan=3, sticky="w", pady=(8, 4))
            for (key, label), default in zip(specs, defaults):
                ttk.Label(holder, text=label, style="Muted.TLabel").pack(
                    side="left", padx=(0, 6))
                field = self._entry(holder, width=11, value=default)
                field.pack(side="left", padx=(0, 16))
                self.insert_entries[key] = field

        # Null als Vorgabe, anders als beim Anhaengen: der haeufigste Fall ist
        # ein Frame an derselben Stelle, und der aendert dann gar nichts.
        field_row(2, "LAENGE  [m]", [("x", "X"), ("y", "Y"), ("z", "Z")],
                  ["0.00000"] * 3)
        field_row(3, "WINKEL  [deg]", [("roll", "Roll"), ("pitch", "Pitch"),
                                       ("yaw", "Yaw")], ["0.000"] * 3)
        field_row(4, "ACHSE", [("ax", "X"), ("ay", "Y"), ("az", "Z")],
                  ["0", "0", "1"])
        field_row(5, "AUSLENKUNG", [("lower", "min"), ("upper", "max")],
                  ["-180.000", "180.000"])
        ttk.Label(tab, textvariable=self.insert_unit_var,
                  style="Muted.TLabel").grid(row=5, column=1, columnspan=3,
                                             sticky="e", padx=(0, 20))

        ttk.Separator(tab, orient="horizontal").grid(
            row=6, column=0, columnspan=4, sticky="ew", pady=14)
        ttk.Checkbutton(tab, text=f"Lage von {joint['child']} und allem dahinter "
                                  f"beibehalten",
                        variable=self.insert_keep_var).grid(
            row=7, column=0, columnspan=4, sticky="w")
        ttk.Label(tab, text=f"Aus: der Ursprung von {below} bleibt stehen und "
                            f"der Teilbaum wandert mit dem neuen Glied mit.",
                  style="Muted.TLabel").grid(row=8, column=0, columnspan=4,
                                             sticky="w", pady=(4, 0))
        ttk.Label(tab, textvariable=self.insert_info_var,
                  style="Muted.TLabel").grid(row=9, column=0, columnspan=4,
                                             sticky="w", pady=(10, 0))

        actions = ttk.Frame(tab, style="Card.TFrame")
        actions.grid(row=10, column=0, columnspan=4, sticky="w", pady=(18, 0))
        ttk.Button(actions, text="Einfuegen",
                   command=self._insert_apply).pack(side="left")
        ttk.Button(actions, text="Abbrechen",
                   command=self._close_insert_window).pack(side="left", padx=(8, 0))

        self._insert_type_changed()

    def _close_insert_window(self):
        if self.insert_window is not None and self.insert_window.winfo_exists():
            self.insert_window.grab_release()
            self.insert_window.destroy()
        self.insert_window = None
        self.insert_entries = {}
        self.insert_below = None

    def _insert_type_changed(self):
        self._type_fields(self.insert_entries, self.insert_unit_var,
                          self.insert_type_var.get())

    def _insert_apply(self):
        """Eingaben pruefen und das neue Glied in die Kette haengen."""
        below = self.insert_below
        if below is None or self._joint_by_name(below) is None:
            self._log(f"Gelenk {below} gibt es nicht mehr", "error")
            self._close_insert_window()
            return

        jtype = self.insert_type_var.get()
        rotational = jtype in ("revolute", "continuous")
        link_name = self.insert_entries["link"].get().strip()
        joint_name = self.insert_entries["joint"].get().strip()

        problems = [p for p in (
            name_problem(link_name, "Link", {l["name"] for l in self.links}),
            name_problem(joint_name, "Gelenk", {j["name"] for j in self.joints}),
        ) if p]
        if link_name and link_name == joint_name:
            problems.append("Link und Gelenk duerfen nicht gleich heissen")
        if problems:
            messagebox.showerror("Einfuegen", "\n".join(problems))
            for problem in problems:
                self._log(problem, "error")
            return

        try:
            xyz = [self._read_entry(self.insert_entries, k) for k in ("x", "y", "z")]
            rpy = [math.radians(self._read_entry(self.insert_entries, k))
                   for k in ("roll", "pitch", "yaw")]
            axis = ([self._read_entry(self.insert_entries, k)
                     for k in ("ax", "ay", "az")] if jtype in NEEDS_AXIS else None)
            if jtype in NEEDS_LIMIT:
                factor = math.radians if rotational else (lambda v: v)
                lower = factor(self._read_entry(self.insert_entries, "lower"))
                upper = factor(self._read_entry(self.insert_entries, "upper"))
            else:
                lower = upper = None
        except ValueError:
            self._log("Eingabe nicht lesbar, nichts eingefuegt", "error")
            return

        keep = self.insert_keep_var.get()

        def mutate(draft):
            insert_limb(draft, link_name, joint_name, jtype, below,
                        xyz, rpy, axis, lower, upper, keep)

        text = (f"{link_name} ueber {joint_name} vor {below} eingefuegt"
                f"{'' if keep else ', Teilbaum mitgewandert'}")
        if self._commit(text, mutate):
            self._tree_select(joint_name)
            self._close_insert_window()

    def _open_remove_window(self, joint_name):
        """Ein einzelnes Glied herausnehmen, ohne den Teilbaum zu verlieren.

        Bewusst mit Rueckfrage in einem eigenen Fenster statt als schlichte
        Messagebox: was verschwindet und was eine Stufe hochwandert, sind
        zwei verschiedene Dinge, und die Umrechnung ist eine echte Wahl.
        """
        for other in (self.add_window, self.insert_window):
            if other is not None and other.winfo_exists():
                self._show_window(other)
                return
        if self._show_window(self.remove_window):
            return
        joint = self._joint_by_name(joint_name)
        if joint is None:
            self._log("Kein Gelenk gewaehlt", "warn")
            return
        self.remove_joint = joint_name

        below = [j for j in self.joints if j["parent"] == joint["child"]]

        window = tk.Toplevel(self.editor or self)
        window.title("Glied herausnehmen")
        window.configure(background=BG)
        window.transient(self.editor or self)
        window.resizable(False, False)
        window.protocol("WM_DELETE_WINDOW", self._close_remove_window)
        self._hold_grab(window)
        self.remove_window = window

        card = self._card(window)
        card.pack(fill="both", expand=True, padx=PAD, pady=PAD)
        tab = ttk.Frame(card, style="Card.TFrame", padding=(18, 16))
        tab.pack(fill="both", expand=True)

        ttk.Label(tab, text=f"Link {joint['child']} und Gelenk {joint_name} "
                            f"entfernen", style="Card.TLabel").grid(
            row=0, column=0, sticky="w")

        if below:
            names = ", ".join(j["name"] for j in below)
            moved = (f"{len(below)} Gelenk(e) wandern von {joint['child']} "
                     f"an {joint['parent']}: {names}")
        else:
            moved = (f"{joint['child']} ist das Ende der Kette, es haengt "
                     f"nichts daran.")
        ttk.Label(tab, text=moved, style="Muted.TLabel").grid(
            row=1, column=0, sticky="w", pady=(6, 0))
        ttk.Label(tab, text=f"Danach: {joint['parent']}  ->  "
                            + (", ".join(j["child"] for j in below) or "(Ende)"),
                  style="Muted.TLabel").grid(row=2, column=0, sticky="w",
                                             pady=(4, 0))

        if joint["type"] != "fixed":
            ttk.Label(tab, text=f"Achtung: {joint_name} ist beweglich. Mit dem "
                                f"Gelenk faellt ein Freiheitsgrad weg.",
                      style="Muted.TLabel").grid(row=3, column=0, sticky="w",
                                                 pady=(10, 0))

        ttk.Separator(tab, orient="horizontal").grid(
            row=4, column=0, sticky="ew", pady=14)

        keep = ttk.Checkbutton(tab, text="Lage der nachfolgenden Glieder "
                                         "beibehalten",
                               variable=self.remove_keep_var)
        keep.grid(row=5, column=0, sticky="w")
        if not below:
            # Nichts zu verschieben, die Wahl waere ohne Wirkung.
            keep.state(["disabled"])
        ttk.Label(tab, text="Aus: die Urspruenge bleiben stehen und der Rest "
                            "der Kette rueckt um das Glied vor.",
                  style="Muted.TLabel").grid(row=6, column=0, sticky="w",
                                             pady=(4, 0))

        actions = ttk.Frame(tab, style="Card.TFrame")
        actions.grid(row=7, column=0, sticky="w", pady=(18, 0))
        ttk.Button(actions, text="Herausnehmen",
                   command=self._remove_apply).pack(side="left")
        ttk.Button(actions, text="Abbrechen",
                   command=self._close_remove_window).pack(side="left", padx=(8, 0))

    def _close_remove_window(self):
        if self.remove_window is not None and self.remove_window.winfo_exists():
            self.remove_window.grab_release()
            self.remove_window.destroy()
        self.remove_window = None
        self.remove_joint = None

    def _remove_apply(self):
        joint_name = self.remove_joint
        joint = self._joint_by_name(joint_name) if joint_name else None
        if joint is None:
            self._log(f"Gelenk {joint_name} gibt es nicht mehr", "error")
            self._close_remove_window()
            return

        gone_link = joint["child"]
        above = joint["parent"]
        keep = self.remove_keep_var.get()
        moved = sum(1 for j in self.joints if j["parent"] == gone_link)

        def mutate(draft):
            remove_limb(draft, joint_name, keep)

        text = (f"{gone_link} und {joint_name} herausgenommen, "
                f"{moved} Gelenk(e) an {above}")
        if self._commit(text, mutate):
            self._tree_select(None)
            self._close_remove_window()

    def _open_visual_for_selection(self):
        """Aus der Gelenkauswahl auf die Koerperseite springen."""
        joint = self._joint_by_name(self.edit_joint_var.get())
        self._focus_visual(joint["child"] if joint else None)

    def _focus_visual(self, link_name=None):
        """Koerperseite zeigen, wahlweise fuer einen bestimmten Link."""
        if self.editor is None or not self.editor.winfo_exists():
            self._open_editor()
        if link_name and any(l["name"] == link_name for l in self.links):
            self.visual_link_var.set(link_name)
            self._visual_load_link()
        if self.edit_pages is not None and self.edit_pages.winfo_exists():
            self.edit_pages.select(1)                  # 0 = Gelenk, 1 = Koerper
        self._show_window(self.editor)

    def _build_visual_tab(self, parent):
        """Karteikarte fuer Form, Groesse, Ursprung und Farbe eines Links.

        Bewusst auf diese vier beschraenkt. Mehrere Koerper je Link und die
        Traegheit bleiben aussen vor: das sind eigene Themen und wuerden aus
        der Seite einen zweiten Editor machen.
        """
        tab = ttk.Frame(parent, style="Card.TFrame", padding=(18, 16))
        self.visual_entries = {}

        head = ttk.Frame(tab, style="Card.TFrame")
        head.grid(row=0, column=0, columnspan=4, sticky="w", pady=(0, 14))
        ttk.Label(head, text="Link", style="Card.TLabel").pack(side="left")
        self.visual_combo = ttk.Combobox(head, textvariable=self.visual_link_var,
                                         state="readonly", width=18)
        self.visual_combo.pack(side="left", padx=(10, 18))
        self.visual_combo.bind("<<ComboboxSelected>>",
                               lambda _e: self._visual_load_link())
        ttk.Label(head, text="Form", style="Card.TLabel").pack(side="left")
        self.visual_shape_combo = ttk.Combobox(
            head, textvariable=self.visual_shape_var, state="readonly",
            width=11, values=list(SHAPES))
        self.visual_shape_combo.pack(side="left", padx=(10, 0))
        self.visual_shape_combo.bind("<<ComboboxSelected>>",
                                     lambda _e: self._visual_shape_changed())

        name_row = ttk.Frame(tab, style="Card.TFrame")
        name_row.grid(row=1, column=0, columnspan=4, sticky="w", pady=(0, 8))
        ttk.Label(name_row, text="NAME", style="Section.TLabel").pack(side="left")
        self.visual_name_entry = self._entry(name_row, width=22)
        self.visual_name_entry.configure(justify="left")
        self.visual_name_entry.pack(side="left", padx=(18, 8))
        self.visual_rename_button = ttk.Button(name_row, text="Umbenennen",
                                               command=self._rename_link)
        self.visual_rename_button.pack(side="left")
        ttk.Label(name_row, textvariable=self.visual_rename_hint_var,
                  style="Muted.TLabel").pack(side="left", padx=(12, 0))

        ttk.Label(tab, text="GROESSE  [m]", style="Section.TLabel").grid(
            row=2, column=0, sticky="w", pady=(4, 4))
        self.visual_size_frame = ttk.Frame(tab, style="Card.TFrame")
        self.visual_size_frame.grid(row=2, column=1, columnspan=3, sticky="w",
                                    pady=(4, 4))

        # Ursprung des Koerpers gegen den Linkframe. Dieselben Beschriftungen
        # und Einheiten wie eine Karteikarte weiter beim Gelenk, damit klar
        # bleibt, dass es dieselbe Groesse ist -- nur wirkt sie hier aufs Bild
        # und nicht auf die Kette.
        ttk.Label(tab, text="URSPRUNG  [m]", style="Section.TLabel").grid(
            row=3, column=0, sticky="w", pady=(10, 4))
        origin_row = ttk.Frame(tab, style="Card.TFrame")
        origin_row.grid(row=3, column=1, columnspan=3, sticky="w", pady=(10, 4))
        for key, label in (("ox", "X"), ("oy", "Y"), ("oz", "Z")):
            ttk.Label(origin_row, text=label, style="Muted.TLabel").pack(
                side="left", padx=(0, 6))
            field = self._entry(origin_row, width=11, value="0.00000")
            field.pack(side="left", padx=(0, 16))
            self.visual_entries[key] = field
        self.visual_half = ttk.Button(origin_row, text="Z = halbe Hoehe",
                                      command=self._visual_half_height)
        self.visual_half.pack(side="left")

        ttk.Label(tab, text="WINKEL  [deg]", style="Section.TLabel").grid(
            row=4, column=0, sticky="w", pady=(4, 4))
        angle_row = ttk.Frame(tab, style="Card.TFrame")
        angle_row.grid(row=4, column=1, columnspan=3, sticky="w", pady=(4, 4))
        for key, label in (("oroll", "Roll"), ("opitch", "Pitch"),
                           ("oyaw", "Yaw")):
            ttk.Label(angle_row, text=label, style="Muted.TLabel").pack(
                side="left", padx=(0, 6))
            field = self._entry(angle_row, width=11, value="0.000")
            field.pack(side="left", padx=(0, 16))
            self.visual_entries[key] = field

        ttk.Label(tab, text="FARBE", style="Section.TLabel").grid(
            row=5, column=0, sticky="w", pady=(10, 4))
        colour_row = ttk.Frame(tab, style="Card.TFrame")
        colour_row.grid(row=5, column=1, columnspan=3, sticky="w", pady=(10, 4))
        self.visual_swatch = tk.Label(colour_row, width=4, relief="solid",
                                      borderwidth=1, background=SURFACE)
        self.visual_swatch.pack(side="left", padx=(0, 10))
        hex_entry = self._entry(colour_row, width=10, value="#a0a0a2")
        hex_entry.configure(justify="left")
        hex_entry.pack(side="left")
        hex_entry.bind("<KeyRelease>", lambda _e: self._visual_preview())
        self.visual_entries["hex"] = hex_entry
        ttk.Button(colour_row, text="Waehlen...",
                   command=self._visual_pick).pack(side="left", padx=(8, 16))
        ttk.Label(colour_row, text="Deckung", style="Muted.TLabel").pack(side="left")
        alpha = self._entry(colour_row, width=8, value="1.000")
        alpha.pack(side="left", padx=(8, 0))
        self.visual_entries["alpha"] = alpha

        actions = ttk.Frame(tab, style="Card.TFrame")
        actions.grid(row=6, column=0, columnspan=4, sticky="w", pady=(16, 0))
        ttk.Button(actions, text="Uebernehmen",
                   command=self._visual_apply).pack(side="left")
        ttk.Checkbutton(actions, text="Kollisionskoerper mitziehen",
                        variable=self.visual_collision_var).pack(
            side="left", padx=(16, 0))

        ttk.Label(tab, textvariable=self.visual_note_var,
                  style="Muted.TLabel", justify="left").grid(
            row=7, column=0, columnspan=4, sticky="w", pady=(12, 0))
        return tab

    def _visual_load_link(self):
        """Felder auf den gewaehlten Link stellen."""
        if not self.visual_entries:
            return
        link = next((l for l in self.links
                     if l["name"] == self.visual_link_var.get()), None)
        if link is None:
            return
        visual = link["visual"]
        self.visual_filename = visual["filename"] if visual else ""
        self.visual_shape_var.set(visual["shape"] if visual and visual["shape"]
                                  else "box")
        rgba = visual["rgba"] if visual and visual["rgba"] else DEFAULT_RGBA
        self._set_entry(self.visual_entries, "hex", rgba_to_hex(rgba))
        self._set_entry(self.visual_entries, "alpha", f"{rgba[3]:.3f}")
        xyz = visual["xyz"] if visual else [0.0, 0.0, 0.0]
        rpy = visual["rpy"] if visual else [0.0, 0.0, 0.0]
        for key, value in zip(("ox", "oy", "oz"), xyz):
            self._set_entry(self.visual_entries, key, f"{value:.5g}")
        for key, value in zip(("oroll", "opitch", "oyaw"), rpy):
            self._set_entry(self.visual_entries, key,
                            f"{math.degrees(value):.4g}")
        # Netz nur anbieten, wenn eine Datei hinterlegt ist: ohne filename
        # waere die Form leer und das Modell kaputt.
        self.visual_shape_combo.configure(
            values=[k for k in SHAPES if k != "mesh" or self.visual_filename])
        # Vorschlag: mitziehen, solange nichts ueberschrieben wird. Hat der
        # Link schon einen Kollisionskoerper, koennte der bewusst groeber sein.
        self.visual_collision_var.set(not self._has_collision(link["name"]))
        self._visual_shape_changed()
        self._visual_preview()
        self._visual_note(link)

        # Der Wurzel-Link bleibt gesperrt. Sein Name steht ausserhalb der URDF
        # noch einmal: als fixed frame in display2.rviz und als Bezug jeder
        # Koordinatenangabe. Ein Umbenennen hier liesse RViz auf einen Frame
        # warten, den niemand mehr sendet.
        is_root = link["name"] in root_links(self.links, self.joints)
        self._set_entry({"n": self.visual_name_entry}, "n", link["name"],
                        enabled=not is_root)
        self.visual_rename_button.state(["disabled"] if is_root
                                        else ["!disabled"])
        self.visual_rename_hint_var.set(
            "Wurzel-Link, Name fest" if is_root else "")

    def _has_collision(self, link_name):
        element = next((l for l in self.xml_root.findall("link")
                        if l.get("name") == link_name), None)
        return element is not None and element.find("collision") is not None

    def _visual_note(self, link):
        """Hinweise, die den Bediener sonst kalt erwischen wuerden."""
        visual = link["visual"]
        notes = []
        if visual is None:
            notes.append("Der Link hat noch kein <visual> und ist in RViz "
                         "unsichtbar. Uebernehmen legt eines an.")
        else:
            if visual["count"] > 1:
                notes.append(f"Der Link hat {visual['count']} Koerper. "
                             f"Bearbeitet wird nur der erste.")
            if visual["shared"]:
                notes.append(f"Bisher gemeinsames Material "
                             f"\"{visual['material']}\". Uebernehmen gibt dem "
                             f"Link ein eigenes, andere bleiben unveraendert.")
        if self._has_collision(link["name"]):
            notes.append("Ein Kollisionskoerper ist vorhanden und wird nur "
                         "angefasst, wenn der Haken gesetzt ist.")
        self.visual_note_var.set("\n".join(notes))

    def _visual_shape_changed(self):
        """Groessenfelder auf die gewaehlte Form umstellen."""
        for child in self.visual_size_frame.winfo_children():
            child.destroy()
        # Farbe und Ursprung haengen nicht an der Form und bleiben stehen.
        for key in list(self.visual_entries):
            if key not in VISUAL_KEEP:
                del self.visual_entries[key]

        shape = self.visual_shape_var.get()
        link = next((l for l in self.links
                     if l["name"] == self.visual_link_var.get()), None)
        known = (link["visual"]["size"]
                 if link and link["visual"] and link["visual"]["shape"] == shape
                 else {})
        for key, label, default in SHAPE_FIELDS[shape]:
            ttk.Label(self.visual_size_frame, text=label,
                      style="Muted.TLabel").pack(side="left", padx=(0, 6))
            field = self._entry(self.visual_size_frame, width=11,
                                value=f"{known.get(key, default):.5g}")
            field.pack(side="left", padx=(0, 16))
            self.visual_entries[key] = field
        if shape == "mesh":
            ttk.Label(self.visual_size_frame,
                      text=os.path.basename(self.visual_filename),
                      style="Muted.TLabel").pack(side="left")
        # Kugel und Netz haben keine Hoehe entlang z, die man halbieren
        # koennte: beim Netz steckt sie in der Datei.
        if self.visual_half is not None and self.visual_half.winfo_exists():
            self.visual_half.state(["!disabled"] if shape in HAS_HEIGHT
                                   else ["disabled"])

    def _visual_half_height(self):
        """Z auf die halbe Bauhoehe setzen.

        Ohne <origin> sitzt ein Quader mittig auf dem Linkframe und ragt zur
        Haelfte hinter ihn zurueck. Fuer ein Armsegment, das entlang z vom
        Gelenk weg zeigt, gehoert dort die halbe Hoehe -- dann beginnt der
        Koerper am Frame. Die Taste nimmt diesen einen haeufigen Fall ab und
        rechnet nichts, was man nicht auch selbst eintippen koennte.
        """
        key = HAS_HEIGHT.get(self.visual_shape_var.get())
        if key is None:
            return
        try:
            height = self._read_entry(self.visual_entries, key)
        except ValueError:
            self._log("Hoehe nicht lesbar", "warn")
            return
        self._set_entry(self.visual_entries, "oz", f"{height / 2.0:.5g}")

    def _visual_preview(self):
        """Farbfeld nachziehen. Unlesbare Eingabe faerbt es neutral."""
        try:
            colour = rgba_to_hex(hex_to_rgb(self.visual_entries["hex"].get()))
        except ValueError:
            colour = SURFACE
        if self.visual_swatch is not None and self.visual_swatch.winfo_exists():
            self.visual_swatch.configure(background=colour)

    def _visual_pick(self):
        from tkinter import colorchooser
        current = self.visual_entries["hex"].get().strip()
        chosen = colorchooser.askcolor(color=current if current.startswith("#")
                                       else None, parent=self.editor)
        if chosen and chosen[1]:
            self._set_entry(self.visual_entries, "hex", chosen[1])
            self._visual_preview()

    def _visual_apply(self):
        link_name = self.visual_link_var.get()
        if not link_name or not any(l["name"] == link_name
                                    for l in self.links):
            self._log("Kein Link gewaehlt", "warn")
            return

        shape = self.visual_shape_var.get()
        try:
            size = {key: self._read_entry(self.visual_entries, key)
                    for key, _label, _default in SHAPE_FIELDS[shape]}
            alpha = self._read_entry(self.visual_entries, "alpha")
            xyz = [self._read_entry(self.visual_entries, k)
                   for k in ("ox", "oy", "oz")]
            rpy = [math.radians(self._read_entry(self.visual_entries, k))
                   for k in ("oroll", "opitch", "oyaw")]
        except ValueError:
            self._log("Eingabe nicht lesbar, Aussehen unveraendert", "error")
            return

        problems = [f"{label} muss groesser als null sein"
                    for key, label, _d in SHAPE_FIELDS[shape] if size[key] <= 0]
        if not 0.0 <= alpha <= 1.0:
            problems.append("Deckung liegt zwischen 0 und 1")
        try:
            rgb = hex_to_rgb(self.visual_entries["hex"].get())
        except ValueError as exc:
            problems.append(str(exc))
            rgb = DEFAULT_RGBA[:3]
        if problems:
            messagebox.showerror("Aussehen", "\n".join(problems))
            for problem in problems:
                self._log(problem, "error")
            return

        rgba = (*rgb, alpha)
        filename = self.visual_filename
        with_collision = self.visual_collision_var.get()

        def mutate(draft):
            set_visual(draft, link_name, shape, size, rgba, filename,
                       xyz, rpy, with_collision)

        moved = any(abs(v) > 1e-9 for v in xyz + rpy)
        text = (f"{link_name}: {SHAPE_LABELS[shape]}, {rgba_to_hex(rgba)}"
                f"{', versetzt' if moved else ''}"
                f"{', Kollision mit' if with_collision else ''}")
        # Kein Schliessen wie bei einem Fenster: die Seite bleibt stehen,
        # damit man am selben Link weiterarbeiten kann. _rebuild_from_root
        # laedt die Felder ueber _edit_fill_combo neu.
        self._commit(text, mutate)

    def _edit_load_coupling(self, joint):
        """Kopplungsfelder auf das gewaehlte Gelenk stellen."""
        if self.edit_mimic_combo is None or not self.edit_mimic_combo.winfo_exists():
            return
        rotational = is_rotational(joint)
        self.edit_mimic_unit_var.set("deg" if rotational else "m")

        # Als Fuehrungsgelenk kommt nur in Frage, was selbst frei stellbar ist
        # und nicht dieses Gelenk. Die Liste zeigt damit genau die gueltigen
        # Wahlmoeglichkeiten, statt sie erst beim Setzen abzulehnen.
        leaders = [j["name"] for j in self.joints
                   if j["type"] != "fixed" and j["name"] != joint["name"]
                   and j["mimic"] is None]
        self.edit_mimic_combo.configure(values=[NO_COUPLING] + leaders)

        mimic = joint["mimic"]
        if mimic is None:
            self.edit_mimic_var.set(NO_COUPLING)
            self._edit_set("factor", "1.0000")
            self._edit_set("mimic_offset", "0.000")
        else:
            self.edit_mimic_var.set(mimic["joint"])
            self._edit_set("factor", f"{mimic['multiplier']:.4f}")
            offset = (math.degrees(mimic["offset"]) if rotational
                      else mimic["offset"])
            self._edit_set("mimic_offset", f"{offset:.3f}")

        # Ein festes Gelenk hat nichts zu koppeln, ein Fuehrungsgelenk darf
        # nicht selbst folgen -- beides waere eine Kette.
        drives = any(j["mimic"] is not None
                     and j["mimic"]["joint"] == joint["name"]
                     for j in self.joints)
        blocked = joint["type"] == "fixed" or drives
        # "!disabled" muss ausdruecklich dastehen: ttk-state() ergaenzt Flaggen,
        # es ersetzt sie nicht. Ein blosses state(["readonly"]) liesse ein
        # zuvor gesetztes "disabled" stehen, und die Auswahl bliebe fuer den
        # Rest der Sitzung tot -- nach dem ersten festen Gelenk in der Liste.
        self.edit_mimic_combo.state(["disabled" if blocked else "!disabled",
                                     "readonly"])

    def _apply_coupling(self):
        """Kopplung setzen, aendern oder loesen."""
        joint = self._edit_current_joint()
        if joint is None:
            self._log("Kein Gelenk gewaehlt", "warn")
            return
        name = joint["name"]
        source = self.edit_mimic_var.get()

        if source == NO_COUPLING:
            if joint["mimic"] is None:
                return                                 # war schon frei
            self._commit(f"Kopplung von {name} geloest",
                         lambda draft: clear_mimic(draft, name))
            return

        problem = coupling_problem(self.joints, name, source)
        if problem:
            messagebox.showerror("Kopplung", problem)
            self._log(problem, "error")
            return

        try:
            factor = self._read_entry(self.edit_entries, "factor")
            offset = self._read_entry(self.edit_entries, "mimic_offset")
        except ValueError:
            self._log("Faktor oder Versatz nicht lesbar", "error")
            return
        if is_rotational(joint):
            offset = math.radians(offset)

        leader = self._joint_by_name(source)
        if is_rotational(joint) != is_rotational(leader):
            self._log(f"{name} und {source} sind verschiedene Bauarten, der "
                      f"Faktor traegt damit eine Einheit", "warn")

        # Warnen, wenn die Kopplung das gekoppelte Gelenk ueber seinen
        # Anschlag treiben wuerde. Abgeschnitten wird beim Rechnen ohnehin,
        # nur merkt es sonst niemand.
        if joint["has_limit"] and leader["has_limit"]:
            reach = [factor * q + offset for q in (leader["lower"], leader["upper"])]
            if min(reach) < joint["lower"] - 1e-9 or max(reach) > joint["upper"] + 1e-9:
                self._log(f"{name} laeuft bei vollem Weg von {source} in "
                          f"seinen Anschlag und wird begrenzt", "warn")

        def mutate(draft):
            set_mimic(draft, name, source, factor, offset)

        self._commit(f"{name} folgt {source} mit {factor:.4g}", mutate)

    def _rename_joint(self):
        """Gelenk umbenennen. Traegt die Sollstellung auf den neuen Namen um."""
        old = self.edit_joint_var.get()
        joint = self._joint_by_name(old)
        if joint is None:
            self._log("Kein Gelenk gewaehlt", "warn")
            return
        new = self.edit_name_entry.get().strip()
        if new == old:
            return                                     # nichts zu tun

        taken = {j["name"] for j in self.joints} - {old}
        problem = name_problem(new, "Gelenk", taken)
        if problem is None and any(l["name"] == new for l in self.links):
            problem = f'"{new}" ist schon ein Linkname'
        if problem:
            messagebox.showerror("Umbenennen", problem)
            self._log(problem, "error")
            return

        # Die Sollstellung haengt am Namen. Ohne diesen Schritt faende
        # _rebuild_from_root nichts und das Gelenk spraenge auf null.
        value = self.positions.pop(old, None)
        if value is not None:
            self.positions[new] = value

        def mutate(draft):
            rename_joint(draft, old, new)

        if self._commit(f"Gelenk {old} heisst jetzt {new}", mutate):
            self.edit_joint_var.set(new)
            self._tree_select(new)
        elif value is not None:                        # Umbau abgelehnt
            self.positions.pop(new, None)
            self.positions[old] = value

    def _rename_link(self):
        """Link umbenennen. Der Wurzel-Link bleibt aussen vor."""
        old = self.visual_link_var.get()
        link = next((l for l in self.links if l["name"] == old), None)
        if link is None:
            self._log("Kein Link gewaehlt", "warn")
            return
        if old in root_links(self.links, self.joints):
            self._log(f"{old} ist der Wurzel-Link und behaelt seinen Namen",
                      "warn")
            return
        new = self.visual_name_entry.get().strip()
        if new == old:
            return

        taken = {l["name"] for l in self.links} - {old}
        problem = name_problem(new, "Link", taken)
        if problem is None and any(j["name"] == new for j in self.joints):
            problem = f'"{new}" ist schon ein Gelenkname'
        if problem:
            messagebox.showerror("Umbenennen", problem)
            self._log(problem, "error")
            return

        def mutate(draft):
            rename_link(draft, old, new)

        if self._commit(f"Link {old} heisst jetzt {new}", mutate):
            self.visual_link_var.set(new)
            self._visual_load_link()

    def _delete_branch(self):
        """Ausgewaehltes Gelenk mit seinem Kindlink und allem dahinter loeschen.

        Einzeln geht nicht: bliebe der Kindlink stehen, waere er ein zweiter
        Wurzel-Link. Was verschwindet, steht vorher in der Rueckfrage.
        """
        joint = self._edit_current_joint()
        if joint is None or self.xml_root is None:
            return

        going_links = subtree_links(self.joints, joint["child"])
        going_joints = [j["name"] for j in self.joints
                        if j["child"] in set(going_links)]

        roots = root_links(self.links, self.joints)
        if joint["child"] in roots:                    # kann eigentlich nicht sein
            messagebox.showerror("Loeschen", "Der Wurzel-Link bleibt bestehen.")
            return

        text = (f"Gelenk {joint['name']} und alles dahinter entfernen?\n\n"
                f"Links ({len(going_links)}): {', '.join(going_links)}\n"
                f"Gelenke ({len(going_joints)}): {', '.join(going_joints)}")
        if len(going_links) > 1:
            text += "\n\nDas ist ein ganzer Teilbaum, nicht nur ein Glied."
        if not messagebox.askyesno("Loeschen", text):
            return

        def mutate(draft):
            remove_branch(draft, going_links)

        self._commit(f"{joint['name']} und {len(going_links)} Link(s) geloescht",
                     mutate)

    def _rebuild_from_root(self):
        """Modell, Tabellen, FK und TF-Ausgabe an den geaenderten Baum anpassen."""
        self.robot_name, self.links, self.joints = parse_root(self.xml_root)
        self.urdf_xml = serialize(self.xml_root)

        # Sollwerte behalten, aber an eventuell engere Grenzen anpassen. Ein
        # Gelenk kann durch den Typwechsel beweglich geworden oder fest
        # geworden sein, deshalb wird die Zuordnung neu aufgebaut.
        positions = {}
        for joint in self.joints:
            if joint["type"] == "fixed":
                continue
            value = self.positions.get(joint["name"], 0.0)
            if joint["has_limit"]:
                value = min(max(value, joint["lower"]), joint["upper"])
            positions[joint["name"]] = value
        self.positions = positions

        self._update_header()
        self._build_joint_inputs()
        self._sync_joint_entries()
        self._fill_link_table()
        self._update_kinematics()
        self._draw_tree()
        # Nach dem Anlegen oder Loeschen stimmt die Auswahlliste nicht mehr.
        if self.editor is not None and self.editor.winfo_exists():
            self._edit_fill_combo()

        # Der Stern haengt am Inhalt, nicht am Zaehlen der Aenderungen:
        # nach Rueckgaengig bis zum Ausgangsstand ist nichts mehr offen.
        self._set_dirty(self.urdf_xml != self.saved_signature)

        # Achsgrenzen an das neue Modell anpassen: neue Gelenke bekommen
        # Startwerte aus der URDF, verschwundene fallen heraus.
        self.moveit_config.sync_joints(self.joints)
        if self.params_window is not None and self.params_window.winfo_exists():
            # Die Tabelle der Achsgrenzen haengt an der Gelenkliste und muss
            # deshalb komplett neu entstehen.
            self._close_params()
            self._open_params()

        self.ros.set_model(self.joints, self.urdf_xml)
        self._push_positions()

    # ----------------------------------------------------------------------
    # ROS und Fusszeile
    # ----------------------------------------------------------------------

    def _start_ros(self):
        error = self.ros.start()
        if error:
            self._log(error, "warn")

    def _toggle_tf(self):
        self.ros.enabled = self.send_tf.get()
        if not self.send_tf.get():
            self._log("TF-Ausgabe angehalten. RViz verliert nach der Cache-Zeit "
                      "von etwa 10 Sekunden alle Frames.", "warn")
        else:
            self._log("TF-Ausgabe laeuft wieder", "ok")

    def _tick(self):
        """Fusszeile aktualisieren. Laeuft im GUI-Thread, greift nie auf ROS zu."""
        elapsed = time.monotonic() - self.session_start
        whole = int(elapsed)
        self.session_var.set(
            f"Session  {whole // 3600:02d}:{whole // 60 % 60:02d}:"
            f"{whole % 60:02d}:{int((elapsed - whole) * 1000):03d}")

        if self.state == STATE_PLANNING:
            self._drive_status()

        for kind, value in self.ros.take_results():
            if kind == "moveit":
                self._log("move_group meldet sich" if value
                          else "move_group ist weg", "ok" if value else "warn")
            elif kind == "moveit_nodes":
                # Mehr als einer im Graphen. Welcher antwortet, entscheidet
                # ab hier nicht mehr die GUI.
                self._log(f"{len(value)} move_group im Graphen: "
                          + ", ".join(value), "warn")
            elif kind == "ros_error":
                # Eine Ausnahme aus dem ROS-Takt. Sie hat den Takt nicht mehr
                # beendet -- aber sie gehoert gelesen, denn irgendetwas hat
                # nicht stattgefunden.
                self._log(f"Fehler im ROS-Takt: {value}", "error")
            elif kind == "plan":
                self._on_plan_result(value)
        self._moveit_watch()
        self._params_status()

        last = self.ros.last_publish
        if not self.ros.running:
            self.refresh_var.set("Last Refresh  --   kein ROS")
        elif not self.send_tf.get():
            self.refresh_var.set("Last Refresh  --   angehalten")
        elif last is None:
            self.refresh_var.set("Last Refresh  --   wartet")
        else:
            age_ms = (time.monotonic() - last) * 1000.0
            cycle = self.ros.cycle_ms
            takt = f"   Takt {cycle:5.1f} ms" if cycle is not None else ""
            self.refresh_var.set(f"Last Refresh {age_ms:6.1f} ms{takt}")
            self._tf_watchdog(age_ms)

        self.after(FOOTER_TICK_MS, self._tick)

    def _on_close(self):
        if self.dirty and not messagebox.askyesno(
                "Beenden", "Es gibt nicht gespeicherte Aenderungen. Trotzdem beenden?"):
            return
        if self.motion_job is not None:
            self.after_cancel(self.motion_job)
            self.motion_job = None
        # Beim geordneten Beenden darf der Arbeitsordner mit: hier ist nichts
        # aufzuklaeren. Liegen bleibt er nur, wenn die GUI selbst abstuerzt --
        # und dann wird er beim naechsten Start eingesammelt.
        self._close_drive()
        self.moveit.stop(keep_workdir=False)
        self.ros.shutdown()
        self.destroy()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("urdf", nargs="?", default=URDF_PATH)
    args, _ = parser.parse_known_args()

    app = UrdfViewer(args.urdf)
    app.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
