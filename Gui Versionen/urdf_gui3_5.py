#!/usr/bin/env python3
"""URDF anzeigen, Achsen vorgeben, Vorwaertskinematik rechnen, TF senden.

3.5 nimmt Achsziele in die Punktfolge.

Bis 3.4 kannte die Punktliste nur Lagen. Jeder Punkt ging ueber
/compute_ik, und das ist genau an der Stelle schwach, an der eine Messung
beginnen soll: in der Nullstellung stehen joint5 auf null und die Achsen 4
und 7 in einer Flucht. Die Rueckwaertskinematik hat dort unendlich viele
Loesungen, und welche herauskommt, entscheidet der Startwert. Dieselbe
Liste zweimal geplant konnte also von zwei verschiedenen Achsstaenden aus
beginnen -- bei gleicher TCP-Lage.

Eine Zeile, die mit J beginnt, traegt deshalb Achswerte:

    J 0 0 0 0 0 0 0   tempo=50 name=Null

Ein Wert je beweglicher Achse, in der Reihenfolge des Fahrfensters, Grad
bzw. Meter bei Schubachsen. Gekoppelte Gelenke stehen nicht dabei.

    Ein Achsziel faehrt immer PTP ohne Ueberschleifen. fahrart= und
    radius= sind auf der Zeile ein Lesefehler, auch als PTP und 0 -- dort
    wirkten sie nicht weiter und saehen nur so aus. Der Modalzustand von
    Fahrart und Radius geht ueber die J-Zeile hinweg; der Bericht zeigt
    die festgelegten Werte in eckigen Klammern.

    Begrenzt wird nichts. Ein Wert jenseits der URDF-Grenze ist ein Fehler
    der Liste. Anders als im Fahrfenster, das begrenzt und meldet: dort
    sieht man das Ergebnis im Feld, in einer Folge von zwanzig Zeilen
    nicht.

    Die Lage des Achsziels kommt aus der Vorwaertskinematik, ueber
    dieselbe _tcp_pose wie die TCP-Zeile. Gebraucht wird sie nur fuer die
    Pruefung der Ueberschleifkugel des Vorgaengers und fuer den Bericht.
    Die Achswerte bleiben das Ziel.

    In der Kette der Rueckwaertskinematik wird ein Achsziel uebersprungen
    und setzt den Startwert fuer den naechsten kartesischen Punkt. Ein
    Achsziel vor einem kartesischen Punkt legt damit auch dessen
    Konfigurationszweig fest.

Die Zuordnung der Werte zu den Gelenken und die Grenzpruefung liegen in
sequenz.loese_achsziel, geprueft in test_sequenz_achsziel.py und
mutation_sequenz_achsziel.py.

3.4 faehrt die geplante Folge ab.

Bis 3.3 endete die Punktfolge im Bericht. Sie war gerechnet, sie war
geprueft, sie liess sich als CSV ansehen -- und dann stand sie. Zum
Vergleich mit einer industriellen Steuerung genuegt das nicht: was dort
gemessen wird, ist die Zeit zwischen Start und letztem Punkt und die
Abweichung der wirklich gefahrenen Bahn von der gerechneten. Beides
entsteht erst beim Fahren.

Gefahren wird ueber denselben TrajectoryPlayer wie ein einzelner Plan.
Das ist die ganze Aenderung an der Bewegung: es gibt weiterhin genau
einen Weg in eine Fahrt, und er heisst _start_playing. Eine zweite
Abtastung fuer Folgen haette zwei Zeitrechnungen ergeben, und zwei
Zeitrechnungen sind beim Messen von Zykluszeiten keine Kleinigkeit.

Die Folge wird gehalten, nicht sofort gefahren -- dieselbe Trennung wie
seit 2.7 zwischen "Folge planen" und "Folge fahren". Der gehaltene Plan
faellt, sobald ein Sollwert gestellt wird oder ein Fahrparameter sich
aendert; beides zieht die Rechnung unter der Bahn weg. Die Pruefung vor
dem Start ist doppelt: der Achsstand gegen den Stand beim Planen (exakt,
wie beim Einzelplan) und der Achsstand gegen den ersten Stuetzpunkt der
zurueckgegebenen Bahn (mit FOLGE_START_TOL). Die zweite faengt den Fall,
in dem Pilz eine Bahn liefert, die woanders beginnt, als gefragt wurde --
der erste Takt waere ein Sprung.

3.3 macht die Abtastzeit der Pilz-Pipeline im Parameterfenster
erreichbar -- die Folgenfahrt ruehrt daran nichts.

3.2 loest die Punkte selbst auf, bevor die Folge hinausgeht.

In 3.1 gingen Lageziele an Pilz, und Pilz rechnete die
Rueckwaertskinematik selbst. Das sieht sparsam aus und ist es nicht. Ein
Blick in trajectory_generator_ptp.cpp zeigt, was dort geschieht:
extractMotionPlanInfo ruft computePoseIK, und zwar mit dem Startzustand
des jeweiligen Teilauftrags als Startwert. Fuer den ersten Auftrag ist
das der Achsstand des Roboters, fuer jeden weiteren das Ziel des
vorigen -- eine Kette, in die von aussen nichts hineinreicht.

Bei sieben Achsen ist das die falsche Stelle fuer diese Rechnung. Die
Loesung ist nicht eindeutig; welche herauskommt, entscheidet der
Startwert, und der steht dort nicht zur Verfuegung. Scheitert es, nennt
die Meldung nur den Link und nicht den Punkt:

    Failed to compute inverse kinematics for link: tcp11 of goal pose

Bei drei Punkten hilft Raten noch, bei acht nicht mehr.

Also andersherum. Die Oberflaeche loest jeden Punkt einzeln ueber
/compute_ik auf -- mit der Loesung des Vorgaengers als Startwert, mit
Kollisionspruefung und mit der Zeitgrenze aus dem Parameterfenster --
und schickt danach Achsziele. Der Zweig joint_constraints in
extractMotionPlanInfo geht an computePoseIK vorbei; die Meldung oben
kann danach nicht mehr entstehen.

Drei Dinge gewinnt man damit, und keins davon ist Geschwindigkeit.

    Der Konfigurationszweig steht fest und ist sichtbar. Zu jedem Punkt
    steht im Bericht, welche Achswerte gewaehlt wurden und wie weit sie
    vom Vorgaenger entfernt liegen. Ein Sprung faellt damit vor dem
    Planen auf und nicht beim Zusehen.

    Es ist zu erkennen, welcher Punkt keine Loesung hat. Die Kette
    bricht an ihm ab, sein Name steht in der Meldung, und seine sechs
    Zahlen stehen daneben.

    Die Suche laeuft ueber schlechte Zweige hinweg. /compute_ik mit
    avoid_collisions reicht dem Loeser eine Gueltigkeitspruefung mit und
    sucht weiter, wenn eine gefundene Loesung in Selbstkollision steht.
    computePoseIK tut das ebenfalls -- aber eben mit einem Startwert,
    den man nicht setzen kann.

Dazu zwei Kleinigkeiten, die 3.1 offen liess: bei einem Folgenfehler
wird jetzt der Auszug aus dem move_group-Protokoll geholt, wie es
_on_plan_result seit langem tut, und der Text zu Fehlercode 99999 ist
berichtigt. Er hiess "kein Planer hat geantwortet", was er nicht ist:
99999 ist FAILURE, der Sammelwert. Nicht geantwortet hat 0, UNDEFINED.

Gefahren wird eine Folge weiterhin nicht.

3.1 plant Punktfolgen. Bis hierher konnte die Oberflaeche genau ein Ziel
anfahren; eine Folge von Punkten war dreimal planen und dreimal fahren, mit
Stillstand an jedem Zwischenpunkt. Das ist fuer eine Zykluszeit die
denkbar schlechteste Loesung: drei Beschleunigungs- und drei Bremsrampen,
obwohl nur der letzte Punkt wirklich angefahren wird.

Der Pilz-Planer kann es besser, und zwar mit einer eigenen Faehigkeit, die
in params_yaml seit langem eingeschaltet ist, ohne dass sie je gerufen
wurde: MoveGroupSequenceService, erreichbar unter /plan_sequence_path.
Sie nimmt eine MotionSequenceRequest entgegen -- eine Liste vollstaendiger
Planauftraege, jeder mit einem zusaetzlichen blend_radius. Liegt der ueber
null, wird die Teilbahn mit der folgenden verschmolzen und der Roboter
haelt an diesem Punkt nicht an.

Das ist dieselbe Groesse, die ABB zonedata nennt, KUKA C_DIS und Fanuc
CNT. Damit wird der Vergleich mit einer industriellen Steuerung zu einer
Messung mit einer einzigen freien Zahl: dieselbe Punktfolge, derselbe
Override, der Radius von null bis zu gross durchgefahren, und aufgetragen
Zykluszeit gegen Bahnabweichung.

Drei Entscheidungen stecken in dieser Fassung.

    Die Punktliste ist modal, und sie zeigt es. Eine Zeile darf Fahrart,
    Tempo, Beschleunigung und Radius mitbringen; laesst sie ein Feld weg,
    gilt der zuletzt gesetzte Wert weiter, wie F1000 im G-Code weitergilt.
    Genau daran scheitert man beim Programmieren einer echten Steuerung:
    ein Tempo, das drei Punkte spaeter noch wirkt. Der Bericht schreibt
    geerbte Werte deshalb in Klammern -- die Zeile sagt nicht nur, was
    gilt, sondern auch, dass es geerbt ist.

    Geprueft wird vor dem Auftrag, nicht danach. Zwei aufeinander
    folgende Ueberschleifkugeln duerfen sich nicht ueberlappen, und der
    letzte Punkt braucht Radius null. MoveIt weiss das auch, sagt aber
    nur, dass die Folge abgelehnt ist. Bei acht Punkten ist das der
    Unterschied zwischen einer Meldung und einer Suche. Die Pruefung
    liegt in sequenz.py, ohne Tkinter und ohne ROS, und ist mit
    test_sequenz.py eigenstaendig zu pruefen.

    Es wird nicht gefahren. 3.1 plant, zeigt an und schreibt eine CSV
    fuer die Bahnansicht. Das Fahren einer Folge braucht den
    Halte-Taster, den Wachhund und die Frage, was beim Loslassen mitten
    im Ueberschleifen geschieht; das ist eine eigene Fassung wert und
    nicht ein Anhaengsel an diese.

Was aus der Antwort zurueckkommt, ist keine Bahn, sondern eine Liste von
Bahnen: planned_trajectories. Ueberschliffene Uebergaenge liegen in
derselben Teilbahn, nicht ueberschliffene trennen sie. Die Zahl der
Teilbahnen ist damit die Antwort des Planers auf die Frage, ob das
Ueberschleifen ueberhaupt gegriffen hat -- eine Zahl, die im Protokoll
steht, weil man sie sonst nirgends sieht. Aneinandergehaengt werden sie
hier: jede weitere beginnt bei t = 0 an dem Ort, an dem die vorige endet,
und ihr erster Stuetzpunkt ist deshalb der letzte der vorigen.

Dazu ein bekanntes Verhalten von MoveIt, das den Abtaster trifft: beim
Ueberschleifen entstehen Stuetzpunkte mit gleichem time_from_start. Der
Abtaster interpoliert ueber die Zeit; zwei gleiche Stempel sind dort eine
Division durch null. bereinige_zeitreihe wirft sie weg und meldet, wie
viele -- still ausgebessert wuerde der Fehler verschwinden, ohne dass
jemand wuesste, dass er da war.

Geprueft wird das in test_sequenz.py und mutation_sequenz.py.

Aus 2.26 unveraendert uebernommen: die TCP-Lage im Kopf des Fahrfensters.

2.26 stellt die TCP-Lage in den Kopf des Fahrfensters. Sie stand bisher nur
im Hauptfenster, und das Fahrfenster liegt oft genug davor.

Zwei Entscheidungen stecken darin, und beide sind der Grund, aus dem es
nicht mit einem zweiten Label getan war.

    Sie rollt nicht mit. Der obere Teil des Fahrfensters liegt seit 2.21 in
    einem Canvas, weil sieben Achsen, zwei Zielraeume und die Fahrparameter
    auf einen Beamer nicht anders passen. Eine Anzeige darin waere
    wegrollbar, und wer den Halte-Taster drueckt, hat die Hand nicht am Rad.
    Die Zeile bekommt deshalb eine eigene Karte in Zeile 0 des Fensters,
    neben dem Canvas und nicht darin. Das Gewicht liegt auf Zeile 1: waechst
    das Fenster, waechst der rollende Teil, und Kopf und Fuss bleiben stehen.

    Sie hat einen eigenen Takt und trotzdem keine eigene Rechnung.
    _update_kinematics fuellt die Frametabelle mit und ist damit zu teuer
    fuer 50 Hz -- waehrend einer Fahrt laeuft es genau einmal, am Ende. Die
    Zeile im Hauptfenster steht in dieser Zeit still, und das ist dort
    richtig. Oben im Fahrfenster waere es falsch: eine Anzeige, die genau
    dann einfriert, wenn sich etwas bewegt, ist schlechter als keine.
    _update_drive_tcp haengt deshalb an _push_positions, dem einen Weg, ueber
    den jeder geaenderte Achsstand hinausgeht -- Schieber, Sollwertfeld, Jog,
    Fahrtakt. An den Aufrufern haette man so viele Stellen zu pflegen, wie es
    Wege gibt, und beim naechsten neuen Weg eine Anzeige, die manchmal
    stehenbleibt. Dazu ein zweiter Aufruf in _update_kinematics: ein neues
    Modell, eine andere Drehkonvention, ein anderer aktiver TCP sind keine
    Achsbewegungen und gehen an _push_positions vorbei.

Gerechnet wird nur bei offenem Fahrfenster; die Pruefung steht vor der
Kinematik und nicht danach. Ist das Fenster zu, kostet der Aufruf im
Fahrtakt nichts.

Die Zahlen kommen aus _pose_text, also aus derselben Vorwaertskinematik und
derselben Anzeigekonvention wie die Zeile im Hauptfenster. _pose_text nimmt
seit 2.26 eine fertige Lage entgegen, statt sie selbst zu holen: das spart
im Fahrtakt einen zweiten Durchlauf, und vor allem koennen Zahl und Faerbung
nicht mehr aus zwei Rechnungen stammen, die zwar fast immer gleich sind,
aber eben nur fast. Was oben steht, ist damit immer dasselbe wie unten --
nur oefter.

Drei Stufen wie bei der Zeile im Hauptfenster, eine Schriftgroesse groesser,
weil man sie aus zwei Metern liest. Rot heisst, dass es nichts anzuzeigen
gibt: kein TCP gewaehlt, oder einer, der nicht an der Basis haengt. Gelb
heisst, dass die Winkel entartet sind -- die Zahlen stimmen, aber zwei von
ihnen springen bei der naechsten kleinen Bewegung. Die Farbe wiederholt nur,
was ohnehin dasteht.

Unter der Zeile steht, was sie nicht ist. Das Fahrfenster traegt kartesische
Zielfelder, die genauso aussehen; die eine Stelle zeigt den Stand, die
andere das Vorhaben. Sie zu verwechseln ist derselbe Fehler wie der, den 2.7
zwischen Sollwert und Fahrziel aufgetrennt hat.

Geprueft wird das in test_fahrtcp_2_26.py.

Aus 2.25 unveraendert uebernommen: der reparierte Bahnrekorder.

2.25 macht den Bahnrekorder wieder gangbar. Das Fehlerbild war eindeutig und
die Ursache trotzdem schlecht zu sehen: der Knopf faerbte sich rot, die
Anzeige blieb bei "0 Punkte  0.0 s", und der Export meldete, es gebe nichts
zu schreiben. Anhalten und erneutes Aufzeichnen half nicht, ein Neustart des
Programms schon.

Zwei Dinge lagen uebereinander.

    Das Wort. In _frame_snapshot stand flat.extend(rpy). Den Namen rpy gibt
    es in dieser Methode nicht; sie rechnet seit 2.20 ueber
    konvention.to_xyz_abc und bekommt die drei Winkel als abc zurueck. Beim
    Umstieg auf die waehlbare Drehkonvention wurde die Zeile darueber
    mitgezogen und diese nicht. Was dabei herauskam, war kein falscher Wert,
    sondern ein NameError -- ein Fehler, den keine Rechnung ueberlebt.

    Der Takt. _record_tick bestellte sich in seiner letzten Zeile selbst
    nach. Fliegt eine Ausnahme aus dem Rumpf, wird diese Zeile nicht
    erreicht, und damit ist der 25-Hz-Takt fuer den Rest der Sitzung weg.
    Das ist der Grund, aus dem ein Wortfehler in einem Blatt der Rechnung
    die ganze Baugruppe stilllegte, und der Grund, aus dem Anhalten nichts
    besserte: den Takt startet ausser __init__ niemand.

Der Wortfehler steht seit 2.20 in der Datei und hat 2.21 bis 2.24
unbeschadet ueberstanden -- niemand hat in dieser Zeit aufgezeichnet, und
der Traceback aus einem Tk-Rueckruf landet in stderr, wo ihn beim Start
ueber display_launch2.py niemand liest. Wie lange ein Fehler unbemerkt
liegen kann, haengt nicht an seiner Groesse, sondern daran, wer hinsieht.

Deshalb ist die Reparatur zweiteilig. Das Wort wird berichtigt, und die
Abtastung wandert in _record_sample. _record_tick haelt nur noch den Takt:
try um die Abtastung, finally um das after. Ein Fehler kostet jetzt den
Puffer und eine Zeile im Log, aber nicht den Takt. Verworfen wird ueber
_drop_record und nicht uebersprungen -- was ein Puffer nach einer Ausnahme
enthaelt, ist nicht mehr zu belegen, und eine Bahn mit einem Loch darin ist
schlechter als keine.

Dieselbe Regel steht schon bei den ROS-Rueckrufen, dort mit demselben
Fehlerbild eine Etage tiefer: der Executor-Thread stirbt still, /tf hoert
auf zu senden, und der Timer laeuft weiter. Ein Takt darf an dem sterben,
was er tut, aber nicht daran, dass er es tut.

Geprueft wird das in test_rekorder_2_25.py: _frame_snapshot gegen sechs
Zahlen je Frame, und der Takt gegen eine absichtlich gesetzte Ausnahme.
Dazu die Mutationssonden -- die erste setzt rpy zurueck, die zweite legt das
after wieder ans Rumpfende. Beide muessen die Pruefung zum Fallen bringen,
sonst prueft sie nichts. Die Reihe sagt vorweg an, welche Datei sie prueft,
und bricht ab, wenn es ein aelterer Stand ist: sonst meldet sie den alten
Fehler als Fehler des neuen.

Eine 2.24 gibt es nicht. Die Nummer bleibt frei.

Aus 2.23 unveraendert uebernommen: Fahrparameter ohne Uebernehmen-Taste.

2.23 nimmt den Fahrparametern die Uebernehmen-Taste. Was ins Feld getippt
wird, gilt, sobald die Eingabe fertig ist.

2.22 hatte dort eine Taste und daneben eine Zeile, die den Unterschied
zwischen Entwurf und Wirkung ansagte. Der Gedanke war richtig und der Weg
umstaendlich: fuer eine Zahl drei Handlungen -- tippen, zur Taste greifen,
druecken -- und wer die dritte vergisst, faehrt mit dem alten Wert und sucht
den Fehler an einer anderen Stelle. Eine vergessene Bestaetigung ist der
schlechteste aller Zustaende, weil das Fenster den neuen Wert zeigt und die
Maschine den alten benutzt.

Uebernommen wird jetzt bei Return und beim Verlassen des Feldes, bei der
Fahrart mit der Auswahl selbst. Das ist die eigentliche Abgrenzung dieser
Version: nicht bei jedem Tastendruck. Wer 40 tippt, hat nach dem ersten
Zeichen eine 4 im Feld stehen, und eine Uebernahme in diesem Augenblick
setzte den Override auf 4 Prozent und wuerfe den gehaltenen Plan weg --
zweimal je Zahl, und einmal davon auf einen Wert, den niemand gemeint hat.
Eine Eingabe ist fertig, wenn der Bediener sie fertig gibt.

Was bleibt, ist die Trennung von Feld und Wert. Sie ist nur nicht mehr an
eine Taste gebunden:

    Die Zeile darunter sagt weiter an, was wirkt. Solange Feld und Wert
    dasselbe sagen, steht dort nur der Wert. Weicht ein Feld ab, weil gerade
    getippt wird, sagt sie das dazu.

    Eine uebernommene Aenderung verwirft den gehaltenen Plan. Die gerechnete
    Bahn traegt Planer und Zeitgesetz in sich; sie danach mit anderen Werten
    abzutakten hiesse, sie waere mit ihnen gerechnet worden. Derselbe Grund
    wie beim geaenderten Sollwert seit 2.7.

    Jedes Feld uebernimmt fuer sich. Mit einer Taste fuer alle drei Werte war
    alles oder nichts richtig; ohne sie ist jede Eingabe eine eigene
    Handlung, und ein unlesbarer Override darf die Fahrart nicht aufhalten.

    Eine unlesbare Eingabe wird nicht uebernommen und nicht weggeraeumt. Das
    Feld bleibt rot stehen, mit dem, was darin steht. Es beim Verlassen still
    auf den alten Wert zurueckzusetzen waere bequemer und genau deshalb
    falsch: der Bediener saehe eine Zahl, die er nicht getippt hat, und
    haette keinen Anhalt, dass seine Eingabe verworfen wurde. Escape holt den
    wirksamen Wert zurueck -- das ist der Rest der alten Verwerfen-Taste.

    Es gibt weiterhin nur einen Wert. Fahrfenster und Parameterfenster zeigen
    dieselbe Groesse; wer in einem uebernimmt, zieht das andere nach,
    ausdruecklich mitsamt der Meldung, dass dort nicht uebernommene Eingaben
    dabei wegfallen.

    Waehrend einer Fahrt wird nichts uebernommen. Die Felder sind dann
    ohnehin gesperrt; die Pruefung steht trotzdem im Uebernehmen selbst, denn
    ein FocusOut kann auch von einem Fenster ausgeloest werden, das sich
    gerade schliesst.

Aus 2.22 unveraendert uebernommen: Groesse und Aufbau des Fahrfensters.

Das Fenster ist seit 2.22 gewachsen: die Hoehe kommt aus dem Bildschirm und
nicht mehr aus einer festen Zahl, die Hinweistexte brechen breiter um. Damit
auf einem kleinen Schirm trotzdem nichts unerreichbar wird, liegt der obere
Teil -- Zielraum und Fahrparameter -- in einem rollbaren Bereich. Der Fuss
mit den vier Fahrtasten rollt bewusst nicht mit: eine Bremse, die man erst
suchen muss, ist keine.

Fahrart und die beiden Overrides stehen seit 2.22 ueberhaupt erst im
Fahrfenster. Bis dahin lagen sie allein im Parameterfenster, unter der
Karteikarte Fahrt. Sachlich war das richtig -- sie gehoeren zu MoveIt und
nicht zum Ziel --, in der Bedienung war es umstaendlich: wer PTP gegen LIN
tauschen wollte, um den Unterschied zu sehen, wechselte dafuer das Fenster
und kam zurueck. Zwei Fenster fuer einen Versuch, und der Zusammenhang
zwischen Einstellung und gefahrener Bahn geht dabei verloren.

Aus 2.21 unveraendert uebernommen: Halte-Taster, Auftragsnummern, Wachhunde.

2.21 legt einen Halte-Taster neben die drei Fahrtasten. Er heisst "FAHREN
(HALTEN)", und er tut alles drei auf einmal: druecken plant und faehrt,
loslassen bremst. Das Vorbild ist der Zustimmungstaster einer Roboterzelle --
die Maschine bewegt sich, solange jemand die Hand am Schalter hat, und keine
Sekunde laenger.

Die drei alten Tasten bleiben stehen. Sie wegzunehmen waere der Verlust der
Aussage von 2.7: zwischen dem Rechnen einer Bahn und dem Bewegen der Maschine
liegt eine bewusste Handlung. Nebeneinander sind beide Wege zu sehen, der
zerlegte und der zusammengefasste, und der Unterschied ist der Lehrstoff.
Beide muenden in denselben _start_playing; einen zweiten Fahrweg gibt es
nicht.

Vier Dinge daran sind keine Nebensache.

    Zwischen Druck und Bewegung liegt eine Antwortzeit. Der Planer braucht
    je nach Auftrag Millisekunden oder Sekunden, und in dieser Zeit kann der
    Taster laengst wieder oben sein. Ein Plan, der danach eintrifft, darf
    nicht mehr fahren. Er darf aber auch nicht einem inzwischen gestellten
    zweiten Auftrag zugerechnet werden -- und genau das tat die Pruefung bis
    2.20, denn sie fragte nur den Zustand: stand die Ablaufsteuerung wieder
    auf PLANEN, sah die alte Antwort aus wie die neue. Seit 2.21 traegt jede
    Anfrage eine Nummer, die Antwort bringt sie zurueck, und was nicht zur
    erwarteten Nummer passt, faellt weg. Die Nummer geht durch die Bruecke
    hindurch bis in die Fehlermeldungen der Frist -- eine Antwort ohne
    Nummer gibt es nicht.

    Ein verlorenes Loslassen ist der gefaehrliche Fall. Fokuswechsel,
    Fenstermanager, ein fremder grab_set: das ButtonRelease kommt nicht an,
    hold_run bleibt stehen, und der Roboter faehrt weiter, obwohl niemand
    mehr drueckt. Tk kann den Zustand der Maustasten nicht abfragen, nur
    Ereignisse melden -- auf ein ausbleibendes Ereignis zu warten ist kein
    Wachhund. Deshalb fragt _hold_watch alle 100 ms das, was abfragbar ist:
    steht der Zeiger noch auf dem Taster, gibt es den Taster noch, ist das
    Fenster noch da. Dazu kommt in _motion_tick die Invariante selbst --
    eine Fahrt aus dem Taster laeuft nur, solange der Taster unten ist.

    Gebremst wird nicht rueckwaerts. Der TrajectoryPlayer kennt brake() und
    keine Umkehrung: die Zeitrate faehrt ueber BRAKE_TIME_S auf null und
    bleibt dort. Ein erneuter Druck waehrend des Bremsens wird deshalb
    abgelehnt und nicht etwa aufgehoben -- die Bahn ist zur Haelfte gefahren,
    ihr Startpunkt gilt nicht mehr, und was danach kommt, ist ein neuer
    Auftrag vom tatsaechlichen Achsstand aus. Der Preis sind die 0.4 s
    Wartezeit nach jedem Loslassen. Die Anzeige sagt sie an, sonst wirkt der
    Taster kaputt.

    Halten heisst nicht weiterfahren. Eine geplante Bahn endet an ihrem
    Ziel, auch wenn der Taster unten bleibt. Das unterscheidet ihn vom
    Joggen, das keine Bahn kennt und faehrt, solange die Taste haelt.

Aus 2.20 unveraendert uebernommen: Drehkonvention, konvention.py, Anzeige.

2.20 macht die Drehkonvention waehlbar. Bis 2.19 zeigte die Oberflaeche
Winkel ausschliesslich als Roll, Pitch, Yaw -- Kardanwinkel in der Reihenfolge
ZYX, dieselbe Konvention, die die URDF im Attribut rpy fuehrt. Das war nie
falsch, aber es war auch nie ausgesprochen. Wer die drei Zahlen abliest, muss
wissen, nach welcher Vorschrift sie gebildet wurden; ohne diese Angabe
beschreiben sie keine Lage.

Drei Konventionen stehen zur Wahl, umgeschaltet wird im Parameterfenster
unter "Anzeige":

    ZYX   Kardanwinkel, R = Rz(A) Ry(B) Rx(C).  Was die URDF fuehrt und was
          KUKA nach DIN 66217 A, B, C nennt. Voreinstellung.
    ZYZ   Eulerwinkel,  R = Rz(A) Ry(B) Rz(C)
    ZXZ   Eulerwinkel,  R = Rz(A) Rx(B) Rz(C)

Die Winkel heissen jetzt in allen drei Faellen A, B und C. Der Name sagt die
Reihenfolge -- A ist die erste Drehung, C die letzte --, die Konvention sagt
die Achsen. Deshalb muss keine Beschriftung beim Umschalten mitwandern, und
deshalb steht bei ZYX jetzt A vorn und C hinten, wo bis 2.19 Roll vorn stand.
Die Zahlen sind dieselben, die Spalten sind vertauscht.

Was daran nicht bloss Geschmack ist: bei ZYX entartet die Darstellung bei
B = +-90 Grad, bei ZYZ und ZXZ bei B = 0 und B = 180 Grad. Der zweite Fall
ist die uebliche Grundstellung. Dort sind A und C nicht mehr einzeln bestimmt,
die Anzeige springt, und zwei Jogtasten bewegen dasselbe. Der Roboter steht
dabei voellig unauffaellig -- es ist eine Eigenschaft der Zahlen und nicht der
Maschine. Die TCP-Zeile sagt es an, sobald B nahe an eine solche Stellung
kommt, und faerbt sich. Damit steht die Darstellungssingularitaet neben der
kinematischen, die _jog_report seit 2.13 als Verstaerkung meldet, und beide
sind als verschiedene Dinge zu sehen.

Die Rechnung selbst liegt in konvention.py, einem eigenen Modul ohne Tkinter
und ohne ROS. Es ist gegen das Produkt dreier Elementardrehungen geprueft,
nicht bloss auf Hin und Zurueck: ein Vorzeichenfehler in einer
ausmultiplizierten Formel faellt beim Rueckrechnen nicht auf, weil die
Umkehrung denselben Fehler noch einmal macht.

Drei Dinge bleiben ausdruecklich bei ZYX:

    Die URDF. Ihr Attribut rpy ist als feste XYZ-Drehung definiert, eine
    andere Konvention kann dort nicht stehen. Geschrieben wird weiterhin
    ueber to_xyz_rpy, und die Formel dafuer ist zeichengleich zu der aus
    2.19 -- gleiche Summationsreihenfolge, gleiche Rundung, alte Dateien
    lesen sich bitgleich wie vorher.

    Die Winkelfelder im Bearbeiten-Fenster, im Anhaengen-Fenster und unter
    Frames und Werkzeuge. Sie zeigen kein Ergebnis einer Rechnung, sondern
    den Inhalt eines Attributs. Wer sie in ZYZ zeigte, muesste beim
    Uebernehmen zurueckrechnen, und die Datei bekaeme eine Aenderung, die
    niemand getippt hat. Die Beschriftung nennt die Konvention jetzt.

    Eine laufende Aufzeichnung. Der Rekorder haelt die Konvention fest, mit
    der er begonnen hat, und schreibt sie in den Kopf der CSV. Umschalten
    waehrend einer Aufzeichnung wird abgelehnt statt die Punkte zu verwerfen:
    sonst haetten Zeilen oberhalb und unterhalb des Umschaltpunktes dieselben
    Spalten und verschiedene Bedeutung, ohne dass die Datei es zeigt.

2.16 legt Frames und Werkzeuge an. Ein eigenes Fenster im Bearbeiten-Menue
schreibt sie unmittelbar in die URDF, nicht in eine Datei daneben.

Der Grund fuer die URDF und gegen einen eigenen Katalog: was dort steht, sehen
robot_state_publisher, RViz und move_group. Ein Katalog daneben waere eine
zweite Wahrheit ueber denselben Sachverhalt, und die beiden driften
auseinander, sobald jemand nur eine von beiden pflegt. Der Preis ist der
Neustart von move_group nach jeder Aenderung -- dieselbe Regel wie bisher,
ohne Ausnahme fuer den vermeintlich harmlosen Fall.

Zwei Rollen, die in URDF beide ein <joint type="fixed"> sind und sich allein
durch ihren Platz in der Kette unterscheiden:

    Werkzeug (TCP)    haengt als Blatt unter einem Link. Steht rechts in der
                      Kette, faehrt also mit dem Flansch mit und geht in die
                      Ableitung ein: v_TCP = v_Flansch + omega x r.

    Bezug (Basis)     kommt ueber die bisherige Wurzel. Steht links, haengt
                      nicht von q ab, bewegt den Roboter nicht. Er aendert,
                      worauf sich alle Zahlen beziehen.

Fuer die zweite Rolle gibt es die neue Funktion insert_above_root. insert_limb
deckt sie nicht ab: das braucht ein Gelenk, unter das eingehaengt wird, und
ueber der Wurzel gibt es keines.

Vier weitere Aenderungen haengen daran:

    _tcp_link fragt zuerst moveit_config.tip und faellt erst dann auf das
    einzige Blatt zurueck. Bis 2.15 gab es nur den zweiten Weg. Der genuegt
    fuer genau ein Werkzeug; beim zweiten -- Greifer und Kamera -- gibt es
    zwei Blaetter, die Ableitung hat keine Grundlage mehr und liefert None.
    Die TCP-Zeile blieb dann leer, ohne dass etwas kaputt war. Die Wahl steht
    bewusst in demselben Feld, das build_srdf in die Kette schreibt und das
    /compute_ik als ik_link_name bekommt: ein Begriff, ein Feld.

    Die TCP-Zeile nennt ihren Bezug. Sie zeigt Zahlen aus
    forward_kinematics, und die rechnet gegen die Wurzel. Sobald ein
    Basisframe darueber kommt, ist die Wurzel eine andere und dieselbe Zeile
    meint etwas anderes -- ohne Fehler, ohne Meldung. Jetzt steht der Bezug
    daneben, wie bei _drive_ik_hint im Fahrfenster seit 2.13.

    Ein Wechsel des aktiven TCP verwirft eine laufende Aufzeichnung.
    _record_frames stellt den TCP ans Ende, und write_csv schreibt den
    letzten Namen als #tcp in den Kopf. Ein Wechsel mitten in der Aufnahme
    haette die Bedeutung der Spalten verschoben, ohne die Spaltenzahl
    anzutasten -- der Kopf haette dann etwas anderes behauptet als die
    Zeilen darunter. _rebuild_from_root faengt das nur bei Modellaenderungen
    ab; die Wahl des TCP ist keine.

    Was die Modellsignatur nicht leistet: sie ist eine Achstabelle. Ein
    fester Frame aendert keine Achse, also bleibt sie gueltig und eine
    gespeicherte Punktetabelle laedt anstandslos -- mit Punkten, die gegen
    einen anderen TCP gelehrt wurden. Die Signatur schuetzt den Achssatz,
    nicht den Bezug. Ein Punkt muesste seinen TCP und seine Basis selbst
    mitfuehren; das gehoert in die Punktetabelle und nicht hierher, deshalb
    steht es hier nur als Vermerk.

2.15 repariert den Bahnrekorder. Der Knopf Aufzeichnen tat nichts sichtbares,
weil der Aufruf dahinter mit einer Ausnahme endete:

    PathRecorder.start() missing 2 required positional arguments:
    'frame_names' and 'chain'

Das ist kein Tippfehler, sondern eine Naht. Die Klasse PathRecorder wurde im
Zweig 1.x erweitert: sie schreibt seither nicht nur den TCP und die Achsen,
sondern zu jedem Punkt alle Frames der Kette, und in den Kopf der CSV den
Aufbau des Modells samt Kopplungen. Die Aufrufe in der Oberflaeche blieben
beim alten Stand mit nur einem Argument. Beim Zusammenfuehren in den Zweig
2.x ist die neue Klasse mitgekommen, die passenden Aufrufer nicht.

Sichtbar war davon nichts als ein Knopf, der nicht reagiert. Ein Rueckruf von
Tk faengt die Ausnahme ab und schreibt sie nach stderr -- unter einem
Launchfile liest die niemand. Dasselbe Muster wie bei 2.11, nur eine Etage
hoeher: nicht der Executor verschluckt den Fehler, sondern die Tk-Schleife.

Vier Aenderungen, drei davon Naht, eine echter Fehler:

    _toggle_record ruft start mit allen Angaben auf: Achsspalten, Framenamen,
    Kette, Kopplungen, Roboternamen.

    _record_tick reicht die Frames mit. Ohne sie stand im Spaltenkopf je Frame
    ein Block, in den Datenzeilen aber nichts -- die CSV waere geschrieben
    worden und beim Einlesen an der Spaltenzahl gescheitert. Die
    Vorwaertskinematik wird dabei einmal je Takt gerechnet und fuer TCP und
    Frames gemeinsam benutzt, nicht zweimal.

    Die Ruhepruefung auf die Achsen rechnete in verschiedenen Einheiten. Die
    Schwelle lag in rad, die Werte kommen in grad beziehungsweise Meter aus
    _joint_snapshot, weil sie so in die CSV gehoeren. Aus 0.5 grad wurden so
    faktisch 0.0087 grad, und der Filter, der stehende Punkte fernhalten soll,
    liess praktisch jeden durch: der Ringpuffer war nach gut einer Minute
    Schieben voll. Der Rekorder bekommt die Schwelle jetzt je Spalte in der
    Einheit dieser Spalte.

    _drop_record wurde nirgends gerufen. Nach einem Modellwechsel blieben die
    alten Punkte liegen. Jetzt raeumen _load und _rebuild_from_root auf, und
    zwar mit reset statt clear -- bei einem anderen Modell passt nicht nur der
    Inhalt nicht mehr, sondern auch die Spaltenaufteilung.

Aus 2.13 unveraendert uebernommen: der Zielraum im Fahrfenster.

2.13 gibt dem Fahrfenster den Zielraum zurueck. Bis 2.6 gab es eine Auswahl
zwischen Achsraum und kartesischem Ziel; sie fiel weg, als das Ziel aus den
Sollwertfeldern in ein eigenes Fenster wanderte, und seither kannte das
Fahrfenster nur Achswerte. Der kartesische Reiter im Hauptfenster stand
daneben und sagte, die Rueckwaertskinematik sei nicht hinterlegt.

Sie ist es. Seit 2.4 laeuft ein move_group mit, und der bedient neben
/plan_kinematic_path auch /compute_ik -- der Probe-Takt fragt beide seit 2.5
ab, benutzt wurde bisher nur der erste. Diese Fassung benutzt den zweiten.

Oben im Fahrfenster stehen zwei Knoepfe:

    Achsraum          sieben Zahlen, je eine Achse, wie bisher
    Kartesisch (IK)   sechs Zahlen, Lage und Drehung des Tip-Links

Der Weg dahinter ist bewusst kein zweiter Weg. Ein kartesisches Ziel wird
zuerst von /compute_ik in Achswerte aufgeloest; diese Werte werden in die
Achszielfelder geschrieben und von dort weiter behandelt wie ein getipptes
Ziel -- dieselbe Begrenzung auf die URDF-Anschlaege, dieselbe Faerbung,
derselbe MotionPlanRequest. Es gibt weiterhin genau einen Planungspfad.

Das ist nicht nur bequem. Wer die Loesung sieht, bevor gefahren wird, sieht
die Sache, um die es bei sieben Achsen geht: dieselbe Lage laesst sich in
unendlich vielen Achsstellungen erreichen, und welche davon herauskommt,
entscheidet der Startwert. Deshalb geht der aktuelle Achsstand als Startwert
mit in die Anfrage. Zwei Aufrufe mit derselben Lage und verschiedenem Start
liefern verschiedene Loesungen, und das laesst sich hier in einem Zug zeigen.

Ein eigener Zustand kommt dazu:

    BEREIT  ->  IK  ->  PLANEN  ->  FAHREN  ->  BREMSEN  ->  BEREIT

IK und PLANEN getrennt zu fuehren kostet drei Zeilen und beantwortet die
haeufigste Frage von selbst: "keine IK-Loesung" heisst, die Lage ist nicht
erreichbar; "kein Plan" heisst, sie ist erreichbar, aber der Weg dorthin
nicht. Wer beides in einem Zustand fuehrt, sucht den Unterschied im
Fehlercode statt in der Anzeige.

Nebenher wird der kartesische Reiter im Hauptfenster ehrlich: er stellt
weiterhin keine Achse -- das ist der Weg ohne MoveIt und bleibt es --, sagt
das aber und reicht seine sechs Werte auf Tastendruck an das Fahrfenster
weiter.

Aus 2.12 unveraendert uebernommen: die Korrektur an joint_names.

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
    ./urdf_gui2_13.py                     # Standardpfad aus URDF_PATH
    ./urdf_gui2_13.py ~/Display/bmKinematic.urdf
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

# Liegt neben dieser Datei. Haelt die drei Drehkonventionen und ist ohne
# Tkinter und ohne ROS zu pruefen -- test_konvention.py laeuft eigenstaendig.
import konvention

# Liegt ebenfalls daneben. Punktfolgen, Modalzustand und die Pruefung der
# Ueberschleifradien. Kein Tkinter, kein ROS, kein NumPy -- test_sequenz.py
# laeuft ohne Bildschirm und ohne gesourctes Environment.
import sequenz

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

RECORD_MIN_JOINT_DEG = 0.5

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
        """Konvention wechseln. Gibt (geaendert, grund) zurueck."""
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

# ---------- Frames und Werkzeuge ----------
# Die beiden Rollen, die ein fester Frame haben kann. In der URDF sind beide
# ein <joint type="fixed">; unterschieden werden sie allein durch die Stelle
# in der Kette. Deshalb steht die Rolle hier als Text und nicht als Typ: sie
# waehlt aus, welche Funktion schreibt, und ist danach nicht mehr abfragbar.
ROLE_TOOL = "Werkzeug (TCP)"
ROLE_BASE = "Bezug (Basis)"

# Eintrag in der TCP-Auswahl fuer "aus dem Modell ableiten". Leer waere
# zweideutig -- ein leeres Feld sieht aus wie ein vergessenes Feld.
TIP_AUTO = "-- einziges Blatt --"

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


def xml_root_links(root):
    """Wurzel-Links unmittelbar aus dem XML-Baum.

    Gegenstueck zu root_links, das auf den geparsten Listen arbeitet. Innerhalb
    einer mutate-Funktion liegt nur der Entwurf als XML vor; die geparste
    Fassung entsteht erst danach in _rebuild_from_root. Ohne diesen Helfer
    muesste man dort zwischendurch parsen, und der Entwurf ist genau in diesem
    Moment moeglicherweise noch ungueltig.
    """
    children = set()
    for joint in root.findall("joint"):
        child = joint.find("child")
        if child is not None and child.get("link"):
            children.add(child.get("link"))
    return [link.get("name") for link in root.findall("link")
            if link.get("name") not in children]


def insert_above_root(root, link_name, joint_name, xyz, rpy):
    """Neuen Link ueber die bisherige Wurzel setzen: der Basis-Offset.

    Aus   R --k--> ...   wird   N --joint_name--> R --k--> ...

    Der Gegenpol zu append_limb. Dort entsteht ein Blatt am unteren Ende der
    Kette, hier eine neue Wurzel am oberen. Beide schreiben ein
    <joint type="fixed">; welche Bedeutung es hat, entscheidet allein die
    Stelle.

    insert_limb deckt den Fall nicht ab, obwohl er verwandt aussieht: es
    braucht `below`, also ein bestehendes Gelenk, dessen Elternteil
    umgeschrieben wird. Ueber der Wurzel fuehrt kein Gelenk hin, es gaebe
    nichts umzuschreiben.

    Kein keep_pose. Die Frage stellt sich nur, wenn oberhalb etwas steht, zu
    dem eine Lage erhalten bleiben koennte; die Wurzel hat definitionsgemaess
    nichts ueber sich. Der ganze Baum haengt danach mit T_neu unter N, und
    genau das ist gemeint: der Roboter steht irgendwo in der Welt.

    Was sich dadurch aendert, ist nicht die Mechanik, sondern der Bezug.
    forward_kinematics rechnet gegen die Wurzel, und die heisst ab jetzt
    anders. Jede Zahl in der Frame-Tabelle ist danach gegen N angeschrieben
    statt gegen R -- fehlerfrei, unauffaellig und trotzdem eine andere
    Aussage. Deshalb nennt die TCP-Zeile ihren Bezug ausdruecklich.
    """
    tops = xml_root_links(root)
    if len(tops) != 1:
        raise ValueError(
            "Ein Basisframe braucht genau eine Wurzel. Gefunden: "
            + (", ".join(sorted(tops)) if tops else "keine"))

    link = ET.SubElement(root, "link")
    link.set("name", link_name)

    joint = ET.SubElement(root, "joint")
    joint.set("name", joint_name)
    joint.set("type", "fixed")
    ET.SubElement(joint, "parent").set("link", link_name)
    ET.SubElement(joint, "child").set("link", tops[0])
    origin = ET.SubElement(joint, "origin")
    origin.set("xyz", fmt_origin(xyz))
    origin.set("rpy", fmt_origin(rpy))
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

# Werte des Planers selbst, nicht des Roboters. Sie landen im Namensraum der
# Pipeline und heissen am Knoten deshalb pilz_industrial_motion_planner.<key>.
#
# Die Abtastzeit ist der Takt, in dem Pilz die kartesische Bahn abtastet und je
# Stuetzpunkt eine IK rechnet. Sie war bis 3.2 nicht erreichbar: der Wert stand
# ausschliesslich als Default-Argument von TrajectoryGenerator::generate() im
# Quellcode, und der einzige Aufrufer uebergab ihn nicht. Erst der Patch
# patch_pilz_sampling_time_1.py macht daraus einen echten ROS-Parameter.
#
# Ohne diesen Patch ist das Feld wirkungslos, nicht schaedlich: move_group
# ignoriert einen unbekannten Parameter im Pipeline-Namensraum. Ob der Patch
# laeuft, sagt die Zeile "PILZ-PATCH aktiv" im move_group-Protokoll.
PLANNER_FIELDS = [
    ("sampling_time", "Abtastzeit", "s", 0.100),
]

# Grenzen des Quellcode-Patches, hier gespiegelt. Die verbindliche Pruefung
# steht in CommandPlanner::initialize() und bricht den Start von move_group ab.
# Diese Kopie dient nur der frueheren Rueckmeldung im Fenster -- weicht sie
# vom Patch ab, gewinnt der Patch.
SAMPLING_TIME_MIN = 1.0e-4
SAMPLING_TIME_MAX = 1.0

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

# Was davon seit 2.22 zusaetzlich im Fahrfenster steht. Bewusst nur diese
# beiden und nicht die ganze Liste: Planungszeit, Toleranzen und
# Stuetzpunktabstand aendert man einmal und laesst sie dann stehen, die
# Overrides aendert man von Versuch zu Versuch. Das Fahrfenster traegt, was
# zum einzelnen Auftrag gehoert, nicht die Einrichtung der Anlage.
DRIVE_PLAN_KEYS = ("velocity_scaling", "acceleration_scaling")


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
        self.planner = {key: default
                        for key, _l, _u, default in PLANNER_FIELDS}
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

        # Sechs Leerzeichen: die Werte gehoeren unter
        # "pilz_industrial_motion_planner:" und heissen am Knoten damit
        # pilz_industrial_motion_planner.<key>. float() ist nicht kosmetisch --
        # "sampling_time: 1" waere in YAML ein Integer und wuerde in rclcpp
        # eine InvalidParameterTypeException ausloesen, "1.0" nicht.
        for key, _label, _unit, _default in PLANNER_FIELDS:
            lines.append(f"      {key}: "
                         f"{yaml_scalar(float(self.planner[key]))}")

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
        # CommandPlanner::initialize() laeuft genau einmal beim Hochfahren und
        # legt die Abtastzeit in den LimitsContainer jedes Loaders. Ein
        # spaeteres Setzen des Parameters aendert nur den Parameterwert, nicht
        # die Abtastung. Deshalb gehoert das Feld in die Neustart-Kennung.
        for key, _l, _u, _d in PLANNER_FIELDS:
            parts.append(f"{key}:{self.planner[key]}")
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

# Takt des Wachhunds am Halte-Taster. Er fragt nicht nach Ereignissen,
# sondern nach dem, was Tk beantworten kann: steht der Zeiger noch auf der
# Taste. 100 ms sind der Ausgleich zwischen zwei Fehlern -- kuerzer belastet
# den GUI-Thread ohne Gewinn, laenger laesst den Roboter nach einem
# verlorenen ButtonRelease sichtbar zu lange weiterfahren. Bei 100 ms liegt
# der schlimmste Fall unter der Bremszeit selbst.
HOLD_WATCH_MS = 100

# Zustaende der Ablaufsteuerung.
STATE_READY = "BEREIT"
# Die Rueckwaertskinematik ist ein eigener Zustand und nicht der Anfang von
# PLANEN. Sie beantwortet eine andere Frage -- "gibt es zu dieser Lage
# ueberhaupt eine Achsstellung" statt "gibt es einen Weg dorthin" --, und sie
# scheitert aus anderen Gruenden. Beides in einem Zustand zu fuehren hiesse,
# den Unterschied im Fehlercode zu verstecken.
STATE_IK = "IK"
STATE_PLANNING = "PLANEN"
# Eine Punktfolge ist ein eigener Zustand und nicht PLANEN. Sie geht an
# einen anderen Dienst, sie scheitert an anderen Dingen -- ueberlappenden
# Radien etwa, die es bei einem einzelnen Ziel gar nicht gibt --, und ihr
# Ergebnis ist eine Liste von Teilbahnen statt einer Bahn. In einem
# Zustand gefuehrt waere der Unterschied nur noch am Fehlertext zu sehen.
# Die Aufloesung der Punkte vor dem Auftrag. Ein eigener Zustand und nicht
# STATE_IK: hier laufen n Anfragen nacheinander, und die Antwort auf die
# dritte darf nicht in denselben Zweig fallen wie die eine Anfrage des
# Fahrfensters. Wer sie zusammenlegte, muesste an jeder Stelle in
# _on_ik_result zusaetzlich fragen, wofuer die Antwort gedacht war.
STATE_SEQ_IK = "FOLGE IK"
STATE_SEQUENCE = "FOLGE"
STATE_MOVING = "FAHREN"
STATE_BRAKING = "BREMSEN"
STATE_ERROR = "FEHLER"

# Zustaende, in denen nichts veraendert werden darf.
STATES_BUSY = (STATE_IK, STATE_PLANNING, STATE_SEQ_IK, STATE_SEQUENCE,
               STATE_MOVING, STATE_BRAKING)

# Zielraum im Fahrfenster. Zwei Zeichenketten statt eines Wahrheitswerts,
# damit in Protokoll und Auswahl dasselbe Wort steht.
SPACE_JOINT = "joint"
SPACE_CART = "cart"

# Zuschlag auf die Zeitgrenze des Solvers, wie PLAN_TIMEOUT_MARGIN_S beim
# Planen. Deutlich kleiner: ein IK-Aufruf ist eine Rechnung von Millisekunden,
# und wenn nach Sekunden nichts kommt, liegt es nicht am Rechnen.
IK_TIMEOUT_MARGIN_S = 5.0

# ---------- Kartesisches Joggen ----------
# Ein Klick geht um eine feste Stufe, Halten faehrt mit fester Geschwindigkeit.
# Zwei Groessen, nicht eine: die Stufe soll gross genug sein, dass ein Klick
# etwas bewirkt, und die Geschwindigkeit klein genug, dass man beim Halten
# noch mitliest.
JOG_STEP_M = 0.01                                      # ein Klick
JOG_STEP_DEG = 0.1
JOG_VEL_M_S = 0.02                                     # gehalten
JOG_VEL_DEG_S = 0.2

# Ab hier wird aus dem Klick ein Halten. Kuerzer, und ein normaler Klick
# faehrt ungewollt weiter; laenger, und das Halten fuehlt sich tot an.
JOG_HOLD_DELAY_MS = 300

# Obergrenze fuer einen einzelnen Zeitschritt. Bleibt die Antwort einmal
# laenger aus -- Solver am Anschlag, Rechner unter Last --, waere der
# nachgeholte Weg sonst ein Sprung. Lieber langsamer als gerissen.
JOG_MAX_DT_S = 0.25

# Warnschwellen fuer die Achsauslastung, als Anteil der Achsgeschwindigkeit,
# die bei der eingestellten Jog-Geschwindigkeit gefordert waere.
#
# Zwei Werte und nicht einer: mit einer einzigen Schwelle kippte die Warnung
# bei jedem Schritt hin und her, sobald die Auslastung um sie herum pendelt --
# und pendeln tut sie, weil jeder Schritt eine eigene IK-Loesung ist. Das ist
# dieselbe Hysterese wie bei einem Schmitt-Trigger: eingeschaltet wird oben,
# ausgeschaltet erst deutlich darunter. Der Abstand ist der ganze Zweck.
# Takt der Achs-Dauerfahrt. Sie braucht keine IK und damit keine Antwort, auf
# die sie warten muesste -- deshalb eine Uhr, waehrend das kartesische Joggen
# an der Antwort haengt. Zwei Schleifen, weil es zwei verschiedene Aufgaben
# sind: die eine fragt einen Dienst, die andere rechnet nichts.
JOG_TICK_MS = 50

JOG_WARN_SHARE = 0.75                                  # ab hier warnen
JOG_CLEAR_SHARE = 0.65                                 # erst hier wieder still

# moveit_msgs/MoveItErrorCodes. Die nackte Zahl hilft niemandem, und der
# Umgang mit diesen Faellen ist ein guter Teil des Versuchs -- deshalb steht
# neben der Uebersetzung auch, woran es meistens liegt.
MOVEIT_ERRORS = {
    1: ("Erfolg", ""),
    # 0 ist UNDEFINED, der Vorbelegungswert des Feldes: niemand hat
    # geantwortet, der Auftrag ist bei keinem Planer angekommen.
    0: ("Keine Antwort", "Der Auftrag ist bei keinem Planer angekommen. "
                         "Pipeline geladen? Pilz installiert?"),
    # 99999 ist FAILURE, der Sammelwert -- und etwas ganz anderes als 0.
    # Hier hat sehr wohl jemand geantwortet: ein Planer hat gearbeitet und
    # eine Ausnahme geworfen, fuer die es keinen eigenen Code gibt. Das
    # Ueberschleifen benutzt ihn zum Beispiel, wenn die Abtastraster zweier
    # Teilbahnen nicht zusammenpassen. Bis 3.1 stand hier derselbe Text wie
    # bei 0, und der schickte die Suche in die falsche Richtung: nicht die
    # Pipeline fehlt, sondern der Grund steht in der Ausgabe von
    # move_group -- deshalb wird sie in diesem Fall selbst geholt.
    99999: ("Fehlgeschlagen", "Sammelfehler ohne eigenen Code. Ein Planer hat "
                              "gearbeitet und aufgegeben; der Grund steht nur "
                              "in der Ausgabe von move_group."),
    999999: ("Fehlgeschlagen", "Sammelfehler ohne eigenen Code. Der Grund "
                               "steht nur in der Ausgabe von move_group."),
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


# Ab dieser Abweichung zwischen Achsstand und erstem Stuetzpunkt wird eine
# geplante Folge nicht gefahren. 1e-4 rad sind 0.0057 grad -- weit unter
# allem, was eine Achse aufloest, und weit ueber dem Rundungsrest, den der
# Weg durch ROS-Nachrichten und zurueck hinterlaesst.
FOLGE_START_TOL = 1e-4


def bahn_startabweichung(punkte, stand):
    """Wie weit steht der Roboter vom ersten Stuetzpunkt einer Bahn weg?

    punkte: [(zeit, {name: pos}, {name: vel}), ...] wie beim
    TrajectoryPlayer. stand: {name: pos}, der Achsstand.

    Liefert (name, betrag, fehlend): die Achse mit der groessten
    Abweichung, deren Betrag, und die Namen, die die Bahn traegt und der
    Stand nicht kennt. Ohne Punkte (None, 0.0, ()).

    Bewusst ohne Tk und ohne ROS und bewusst ohne Entscheidung: die
    Funktion vergleicht und urteilt nicht. Ob 1e-5 noch geht und 1e-3
    nicht mehr, ist eine Festlegung der Oberflaeche und steht in
    FOLGE_START_TOL -- hier stuende sie an der falschen Stelle, und beim
    Pruefen muesste man sie mitpruefen.

    Fehlende Achsen werden nicht als Abweichung null gezaehlt. Eine Bahn
    mit einer Achse, die der Stand nicht kennt, gehoert zu einem anderen
    Modell; das ist kein kleiner Unterschied, sondern ein anderer Roboter.
    """
    if not punkte:
        return (None, 0.0, ())
    soll = punkte[0][1]
    fehlend = tuple(sorted(name for name in soll if name not in stand))
    schlimmste = None
    betrag = 0.0
    for name, wert in soll.items():
        if name not in stand:
            continue
        ab = abs(float(stand[name]) - float(wert))
        if schlimmste is None or ab > betrag:
            schlimmste = name
            betrag = ab
    return (schlimmste, betrag, fehlend)


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
                 min_angle=math.radians(RECORD_MIN_ANGLE_DEG),
                 min_joint=math.radians(RECORD_MIN_JOINT_DEG)):
        self.points = collections.deque(maxlen=max_points)
        self.min_dist = min_dist
        self.min_angle = min_angle
        self.min_joint = min_joint
        self.joint_names = []                          # Reihenfolge der Achsspalten
        # Ruheschwelle je Achsspalte, in der Einheit dieser Spalte. Eine
        # gemeinsame Schwelle gibt es nicht: die Werte kommen so herein, wie
        # sie in die CSV gehoeren, und das sind grad bei einem Drehgelenk und
        # Meter bei einem Schubgelenk. min_joint bleibt als Rueckfall fuer den
        # Aufrufer, der nichts angibt.
        self.joint_tolerances = []
        self.frame_names = []                          # Reihenfolge der Frameblocks
        self.chain = []                                # [(kind, elter, gelenk)]
        self.couplings = []                            # [(gelenk, fuehrung, m, b)]
        self.robot_name = ""
        self.active = False
        # Die Konvention, in der die abgelegten Winkel gemeint sind. Sie
        # gehoert zu den Punkten und nicht zur Anzeige: eine Datei muss
        # deutbar bleiben, wenn die Oberflaeche laengst anders eingestellt
        # ist. Gesetzt wird sie beim Start und danach nicht mehr -- das ist
        # der Grund, aus dem das Umschalten waehrend einer Aufzeichnung
        # abgelehnt wird.
        self.konvention = konvention.STANDARD
        self.start_time = None                         # time.monotonic()
        self.length = 0.0                              # gefahrener Weg des TCP
        self.overflow = False                          # aeltester Punkt verworfen

    # -- Steuerung ---------------------------------------------------------

    def start(self, joint_names, frame_names, chain,
              couplings=(), robot_name="", joint_tolerances=(),
              drehkonvention=None):
        """Aufzeichnung beginnen oder fortsetzen.

        Wechselt der Achssatz, der Framesatz oder die Drehkonvention, sind
        die alten Punkte nicht mehr spaltenrichtig zu schreiben. Dann wird
        verworfen statt vermischt. Bei der Konvention ist der Grund ein
        anderer als bei den Achsen: die Spaltenzahl bliebe gleich, nur die
        Bedeutung waere ab der Umschaltzeile eine andere -- ein Unterschied,
        den die Datei nicht zeigt und den auch der Leser nicht finden kann.
        """
        joints, frames = list(joint_names), list(frame_names)
        gewaehlt = konvention.pruefe(drehkonvention or self.konvention)
        if (joints != self.joint_names or frames != self.frame_names
                or gewaehlt != self.konvention):
            self.clear()
            self.joint_names, self.frame_names = joints, frames
            self.konvention = gewaehlt
        self.joint_tolerances = list(joint_tolerances)
        self.chain = list(chain)
        self.couplings = list(couplings)
        self.robot_name = robot_name
        self.active = True
        return len(self.points)

    def stop(self):
        self.active = False

    def clear(self):
        """Nur die Punkte. Die Spaltenaufteilung bleibt, damit sich eine
        angehaltene Aufzeichnung fortsetzen laesst."""
        self.points.clear()
        self.start_time = None
        self.length = 0.0
        self.overflow = False

    def reset(self):
        """Punkte und Spaltenaufteilung verwerfen. Fuer den Modellwechsel:
        danach passt weder der Achssatz noch der Framesatz."""
        self.clear()
        self.joint_names = []
        self.joint_tolerances = []
        self.frame_names = []
        self.chain = []
        self.couplings = []
        self.konvention = konvention.STANDARD

    # -- Abtastung ---------------------------------------------------------

    def _tolerance(self, index):
        """Ruheschwelle der Achsspalte `index`, sonst der alte Rueckfallwert."""
        if index < len(self.joint_tolerances):
            return self.joint_tolerances[index]
        return self.min_joint

    def sample(self, xyz, abc, values, frames=(), now=None):
        """Einen Punkt anbieten. Gibt zurueck, ob er abgelegt wurde.

        `frames` ist flach: je Frame sechs Zahlen, x y z A B C, in der
        Reihenfolge von `frame_names`. Die Winkel sind in der Konvention
        gemeint, die `start` festgelegt hat -- der Rekorder rechnet nicht um
        und prueft es auch nicht nach, er kann es nicht.
        """
        if not self.active:
            return False
        now = time.monotonic() if now is None else now
        if self.start_time is None:
            self.start_time = now

        step = 0.0
        if self.points:
            _t, last_xyz, last_abc, last_values, _f = self.points[-1]
            step = math.sqrt(sum((a - b) ** 2 for a, b in zip(xyz, last_xyz)))
            # Die drei Winkel reichen als Ruhekriterium. Nahe der Entartung
            # springen sie, das erzeugt hoechstens einen Punkt zu viel, nie
            # zu wenig -- bei ZYZ und ZXZ ist das haeufiger als bei ZYX, weil
            # die Entartung dort in der Grundstellung liegt.
            turn = max(angle_delta(a, b) for a, b in zip(abc, last_abc))
            # Dritte Bedingung fuer den redundanten Fall: der TCP steht, der
            # Arm stellt sich trotzdem um. Die Schwelle steht in derselben
            # Einheit wie die Spalte, deshalb je Spalte eine eigene.
            moved = any(abs(a - b) >= self._tolerance(index)
                        for index, (a, b) in enumerate(zip(values, last_values)))
            if step < self.min_dist and turn < self.min_angle and not moved:
                return False

        if len(self.points) == self.points.maxlen:
            self.overflow = True
        self.points.append((now - self.start_time, tuple(xyz), tuple(abc),
                            tuple(values), tuple(frames)))
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

    def header(self):
        """Spaltenkopf. Erst der TCP, dann die Achsen, dann je Frame ein Block.

        Die Winkelspalten tragen die Achse im Namen: a_z_deg, b_y_deg,
        c_x_deg bei ZYX. Nicht aus Sorgfalt allein -- eine Datei, deren
        Spalten roll_deg heissen und in ZYZ geschrieben sind, liest sich
        anstandslos und bedeutet etwas anderes. Der Name macht den Fehler
        sichtbar, die Kennung im Kopf macht ihn pruefbar.
        """
        columns = ["t_s", "x_m", "y_m", "z_m"]
        columns += list(konvention.spaltennamen("", self.konvention))
        columns += list(self.joint_names)
        for name in self.frame_names:
            columns += [f"{name}_x_m", f"{name}_y_m", f"{name}_z_m"]
            columns += list(konvention.spaltennamen(f"{name}_", self.konvention))
        return columns

    def write_csv(self, path):
        """Punkte als CSV ablegen. Winkel in grad, Laengen in m.

        Vor dem Spaltenkopf stehen Kommentarzeilen mit Roboternamen, Kette und
        Kopplungen. Sie beginnen mit einem Rautenzeichen, halten aber dasselbe
        Trennzeichen ein und sind mit demselben Leser zu verarbeiten. So wandert
        der Aufbau des Modells mit der Datei mit und geht beim Kopieren nicht
        verloren, wie es bei einer Begleitdatei passieren wuerde.

        Die Kopplungen gehoeren dazu, weil ein gekoppeltes Gelenk in den Spalten
        wie eine freie Achse aussieht. Ohne den Hinweis waere spaeter nicht mehr
        zu erkennen, dass es einem anderen folgt.
        """
        with open(path, "w", newline="") as handle:
            writer = csv.writer(handle, delimiter=";")
            writer.writerow(["#roboter", self.robot_name])
            writer.writerow(["#erzeugt", datetime.now().strftime("%Y-%m-%d %H:%M:%S")])
            writer.writerow(["#rate_hz", f"{RECORD_RATE_HZ:g}"])
            # Ohne diese Zeile waeren die drei Winkelspalten nicht deutbar.
            # Sie steht vor dem Spaltenkopf, damit ein Leser sie kennt,
            # bevor er die Spalten aufteilt.
            writer.writerow(["#" + konvention.KENNUNG_SCHLUESSEL,
                             self.konvention])
            writer.writerow(["#tcp", self.frame_names[-1] if self.frame_names else ""])
            for child, parent, joint in self.chain:
                writer.writerow(["#kette", child, parent, joint])
            for name, source, multiplier, offset in self.couplings:
                writer.writerow(["#kopplung", name, source,
                                 f"{multiplier:.6g}", f"{offset:.6g}"])
            writer.writerow(self.header())

            for stamp, xyz, abc, values, frames in self.points:
                row = [f"{stamp:.4f}"]
                row += [f"{v:.6f}" for v in xyz]
                row += [f"{math.degrees(v):.4f}" for v in abc]
                row += [f"{v:.6f}" for v in values]
                for index in range(0, len(frames), 6):
                    block = frames[index:index + 6]
                    row += [f"{v:.6f}" for v in block[:3]]
                    row += [f"{math.degrees(v):.4f}" for v in block[3:]]
                writer.writerow(row)
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
        # Eigene Frage, eigene Antwort: die Sequence-Capability ist ein
        # Plugin, das move_group nur laedt, wenn es in den Parametern
        # steht. Sie fehlt genau dann, wenn jemand move_group von aussen
        # startet -- und dann meldet moveit_ready trotzdem bereit.
        self.sequence_ready = False

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
        self._ik_client = None
        self._sequence_client = None
        # (Art, Future, Ablauf, Frist). Die Art steht seit 2.13 mit dabei,
        # weil zwei Dienste dieselbe Stelle benutzen: eine abgelaufene
        # IK-Anfrage muss als "ik" zurueckkommen und nicht als "plan", sonst
        # wartet die Ablaufsteuerung auf eine Antwort, die sie unter falschem
        # Namen bereits bekommen hat.
        self._pending = None
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

    def request_ik(self, payload):
        """Aus dem GUI-Thread. Wie request_plan, nur fuer /compute_ik.

        Auch hier enthaelt payload nur Zahlen, Zeichenketten und Tupel. Die
        PoseStamped entsteht drueben; der GUI-Thread rechnet die Drehung
        vorher in ein Quaternion um, weil das reine Mathematik ist und in
        to_quaternion schon steht.
        """
        self._jobs.append(("ik", dict(payload)))

    def request_sequence(self, payload):
        """Aus dem GUI-Thread. Wie request_plan, nur fuer eine ganze Folge.

        payload traegt unter items eine Liste von Auftraegen, jeder mit
        blend_radius. Gebaut hat sie sequenz.Folge.nutzlast, geprueft
        ebenfalls -- hier drin wird nichts mehr nachgerechnet.
        """
        self._jobs.append(("sequence", dict(payload)))

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
        sender = {"plan": self._send_plan, "ik": self._send_ik,
                  "sequence": self._send_sequence}.get(kind)
        if sender is None:
            return
        # Die Nummer des Auftrags reist mit. Sie ist der einzige Weg, auf dem
        # der GUI-Thread eine verspaetete Antwort von einer erwarteten
        # unterscheiden kann -- am Inhalt sind sie nicht zu trennen.
        tag = payload.get("tag")
        try:
            sender(payload)
        except Exception as exc:                       # nichts darf den Takt toeten
            self._pending = None
            self.results.append((kind, {"tag": tag,
                                        "error": f"Auftrag fehlgeschlagen: {exc}"}))

    def _drop_client(self, reason):
        """Client wegwerfen und eine offene Anfrage als Fehler abschliessen."""
        if self._pending is not None:
            kind, future, tag = (self._pending[0], self._pending[1],
                                 self._pending[4])
            self._pending = None
            try:
                future.cancel()
            except Exception:
                pass
            self.results.append((kind, {"tag": tag,
                                        "error": f"Anfrage verworfen: {reason}"}))
        for name in ("_plan_client", "_ik_client", "_sequence_client"):
            client = getattr(self, name)
            if client is None:
                continue
            try:
                self.node.destroy_client(client)
            except Exception:
                pass
            setattr(self, name, None)

    def _check_pending(self):
        """Frist einer offenen Anfrage pruefen. Laeuft im Auftragstakt mit.

        Der Rueckweg in die GUI fuehrt allein ueber den Callback des Futures.
        Kommt keine Antwort, wird er nie gerufen, und die Ablaufsteuerung
        bleibt in PLANEN stehen. Deshalb endet hier jede Anfrage, spaetestens
        nach der Frist, die der Auftrag mitgebracht hat.
        """
        if self._pending is None:
            return
        kind, future, deadline, timeout, tag = self._pending
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
        dienst = {"ik": "/compute_ik",
                  "sequence": "/plan_sequence_path"}.get(
                      kind, "/plan_kinematic_path")
        # Auch die Fristmeldung traegt die Nummer. Ohne sie waere sie die
        # eine Antwort, die der GUI-Thread nicht zuordnen kann -- und das
        # ist ausgerechnet die, nach der er am ehesten neu fragt. Die
        # abgebrochene Anfrage kann danach trotzdem noch eintreffen, wenn
        # cancel() zu spaet kam: dann tragen beide dieselbe Nummer, und die
        # zweite faellt weg, weil job_tag nach der ersten schon None ist.
        self.results.append((kind, {
            "tag": tag,
            "error": f"{dienst} antwortet seit {timeout:.0f} s nicht. "
                     "Laeuft ein zweiter move_group?"}))

    def _send_plan(self, payload):
        from moveit_msgs.srv import GetMotionPlan
        from moveit_msgs.msg import Constraints, JointConstraint

        tag = payload.get("tag")
        if self._plan_client is None:
            self._plan_client = self.node.create_client(
                GetMotionPlan, "/plan_kinematic_path")
        if not self._plan_client.service_is_ready():
            self.results.append(("plan", {
                "tag": tag,
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
        self._pending = ("plan", future, time.monotonic() + timeout, timeout,
                         tag)
        # Die Nummer wird an den Rueckruf gebunden und nicht aus _pending
        # gelesen: _read_plan setzt _pending als Erstes auf None, und bis der
        # Rueckruf laeuft, kann laengst ein zweiter Auftrag darin stehen.
        future.add_done_callback(
            lambda done, t=tag: self._on_plan_done(done, t))

    def _on_plan_done(self, future, tag=None):
        """Rueckruf aus rclpy. Faengt alles, sonst faellt es in den Executor."""
        self._guard("Planantwort", lambda: self._read_plan(future, tag))

    def _read_plan(self, future, tag=None):
        """Antwort in reine Zahlen umsetzen und dem GUI-Thread hinlegen."""
        self._pending = None
        try:
            response = future.result()
        except Exception as exc:
            self.results.append(("plan", {"tag": tag, "error": str(exc)}))
            return

        result = response.motion_plan_response
        code = int(result.error_code.val)
        payload = {"tag": tag, "code": code, "points": []}
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

    # -- Punktfolgen -------------------------------------------------------

    def _send_sequence(self, payload):
        """Eine ganze Folge an /plan_sequence_path. Laeuft im Executor-Thread.

        Der Dienst plant und faehrt nicht -- er gibt die Bahnen zurueck. Das
        passt zu dieser Oberflaeche, die ohnehin selbst abtastet; die
        Aktionsvariante MoveGroupSequenceAction wuerde ausfuehren wollen und
        dafuer einen Controller brauchen, den es hier nicht gibt.

        Zwei Regeln des Planers stecken im Aufbau, und beide sind nicht zu
        umgehen:

            Nur der erste Auftrag traegt einen Startzustand. Jede weitere
            Teilbahn beginnt am vorigen Ziel. Ein zweiter Startzustand ist
            kein zusaetzlicher Hinweis, sondern ein Widerspruch. Welcher
            Posten ihn traegt, hat sequenz.Folge.nutzlast entschieden; hier
            wird nur genommen, was dasteht.

            Scheitert ein einziger Auftrag, wird keiner ausgefuehrt. Die
            Folge ist ganz oder gar nicht -- das ist der Grund, aus dem die
            Pruefung vorher so ausfuehrlich ist.

        Ein Ziel kommt auf zwei Wegen herein. Stehen Achswerte da, wird eine
        JointConstraint gebaut wie beim einzelnen Auftrag. Steht eine Lage
        da, wird sie zu einer Kugel um den Ort und einer Drehbedingung -- und
        move_group rechnet die Rueckwaertskinematik selbst. Bei sieben Achsen
        heisst das, dass der Konfigurationszweig nicht gesichert ist. Der
        Fall steht deshalb ausdruecklich hier und nicht als stiller Zweig.
        """
        from moveit_msgs.srv import GetMotionSequence
        from moveit_msgs.msg import (Constraints, JointConstraint,
                                     MotionSequenceItem, OrientationConstraint,
                                     PositionConstraint)
        from geometry_msgs.msg import Pose
        from shape_msgs.msg import SolidPrimitive

        tag = payload.get("tag")
        if self._sequence_client is None:
            self._sequence_client = self.node.create_client(
                GetMotionSequence, "/plan_sequence_path")
        if not self._sequence_client.service_is_ready():
            self.results.append(("sequence", {
                "tag": tag,
                "error": "/plan_sequence_path antwortet nicht. Traegt "
                         "move_group die Sequence-Capability?"}))
            return

        request = GetMotionSequence.Request()
        for eintrag in payload["items"]:
            item = MotionSequenceItem()
            item.blend_radius = float(eintrag["blend_radius"])

            plan = item.req
            plan.group_name = eintrag["group"]
            plan.planner_id = eintrag["planner_id"]
            plan.pipeline_id = "pilz_industrial_motion_planner"
            plan.num_planning_attempts = int(eintrag["attempts"])
            plan.allowed_planning_time = float(eintrag["planning_time"])
            plan.max_velocity_scaling_factor = float(
                eintrag["velocity_scaling"])
            plan.max_acceleration_scaling_factor = float(
                eintrag["acceleration_scaling"])

            start = eintrag.get("start")
            if start is not None:
                plan.start_state.joint_state.name = list(start)
                plan.start_state.joint_state.position = [
                    float(value) for value in start.values()]
                plan.start_state.is_diff = False

            toleranz = float(eintrag["tolerance"])
            goal = Constraints()
            achsen = eintrag.get("goal")
            if achsen:
                for name, value in achsen.items():
                    constraint = JointConstraint()
                    constraint.joint_name = name
                    constraint.position = float(value)
                    constraint.tolerance_above = toleranz
                    constraint.tolerance_below = toleranz
                    constraint.weight = 1.0
                    goal.joint_constraints.append(constraint)
            else:
                lage = eintrag["pose"]
                ort = PositionConstraint()
                ort.header.frame_id = lage["frame"]
                ort.link_name = lage["tip"]
                kugel = SolidPrimitive()
                kugel.type = SolidPrimitive.SPHERE
                kugel.dimensions = [float(lage["position_tolerance"])]
                ort.constraint_region.primitives.append(kugel)
                mitte = Pose()
                px, py, pz = lage["position"]
                mitte.position.x = float(px)
                mitte.position.y = float(py)
                mitte.position.z = float(pz)
                # Die Kugel ist drehsymmetrisch; die Drehung des Bereichs hat
                # keine Bedeutung. Eine gueltige Einheitsdrehung muss aber
                # dastehen, sonst ist das Quaternion die Null.
                mitte.orientation.w = 1.0
                ort.constraint_region.primitive_poses.append(mitte)
                ort.weight = 1.0
                goal.position_constraints.append(ort)

                drehung = OrientationConstraint()
                drehung.header.frame_id = lage["frame"]
                drehung.link_name = lage["tip"]
                qx, qy, qz, qw = lage["orientation"]
                drehung.orientation.x = float(qx)
                drehung.orientation.y = float(qy)
                drehung.orientation.z = float(qz)
                drehung.orientation.w = float(qw)
                drehung.absolute_x_axis_tolerance = toleranz
                drehung.absolute_y_axis_tolerance = toleranz
                drehung.absolute_z_axis_tolerance = toleranz
                drehung.weight = 1.0
                goal.orientation_constraints.append(drehung)
            plan.goal_constraints.append(goal)
            request.request.items.append(item)

        timeout = float(payload.get("timeout") or PLAN_TIMEOUT_MARGIN_S)
        future = self._sequence_client.call_async(request)
        self._pending = ("sequence", future, time.monotonic() + timeout,
                         timeout, tag)
        future.add_done_callback(
            lambda done, t=tag: self._on_sequence_done(done, t))

    def _on_sequence_done(self, future, tag=None):
        """Rueckruf aus rclpy. Wie bei der Planantwort durch _guard."""
        self._guard("Folgenantwort", lambda: self._read_sequence(future, tag))

    def _read_sequence(self, future, tag=None):
        """Antwort in reine Zahlen umsetzen. Aus vielen Bahnen wird eine.

        planned_trajectories ist eine Liste, keine Bahn. Ueberschliffene
        Uebergaenge liegen in derselben Teilbahn, nicht ueberschliffene
        trennen sie -- die Zahl der Teilbahnen sagt damit, ob das
        Ueberschleifen gegriffen hat, und wandert deshalb mit hinaus.

        Jede weitere Teilbahn beginnt wieder bei t = 0. Ihr erster
        Stuetzpunkt ist derselbe Ort zur selben Zeit wie der letzte der
        vorigen; uebernommen wuerde er zweimal, und der zweite traege den
        Zeitstempel des ersten. Er faellt deshalb weg -- nicht als Korrektur,
        sondern weil er dieselbe Zeile ein zweites Mal ist.
        """
        self._pending = None
        try:
            response = future.result()
        except Exception as exc:
            self.results.append(("sequence", {"tag": tag, "error": str(exc)}))
            return

        result = response.response
        code = int(result.error_code.val)
        payload = {"tag": tag, "code": code, "points": [], "teilbahnen": []}
        if code != 1:
            self.results.append(("sequence", payload))
            return

        points = []
        offset = 0.0
        for nummer, bahn in enumerate(result.planned_trajectories):
            trajectory = bahn.joint_trajectory
            names = list(trajectory.joint_names)
            roh = list(trajectory.points)
            if nummer and roh:
                roh = roh[1:]
            gezaehlt = 0
            for point in roh:
                seconds = offset + (point.time_from_start.sec
                                    + point.time_from_start.nanosec * 1e-9)
                positions = {name: float(value)
                             for name, value in zip(names, point.positions)}
                raw = point.velocities
                velocities = ({name: float(value)
                               for name, value in zip(names, raw)}
                              if len(raw) == len(names)
                              else {name: 0.0 for name in names})
                points.append((seconds, positions, velocities))
                gezaehlt += 1
            if points:
                offset = points[-1][0]
            payload["teilbahnen"].append(gezaehlt)
        payload["points"] = points
        payload["planning_time"] = float(result.planning_time)
        self.results.append(("sequence", payload))

    # -- Rueckwaertskinematik ----------------------------------------------

    def _send_ik(self, payload):
        """Eine Lage an /compute_ik geben. Laeuft im Executor-Thread.

        Der Dienst rechnet und faehrt nicht: er liefert eine Achsstellung
        oder einen Fehlercode. Was mit der Stellung geschieht, entscheidet
        die GUI -- hier wird nur gefragt.

        Der Startwert ist kein Beiwerk. Bei einer redundanten Kette waehlt er
        aus, welche der unendlich vielen Loesungen der Solver findet; ohne
        ihn nimmt KDL einen zufaelligen Wurf und liefert bei gleicher Lage
        von Aufruf zu Aufruf eine andere Stellung.
        """
        from moveit_msgs.srv import GetPositionIK
        from geometry_msgs.msg import PoseStamped
        from builtin_interfaces.msg import Duration

        tag = payload.get("tag")
        if self._ik_client is None:
            self._ik_client = self.node.create_client(
                GetPositionIK, "/compute_ik")
        if not self._ik_client.service_is_ready():
            self.results.append(("ik", {
                "tag": tag,
                "error": "/compute_ik antwortet nicht"}))
            return

        request = GetPositionIK.Request()
        ik = request.ik_request
        ik.group_name = payload["group"]
        ik.ik_link_name = payload["tip"]
        ik.avoid_collisions = bool(payload["avoid_collisions"])

        pose = PoseStamped()
        # Ohne Zeitstempel heisst "der neueste Stand". Das ist hier richtig:
        # der Bezugsframe ist die Wurzel der Kette, es ist nichts zu
        # transformieren, und ein Stempel wuerde nur eine Frist erfinden.
        pose.header.frame_id = payload["frame"]
        px, py, pz = payload["position"]
        pose.pose.position.x = float(px)
        pose.pose.position.y = float(py)
        pose.pose.position.z = float(pz)
        qx, qy, qz, qw = payload["orientation"]
        pose.pose.orientation.x = float(qx)
        pose.pose.orientation.y = float(qy)
        pose.pose.orientation.z = float(qz)
        pose.pose.orientation.w = float(qw)
        ik.pose_stamped = pose

        seed = payload["seed"]
        ik.robot_state.joint_state.name = list(seed)
        ik.robot_state.joint_state.position = [
            float(value) for value in seed.values()]
        ik.robot_state.is_diff = False

        seconds = max(0.0, float(payload["solver_timeout"]))
        whole = int(seconds)
        nanos = min(999999999, int(round((seconds - whole) * 1e9)))
        ik.timeout = Duration(sec=whole, nanosec=nanos)

        timeout = float(payload.get("timeout") or IK_TIMEOUT_MARGIN_S)
        future = self._ik_client.call_async(request)
        self._pending = ("ik", future, time.monotonic() + timeout, timeout,
                         tag)
        future.add_done_callback(
            lambda done, t=tag: self._on_ik_done(done, t))

    def _on_ik_done(self, future, tag=None):
        """Rueckruf aus rclpy. Wie bei der Planantwort durch _guard."""
        self._guard("IK-Antwort", lambda: self._read_ik(future, tag))

    def _read_ik(self, future, tag=None):
        """Antwort in ein Woerterbuch aus Zahlen umsetzen.

        Die Antwort traegt den Zustand der ganzen Gruppe, nicht nur die
        Achsen des Ziels. Aussortiert wird drueben im GUI-Thread, denn nur
        der weiss, welche Gelenke gekoppelt sind.
        """
        self._pending = None
        try:
            response = future.result()
        except Exception as exc:
            self.results.append(("ik", {"tag": tag, "error": str(exc)}))
            return

        state = response.solution.joint_state
        self.results.append(("ik", {
            "tag": tag,
            "code": int(response.error_code.val),
            "values": {name: float(value)
                       for name, value in zip(state.name, state.position)},
        }))

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
        # Eigene Frage. Die Capability ist ein Plugin und kann fehlen,
        # waehrend alles andere steht -- etwa bei einem move_group, den
        # jemand von aussen ohne unsere Parameter gestartet hat. Ohne
        # diese Zeile faellt das erst beim Planen einer Folge auf.
        folge = mine and "/plan_sequence_path" in services
        if folge != self.sequence_ready:
            self.sequence_ready = folge
            self.results.append(("sequence_ready", folge))

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
# A, B, C statt R, P, Y. Die Ueberschrift bleibt beim Umschalten stehen --
# das ist der Grund fuer die Namen: A ist immer die erste Drehung, welche
# Achse das ist, sagt die Konvention in der TCP-Zeile. Nebenbei verschwindet
# die Doppeldeutigkeit des alten Kopfes, in dem Y einmal fuer die Y-Achse und
# einmal fuer Yaw stand.
FRAME_COLUMNS = [
    ("x", "X [m]",   64, "e"),
    ("y", "Y [m]",   64, "e"),
    ("z", "Z [m]",   64, "e"),
    ("a", "A [deg]", 66, "e"),
    ("b", "B [deg]", 66, "e"),
    ("c", "C [deg]", 66, "e"),
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
               ("a", "A", "deg"), ("b", "B", "deg"), ("c", "C", "deg")]

# Schluessel der drei Drehfelder. An mehreren Stellen gebraucht, deshalb hier
# und nicht als Literal im Code verstreut.
ANGLE_KEYS = ("a", "b", "c")

MOTION_FIELDS = [("jerk", "Jerk", "m/s3", 10.0),
                 ("acceleration", "Acceleration", "m/s2", 2.0),
                 ("speed", "Speed", "m/s", 0.5)]

PAD = 10                                               # ein Rastermass fuer alles

# Masse des Fahrfensters. Bis 2.21 stand hier eine feste Groesse, die zu den
# Feldern von damals passte; mit den Fahrparametern passt sie nicht mehr. Die
# Hoehe wird deshalb aus dem Bildschirm abgeleitet und nur nach oben begrenzt.
DRIVE_WIDTH = 860
DRIVE_HEIGHT_MAX = 1040
DRIVE_SCREEN_MARGIN = 120                              # Leiste, Rahmen, Rand
DRIVE_MIN_WIDTH = 620
DRIVE_MIN_HEIGHT = 520
DRIVE_WRAP = 620                                       # Umbruch der Hinweistexte

# Masse des Folgenfensters. Links die Liste, rechts der Bericht -- beide
# brauchen Breite, weil eine Punktzeile mit benannten Feldern lang wird
# und ein umgebrochener Bericht seine Spalten verliert.
SEQ_WIDTH = 1120
SEQ_HEIGHT = 720
SEQ_MIN_WIDTH = 780

# Ab diesem Einzelweg einer Achse zwischen zwei Punkten wird der Bericht
# gelb. Kein Fehler, sondern ein Hinweis: bei sieben Achsen bedeutet ein
# solcher Sprung fast immer einen Wechsel des Konfigurationszweigs, und
# der faellt sonst erst auf, wenn der Arm ihn faehrt. Der Wert ist
# grosszuegig gewaehlt -- zwei dicht beieinanderliegende Punkte koennen
# durchaus 60 Grad an einer Achse auseinanderliegen, ohne dass etwas
# faul ist, aber 90 sind es selten.
SEQ_JUMP_DEG = 90.0
SEQ_MIN_HEIGHT = 480

# Steht beim ersten Oeffnen im Feld. Kein leeres Blatt: das Format ist
# ohne Beispiel nicht zu raten, und ein Beispiel, das man ueberschreibt,
# ist billiger als eine Hilfeseite, die man sucht.
SEQ_BEISPIEL = (
    "# Punktfolge. Je Zeile: X Y Z A B C, danach benannte Felder.\n"
    "# Felder: fahrart=PTP|LIN|CIRC  tempo=%  beschleunigung=%\n"
    "#         radius=m (Ueberschleifen)  name=Text\n"
    "# Ein gesetztes Feld gilt weiter, bis es jemand aendert.\n"
    "# Der letzte Punkt braucht radius=0.\n"
    "# J und je bewegliche Achse ein Wert (deg, Schubachse m): Achsziel,\n"
    "#   immer PTP ohne Ueberschleifen. Erlaubt: tempo= beschleunigung= name=\n"
    "\n"
    "1.0 1.0 1.0   45  0 10   fahrart=LIN tempo=80 radius=0.05 name=Anfahrt\n"
    "2.0 1.0 1.5   30 10 25\n"
    "0.0 1.0 1.0    1 20 30   radius=0 name=Ende\n")


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
        self.frame_window = None                       # Frame-Fenster oder None
        self.frame_entries = {}                        # Zahlenfelder des Frames
        self.frame_tip_combo = None                    # Auswahl des aktiven TCP
        self.frame_list_var = None                     # Aufstellung der festen Frames
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

        # ---------- Auftragsnummer ----------
        # Jede Anfrage an /plan_kinematic_path oder /compute_ik bekommt eine
        # Nummer, und die Antwort bringt sie zurueck. Erwartet wird immer
        # hoechstens eine: job_tag traegt die Nummer der offenen Anfrage,
        # None heisst, es wird keine erwartet.
        #
        # Bis 2.20 stand an dieser Stelle allein die Frage, ob der Zustand
        # noch PLANEN ist. Sie reicht nicht. Wer den Halte-Taster kurz
        # antippt, ein Ziel aendert und wieder drueckt, hat zwei Auftraege
        # unterwegs -- die alte Antwort trifft in einen Zustand, der wieder
        # PLANEN heisst, und sieht damit aus wie die neue. Gefahren wuerde
        # dann zum vorigen Ziel, und die Driftpruefung in _drive_start faende
        # nichts, weil sie den Startpunkt des zweiten Auftrags prueft.
        self.job_seq = 0                               # laufende Nummer
        self.job_tag = None                            # erwartete Antwort

        # ---------- Halte-Taster ----------
        # hold_run ist die Wahrheit ueber die Taste, hold_drive die ueber die
        # Fahrt. Zwei Felder, weil sie Verschiedenes sagen: die Taste kann
        # unten sein, waehrend noch geplant wird und nichts faehrt, und eine
        # Fahrt aus dem Taster laeuft weiter, bis die Bremsrampe durch ist,
        # obwohl die Taste schon oben ist.
        self.hold_run = False                          # Taster ist unten
        self.hold_drive = False                        # laufende Fahrt kam vom Taster
        self.hold_watch_job = None                     # after-Kennung des Wachhunds
        self.hold_button = None                        # der Taster selbst
        self.hold_buttons = []                         # fuer die Sperren

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

        # ---------- Zielraum ----------
        # Seit 2.13 hat das Fahrfenster zwei Zielraeume. Beide Feldsaetze
        # bestehen gleichzeitig, sichtbar ist immer nur einer: sie beim
        # Umschalten neu zu bauen hiesse, die getippten Werte des anderen
        # Raums wegzuwerfen, und genau das erwartet niemand.
        self.drive_space_var = tk.StringVar(value=SPACE_JOINT)
        self.drive_cart_entries = {}                   # x,y,z,roll,pitch,yaw
        self.drive_radios = []
        self.drive_joint_box = None
        self.drive_cart_box = None
        self.drive_ik_var = tk.StringVar(value="")

        # ---------- Fahrparameter im Fahrfenster ----------
        # Dieselben drei Groessen wie auf der Karteikarte Fahrt, in einem
        # zweiten Fenster. Bewusst eigene Variablen und nicht planner_var:
        # jedes Fenster fuehrt seinen eigenen Entwurf, und abgeglichen wird
        # erst beim Uebernehmen ueber die Konfiguration. Teilten sich beide
        # eine Variable, aenderte das Tippen im einen Fenster stumm die
        # Anzeige im anderen, ohne dass irgendwo etwas wirksam geworden waere.
        self.drive_param_entries = {}                  # Schluessel -> Feld
        self.drive_param_widgets = []                  # sperren waehrend der Fahrt
        self.drive_planner_var = tk.StringVar(value=PLANNER_IDS[0])
        self.drive_param_var = tk.StringVar(value="")
        # Was zuletzt uebernommen wurde, Feld fuer Feld, als Text. Seit 2.23
        # gibt es keine Uebernehmen-Taste mehr, also auch keinen Augenblick,
        # in dem ein Entwurf offiziell endet. Ob ein Feld abweicht, wird
        # deshalb nicht gemerkt, sondern verglichen -- eine gemerkte Marke
        # koennte falsch stehen, ein Vergleich nicht.
        self.drive_param_shown = {}                    # Schluessel -> Text
        self.drive_scroll_canvas = None

        # ---------- Joggen ----------
        # Die Sollpose wird gefuehrt und nicht bei jedem Schritt aus dem TCP
        # zurueckgelesen. Zurueckgelesen summierte sich der Restfehler der IK
        # auf, und nach zwanzig Schritten laege die Bahn neben der Geraden --
        # sichtbar erst, wenn niemand mehr weiss, woher es kommt.
        self.jog_key = None                            # "x".."yaw" oder None
        self.jog_sign = 0
        self.jog_pose = None                           # gefuehrte Sollpose, SI
        self.jog_pending = False                       # IK unterwegs
        self.jog_hold = False                          # Dauerfahrt laeuft
        self.jog_hold_job = None                       # after-Kennung
        self.jog_last = None                           # Zeit des letzten Schritts
        self.jog_delta = 0.0                           # Weg der offenen Anfrage
        self.jog_before = None                         # Achsstand davor
        self.jog_buttons = []
        self.jog_var = tk.StringVar(value="")
        self.jog_label = None                          # traegt die Warnfarbe
        self.jog_joint = None                          # gejoggte Achse oder None
        self.jog_joint_sign = 0
        self.jogj_hold = False
        self.jogj_hold_job = None
        self.jogj_job = None                           # laufender Takt
        self.jogj_last = None
        self.jogj_buttons = []
        # Beschreibt die Stellung, nicht die Taste: deshalb ueberdauert es
        # das Loslassen und faellt erst, wenn die Auslastung wirklich sinkt.
        self.jog_warned = False

        self.ik_started = None                         # fuer die Dauer
        self.ik_asked = None                           # Lage, fuer die Diagnose

        # Anzeigekonvention. Der Rueckruf wird einmal eingetragen und bleibt
        # bestehen, auch wenn das Parameterfenster zwischendurch zu ist.
        self.convention_var = tk.StringVar(value=ANZEIGE.name)
        self.convention_note_var = tk.StringVar(value=ANZEIGE.beschreibung)
        self.convention_marks = {}                     # Name -> Label
        self.tcp_label = None
        self.ik_frames = None                          # (Bezug, Tip)
        self.plan_hold = None                          # geplant, noch nicht gefahren
        # Die Punktfolge. Getrennt von plan_hold und nicht dieselbe
        # Ablage: ein gehaltener Einzelplan wird gefahren, eine geplante
        # Folge in 3.1 nur angesehen. Zwei Dinge, zwei Namen -- dieselbe
        # Trennung wie zwischen Sollwert und Fahrziel.
        self.seq_window = None
        self.seq_text = None                           # die Punktliste
        self.seq_report = None                         # der Bericht daneben
        self.seq_points = []                           # Stuetzpunkte der Folge
        self.seq_teilbahnen = []                       # Punkte je Teilbahn
        self.seq_dropped = ()                           # verworfene Stempel
        # Die geplante Folge, fahrbereit. Getrennt von plan_hold und nicht
        # dieselbe Ablage: ein Einzelplan entsteht aus dem Fahrfenster, eine
        # Folge aus der Punktliste, und beide duerfen nebeneinander liegen,
        # ohne sich zu ueberschreiben. Gefahren wird trotzdem nur eines --
        # dafuer sorgt STATES_BUSY und nicht die Ablage.
        self.seq_hold = None                           # TrajectoryPlayer
        self.seq_hold_start = None                     # Achsstand beim Planen
        self.seq_drive_button = None                   # Taste 'Folge fahren'
        self.seq_folge = None                          # zu den Punkten gehoerig
        self.seq_started = None
        # Die Kette der IK-Aufrufe. seq_ik_seed wandert von Punkt zu Punkt
        # weiter und ist der Grund, aus dem die Kette ueberhaupt eine ist:
        # jede Loesung wird zum Startwert der naechsten, damit der
        # Konfigurationszweig ueber die ganze Folge derselbe bleibt.
        self.seq_schritte = ()                         # aufgeloeste Punkte
        self.seq_ik_index = 0                          # welcher laeuft
        self.seq_ik_seed = None                        # Startwert des naechsten
        self.seq_ik_start = None                       # Achsstand beim Start
        self.seq_ik_frames = None                      # (Bezug, Tip)
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
        # Zweite Anzeige derselben Lage, nicht zweite Quelle: der Text kommt
        # aus _pose_text wie die Zeile im Hauptfenster. Sie hat nur einen
        # anderen Takt -- siehe _update_drive_tcp.
        self.drive_tcp_var = tk.StringVar(value="")
        self.drive_tcp_label = None                    # Kopf des Fahrfensters
        self.seq_status_var = tk.StringVar(value="keine Folge geplant")
        self.session_var = tk.StringVar(value="Session  00:00:00:000")
        self.refresh_var = tk.StringVar(value="Last Refresh  --")
        self.edit_joint_var = tk.StringVar(value="")
        self.edit_info_var = tk.StringVar(value="")
        self.edit_unit_var = tk.StringVar(value="")
        self.edit_type_var = tk.StringVar(value="")
        self.add_type_var = tk.StringVar(value="revolute")
        self.add_parent_var = tk.StringVar(value="")
        self.add_unit_var = tk.StringVar(value="")
        self.frame_role_var = tk.StringVar(value=ROLE_TOOL)
        self.frame_parent_var = tk.StringVar(value="")
        self.frame_tip_var = tk.StringVar(value=TIP_AUTO)
        self.frame_hint_var = tk.StringVar(value="")
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

        # Erst hier, nicht im Zustandsteil: der Rueckruf zeichnet die
        # Frametabelle und die TCP-Zeile neu, und die gibt es vorher nicht.
        ANZEIGE.melden(self._convention_changed)

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
        # Dieselbe Zeile in drei Stufen. Die Farbe traegt keine eigene
        # Aussage -- sie wiederholt, was im Text steht. Wer die Farbe nicht
        # unterscheiden kann, verliert nichts.
        style.configure("TcpWarn.TLabel", background=SURFACE, foreground=WARN,
                        font=("TkFixedFont", 9))
        style.configure("TcpBad.TLabel", background=SURFACE,
                        foreground=COLOR_STOP, font=("TkFixedFont", 9))
        # Dieselben drei Stufen eine Nummer groesser. Die Kopfzeile des
        # Fahrfensters liest man aus zwei Metern Abstand, waehrend die Hand
        # am Halte-Taster liegt -- 9 Punkt reichen dafuer nicht. Eigene
        # Stilnamen und keine geaenderten: die Zeile im Hauptfenster steht
        # dicht bei der Frametabelle und soll dort bleiben, wie sie ist.
        style.configure("DriveTcp.TLabel", background=SURFACE,
                        foreground=OK_GREEN, font=("TkFixedFont", 11))
        style.configure("DriveTcpWarn.TLabel", background=SURFACE,
                        foreground=WARN, font=("TkFixedFont", 11))
        style.configure("DriveTcpBad.TLabel", background=SURFACE,
                        foreground=COLOR_STOP, font=("TkFixedFont", 11))

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

        style.configure("TRadiobutton", background=SURFACE, foreground=TEXT,
                        indicatorcolor=SURFACE, focuscolor=SURFACE)
        style.map("TRadiobutton", background=[("active", SURFACE)],
                  foreground=[("disabled", DISABLED)],
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
        folge = ttk.Button(tools, text="Punktfolge...",
                           command=self._open_sequence, width=14)
        folge.pack(fill="x", pady=(6, 0))
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
        self.tcp_label = ttk.Label(tcp_inner, textvariable=self.tcp_var,
                                   style="Tcp.TLabel")
        self.tcp_label.pack(side="left")

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
            pad = (10, 3) if key == "a" else (0, 3)
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
        ttk.Button(buttons, text="Ins Fahrfenster",
                   command=self._cart_to_drive).pack(side="left", padx=(8, 0))

        note = tk.Label(tab, background=TINT_OK, foreground=OK_GREEN,
                        justify="left",
                        text="Dieser Reiter stellt keine Achse -- er prueft "
                             "und meldet. Aufgeloest wird im Fahrfenster "
                             "unter 'Kartesisch (IK)': dort rechnet "
                             "/compute_ik, danach wird geplant und gefahren.",
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
        self._seq_drop("das Modell hat gewechselt")
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
        # Bis 2.21 stand hier eine feste Groesse. Sieben Achsen, zwei
        # Zielraeume, die Fahrparameter und vier Tasten im Fuss passen dort
        # nicht mehr hinein. Die Hoehe kommt deshalb aus dem Bildschirm und
        # wird nur nach oben begrenzt -- ein Fenster, das ueber die untere
        # Kante hinausragt, versteckt genau die Tasten, die man sucht.
        height = min(DRIVE_HEIGHT_MAX,
                     max(DRIVE_MIN_HEIGHT,
                         window.winfo_screenheight() - DRIVE_SCREEN_MARGIN))
        width = min(DRIVE_WIDTH,
                    max(DRIVE_MIN_WIDTH, window.winfo_screenwidth() - 80))
        window.geometry(f"{width}x{height}")
        window.minsize(DRIVE_MIN_WIDTH, DRIVE_MIN_HEIGHT)
        self.drive_window = window
        self.drive_entries = {}
        self.drive_cart_entries = {}
        self.drive_widgets = []
        self.drive_radios = []
        self.drive_param_entries = {}
        self.drive_param_widgets = []
        self.drive_param_shown = {}

        window.columnconfigure(0, weight=1)
        # Zeile 0 ist die TCP-Zeile, Zeile 1 der rollende Teil, Zeile 2 der
        # Fuss. Nur Zeile 1 bekommt Gewicht: waechst das Fenster, waechst der
        # rollende Teil, und die beiden festen Zeilen bleiben, wo sie sind.
        window.rowconfigure(1, weight=1)

        # -- TCP-Zeile -----------------------------------------------------
        # Sie steht ausserhalb des Canvas und deshalb ausserhalb der Rollung.
        # Das ist der ganze Punkt: eine Anzeige, die man wegrollen kann, ist
        # waehrend einer Fahrt nicht da, und waehrend einer Fahrt hat man die
        # Hand am Taster und nicht am Rad. Aus demselben Grund steht sie oben
        # und nicht im Fuss -- der Fuss traegt schon die Auftragszeilen.
        head = self._card(window, row=0, column=0, sticky="ew",
                          padx=PAD, pady=(PAD, 0))
        head.columnconfigure(0, weight=1)
        head_inner = tk.Frame(head, background=SURFACE, padx=14, pady=10)
        head_inner.grid(row=0, column=0, sticky="ew")
        head_inner.columnconfigure(0, weight=1)
        ttk.Label(head_inner, text="TCP-LAGE, GERECHNET",
                  style="Section.TLabel").grid(row=0, column=0, sticky="w",
                                               pady=(0, 6))
        self.drive_tcp_label = ttk.Label(
            head_inner, textvariable=self.drive_tcp_var,
            style="DriveTcp.TLabel", justify="left", wraplength=DRIVE_WRAP)
        self.drive_tcp_label.grid(row=1, column=0, sticky="w")
        ttk.Label(head_inner,
                  text="Vorwaertskinematik aus den Achswerten. Kein Sollwert "
                       "und kein Zielfeld -- was hier steht, ist der Stand, "
                       "nicht das Vorhaben.",
                  style="Muted.TLabel", justify="left",
                  wraplength=DRIVE_WRAP).grid(row=2, column=0, sticky="w",
                                              pady=(6, 0))

        card = self._card(window, row=1, column=0, sticky="nsew",
                          padx=PAD, pady=(PAD, 0))
        card.columnconfigure(0, weight=1)
        card.rowconfigure(0, weight=1)

        # Der obere Teil rollt. Das ist keine Verschoenerung, sondern die
        # Antwort auf einen Schirm, der kleiner ist als das, was darauf soll:
        # ohne ihn waeren die untersten Zielfelder eines siebenachsigen Arms
        # auf einem Beamer schlicht nicht erreichbar. Der Fuss rollt
        # ausdruecklich nicht mit -- siehe weiter unten.
        holder = tk.Frame(card, background=SURFACE)
        holder.grid(row=0, column=0, sticky="nsew")
        holder.columnconfigure(0, weight=1)
        holder.rowconfigure(0, weight=1)

        canvas = tk.Canvas(holder, background=SURFACE, highlightthickness=0)
        canvas.grid(row=0, column=0, sticky="nsew")
        scroll = ttk.Scrollbar(holder, orient="vertical", command=canvas.yview)
        scroll.grid(row=0, column=1, sticky="ns")
        canvas.configure(yscrollcommand=scroll.set)
        self.drive_scroll_canvas = canvas

        inside = tk.Frame(canvas, background=SURFACE)
        window_id = canvas.create_window((0, 0), window=inside, anchor="nw")
        inside.columnconfigure(0, weight=1)
        inside.bind("<Configure>", lambda _e: canvas.configure(
            scrollregion=canvas.bbox("all")))
        # Ohne diese Zeile behaelt der Inhalt seine Wunschbreite und klebt
        # links, waehrend rechts weisse Flaeche steht.
        canvas.bind("<Configure>",
                    lambda e: canvas.itemconfigure(window_id, width=e.width))
        # Das Rad wird am Fenster gebunden und nicht am Canvas. Tk stellt ein
        # Ereignis der Reihe nach an Widget, Klasse, Toplevel und all zu --
        # ueber einem Zielfeld traefe eine Bindung am Canvas also nie zu.
        window.bind("<MouseWheel>", self._drive_wheel, add="+")
        window.bind("<Button-4>", self._drive_wheel, add="+")
        window.bind("<Button-5>", self._drive_wheel, add="+")

        self.drive_target_frame = tk.Frame(inside, background=SURFACE,
                                           padx=14, pady=12)
        self.drive_target_frame.grid(row=0, column=0, sticky="nsew")
        self._build_drive_targets()

        param_frame = tk.Frame(inside, background=SURFACE, padx=14, pady=12)
        param_frame.grid(row=1, column=0, sticky="ew")
        self._build_drive_params(param_frame)

        foot = self._card(window, row=2, column=0, sticky="ew",
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

        # Der Halte-Taster steht in einer eigenen Zeile und nicht neben den
        # drei anderen. Nicht aus Platzgruenden: er ist keine vierte Taste in
        # derselben Reihe, sondern ein zweiter Weg durch dieselben drei
        # Schritte, und das soll man sehen.
        hold_row = tk.Frame(inner, background=SURFACE)
        hold_row.grid(row=4, column=0, sticky="ew", pady=(10, 0))

        hold = self._flat_button(hold_row, "FAHREN  (HALTEN)",
                                 lambda: None,
                                 COLOR_HOLD, COLOR_HOLD_HOVER, width=20,
                                 height=2)
        # Kein command: der wuerde erst beim Loslassen feuern, und dann waere
        # aus dem Halten nie eine Fahrt geworden. Dieselbe Bauart wie bei den
        # Jogtasten seit 2.13.
        hold.bind("<ButtonPress-1>", self._hold_press)
        hold.bind("<ButtonRelease-1>", self._hold_release)
        # Rutscht der Zeiger von der Taste, kommt kein Release mehr. Mit
        # add="+", damit die Farbrueckstellung aus _flat_button erhalten
        # bleibt -- ohne das haenge der Taster optisch fest.
        hold.bind("<Leave>", self._hold_release, add="+")
        hold.pack(side="left")
        self.hold_button = hold
        self.hold_buttons = [hold]

        # Der Fokus kann ohne Zutun des Bedieners wegwandern: ein anderes
        # Fenster kommt nach vorn, der Fenstermanager greift ein. Das
        # ButtonRelease geht dann woandershin und kommt hier nie an.
        window.bind("<FocusOut>", self._hold_window_event, add="+")
        window.bind("<Unmap>", self._hold_window_event, add="+")

        ttk.Label(hold_row,
                  text="Druecken plant und faehrt, Loslassen bremst.\n"
                       f"Nach dem Loslassen {BRAKE_TIME_S:.1f} s Bremszeit, "
                       "erst danach nimmt der Taster wieder an.",
                  style="Muted.TLabel", justify="left",
                  wraplength=230).pack(side="left", padx=(12, 0))

        note = tk.Label(inner, background=TINT_OK, foreground=OK_GREEN,
                        justify="left", anchor="w", padx=10, pady=8,
                        wraplength=DRIVE_WRAP,
                        text="Ein Ziel hier bewegt nichts. Erst Planen, dann "
                             "Fahrt starten. Waehrend der Fahrt sind die "
                             "Sollwertfelder gesperrt. Bei einem kartesischen "
                             "Ziel loest Planen zuerst die Rueckwaerts"
                             "kinematik auf; die Achswerte stehen danach "
                             "unter Achsraum.\n\n"
                             "Der Halte-Taster darueber geht denselben Weg in "
                             "einem Griff: er plant selbst und faehrt die "
                             "Bahn, solange er gehalten wird. Die Bahn endet "
                             "trotzdem an ihrem Ziel -- gehalten wird nicht "
                             "weitergefahren. Wer joggen will, nimmt die "
                             "Tasten am Zielfeld; die fahren ohne Plan.\n\n"
                             "Fahrart und Overrides stehen oben unter "
                             "FAHRPARAMETER. Sie gelten, sobald die Eingabe "
                             "fertig ist -- Return oder Feld verlassen --, "
                             "und verwerfen dabei den gehaltenen Plan.")
        note.grid(row=5, column=0, sticky="ew", pady=(12, 0))

        self._update_drive_tcp()                       # nicht leer aufgehen
        self._drive_status()
        self._update_locks()
        # Wie das Parameterfenster bewusst ohne transient() und ohne
        # grab_set: man will nebenher am Baum sehen, wohin es geht.

    def _drive_wheel(self, event):
        """Mausrad im Fahrfenster auf den rollbaren Bereich lenken."""
        canvas = self.drive_scroll_canvas
        if canvas is None or not canvas.winfo_exists():
            return
        # X11 meldet das Rad als Taste 4 und 5, Windows und macOS als
        # MouseWheel mit Betrag. Beides landet hier.
        if getattr(event, "num", None) == 4:
            step = -1
        elif getattr(event, "num", None) == 5:
            step = 1
        else:
            delta = getattr(event, "delta", 0)
            if not delta:
                return
            step = -1 if delta > 0 else 1
        canvas.yview_scroll(step, "units")

    # -- Fahrparameter ------------------------------------------------------

    def _build_drive_params(self, box):
        """Fahrart und die beiden Overrides. Seit 2.23 ohne Uebernehmen-Taste.

        Dieselben Groessen wie auf der Karteikarte Fahrt im Parameterfenster.
        Sie stehen hier ein zweites Mal, weil sie zum einzelnen Auftrag
        gehoeren: wer PTP gegen LIN tauscht, um den Unterschied an der Bahn
        zu sehen, soll dafuer nicht das Fenster wechseln muessen.

        Uebernommen wird bei Return und beim Verlassen des Feldes, bei der
        Fahrart mit der Auswahl. Nicht bei jedem Tastendruck -- eine 4 auf
        dem Weg zur 40 ist keine Eingabe, sondern eine halbe.
        """
        box.columnconfigure(1, weight=1)

        ttk.Label(box, text="FAHRPARAMETER", style="Section.TLabel").grid(
            row=0, column=0, columnspan=3, sticky="w", pady=(0, 10))

        ttk.Label(box, text="Fahrart", style="Card.TLabel").grid(
            row=1, column=0, sticky="w", pady=3)
        combo = ttk.Combobox(box, textvariable=self.drive_planner_var,
                             values=list(PLANNER_IDS), state="readonly",
                             width=10)
        combo.grid(row=1, column=1, sticky="w", padx=(10, 8), pady=3)
        # Eine Auswahl aus einer Liste ist mit dem Klick fertig. Hier gibt es
        # kein Zwischenstadium, das man erst abwarten muesste.
        combo.bind("<<ComboboxSelected>>", self._drive_planner_commit)
        self.drive_param_widgets.append(combo)

        labels = {key: label for key, label, _u, _d in PLAN_FIELDS}
        for offset, key in enumerate(DRIVE_PLAN_KEYS):
            row = 2 + offset
            ttk.Label(box, text=labels.get(key, key),
                      style="Card.TLabel").grid(row=row, column=0, sticky="w",
                                                pady=3)
            entry = self._entry(box, width=10)
            entry.grid(row=row, column=1, sticky="w", padx=(10, 8), pady=3)
            # Return sagt "fertig", das Verlassen des Feldes sagt dasselbe
            # ohne Tastendruck. Beides fuehrt in dieselbe Stelle, damit es
            # keinen Weg gibt, der an der Pruefung vorbeikommt.
            entry.bind("<Return>",
                       lambda _e, k=key: self._drive_param_commit(k))
            entry.bind("<FocusOut>",
                       lambda _e, k=key: self._drive_param_commit(k))
            # Der Rest der Verwerfen-Taste: den wirksamen Wert zurueckholen.
            entry.bind("<Escape>", lambda _e: self._drive_params_fill())
            # Faerbt nur die Statuszeile. Uebernommen wird hier nichts.
            entry.bind("<KeyRelease>", self._drive_params_touched)
            ttk.Label(box, text="%", style="Muted.TLabel", width=4).grid(
                row=row, column=2, sticky="w", pady=3)
            self.drive_param_entries[key] = entry
            self.drive_param_widgets.append(entry)

        row = 2 + len(DRIVE_PLAN_KEYS)
        ttk.Label(box, textvariable=self.drive_param_var, style="Mono.TLabel",
                  wraplength=DRIVE_WRAP, justify="left").grid(
            row=row, column=0, columnspan=3, sticky="w", pady=(12, 0))

        ttk.Label(box,
                  text="Uebernommen wird bei Return und beim Verlassen des "
                       "Feldes, die Fahrart mit der Auswahl. Escape holt den "
                       "wirksamen Wert zurueck. Ein uebernommener Wert wirkt "
                       "beim naechsten Auftrag, ohne Neustart von move_group "
                       "-- und verwirft einen gehaltenen Plan, weil die "
                       "gerechnete Bahn Planer und Zeitgesetz schon in sich "
                       "traegt. 0 Prozent ist nicht zugelassen: MoveIt liest "
                       "daraus nicht Halt, sondern 100.",
                  style="Muted.TLabel", wraplength=DRIVE_WRAP,
                  justify="left").grid(row=row + 1, column=0, columnspan=3,
                                       sticky="w", pady=(8, 0))

        self._drive_params_fill()

    def _drive_params_fill(self):
        """Felder aus der Konfiguration setzen. Danach weicht keins mehr ab."""
        if not self.drive_param_entries:
            return
        config = self.moveit_config
        self.drive_planner_var.set(config.planner_id)
        self.drive_param_shown = {}
        for key, entry in self.drive_param_entries.items():
            if not entry.winfo_exists():
                continue
            value = config.plan.get(key)
            text = "" if value is None else f"{float(value):.4g}"
            entry.configure(state="normal", background=BG_OK)
            entry.delete(0, "end")
            entry.insert(0, text)
            self.drive_param_shown[key] = text
        self._drive_param_status()
        self._update_locks()

    def _drive_param_dirty(self):
        """Weicht ein Feld von dem ab, was zuletzt uebernommen wurde?

        Gefragt und nicht gemerkt. Eine Marke muesste an jeder Stelle
        richtig gesetzt werden, an der ein Feld sich aendert -- und seit die
        Uebernehmen-Taste weg ist, gibt es davon mehr als eine.
        """
        for key, entry in self.drive_param_entries.items():
            if not entry.winfo_exists():
                continue
            if entry.get().strip() != self.drive_param_shown.get(key, ""):
                return True
        return False

    def _drive_params_touched(self, _event=None):
        """Es ist getippt worden. Uebernommen wird deshalb noch nichts."""
        self._drive_param_status()

    def _drive_param_status(self):
        """Was gilt, und ob im Fenster gerade etwas anderes steht.

        Die Zeile trennt zwei Dinge, die leicht verschmelzen: den Wert, mit
        dem die naechste Bahn gerechnet wird, und den Text in einem Feld.
        Solange sie gleich sind, steht hier nur der Wert.
        """
        config = self.moveit_config
        text = (f"wirksam  {config.planner_id}   "
                f"v {float(config.plan['velocity_scaling']):g} %   "
                f"a {float(config.plan['acceleration_scaling']):g} %")
        if self._drive_param_dirty():
            text += "   |   Eingabe offen, Return oder Feld verlassen"
        if config.planner_id == "CIRC":
            # CIRC braucht einen Hilfspunkt auf dem Bogen. Den kennt dieses
            # Fenster nicht, also lehnt Pilz den Auftrag ab. Das hier zu
            # verschweigen hiesse, den Fehler erst beim Planen zu zeigen.
            text += ("\nCIRC braucht einen Zwischenpunkt und wird ohne ihn "
                     "abgelehnt. Fuer eine Fahrt aus diesem Fenster: PTP "
                     "oder LIN.")
        self.drive_param_var.set(text)

    def _drive_params_locked(self):
        """Waehrend einer Fahrt wird nichts uebernommen.

        Die Felder sind dann gesperrt, die Pruefung steht trotzdem hier: ein
        FocusOut kommt auch von einem Fenster, das gerade schliesst, und von
        einem Fenstermanager, der den Fokus wegnimmt. Auf eine Sperre zu
        vertrauen, die anderswo gesetzt wird, hiesse dasselbe wie beim
        Halte-Taster auf ein ausbleibendes Ereignis zu warten.
        """
        if not self._busy():
            return False
        self._log("Fahrparameter sind waehrend "
                  f"'{self.lock_reason}' gesperrt", "error")
        return True

    def _drive_param_commit(self, key, _event=None):
        """Ein Override uebernehmen. Jedes Feld fuer sich.

        Mit einer Taste fuer alle drei Werte war alles oder nichts richtig.
        Ohne sie ist jede Eingabe eine eigene Handlung, und ein unlesbarer
        Override darf die Fahrart nicht aufhalten.
        """
        entry = self.drive_param_entries.get(key)
        if entry is None or not entry.winfo_exists():
            return False

        text = entry.get().strip()
        if text == self.drive_param_shown.get(key, ""):
            # Unveraendert. Ein FocusOut allein ist keine Eingabe, und eine
            # Meldung darueber waere Laerm: das Feld wird bei jedem
            # Fensterwechsel verlassen.
            self._drive_param_status()
            return True

        if self._drive_params_locked():
            self._drive_params_fill()
            return False

        labels = {k: label for k, label, _u, _d in PLAN_FIELDS}
        label = labels.get(key, key)
        try:
            value = float(text.replace(",", "."))
        except ValueError:
            # Stehenlassen und faerben. Still auf den alten Wert
            # zurueckzusetzen hiesse, der Bediener saehe eine Zahl, die er
            # nicht getippt hat, und wuesste nicht, dass seine weg ist.
            entry.configure(background=BG_INVALID)
            self._log(f"{label} ist nicht lesbar, nicht uebernommen", "error")
            self._drive_param_status()
            return False

        if value <= 0.0:
            entry.configure(background=BG_INVALID)
            self._log(f"{label} muss groesser als null sein, "
                      "nicht uebernommen", "error")
            self._drive_param_status()
            return False

        if value > 100.0:
            value = 100.0
            entry.configure(background=BG_CLAMPED)
            self._log(f"{label} auf 100 Prozent begrenzt", "warn")
        else:
            entry.configure(background=BG_OK)

        # Das Feld traegt danach den uebernommenen Wert und nicht mehr die
        # Eingabe. Nur so sagt der Vergleich in _drive_param_dirty die
        # Wahrheit -- und bei einer Begrenzung sieht man, was gilt.
        gezeigt = f"{value:.4g}"
        entry.delete(0, "end")
        entry.insert(0, gezeigt)
        self.drive_param_shown[key] = gezeigt

        if float(self.moveit_config.plan[key]) == value:
            self._drive_param_status()
            return True

        self.moveit_config.plan[key] = value
        self._drive_params_effect(f"{label} {value:g} %")
        return True

    def _drive_planner_commit(self, _event=None):
        """Die Fahrart uebernehmen. Die Auswahl selbst ist die Handlung."""
        planner = self.drive_planner_var.get()
        if planner not in PLANNER_IDS:
            # Ein Auswahlfeld sollte nichts anderes liefern koennen. Die
            # Pruefung kostet nichts, und die Alternative waere eine
            # Konfiguration, die erst Pilz beanstandet.
            self._log(f"Fahrart '{planner}' ist unbekannt", "error")
            self.drive_planner_var.set(self.moveit_config.planner_id)
            return False
        if planner == self.moveit_config.planner_id:
            self._drive_param_status()
            return True
        if self._drive_params_locked():
            self.drive_planner_var.set(self.moveit_config.planner_id)
            return False

        self.moveit_config.planner_id = planner
        self._drive_params_effect(f"Fahrart {planner}")
        return True

    def _drive_params_effect(self, was):
        """Was nach jeder uebernommenen Aenderung gleich ablaeuft.

        Eine Stelle fuer alle drei Werte. Zwei Stellen waeren zwei Chancen,
        das Verwerfen des Plans zu vergessen.
        """
        self._log(f"Fahrparameter uebernommen: {was}", "ok")
        # Der gehaltene Plan gehoert zu den Werten, mit denen er gerechnet
        # wurde. Ihn mit neuen abzutakten hiesse, er waere mit ihnen
        # entstanden -- derselbe Grund wie beim geaenderten Sollwert.
        self._drop_plan("Fahrparameter geaendert")
        self._seq_drop("Fahrparameter geaendert")

        # Ein Wert, zwei Fenster. Das Parameterfenster zeigt jetzt etwas
        # anderes als die Konfiguration und wird deshalb nachgezogen. Dass
        # dabei ein dort begonnener Entwurf wegfaellt, wird gesagt und nicht
        # stillschweigend getan.
        if self.params_window is not None and self.params_window.winfo_exists():
            self._params_fill()
            self._log("Parameterfenster nachgezogen; dort nicht uebernommene "
                      "Eingaben sind dabei verworfen", "warn")

        self._drive_param_status()
        self._update_locks()

    def _close_drive(self):
        self._jog_stop("Fahrfenster geschlossen")
        self._jogj_stop("Fahrfenster geschlossen")
        # Der Taster verschwindet mit dem Fenster, sein ButtonRelease kommt
        # also nie. Ohne diese Zeile fuehre eine laufende Fahrt weiter, und
        # der Wachhund haette nichts mehr, worauf er zeigen koennte.
        self._hold_abort("Fahrfenster geschlossen")
        self.jog_buttons = []
        self.jogj_buttons = []
        self.hold_buttons = []
        self.hold_button = None
        if self.drive_window is not None and self.drive_window.winfo_exists():
            self.drive_window.destroy()
        self.drive_window = None
        # Ohne diese Zeile zeigte drive_tcp_label auf ein zerstoertes Widget.
        # _update_drive_tcp faengt das ueber winfo_exists ab, aber sich auf
        # den Fang zu verlassen hiesse, den Zeiger absichtlich falsch stehen
        # zu lassen.
        self.drive_tcp_label = None
        self.drive_entries = {}
        self.drive_cart_entries = {}
        self.drive_widgets = []
        self.drive_radios = []
        self.drive_joint_box = None
        self.drive_cart_box = None
        self.drive_target_frame = None
        # Eine Eingabe, die nie fertig wurde, verschwindet mit dem Fenster.
        # Sie zu behalten hiesse, beim naechsten Oeffnen stuende etwas im
        # Feld, was nirgends gilt.
        self.drive_param_entries = {}
        self.drive_param_widgets = []
        self.drive_param_shown = {}
        self.drive_scroll_canvas = None
        # Die Tasten sind mit zerstoert, ihre Eintraege nicht.
        self.drive_buttons = [b for b in self.drive_buttons if b.winfo_exists()]
        self.start_buttons = [b for b in self.start_buttons if b.winfo_exists()]

    def _build_drive_targets(self):
        """Zielfelder passend zum Modell aufbauen. Auch nach einem Ladevorgang.

        Seit 2.13 stehen hier zwei Feldsaetze uebereinander, von denen immer
        genau einer sichtbar ist. Gebaut werden beide: sie beim Umschalten
        neu zu erzeugen wuerde die Eingaben des jeweils anderen Raums
        wegwerfen, und das erwartet niemand von einer Auswahl.
        """
        frame = getattr(self, "drive_target_frame", None)
        if frame is None or not frame.winfo_exists():
            return
        for child in frame.winfo_children():
            child.destroy()
        self.drive_entries = {}
        self.drive_cart_entries = {}
        self.drive_radios = []
        # Die Kinder sind eben zerstoert worden, ihre Eintraege in der Liste
        # nicht. Ohne dieses Sieb wuechse sie mit jedem Laden.
        self.drive_widgets = [w for w in self.drive_widgets if w.winfo_exists()]
        self.jog_buttons = [b for b in self.jog_buttons if b.winfo_exists()]
        self.jogj_buttons = [b for b in self.jogj_buttons if b.winfo_exists()]
        self._jog_stop("Zielfelder neu aufgebaut")
        self._jogj_stop("Zielfelder neu aufgebaut")
        # Der Taster selbst sitzt im Fuss und wird hier nicht zerstoert -- der
        # Wachhund fiele also nicht auf, und ein gehaltener Taster fuehre
        # weiter, waehrend die Zielfelder unter dem laufenden Auftrag neu
        # entstehen. Genau der Fall, den 2.7 mit dem gehaltenen Plan schon
        # einmal hatte: die Bahn gehoert zu Werten, die es nicht mehr gibt.
        self._hold_abort("Zielfelder neu aufgebaut")

        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(2, weight=1)

        ttk.Label(frame, text="ZIELRAUM", style="Section.TLabel").grid(
            row=0, column=0, sticky="w", pady=(0, 6))

        choose = tk.Frame(frame, background=SURFACE)
        choose.grid(row=1, column=0, sticky="w", pady=(0, 12))
        for label, value in (("Achsraum", SPACE_JOINT),
                             ("Kartesisch (IK)", SPACE_CART)):
            radio = ttk.Radiobutton(choose, text=label, value=value,
                                    variable=self.drive_space_var,
                                    command=self._drive_space_changed)
            radio.pack(side="left", padx=(0, 18))
            self.drive_radios.append(radio)

        self.drive_joint_box = tk.Frame(frame, background=SURFACE)
        self.drive_cart_box = tk.Frame(frame, background=SURFACE)
        self._build_drive_joint_box(self.drive_joint_box)
        self._build_drive_cart_box(self.drive_cart_box)
        self._drive_space_changed()

    def _build_drive_joint_box(self, box):
        """Ein Zielfeld je beweglicher Achse.

        Gekoppelte Gelenke stehen nicht dabei: sie haben keinen eigenen
        Freiheitsgrad, ein Ziel fuer sie waere keins.
        """
        box.columnconfigure(1, weight=1)
        ttk.Label(box, text="ZIEL IM ACHSRAUM", style="Section.TLabel").grid(
            row=0, column=0, columnspan=5, sticky="w", pady=(0, 10))

        movable = [j for j in self.joints
                   if j["type"] != "fixed" and j["mimic"] is None]
        if not movable:
            ttk.Label(box, text="keine beweglichen Gelenke",
                      style="Muted.TLabel").grid(row=1, column=0, sticky="w")
            return

        for index, joint in enumerate(movable, start=1):
            name = joint["name"]
            step = slider_step(joint)
            digits = value_digits(step)
            value = self.positions.get(name, 0.0)
            shown = math.degrees(value) if is_rotational(joint) else value

            ttk.Label(box, text=name, style="Card.TLabel").grid(
                row=index, column=0, sticky="w", pady=3)
            entry = self._entry(box, width=10, value=f"{shown:.{digits}f}")
            entry.grid(row=index, column=1, sticky="e", padx=(10, 8), pady=3)
            # Eine Zahl im Feld ist noch kein Auftrag: Return plant, es faehrt
            # weiterhin nichts von allein.
            entry.bind("<Return>", lambda _e: self._drive_plan())
            self.drive_entries[name] = entry

            ttk.Label(box, text="deg" if is_rotational(joint) else "m",
                      style="Muted.TLabel", width=4).grid(
                row=index, column=2, sticky="w", pady=3)

            # Dieselben Tasten wie kartesisch, nur ohne Umweg ueber die IK.
            # Sie schreiben nicht in das Zielfeld daneben, sondern stellen
            # die Achse -- das Feld zieht danach nach, wie ueberall beim
            # Joggen.
            jog = tk.Frame(box, background=SURFACE)
            jog.grid(row=index, column=3, sticky="w", padx=(10, 0), pady=3)
            for text, sign in (("\u2212", -1), ("+", 1)):
                button = ttk.Button(jog, text=text, width=2,
                                    style="Mini.TButton")
                button.bind("<ButtonPress-1>",
                            lambda _e, n=name, s=sign: self._jogj_press(n, s))
                button.bind("<ButtonRelease-1>", self._jogj_release)
                button.bind("<Leave>", self._jogj_release)
                button.pack(side="left", padx=(0, 3))
                self.jogj_buttons.append(button)

            low, high = slider_range(joint)
            places = limit_digits(joint)
            ttk.Label(box, text=f"{low:.{places}f} .. {high:.{places}f}",
                      style="Muted.TLabel").grid(row=index, column=4,
                                                 sticky="e", padx=(8, 0), pady=3)

        row = len(movable) + 1
        ttk.Label(box, text=f"Joggen: Klick {JOG_STEP_DEG:g} deg bzw. "
                            f"{JOG_STEP_M * 1000:g} mm, Halten "
                            f"{JOG_VEL_DEG_S:g} deg/s bzw. {JOG_VEL_M_S:g} m/s. "
                            f"Stellt die Achse unmittelbar, ohne move_group.",
                  style="Muted.TLabel", wraplength=DRIVE_WRAP, justify="left").grid(
            row=row, column=0, columnspan=5, sticky="w", pady=(12, 0))

        row += 1
        buttons = tk.Frame(box, background=SURFACE)
        buttons.grid(row=row, column=0, columnspan=5, sticky="w", pady=(14, 0))
        for label, command in (("Ist-Werte", self._drive_from_current),
                               ("Nullstellung", self._drive_zero)):
            button = ttk.Button(buttons, text=label, command=command, width=13)
            button.pack(side="left", padx=(0, 8))
            self.drive_widgets.append(button)

    def _build_drive_cart_box(self, box):
        """Sechs Zahlen: Lage und Drehung des Tip-Links im Bezugsframe.

        Die Werte sind bewusst mit der aktuellen TCP-Lage vorbelegt und nicht
        mit null. Null ist bei einem Arm fast immer der Sockelmittelpunkt,
        also eine Lage, die es nicht gibt -- und der erste Versuch endete
        dann mit "keine IK-Loesung", ohne dass jemand etwas falsch gemacht
        haette.
        """
        box.columnconfigure(1, weight=1)
        ttk.Label(box, text="ZIEL IM BASISSYSTEM", style="Section.TLabel").grid(
            row=0, column=0, columnspan=3, sticky="w", pady=(0, 10))

        start = {}
        pose = self._tcp_pose()
        if pose is not None:
            xyz, abc = pose
            start = {"x": xyz[0], "y": xyz[1], "z": xyz[2],
                     "a": math.degrees(abc[0]),
                     "b": math.degrees(abc[1]),
                     "c": math.degrees(abc[2])}

        for row, (key, label, unit) in enumerate(CART_FIELDS, start=1):
            pad = (10, 3) if key == "a" else (0, 3)
            digits = 4 if unit == "m" else 2
            ttk.Label(box, text=label, style="Card.TLabel").grid(
                row=row, column=0, sticky="w", pady=pad)
            entry = self._entry(box, width=12,
                                value=f"{start.get(key, 0.0):.{digits}f}")
            entry.grid(row=row, column=1, sticky="e", padx=(10, 8), pady=pad)
            entry.bind("<Return>", lambda _e: self._drive_plan())
            ttk.Label(box, text=unit, style="Muted.TLabel", width=4).grid(
                row=row, column=2, sticky="w", pady=pad)
            self.drive_cart_entries[key] = entry

            # Die Jogtasten stehen bewusst neben dem Zielfeld und schreiben
            # trotzdem nicht hinein. Das Feld beschreibt ein Ziel, zu dem
            # geplant wird; die Tasten bewegen unmittelbar. Zwei Bedeutungen
            # in einem Feld waeren wieder der Fehler aus 2.4.
            jog = tk.Frame(box, background=SURFACE)
            jog.grid(row=row, column=3, sticky="w", padx=(10, 0), pady=pad)
            for text, sign in (("\u2212", -1), ("+", 1)):
                button = ttk.Button(jog, text=text, width=2,
                                    style="Mini.TButton")
                # Kein command: der Befehl feuert erst beim Loslassen, und
                # dann waere aus dem Halten nie eine Fahrt geworden.
                button.bind("<ButtonPress-1>",
                            lambda _e, k=key, s=sign: self._jog_press(k, s))
                button.bind("<ButtonRelease-1>", self._jog_release)
                # Rutscht der Zeiger von der Taste, kommt kein Release mehr --
                # ohne diese Bindung fuehre der Roboter weiter.
                button.bind("<Leave>", self._jog_release)
                button.pack(side="left", padx=(0, 3))
                self.jog_buttons.append(button)

        row = len(CART_FIELDS) + 1
        self.jog_label = ttk.Label(box, textvariable=self.jog_var,
                                   style="Tcp.TLabel", wraplength=DRIVE_WRAP,
                                   justify="left")
        self.jog_label.grid(row=row, column=0, columnspan=4, sticky="w",
                            pady=(12, 0))
        ttk.Label(box, text=f"Joggen: Klick {JOG_STEP_M * 1000:g} mm bzw. "
                            f"{JOG_STEP_DEG:g} deg, Halten {JOG_VEL_M_S:g} m/s "
                            f"bzw. {JOG_VEL_DEG_S:g} deg/s. Faehrt sofort, "
                            f"ohne Plan.",
                  style="Muted.TLabel", wraplength=DRIVE_WRAP, justify="left").grid(
            row=row + 1, column=0, columnspan=4, sticky="w", pady=(4, 0))

        buttons = tk.Frame(box, background=SURFACE)
        buttons.grid(row=row + 2, column=0, columnspan=4, sticky="w",
                     pady=(14, 0))
        button = ttk.Button(buttons, text="TCP uebernehmen", width=17,
                            command=self._drive_cart_from_tcp)
        button.pack(side="left", padx=(0, 8))
        self.drive_widgets.append(button)

        ttk.Label(box, textvariable=self.drive_ik_var, style="Muted.TLabel",
                  wraplength=DRIVE_WRAP, justify="left").grid(
            row=row + 3, column=0, columnspan=4, sticky="w", pady=(14, 0))
        self._drive_ik_hint()

    def _drive_space_changed(self):
        """Umschalten zwischen den beiden Zielraeumen.

        Ein gehaltener Plan faellt dabei weg. Er waere nicht falsch -- die
        Bahn bleibt dieselbe --, aber das Ziel, zu dem er gehoert, ist nicht
        mehr zu sehen. Ein Plan, dessen Ziel im verdeckten Feldsatz steht,
        ist kein Plan, sondern ein Raetsel.
        """
        joint_box = getattr(self, "drive_joint_box", None)
        cart_box = getattr(self, "drive_cart_box", None)
        if joint_box is None or not joint_box.winfo_exists():
            return
        if cart_box is None or not cart_box.winfo_exists():
            return

        cart = self.drive_space_var.get() == SPACE_CART
        shown, hidden = (cart_box, joint_box) if cart else (joint_box, cart_box)
        shown.grid(row=2, column=0, sticky="nsew")
        hidden.grid_remove()
        if cart:
            self._drive_ik_hint()
        self._drop_plan()

    def _drive_ik_frames(self):
        """(Bezugsframe, Tip-Link) fuer die IK-Anfrage.

        Erste Quelle ist die SRDF, denn genau die liest move_group. Laeuft
        noch keiner, wird dasselbe aus dem Modell abgeleitet, wie build_srdf
        es taete -- sonst stuende im Fenster eine Kette, die spaeter eine
        andere ist.
        """
        chain = self._srdf_chain()
        if chain:
            return chain
        roots = root_links(self.links, self.joints)
        base = roots[0] if roots else ""
        tip = self.moveit_config.tip
        if not tip:
            leaves = [n for n in leaf_links(self.links, self.joints)
                      if n != base]
            tip = leaves[0] if leaves else ""
        return base, tip

    def _drive_ik_hint(self):
        """Eine Zeile darueber, worauf sich die sechs Zahlen beziehen."""
        base, tip = self._drive_ik_frames()
        quelle = "SRDF" if self._srdf_chain() else "Modell"
        self.drive_ik_var.set(
            f"Bezug {base or '?'}, Tip-Link {tip or '?'} (aus {quelle}). "
            f"Der aktuelle Achsstand geht als Startwert mit -- bei einer "
            f"redundanten Kette waehlt er die Loesung aus.")

    def _drive_cart_from_tcp(self):
        """Kartesische Zielfelder auf die aktuelle TCP-Lage setzen."""
        pose = self._tcp_pose()
        if pose is None:
            self._log("Kein eindeutiger Endeffektor, nichts uebernommen", "warn")
            return
        xyz, abc = pose
        values = {"x": xyz[0], "y": xyz[1], "z": xyz[2],
                  "a": math.degrees(abc[0]),
                  "b": math.degrees(abc[1]),
                  "c": math.degrees(abc[2])}
        self._drive_write_cart(values)

    def _drive_write_cart(self, values):
        """Sechs Zahlen in die kartesischen Zielfelder schreiben."""
        for key, _label, unit in CART_FIELDS:
            entry = self.drive_cart_entries.get(key)
            if entry is None or not entry.winfo_exists():
                continue
            if key not in values:
                continue
            digits = 4 if unit == "m" else 2
            entry.configure(state="normal", background=BG_OK)
            entry.delete(0, "end")
            entry.insert(0, f"{values[key]:.{digits}f}")
        self._update_locks()

    def _drive_write_targets(self, values):
        """Achswerte in die Zielfelder schreiben. Bewegt nichts.

        Hierhin schreibt die IK ihr Ergebnis. Es steht damit an derselben
        Stelle wie ein getippter Wert und wird von dort an genauso behandelt
        -- begrenzt, gefaerbt, geplant. Ein zweiter Weg in den Planer waere
        ein zweiter Ort, an dem eine Achsgrenze vergessen werden kann.
        """
        for joint in self.joints:
            if joint["type"] == "fixed" or joint["mimic"] is not None:
                continue
            entry = self.drive_entries.get(joint["name"])
            if entry is None or not entry.winfo_exists():
                continue
            if joint["name"] not in values:
                continue
            value = values[joint["name"]]
            shown = math.degrees(value) if is_rotational(joint) else value
            entry.configure(state="normal", background=BG_OK)
            entry.delete(0, "end")
            entry.insert(0, f"{shown:.{value_digits(slider_step(joint))}f}")
        self._update_locks()

    def _read_cart_target(self):
        """Sechs Zahlen lesen und in SI umrechnen. None heisst: unlesbar."""
        values = {}
        invalid = 0
        for key, label, unit in CART_FIELDS:
            entry = self.drive_cart_entries.get(key)
            if entry is None or not entry.winfo_exists():
                continue
            try:
                typed = float(entry.get().replace(",", "."))
            except ValueError:
                entry.configure(background=BG_INVALID)
                self._log(f"{label} ist nicht lesbar", "error")
                invalid += 1
                continue
            entry.configure(background=BG_OK)
            values[key] = math.radians(typed) if unit == "deg" else typed

        if invalid or len(values) != len(CART_FIELDS):
            return None
        return values

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
        elif self.state == STATE_IK and self.ik_started is not None:
            waited = time.monotonic() - self.ik_started
            hint = f"{self.moveit_config.solver.split('/')[-1]}, "
            hint += f"wartet {waited:.1f} s"
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
        for entry in self.drive_cart_entries.values():
            if entry.winfo_exists():
                entry.configure(state=setpoints)
        for widget in self.drive_widgets:
            if widget.winfo_exists():
                widget.configure(state=setpoints)
        # Der Zielraum ist Teil des Auftrags. Waehrend er laeuft, wird er
        # nicht gewechselt -- sonst zeigte das Fenster einen anderen Raum,
        # als der gerade gerechnete Plan meint.
        for radio in self.drive_radios:
            if radio.winfo_exists():
                radio.configure(state=setpoints)

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
        # Joggen braucht dasselbe wie das Planen: einen antwortenden
        # move_group mit demselben Modell. Es braucht keinen Plan -- es ist
        # der Weg ohne Bahn.
        for button in self.jog_buttons:
            if button.winfo_exists():
                button.configure(state="normal" if plannable else "disabled")

        # Achsjoggen braucht kein move_group: es stellt die Achse selbst,
        # wie die Sollwertfelder links. Es haengt deshalb an denselben
        # Bedingungen wie sie und nicht an denen des Planens.
        for button in self.jogj_buttons:
            if button.winfo_exists():
                button.configure(state=setpoints)

        # Die Folgentaste haengt an denselben zwei Gruenden wie alles andere
        # und zusaetzlich an einer gehaltenen Folge. Hier gerufen und nicht
        # nebenher: es gibt genau eine Stelle, an der Bedienelemente scharf
        # werden, und das soll so bleiben.
        self._seq_locks()

        # Der Halte-Taster plant selbst, er braucht also dasselbe wie das
        # Planen und keinen gehaltenen Plan. Waehrend der eigenen Fahrt
        # bleibt er ausdruecklich scharf: ein stumpfer Taster nimmt in Tk
        # zwar weiter Bindungen an, aber sich darauf zu verlassen hiesse,
        # das Loslassen an eine Feinheit der Ereignisverteilung zu haengen.
        # Genau daran darf die Bremse nicht haengen.
        holdable = plannable or self.hold_run
        for button in self.hold_buttons:
            if button.winfo_exists():
                button.configure(
                    state="normal" if holdable else "disabled",
                    background=button.base_colour if holdable else DISABLED_BG)

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
                # Bis 2.21 bekam auch das Auswahlfeld der Fahrart hier
                # "normal" und wurde damit frei beschreibbar: nach der ersten
                # Sperrung liess sich ein beliebiger Text hineintippen, der
                # ueber _params_read als planner_id in die Konfiguration
                # gelangte. Bemerkt haette man es erst an der Absage von Pilz.
                if isinstance(widget, ttk.Combobox):
                    widget.configure(
                        state="disabled" if busy else "readonly")
                else:
                    widget.configure(state=model)
        # Die Fahrparameter im Fahrfenster sind dieselben Werte und haengen
        # deshalb an derselben Bedingung: waehrend einer Fahrt wird nicht
        # daran gedreht. STOP sperrt sie dagegen nicht -- gestoppt darf man
        # den naechsten Auftrag vorbereiten.
        for widget in self.drive_param_widgets:
            if widget.winfo_exists():
                # Ein Combobox bleibt readonly und wird nicht schreibbar.
                if isinstance(widget, ttk.Combobox):
                    widget.configure(
                        state="disabled" if busy else "readonly")
                else:
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

    # -- Auftragsnummer -----------------------------------------------------

    def _job_open(self):
        """Eine neue Auftragsnummer vergeben und als erwartet vermerken."""
        self.job_seq += 1
        self.job_tag = self.job_seq
        return self.job_tag

    def _job_drop(self):
        """Keine Antwort mehr erwarten. Was noch kommt, faellt weg.

        Gerufen ueberall dort, wo eine laufende Rechnung aufgegeben wird.
        Die Anfrage im Solver bricht dadurch nicht ab -- sie laeuft zu Ende
        und legt ihr Ergebnis ab wie jede andere. Nur nimmt es hier niemand
        mehr an.
        """
        self.job_tag = None

    def _job_mine(self, result):
        """True, wenn diese Antwort zu der Anfrage gehoert, die laeuft.

        Zwei Faelle fallen hier weg und sind verschiedene Dinge: eine
        Antwort, auf die niemand mehr wartet (job_tag ist None), und eine,
        die zu einem frueheren Auftrag gehoert. Beide sahen bis 2.20 gleich
        aus wie die erwartete, weil an einer Planantwort nichts steht, was
        sie einer Anfrage zuordnet.

        Eine Antwort ohne Nummer wird abgelehnt und nicht durchgelassen.
        Durchlassen waere die bequeme Wahl und die falsche: eine Bruecke,
        die die Nummer verliert, faellt sonst nie auf, und die Absicherung
        waere ab dem Tag wirkungslos, an dem jemand einen Ablageort in der
        Bruecke uebersieht.
        """
        if self.job_tag is None:
            return False
        return result.get("tag") == self.job_tag

    # ----------------------------------------------------------------------
    # PTP fahren
    # ----------------------------------------------------------------------

    def _drive_plan(self):
        """Ziel aus dem Fahrfenster planen lassen. Es faehrt noch nichts.

        Bis 2.6 hing beides an einer Taste, und die Zielwerte standen in
        denselben Feldern, die die Achse sofort stellen. Damit war der Start
        immer gleich dem Ziel und die Bahn leer. Seit 2.7 sind es drei
        getrennte Handlungen: stellen, planen, fahren.

        Hier stehen nur die Pruefungen, die fuer beide Zielraeume gelten, und
        die Weiche. Ein Ziel im Achsraum geht unmittelbar an den Planer; ein
        kartesisches wird zuerst von /compute_ik in Achswerte aufgeloest und
        laeuft danach denselben Weg.
        """
        if self.state in STATES_BUSY:
            self._log(f"Es laeuft bereits: {self.state}", "error")
            return
        if self.jog_key is not None or self.jog_pending:
            self._log("Es wird gerade gejoggt, erst die Taste loslassen",
                      "error")
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

        if self.drive_space_var.get() == SPACE_CART:
            if not self.drive_cart_entries:
                self._log("Das Fahrfenster ist nicht offen, kein Ziel "
                          "vorhanden", "error")
                return
            self._ik_request()
            return

        if not self.drive_entries:
            self._log("Das Fahrfenster ist nicht offen, kein Ziel vorhanden",
                      "error")
            return
        self._plan_joint_goal()

    def _plan_joint_goal(self):
        """Den Auftrag aus den Achszielfeldern stellen.

        Der einzige Weg in /plan_kinematic_path. Ein kartesisches Ziel kommt
        hier ebenfalls an, nur eben nachdem die IK es in Achswerte
        uebersetzt und in dieselben Felder geschrieben hat.
        """
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
        payload["tag"] = self._job_open()

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
        if not self._job_mine(result):
            # Der Zustand allein reicht nicht: er kann inzwischen wieder
            # PLANEN heissen, weil ein zweiter Auftrag laeuft. Diese Antwort
            # gehoert dann zum ersten und beschreibt einen Weg zu einem Ziel,
            # das niemand mehr meint.
            self._log("Verspaetete Planantwort verworfen: sie gehoert zu "
                      "einem abgebrochenen Auftrag", "warn")
            return
        self._job_drop()

        took = (time.monotonic() - self.plan_started
                if self.plan_started else 0.0)
        self.plan_started = None

        # Auf jedem Weg, der hier ohne Bahn herausgeht, faellt der Taster.
        # Sonst stuende er auf gehalten, waehrend nichts mehr unterwegs ist:
        # der Wachhund liefe, die Anzeige loege, und der naechste Druck
        # wuerde abgelehnt, weil der erste nie zu Ende ging. Wer nach einem
        # Fehler weiterwill, hebt die Hand und drueckt neu -- dieselbe Regel
        # wie nach jedem anderen Abbruch.
        if result.get("error"):
            self._hold_stop("die Planung ist gescheitert")
            self._set_state(STATE_ERROR, "Planung")
            self._log(f"Kein Plan: {result['error']}", "error")
            self._plan_diagnosis()
            return

        code = int(result.get("code", 0))
        if code != 1:
            self._hold_stop("die Planung ist gescheitert")
            self._set_state(STATE_ERROR, "Planung")
            self._log(f"Kein Plan: {moveit_error_text(code)}", "error")
            self._plan_diagnosis()
            return

        points = result.get("points") or []
        if len(points) < 2:
            # Steht der Roboter schon am Ziel, liefert Pilz einen einzelnen
            # Punkt. Das ist kein Fehler, es gibt nur nichts zu fahren.
            self._hold_stop("es gibt nichts zu fahren")
            self._set_state(STATE_READY)
            self._log("Ziel bereits erreicht, keine Fahrt noetig", "ok")
            return

        try:
            player = TrajectoryPlayer(points)
        except ValueError as exc:
            self._hold_stop("die Bahn liess sich nicht aufbauen")
            self._set_state(STATE_ERROR, "Bahn")
            self._log(str(exc), "error")
            return

        # Der Taster ist die bewusste Handlung, und er ist noch unten: dann
        # wird jetzt gefahren, ohne den Plan zwischenzulegen. Ist er
        # inzwischen oben, faellt der Plan in den normalen Weg zurueck und
        # wartet auf "Fahrt starten" -- gerechnet ist er ja, und ihn
        # wegzuwerfen waere die Arbeit umsonst.
        if self.hold_run:
            self._hold_launch(player, took, len(points))
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

    # -- Rueckwaertskinematik ----------------------------------------------

    # ----------------------------------------------------------------------
    # Kartesisches Joggen
    # ----------------------------------------------------------------------
    #
    # Anders als Planen und Fahren: hier liegt zwischen Rechnen und Bewegen
    # keine zweite Handlung. Das ist Absicht und kein Rueckfall hinter 2.7.
    # Joggen ist die Handlung -- der Finger auf der Taste ist die Freigabe,
    # und beim Loslassen steht es. Eine Bahn, die man erst plant und dann
    # startet, waere zum Ertasten des Arbeitsraums unbrauchbar.
    #
    # Die Schleife haengt an der Antwort und nicht an einer Uhr. Bei 20 Hz
    # ueber /compute_ik stauten sich sonst Anfragen, sobald der Solver
    # laenger braucht als der Takt -- und genau dort, wo es interessant wird,
    # in Singularitaetsnaehe, braucht er laenger. Der Weg je Schritt kommt
    # deshalb aus der verstrichenen Zeit: die Geschwindigkeit stimmt dann
    # unabhaengig davon, wie schnell geantwortet wird.

    def _jog_ready(self):
        """(True, "") wenn gejoggt werden darf, sonst (False, Grund)."""
        if self.stopped:
            return False, "STOP steht an"
        if self.jog_key is not None or self.jog_joint is not None:
            return False, "es wird bereits gejoggt"
        if self.state != STATE_READY:
            return False, f"Zustand ist {self.state}"
        if not self.ros.moveit_ready:
            return False, "move_group antwortet nicht"
        if self.moveit_stale:
            return False, "move_group liest ein anderes Modell"
        return True, ""

    def _jog_press(self, key, sign):
        """Taste gedrueckt: ein Schritt sofort, Dauerfahrt nach kurzer Frist."""
        ok, why = self._jog_ready()
        if not ok:
            self._log(f"Joggen nicht moeglich: {why}", "warn")
            return

        pose = self._tcp_pose()
        if pose is None:
            self._log("Kein eindeutiger Endeffektor, es wird nicht gejoggt",
                      "error")
            return
        xyz, abc = pose
        self.jog_pose = {"x": xyz[0], "y": xyz[1], "z": xyz[2],
                         "a": abc[0], "b": abc[1], "c": abc[2]}

        # In der Entartung laufen zwei der drei Drehtasten auf dieselbe
        # Bewegung. Das ist kein Fehler und wird deshalb nicht verhindert,
        # sondern gesagt.
        if key in ANGLE_KEYS and ANZEIGE.entartet(abc):
            self._log(f"B steht nahe an der Entartung von {ANZEIGE.name}: "
                      f"A und C drehen dort um dieselbe Achse", "warn")
        self.jog_key, self.jog_sign = key, sign
        self.jog_hold = False
        self.jog_hold_job = self.after(JOG_HOLD_DELAY_MS, self._jog_begin_hold)
        self._jog_step(discrete=True)

    def _jog_begin_hold(self):
        """Aus dem Klick wird ein Halten."""
        self.jog_hold_job = None
        if self.jog_key is None:
            return
        self.jog_hold = True
        self.jog_last = time.monotonic()
        if not self.jog_pending:
            self._jog_step()

    def _jog_release(self, _event=None):
        """Taste losgelassen. Eine noch offene Antwort wird angewandt.

        Sie zu verwerfen waere schlechter: der Roboter stuende dann an einer
        Stelle, die keiner gerechneten Achsstellung entspricht.
        """
        self._jog_stop("")

    def _jog_stop(self, reason):
        """Joggen beenden. Einzige Stelle, an der die Schleife abreisst."""
        if self.jog_hold_job is not None:
            self.after_cancel(self.jog_hold_job)
            self.jog_hold_job = None
        was = self.jog_key
        self.jog_key = None
        self.jog_sign = 0
        self.jog_hold = False
        self.jog_pose = None
        self.jog_last = None
        if was is not None and reason:
            self._log(f"Joggen beendet: {reason}", "warn")

    def _jog_step(self, discrete=False):
        """Sollpose um einen Schritt weiterschieben und die IK fragen."""
        if self.jog_key is None or self.jog_pose is None:
            return
        key = self.jog_key
        rotational = key in ANGLE_KEYS

        if discrete:
            delta = math.radians(JOG_STEP_DEG) if rotational else JOG_STEP_M
        else:
            now = time.monotonic()
            elapsed = min(now - (self.jog_last or now), JOG_MAX_DT_S)
            self.jog_last = now
            speed = (math.radians(JOG_VEL_DEG_S) if rotational
                     else JOG_VEL_M_S)
            delta = speed * elapsed
        if delta <= 0.0:
            return

        self.jog_pose[key] += self.jog_sign * delta
        built = self._ik_payload(self.jog_pose)
        if built is None:
            self._jog_stop("keine Kette")
            return

        self.jog_delta = delta
        self.jog_before = {n: float(v) for n, v in self.positions.items()}
        self.jog_pending = True
        self.ros.request_ik(built[2])

    def _on_jog_result(self, result):
        """Antwort auf einen Jog-Schritt. Setzt die Achsen unmittelbar."""
        self.jog_pending = False

        if result.get("error") or int(result.get("code", 0)) != 1:
            text = result.get("error") or moveit_error_text(
                int(result.get("code", 0)))
            self._jog_stop("")
            self._log(f"Joggen gestoppt, keine IK-Loesung: {text}", "error")
            self.jog_var.set("keine Loesung -- Grenze des Arbeitsraums "
                             "oder Singularitaet")
            return

        values = result.get("values") or {}
        wanted = [j["name"] for j in self.joints
                  if j["type"] != "fixed" and j["mimic"] is None]
        missing = [n for n in wanted if n not in values]
        if missing:
            self._jog_stop("")
            self._log(f"Die IK-Antwort fuehrt {', '.join(missing)} nicht",
                      "error")
            return

        before = self.jog_before or {}
        for name in wanted:
            self.positions[name] = values[name]
        self._push_positions()
        self._sync_sliders()
        self._sync_joint_entries()
        self._update_kinematics()
        self._jog_sync_targets()
        self._jog_report(before, values, wanted)

        # Nachladen, solange die Taste liegt. Nur hier -- deshalb kann sich
        # nichts stauen, egal wie lange der Solver braucht.
        if self.jog_key is not None and self.jog_hold:
            self._jog_step()

    def _jog_report(self, before, values, wanted):
        """Verstaerkung anzeigen: wieviel Achsweg kostet ein Millimeter Bahn.

        Kein Umweg ueber die Jacobi-Matrix und keine Singulaerwertzerlegung.
        Beide Groessen fallen bei jedem Schritt ohnehin an, und der Quotient
        ist genau die Zahl, die hinter einer Pilz-Meldung wie "Achsgeschwin-
        digkeit 15.88 bei Grenze 1" steckt: nicht zu schnell gefahren,
        sondern an einer Stelle, an der ein kleiner Bahnweg viel Achsweg
        kostet.

        Er waechst in Singularitaetsnaehe ueber alle Grenzen -- das ist die
        Singularitaet, in einer ablesbaren Zahl. Optisch ist an dieser Stelle
        nichts zu sehen, und genau deshalb steht sie hier.
        """
        step = self.jog_delta
        if step <= 0.0:
            return
        rotational = self.jog_key in ANGLE_KEYS
        speed = math.radians(JOG_VEL_DEG_S) if rotational else JOG_VEL_M_S
        unit = "rad/rad" if rotational else "rad/m"
        source = "deg/s" if rotational else "m/s"
        shown_speed = JOG_VEL_DEG_S if rotational else JOG_VEL_M_S

        worst_name, worst_ratio, worst_share = "", 0.0, 0.0
        total = 0.0
        for name in wanted:
            change = abs(values[name] - before.get(name, 0.0))
            total += change * change
            ratio = change / step
            if ratio > worst_ratio:
                worst_name, worst_ratio = name, ratio
            limit = (self.moveit_config.limits.get(name) or {}).get("velocity")
            if limit:
                share = ratio * speed / float(limit)
                worst_share = max(worst_share, share)

        text = (f"Verstaerkung {math.sqrt(total) / step:7.1f} {unit}   "
                f"schnellste Achse {worst_name or '-'} "
                f"{worst_ratio * speed:6.3f} rad/s bei "
                f"{shown_speed:g} {source}")
        if worst_share > 0.0:
            text += f"   {worst_share * 100:.0f} % der Achsgrenze"
        self.jog_var.set(text + self._jog_warn(worst_share, worst_name))

    def _jog_warn(self, share, worst):
        """Warnstufe setzen und einmalig melden. Gibt den Zusatztext zurueck.

        Zwei Schwellen statt einer. Jeder Schritt ist eine eigene IK-Loesung,
        und die Auslastung schwankt deshalb auch bei ruhiger Bewegung um
        einige Prozent. Mit einer einzelnen Schwelle bei 75 Prozent kaeme die
        Meldung mehrmals in der Sekunde und ginge damit unter -- dieselbe
        Ueberlegung wie beim Schmitt-Trigger, und dieselbe Loesung.

        Gemeldet wird nur der Uebergang, nicht der Zustand. Eine Warnung, die
        bei jedem Schritt wiederholt wird, ist keine Warnung mehr.

        Der Begriff ist mit Bedacht gewaehlt: gemessen wird die Verstaerkung
        zwischen Bahn- und Achsweg, nicht die Singularitaet selbst. Nahe
        einer Singularitaet waechst sie ueber alle Grenzen, das ist der
        haeufigste Grund. Sie waechst aber auch bei einer Achse mit
        auffaellig niedriger Grenze -- und dann steht das Problem in der
        URDF und nicht in der Mechanik.
        """
        if share <= 0.0:
            self._jog_level("Tcp.TLabel")
            return ""

        if share > 1.0:
            self._jog_level("TcpBad.TLabel")
            if not self.jog_warned:
                self.jog_warned = True
                self._log(f"{worst} liegt {share:.1f}-fach ueber seiner "
                          f"Geschwindigkeitsgrenze. Eine Gerade durch diese "
                          f"Stellung lehnt Pilz ab.", "error")
            return f"   UEBER DER ACHSGRENZE ({share:.1f}-fach)"

        if share >= JOG_WARN_SHARE:
            self._jog_level("TcpWarn.TLabel")
            if not self.jog_warned:
                self.jog_warned = True
                self._log(f"Singularitaetsnaehe: {worst} laeuft bei "
                          f"{share * 100:.0f} Prozent seiner Grenze, obwohl "
                          f"langsam gejoggt wird. Hier kostet wenig Bahnweg "
                          f"viel Achsweg.", "warn")
            return "   SINGULARITAETSNAEHE"

        if share < JOG_CLEAR_SHARE and self.jog_warned:
            self.jog_warned = False
            self._log(f"Achsauslastung wieder unter "
                      f"{JOG_CLEAR_SHARE * 100:.0f} Prozent", "ok")

        # Zwischen den Schwellen bleibt stehen, was gilt: einmal gewarnt,
        # bleibt die Zeile gefaerbt, bis wirklich Abstand da ist.
        self._jog_level("TcpWarn.TLabel" if self.jog_warned
                        else "Tcp.TLabel")
        return "   SINGULARITAETSNAEHE" if self.jog_warned else ""

    def _jog_level(self, style):
        if self.jog_label is not None and self.jog_label.winfo_exists():
            self.jog_label.configure(style=style)

    def _jog_sync_targets(self):
        """Beide Zielfeldsaetze auf den erreichten Stand nachziehen.

        Waehrend des Joggens ist das Ziel der Ist-Stand: der Roboter steht
        dort, wo zuletzt hingefahren wurde. Die Felder behalten damit ihre
        Bedeutung -- sie beschreiben weiterhin ein Ziel, nur eben das gerade
        erreichte. Ohne dieses Nachziehen stuende nach zwanzig Klicks eine
        Lage darin, die niemand mehr meint, und ein anschliessendes "Planen"
        fuehre wortlos dorthin zurueck.

        Bewusst ohne _drive_write_cart: das ruft _update_locks, und das geht
        durch saemtliche Felder, Schieber und Tasten. Zwanzigmal in der
        Sekunde ist das spuerbar, und zu tun gibt es dabei nichts.
        """
        pose = self._tcp_pose()
        if pose is not None:
            xyz, abc = pose
            values = {"x": xyz[0], "y": xyz[1], "z": xyz[2],
                      "a": math.degrees(abc[0]),
                      "b": math.degrees(abc[1]),
                      "c": math.degrees(abc[2])}
            for key, _label, unit in CART_FIELDS:
                entry = self.drive_cart_entries.get(key)
                if entry is None or not entry.winfo_exists():
                    continue
                if str(entry.cget("state")) != "normal":
                    continue
                entry.delete(0, "end")
                entry.insert(0, f"{values[key]:.{4 if unit == 'm' else 2}f}")

        for joint in self.joints:
            if joint["type"] == "fixed" or joint["mimic"] is not None:
                continue
            entry = self.drive_entries.get(joint["name"])
            if entry is None or not entry.winfo_exists():
                continue
            if str(entry.cget("state")) != "normal":
                continue
            value = self.positions.get(joint["name"], 0.0)
            shown = math.degrees(value) if is_rotational(joint) else value
            entry.delete(0, "end")
            entry.insert(0, f"{shown:.{value_digits(slider_step(joint))}f}")

    # -- Achsjoggen ---------------------------------------------------------
    #
    # Derselbe Griff, anderer Weg: hier wird die Achse unmittelbar gestellt,
    # ohne /compute_ik und ohne move_group. Es ist der Weg, den die
    # Sollwertfelder links seit jeher gehen, nur mit einer Taste statt einer
    # Zahl -- und deshalb funktioniert er auch, wenn MoveIt gar nicht laeuft.
    #
    # Die Schleife haengt an einer Uhr und nicht an einer Antwort, weil es
    # keine gibt, auf die zu warten waere. Ein Stau kann so nicht entstehen.

    def _jogj_ready(self):
        if self.stopped:
            return False, "STOP steht an"
        if self.jog_key is not None or self.jog_joint is not None:
            return False, "es wird bereits gejoggt"
        if self._busy():
            return False, f"es laeuft {self.lock_reason}"
        return True, ""

    def _jogj_press(self, name, sign):
        ok, why = self._jogj_ready()
        if not ok:
            self._log(f"Joggen nicht moeglich: {why}", "warn")
            return
        joint = self._joint_by_name(name)
        if joint is None or joint["type"] == "fixed" or joint["mimic"] is not None:
            return
        self.jog_joint, self.jog_joint_sign = name, sign
        self.jogj_hold = False
        self.jogj_hold_job = self.after(JOG_HOLD_DELAY_MS,
                                        self._jogj_begin_hold)
        self._jogj_step(discrete=True)

    def _jogj_begin_hold(self):
        self.jogj_hold_job = None
        if self.jog_joint is None:
            return
        self.jogj_hold = True
        self.jogj_last = time.monotonic()
        self._jogj_step()

    def _jogj_release(self, _event=None):
        self._jogj_stop("")

    def _jogj_stop(self, reason):
        """Achsjoggen beenden. Einzige Stelle, an der der Takt abreisst."""
        for attribute in ("jogj_hold_job", "jogj_job"):
            job = getattr(self, attribute, None)
            if job is not None:
                self.after_cancel(job)
                setattr(self, attribute, None)
        was = self.jog_joint
        self.jog_joint = None
        self.jog_joint_sign = 0
        self.jogj_hold = False
        self.jogj_last = None
        if was is not None and reason:
            self._log(f"Achsjoggen beendet: {reason}", "warn")

    def _jogj_step(self, discrete=False):
        """Eine Achse um einen Schritt weiterstellen."""
        self.jogj_job = None
        name = self.jog_joint
        if name is None:
            return
        joint = self._joint_by_name(name)
        if joint is None:
            self._jogj_stop("Achse gibt es nicht mehr")
            return

        rotational = is_rotational(joint)
        if discrete:
            delta = math.radians(JOG_STEP_DEG) if rotational else JOG_STEP_M
        else:
            now = time.monotonic()
            elapsed = min(now - (self.jogj_last or now), JOG_MAX_DT_S)
            self.jogj_last = now
            speed = math.radians(JOG_VEL_DEG_S) if rotational else JOG_VEL_M_S
            delta = speed * elapsed
        if delta <= 0.0:
            self._jogj_arm()
            return

        value = self.positions.get(name, 0.0) + self.jog_joint_sign * delta
        if joint["has_limit"]:
            limited = min(max(value, joint["lower"]), joint["upper"])
            if abs(limited - value) > 1e-12:
                # Am Anschlag weiterzutakten hiesse, eine Taste zu halten,
                # unter der sich nichts mehr bewegt. Lieber anhalten und es
                # sagen.
                self.positions[name] = limited
                self._jogj_apply()
                self._jogj_stop("")
                self._log(f"{name} steht am Anschlag", "warn")
                return
            value = limited

        self.positions[name] = value
        self._jogj_apply()
        if self.jogj_hold:
            self._jogj_arm()

    def _jogj_arm(self):
        self.jogj_job = self.after(JOG_TICK_MS, self._jogj_step)

    def _jogj_apply(self):
        """Derselbe Sendeweg wie ueberall: Kopplungen, TF, Anzeige."""
        self._push_positions()
        self._sync_sliders()
        self._sync_joint_entries()
        self._update_kinematics()
        self._jog_sync_targets()

    def _ik_payload(self, pose, seed=None):
        """(Bezug, Tip, Anfrage) aus sechs Zahlen in SI. None heisst: geht nicht.

        Seit 2.17 an einer Stelle, weil es mehrere Aufrufer gibt: das Planen
        mit einem kartesischen Ziel, das Joggen und seit 3.2 die Kette vor
        einer Punktfolge. Zwei Fassungen derselben Anfrage waeren zwei
        Stellen, an denen der Startwert vergessen werden kann -- und ohne
        ihn waehlt der Solver bei einer redundanten Kette jedes Mal eine
        andere Loesung.

        seed ueberschreibt genau diesen Startwert. Ohne Angabe gilt der
        Achsstand des Roboters, und das ist fuer Fahrfenster und Joggen
        richtig: dort wird von dort aus gefahren, wo der Arm steht. In
        einer Folge steht er beim zweiten Punkt aber laengst nicht mehr
        dort, wo er beim dritten stehen wird. Der Startwert ist dann die
        Loesung des Vorgaengers -- und weil sie hier hineingereicht werden
        kann, laesst sich die Kette von aussen fuehren. Genau das ist bei
        Pilz nicht moeglich.
        """
        base, tip = self._drive_ik_frames()
        if not tip:
            self._log("Kein eindeutiger Tip-Link. Im Parameterfenster unter "
                      "Kinematik eintragen.", "error")
            return None
        if not base:
            self._log("Kein Wurzellink gefunden, die Lage haette keinen "
                      "Bezug", "error")
            return None

        # Drehung als Quaternion. Reine Mathematik, deshalb hier und nicht
        # drueben: der Executor-Thread baut Nachrichten, er rechnet nicht.
        # Ueber die Anzeigekonvention, nicht ueber from_rpy_xyz: die drei
        # Zahlen kommen aus einem Feld, das in der gewaehlten Konvention
        # beschriftet ist. Waeren sie hier als rpy gelesen, ergaebe dasselbe
        # Zielfeld je nach Einstellung eine andere Lage -- ohne Meldung.
        matrix = ANZEIGE.baue((pose["a"], pose["b"], pose["c"]),
                              (pose["x"], pose["y"], pose["z"]))

        # Startwert. Gekoppelte Gelenke gehen mit heraus -- MoveIt erwartet
        # sie im Zustand, auch wenn sie keinen Freiheitsgrad haben.
        if seed is None:
            apply_couplings(self.joints, self.positions)
            seed = {j["name"]: float(self.positions.get(j["name"], 0.0))
                    for j in self.joints if j["type"] != "fixed"}
        else:
            seed = {name: float(value) for name, value in seed.items()}

        attempts = max(1, int(self.moveit_config.plan["planning_attempts"]))
        solver_timeout = max(
            0.001,
            float(self.moveit_config.kinematics["kinematics_solver_timeout"]))
        payload = {
            "group": self.moveit_config.group,
            "tip": tip,
            "frame": base,
            "position": (pose["x"], pose["y"], pose["z"]),
            "orientation": to_quaternion(matrix),
            "seed": seed,
            "avoid_collisions": bool(self.moveit_config.check_collisions),
            "solver_timeout": solver_timeout,
        }
        payload["timeout"] = solver_timeout * attempts + IK_TIMEOUT_MARGIN_S
        return base, tip, payload

    def _ik_request(self):
        """Kartesisches Ziel an /compute_ik geben. Es faehrt noch nichts.

        Die Anfrage traegt drei Dinge, die man leicht fuer Beiwerk haelt und
        die es nicht sind:

            der Tip-Link      auf ihn bezieht sich die Lage. Steht dort ein
                              anderer Link als in der SRDF-Kette, antwortet
                              der Dienst mit "Linkname ungueltig" -- oder,
                              schlimmer, mit einer Loesung fuer den falschen
                              Punkt
            der Startwert     bei sieben Achsen ist die Loesung nicht
                              eindeutig. Welche herauskommt, entscheidet der
                              Startwert; deshalb geht der aktuelle Stand mit
            die Zeitgrenze    KDL sucht numerisch und gibt nach dieser Zeit
                              auf. Sie steht im Parameterfenster unter
                              Kinematik und wirkt hier unmittelbar
        """
        pose = self._read_cart_target()
        if pose is None:
            return

        built = self._ik_payload(pose)
        if built is None:
            return
        base, tip, payload = built
        # Nur der Weg ueber das Planen bekommt eine Nummer. Ein Jog-Schritt
        # geht durch _ik_payload ohne diese Zeile und wird in _on_ik_result
        # vor der Nummernpruefung abgezweigt -- er haengt nicht an der
        # Zustandsmaschine und darf auch nicht an ihrer Buchhaltung haengen.
        payload["tag"] = self._job_open()

        self._drop_plan()
        # Mit der Konvention, in der die Zahlen gemeint waren. Eine
        # Diagnose, die nach einem Umschalten gelesen wird, zeigt sonst
        # dieselben Zahlen unter einer anderen Vorschrift an.
        self.ik_asked = dict(pose, konvention=ANZEIGE.name)
        self.ik_frames = (base, tip)
        self.ik_started = time.monotonic()
        self._set_state(STATE_IK, tip)
        self.ros.request_ik(payload)

    def _on_ik_result(self, result):
        """Antwort von /compute_ik. Kommt ueber _tick aus dem Executor-Thread.

        Bei Erfolg geht es ohne weiteren Tastendruck in die Planung. Das ist
        kein zweiter Weg an der Bedienung vorbei: der Bediener hat "Planen"
        gedrueckt, und die IK ist ein Rechenschritt darin, kein eigener
        Auftrag. Sichtbar bleibt sie trotzdem -- die Loesung steht danach in
        den Achszielfeldern und im Protokoll.
        """
        # Ein Jog-Schritt laeuft ueber denselben Dienst, aber nicht ueber die
        # Zustandsmaschine: _set_state protokolliert jeden Wechsel, und bei
        # einem Schritt alle fuenfzig Millisekunden waere das Meldungsfenster
        # nach zehn Sekunden nur noch Rauschen.
        if self.jog_pending:
            self._on_jog_result(result)
            return

        # Die Kette vor einer Punktfolge geht ueber denselben Dienst und
        # denselben Ablageort. Getrennt wird sie am Zustand: alles darunter
        # schreibt in die Zielfelder des Fahrfensters und plant im
        # Anschluss, und beides waere hier falsch.
        if self.state == STATE_SEQ_IK:
            self._on_seq_ik_result(result)
            return

        if self.state != STATE_IK:
            return                                     # abgebrochen, verworfen
        if not self._job_mine(result):
            self._log("Verspaetete IK-Antwort verworfen: sie gehoert zu "
                      "einem abgebrochenen Auftrag", "warn")
            return
        self._job_drop()

        took = (time.monotonic() - self.ik_started
                if self.ik_started else 0.0)
        self.ik_started = None

        # Wie auf der Planseite: jeder Weg ohne Loesung laesst den Taster
        # fallen. Die IK ist der erste von zwei Schritten, und ein Taster,
        # der den zweiten noch erwartet, waere eine Zusage ohne Deckung.
        if result.get("error"):
            self._hold_stop("die IK ist gescheitert")
            self._set_state(STATE_ERROR, "IK")
            self._log(f"Keine IK-Loesung: {result['error']}", "error")
            self._ik_diagnosis()
            return

        code = int(result.get("code", 0))
        if code != 1:
            self._hold_stop("die IK ist gescheitert")
            self._set_state(STATE_ERROR, "IK")
            self._log(f"Keine IK-Loesung: {moveit_error_text(code)}", "error")
            self._ik_diagnosis()
            return

        values = result.get("values") or {}
        wanted = [j["name"] for j in self.joints
                  if j["type"] != "fixed" and j["mimic"] is None]
        missing = [name for name in wanted if name not in values]
        if missing:
            # Der Dienst hat geantwortet, aber fuer eine andere Kette. Das
            # ist kein IK-Fehler, sondern ein Konfigurationsfehler, und er
            # gehoert anders benannt.
            self._hold_stop("die Antwort passt nicht zur Gruppe")
            self._set_state(STATE_ERROR, "IK")
            self._log(f"Die Antwort fuehrt {', '.join(missing)} nicht. "
                      f"Gehoeren diese Achsen zur Gruppe "
                      f"{self.moveit_config.group}?", "error")
            self._ik_diagnosis()
            return

        self._set_state(STATE_READY)
        self._drive_write_targets(values)

        shown = []
        for name in wanted:
            joint = self._joint_by_name(name)
            value = values[name]
            if joint is not None and is_rotational(joint):
                shown.append(f"{name} {math.degrees(value):.2f} deg")
            else:
                shown.append(f"{name} {value:.4f} m")
        self._log(f"IK in {took * 1000:.0f} ms geloest: " + ", ".join(shown),
                  "ok")
        self._log("Die Loesung steht jetzt als Achsziel im Fahrfenster. Von "
                  "hier an ist der Weg derselbe wie bei einem getippten "
                  "Achsziel.", "info")
        self._plan_joint_goal()

    def _ik_diagnosis(self):
        """Nach einer erfolglosen IK zeigen, wonach gefragt wurde.

        Dieselbe Absicht wie bei _plan_diagnosis: was gefragt wurde, weiss
        die GUI. Warum nichts herauskam, weiss der Solver -- und der sagt es
        allein ueber einen Fehlercode, der drei sehr verschiedene Faelle
        zusammenfasst.
        """
        asked = self.ik_asked or {}
        base, tip = self.ik_frames or ("?", "?")
        if asked:
            self._log("Gefragt war: X {0:.4f}  Y {1:.4f}  Z {2:.4f} m   "
                      "A {3:.2f}  B {4:.2f}  C {5:.2f} deg  {6}".format(
                          asked["x"], asked["y"], asked["z"],
                          math.degrees(asked["a"]),
                          math.degrees(asked["b"]),
                          math.degrees(asked["c"]),
                          asked.get("konvention", ANZEIGE.name)), "warn")
        self._log(f"Bezug {base}, Tip-Link {tip}, Gruppe "
                  f"{self.moveit_config.group}, Solver "
                  f"{self.moveit_config.solver.split('/')[-1]}, Zeitgrenze "
                  f"{self.moveit_config.kinematics['kinematics_solver_timeout']:g} s",
                  "warn")
        self._log("Keine Loesung heisst dreierlei, und die Faelle sind zu "
                  "unterscheiden: die Lage liegt ausserhalb des Arbeitsraums; "
                  "die Lage ist erreichbar, die Drehung mit dieser Kette "
                  "aber nicht; oder der Solver hatte in seiner Zeitgrenze zu "
                  "wenig Versuche. Der dritte Fall ist der einzige, der sich "
                  "durch Warten loesen laesst -- Zeitgrenze im "
                  "Parameterfenster unter Kinematik.", "warn")
        self._moveit_log_tail(8)

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

    def _start_playing(self, player, from_hold=False):
        """Eine Bahn abtasten. Der eine Weg in eine Bewegung, fuer beide Wege.

        from_hold sagt, ob die Fahrt aus dem Halte-Taster kam. Sie unterliegt
        dann einer zusaetzlichen Bedingung, die _motion_tick in jedem Takt
        prueft: sie laeuft nur, solange der Taster unten ist.
        """
        self.plan_hold = None                          # er wird jetzt gefahren
        self.plan_hold_start = None
        self.hold_drive = bool(from_hold)
        self.drive_plan_var.set("wird gefahren"
                                + (", Taster gehalten" if from_hold else ""))
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

        # Der zweite Wachhund, und der eigentliche. _hold_watch fragt alle
        # 100 ms nach dem Zeiger; hier steht die Bedingung selbst, in jedem
        # Takt, an der Stelle, an der die Bewegung entsteht: eine Fahrt aus
        # dem Halte-Taster laeuft nur, solange der Taster unten ist. Wie
        # hold_run gefallen ist -- Loslassen, Wachhund, geschlossenes
        # Fenster, STOP -- spielt hier keine Rolle mehr.
        if self.hold_drive and not self.hold_run and not player.braking:
            self._brake("Halte-Taster nicht mehr gehalten")

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
        # Die Fahrt ist vorbei, ihre Herkunft damit auch. Der Taster kann
        # dabei durchaus noch unten sein: eine Bahn endet an ihrem Ziel, und
        # Halten faehrt sie nicht weiter. hold_run bleibt deshalb stehen und
        # faellt erst beim Loslassen -- sonst naehme der Taster sofort einen
        # zweiten Auftrag an, ohne dass jemand die Hand gehoben hat.
        was_hold = self.hold_drive
        self.hold_drive = False
        if self.motion_job is not None:
            self.after_cancel(self.motion_job)
            self.motion_job = None
        self._update_kinematics()
        self._set_state(STATE_READY)
        if was_hold and self.hold_run:
            self.drive_plan_var.set("Ziel erreicht, Taster noch gehalten")
            self._log("Die Bahn ist zu Ende. Halten faehrt nicht weiter -- "
                      "zum naechsten Auftrag erst loslassen.", "info")

    def _brake(self, why):
        """Anhalten einleiten. Die Bahn wird nicht verlassen, nur langsamer.

        Ein Sprung auf null waere genau die Grenzverletzung, um die es in
        dieser Version geht. Deshalb faehrt die Zeitrate ueber BRAKE_TIME_S
        auf null, und der Roboter bleibt dabei auf seiner Bahn stehen.
        """
        # Das Joggen haengt nicht an der Zustandsmaschine und muss deshalb
        # ausdruecklich fallen. Sonst liefe die Schleife weiter, waehrend die
        # Oberflaeche gesperrt meldet.
        self._jog_stop(why)
        self._jogj_stop(why)
        # Derselbe Grund beim Halte-Taster, und er wiegt schwerer: bremsen
        # heisst anhalten, und ein Taster, der danach noch als gehalten
        # gilt, liesse die naechste Planantwort sofort wieder losfahren.
        # Wer bremst, hat das nicht gemeint -- gleich, ob er STOP gedrueckt
        # hat, das Modell wechselt oder die Konvention umschaltet.
        self._hold_stop(why)

        if self.state == STATE_SEQ_IK:
            # Die Kette laeuft ueber mehrere Anfragen. Sie bricht deshalb
            # nicht dadurch ab, dass eine Antwort verworfen wird -- die
            # naechste wuerde sonst trotzdem gestellt. Der Zustand wechselt,
            # und _seq_ik_next wird nicht mehr gerufen.
            self.ik_started = None
            self.seq_started = None
            self._job_drop()
            offen = self.seq_ik_index + 1
            gesamt = len(self.seq_schritte)
            self._seq_ik_reset()
            self._set_state(STATE_READY, "Folge verworfen")
            self._log(f"Folgen-IK abgebrochen bei Punkt {offen} von "
                      f"{gesamt} ({why})", "warn")
            return
        if self.state == STATE_IK:
            # Dieselbe Regel wie beim Planen: die Antwort kommt noch, aber
            # _on_ik_result laesst sie fallen, weil der Zustand nicht mehr IK
            # ist. Abgebrochen wird die Rechnung im Solver dadurch nicht --
            # sie dauert Millisekunden, und ein Abbruch waere teurer als das
            # Ergebnis.
            self.ik_started = None
            self._job_drop()
            self._set_state(STATE_READY, "IK verworfen")
            self._log(f"IK abgebrochen ({why})", "warn")
            return
        if self.state == STATE_PLANNING:
            # Der Auftrag laeuft schon. Die Antwort wird verworfen, weil der
            # Zustand nicht mehr PLANEN ist.
            self.plan_started = None
            self.plan_hold_start = None
            self.plan_hold_goal = None
            self._job_drop()
            self.drive_plan_var.set("kein Plan")
            self._set_state(STATE_READY, "Planung verworfen")
            self._log(f"Planung abgebrochen ({why})", "warn")
            return
        if self.player is None or self.state != STATE_MOVING:
            return
        self.player.brake()
        self._set_state(STATE_BRAKING, why)

    # ----------------------------------------------------------------------
    # Halte-Taster
    # ----------------------------------------------------------------------
    #
    # Drei Handlungen auf einer Taste, und zwei Wachhunde darunter. Der Weg
    # selbst ist kurz: druecken ruft _drive_plan, die Antwort landet in
    # _on_plan_result, und weil hold_run steht, geht sie ohne weiteren
    # Tastendruck in _start_playing. Loslassen ruft _brake. Ein zweiter
    # Fahrweg entsteht dabei nicht -- es ist derselbe, nur ohne die Pause
    # dazwischen.
    #
    # Der Aufwand steckt in dem, was zwischen Druck und Antwort passieren
    # kann, und das ist mehr, als man beim Hinsehen vermutet.

    def _hold_ready(self):
        """(True, "") wenn der Taster jetzt einen Auftrag stellen darf."""
        if self.hold_run:
            return False, "er ist schon gedrueckt"
        if self.stopped:
            return False, "STOP steht an"
        if self.jog_key is not None or self.jog_pending:
            return False, "es wird gerade kartesisch gejoggt"
        if self.jog_joint is not None:
            return False, "es wird gerade eine Achse gejoggt"
        if self.state == STATE_BRAKING:
            # Der haeufigste abgelehnte Fall, und der einzige, der wie ein
            # Fehler aussieht und keiner ist. Die Bahn wird gerade auf null
            # gefuehrt; ihr Startpunkt gilt nicht mehr, und der neue steht
            # erst fest, wenn sie steht.
            return False, (f"es wird noch gebremst, das dauert bis zu "
                           f"{BRAKE_TIME_S:.1f} s")
        if self.state in STATES_BUSY:
            return False, f"Zustand ist {self.state}"
        if not self.ros.moveit_ready:
            return False, "move_group antwortet nicht"
        if self.moveit_stale:
            return False, "move_group liest ein anderes Modell"
        return True, ""

    def _hold_press(self, _event=None):
        """Taster gedrueckt: planen, und bei Erfolg unmittelbar fahren."""
        ok, why = self._hold_ready()
        if not ok:
            self._log(f"Halte-Taster nimmt nicht an: {why}", "warn")
            return

        self.hold_run = True
        self.drive_plan_var.set("Taster gehalten, es wird geplant")
        self._log("Halte-Taster gedrueckt. Es wird geplant und, sobald die "
                  "Bahn da ist, gefahren -- solange der Taster unten bleibt.",
                  "info")
        self.hold_watch_job = self.after(HOLD_WATCH_MS, self._hold_watch)
        self._drive_plan()

        # _drive_plan kann ohne Auftrag zurueckkehren: unlesbares Zielfeld,
        # kein bewegliches Gelenk, Ziel gleich dem Stand. Dann steht der
        # Taster auf gehalten, ohne dass je eine Antwort kommt -- der
        # Wachhund liefe, die Anzeige loege. Die Bedingung ist nicht "es gab
        # keinen Fehler", sondern "es ist etwas unterwegs", und das steht im
        # Zustand.
        if self.state not in (STATE_PLANNING, STATE_IK):
            self._hold_stop("es ist kein Auftrag daraus geworden")

    def _hold_release(self, _event=None):
        """Taster losgelassen. Bremsen, gleich in welchem Schritt es steht.

        _brake trifft alle drei Faelle: eine laufende IK wird verworfen, eine
        laufende Planung ebenso, eine Fahrt wird ueber die Rampe angehalten.
        Steht nichts an, tut es nichts. Deshalb steht hier keine Fallunter-
        scheidung -- sie waere eine zweite Ausgabe derselben in _brake.
        """
        if not self.hold_run:
            return
        self._brake("Halte-Taster losgelassen")

    def _hold_stop(self, reason=""):
        """Den Taster logisch loslassen. Einzige Stelle, an der hold_run faellt.

        Logisch, nicht koerperlich: die Taste kann durchaus noch unten sein.
        Nach einem Abbruch ist das sogar der Regelfall, und es ist richtig
        so -- wer weiterfahren will, hebt die Hand und drueckt neu. Ein
        Taster, der sich von selbst wieder scharf macht, waere das Gegenteil
        von dem, wofuer es ihn gibt.
        """
        if self.hold_watch_job is not None:
            try:
                self.after_cancel(self.hold_watch_job)
            except Exception:
                pass
            self.hold_watch_job = None
        was = self.hold_run
        self.hold_run = False
        if not was:
            return
        if reason:
            self._log(f"Halte-Taster faellt: {reason}", "warn")
        self._update_locks()

    def _hold_abort(self, reason):
        """Halten aufgeben und dabei bremsen. Fuer alles, was von aussen kommt."""
        if not self.hold_run:
            return
        self._brake(f"Halte-Taster: {reason}")

    def _hold_window_event(self, event=None):
        """Fokus oder Sichtbarkeit des Fahrfensters hat sich geaendert.

        Der Filter auf das Fenster selbst ist noetig: Tk meldet FocusOut und
        Unmap auch fuer Kindwidgets, und ohne ihn braeche schon das Setzen
        des Fokus auf ein Zielfeld die Fahrt ab.
        """
        if not self.hold_run:
            return
        if (event is not None
                and getattr(event, "widget", None) is not self.drive_window):
            return
        self._hold_abort("das Fahrfenster ist nicht mehr im Vordergrund")

    def _hold_watch(self):
        """Nachsehen, ob der Taster wirklich noch gehalten wird.

        Der Wachhund, der nicht auf ein Ereignis wartet. Tk kann den Zustand
        der Maustasten nicht abfragen, es kann nur Ereignisse melden -- und
        genau das Ereignis, um das es geht, ist ja das ausgebliebene. Ein
        ButtonRelease geht verloren, wenn der Fenstermanager dazwischenkommt,
        ein fremdes grab_set den Zeiger holt oder die Taste unter dem Zeiger
        verschwindet. hold_run bliebe stehen, und der Roboter fuehre weiter,
        obwohl niemand mehr drueckt.

        Gefragt wird deshalb nach dem, was Tk beantworten kann: gibt es die
        Taste noch, ist sie sichtbar, steht das Fenster noch, und steht der
        Zeiger noch darueber. Der letzte Punkt ueberschneidet sich mit der
        Bindung an <Leave> -- absichtlich. Die Bindung ist der schnelle Weg,
        der Wachhund der, der auch dann greift, wenn das Leave nicht kam.

        Was er nicht kann: einen Zeiger erkennen, der still auf der Taste
        steht, waehrend die Maustaste laengst oben ist. Diesen Fall deckt
        allein das ButtonRelease ab. Er ist selten -- er verlangt, dass das
        Loslassen verlorengeht, ohne dass sich sonst etwas aendert -- und er
        ist der Grund, warum in _motion_tick zusaetzlich die Invariante
        selbst steht.
        """
        self.hold_watch_job = None
        if not self.hold_run:
            return

        button = self.hold_button
        lost = None
        if button is None or not button.winfo_exists():
            lost = "die Taste gibt es nicht mehr"
        elif self.drive_window is None or not self.drive_window.winfo_exists():
            lost = "das Fahrfenster ist zu"
        elif not button.winfo_ismapped():
            lost = "die Taste ist nicht mehr sichtbar"
        else:
            try:
                under = self.winfo_containing(*self.winfo_pointerxy())
            except Exception:
                # Zeiger auf einem anderen Bildschirm oder in einem fremden
                # Fenster: keine Auskunft ist hier dasselbe wie ein Nein.
                under = None
            if under is not button:
                lost = "der Zeiger steht nicht mehr auf der Taste"

        if lost is not None:
            self._hold_abort(lost)
            return
        self.hold_watch_job = self.after(HOLD_WATCH_MS, self._hold_watch)

    def _hold_launch(self, player, took, count):
        """Die frisch geplante Bahn unmittelbar anfahren.

        Gerufen aus _on_plan_result, wenn der Taster beim Eintreffen der
        Antwort noch unten ist. Die Driftpruefung steht hier genauso wie in
        _drive_start und aus demselben Grund: eine Bahn gehoert zu dem
        Achsstand, aus dem heraus sie gerechnet wurde. Waehrend PLANEN sind
        die Sollwertfelder gesperrt, sie sollte also nie ausloesen -- und
        genau deshalb bleibt sie stehen. Eine Pruefung, die nur dann etwas
        findet, wenn eine Annahme gebrochen ist, ist die, die man behaelt.
        """
        start = self.plan_hold_start or {}
        drift = max((abs(self.positions.get(name, 0.0) - value)
                     for name, value in start.items()), default=0.0)
        if drift > 1e-9:
            self.plan_hold_start = None
            self.plan_hold_goal = None
            self.drive_plan_var.set("kein Plan")
            self._set_state(STATE_READY)
            self._log("Achsstand hat sich seit dem Planen geaendert. Es wird "
                      "nicht gefahren -- der erste Stuetzpunkt waere ein "
                      "Sprung.", "error")
            self._hold_stop("Achsstand hat sich geaendert")
            return

        self._log(f"Plan in {took * 1000:.0f} ms, {count} Stuetzpunkte, "
                  f"Bahndauer {player.duration:.3f} s. Der Taster ist unten, "
                  f"es wird gefahren.", "ok")
        self._start_playing(player, from_hold=True)

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
                 ("Pipeline", self._build_param_pipeline),
                 ("Anzeige", self._build_param_display)]
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

        # Eigene Ueberschrift statt einer weiteren Zeile bei den Bahngrenzen.
        # Die Abtastzeit ist keine Grenze, sondern ein Takt; steht sie in
        # derselben Gruppe, liest sie sich als Obergrenze, und genau das ist
        # sie nicht.
        basis = len(CARTESIAN_FIELDS) + 2
        ttk.Label(page, text="ABTASTUNG   (Neustart noetig)",
                  style="Section.TLabel").grid(row=basis, column=0,
                                               columnspan=3, sticky="w",
                                               pady=(18, 10))
        for row, (key, label, unit, _default) in enumerate(PLANNER_FIELDS,
                                                           start=basis + 1):
            self._param_row(page, row, key, label, unit)
        self._param_note(
            page, basis + len(PLANNER_FIELDS) + 1,
            "Wirkt nur mit gepatchtem Pilz-Planer, sonst bleibt es bei fest "
            "verdrahteten 0,1 s. Halbierung der Abtastzeit verdoppelt die Zahl "
            "der IK-Aufrufe je LIN-Bahn und damit ungefaehr die Planungszeit. "
            "Alle zu verschmelzenden Bahnen muessen dieselbe Abtastzeit "
            "haben; der Wert gilt deshalb fuer PTP, LIN und CIRC gemeinsam.")

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

    def _build_param_display(self, page):
        """Drehkonvention waehlen. Wirkt sofort und ueberall.

        Kein "Uebernehmen": der Wert geht nicht an move_group, es wird nichts
        neu gestartet, und es gibt nichts, was zwischen Auswahl und Wirkung
        liegen muesste. Wer umschaltet, will sehen, was aus denselben Zahlen
        wird -- und genau das ist der Zweck dieser Seite.
        """
        page.columnconfigure(1, weight=1)
        ttk.Label(page, text="DREHKONVENTION   (wirkt sofort)",
                  style="Section.TLabel").grid(row=0, column=0, columnspan=3,
                                               sticky="w", pady=(0, 10))

        ttk.Label(page, text="Konvention", style="Card.TLabel").grid(
            row=1, column=0, sticky="w", pady=3)
        combo = ttk.Combobox(page, textvariable=self.convention_var,
                             values=list(konvention.KONVENTIONEN),
                             state="readonly", width=10)
        combo.grid(row=1, column=1, sticky="e", padx=(10, 8), pady=3)
        combo.bind("<<ComboboxSelected>>", lambda _e: self._convention_apply())
        self.param_widgets.append(combo)

        self.convention_note_var.set(konvention.BESCHREIBUNG[ANZEIGE.name])
        ttk.Label(page, textvariable=self.convention_note_var,
                  style="Mono.TLabel", wraplength=520, justify="left").grid(
            row=2, column=0, columnspan=3, sticky="w", pady=(8, 0))

        ttk.Label(page, text="ACHSEN DER DREI DREHUNGEN",
                  style="Section.TLabel").grid(row=3, column=0, columnspan=3,
                                               sticky="w", pady=(16, 6))
        table = ttk.Frame(page, style="Card.TFrame")
        table.grid(row=4, column=0, columnspan=3, sticky="w")
        for spalte, kopf in enumerate(("", "A", "B", "C", "entartet bei B =")):
            ttk.Label(table, text=kopf, style="Muted.TLabel").grid(
                row=0, column=spalte, sticky="w", padx=(0, 18), pady=(0, 4))
        for zeile, name in enumerate(konvention.KONVENTIONEN, start=1):
            stil = "Tcp.TLabel" if name == ANZEIGE.name else "Mono.TLabel"
            marke = ttk.Label(table, text=name, style=stil)
            marke.grid(row=zeile, column=0, sticky="w", padx=(0, 18))
            self.convention_marks[name] = marke
            for spalte, achse in enumerate(konvention.ACHSEN[name], start=1):
                ttk.Label(table, text=achse, style="Mono.TLabel").grid(
                    row=zeile, column=spalte, sticky="w", padx=(0, 18))
            grenzen = ", ".join(f"{v:g}" for v in konvention.B_SINGULAER_DEG[name])
            ttk.Label(table, text=f"{grenzen} deg", style="Mono.TLabel").grid(
                row=zeile, column=4, sticky="w")

        self._param_note(
            page, 5,
            "A ist immer die erste Drehung, C die letzte. Die Beschriftung "
            "aendert sich beim Umschalten deshalb nicht -- nur die Zahlen und "
            "die Achsen, um die gedreht wird. Bei ZYZ und ZXZ liegt die "
            "Entartung bei B = 0, also in der Grundstellung: dort drehen A und "
            "C um dieselbe Achse, die Anzeige springt und zwei Jogtasten "
            "bewegen dasselbe. Der Roboter steht dabei unauffaellig. Das ist "
            "eine Eigenschaft der Zahlen und nicht der Maschine -- die "
            "kinematische Singularitaet meldet das Fahrfenster getrennt davon "
            "als Verstaerkung in rad/m.")

        self._param_note(
            page, 6,
            "Die URDF fuehrt rpy und kennt nur ZYX. Was in die Datei "
            "geschrieben wird, folgt dieser Einstellung nicht: die "
            "Winkelfelder im Bearbeiten-Fenster, im Anhaengen-Fenster und "
            "unter Frames und Werkzeuge bleiben bei ZYX und sind dort "
            "entsprechend beschriftet.", TINT_OK, OK_GREEN)

    def _convention_apply(self):
        """Auswahl uebernehmen und alles Sichtbare neu zeichnen."""
        gewaehlt = self.convention_var.get()
        geaendert, grund = ANZEIGE.setzen(gewaehlt)
        if not geaendert:
            if grund != "unveraendert":
                self._log(f"Drehkonvention nicht umgeschaltet: {grund}", "warn")
            self.convention_var.set(ANZEIGE.name)      # Auswahl zuruecksetzen
            return
        self._log(f"Drehkonvention {ANZEIGE.name} -- {ANZEIGE.beschreibung}",
                  "ok")

    def _convention_changed(self, name):
        """Rueckruf der Anzeigekonvention. Alles neu zeichnen, was Winkel zeigt.

        Ein Rueckruf und kein direkter Aufruf: der Umschalter sitzt im
        Parameterfenster, die betroffenen Zahlen stehen in vier anderen.
        Wuerde die Seite sie einzeln anfassen, muesste sie wissen, welche
        gerade offen sind.
        """
        self.convention_note_var.set(konvention.BESCHREIBUNG[name])
        for schluessel, marke in self.convention_marks.items():
            if marke.winfo_exists():
                marke.configure(style="Tcp.TLabel" if schluessel == name
                                else "Mono.TLabel")

        # Die gefuehrte Sollpose des Joggens ist in der alten Konvention
        # angeschrieben. Weiterschieben hiesse, Zahlen einer Vorschrift mit
        # den Schritten einer anderen zu bewegen.
        self._jog_stop("Drehkonvention geaendert")

        self._update_kinematics()
        if self.cart_entries:
            self._cart_from_tcp()
        if self.drive_cart_entries:
            self._drive_cart_from_tcp()

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
        for key, _label, _unit, _default in PLANNER_FIELDS:
            put(key, config.planner[key], digits=6)
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

        for key, label, _unit, _default in PLANNER_FIELDS:
            good, value = self._read_param(key, label, sign="positive")
            ok = ok and good
            if not good:
                continue
            # Bewusst kein Begrenzen wie bei den Override-Prozenten: eine still
            # zurechtgebogene Abtastzeit wuerde eine ganze Messreihe entwerten,
            # ohne dass es auffaellt. Der Wert wird abgelehnt, der Bediener
            # entscheidet.
            if key == "sampling_time" and not (
                    SAMPLING_TIME_MIN <= value <= SAMPLING_TIME_MAX):
                entry = self.param_entries.get(key)
                if entry is not None:
                    entry.configure(background=BG_INVALID)
                self._log(f"Abtastzeit ausserhalb {SAMPLING_TIME_MIN:g} bis "
                          f"{SAMPLING_TIME_MAX:g} s", "error")
                ok = False
                continue
            draft.planner[key] = value

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

        planner = self.planner_var.get()
        if planner not in PLANNER_IDS:
            # Ein Auswahlfeld sollte nichts anderes liefern koennen. Die
            # Pruefung steht hier trotzdem: sie kostet nichts, und die
            # Alternative waere eine Konfiguration, die nur Pilz beanstandet.
            self._log(f"Fahrart '{planner}' ist unbekannt", "error")
            ok = False
        else:
            draft.planner_id = planner
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
        # Was pro Auftrag wirkt, gehoert zum gehaltenen Plan. Bis 2.21 fiel
        # dieser Fall hier durch: das Fahrfenster verwarf den Plan bei jedem
        # geaenderten Sollwert, das Parameterfenster aber nicht bei einer
        # geaenderten Fahrart. Die gehaltene Bahn war dann mit PTP gerechnet
        # und hiess LIN. Beide Wege verwerfen jetzt.
        job_before = (self.moveit_config.planner_id,
                      tuple(self.moveit_config.plan[key]
                            for key, _l, _u, _d in PLAN_FIELDS))
        self.moveit_config = draft
        self._sync_motion_from_config()
        job_after = (draft.planner_id,
                     tuple(draft.plan[key] for key, _l, _u, _d in PLAN_FIELDS))
        if job_before != job_after:
            self._drop_plan("Bewegungsparameter geaendert")
            self._seq_drop("Bewegungsparameter geaendert")
        # Derselbe Wert steht seit 2.22 auch im Fahrfenster. Er wird von hier
        # aus nachgezogen, damit nicht zwei Fenster zwei Zahlen fuer dieselbe
        # Sache zeigen. Eine dort offene Eingabe faellt dabei weg -- die
        # Richtung ist symmetrisch zu der aus _drive_params_effect.
        offen = self._drive_param_dirty()
        self._drive_params_fill()
        if offen:
            self._log("Fahrfenster nachgezogen; die dort offene Eingabe der "
                      "Fahrparameter ist verworfen", "warn")
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
        # Andere Achsen, andere Frames: die alten Punkte liessen sich nicht
        # mehr spaltenrichtig schreiben. Beim ersten Laden ist der Puffer
        # leer, dann bleibt es still.
        self._drop_record("anderes Modell geladen")
        self.history.clear()
        self._update_undo()
        self.saved_signature = serialize(root)
        self._set_dirty(False)

        # Zweiter Weg neben _rebuild_from_root, deshalb muss die Anpassung
        # der Achsgrenzen hier ebenfalls stehen. Beim ersten Laden gibt es
        # das Parameterfenster noch nicht, spaeter wird es neu aufgebaut.
        self.moveit_config.sync_joints(joints)
        self._sync_tip(links)
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
        # Und dasselbe fuer die Folge. Getrennt und nicht ueber _drop_plan:
        # der Einzelplan faellt aus mehr Gruenden als sie, und ein
        # gemeinsamer Aufruf haette die Folge auch dann verworfen, wenn
        # bloss der Zielraum im Fahrfenster gewechselt wurde. Waehrend die
        # Folge selbst gefahren wird, ist seq_hold leer -- _seq_drive hat
        # sie uebernommen --, deshalb trifft die Zeile den eigenen Takt
        # nicht.
        if self.seq_hold is not None:
            self._seq_drop("eine Achse wurde gestellt")
        self.ros.set_positions(self.positions)
        # Hier und nicht in den einzelnen Aufrufern: _push_positions ist der
        # eine Weg, ueber den ein geaenderter Achsstand hinausgeht -- Schieber,
        # Sollwertfeld, Jog, Fahrtakt. Wer die Zeile in die Aufrufer streute,
        # haette so viele Stellen zu pflegen, wie es Wege gibt, und beim
        # naechsten neuen Weg eine Anzeige, die manchmal stehenbleibt. Nach
        # apply_couplings, damit gefuehrte Achsen mitgerechnet sind.
        self._update_drive_tcp()

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
        xyz, abc = pose
        for key, value in zip(("x", "y", "z"), xyz):
            self.cart_entries[key].delete(0, "end")
            self.cart_entries[key].insert(0, f"{value:.4f}")
        for key, value in zip(ANGLE_KEYS, abc):
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
                  "A {a:.2f}  B {b:.2f}  C {c:.2f} deg  {k}".format(
                      k=ANZEIGE.name, **target))
        self._log("Die Eingabe ist lesbar. Achswerte rechnet das Fahrfenster: "
                  "'Ins Fahrfenster' und dort Planen.", "info")

    def _cart_to_drive(self):
        """Die sechs Werte an das Fahrfenster geben und dort auf IK stellen.

        Der Reiter bleibt damit, was er war -- eine Anzeige und ein
        Rechenblatt --, und die Fahrt beginnt weiterhin nur an einer Stelle.
        """
        values = {}
        for key, label, _unit in CART_FIELDS:
            entry = self.cart_entries[key]
            try:
                values[key] = float(entry.get().replace(",", "."))
            except ValueError:
                entry.configure(background=BG_INVALID)
                self._log(f"{label} ist nicht lesbar", "error")
                return
            entry.configure(background=BG_OK)

        self._open_drive()
        if not self.drive_cart_entries:
            self._log("Das Fahrfenster liess sich nicht oeffnen", "error")
            return
        self.drive_space_var.set(SPACE_CART)
        self._drive_space_changed()
        self._drive_write_cart(values)
        self._log("Ziel ins Fahrfenster uebernommen. Dort Planen, dann "
                  "Fahrt starten.", "ok")

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

    def _record_tolerances(self):
        """Ruheschwelle je Achsspalte, in der Einheit dieser Spalte.

        _joint_snapshot liefert grad bei einem Drehgelenk und Meter bei einem
        Schubgelenk, weil die Werte so in die CSV gehoeren. Eine Schwelle in
        rad gegen einen Wert in grad zu halten ist um den Faktor 57 daneben --
        genau dieser Vergleich stand bis 2.14 im Rekorder.
        """
        return [RECORD_MIN_JOINT_DEG if is_rotational(j) else RECORD_MIN_DIST_M
                for j in self._movable_joints()]

    def _frame_order(self):
        """Linknamen so geordnet, dass der Vater vor dem Kind steht.

        Dieselbe Reihenfolge wie in der Baumansicht. Ein nicht erreichbarer
        Rest -- bei einem Kreis in der URDF -- wird angehaengt statt
        weggelassen: die Spalte soll da sein, auch wenn ihr Inhalt fragwuerdig
        ist. Weglassen hiesse, den Fehler in der Datei unsichtbar zu machen.
        """
        parents = self._frame_parents()
        order, pending = [], [l["name"] for l in self.links]
        while pending:
            progressed = False
            for name in list(pending):
                if parents.get(name, "") in pending:
                    continue                           # Vater kommt noch
                order.append(name)
                pending.remove(name)
                progressed = True
            if not progressed:
                order.extend(pending)                  # Kreis, Rest anhaengen
                break
        return order

    def _record_frames(self):
        """Framenamen fuer die CSV. Der TCP steht zuletzt.

        write_csv schreibt den letzten Namen als #tcp in den Kopf. Bei einer
        offenen Kette faellt er ohnehin ans Ende; bei einem verzweigten Baum
        wuerde er es nicht, deshalb wird er hier bewusst dorthin gesetzt.
        """
        order = self._frame_order()
        tcp = self._tcp_link()
        if tcp is not None and tcp in order:
            order = [n for n in order if n != tcp] + [tcp]
        return order

    def _frame_snapshot(self, poses, names):
        """Je Frame sechs Zahlen: x y z in m, A B C in rad.

        Flach, in der Reihenfolge von `names`. Der Rekorder rechnet die Winkel
        erst beim Schreiben in grad um, deshalb bleiben sie hier in rad.

        Ein Frame ohne Lage -- abgehaengter Teilbaum -- bekommt Nullen. Die
        Zeile behaelt so ihre Spaltenzahl; erkennbar bleibt der Fall an den
        Nullen, die sich nicht bewegen.
        """
        flat = []
        for name in names:
            matrix = poses.get(name)
            if matrix is None:
                flat.extend((0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
                continue
            xyz, abc = konvention.to_xyz_abc(matrix, self.recorder.konvention)
            flat.extend(xyz)
            # abc und nicht rpy: die drei Winkel gehoeren der Konvention des
            # Rekorders, nicht der festen ZYX-Folge der URDF. Bis 2.24 stand
            # hier der alte Name, den es in dieser Methode nicht gibt.
            flat.extend(abc)
        return flat

    def _record_chain(self):
        """Kette als [(kind, elter, gelenk)] fuer den Kopf der CSV."""
        return [(j["child"], j["parent"], j["name"]) for j in self.joints]

    def _record_couplings(self):
        """Kopplungen als [(gelenk, fuehrung, faktor, offset)].

        Der Offset wandert in die Einheit der Spalte, also grad bei einem
        Drehgelenk. Dann gilt die Zeile so, wie sie dasteht, unmittelbar fuer
        die Zahlen daneben: Spalte = faktor * Fuehrungsspalte + offset. In rad
        geschrieben waere sie richtig und trotzdem unbrauchbar.
        """
        rows = []
        for joint in self.joints:
            mimic = joint["mimic"]
            if mimic is None or joint["type"] == "fixed":
                continue
            offset = (math.degrees(mimic["offset"]) if is_rotational(joint)
                      else mimic["offset"])
            rows.append((joint["name"], mimic["joint"],
                         mimic["multiplier"], offset))
        return rows

    def _toggle_record(self):
        if self.recorder.active:
            self.recorder.stop()
            ANZEIGE.freigeben()
            self._log(f"Aufzeichnung angehalten, {self.recorder.count} Punkte "
                      f"in {self.recorder.duration:.1f} s", "ok")
        else:
            frames = self._record_frames()
            vorher = self.recorder.konvention
            kept = self.recorder.start(
                self._record_columns(), frames, self._record_chain(),
                couplings=self._record_couplings(),
                robot_name=self.robot_name,
                joint_tolerances=self._record_tolerances(),
                drehkonvention=ANZEIGE.name)
            # Solange geschrieben wird, steht die Konvention fest. Der
            # Umschalter im Parameterfenster meldet den Grund, statt die
            # Punkte stillschweigend zu verwerfen.
            ANZEIGE.sperren("eine Aufzeichnung laeuft")
            if vorher != self.recorder.konvention and not kept:
                self._log(f"Drehkonvention der Aufzeichnung: "
                          f"{self.recorder.konvention}", "info")
            self.record_warned = False
            self._log(f"Aufzeichnung laeuft, {kept} Punkte im Puffer" if kept
                      else "Aufzeichnung laeuft", "ok")
            self._log(f"Aufgezeichnet werden TCP, "
                      f"{len(self.recorder.joint_names)} Achsen und "
                      f"{len(frames)} Frames mit {RECORD_RATE_HZ:.0f} Hz",
                      "info")
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
        """Nach einem Modellwechsel passen die alten Punkte nicht mehr.

        reset statt clear: bei einem anderen Modell passt nicht nur der Inhalt
        nicht mehr, sondern auch die Spaltenaufteilung. clear haelt sie fest,
        damit sich eine angehaltene Aufzeichnung fortsetzen laesst -- genau das
        ist hier nicht gemeint.
        """
        count, running = self.recorder.count, self.recorder.active
        self.recorder.stop()
        self.recorder.reset()
        self._update_record_button()
        self._update_record_status()
        if count or running:
            self._log(f"{count} Bahnpunkte verworfen: {reason}", "warn")

    def _record_tick(self):
        """Eigener Takt. Er laeuft weiter, was auch immer die Abtastung tut.

        Die Neubestellung steht in einem finally und nicht am Ende des
        Rumpfes. Der Unterschied ist der zwischen einem verlorenen Punkt und
        einem verlorenen Takt: faellt die Zeile aus, kommt kein after mehr,
        und die Aufzeichnung ist fuer den Rest der Sitzung tot -- auch nach
        Anhalten und erneutem Aufzeichnen, denn den Takt startet niemand ein
        zweites Mal. Genau das war der Fehler bis 2.24.

        Dieselbe Regel gilt hier wie bei den ROS-Rueckrufen: ein Takt darf
        an dem sterben, was er tut, aber nicht daran, dass er es tut.
        """
        try:
            self._record_sample()
        except Exception as exc:                       # noqa: BLE001 -- Absicht
            # Verworfen und nicht stillschweigend uebersprungen: was der
            # Puffer nach einem Fehler enthaelt, ist nicht mehr zu belegen.
            try:
                self._drop_record(f"Fehler im Rekordertakt: {exc}")
                self._log(traceback.format_exc(), "error")
            except Exception:                          # noqa: BLE001
                traceback.print_exc()                  # Log selbst kaputt
        finally:
            self.after(RECORD_TICK_MS, self._record_tick)

    def _record_sample(self):
        """Nimmt genau einen Punkt entgegen und filtert ihn."""
        if self.recorder.active:
            # Einmal rechnen, zweimal benutzen: der TCP ist einer der Frames.
            # Bei 25 Hz waere ein zweiter Durchlauf der Vorwaertskinematik
            # nicht teuer, aber es waeren zwei Staende, die auseinanderlaufen
            # koennen, wenn dazwischen ein Sollwert eintrifft.
            poses = forward_kinematics(self.links, self.joints, self.positions)
            tcp = self._tcp_link()
            values = self._joint_snapshot()
            if tcp is None or tcp not in poses:
                if not self.record_warned:
                    self._log("Kein eindeutiger Endeffektor, es wird nichts "
                              "aufgezeichnet", "warn")
                    self.record_warned = True
            elif len(values) != len(self.recorder.joint_names):
                # Der Achssatz hat sich geaendert, ohne dass jemand geraeumt
                # haette. Weiterschreiben hiesse, Zeilen verschiedener Breite
                # in dieselbe Datei zu legen.
                self._drop_record("Achssatz waehrend der Aufzeichnung geaendert")
            else:
                # Ueber die Konvention des Rekorders und nicht ueber ANZEIGE:
                # sie sind waehrend einer Aufzeichnung zwar gleich, aber das
                # ist eine Zusicherung der Sperre und keine der Rechnung.
                xyz, abc = konvention.to_xyz_abc(poses[tcp],
                                                 self.recorder.konvention)
                # Die Framenamen kommen aus dem Rekorder, nicht aus dem
                # Modell: nur so passt die Zeile in jedem Fall zu dem
                # Spaltenkopf, der spaeter geschrieben wird.
                frames = self._frame_snapshot(poses, self.recorder.frame_names)
                if self.recorder.sample(xyz, abc, values, frames):
                    self._update_record_status()

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
    # Punktfolge
    # ----------------------------------------------------------------------

    def _open_sequence(self):
        """Fenster fuer die Punktliste. Plant, prueft und faehrt.

        Links die Liste, rechts der Bericht. Zwei Felder und nicht eines: das
        eine wird getippt, das andere gelesen. In einem Feld waere der
        Bericht beim naechsten Tastendruck weg oder die Liste dazwischen
        verstreut.
        """
        if self._show_window(self.seq_window):
            return

        window = tk.Toplevel(self)
        self.seq_window = window
        window.title("Punktfolge")
        window.configure(background=BG)
        window.minsize(SEQ_MIN_WIDTH, SEQ_MIN_HEIGHT)
        window.geometry(f"{SEQ_WIDTH}x{SEQ_HEIGHT}")
        window.protocol("WM_DELETE_WINDOW", self._close_sequence)
        window.columnconfigure(0, weight=3, minsize=420)
        window.columnconfigure(1, weight=4, minsize=340)
        window.rowconfigure(1, weight=1)

        kopf = ttk.Frame(window, padding=(PAD, PAD, PAD, 0))
        kopf.grid(row=0, column=0, columnspan=2, sticky="ew")
        ttk.Label(kopf, textvariable=self.seq_status_var,
                  style="Title.TLabel").pack(side="left")
        ttk.Label(kopf, text="Lagen werden zuerst ueber /compute_ik "
                             "aufgeloest, J-Zeilen sind schon Achsziele; "
                             "hinaus geht die Folge nur mit Achszielen. "
                             "Gefahren wird sie mit "
                             "'Folge fahren', und nur vom Stand aus, an "
                             "dem geplant wurde.",
                  style="Muted.TLabel", wraplength=DRIVE_WRAP).pack(
                      side="left", padx=(16, 0))

        links = self._card(window, row=1, column=0, sticky="nsew",
                           padx=(PAD, PAD // 2), pady=PAD)
        links.rowconfigure(0, weight=1)
        links.columnconfigure(0, weight=1)
        rahmen = ttk.Frame(links, style="Card.TFrame", padding=(12, 10))
        rahmen.grid(row=0, column=0, sticky="nsew")
        rahmen.rowconfigure(1, weight=1)
        rahmen.columnconfigure(0, weight=1)
        ttk.Label(rahmen, text="PUNKTLISTE", style="Section.TLabel").grid(
            row=0, column=0, sticky="w", pady=(0, 6))
        self.seq_text = tk.Text(rahmen, wrap="none", undo=True,
                                font=("TkFixedFont", 10), relief="flat",
                                background=SURFACE, foreground=TEXT,
                                highlightthickness=1,
                                highlightbackground=BORDER,
                                padx=8, pady=6)
        self.seq_text.grid(row=1, column=0, sticky="nsew")
        rolle = ttk.Scrollbar(rahmen, orient="vertical",
                              command=self.seq_text.yview)
        rolle.grid(row=1, column=1, sticky="ns")
        self.seq_text.configure(yscrollcommand=rolle.set)
        self.seq_text.insert("1.0", SEQ_BEISPIEL)

        rechts = self._card(window, row=1, column=1, sticky="nsew",
                            padx=(PAD // 2, PAD), pady=PAD)
        rechts.rowconfigure(0, weight=1)
        rechts.columnconfigure(0, weight=1)
        rrahmen = ttk.Frame(rechts, style="Card.TFrame", padding=(12, 10))
        rrahmen.grid(row=0, column=0, sticky="nsew")
        rrahmen.rowconfigure(1, weight=1)
        rrahmen.columnconfigure(0, weight=1)
        ttk.Label(rrahmen, text="BERICHT", style="Section.TLabel").grid(
            row=0, column=0, sticky="w", pady=(0, 6))
        self.seq_report = tk.Text(rrahmen, wrap="none", state="disabled",
                                  font=("TkFixedFont", 10), relief="flat",
                                  background=LOG_BG, foreground=TEXT,
                                  highlightthickness=1,
                                  highlightbackground=BORDER,
                                  padx=8, pady=6)
        self.seq_report.grid(row=1, column=0, sticky="nsew")
        rrolle = ttk.Scrollbar(rrahmen, orient="vertical",
                               command=self.seq_report.yview)
        rrolle.grid(row=1, column=1, sticky="ns")
        self.seq_report.configure(yscrollcommand=rrolle.set)
        for tag, colour in (("info", TEXT), ("ok", OK_GREEN),
                            ("warn", WARN), ("error", COLOR_STOP)):
            self.seq_report.tag_configure(tag, foreground=colour)

        fuss = ttk.Frame(window, padding=(PAD, 0, PAD, PAD))
        fuss.grid(row=2, column=0, columnspan=2, sticky="ew")
        ttk.Button(fuss, text="Pruefen", command=self._seq_check,
                   width=16).pack(side="left")
        ttk.Button(fuss, text="Lage anfuegen", command=self._seq_append_current,
                   width=16).pack(side="left", padx=(8, 0))
        ttk.Button(fuss, text="Folge planen", command=self._seq_plan,
                   width=16).pack(side="left", padx=(8, 0))
        # Die Taste steht neben 'Folge planen' und nicht an ihrer Stelle:
        # zwischen Rechnen und Bewegen liegt eine Handlung, und sie soll
        # sichtbar zwei Handlungen bleiben. Wer dieselbe Folge zweimal
        # fahren will, plant sie zweimal -- der Achsstand ist nach der
        # ersten Fahrt ein anderer, und damit gilt die Bahn nicht mehr.
        self.seq_drive_button = ttk.Button(fuss, text="Folge fahren",
                                           command=self._seq_drive, width=16)
        self.seq_drive_button.pack(side="left", padx=(8, 0))
        ttk.Button(fuss, text="Bahn als CSV...", command=self._seq_export,
                   width=16).pack(side="left", padx=(8, 0))
        ttk.Button(fuss, text="Schliessen", command=self._close_sequence,
                   width=16).pack(side="right")

        self._seq_write("Punktliste eingeben und 'Pruefen' druecken.\n"
                        "Geprueft wird ohne move_group: das Format, die "
                        "Grenzen und die Ueberschleifradien.\n", "info")

    def _close_sequence(self):
        window = self.seq_window
        self.seq_window = None
        self.seq_text = None
        self.seq_report = None
        # Die Taste geht mit dem Fenster. Die gehaltene Bahn nicht: sie
        # gehoert zum Achsstand und nicht zum Fenster, und ein geschlossenes
        # Fenster ist kein Grund, eine gerechnete Bahn wegzuwerfen. Beim
        # naechsten Oeffnen ist sie wieder fahrbar.
        self.seq_drive_button = None
        if window is not None and window.winfo_exists():
            window.destroy()

    # -- Bericht -----------------------------------------------------------

    def _seq_write(self, text, level="info", clear=False):
        """In das Berichtsfeld schreiben. Ersetzt nichts im Hauptprotokoll.

        Zwei Ausgaben nebeneinander, und das mit Absicht: das Hauptprotokoll
        haelt den Ablauf der Anlage fest -- Zustaende, Auftraege, Fehler --,
        das Berichtsfeld haelt eine Tabelle, die man liest und vergleicht.
        Eine Tabelle in einem Protokoll mit Zeitstempeln davor ist keine
        Tabelle mehr.
        """
        widget = self.seq_report
        if widget is None or not widget.winfo_exists():
            return
        widget.configure(state="normal")
        if clear:
            widget.delete("1.0", "end")
        widget.insert("end", text if text.endswith("\n") else text + "\n",
                      level)
        widget.see("end")
        widget.configure(state="disabled")

    def _seq_status(self):
        if self.seq_started is None:
            return
        gelaufen = time.monotonic() - self.seq_started
        if self.state == STATE_SEQ_IK:
            self.seq_status_var.set(
                f"IK Punkt {self.seq_ik_index + 1} von "
                f"{len(self.seq_schritte)}  {gelaufen:4.1f} s")
        elif self.state == STATE_SEQUENCE:
            self.seq_status_var.set(
                f"Folge wird geplant  {gelaufen:4.1f} s")

    # -- Lesen und pruefen -------------------------------------------------

    def _seq_modal(self):
        """Anfangszustand der Modalfelder aus den Planparametern.

        Nicht aus MODAL_START: was im Parameterfenster steht, gilt auch fuer
        eine Folge. Sonst faehrt dieselbe Punktliste je nach Weg mit einem
        anderen Override, und der Unterschied stuende nirgends.
        """
        plan = self.moveit_config.plan
        return {
            "fahrart": self.moveit_config.planner_id,
            "tempo": float(plan["velocity_scaling"]),
            "beschleunigung": float(plan["acceleration_scaling"]),
            "radius": float(plan["blend_radius"]),
        }

    def _seq_read(self):
        """Punktliste lesen und pruefen. None heisst: es geht nicht weiter.

        Der Bericht wird in jedem Fall geschrieben, auch bei Fehlern. Wer
        eine Liste eintippt, will alle Beanstandungen auf einmal sehen und
        nicht eine nach der anderen.
        """
        if self.seq_text is None or not self.seq_text.winfo_exists():
            self._log("Das Folgenfenster ist nicht offen", "error")
            return None

        text = self.seq_text.get("1.0", "end")
        punkte, lesefehler = sequenz.lies_liste(text)
        self._seq_write("", clear=True)
        for befund in lesefehler:
            self._seq_write(befund.text, "error")
        if lesefehler:
            self._seq_write(f"{len(lesefehler)} Zeile(n) uebersprungen.",
                            "warn")
        if not punkte:
            self._seq_write("Keine lesbaren Punkte.", "error")
            return None

        # Seit 3.5. Vor dem Aufloesen, weil die Pruefung der Radien die Lage
        # der Achsziele braucht.
        achsfehler = self._seq_achsziele(punkte)

        folge = sequenz.Folge(punkte, modal=self._seq_modal())
        schritte = folge.aufloesen()
        befunde = folge.pruefe(schritte)

        modal = self._seq_modal()
        self._seq_write(
            f"Modalzustand am Anfang: {modal['fahrart']}, "
            f"{modal['tempo']:g} %, {modal['beschleunigung']:g} %, "
            f"Radius {modal['radius']:g} m  (aus dem Parameterfenster)",
            "info")
        self._seq_write("")
        self._seq_write("  Nr  Name          X        Y        Z    "
                        "  Fahrart     Tempo      Beschl     Radius", "info")
        for zeile in folge.bericht(schritte).splitlines():
            self._seq_write(zeile, "info")
        self._seq_write("")

        for zeile in folge.spielraum(schritte):
            eng = zeile["frei"] <= 0.0
            self._seq_write(
                f"{zeile['namen'][0]:<12} -> {zeile['namen'][1]:<12} "
                f"Abstand {zeile['abstand']:8.4f} m, "
                f"belegt {zeile['belegt']:8.4f}, "
                f"frei {zeile['frei']:8.4f}",
                "error" if eng else "info")

        if befunde:
            self._seq_write("")
            for befund in befunde:
                stufe = {sequenz.FEHLER: "error",
                         sequenz.WARNUNG: "warn"}.get(befund.stufe, "info")
                self._seq_write(befund.text, stufe)

        harte = [b for b in befunde if b.stufe == sequenz.FEHLER]
        # Ein gescheitertes Achsziel steht in harte ohnehin als "nicht
        # aufgeloest" und wird dort gezaehlt. achsfehler steht in der
        # Bedingung trotzdem: die Folge haelt nicht erst an, weil pruefe()
        # es auch bemerkt.
        if harte or lesefehler or achsfehler:
            self._seq_write("")
            self._seq_write(f"{len(harte) + len(lesefehler)} Fehler. "
                            f"So geht die Folge nicht hinaus.", "error")
            self.seq_status_var.set(
                f"{len(harte) + len(lesefehler)} Fehler in der Punktliste")
            return None

        self._seq_write("")
        self._seq_write(f"{len(schritte)} Punkte, keine Beanstandung.", "ok")
        self.seq_status_var.set(f"{len(schritte)} Punkte, geprueft")
        return folge

    # -- Achsziele (seit 3.5) ----------------------------------------------

    def _seq_gelenke(self):
        """Die beweglichen Gelenke fuer sequenz.loese_achsziel.

        Dieselbe Auswahl und Reihenfolge wie die Zielfelder des
        Fahrfensters: ohne feste und ohne gekoppelte Gelenke.
        """
        return tuple(
            {"name": j["name"],
             "drehend": is_rotational(j),
             "unten": j["lower"] if j["has_limit"] else None,
             "oben": j["upper"] if j["has_limit"] else None}
            for j in self.joints
            if j["type"] != "fixed" and j["mimic"] is None)

    def _seq_achsziele(self, punkte):
        """Achsziele zuordnen und ihre Lage nachtragen. Gibt die Fehler zurueck.

        Die Lage kommt aus _tcp_pose mit dem Achsstand des Ziels, also aus
        derselben Vorwaertskinematik und Anzeigekonvention wie 'Lage
        anfuegen'. Kopplungen werden vorher ausgewertet, wie bei jedem
        anderen Stand.
        """
        gelenke = self._seq_gelenke()
        fehler = []
        gesehen = False
        for index, punkt in enumerate(punkte):
            if punkt.achsziel is None:
                continue
            if not gesehen:
                gesehen = True
                self._seq_write(
                    "Achsziele (J): Werte in der Reihenfolge "
                    + (" ".join(g["name"] for g in gelenke) or "-")
                    + ", Grad bzw. Meter bei Schubachsen.", "info")
            achsen, befunde = sequenz.loese_achsziel(punkt, gelenke, index)
            fehler.extend(befunde)
            if achsen is None:
                continue
            punkt.achsen = achsen

            stand = dict(self.positions)
            stand.update(achsen)
            apply_couplings(self.joints, stand)
            lage = self._tcp_pose(stand)
            if lage is None:
                fehler.append(sequenz.Befund(
                    sequenz.FEHLER,
                    f"{punkt.name or f'P{index + 1}'}: kein eindeutiger "
                    f"Endeffektor, das Achsziel hat keine Lage. Ohne sie ist "
                    f"der Ueberschleifradius des Vorgaengers nicht zu "
                    f"pruefen. Im Parameterfenster unter Kinematik den "
                    f"Tip-Link eintragen.", stelle=index))
                continue
            xyz, abc = lage
            punkt.ergaenze_lage(xyz, tuple(math.degrees(w) for w in abc))

        for befund in fehler:
            self._seq_write(befund.text, "error")
        if gesehen:
            self._seq_write("")
        return fehler

    def _seq_seed_setzen(self, achsen):
        """Startwert fuer den naechsten Punkt der Kette aus einem Achsstand.

        Ueber eine Kopie des Achsstands, damit gekoppelte Gelenke
        mitgerechnet werden. Ein Weg fuer IK-Loesungen und Achsziele.
        """
        stand = dict(self.positions)
        stand.update(achsen)
        apply_couplings(self.joints, stand)
        self.seq_ik_seed = {j["name"]: float(stand.get(j["name"], 0.0))
                            for j in self.joints if j["type"] != "fixed"}

    def _seq_achsziel_uebernehmen(self, schritt):
        """Ein Achsziel in die Kette nehmen, ohne /compute_ik. False: Abbruch.

        Die Achswerte stehen seit _seq_read am Punkt. Hier werden sie
        berichtet und als Startwert eingesetzt. Ein grosser Schritt ist bei
        einem Achsziel keine Warnung: er ist eingetragen und nicht von einem
        Loeser gewaehlt.
        """
        achsen = schritt.punkt.achsen or {}
        wanted = [j["name"] for j in self.joints
                  if j["type"] != "fixed" and j["mimic"] is None]
        missing = [name for name in wanted if name not in achsen]
        if not achsen or missing:
            text = (f"{schritt.name}: Achsziel ohne Achswerte fuer "
                    f"{', '.join(missing) or 'alle Achsen'}. Das ist ein "
                    f"Fehler in der Aufloesung, nicht in der Liste.")
            self._set_state(STATE_ERROR, "Folge-IK")
            self._seq_write("")
            self._seq_write(text, "error")
            self._log(text, "error")
            self.seq_status_var.set(f"Achsziel {schritt.name} ohne Achswerte")
            self.seq_started = None
            self._seq_ik_reset()
            return False

        weit, weitester = 0.0, ""
        gezeigt = []
        for name in wanted:
            joint = self._joint_by_name(name)
            wert = float(achsen[name])
            if joint is not None and is_rotational(joint):
                gezeigt.append(f"{math.degrees(wert):8.2f}")
            else:
                gezeigt.append(f"{wert:8.4f}")
            weg = abs(wert - float(self.seq_ik_seed.get(name, 0.0)))
            if weg > weit:
                weit, weitester = weg, name
        joint = self._joint_by_name(weitester)
        drehend = joint is not None and is_rotational(joint)
        sprung = math.degrees(weit) if drehend else weit
        self._seq_write(
            f"{schritt.name:<10} " + " ".join(gezeigt)
            + f"   groesster Schritt {weitester or '-'} {sprung:.2f} "
            + ("deg" if drehend else "m") + "   Achsziel, keine IK", "info")
        self._seq_seed_setzen(achsen)
        return True

    def _seq_check(self):
        self._seq_read()

    def _seq_append_current(self):
        """Die aktuelle TCP-Lage als Zeile anfuegen.

        Der uebliche Weg, an Punkte zu kommen: den Arm hinstellen und die
        Lage uebernehmen -- dasselbe, was an einer Steuerung Teachen heisst.
        Die Zahlen kommen aus derselben Vorwaertskinematik und derselben
        Anzeigekonvention wie die TCP-Zeile; eine zweite Rechnung gibt es
        nicht.
        """
        if self.seq_text is None or not self.seq_text.winfo_exists():
            return
        pose = self._tcp_pose()
        if pose is None:
            self._log("Kein eindeutiger Endeffektor, es gibt keine Lage zum "
                      "Anfuegen", "error")
            return
        xyz, abc = pose
        zeile = ("%.4f %.4f %.4f  %.3f %.3f %.3f  radius=0\n"
                 % (xyz[0], xyz[1], xyz[2],
                    math.degrees(abc[0]), math.degrees(abc[1]),
                    math.degrees(abc[2])))
        inhalt = self.seq_text.get("1.0", "end")
        if inhalt and not inhalt.endswith("\n"):
            self.seq_text.insert("end", "\n")
        self.seq_text.insert("end", zeile)
        self.seq_text.see("end")
        self._log(f"Lage angefuegt ({ANZEIGE.name})", "ok")

    # -- Planen ------------------------------------------------------------

    def _seq_plan(self):
        """Die Folge an /plan_sequence_path geben. Es faehrt nichts.

        Dieselben Vorpruefungen wie beim einzelnen Auftrag, und eine mehr:
        die Capability. Sie fehlt genau dann, wenn move_group ohne unsere
        Parameter laeuft -- und dann steht alles andere, was hier geprueft
        wird, auf bereit.
        """
        if self.state in STATES_BUSY:
            self._log(f"Es laeuft bereits: {self.state}", "error")
            return
        if self.stopped:
            self._log("STOP steht an, es wird nicht geplant", "error")
            return
        if not self.ros.moveit_ready:
            self._log("move_group antwortet nicht. Im Parameterfenster "
                      "starten oder extern hochfahren.", "error")
            return
        if self.moveit_stale:
            self._log("move_group liest ein anderes Modell als dieses. "
                      "Erst neu starten.", "error")
            return
        if not self.ros.sequence_ready:
            self._log("/plan_sequence_path ist nicht da. move_group muss die "
                      "Sequence-Capability tragen -- sie steht in "
                      "params_yaml und wird erst beim Start gelesen.",
                      "error")
            return

        folge = self._seq_read()
        if folge is None:
            return

        base, tip = self._drive_ik_frames()
        if not tip or not base:
            self._log("Ohne Bezug und Tip-Link hat eine Lage keinen Sinn. "
                      "Im Parameterfenster unter Kinematik eintragen.",
                      "error")
            return

        # Startzustand. Gekoppelte Gelenke gehen mit heraus -- MoveIt erwartet
        # sie im Zustand, auch wenn sie keinen Freiheitsgrad haben. Dieselbe
        # Regel wie bei _ik_payload.
        apply_couplings(self.joints, self.positions)
        start = {j["name"]: float(self.positions.get(j["name"], 0.0))
                 for j in self.joints if j["type"] != "fixed"}

        schritte = folge.aufloesen()
        if self.moveit_config.use_ruckig and any(s.radius > 0.0
                                                 for s in schritte):
            # Pilz erzeugt seine Teilbahnen auf einem festen Zeitraster, und
            # der Blender verlangt, dass beide Teilbahnen dasselbe tragen.
            # Ruckig rechnet die Zeitstempel danach neu -- zeitoptimal und
            # ruckbegrenzt, aber eben nicht mehr gleichmaessig. Das Ergebnis
            # ist "Second trajectory violates sampling time" und ein
            # Fehlercode 99999, der darueber nichts sagt.
            self._seq_write("")
            self._seq_write(
                "Ruckig-Glaettung ist eingeschaltet und es stehen Radien "
                "ueber null in der Liste. Beides zusammen scheitert in der "
                "Regel: Ruckig rechnet die Zeitstempel neu, und das "
                "Ueberschleifen verlangt ein gleichmaessiges Raster. Im "
                "Parameterfenster unter Pipeline abschalten und move_group "
                "neu starten.", "warn")

        self.seq_points = []
        self.seq_teilbahnen = []
        self.seq_dropped = ()
        self.seq_folge = folge
        self.seq_schritte = schritte
        self.seq_ik_index = 0
        self.seq_ik_seed = dict(start)
        self.seq_ik_start = dict(start)
        self.seq_ik_frames = (base, tip)
        self.seq_started = time.monotonic()
        self._set_state(STATE_SEQ_IK, f"1 von {len(schritte)}")
        self._seq_write("")
        achsziele = sum(1 for s in schritte if s.punkt.achsziel is not None)
        self._seq_write(
            f"{len(schritte) - achsziele} Punkt(e) werden einzeln ueber "
            f"/compute_ik aufgeloest, jeder mit der Loesung des Vorgaengers "
            f"als Startwert."
            + (f" {achsziele} Achsziel(e) gehen ohne IK hinaus und geben "
               f"dem folgenden Punkt den Startwert." if achsziele else ""),
            "info")
        self._seq_ik_next()

    # -- Die Kette der Rueckwaertskinematik ---------------------------------

    def _seq_ik_reset(self):
        """Die Kette vergessen. Eine Stelle, damit nichts halb stehenbleibt."""
        self.seq_schritte = ()
        self.seq_ik_index = 0
        self.seq_ik_seed = None
        self.seq_ik_start = None
        self.seq_ik_frames = None

    def _seq_ik_next(self):
        """Den naechsten Punkt aufloesen lassen, oder die Folge absenden.

        Die Schleife haengt an der Antwort und nicht an einer Uhr -- dieselbe
        Bauart wie beim Joggen seit 2.13. Ein Takt koennte Anfragen stapeln,
        sobald der Loeser laenger braucht als der Takt, und laenger braucht
        er genau dort, wo es eng wird.
        """
        schritte = self.seq_schritte
        # Seit 3.5: Achsziele brauchen keine Anfrage. Sie werden in einer
        # Schleife uebernommen -- es gibt nichts, worauf zu warten waere, und
        # eine Rekursion je Achsziel waere bei langen Listen eine Tiefe, die
        # niemand braucht.
        while (self.seq_ik_index < len(schritte)
               and schritte[self.seq_ik_index].punkt.achsziel is not None):
            if not self._seq_achsziel_uebernehmen(
                    schritte[self.seq_ik_index]):
                return
            self.seq_ik_index += 1
        index = self.seq_ik_index
        if index >= len(schritte):
            self._seq_send()
            return

        schritt = schritte[index]
        xyz, abc = schritt.punkt.xyz, schritt.punkt.abc
        # Die Punktliste traegt Grad, /compute_ik will Bogenmass. Umgerechnet
        # wird hier und nicht in sequenz.py: dort steht die Zahl so, wie sie
        # in der Zeile stand, und was sie bedeutet, entscheidet erst die
        # Anzeigekonvention.
        pose = {"x": xyz[0], "y": xyz[1], "z": xyz[2],
                "a": math.radians(abc[0]), "b": math.radians(abc[1]),
                "c": math.radians(abc[2])}

        built = self._ik_payload(pose, seed=self.seq_ik_seed)
        if built is None:
            self._set_state(STATE_ERROR, "Folge-IK")
            self._seq_ik_reset()
            return
        base, tip, payload = built
        payload["tag"] = self._job_open()

        self.ik_asked = dict(pose, konvention=ANZEIGE.name)
        self.ik_frames = (base, tip)
        self.ik_started = time.monotonic()
        self.ros.request_ik(payload)

    def _on_seq_ik_result(self, result):
        """Eine Loesung der Kette. Uebernehmen und den naechsten stellen.

        Anders als _on_ik_result schreibt hier nichts in die Zielfelder des
        Fahrfensters und es wird nichts geplant. Die Loesung gehoert zu einem
        Punkt der Liste und nicht zu dem Ziel, das dort steht.
        """
        if not self._job_mine(result):
            self._log("Verspaetete IK-Antwort der Folge verworfen: sie "
                      "gehoert zu einem abgebrochenen Auftrag", "warn")
            return
        self._job_drop()

        index = self.seq_ik_index
        schritt = self.seq_schritte[index]
        took = time.monotonic() - (self.ik_started or time.monotonic())
        self.ik_started = None

        fehler = result.get("error")
        code = int(result.get("code", 0))
        if fehler or code != 1:
            text = fehler or moveit_error_text(code)
            self._seq_ik_fail(schritt, text)
            return

        values = result.get("values") or {}
        wanted = [j["name"] for j in self.joints
                  if j["type"] != "fixed" and j["mimic"] is None]
        missing = [name for name in wanted if name not in values]
        if missing:
            self._seq_ik_fail(
                schritt,
                f"Die Antwort fuehrt {', '.join(missing)} nicht. Gehoeren "
                f"diese Achsen zur Gruppe {self.moveit_config.group}?")
            return

        # Der Punkt traegt die Loesung ab jetzt selbst. nutzlast baut daraus
        # eine JointConstraint, und Pilz rechnet keine IK mehr.
        schritt.punkt.achsen = {name: float(values[name]) for name in wanted}

        weit, weitester = 0.0, ""
        for name in wanted:
            weg = abs(values[name] - float(self.seq_ik_seed.get(name, 0.0)))
            if weg > weit:
                weit, weitester = weg, name

        gezeigt = []
        for name in wanted:
            joint = self._joint_by_name(name)
            if joint is not None and is_rotational(joint):
                gezeigt.append(f"{math.degrees(values[name]):8.2f}")
            else:
                gezeigt.append(f"{values[name]:8.4f}")
        # Der groesste Einzelweg und nicht die Summe: ein Sprung ist immer
        # der einer Achse, und eine Summe verteilt ihn auf sieben.
        joint = self._joint_by_name(weitester)
        sprung = (math.degrees(weit) if joint is not None
                  and is_rotational(joint) else weit)
        einheit = ("deg" if joint is not None and is_rotational(joint)
                   else "m")
        auffaellig = joint is not None and is_rotational(joint) \
            and math.degrees(weit) > SEQ_JUMP_DEG
        self._seq_write(
            f"{schritt.name:<10} " + " ".join(gezeigt)
            + f"   groesster Schritt {weitester} {sprung:.2f} {einheit}"
            + f"   {took * 1000:.0f} ms",
            "warn" if auffaellig else "info")
        if auffaellig:
            self._seq_write(
                f"   {weitester} springt um mehr als {SEQ_JUMP_DEG:g} deg. "
                f"Die Loesung liegt in einem anderen Konfigurationszweig als "
                f"die des Vorgaengers -- die Bahn dorthin wird gross, auch "
                f"wenn die beiden Punkte dicht beieinanderliegen.", "warn")

        # Startwert des naechsten Punktes. Ueber eine Kopie des Achsstands,
        # damit gekoppelte Gelenke mitgerechnet werden -- MoveIt erwartet sie
        # im Zustand, auch wenn sie keinen Freiheitsgrad haben.
        self._seq_seed_setzen(schritt.punkt.achsen)

        self.seq_ik_index += 1
        self._seq_ik_next()

    def _seq_ik_fail(self, schritt, text):
        """Die Kette bricht an einem bestimmten Punkt ab, und der wird genannt.

        Der ganze Grund fuer diese Fassung steht in dieser Methode. Pilz
        meldet bei derselben Lage nur den Linknamen; welcher der acht Punkte
        gemeint war, steht nirgends. Hier ist es der, an dem die Kette haelt,
        und seine sechs Zahlen stehen daneben.
        """
        xyz, abc = schritt.punkt.xyz, schritt.punkt.abc
        self._set_state(STATE_ERROR, "Folge-IK")
        self._seq_write("")
        self._seq_write(f"{schritt.name}: keine IK-Loesung. {text}", "error")
        self._seq_write(
            "   Gefragt war  X {0:.4f}  Y {1:.4f}  Z {2:.4f} m   "
            "A {3:.2f}  B {4:.2f}  C {5:.2f} deg  {6}".format(
                xyz[0], xyz[1], xyz[2], abc[0], abc[1], abc[2],
                ANZEIGE.name), "error")
        self._seq_write(
            "   Keine Loesung heisst dreierlei: die Lage liegt ausserhalb "
            "des Arbeitsraums; sie ist erreichbar, die Drehung mit dieser "
            "Kette aber nicht; oder der Loeser hatte in seiner Zeitgrenze zu "
            "wenig Versuche. Nur der dritte Fall loest sich durch Warten -- "
            "Zeitgrenze im Parameterfenster unter Kinematik.", "warn")
        self._log(f"Folgen-IK gescheitert bei {schritt.name} "
                  f"(Punkt {self.seq_ik_index + 1} von "
                  f"{len(self.seq_schritte)}): {text}", "error")
        self.seq_status_var.set(f"keine IK-Loesung fuer {schritt.name}")
        self.seq_started = None
        self._seq_ik_reset()

    def _seq_send(self):
        """Alle Punkte sind aufgeloest. Jetzt geht die Folge hinaus.

        Sie traegt ab hier ausschliesslich Achsziele. Steckt trotzdem eine
        Lage darin, ist ein Punkt ohne Loesung durchgerutscht -- das waere
        ein Fehler in der Kette und kein Grund, es Pilz herausfinden zu
        lassen.
        """
        folge = self.seq_folge
        base, tip = self.seq_ik_frames or ("", "")
        plan = self.moveit_config.plan
        try:
            payload = folge.nutzlast(
                gruppe=self.moveit_config.group,
                start=self.seq_ik_start or {},
                toleranz=math.radians(float(plan["goal_orientation_tol"])),
                planungszeit=float(plan["planning_time"]),
                versuche=max(1, int(plan["planning_attempts"])),
                tag=self._job_open(),
                tcp=tip)
        except ValueError as exc:
            self._job_drop()
            self._set_state(STATE_ERROR, "Folge")
            self._log(f"Folge nicht tragfaehig: {exc}", "error")
            self._seq_ik_reset()
            return

        ohne = [eintrag.get("name", "?") for eintrag in payload["items"]
                if "goal" not in eintrag]
        if ohne:
            self._job_drop()
            self._set_state(STATE_ERROR, "Folge")
            self._log(f"Ohne Achswerte: {', '.join(ohne)}. Die Folge geht "
                      f"nicht hinaus.", "error")
            self._seq_write(f"{len(ohne)} Punkt(e) ohne Achswerte. Das ist "
                            f"ein Fehler in der Aufloesung, nicht in der "
                            f"Liste.", "error")
            self._seq_ik_reset()
            return

        self.seq_started = time.monotonic()
        self._set_state(STATE_SEQUENCE, f"{len(payload['items'])} Punkte")
        self._seq_write("")
        self._seq_write(f"Alle Punkte aufgeloest. {len(payload['items'])} "
                        f"Auftraege mit Achszielen an /plan_sequence_path. "
                        f"Pilz rechnet keine Rueckwaertskinematik mehr.",
                        "ok")
        self.ros.request_sequence(payload)

    def _seq_diagnosis(self):
        """Nach einer gescheiterten Folge zeigen, womit gefragt wurde.

        Dieselbe Absicht wie _plan_diagnosis, und bei einer Folge noetiger:
        der Fehlercode gilt der ganzen Liste, und 99999 sagt ueber sie
        nichts. Was drinstand, weiss die Oberflaeche; warum es scheiterte,
        steht in der Ausgabe von move_group.
        """
        schritte = self.seq_schritte or ()
        if schritte:
            self._log(f"Auftrag war: Gruppe {self.moveit_config.group}, "
                      f"{len(schritte)} Punkte, Fahrarten "
                      + ", ".join(sorted({s.fahrart for s in schritte}))
                      + ", Radien "
                      + ", ".join(f"{s.radius:g}" for s in schritte), "warn")
        if self.moveit_config.use_ruckig:
            self._log("Ruckig-Glaettung ist eingeschaltet. Sie rechnet die "
                      "Zeitstempel neu und macht das Abtastraster "
                      "ungleichmaessig -- das Ueberschleifen verlangt ein "
                      "gleichmaessiges. Erster Versuch: abschalten und "
                      "move_group neu starten.", "warn")
        self._moveit_log_tail(12)

    def _on_sequence_result(self, result):
        """Ergebnis der Folgenplanung. Kommt ueber _tick aus dem Executor."""
        if self.state != STATE_SEQUENCE:
            return
        if not self._job_mine(result):
            self._log("Verspaetete Folgenantwort verworfen: sie gehoert zu "
                      "einem abgebrochenen Auftrag", "warn")
            return
        self._job_drop()

        took = (time.monotonic() - self.seq_started
                if self.seq_started else 0.0)
        self.seq_started = None

        if result.get("error"):
            self._set_state(STATE_ERROR, "Folge")
            self._log(f"Keine Folge: {result['error']}", "error")
            self._seq_write(result["error"], "error")
            self._seq_diagnosis()
            return

        code = int(result.get("code", 0))
        if code != 1:
            self._set_state(STATE_ERROR, "Folge")
            text = moveit_error_text(code)
            self._log(f"Keine Folge: {text}", "error")
            self._seq_write(text, "error")
            self._seq_write(
                "Bei einer Folge scheitert alles, sobald ein einziger "
                "Auftrag scheitert. Der Fehlercode gilt der Folge, nicht "
                "einem bestimmten Punkt.", "warn")
            self._seq_diagnosis()
            return

        roh = result.get("points") or []
        teilbahnen = list(result.get("teilbahnen") or [])
        if len(roh) < 2:
            self._set_state(STATE_READY)
            self._log("Die Folge ergibt keine Bahn: der Roboter steht "
                      "bereits an allen Punkten", "warn")
            return

        # Erst hier bereinigt, nicht in der Bruecke: die Bruecke gibt zurueck,
        # was gekommen ist, und was ausgebessert wurde, gehoert ins
        # Protokoll. Ein still geglaetteter Fehler waere kein Fehler mehr.
        punkte, verworfen = sequenz.bereinige_zeitreihe(roh)
        befunde = sequenz.pruefe_bahn(roh)

        self.seq_points = punkte
        self.seq_teilbahnen = teilbahnen
        self.seq_dropped = verworfen
        dauer = punkte[-1][0] - punkte[0][0] if len(punkte) > 1 else 0.0

        self._set_state(STATE_READY)
        self._seq_write("")
        self._seq_write(
            f"Folge geplant in {took * 1000:.0f} ms "
            f"(Planer meldet {result.get('planning_time', 0.0) * 1000:.0f} ms).",
            "ok")
        self._seq_write(
            f"{len(teilbahnen)} Teilbahn(en), {len(roh)} Stuetzpunkte, "
            f"Bahndauer {dauer:.3f} s.", "ok")
        # Die eigentliche Aussage ueber das Ueberschleifen. Eine Teilbahn
        # heisst: alle Uebergaenge sind verschmolzen. So viele Teilbahnen wie
        # Punkte heisst: keiner.
        if len(teilbahnen) == 1:
            self._seq_write(
                "Eine einzige Teilbahn -- alle Uebergaenge wurden "
                "verschmolzen, der Roboter haelt zwischendurch nicht an.",
                "ok")
        else:
            self._seq_write(
                f"{len(teilbahnen)} Teilbahnen: an "
                f"{len(teilbahnen) - 1} Uebergang/Uebergaengen wird "
                f"angehalten. Punkte je Teilbahn: "
                + ", ".join(str(n) for n in teilbahnen), "warn")

        for befund in befunde:
            stufe = {sequenz.FEHLER: "error",
                     sequenz.WARNUNG: "warn"}.get(befund.stufe, "info")
            self._seq_write(befund.text, stufe)
        if verworfen:
            self._log(f"{len(verworfen)} Stuetzpunkte der Folge ohne "
                      f"wachsende Zeit verworfen", "warn")

        # Ab hier wird die Folge fahrbereit gemacht. Scheitert das, bleiben
        # die Punkte trotzdem liegen: die CSV-Ausgabe haengt an seq_points
        # und nicht an der Fahrbereitschaft. Eine Bahn, die sich ansehen,
        # aber nicht fahren laesst, ist mehr als keine.
        start = dict(self.seq_ik_start or {})
        if not start:
            # Ohne den Stand beim Planen gibt es nichts, wogegen der Stand
            # beim Starten zu pruefen waere. Nicht ersatzweise den aktuellen
            # Stand nehmen: er ist derselbe, solange nichts dazwischenkam --
            # und wenn doch etwas dazwischenkam, verdeckte die Ersatzannahme
            # genau den Fall, den die Pruefung fangen soll.
            self._log("Der Achsstand beim Planen fehlt. Die Folge laesst "
                      "sich ansehen und ausgeben, aber nicht fahren.",
                      "error")
        else:
            try:
                player = TrajectoryPlayer(punkte)
            except ValueError as exc:
                self._log(f"Die Folge laesst sich nicht abtasten: {exc}",
                          "error")
            else:
                self.seq_hold = player
                self.seq_hold_start = start

        self.seq_status_var.set(
            f"{len(punkte)} Punkte, {dauer:.2f} s, "
            f"{len(teilbahnen)} Teilbahn(en)"
            + ("  fahrbereit" if self.seq_hold is not None else ""))
        if self.seq_hold is not None:
            self._seq_write(
                f"Fahrbereit. Bahndauer {dauer:.3f} s. Zum Fahren "
                f"'Folge fahren'. Der Achsstand muss dabei noch derselbe "
                f"sein wie eben beim Planen.", "ok")
        self._log(f"Folge geplant: {len(teilbahnen)} Teilbahn(en), "
                  f"{len(punkte)} Stuetzpunkte, {dauer:.3f} s. "
                  + ("Zum Fahren 'Folge fahren'."
                     if self.seq_hold is not None
                     else "Gefahren wird sie nicht."), "ok")
        self._update_locks()

    # -- Fahren -------------------------------------------------------------

    def _seq_drive(self):
        """Die gehaltene Folge abfahren. Derselbe Weg wie beim Einzelplan.

        Gefahren wird ueber _start_playing, also ueber denselben Takt,
        dieselbe Bremse und dieselbe Zeitrechnung wie ein einzelner Plan.
        Was hier steht, ist ausschliesslich die Pruefung davor -- und die
        ist bei einer Folge strenger als beim Einzelplan, weil eine Folge
        ueber mehrere Teilbahnen laeuft und ein falscher Startpunkt sich
        nicht nach dem ersten Punkt von selbst erledigt.
        """
        if self.state in STATES_BUSY:
            self._log(f"Es laeuft bereits: {self.state}", "error")
            return
        if self.stopped:
            self._log("STOP steht an, es wird nicht gefahren", "error")
            return
        if self.seq_hold is None:
            self._log("Keine fahrbereite Folge. Erst 'Folge planen'.",
                      "error")
            return

        # Erste Pruefung: der Stand gegen den Stand beim Planen. Dieselbe
        # Regel und dieselbe Schwelle wie in _drive_start -- der Vergleich
        # laeuft zwischen zwei Zahlen, die beide aus derselben Ablage
        # stammen, und da ist jede Abweichung ueber der Rechengenauigkeit
        # eine gestellte Achse.
        start = self.seq_hold_start or {}
        drift = max((abs(self.positions.get(name, 0.0) - wert)
                     for name, wert in start.items()), default=0.0)
        if drift > 1e-9:
            self._seq_drop("Achsstand hat sich seit dem Planen geaendert")
            return

        # Zweite Pruefung: der Stand gegen den ersten Stuetzpunkt der Bahn,
        # die zurueckkam. Die erste sagt, dass niemand etwas gestellt hat;
        # diese sagt, dass die Bahn dort anfaengt, wo der Roboter steht.
        # Beides ist nicht dasselbe: die Bahn kommt aus move_group, und was
        # von dort kommt, wird geprueft und nicht geglaubt.
        name, ab, fehlend = bahn_startabweichung(self.seq_hold.points,
                                                 self.positions)
        if fehlend:
            self._seq_drop(f"die Bahn traegt unbekannte Achsen: "
                           f"{', '.join(fehlend)}")
            return
        if ab > FOLGE_START_TOL:
            self._log(f"Der erste Stuetzpunkt liegt {ab:.6f} rad neben dem "
                      f"Achsstand, groesste Abweichung an {name}. Der erste "
                      f"Takt waere ein Sprung.", "error")
            self._seq_drop("die Bahn faengt nicht am Achsstand an")
            return

        player = self.seq_hold
        self.seq_hold = None
        self.seq_hold_start = None
        self._seq_write("")
        self._seq_write(f"Fahrt gestartet. {len(player.points)} "
                        f"Stuetzpunkte, {player.duration:.3f} s Bahnzeit.",
                        "ok")
        self._log(f"Folge wird gefahren: {len(player.points)} Stuetzpunkte, "
                  f"{player.duration:.3f} s", "ok")
        self._start_playing(player)

    def _seq_drop(self, reason=""):
        """Die gehaltene Folge verwerfen. Eine Stelle, wie bei _drop_plan.

        Die Punkte selbst bleiben liegen. Verworfen wird die
        Fahrbereitschaft und nicht das Ergebnis: seq_points traegt die
        gerechnete Bahn, und die bleibt richtig, auch wenn der Roboter
        inzwischen woanders steht. Fahren laesst sie sich dann nicht mehr,
        ansehen und ausgeben schon.
        """
        if self.seq_hold is None:
            self.seq_hold_start = None
            return
        self.seq_hold = None
        self.seq_hold_start = None
        if reason:
            self._log(f"Geplante Folge nicht mehr fahrbereit: {reason}",
                      "warn")
            self._seq_write(f"Nicht mehr fahrbereit: {reason}. Die Punkte "
                            f"bleiben stehen, gefahren wird erst nach "
                            f"erneutem Planen.", "warn")
        if self.seq_points:
            dauer = (self.seq_points[-1][0] - self.seq_points[0][0]
                     if len(self.seq_points) > 1 else 0.0)
            self.seq_status_var.set(
                f"{len(self.seq_points)} Punkte, {dauer:.2f} s, "
                f"nicht fahrbereit")
        self._update_locks()

    def _seq_locks(self):
        """Die Taste 'Folge fahren' scharf oder stumpf.

        Eigene Stelle und trotzdem aus _update_locks gerufen: die Regel
        gehoert zur Folge, der Zeitpunkt zu den Sperren. Wer sie in
        _update_locks schriebe, haette dort einen Block, der bei
        geschlossenem Fenster auf nichts zeigt.
        """
        taste = self.seq_drive_button
        if taste is None or not taste.winfo_exists():
            return
        fahrbar = (self.seq_hold is not None
                   and not self._busy()
                   and not self.stopped)
        taste.configure(state="normal" if fahrbar else "disabled")

    # -- Ausgabe -----------------------------------------------------------

    def _seq_export(self):
        """Die geplante Folge als CSV fuer die Bahnansicht schreiben.

        Derselbe Aufbau wie eine Aufzeichnung des Bahnrekorders, damit
        bahn_viewer2.py sie ohne Sonderfall liest. Der Unterschied zur
        Aufzeichnung ist nicht das Format, sondern die Herkunft: hier steht
        die gerechnete Bahn, dort die gefahrene. Zusammengelegt waeren beide
        nicht mehr zu unterscheiden -- deshalb traegt der Dateiname 'plan'.

        Ein eigener Rekorder mit Schwellen auf null, nicht der laufende: die
        Filterung, die eine Aufzeichnung braucht, waere hier falsch. Der
        Planer hat die Stuetzpunkte gesetzt, und wer sie ansieht, will genau
        die sehen und nicht eine Auswahl daraus. Der laufende Rekorder bleibt
        unangetastet; eine Aufzeichnung, die durch einen Export ihre Punkte
        verliert, waere ein Ueberraschung der unangenehmen Sorte.
        """
        if not self.seq_points:
            self._log("Keine geplante Folge vorhanden. Erst 'Folge planen'.",
                      "warn")
            return
        tcp = self._tcp_link()
        if tcp is None:
            self._log("Kein eindeutiger Endeffektor, die CSV haette keine "
                      "TCP-Spalten", "error")
            return

        path = filedialog.asksaveasfilename(
            title="Geplante Folge exportieren",
            initialdir=backup_dir_for(self.urdf_path),
            initialfile=f"plan_{datetime.now():%Y%m%d_%H%M%S}.csv",
            defaultextension=".csv",
            filetypes=[("CSV", "*.csv"), ("Alle Dateien", "*.*")])
        if not path:
            return

        # Alle drei Ruheschwellen auf null: es wird nichts gefiltert. Der
        # Rekorder verwirft einen Punkt nur, wenn er nach allen drei Massen
        # ruht -- eine einzige Null genuegt deshalb schon. Trotzdem stehen
        # alle drei da: wer die Zeile liest, soll nicht erst die Verknuepfung
        # in sample nachschlagen muessen, um zu wissen, dass hier nichts
        # herausfaellt. Der Preis ist, dass sich einzelne der drei Zahlen
        # nicht durch eine Sonde absichern lassen -- die Sonde deckt den
        # Aufruf als Ganzes ab.
        recorder = PathRecorder(min_dist=0.0, min_angle=0.0, min_joint=0.0)
        frames = self._record_frames()
        recorder.start(self._record_columns(), frames, self._record_chain(),
                       couplings=self._record_couplings(),
                       robot_name=self.robot_var.get(),
                       drehkonvention=ANZEIGE.name)

        # Auf einer Kopie rechnen. Der angezeigte Stand des Roboters gehoert
        # dem Bediener und darf sich durch einen Export nicht bewegen.
        stand = dict(self.positions)
        geschrieben = 0
        for stamp, positions, _velocities in self.seq_points:
            for name, value in positions.items():
                if name in stand:
                    stand[name] = float(value)
            apply_couplings(self.joints, stand)
            poses = forward_kinematics(self.links, self.joints, stand)
            if tcp not in poses:
                continue
            xyz, abc = konvention.to_xyz_abc(poses[tcp], recorder.konvention)
            werte = [math.degrees(stand.get(j["name"], 0.0))
                     if is_rotational(j) else stand.get(j["name"], 0.0)
                     for j in self._movable_joints()]
            flach = self._frame_snapshot(poses, frames)
            if recorder.sample(xyz, abc, werte, flach, now=stamp):
                geschrieben += 1

        if not geschrieben:
            self._log("Nichts zu schreiben: kein Stuetzpunkt lieferte eine "
                      "TCP-Lage", "error")
            return
        try:
            written = recorder.write_csv(path)
        except OSError as exc:
            messagebox.showerror("Export", f"Schreiben fehlgeschlagen:\n{exc}")
            self._log(f"Export fehlgeschlagen: {exc}", "error")
            return
        self._log(f"{written} Punkte der geplanten Folge nach {path} "
                  f"geschrieben", "ok")
        self._seq_write("")
        self._seq_write(f"{written} Punkte nach {path} geschrieben. "
                        f"Ansehen mit ./bahn_viewer2.py {path}", "ok")


    # ----------------------------------------------------------------------
    # Kinematik
    # ----------------------------------------------------------------------

    def _tcp_link(self):
        """Name des aktiven Endeffektors oder None.

        Getrennt von _tcp_pose, weil der Rekorder den Namen braucht und die
        Lage aus einem Satz Frames nimmt, den er ohnehin schon gerechnet hat.

        Zwei Quellen in fester Reihenfolge:

            1. moveit_config.tip, sofern gesetzt und im Modell vorhanden,
            2. sonst das einzige Blatt ausser der Wurzel.

        Bis 2.15 gab es nur den zweiten Weg, und er traegt, solange genau ein
        Werkzeug am Flansch haengt. Beim zweiten -- Greifer und Kamera,
        Brenner und Messspitze -- gibt es zwei Blaetter. Die Ableitung hat
        dann keine Grundlage, nach der sie waehlen koennte, und liefert None:
        die TCP-Zeile bleibt leer, der Rekorder findet keinen TCP, und kaputt
        ist trotzdem nichts. Ein Zustand, den man erst versteht, wenn man
        diese Funktion gelesen hat.

        Die Wahl steht bewusst in moveit_config.tip und nicht in einem eigenen
        Feld daneben. Genau dieser Wert geht als Kettenende in build_srdf und
        als ik_link_name an /compute_ik. Ein zweites Feld waere ein zweiter
        Begriff fuer dieselbe Sache, und die beiden waeren irgendwann
        verschieden.

        Der Name muss kein Blatt sein. Wer den Flansch waehlt, obwohl ein
        Werkzeug darunter haengt, meint das vermutlich so -- etwa um zu
        zeigen, was der Werkzeug-Offset ausmacht.
        """
        known = {l["name"] for l in self.links}
        chosen = self.moveit_config.tip
        if chosen and chosen in known:
            return chosen
        roots = root_links(self.links, self.joints)
        base = roots[0] if roots else "?"
        tcps = [n for n in leaf_links(self.links, self.joints) if n != base]
        return tcps[0] if len(tcps) == 1 else None

    def _tcp_pose(self, positions=None):
        """Lage des einzigen Endeffektors oder None, als (xyz, (A, B, C)).

        In der gewaehlten Anzeigekonvention. Alle Aufrufer zeigen die Zahlen
        an oder legen sie in ein Zielfeld; keiner schreibt sie in eine Datei.
        Der Weg in die URDF laeuft ueber to_xyz_rpy und bleibt bei ZYX.

        positions ist seit 3.5 waehlbar, fuer die Lage eines Achsziels. Ohne
        Angabe gilt der aktuelle Stand, wie bisher.
        """
        tcp = self._tcp_link()
        if tcp is None:
            return None
        stand = self.positions if positions is None else positions
        poses = forward_kinematics(self.links, self.joints, stand)
        if tcp not in poses:
            return None
        return ANZEIGE.zerlege(poses[tcp])

    def _tcp_mark_degenerate(self, entartet):
        """Die TCP-Zeile faerben, wenn A und C zerfallen.

        Die Farbe traegt keine eigene Aussage: der Name der Konvention steht
        ohnehin in der Zeile, und ein Blick auf B sagt dasselbe. Sie ist eine
        Wiederholung fuer den, der nicht hinsieht -- so wie bei den Stufen des
        Jogberichts seit 2.13.
        """
        label = getattr(self, "tcp_label", None)
        if label is None or not label.winfo_exists():
            return
        label.configure(style="TcpWarn.TLabel" if entartet else "Tcp.TLabel")

    def _update_drive_tcp(self):
        """Die TCP-Zeile im Kopf des Fahrfensters nachziehen.

        Eigener Takt und trotzdem keine zweite Quelle: die Zahlen kommen aus
        _pose_text, also aus derselben Vorwaertskinematik und derselben
        Anzeigekonvention wie die Zeile im Hauptfenster. Was hier steht, ist
        immer dasselbe wie dort -- nur oefter.

        Oefter ist der Punkt. _update_kinematics fuellt die Frametabelle mit
        und ist damit zu teuer fuer 50 Hz; waehrend einer Fahrt laeuft es
        genau einmal, am Ende. Die Zeile im Hauptfenster steht in dieser Zeit
        still, und das ist dort richtig -- eine Tabelle, die im Fahrtakt
        springt, liest niemand. Eine TCP-Anzeige, die genau dann einfriert,
        wenn sich etwas bewegt, waere dagegen schlimmer als keine.

        Gerechnet wird nur bei offenem Fenster. Die Pruefung steht vor der
        Kinematik und nicht danach: ist das Fahrfenster zu, kostet der
        Aufruf im Fahrtakt nichts.

        Drei Stufen, wie bei der Zeile im Hauptfenster. Rot heisst, dass es
        nichts anzuzeigen gibt -- kein TCP, oder einer, der nicht an der
        Basis haengt. Gelb heisst, dass die Winkel entartet sind: die Zahlen
        stimmen, aber zwei von ihnen springen bei der naechsten kleinen
        Bewegung. Die Farbe wiederholt nur, was ohnehin dasteht.
        """
        label = self.drive_tcp_label
        if label is None or not label.winfo_exists():
            return
        tcp = self._tcp_link()
        if tcp is None:
            self.drive_tcp_var.set("Kein Endeffektor gewaehlt")
            label.configure(style="DriveTcpBad.TLabel")
            return
        pose = self._tcp_pose()
        if pose is None:
            self.drive_tcp_var.set(f"TCP {tcp} haengt nicht an der Basis")
            label.configure(style="DriveTcpBad.TLabel")
            return
        self.drive_tcp_var.set(f"{tcp}\n{self._pose_text(pad=True, pose=pose)}")
        label.configure(style="DriveTcpWarn.TLabel"
                        if ANZEIGE.entartet(pose[1]) else "DriveTcp.TLabel")

    def _pose_text(self, pad=True, pose=None):
        """TCP-Lage als Text. Mit pad stehen die Spalten im Protokoll untereinander.

        pose wird durchgereicht und nicht neu geholt, wenn der Aufrufer die
        Lage schon hat. Das spart im Fahrtakt einen zweiten Durchlauf der
        Vorwaertskinematik -- und, wichtiger, es kann keine zwei Staende in
        einer Zeile geben. Ohne den Durchgriff stuenden Zahl und Faerbung aus
        zwei Rechnungen nebeneinander, die zwar fast immer gleich sind, aber
        eben nur fast.

        Die Konvention steht dabei. Drei Winkel ohne die Vorschrift, nach der
        sie gebildet wurden, beschreiben keine Lage -- im Protokoll faellt das
        besonders auf, weil dort Zeilen von vor und nach einem Umschalten
        untereinander stehen.
        """
        if pose is None:
            pose = self._tcp_pose()
        if pose is None:
            return "Achswerte uebernommen"
        xyz, abc = pose
        width, digits = (8, 2) if pad else (0, 1)
        return (f"X {xyz[0]:{width}.4f} Y {xyz[1]:{width}.4f} Z {xyz[2]:{width}.4f} m   "
                f"A {math.degrees(abc[0]):{width}.{digits}f} "
                f"B {math.degrees(abc[1]):{width}.{digits}f} "
                f"C {math.degrees(abc[2]):{width}.{digits}f} deg  {ANZEIGE.name}")

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

            xyz, abc = ANZEIGE.zerlege(matrix)
            tags = ("tcp",) if name in leaves and name != base else ()
            self.frame_table.item(item, tags=tags, values=(
                f"{xyz[0]:.4f}", f"{xyz[1]:.4f}", f"{xyz[2]:.4f}",
                f"{math.degrees(abc[0]):.2f}",
                f"{math.degrees(abc[1]):.2f}",
                f"{math.degrees(abc[2]):.2f}"))

        self._fill_joint_table()
        self.after_idle(self.frame_table.fit)

        # Der Bezug steht seit 2.16 in der Zeile. Die Zahlen kommen aus
        # forward_kinematics und sind gegen die Wurzel angeschrieben; kommt
        # ein Basisframe darueber, ist die Wurzel eine andere und dieselbe
        # Zeile meint etwas anderes. Ohne Fehler, ohne Meldung -- der Bezug
        # ist die einzige Stelle, an der das auffaellt.
        tcps = [n for n in leaves if n != base]
        tcp = self._tcp_link()
        if tcp is not None and tcp in poses:
            quelle = "gewaehlt" if self.moveit_config.tip == tcp else "abgeleitet"
            self.tcp_var.set(f"TCP {tcp} gegen {base} ({quelle})   "
                             f"{self._pose_text(pad=False)}")
            # Die Entartung faerbt die Zeile. Sichtbar ist sonst nichts: die
            # Zahlen bleiben lesbar, sie springen nur bei der naechsten
            # kleinen Bewegung um zweistellige Betraege. Bei ZYZ und ZXZ
            # liegt die Stelle in der Grundstellung, also dort, wo man am
            # ehesten hinschaut.
            _, abc = ANZEIGE.zerlege(poses[tcp])
            self._tcp_mark_degenerate(ANZEIGE.entartet(abc))
        elif tcps:
            self.tcp_var.set(
                f"Mehrere Endeffektoren, keiner gewaehlt: "
                f"{', '.join(sorted(tcps))}")
        else:
            self.tcp_var.set("Kein Endeffektor gefunden")

        # Nicht jede Aenderung ist eine Achsbewegung: ein neues Modell, eine
        # andere Drehkonvention, ein anderer aktiver TCP gehen an
        # _push_positions vorbei und muessen trotzdem oben ankommen.
        self._update_drive_tcp()

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
        field_row(3, "WINKEL  rpy [deg]  ZYX",
                  [("roll", "Roll"), ("pitch", "Pitch"), ("yaw", "Yaw")])
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
        # Eigener Knopf und nicht unter "Glied anhaengen": dort waeren Achse
        # und Anschlaege mitzufuellen, die ein fester Frame nicht hat. Der Weg
        # ueber "Glied anhaengen" mit Typ fixed bleibt bestehen und macht
        # dasselbe -- dieses Fenster fragt nur nach dem, was hier zaehlt, und
        # nennt die Rolle beim Namen.
        ttk.Button(structure2, text="Frames und Werkzeuge...",
                   command=self._open_frame_window).pack(side="left")
        ttk.Button(structure2, text="Gelenk und Teilbaum loeschen",
                   command=self._delete_branch).pack(side="left", padx=(8, 0))
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
        self._close_frame_window()
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
        field_row(3, "WINKEL  rpy [deg]  ZYX",
                  [("roll", "Roll"), ("pitch", "Pitch"), ("yaw", "Yaw")],
                  ["0.000"] * 3)
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

    # ----------------------------------------------------------------------
    # Frames und Werkzeuge
    # ----------------------------------------------------------------------

    def _sync_tip(self, links):
        """Aktiven TCP verwerfen, wenn der Link nicht mehr da ist.

        Der Name steht in moveit_config.tip und geht von dort in build_srdf.
        Zeigt er ins Leere, entsteht eine SRDF mit einem Kettenende, das es
        nicht gibt: move_group faehrt hoch, findet die Gruppe nicht und
        antwortet auf nichts mehr. Ein Fehlerbild, das nach ROS aussieht und
        aus dieser Oberflaeche stammt.

        Zwei Wege fuehren dorthin -- ein anderes Modell laden und einen
        Teilbaum loeschen -- deshalb steht die Pruefung hier und wird von
        beiden gerufen. Zurueckgesetzt wird auf leer, nicht auf ein anderes
        Blatt: raten waere schlimmer als ableiten, und ableiten tut
        _tcp_link ohnehin.
        """
        tip = self.moveit_config.tip
        if not tip or tip in {l["name"] for l in links}:
            return
        self.moveit_config.tip = ""
        self._log(f"Aktiver TCP {tip} ist nicht mehr im Modell, "
                  f"wird wieder abgeleitet", "warn")

    def _fixed_frames(self):
        """[(link, elter, gelenk)] aller festen Frames, Wurzel zuerst.

        Feste Gelenke sind die Frames im Wortsinn: sie tragen keinen
        Freiheitsgrad, sondern nur eine Lage. Beweglichen Gelenken gehoert
        ihre Lage nicht allein -- dort kommt der Achswert hinzu -- deshalb
        stehen sie nicht in dieser Liste.
        """
        order = {name: index for index, name in enumerate(self._frame_order())}
        rows = [(j["child"], j["parent"], j["name"])
                for j in self.joints if j["type"] == "fixed"]
        rows.sort(key=lambda row: order.get(row[0], len(order)))
        return rows

    def _open_frame_window(self):
        """Frames und Werkzeuge anlegen und den aktiven TCP waehlen.

        Ein eigenes Fenster und keine weitere Karteikarte im Editor: was hier
        entsteht, ist kein Gelenk. Die Gelenkseite fragt nach Achse und
        Anschlaegen, und beides waere hier leer -- Felder, die nie etwas
        aufnehmen, sind eine Einladung, sie doch zu fuellen.

        Die beiden Rollen schreiben an verschiedene Enden der Kette und
        brauchen deshalb verschiedene Angaben. Werkzeug fragt nach dem Link,
        unter den es haengt; Bezug fragt danach nicht, weil es nur eine
        Wurzel gibt, ueber die es gehen kann.
        """
        if self.add_window is not None and self.add_window.winfo_exists():
            self._show_window(self.add_window)         # erst das andere zuende
            return
        if self._show_window(self.frame_window):
            self._frame_refresh()
            return
        if not self.links:
            self._log("Kein Modell geladen", "warn")
            return

        window = tk.Toplevel(self.editor or self)
        window.title("Frames und Werkzeuge")
        window.configure(background=BG)
        window.transient(self.editor or self)
        window.resizable(False, False)
        window.protocol("WM_DELETE_WINDOW", self._close_frame_window)
        self._hold_grab(window)
        self.frame_window = window

        card = self._card(window)
        card.pack(fill="both", expand=True, padx=PAD, pady=PAD)
        tab = ttk.Frame(card, style="Card.TFrame", padding=(18, 16))
        tab.pack(fill="both", expand=True)
        self.frame_entries = {}

        link_names = [l["name"] for l in self.links]
        joint_names = {j["name"] for j in self.joints}

        # -- Rolle ---------------------------------------------------------
        head = ttk.Frame(tab, style="Card.TFrame")
        head.grid(row=0, column=0, columnspan=4, sticky="w", pady=(0, 12))
        ttk.Label(head, text="ROLLE", style="Section.TLabel").pack(side="left")
        for role in (ROLE_TOOL, ROLE_BASE):
            ttk.Radiobutton(head, text=role, value=role,
                            variable=self.frame_role_var,
                            command=self._frame_role_changed).pack(
                side="left", padx=(14, 0))

        ttk.Label(tab, textvariable=self.frame_hint_var, style="Muted.TLabel",
                  wraplength=520, justify="left").grid(
            row=1, column=0, columnspan=4, sticky="w", pady=(0, 12))

        # -- Namen ---------------------------------------------------------
        names = ttk.Frame(tab, style="Card.TFrame")
        names.grid(row=2, column=0, columnspan=4, sticky="w", pady=(0, 6))
        ttk.Label(names, text="Framename", style="Card.TLabel").pack(side="left")
        entry = self._entry(names, width=16,
                            value=self._suggest_name("tcp", set(link_names)))
        entry.configure(justify="left")
        entry.pack(side="left", padx=(10, 18))
        self.frame_entries["link"] = entry

        ttk.Label(names, text="Gelenkname", style="Card.TLabel").pack(side="left")
        entry = self._entry(names, width=16,
                            value=self._suggest_name("frame", joint_names))
        entry.configure(justify="left")
        entry.pack(side="left", padx=(10, 0))
        self.frame_entries["joint"] = entry

        # -- Anschlusspunkt ------------------------------------------------
        hook = ttk.Frame(tab, style="Card.TFrame")
        hook.grid(row=3, column=0, columnspan=4, sticky="w", pady=(0, 6))
        self.frame_parent_label = ttk.Label(hook, text="haengt an",
                                            style="Card.TLabel")
        self.frame_parent_label.pack(side="left")
        self.frame_parent_combo = ttk.Combobox(
            hook, textvariable=self.frame_parent_var, state="readonly",
            width=20, values=link_names)
        self.frame_parent_combo.pack(side="left", padx=(10, 0))
        current = self._tcp_link()
        self.frame_parent_var.set(current if current in link_names
                                  else link_names[-1])

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
                self.frame_entries[key] = field

        field_row(4, "LAENGE  [m]", [("x", "X"), ("y", "Y"), ("z", "Z")],
                  ["0.00000", "0.00000", "0.00000"])
        field_row(5, "WINKEL  rpy [deg]  ZYX",
                  [("roll", "Roll"), ("pitch", "Pitch"), ("yaw", "Yaw")],
                  ["0.000"] * 3)

        actions = ttk.Frame(tab, style="Card.TFrame")
        actions.grid(row=6, column=0, columnspan=4, sticky="w", pady=(16, 0))
        ttk.Button(actions, text="Anlegen", command=self._frame_apply).pack(
            side="left")
        ttk.Button(actions, text="Schliessen",
                   command=self._close_frame_window).pack(side="left", padx=(8, 0))

        ttk.Separator(tab, orient="horizontal").grid(
            row=7, column=0, columnspan=4, sticky="ew", pady=16)

        # -- Aktiver TCP ---------------------------------------------------
        ttk.Label(tab, text="AKTIVER TCP", style="Section.TLabel").grid(
            row=8, column=0, sticky="w")
        choose = ttk.Frame(tab, style="Card.TFrame")
        choose.grid(row=8, column=1, columnspan=3, sticky="w")
        self.frame_tip_combo = ttk.Combobox(
            choose, textvariable=self.frame_tip_var, state="readonly", width=24)
        self.frame_tip_combo.pack(side="left")
        ttk.Button(choose, text="Setzen", command=self._frame_set_tip).pack(
            side="left", padx=(10, 0))

        ttk.Label(tab, text="Die Wahl geht als Kettenende in die SRDF und als "
                            "ik_link_name an /compute_ik. Sie wirkt erst nach "
                            "einem Neustart von move_group.",
                  style="Muted.TLabel", wraplength=520, justify="left").grid(
            row=9, column=0, columnspan=4, sticky="w", pady=(8, 0))

        ttk.Separator(tab, orient="horizontal").grid(
            row=10, column=0, columnspan=4, sticky="ew", pady=16)

        ttk.Label(tab, text="FESTE FRAMES", style="Section.TLabel").grid(
            row=11, column=0, sticky="nw")
        self.frame_list_var = tk.StringVar(value="")
        # Festbreite, damit die Spalte "unter ..." untereinander steht. Die
        # Namen sind verschieden lang; mit der Grundschrift saehe die
        # Aufstellung aus wie eine Liste ohne Ordnung.
        ttk.Label(tab, textvariable=self.frame_list_var, style="Muted.TLabel",
                  justify="left", font="TkFixedFont").grid(
            row=11, column=1, columnspan=3, sticky="w")

        self._frame_role_changed()
        self._frame_refresh()

    def _close_frame_window(self):
        if self.frame_window is not None and self.frame_window.winfo_exists():
            self.frame_window.grab_release()
            self.frame_window.destroy()
        self.frame_window = None
        self.frame_entries = {}
        self.frame_tip_combo = None
        self.frame_list_var = None

    def _frame_role_changed(self):
        """Anschlussfeld und Erklaerung auf die gewaehlte Rolle umschalten."""
        if not self.frame_entries:
            return
        tool = self.frame_role_var.get() == ROLE_TOOL
        state = "readonly" if tool else "disabled"
        self.frame_parent_combo.configure(state=state)
        self.frame_parent_label.configure(
            text="haengt an" if tool else "geht ueber die Wurzel")

        roots = root_links(self.links, self.joints)
        base = roots[0] if roots else "?"
        if tool:
            self.frame_hint_var.set(
                "Blatt am unteren Ende der Kette. Steht rechts in der "
                "Transformation, faehrt mit dem Flansch mit und geht in die "
                "Ableitung ein: eine Drehung am Flansch bewegt den Punkt.")
        else:
            self.frame_hint_var.set(
                f"Neue Wurzel ueber {base}. Steht links in der Transformation "
                f"und haengt nicht von den Achswerten ab -- der Roboter bewegt "
                f"sich nicht. Aber alle Frames sind danach gegen den neuen "
                f"Link angeschrieben statt gegen {base}.")

    def _frame_refresh(self):
        """Auswahlliste und Aufstellung an das aktuelle Modell anpassen."""
        if self.frame_window is None or not self.frame_window.winfo_exists():
            return
        link_names = [l["name"] for l in self.links]
        if self.frame_tip_combo is not None:
            self.frame_tip_combo.configure(values=[TIP_AUTO] + link_names)
            tip = self.moveit_config.tip
            self.frame_tip_var.set(tip if tip in link_names else TIP_AUTO)
        if self.frame_list_var is not None:
            rows = self._fixed_frames()
            if rows:
                width = max(len(child) for child, _p, _j in rows)
                self.frame_list_var.set("\n".join(
                    f"{child:<{width}}  unter {parent}  ({joint})"
                    for child, parent, joint in rows))
            else:
                self.frame_list_var.set("keine")
        self._frame_role_changed()

    def _frame_apply(self):
        """Eingaben pruefen und den Frame an der passenden Stelle einsetzen."""
        role = self.frame_role_var.get()
        link_name = self.frame_entries["link"].get().strip()
        joint_name = self.frame_entries["joint"].get().strip()
        parent = self.frame_parent_var.get()

        problems = [p for p in (
            name_problem(link_name, "Link", {l["name"] for l in self.links}),
            name_problem(joint_name, "Gelenk", {j["name"] for j in self.joints}),
        ) if p]
        if link_name and link_name == joint_name:
            problems.append("Link und Gelenk duerfen nicht gleich heissen")
        if role == ROLE_TOOL and parent not in {l["name"] for l in self.links}:
            problems.append("Kein gueltiger Anschlusspunkt gewaehlt")
        if problems:
            messagebox.showerror("Frame", "\n".join(problems))
            for problem in problems:
                self._log(problem, "error")
            return

        try:
            xyz = [self._read_entry(self.frame_entries, k) for k in ("x", "y", "z")]
            rpy = [math.radians(self._read_entry(self.frame_entries, k))
                   for k in ("roll", "pitch", "yaw")]
        except ValueError:
            self._log("Eingabe nicht lesbar, nichts angelegt", "error")
            return

        if role == ROLE_TOOL:
            def mutate(draft):
                # Achse und Anschlaege entfallen: ein fester Frame traegt
                # keinen Freiheitsgrad. append_limb entfernt beides, wenn es
                # None bekommt.
                append_limb(draft, link_name, joint_name, "fixed", parent,
                            xyz, rpy, None, None, None)

            text = f"Werkzeug {link_name} unter {parent} angelegt"
        else:
            def mutate(draft):
                insert_above_root(draft, link_name, joint_name, xyz, rpy)

            text = f"Basisframe {link_name} ueber die Wurzel gesetzt"

        if not self._commit(text, mutate):
            return

        # Nach dem Anlegen sind Namen und Modell andere. Neue Vorschlaege,
        # damit ein zweiter Frame nicht am doppelten Namen scheitert.
        self._set_entry(self.frame_entries, "link",
                        self._suggest_name("tcp", {l["name"] for l in self.links}))
        self._set_entry(self.frame_entries, "joint",
                        self._suggest_name("frame", {j["name"] for j in self.joints}))
        self._frame_refresh()

        if role == ROLE_BASE:
            self._log(f"Bezug der Frame-Tabelle ist jetzt {link_name}", "warn")
        elif self._tcp_link() != link_name:
            self._log(f"{link_name} ist nicht der aktive TCP -- "
                      f"unter AKTIVER TCP waehlen", "warn")

    def _frame_set_tip(self):
        """Aktiven TCP setzen. Schreibt in dasselbe Feld, das die SRDF liest."""
        choice = self.frame_tip_var.get()
        tip = "" if choice == TIP_AUTO else choice
        if tip and tip not in {l["name"] for l in self.links}:
            self._log(f"{tip} ist kein Link des Modells", "error")
            return
        if tip == self.moveit_config.tip:
            self._log("Aktiver TCP unveraendert", "info")
            return

        # Eine laufende Aufzeichnung faellt. _record_frames stellt den TCP ans
        # Ende, und write_csv schreibt den letzten Namen als #tcp in den Kopf.
        # Ein Wechsel mitten in der Aufnahme laesst die Spaltenzahl gleich und
        # verschiebt nur die Bedeutung -- die Datei waere lesbar und falsch.
        # _rebuild_from_root prueft das, laeuft hier aber nicht: am Modell
        # aendert sich nichts.
        if self.recorder.count or self.recorder.active:
            self._drop_record("Aktiver TCP geaendert")

        self.moveit_config.tip = tip
        self._moveit_stale("Aktiver TCP geaendert")
        self._update_kinematics()
        self._drive_ik_hint()
        self._frame_refresh()
        self._log(f"Aktiver TCP: {tip or 'aus dem Modell abgeleitet'}", "ok")

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
        field_row(3, "WINKEL  rpy [deg]  ZYX",
                  [("roll", "Roll"), ("pitch", "Pitch"), ("yaw", "Yaw")],
                  ["0.000"] * 3)
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

        # Umbenennen, Anlegen, Herausnehmen, Typwechsel: jedes davon kann den
        # Achssatz oder den Framesatz verschieben. Laeuft dann noch eine
        # Aufzeichnung, gehoerten die naechsten Punkte in andere Spalten als
        # die bisherigen. Verglichen wird der Aufbau, nicht das Ereignis --
        # eine Farbaenderung am Visual ruehrt hier nichts an.
        if self.recorder.count or self.recorder.active:
            if (self._record_columns() != self.recorder.joint_names
                    or self._record_frames() != self.recorder.frame_names):
                self._drop_record("Aufbau des Modells geaendert")

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
        self._sync_tip(self.links)
        if self.params_window is not None and self.params_window.winfo_exists():
            # Die Tabelle der Achsgrenzen haengt an der Gelenkliste und muss
            # deshalb komplett neu entstehen.
            self._close_params()
            self._open_params()
        if self.frame_window is not None and self.frame_window.winfo_exists():
            self._frame_refresh()

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

        if self.state in (STATE_IK, STATE_PLANNING):
            self._drive_status()
        if self.state in (STATE_SEQ_IK, STATE_SEQUENCE):
            self._seq_status()

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
            elif kind == "ik":
                self._on_ik_result(value)
            elif kind == "sequence":
                self._on_sequence_result(value)
            elif kind == "sequence_ready":
                # Nur melden, wenn es fehlt. Dass etwas da ist, das da
                # sein soll, ist keine Nachricht.
                if not value and self.ros.moveit_ready:
                    self._log("/plan_sequence_path fehlt. move_group laeuft, "
                              "traegt aber die Sequence-Capability nicht. "
                              "Punktfolgen gehen damit nicht.", "warn")
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
        self._close_sequence()
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
