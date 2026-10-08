#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
patch_pilz_sampling_time_1.py
=============================

Macht die Abtastzeit des Pilz Industrial Motion Planner zu einem echten
ROS-Parameter.

AUSGANGSLAGE
------------
`TrajectoryGenerator::generate(scene, req, res, double sampling_time = 0.1)`
deklariert die Abtastzeit als Default-Argument. Der einzige Aufrufer,
`PlanningContextBase::solve()`, uebergibt das vierte Argument nicht. Damit ist
0,1 s eine Konstante, die von aussen nicht erreichbar ist. Fuer eine Messreihe
ueber mehrere Abtastzeiten waere pro Stuetzstelle ein Rebuild noetig.

WARUM DER WEG UEBER LimitsContainer
-----------------------------------
Naheliegend waere, die Abtastzeit als zusaetzliches Konstruktor-Argument durch
PlanningContextBase -> PlanningContextPTP/LIN/CIRC -> PlanningContextLoader ->
PlanningContextLoaderPTP/LIN/CIRC zu reichen. Das sind neun Dateien, und
PlanningContextLoader ist eine exportierte pluginlib-Basisklasse: eine
Signaturaenderung dort bricht die ABI gegenueber jedem extern gebauten Loader.

`LimitsContainer` ist dagegen das einzige Objekt, das ohnehin bereits den
vollstaendigen Weg CommandPlanner -> Loader -> PlanningContext ->
TrajectoryGenerator zurueckgelegt hat. Ein zusaetzliches Feld dort kostet vier
Dateien, keine Signaturaenderung und keinen ABI-Bruch.

Der Preis ist die Benennung: eine Abtastzeit ist keine Grenze. Das ist eine
bewusst in Kauf genommene Unschaerfe und im erzeugten Code so kommentiert.
Wer die semantisch saubere Variante will, muss die neun Dateien anfassen.

WAS DAS SKRIPT AENDERT (4 Dateien)
----------------------------------
1. include/pilz_industrial_motion_planner/limits_container.hpp
   Konstanten DEFAULT_/MIN_/MAX_SAMPLING_TIME, Feld sampling_time_,
   Setter/Getter.
2. src/limits_container.cpp
   Initialisierung im Konstruktor, Implementierung von Setter/Getter.
3. include/pilz_industrial_motion_planner/planning_context_base.hpp
   solve() uebergibt limits_.getSamplingTime() als viertes Argument.
4. src/pilz_industrial_motion_planner.cpp
   CommandPlanner::initialize() liest den Parameter <ns>.sampling_time,
   prueft ihn, protokolliert ihn und legt ihn in den LimitsContainer.

KEINE STILLE KORREKTUR
----------------------
Ein Wert ausserhalb [1e-4, 1.0] fuehrt zu RCLCPP_ERROR und `return false` aus
initialize(). move_group startet dann nicht. Das ist Absicht: eine still
zurechtgebogene Abtastzeit wuerde eine ganze Messreihe unbrauchbar machen,
ohne dass es jemand merkt. Ein Startabbruch faellt sofort auf.

NACHWEIS, DASS DER PATCH LAEUFT
-------------------------------
initialize() gibt beim Start eine RCLCPP_WARN-Zeile "PILZ-PATCH aktiv" mit
Abtastzeit und Build-Zeitstempel aus. Fehlt die Zeile im move_group-Log, ist
das Binaerpaket aus /opt/ros/jazzy geladen und nicht der Overlay-Build.

VERWENDUNG
----------
    python3 patch_pilz_sampling_time_1.py <pfad-zum-paket>
    python3 patch_pilz_sampling_time_1.py <pfad-zum-paket> --zurueck
    python3 patch_pilz_sampling_time_1.py <pfad-zum-paket> --pruefen

<pfad-zum-paket> ist das Verzeichnis, das package.xml und CMakeLists.txt
enthaelt, also z. B.
    ~/ws_moveit/src/moveit2_src/moveit_planners/pilz_industrial_motion_planner

ANSCHLIESSEND IN DER YAML
-------------------------
    pilz_industrial_motion_planner:
      planning_plugin: pilz_industrial_motion_planner/CommandPlanner
      sampling_time: 0.008

Der Dezimalpunkt ist Pflicht. `sampling_time: 1` waere in YAML eine Ganzzahl;
darauf reagiert das Skript mit einer eigenen Fehlermeldung statt mit einer
rclcpp-Ausnahme.

Getestet gegen pilz_industrial_motion_planner 2.12.4 (ROS 2 Jazzy).
"""

import argparse
import difflib
import os
import re
import sys

# Version, gegen die die Anker geprueft wurden. Andere Versionen koennen
# funktionieren, werden aber gemeldet -- siehe pruefe_stand().
ERWARTETE_VERSION = "2.12.4"

# Erkennungsmarke. Steht in jeder geaenderten Datei. Verhindert doppeltes
# Anwenden und dient --pruefen als Nachweis.
MARKE = "PILZ-PATCH sampling_time"

BACKUP_ENDUNG = ".pilzorig"


# ---------------------------------------------------------------------------
# Ersetzungsregeln
# ---------------------------------------------------------------------------
# Jede Regel ist (relativer_pfad, anker, ersatz). Der Anker muss im
# Dateiinhalt GENAU EINMAL vorkommen. Kommt er nicht oder mehrfach vor, bricht
# das Skript mit Klartext ab -- kein Raten, keine Teilanwendung.

H_LIMITS = "include/pilz_industrial_motion_planner/limits_container.hpp"
C_LIMITS = "src/limits_container.cpp"
H_CONTEXT = "include/pilz_industrial_motion_planner/planning_context_base.hpp"
C_PLANNER = "src/pilz_industrial_motion_planner.cpp"


REGELN = [

    # -- 1a. Konstanten vor der Klassendefinition -------------------------
    (
        H_LIMITS,
        "namespace pilz_industrial_motion_planner\n"
        "{\n"
        "/**\n"
        " * @brief This class combines CartesianLimit and JointLimits into on single\n"
        " * class.\n"
        " */\n"
        "class LimitsContainer\n",

        "namespace pilz_industrial_motion_planner\n"
        "{\n"
        "// " + MARKE + " -----------------------------------------------\n"
        "// Abtastzeit der Trajektorienerzeugung. Bis zu diesem Patch war der Wert\n"
        "// ausschliesslich als Default-Argument von TrajectoryGenerator::generate()\n"
        "// vorhanden und damit von aussen nicht erreichbar.\n"
        "//\n"
        "// DEFAULT_SAMPLING_TIME haelt exakt den bisherigen Wert. Wer den Parameter\n"
        "// nicht setzt, bekommt bit-genau das Verhalten vor dem Patch -- das ist die\n"
        "// Voraussetzung dafuer, dass alte und neue Messreihen vergleichbar bleiben.\n"
        "//\n"
        "// Die Grenzen sind bewusst weit. Sie fangen Tippfehler ab (0 s, negative\n"
        "// Werte, versehentlich Millisekunden statt Sekunden), nicht unguenstige\n"
        "// Parametrierungen. Was innerhalb der Grenzen sinnvoll ist, entscheidet die\n"
        "// Messung, nicht der Planer.\n"
        "inline constexpr double DEFAULT_SAMPLING_TIME{ 0.1 };\n"
        "inline constexpr double MIN_SAMPLING_TIME{ 1.0e-4 };\n"
        "inline constexpr double MAX_SAMPLING_TIME{ 1.0 };\n"
        "// ------------------------------------------------------------------------\n"
        "\n"
        "/**\n"
        " * @brief This class combines CartesianLimit and JointLimits into on single\n"
        " * class.\n"
        " */\n"
        "class LimitsContainer\n",
    ),

    # -- 1b. Setter/Getter in den oeffentlichen Teil ----------------------
    (
        H_LIMITS,
        "  const cartesian_limits::Params& getCartesianLimits() const;\n"
        "\n"
        "private:\n",

        "  const cartesian_limits::Params& getCartesianLimits() const;\n"
        "\n"
        "  // " + MARKE + " ---------------------------------------------\n"
        "  // Anmerkung zur Einordnung: eine Abtastzeit ist keine Grenze und gehoert\n"
        "  // fachlich nicht in einen LimitsContainer. Sie sitzt hier, weil dieser\n"
        "  // Container das einzige Objekt ist, das bereits von CommandPlanner ueber\n"
        "  // den PlanningContextLoader bis in den TrajectoryGenerator durchgereicht\n"
        "  // wird. Der fachlich saubere Weg -- ein zusaetzliches\n"
        "  // Konstruktor-Argument entlang derselben Kette -- wuerde die Signatur der\n"
        "  // exportierten pluginlib-Basisklasse PlanningContextLoader aendern und\n"
        "  // damit jeden extern gebauten Loader brechen. Der Kompromiss ist bewusst\n"
        "  // gewaehlt und hier dokumentiert, damit er nicht spaeter als Versehen\n"
        "  // gelesen wird.\n"
        "\n"
        "  /**\n"
        "   * @brief Setzt die Abtastzeit der Trajektorienerzeugung.\n"
        "   * @param sampling_time Abtastzeit in Sekunden.\n"
        "   * @note Keine Pruefung an dieser Stelle. Geprueft wird einmalig beim\n"
        "   *       Einlesen des Parameters in CommandPlanner::initialize(), damit\n"
        "   *       ein Fehler beim Start auffaellt und nicht erst bei der Planung.\n"
        "   */\n"
        "  void setSamplingTime(double sampling_time);\n"
        "\n"
        "  /**\n"
        "   * @brief Liefert die Abtastzeit der Trajektorienerzeugung in Sekunden.\n"
        "   * @return Der gesetzte Wert, sonst DEFAULT_SAMPLING_TIME.\n"
        "   */\n"
        "  double getSamplingTime() const;\n"
        "  // ----------------------------------------------------------------------\n"
        "\n"
        "private:\n",
    ),

    # -- 1c. Feld im privaten Teil ----------------------------------------
    (
        H_LIMITS,
        "  /// The cartesian limits\n"
        "  cartesian_limits::Params cartesian_limits_;\n",

        "  /// The cartesian limits\n"
        "  cartesian_limits::Params cartesian_limits_;\n"
        "\n"
        "  /// " + MARKE + ": Abtastzeit in Sekunden.\n"
        "  /// Kein has_-Flag wie bei den echten Grenzen: der Vorgabewert ist\n"
        "  /// gueltig und entspricht dem Verhalten vor dem Patch. Ein Flag wuerde\n"
        "  /// einen Zustand \"nicht gesetzt\" erzeugen, den es fachlich nicht gibt.\n"
        "  double sampling_time_;\n",
    ),

    # -- 2a. Konstruktor-Initialisierung ----------------------------------
    (
        C_LIMITS,
        "LimitsContainer::LimitsContainer() : has_joint_limits_(false), has_cartesian_limits_(false)\n"
        "{\n"
        "}\n",

        "// " + MARKE + ": sampling_time_ wird auf den Wert vorbelegt, der vor\n"
        "// dem Patch als Default-Argument von TrajectoryGenerator::generate() galt.\n"
        "LimitsContainer::LimitsContainer()\n"
        "  : has_joint_limits_(false), has_cartesian_limits_(false), sampling_time_(DEFAULT_SAMPLING_TIME)\n"
        "{\n"
        "}\n",
    ),

    # -- 2b. Implementierung Setter/Getter --------------------------------
    (
        C_LIMITS,
        "const cartesian_limits::Params& LimitsContainer::getCartesianLimits() const\n"
        "{\n"
        "  return cartesian_limits_;\n"
        "}\n",

        "const cartesian_limits::Params& LimitsContainer::getCartesianLimits() const\n"
        "{\n"
        "  return cartesian_limits_;\n"
        "}\n"
        "\n"
        "// " + MARKE + " -----------------------------------------------\n"
        "void LimitsContainer::setSamplingTime(double sampling_time)\n"
        "{\n"
        "  sampling_time_ = sampling_time;\n"
        "}\n"
        "\n"
        "double LimitsContainer::getSamplingTime() const\n"
        "{\n"
        "  return sampling_time_;\n"
        "}\n"
        "// ------------------------------------------------------------------------\n",
    ),

    # -- 3. solve() reicht die Abtastzeit durch ---------------------------
    (
        H_CONTEXT,
        "  generator_.generate(getPlanningScene(), request_, res);\n",

        "  // " + MARKE + ": bis zu diesem Patch stand hier ein Aufruf mit drei\n"
        "  // Argumenten. Das vierte Argument von generate() hat den Vorgabewert 0.1,\n"
        "  // und weil dieser Aufruf der einzige im Paket ist, war die Abtastzeit\n"
        "  // damit faktisch fest verdrahtet -- sichtbar nur, wenn man die\n"
        "  // Signatur im Header nachschlaegt. Genau das war schwer zu sehen: der\n"
        "  // Aufruf las sich vollstaendig, obwohl er es nicht war.\n"
        "  generator_.generate(getPlanningScene(), request_, res, limits_.getSamplingTime());\n",
    ),

    # -- 4a. Include fuer den Catch-Zweig ---------------------------------
    # InvalidParameterTypeException kaeme hier zwar ueber transitive Includes
    # herein, aber genau darauf soll sich nichts verlassen: ein aufgeraeumter
    # Include-Baum in einer spaeteren MoveIt-Version wuerde den Patch sonst
    # ohne erkennbaren Zusammenhang zerlegen.
    (
        C_PLANNER,
        "#include <rclcpp/logging.hpp>\n",

        "#include <rclcpp/logging.hpp>\n"
        "#include <rclcpp/exceptions/exceptions.hpp>  // " + MARKE + "\n",
    ),

    # -- 4b. Parameter einlesen und pruefen -------------------------------
    (
        C_PLANNER,
        "  // Load the planning context loader\n"
        "  planner_context_loader_ = std::make_unique<pluginlib::ClassLoader<PlanningContextLoader>>(\n",

        "  // " + MARKE + " ---------------------------------------------\n"
        "  // Abtastzeit aus dem Namensraum der Planungs-Pipeline lesen, also z. B.\n"
        "  // 'pilz_industrial_motion_planner.sampling_time'. Bewusst NICHT unter\n"
        "  // robot_description_planning.cartesian_limits: die Abtastzeit ist eine\n"
        "  // Eigenschaft des Planers, nicht des Roboters. Zwei Pipelines desselben\n"
        "  // Roboters duerfen unterschiedlich abtasten.\n"
        "  const std::string sampling_time_param = ns.empty() ? \"sampling_time\" : ns + \".sampling_time\";\n"
        "\n"
        "  if (!node->has_parameter(sampling_time_param))\n"
        "  {\n"
        "    node->declare_parameter<double>(sampling_time_param, DEFAULT_SAMPLING_TIME);\n"
        "  }\n"
        "\n"
        "  double sampling_time = DEFAULT_SAMPLING_TIME;\n"
        "  try\n"
        "  {\n"
        "    sampling_time = node->get_parameter(sampling_time_param).as_double();\n"
        "  }\n"
        "  catch (const rclcpp::exceptions::InvalidParameterTypeException& ex)\n"
        "  {\n"
        "    // Haeufige Stolperfalle: 'sampling_time: 1' ist in YAML eine Ganzzahl,\n"
        "    // 'sampling_time: 1.0' ein Gleitkommawert. Ohne diesen Zweig kaeme eine\n"
        "    // rclcpp-Ausnahme ohne Bezug zur Ursache.\n"
        "    RCLCPP_ERROR(getLogger(),\n"
        "                 \"PILZ-PATCH: Parameter '%s' hat den falschen Typ (%s). In der YAML mit \"\n"
        "                 \"Dezimalpunkt schreiben, also 0.008 statt 8e-3 oder 1 statt 1.0.\",\n"
        "                 sampling_time_param.c_str(), ex.what());\n"
        "    return false;\n"
        "  }\n"
        "\n"
        "  // Bewusst kein Zurechtbiegen: eine still korrigierte Abtastzeit wuerde eine\n"
        "  // ganze Messreihe entwerten, ohne dass es im Log auffaellt. Ein\n"
        "  // Startabbruch faellt dagegen sofort auf.\n"
        "  if (!(sampling_time >= MIN_SAMPLING_TIME && sampling_time <= MAX_SAMPLING_TIME))\n"
        "  {\n"
        "    RCLCPP_ERROR(getLogger(),\n"
        "                 \"PILZ-PATCH: Parameter '%s' = %.6f s liegt ausserhalb des zulaessigen \"\n"
        "                 \"Bereichs [%.6f, %.6f] s. Initialisierung abgebrochen.\",\n"
        "                 sampling_time_param.c_str(), sampling_time, MIN_SAMPLING_TIME, MAX_SAMPLING_TIME);\n"
        "    return false;\n"
        "  }\n"
        "\n"
        "  // Nachweis, dass dieser Build geladen ist und nicht das Binaerpaket aus\n"
        "  // /opt/ros. Bewusst WARN und nicht INFO: bei Standardkonfiguration von\n"
        "  // move_group ist INFO oft weggefiltert, und eine Zeile, die man nur\n"
        "  // manchmal sieht, taugt nicht als Nachweis.\n"
        "  RCLCPP_WARN(getLogger(),\n"
        "              \"PILZ-PATCH aktiv | sampling_time = %.6f s | Parameter '%s' | Vorgabe %.6f s | \"\n"
        "              \"Build %s %s\",\n"
        "              sampling_time, sampling_time_param.c_str(), DEFAULT_SAMPLING_TIME, __DATE__, __TIME__);\n"
        "  // ----------------------------------------------------------------------\n"
        "\n"
        "  // Load the planning context loader\n"
        "  planner_context_loader_ = std::make_unique<pluginlib::ClassLoader<PlanningContextLoader>>(\n",
    ),

    # -- 4c. Abtastzeit in den LimitsContainer legen ----------------------
    (
        C_PLANNER,
        "    limits.setJointLimits(aggregated_limit_active_joints_);\n"
        "    limits.setCartesianLimits(params_);\n",

        "    limits.setJointLimits(aggregated_limit_active_joints_);\n"
        "    limits.setCartesianLimits(params_);\n"
        "    // " + MARKE + ": pro Loader gesetzt, weil jeder Loader seinen\n"
        "    // eigenen LimitsContainer per Wert bekommt. PTP, LIN und CIRC teilen\n"
        "    // sich damit denselben Wert -- das ist gewollt, denn der Pilz-Blender\n"
        "    // verlangt uebereinstimmende Abtastzeiten der zu verschmelzenden Bahnen\n"
        "    // (siehe determineAndCheckSamplingTime in trajectory_functions.cpp).\n"
        "    // Getrennte Abtastzeiten pro Befehlsart wuerden jede Sequenz mit\n"
        "    // gemischten Befehlen unverschmelzbar machen.\n"
        "    limits.setSamplingTime(sampling_time);\n",
    ),
]


# ---------------------------------------------------------------------------
# Pruefungen
# ---------------------------------------------------------------------------

def fehler(text):
    """Bricht mit Klartext ab. Kein Traceback -- die Ursache steht in der Zeile."""
    print("\nABBRUCH: " + text + "\n", file=sys.stderr)
    sys.exit(1)


def pruefe_stand(wurzel):
    """
    Stellt sicher, dass wir im richtigen Paket und moeglichst in der richtigen
    Version stehen.

    Hintergrund: ein Patch, der versehentlich gegen einen anderen Stand
    angewendet wird, faellt erst beim Kompilieren auf -- oder schlimmer, gar
    nicht, weil ein Anker zufaellig passt.
    """
    pkg = os.path.join(wurzel, "package.xml")
    if not os.path.isfile(pkg):
        fehler("Keine package.xml unter '%s'. Erwartet wird das Paketverzeichnis "
               "von pilz_industrial_motion_planner, also das Verzeichnis mit "
               "package.xml, CMakeLists.txt, include/ und src/." % wurzel)

    with open(pkg, encoding="utf-8") as fh:
        inhalt = fh.read()

    treffer = re.search(r"<name>\s*([^<]+?)\s*</name>", inhalt)
    if not treffer or treffer.group(1) != "pilz_industrial_motion_planner":
        fehler("package.xml gehoert zu '%s', erwartet wird "
               "'pilz_industrial_motion_planner'."
               % (treffer.group(1) if treffer else "?"))

    treffer = re.search(r"<version>\s*([^<]+?)\s*</version>", inhalt)
    version = treffer.group(1) if treffer else "?"
    if version != ERWARTETE_VERSION:
        print("HINWEIS: Paketversion ist %s, geprueft wurde gegen %s."
              % (version, ERWARTETE_VERSION))
        print("         Das Skript faehrt fort, bricht aber ab, sobald ein Anker")
        print("         nicht mehr eindeutig passt. Ein Fehlschlag hier bedeutet")
        print("         nicht, dass der Patch unmoeglich ist -- nur, dass die")
        print("         Anker angepasst werden muessen.\n")
    return version


def lies(pfad):
    with open(pfad, encoding="utf-8") as fh:
        return fh.read()


def schreib(pfad, inhalt):
    with open(pfad, "w", encoding="utf-8") as fh:
        fh.write(inhalt)


def betroffene_dateien():
    return sorted({regel[0] for regel in REGELN})


# ---------------------------------------------------------------------------
# Befehle
# ---------------------------------------------------------------------------

def befehl_pruefen(wurzel):
    version = pruefe_stand(wurzel)
    print("Paket:   pilz_industrial_motion_planner %s" % version)
    print("Pfad:    %s\n" % os.path.abspath(wurzel))

    offen, fertig = [], []
    for rel in betroffene_dateien():
        pfad = os.path.join(wurzel, rel)
        if not os.path.isfile(pfad):
            fehler("Datei fehlt: %s" % rel)
        (fertig if MARKE in lies(pfad) else offen).append(rel)

    for rel in fertig:
        print("  gepatcht      %s" % rel)
    for rel in offen:
        print("  unveraendert  %s" % rel)

    print()
    if not offen:
        print("Zustand: vollstaendig gepatcht.")
    elif not fertig:
        print("Zustand: unveraendert (Originalstand).")
    else:
        fehler("Zustand: teilweise gepatcht. Das darf nicht vorkommen und deutet "
               "auf einen abgebrochenen Lauf oder eine Handaenderung hin. "
               "Mit --zurueck den Ausgangsstand herstellen und neu anwenden.")
    return 0


def befehl_anwenden(wurzel, zeige_diff):
    pruefe_stand(wurzel)

    # Schritt 1: alles einlesen und pruefen, nichts schreiben.
    # Ein halb angewendeter Patch ist schlimmer als gar keiner.
    inhalte = {}
    for rel in betroffene_dateien():
        pfad = os.path.join(wurzel, rel)
        if not os.path.isfile(pfad):
            fehler("Datei fehlt: %s" % rel)
        text = lies(pfad)
        if MARKE in text:
            fehler("'%s' enthaelt bereits die Marke '%s'. Der Patch wurde schon "
                   "angewendet. Mit --zurueck zuruecksetzen, dann erneut "
                   "anwenden." % (rel, MARKE))
        inhalte[rel] = text

    original = dict(inhalte)

    # Schritt 2: Anker pruefen und ersetzen -- weiterhin nur im Speicher.
    for nummer, (rel, anker, ersatz) in enumerate(REGELN, start=1):
        anzahl = inhalte[rel].count(anker)
        if anzahl != 1:
            fehler("Regel %d (%s): Anker kommt %d-mal vor, erwartet genau einmal.\n"
                   "Der Quellstand weicht ab. Anker (erste Zeile):\n    %s"
                   % (nummer, rel, anzahl, anker.splitlines()[0]))
        inhalte[rel] = inhalte[rel].replace(anker, ersatz, 1)

    # Schritt 3: Sicherung anlegen, dann schreiben.
    for rel in betroffene_dateien():
        pfad = os.path.join(wurzel, rel)
        sicherung = pfad + BACKUP_ENDUNG
        if os.path.exists(sicherung):
            fehler("Sicherung '%s' existiert bereits. Sie stammt aus einem "
                   "frueheren Lauf und wuerde ueberschrieben. Erst pruefen, "
                   "welcher Stand darin steht." % (rel + BACKUP_ENDUNG))
        schreib(sicherung, original[rel])
        schreib(pfad, inhalte[rel])
        print("  geaendert  %s   (Sicherung: %s)" % (rel, os.path.basename(sicherung)))

    if zeige_diff:
        print()
        for rel in betroffene_dateien():
            for zeile in difflib.unified_diff(
                    original[rel].splitlines(keepends=True),
                    inhalte[rel].splitlines(keepends=True),
                    fromfile="a/" + rel, tofile="b/" + rel):
                sys.stdout.write(zeile)

    print("\nPatch angewendet. Naechste Schritte:")
    print("    cd ~/ws_moveit")
    print("    colcon build --packages-select pilz_industrial_motion_planner \\")
    print("                 --cmake-args -DCMAKE_BUILD_TYPE=Release")
    print("    source /opt/ros/jazzy/setup.bash")
    print("    source ~/ws_moveit/install/setup.bash")
    print("    source ~/ros2_ws/install/setup.bash")
    print("    ros2 pkg prefix pilz_industrial_motion_planner   # muss ~/ws_moveit/install zeigen")
    print("\nBeim Start von move_group muss im Log stehen:  'PILZ-PATCH aktiv | sampling_time = ...'")
    return 0


def befehl_zurueck(wurzel):
    pruefe_stand(wurzel)
    zurueckgesetzt = 0
    for rel in betroffene_dateien():
        pfad = os.path.join(wurzel, rel)
        sicherung = pfad + BACKUP_ENDUNG
        if not os.path.exists(sicherung):
            print("  keine Sicherung  %s" % rel)
            continue
        schreib(pfad, lies(sicherung))
        os.remove(sicherung)
        print("  zurueckgesetzt   %s" % rel)
        zurueckgesetzt += 1

    if zurueckgesetzt == 0:
        fehler("Keine einzige Sicherung gefunden. Es gibt nichts zurueckzusetzen, "
               "oder die Sicherungen wurden entfernt. In dem Fall hilft nur "
               "'git checkout -- .' im Quellbaum.")
    print("\n%d Datei(en) zurueckgesetzt. Neu bauen nicht vergessen." % zurueckgesetzt)
    return 0


def main():
    parser = argparse.ArgumentParser(
        description="Macht sampling_time im Pilz-Planer zum ROS-Parameter.")
    parser.add_argument("paket",
                        help="Verzeichnis von pilz_industrial_motion_planner "
                             "(enthaelt package.xml)")
    parser.add_argument("--zurueck", action="store_true",
                        help="Aenderungen aus den Sicherungen zuruecknehmen")
    parser.add_argument("--pruefen", action="store_true",
                        help="Nur den Zustand melden, nichts aendern")
    parser.add_argument("--diff", action="store_true",
                        help="Nach dem Anwenden den vollstaendigen Diff ausgeben")
    args = parser.parse_args()

    if args.zurueck and args.pruefen:
        fehler("--zurueck und --pruefen schliessen sich aus.")

    wurzel = os.path.expanduser(args.paket)
    if not os.path.isdir(wurzel):
        fehler("'%s' ist kein Verzeichnis." % wurzel)

    if args.pruefen:
        return befehl_pruefen(wurzel)
    if args.zurueck:
        return befehl_zurueck(wurzel)
    return befehl_anwenden(wurzel, args.diff)


if __name__ == "__main__":
    sys.exit(main())
