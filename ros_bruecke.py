"""ros_bruecke -- RosBridge. rclpy und die Nachrichtentypen werden erst in
start() bzw.
in den Methoden importiert; das Modul laedt deshalb auch ohne
gesourctes Environment.

Seit urdf_gui2_1 ein eigenes Modul. Der Inhalt ist zeilengleich aus
urdf_gui3_7.py verschoben (dort Zeilen 4112-4975); neu sind
nur dieser Kopf, die Importe und STAND.
"""

import collections
import threading
import traceback
import time

from einstellungen import (TF_RATE_HZ)
from matrix import (to_quaternion)
from kinematik import (local_transform)
from moveit_konfig import (PLAN_TIMEOUT_MARGIN_S)
from bahn import (IK_TIMEOUT_MARGIN_S, JOB_RATE_HZ, SPIN_ERROR_LIMIT)

# Muss zu MODUL_STAENDE in urdf_gui2_1.py passen.
STAND = 1


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
