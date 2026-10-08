# Abkürzungen und Fachbegriffe {#begriffe .teil .unnumbered}

::: teilkopf
Gilt für beide Teile. Fachbegriffe aus Robotik, ROS 2, MoveIt und
Softwaretechnik stehen im Text so, wie sie dort heißen, also meist englisch
(Node, Service, Callback, Watchdog, Inverse Kinematics …). Wo der Code oder die
Oberfläche einen deutschen Namen verwendet, steht er in der mittleren Spalte;
in der Anleitung werden Beschriftungen der Oberfläche immer wörtlich zitiert.
:::

## Abkürzungen {#abkuerzungen .unnumbered}

| Abkürzung | ausgeschrieben | Bedeutung hier |
|---|---|---|
| A, B, C | – | Winkel der TCP-Orientierung in Grad: erste, zweite, dritte Drehung der gewählten Drehkonvention |
| ACM | Allowed Collision Matrix | Linkpaare, die nicht auf Kollision geprüft werden (`disable_collisions` in der SRDF) |
| CIRC | Circular Motion | Kreisbewegung des Pilz-Planners; braucht einen Hilfspunkt |
| CSV | Comma-Separated Values | Textformat der Bahnen (`TCP_Fahrten/`), hier mit `;` getrennt |
| ESU | Extended Security Updates | verlängerte Sicherheitsupdates für Windows 10 nach Ende des regulären Supports |
| FK | Forward Kinematics | Lage aller Links aus den Achswerten |
| GUI | Graphical User Interface | die Oberfläche `urdf_gui.py` |
| IK | Inverse Kinematics | Achswerte zu einer vorgegebenen TCP-Lage; kommt immer aus MoveIt |
| IPO | Interpolation | Interpolationstakt der GUI (`IPO_TICK_MS` = 20 ms) |
| JSON | JavaScript Object Notation | Format von Auftrag und Antwort der Kindprozesse und des Wrappers |
| KDL | Kinematics and Dynamics Library (Orocos) | Standard-IK-Solver von MoveIt |
| LIN | Linear Motion | TCP fährt auf einer Geraden im kartesischen Raum |
| LMA | Levenberg-Marquardt-Algorithmus | alternativer IK-Solver von MoveIt |
| PID | Process Identifier | Prozessnummer, Teil des Node-Namens `move_group_gui<PID>_<n>` |
| PTP | Point-to-Point | alle Achsen fahren synchron ins Ziel; die TCP-Bahn ist nicht vorgegeben |
| QoS | Quality of Service | Zustellregeln eines ROS-Topics (z. B. `transient local` für `robot_description`) |
| rclcpp, rclpy | ROS Client Library für C++ bzw. Python | Programmierschnittstelle zu ROS 2 |
| ROS 2 | Robot Operating System 2 | Middleware; verwendet wird die Distribution Jazzy |
| rpy | Roll, Pitch, Yaw | Winkel in der URDF (`<origin rpy="…">`), immer ZYX |
| RViz2 | ROS Visualization | 3D-Anzeige des Roboters |
| SRDF | Semantic Robot Description Format | Planning Group, Group States, ACM; wird von der GUI erzeugt |
| TCP | Tool Center Point | Werkzeugpunkt; hier der Tip-Link bzw. der gewählte Werkzeug-Frame |
| TF | Transform (Paket `tf2`) | Lagen aller Frames zueinander, Topic `/tf` |
| TOPP-RA | Time-Optimal Path Parameterization based on Reachability Analysis | Time Parameterization aus Intrinsic Core |
| TOTG | Time-Optimal Trajectory Generation | Time Parameterization aus MoveIt |
| TRAC-IK | – | alternativer IK-Solver (eigenes Plugin) |
| URDF | Unified Robot Description Format | XML-Modell des Roboters: Links, Joints, Grenzen, Aussehen |
| WSL, WSLg | Windows Subsystem for Linux (mit GUI-Unterstützung) | Ubuntu unter Windows; WSLg zeigt die Linux-Fenster an |
| YAML | YAML Ain't Markup Language | Format der Parameterdatei von `move_group` |
| ZYX, ZYZ, ZXZ | – | Drehkonventionen für A, B, C (intrinsisch), siehe Teil I, Abschnitt 4.2 |

## Fachbegriffe {#fachbegriffe .unnumbered}

| Begriff | im Code / in der Oberfläche | Bedeutung |
|---|---|---|
| Achsziel | `achsziel`, Zeile `J …` | Ziel in Achswerten statt als Lage; fährt immer PTP mit Radius 0 |
| Auftrag, Antwort | `auftrag.json`, `antwort.json`, `request_*` | Daten an `RosBridge` oder einen Kindprozess und deren Ergebnis |
| Blending | Überschleifen, `blend_radius`, `radius=` | Übergang in den nächsten Punkt ohne Halt; Radius 0 heißt Genauhalt |
| Callback | – | Funktion, die bei einem Ereignis aufgerufen wird (Timer, Umschalten der Drehkonvention) |
| Capability | `capabilities` | Erweiterung von `move_group`, hier `MoveGroupSequenceService` für Punktfolgen |
| Cartesian Limits | Reiter „Kartesisch“, `cartesian_limits` | Bahngrenzen für LIN und CIRC bei Pilz |
| Endeffektor | „Mehrere Endeffektoren …“ | Ende der kinematischen Kette, an dem das Werkzeug sitzt |
| Exception | – | Ausnahmefehler in Python; die Takte fangen sie ab und melden sie |
| Executor | `SingleThreadedExecutor` | arbeitet Timer und Callbacks eines ROS-Nodes ab; hier im eigenen Thread |
| Frame | – | Koordinatensystem; jeder Link hat eines |
| Genauhalt | Radius 0 | der Roboter hält im Punkt an |
| Glied | „Glied anhaengen“, `append_limb` | Joint mit dem dahinterliegenden Link |
| Group State | `group_state` „null“ | benannte Achsstellung in der SRDF |
| Halte-Taster | „FAHREN (HALTEN)“, `_hold_*` | plant und fährt, solange er gedrückt ist; Loslassen bremst |
| Hilfspunkt | `hilfspunkt` | Zwischenpunkt, durch den eine CIRC-Bewegung läuft |
| IK-Ast | `ast_folgen` | eine von mehreren Achslösungen zur selben Lage (Roboterkonfiguration) |
| Jerk | Ruck, `jerk` | dritte Ableitung der Lage nach der Zeit |
| Joggen | `_jog_*` | Verfahren in kleinen Schritten von Hand |
| Joint | Gelenk | verbindet zwei Links: `revolute`, `continuous`, `prismatic`, `fixed` |
| Joint Limits | Reiter „Achsgrenzen“, `joint_limits` | Grenzen für Geschwindigkeit, Beschleunigung, Verzögerung und Jerk je Achse |
| Joint Space | Achsraum, Gelenkraum | Raum der Achswerte, im Gegensatz zum kartesischen Raum |
| Kindprozess | `TotgProzess`, `MoveItProcess` | eigener Prozess, den die GUI startet und beendet |
| Kopplung | `<mimic>`, „KOPPLUNG“ | Achse folgt einer anderen: Faktor · Führung + Versatz |
| Lage | „TCP-LAGE“, `lage` | Pose: Position (X, Y, Z) und Orientierung (A, B, C) |
| Leaf-Link | Blatt, `leaf_links` | Link ohne Nachfolger; Kandidat für den TCP |
| Link | – | starrer Körper des Modells |
| Lock | `threading.Lock` | Sperre für Daten, die zwei Threads teilen |
| Mock, Stub | – | Attrappe im Test, die ein echtes Modul oder Programm ersetzt |
| Modalzustand | `modal`, `MODAL_START` | in der Punktliste gesetzte Werte gelten für alle folgenden Zeilen |
| `move_group` | – | Node von MoveIt für Planung und IK; hier ein Kindprozess der GUI |
| MoveItPy | `moveit_py` | Python-Schnittstelle zu MoveIt mit eigenem rclcpp-Node |
| Node | – | Teilnehmer im ROS-Graphen, z. B. `urdf_gui_tf` |
| Nutzlast | `Folge.nutzlast` | hier: Payload, also die Daten einer Anfrage – nicht die Traglast des Roboters |
| Override | „Override Geschwindigkeit“, `velocity_scaling` | Prozentfaktor auf Geschwindigkeit bzw. Beschleunigung |
| Pilz | `pilz_industrial_motion_planner` | Planner von MoveIt für PTP, LIN, CIRC und Punktfolgen |
| Planner, Planning Pipeline | `planning_pipelines` | Planungsverfahren von MoveIt und die Kette aus Planner und Adaptern |
| Planning Group | Planungsgruppe, `manipulator` | Satz von Joints, für den geplant wird; hier eine Kette von der Wurzel zum Tip |
| Planning Scene Monitor | – | hält in MoveIt das aktuelle Modell und die Umgebung |
| Punktfolge, Punktliste | `sequenz`, Fenster „Punktfolge“ | Folge von Zielen, die als eine Bahn geplant wird |
| Request/Response Adapter | `request_adapters`, `response_adapters` | Prüf- und Nachbearbeitungsschritte um den Planner |
| Resampling | Neuabtastung, `lin_neuabtastung`, `resample_dt` | neue, gleichmäßige Abtastung einer Bahn |
| Root-Link | Wurzel, `root_links` | erster Link des Modells; Bezug aller Lagen |
| Ruckig | „Ruckig-Glaettung“ | Bibliothek, die eine Bahn auf begrenzten Jerk glättet |
| Service | Dienst | Anfrage/Antwort im ROS-Graphen, z. B. `/compute_ik` |
| Session | – | Prozessgruppe eines Kindprozesses; wird als Ganzes beendet |
| Singularität | – | Achsstellung, in der der Roboter einen Freiheitsgrad verliert; davon zu trennen ist die Winkelsingularität der Darstellung A, B, C |
| Slider | Schieber, `_on_slider` | Schieberegler je Achse im Hauptfenster |
| Stand | `STAND`, `MODUL_STAENDE` | Versionsnummer einer Datei bzw. einer Schnittstelle |
| State Machine | Ablaufsteuerung, `STATE_*`, `_set_state` | Zustände `BEREIT`, `IK`, `PLANEN`, `FOLGE IK`, `FOLGE`, `FAHREN`, `BREMSEN`, `FEHLER` |
| Stützpunkt | `points`, `punkte` | Punkt einer Trajektorie: Zeit, Achswerte, Geschwindigkeiten |
| Tag | Auftragsnummer, `tag` | Nummer, an der die GUI die Antwort zu ihrem Auftrag erkennt |
| Teilbahn, Abschnitt | `teilbahnen`, `abschnitte` | Stück einer Folge zwischen zwei Halten |
| Thread | – | Ausführungsstrang innerhalb eines Prozesses |
| Time Parameterization | Zeitparametrierung | ordnet einem geometrischen Pfad einen Zeitverlauf zu (TOTG, TOPP-RA) |
| Timeout | Frist, `timeout`, `FRIST_S` | Zeit, nach der ein Auftrag als gescheitert gilt |
| Timer | – | ruft einen Callback in festem Takt auf |
| Tip-Link | `tip` | letzter Link der Planning Group; an ihm rechnen IK und LIN |
| Topic | – | Datenkanal im ROS-Graphen, z. B. `/joint_states` |
| Trajektorie, Bahn | `TrajectoryPlayer` | Weg mit Zeitverlauf |
| Umkehrpunkt | `umkehr` | Punkt, an dem eine Achse die Richtung wechselt; dort wird angehalten |
| Verrundung | `rundung_translation_m`, `rundung_rot` | kartesische Eckenrundung bei Intrinsic |
| Wall-Clock-Zeit | Wanduhr, `wall_drift` | tatsächlich vergangene Zeit, im Gegensatz zur Bahnzeit aus den Takten |
| Watchdog | `_hold_watch`, `_moveit_watch`, `_tf_watchdog` | prüft zyklisch einen Zustand und greift ein, wenn er nicht stimmt |
| Wrapper | `intrinsic_toppra_wrapper` | eigenes C++-Programm, das Funktionen von Intrinsic Core über JSON aufrufbar macht |
| Zielraum | „Achsraum“, „Kartesisch (IK)“ | Art, in der im Fahrfenster das Ziel vorgegeben wird |
