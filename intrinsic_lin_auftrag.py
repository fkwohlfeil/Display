#!/usr/bin/env python3
"""Auftrag fuer intrinsic_lin_rechner.py aus einem TOTG-Auftrag der GUI.

Aufruf:
    intrinsic_lin_auftrag.py TOTG_AUFTRAG.json LIN_AUFTRAG.json \\
        --wrapper PFAD [--tip LINK] [--rundung-t M] [--rundung-r RAD]

Wozu

    Solange die Oberflaeche die LIN-Variante noch nicht selbst anbietet, ist
    das der kurze Weg zu einem echten Auftrag: die GUI schreibt fuer jede
    TOTG-Folge einen Auftrag nach /tmp/urdf_gui_totg_*/auftrag.json, und der
    Ordner bleibt bis zum naechsten Auftrag liegen. Er traegt schon
    Parameterdatei, Gruppe, Gelenke, Startzustand, Wegpunkte, Abschnitte und
    Overrides. Dazu kommen hier:

        tip       aus der SRDF der Parameterdatei (Kette der Gruppe), wenn
                  nicht angegeben
        grenzen   Positionen aus der URDF, v und a aus
                  robot_description_planning/joint_limits -- dieselben
                  Werte, mit denen move_group und TOTG rechnen
        rundung   kartesische Eckenverrundung; Standard wie im Motion
                  Planner von Intrinsic (1 mm, 0,01 rad)

    Selbst geschrieben. Kein Tk, kein ROS.

Was nicht still passiert

    Eine Achse ohne Beschleunigungsgrenze ist ein Fehler (TOPP-RA braucht
    sie, wie TOTG). Endlos drehende Achsen (continuous) haben keine
    Positionsgrenze; JSON kann "unendlich" nicht darstellen, deshalb stehen
    sie mit +-1e9 im Auftrag und werden in "grenzen_hinweise" genannt.
"""

import argparse
import json
import sys
import xml.etree.ElementTree as ET

STAND = 1
ENDLOS = 1.0e9


def parameter_lesen(pfad):
    import yaml
    with open(pfad, encoding="utf-8") as datei:
        daten = yaml.safe_load(datei)
    for knoten in (daten or {}).values():
        if isinstance(knoten, dict) and isinstance(
                knoten.get("ros__parameters"), dict):
            return knoten["ros__parameters"]
    raise ValueError(f"{pfad}: kein ros__parameters-Block")


def tip_aus_srdf(srdf, gruppe):
    wurzel = ET.fromstring(srdf)
    for g in wurzel.findall("group"):
        if g.get("name") != gruppe:
            continue
        ketten = g.findall("chain")
        if len(ketten) != 1:
            raise ValueError(f"Gruppe {gruppe} hat {len(ketten)} Ketten, "
                             f"erwartet eine -- --tip angeben")
        return ketten[0].get("tip_link")
    raise ValueError(f"Gruppe {gruppe} steht nicht in der SRDF")


def grenzen_lesen(params, gelenke):
    urdf = ET.fromstring(params["robot_description"])
    planung = (params.get("robot_description_planning") or {}).get(
        "joint_limits") or {}
    gelenk_xml = {j.get("name"): j for j in urdf.findall("joint")}
    grenzen = {"min_position": [], "max_position": [], "max_velocity": [],
               "max_acceleration": []}
    hinweise = []
    for name in gelenke:
        j = gelenk_xml.get(name)
        if j is None:
            raise ValueError(f"Gelenk {name} steht nicht in der URDF")
        art = j.get("type")
        limit = j.find("limit")
        if art == "continuous":
            unten, oben = -ENDLOS, ENDLOS
            hinweise.append(f"{name}: continuous, Positionsgrenze +-{ENDLOS:g}")
        else:
            if limit is None or limit.get("lower") is None \
                    or limit.get("upper") is None:
                raise ValueError(f"Gelenk {name} ({art}) ohne lower/upper")
            unten, oben = float(limit.get("lower")), float(limit.get("upper"))
        eintrag = planung.get(name) or {}
        if eintrag.get("has_velocity_limits") and "max_velocity" in eintrag:
            v = float(eintrag["max_velocity"])
        elif limit is not None and limit.get("velocity") is not None:
            v = float(limit.get("velocity"))
            hinweise.append(f"{name}: max_velocity aus der URDF")
        else:
            raise ValueError(f"Gelenk {name} ohne Geschwindigkeitsgrenze")
        if eintrag.get("has_acceleration_limits") \
                and "max_acceleration" in eintrag:
            a = float(eintrag["max_acceleration"])
        else:
            raise ValueError(f"Gelenk {name} ohne Beschleunigungsgrenze in "
                             f"robot_description_planning -- TOPP-RA braucht "
                             f"sie, wie TOTG")
        grenzen["min_position"].append(unten)
        grenzen["max_position"].append(oben)
        grenzen["max_velocity"].append(v)
        grenzen["max_acceleration"].append(a)
    return grenzen, hinweise


def lin_auftrag(totg, wrapper, tip=None, rundung_t=0.001, rundung_r=0.01):
    params = parameter_lesen(totg["params_datei"])
    gelenke = list(totg["gelenke"])
    if tip is None:
        tip = tip_aus_srdf(params["robot_description_semantic"],
                           totg["gruppe"])
    grenzen, hinweise = grenzen_lesen(params, gelenke)
    return {
        "stand": STAND,
        "wrapper": wrapper,
        "params_datei": totg["params_datei"],
        "gruppe": totg["gruppe"],
        "tip": tip,
        "gelenke": gelenke,
        "start": totg["start"],
        "namen": totg.get("namen"),
        "wegpunkte": totg["wegpunkte"],
        "abschnitte": totg.get("abschnitte"),
        "velocity_scaling": totg["velocity_scaling"],
        "acceleration_scaling": totg["acceleration_scaling"],
        "resample_dt": totg["resample_dt"],
        "rundung_translation_m": float(rundung_t),
        "rundung_rotation_rad": float(rundung_r),
        "grenzen": grenzen,
        "grenzen_hinweise": hinweise,
        "quelle": "intrinsic_lin_auftrag.py aus TOTG-Auftrag",
    }


def main(argv):
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("totg_auftrag")
    p.add_argument("lin_auftrag")
    p.add_argument("--wrapper", required=True)
    p.add_argument("--tip")
    p.add_argument("--rundung-t", type=float, default=0.001)
    p.add_argument("--rundung-r", type=float, default=0.01)
    a = p.parse_args(argv[1:])
    with open(a.totg_auftrag, encoding="utf-8") as datei:
        totg = json.load(datei)
    auftrag = lin_auftrag(totg, a.wrapper, a.tip, a.rundung_t, a.rundung_r)
    with open(a.lin_auftrag, "w", encoding="utf-8") as datei:
        json.dump(auftrag, datei, indent=1)
        datei.write("\n")
    print(f"{a.lin_auftrag}: {len(auftrag['wegpunkte'])} Wegpunkte, "
          f"{len(auftrag['abschnitte'] or [1])} Abschnitt(e), tip "
          f"{auftrag['tip']}")
    for h in auftrag["grenzen_hinweise"]:
        print("Hinweis:", h)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
