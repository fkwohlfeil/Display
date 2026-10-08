#!/usr/bin/env python3
"""Prueft die Zerlegung urdf_gui3_7.py -> urdf_gui2_1.py + Module.

Braucht urdf_gui3_7.py im selben Ordner. Kein Fenster, kein ROS.

    PYTHONDONTWRITEBYTECODE=1 python3 pruefe_zerlegung_2_1.py

Fuer jede Funktion und Klasse des Originals:
  - Der Quelltext im neuen Modul ist zeichengleich.
  - Jeder globale Name, den sie liest, ist dort gebunden und zeigt auf
    dasselbe Objekt bzw. denselben Wert wie im Original.
Dazu: jede Konstante des Originals existiert mit gleichem Wert.

Sonden, die im Container erkannt wurden: fehlender Import in ros_bruecke,
geaenderter TREE_GAP_X in stil, geaenderter IPO_DT in bahn.
"""
import inspect, types, importlib, sys
alt = importlib.import_module("urdf_gui3_7")
gui = importlib.import_module("gui")
mods = [importlib.import_module(m) for m in (
    "einstellungen stil matrix anzeige urdf_modell kinematik moveit_konfig "
    "bahn totg_prozess ros_bruecke").split()] + [gui]
wo = {}
for m in mods:
    for n, v in vars(m).items():
        if getattr(v, "__module__", None) == m.__name__ and isinstance(v, (types.FunctionType, type)):
            wo[n] = m

def codes(obj):
    if isinstance(obj, type):
        for v in vars(obj).values():
            f = v.fget if isinstance(v, property) else getattr(v, "__func__", v)
            if isinstance(f, types.FunctionType):
                yield from codes_of(f.__code__)
    else:
        yield from codes_of(obj.__code__)

def codes_of(c):
    yield c
    for k in c.co_consts:
        if isinstance(k, types.CodeType):
            yield from codes_of(k)

def gleich(a, b, name):
    if isinstance(a, types.ModuleType):
        return a is b
    if isinstance(a, (types.FunctionType, type)) and getattr(a, "__module__", "") != "urdf_gui3_7":
        return a is b
    if isinstance(a, (types.FunctionType, type)):
        return a.__qualname__ == b.__qualname__ and inspect.getsource(a) == inspect.getsource(b)
    if name == "ANZEIGE":
        return type(a).__name__ == type(b).__name__
    return type(a) is type(b) and a == b

fehler = 0; geprueft = 0; namen = 0
for n, v in vars(alt).items():
    if not (isinstance(v, (types.FunctionType, type)) and v.__module__ == "urdf_gui3_7"):
        continue
    m = wo.get(n)
    if m is None:
        print("FEHLT", n); fehler += 1; continue
    neu = getattr(m, n)
    if inspect.getsource(v) != inspect.getsource(neu):
        print("QUELLE ANDERS", n); fehler += 1
    geprueft += 1
    for c in codes(v):
        for g in c.co_names:
            if g in vars(alt) and g not in ("__name__", "__doc__"):
                namen += 1
                if g not in vars(m):
                    print(f"{n}: {g} in {m.__name__} nicht gebunden"); fehler += 1
                elif not gleich(vars(alt)[g], vars(m)[g], g):
                    print(f"{n}: {g} zeigt woanders hin"); fehler += 1
# Konstanten: jeder Wert des Originals existiert gleich im neuen Aufbau
konst = 0
for n, v in vars(alt).items():
    if n.isupper() and not n.startswith("_"):
        treffer = [m for m in mods if n in vars(m) and gleich(v, vars(m)[n], n)]
        konst += 1
        if not treffer:
            print("KONSTANTE", n); fehler += 1
print(f"{geprueft} Funktionen/Klassen, {namen} globale Namensbezuege, {konst} Konstanten, Fehler: {fehler}")
sys.exit(1 if fehler else 0)
