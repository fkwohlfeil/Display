# -*- coding: utf-8 -*-
"""sonden_mathe_jacobi.py -- Mutationssonden fuer mathe.py und jacobi.py.

Getrennte Suite, erst nach gruener Testsuite auszufuehren.  Jede Sonde
verfaelscht gezielt eine Stelle im Quelltext und erwartet, dass die
Testsuite daraufhin FEHLSCHLAEGT.  Eine Sonde, die durchrutscht, bedeutet:
die betreffende Sicherung ist nicht getestet.

Der Lauf findet in einem Wegwerfverzeichnis statt; die Originaldateien
werden nicht angefasst.  Aufruf jeweils mit -B und geleertem __pycache__,
sonst liefert veralteter Bytecode falsche Ergebnisse.

Aufruf:   python3 -B sonden_mathe_jacobi.py
"""

import os
import shutil
import subprocess
import sys
import tempfile

HIER = os.path.dirname(os.path.abspath(__file__))
DATEIEN = ["mathe.py", "jacobi.py", "test_mathe_jacobi.py"]

# (Name, Datei, Suchtext, Ersatztext)
SONDEN = [
    ("Kreuzprodukt: erste Komponente im Vorzeichen gedreht",
     "mathe.py",
     "return [a[1] * b[2] - a[2] * b[1],",
     "return [a[2] * b[1] - a[1] * b[2],"),

    ("Jacobi: Bezugspunkt und Gelenkursprung vertauscht",
     "jacobi.py",
     "            r = sub3(p_bezug, p)",
     "            r = sub3(p, p_bezug)"),

    ("Jacobi: letzte Spalte faellt weg (off-by-one bei sieben Achsen)",
     "jacobi.py",
     "    J = [[0.0] * n for _ in range(6)]\n    for i in range(n):\n        typ, z, p = gelenke[i]",
     "    J = [[0.0] * n for _ in range(6)]\n    for i in range(n - 1):\n        typ, z, p = gelenke[i]"),

    ("Jacobi: Rotationsanteil des Drehgelenks fehlt",
     "jacobi.py",
     "            J[3][i] = z[0]\n            J[4][i] = z[1]\n            J[5][i] = z[2]",
     "            J[3][i] = 0.0\n            J[4][i] = 0.0\n            J[5][i] = 0.0"),

    ("twist_diff: Drehung im Werkzeugframe statt im Basisframe",
     "mathe.py",
     "    dR = mat_mal_mat(R_b, transponiert(R_a))",
     "    dR = mat_mal_mat(transponiert(R_a), R_b)"),

    ("log_so3: Umschaltung auf den kuerzeren Drehweg entfaellt",
     "mathe.py",
     "    if w < 0.0:\n        w, x, y, z = -w, -x, -y, -z\n    nv = math.sqrt(x * x + y * y + z * z)\n    if nv < KLEIN:\n        # Kleinwinkelfall",
     "    nv = math.sqrt(x * x + y * y + z * z)\n    if nv < KLEIN:\n        # Kleinwinkelfall"),

    ("exp_so3: Reihenentwicklung fuer kleine Winkel entfaellt",
     "mathe.py",
     "    if t < KLEIN:\n        t2 = t * t\n        a = 1.0 - t2 / 6.0\n        b = 0.5 - t2 / 24.0\n    else:",
     "    if False:\n        t2 = t * t\n        a = 1.0 - t2 / 6.0\n        b = 0.5 - t2 / 24.0\n    else:"),

    ("quat_von_matrix: nur der Spur-Zweig, keine Fallunterscheidung",
     "mathe.py",
     "    if sp > 0.0:\n        s = math.sqrt(sp + 1.0) * 2.0",
     "    if True:\n        s = math.sqrt(abs(sp) + 1.0) * 2.0"),

    ("det_lu: Vorzeichen der Zeilenvertauschungen wird nicht gefuehrt",
     "jacobi.py",
     "            vorzeichen = -vorzeichen",
     "            vorzeichen = vorzeichen"),

    ("produkt_jjt: obere Dreiecksmatrix wird nicht gespiegelt",
     "jacobi.py",
     "            M[i][j] = s\n            M[j][i] = s",
     "            M[i][j] = s"),

    ("manipulierbarkeit: Wurzel vergessen",
     "jacobi.py",
     "    if d <= 0.0:\n        return 0.0                   # kleine negative Werte sind Rundung\n    return math.sqrt(d)",
     "    if d <= 0.0:\n        return 0.0                   # kleine negative Werte sind Rundung\n    return d"),

    ("manipulierbarkeit: Zeilenpruefung entfaellt",
     "jacobi.py",
     "    if m > n:\n        raise ValueError(\"manipulierbarkeit:",
     "    if False:\n        raise ValueError(\"manipulierbarkeit:"),

    ("cholesky: Pruefung auf positive Definitheit entfaellt",
     "jacobi.py",
     "                if rest <= 0.0:\n                    raise ValueError(\"cholesky:",
     "                if False:\n                    raise ValueError(\"cholesky:"),

    ("cholesky: Abzug der bisherigen Summe fehlt",
     "jacobi.py",
     "                L[i][j] = (M[i][j] - s) / L[j][j]",
     "                L[i][j] = M[i][j] / L[j][j]"),

    ("loese_spd: Rueckwaertseinsetzen nutzt L statt L^T",
     "jacobi.py",
     "            s -= L[k][i] * y[k]",
     "            s -= L[i][k] * y[k]"),

    ("eigen_symmetrisch: aufsteigend statt absteigend sortiert",
     "jacobi.py",
     "    paare.sort(key=lambda pi: (-pi[0], pi[1]))",
     "    paare.sort(key=lambda pi: (pi[0], pi[1]))"),

    ("eigen_symmetrisch: Eigenvektoren werden nicht mitgedreht",
     "jacobi.py",
     "                for k in range(n):\n                    vkp = V[k][p]\n                    vkq = V[k][q]\n                    V[k][p] = c * vkp - s * vkq\n                    V[k][q] = s * vkp + c * vkq",
     "                for k in range(n):\n                    pass"),

    ("dls_schritt: Daempfung nicht quadriert",
     "jacobi.py",
     "    M = produkt_jjt(A)\n    l2 = lam * lam",
     "    M = produkt_jjt(A)\n    l2 = lam"),

    ("dls_schritt: kartesische Gewichtung wirkt nicht auf die rechte Seite",
     "jacobi.py",
     "    b = [wx[i] * dx[i] for i in range(m)]",
     "    b = [dx[i] for i in range(m)]"),

    ("dls_schritt: Gelenkgewichtung am Ende vergessen",
     "jacobi.py",
     "        dq[j] = wq[j] * s",
     "        dq[j] = s"),

    ("jacobi_numerisch: einseitige statt zentraler Differenz",
     "test_mathe_jacobi.py",
     None,   # Sonde greift in jacobi.py, siehe unten
     None),
]

# Die letzte Sonde ausformulieren (sie betrifft jacobi.py, nicht den Test).
SONDEN[-1] = ("jacobi_numerisch: einseitige statt zentraler Differenz",
              "jacobi.py",
              "        qm[i] -= h",
              "        qm[i] -= 0.0")


def lauf(verzeichnis):
    """Testsuite im gegebenen Verzeichnis ausfuehren, Rueckgabecode liefern."""
    cache = os.path.join(verzeichnis, "__pycache__")
    if os.path.isdir(cache):
        shutil.rmtree(cache)
    p = subprocess.run([sys.executable, "-B", "test_mathe_jacobi.py"],
                       cwd=verzeichnis, capture_output=True, text=True, timeout=300)
    return p.returncode, p.stdout + p.stderr


def main():
    # Grundlauf: unveraendert muss gruen sein, sonst sind die Sonden wertlos
    with tempfile.TemporaryDirectory() as tmp:
        for d in DATEIEN:
            shutil.copy(os.path.join(HIER, d), tmp)
        rc, ausgabe = lauf(tmp)
        if rc != 0:
            print("ABBRUCH: der unveraenderte Grundlauf ist nicht gruen.")
            print(ausgabe)
            return 2
    print("Grundlauf gruen. Starte %d Sonden.\n" % len(SONDEN))

    durchgerutscht = []
    fehlerhaft = []
    for name, datei, alt, neu in SONDEN:
        with tempfile.TemporaryDirectory() as tmp:
            for d in DATEIEN:
                shutil.copy(os.path.join(HIER, d), tmp)
            pfad = os.path.join(tmp, datei)
            with open(pfad, "r", encoding="utf-8") as f:
                quelle = f.read()
            anzahl = quelle.count(alt)
            if anzahl != 1:
                fehlerhaft.append((name, "Suchtext kommt %d mal vor, erwartet genau 1" % anzahl))
                print("  DEFEKT  %s" % name)
                continue
            with open(pfad, "w", encoding="utf-8") as f:
                f.write(quelle.replace(alt, neu))
            rc, _ = lauf(tmp)
            if rc == 0:
                durchgerutscht.append(name)
                print("  OFFEN   %s" % name)
            else:
                print("  gefangen  %s" % name)

    print()
    if fehlerhaft:
        print("Sonden mit defektem Suchtext: %d" % len(fehlerhaft))
        for name, grund in fehlerhaft:
            print("  - %s (%s)" % (name, grund))
    if durchgerutscht:
        print("NICHT GEFANGEN: %d Sonden" % len(durchgerutscht))
        for name in durchgerutscht:
            print("  - " + name)
        print("\nJede dieser Zeilen bezeichnet eine Sicherung ohne Test.")
        return 1
    if fehlerhaft:
        return 1
    print("Alle %d Sonden wurden von der Testsuite gefangen." % len(SONDEN))
    return 0


if __name__ == "__main__":
    sys.exit(main())
