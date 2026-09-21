#!/bin/bash
# Mutationsproben. -B und ein geloeschter Cache sind Pflicht: die Proben
# schreiben die Datei im Sekundentakt neu, und ein .pyc aus derselben Sekunde
# wird sonst weiterverwendet. Genau das hat beim ersten Lauf drei Proben
# verfaelscht.
probe () {
  name="$1"; alt="$2"; neu="$3"
  cp konvention.py konvention_sicher.py
  python3 -B - "$alt" "$neu" <<'PY'
import sys
alt, neu = sys.argv[1], sys.argv[2]
text = open("konvention.py").read()
assert text.count(alt) == 1, "Muster kommt %d mal vor" % text.count(alt)
open("konvention.py", "w").write(text.replace(alt, neu))
PY
  if [ $? -ne 0 ]; then echo "PROBE DEFEKT   $name"; mv konvention_sicher.py konvention.py; return; fi
  rm -rf __pycache__
  if python3 -B test_konvention.py > /tmp/aus.txt 2>&1; then
    echo "NICHT ERKANNT  $name"
  else
    echo "erkannt        $name  ($(grep -c GEFALLEN /tmp/aus.txt) Meldungen)"
  fi
  mv konvention_sicher.py konvention.py
  rm -rf __pycache__
}

probe "ZYZ Vorzeichen in r01"        "-ca * cb * sc - sa * cc"  "-ca * cb * sc + sa * cc"
probe "ZXZ Vorzeichen in r12"        "-ca * sb]"                "ca * sb]"
probe "ZYX A und C vertauscht"       "        return xyz, (a, b, c)

    if konvention == \"ZYZ\"" "        return xyz, (c, b, a)

    if konvention == \"ZYZ\""
probe "ZXZ fehlendes Minus in A"     "a = math.atan2(matrix[0][2], -matrix[1][2])"  "a = math.atan2(matrix[0][2], matrix[1][2])"
probe "ZYZ Sonderfall B=pi"          "a = math.atan2(-matrix[1][0], -matrix[0][0])" "a = math.atan2(matrix[1][0], matrix[0][0])"
probe "ZYZ acos statt atan2"         "        b = math.atan2(laenge, matrix[2][2])" "        b = math.acos(max(-1.0, min(1.0, matrix[2][2])))"
probe "B-Bereich falsch angegeben"   "\"ZYZ\": (0.0, 180.0),
    \"ZXZ\": (0.0, 180.0),
}

# Werte" "\"ZYZ\": (-180.0, 180.0),
    \"ZXZ\": (0.0, 180.0),
}

# Werte"
probe "ZYX Klammerung geaendert"     "ca * sb * sc - sa * cc"   "ca * (sb * sc) - sa * cc"
probe "Schwelle zu grob"             "EPS = 1e-9" "EPS = 1e-4"
probe "Standard auf ZYZ"             "STANDARD = \"ZYX\"" "STANDARD = \"ZYZ\""
