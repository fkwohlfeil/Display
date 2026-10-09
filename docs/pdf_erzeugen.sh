#!/bin/bash
# Erzeugt docs/Dokumentation_URDF_GUI.pdf aus den Markdown-Dateien
# (Begriffe.md, Entwicklerdokumentation.md, Bedienerdokumentation.md).
#
# Aufruf (in WSL):  bash docs/pdf_erzeugen.sh
#
# Braucht pandoc (WSL) und Chrome oder Edge unter Windows. Pandoc baut eine
# eigenstaendige HTML-Datei (Bilder und Stil eingebettet), der Browser druckt
# sie ohne Fenster nach PDF. Seitenkopf und Seitenzahlen kommen aus
# pdf_stil.css (@page). Gearbeitet wird im Windows-Temp-Ordner, weil der
# Browser dort ohne Umweg ueber \\wsl.localhost lesen und schreiben kann.
set -euo pipefail

HIER="$(cd "$(dirname "$0")" && pwd)"
ZIEL="$HIER/Dokumentation_URDF_GUI.pdf"

BROWSER=""
for kandidat in \
    "/mnt/c/Program Files/Google/Chrome/Application/chrome.exe" \
    "/mnt/c/Program Files (x86)/Microsoft/Edge/Application/msedge.exe" \
    "/mnt/c/Program Files/Microsoft/Edge/Application/msedge.exe"; do
  if [ -x "$kandidat" ]; then BROWSER="$kandidat"; break; fi
done
[ -n "$BROWSER" ] || { echo "Kein Chrome/Edge unter Windows gefunden" >&2; exit 1; }

WINTEMP="$(cd /mnt/c && cmd.exe /c 'echo %TEMP%' 2>/dev/null | tr -d '\r')"
ARBEIT="$(wslpath -u "$WINTEMP")/urdf_gui_doku"
mkdir -p "$ARBEIT"
HTML="$ARBEIT/Dokumentation.html"
PDF="$ARBEIT/Dokumentation.pdf"
rm -f "$PDF"

cd "$HIER"

# Codezeilen ueber ~95 Zeichen bricht der Browser in der PDF um. Wer sie von
# dort kopiert, bekommt Zeilenumbrueche mitten im Befehl. Mit `\` kuerzen.
awk 'FNR==1{im_code=0} /^ *```/{im_code=!im_code; next}
     im_code && length($0)>95 {printf "WARNUNG %s:%d: Codezeile mit %d Zeichen wird umbrochen\n", FILENAME, FNR, length($0)}' \
  Begriffe.md Entwicklerdokumentation.md Bedienerdokumentation.md >&2

pandoc Begriffe.md Entwicklerdokumentation.md Bedienerdokumentation.md \
  --from markdown --to html5 --standalone --embed-resources \
  --toc --toc-depth=2 --css pdf_stil.css --columns=10000 \
  --lua-filter befehle_ganz.lua \
  --metadata title="URDF-GUI für Bahnplanung" \
  --metadata subtitle="Entwickler- und Bedienerdokumentation" \
  --metadata date="Stand: urdf_gui.py 2.3 · Oktober 2026" \
  --metadata toc-title="Inhalt" \
  --metadata lang=de \
  -o "$HTML"

# Eigenes Profil, damit ein offenes Browserfenster nicht stoert.
"$BROWSER" --headless=new --disable-gpu --no-pdf-header-footer \
  --user-data-dir="$(wslpath -w "$ARBEIT")\\profil" \
  --print-to-pdf="$(wslpath -w "$PDF")" \
  "file:///$(wslpath -w "$HTML" | sed 's|\\|/|g')" >/dev/null 2>&1 || true

[ -s "$PDF" ] || { echo "PDF nicht erzeugt" >&2; exit 1; }
cp "$PDF" "$ZIEL"
echo "geschrieben: $ZIEL"
