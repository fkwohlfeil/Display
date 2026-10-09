-- Pandoc-Filter fuer pdf_erzeugen.sh: markiert kurze Inline-Codes mit der
-- Klasse "ganz". pdf_stil.css bricht sie im Fliesstext nicht um, damit ein
-- aus der PDF kopierter Befehl keinen Zeilenumbruch enthaelt. Laengere Codes
-- (Signaturen in Teil I) bleiben umbrechbar, sonst ragen sie aus der Seite.
local MAX_ZEICHEN = 50

function Code(el)
  if utf8.len(el.text) <= MAX_ZEICHEN then
    el.classes:insert('ganz')
  end
  return el
end
