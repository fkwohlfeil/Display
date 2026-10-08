// json_klein.h -- kleiner JSON-Leser und -Schreiber fuer den
// intrinsic_toppra_wrapper. Eigener Code, keine Fremdbibliothek.
//
// Warum eigen: Die JSON-Bibliothek aus dem Intrinsic-Build (nlohmann_json)
// liess sich nicht laden. Auftrag und Antwort brauchen nur Objekte, Listen,
// Zahlen, Texte, Wahrheitswerte und null -- das deckt diese Datei ab.
//
// Zahlen werden mit 17 signifikanten Stellen geschrieben, sodass jeder
// double beim Wiedereinlesen bitgleich herauskommt. Objekte behalten die
// Reihenfolge ihrer Schluessel. Nicht endliche Zahlen (inf, nan) lassen sich
// in JSON nicht darstellen; der Schreiber bricht dann mit Fehler ab, statt
// still etwas anderes zu schreiben.

#ifndef INTRINSIC_TOPPRA_WRAPPER_JSON_KLEIN_H_
#define INTRINSIC_TOPPRA_WRAPPER_JSON_KLEIN_H_

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace json_klein {

class Wert {
 public:
  enum class Typ { kNull, kBool, kZahl, kText, kListe, kObjekt };

  Wert() = default;
  Wert(bool b) : typ_(Typ::kBool), bool_(b) {}                  // NOLINT
  Wert(double z) : typ_(Typ::kZahl), zahl_(z) {}                // NOLINT
  Wert(int z) : typ_(Typ::kZahl), zahl_(z) {}                   // NOLINT
  Wert(long z) : typ_(Typ::kZahl), zahl_(static_cast<double>(z)) {}  // NOLINT
  Wert(unsigned long z)                                         // NOLINT
      : typ_(Typ::kZahl), zahl_(static_cast<double>(z)) {}
  Wert(const char* t) : typ_(Typ::kText), text_(t) {}           // NOLINT
  Wert(std::string t) : typ_(Typ::kText), text_(std::move(t)) {}  // NOLINT
  Wert(const std::vector<double>& zahlen) : typ_(Typ::kListe) {  // NOLINT
    for (double z : zahlen) liste_.emplace_back(z);
  }
  Wert(const std::vector<std::string>& texte) : typ_(Typ::kListe) {  // NOLINT
    for (const std::string& t : texte) liste_.emplace_back(t);
  }

  static Wert Liste() {
    Wert w;
    w.typ_ = Typ::kListe;
    return w;
  }
  static Wert Objekt() {
    Wert w;
    w.typ_ = Typ::kObjekt;
    return w;
  }

  Typ typ() const { return typ_; }
  bool IstZahl() const { return typ_ == Typ::kZahl; }
  bool IstListe() const { return typ_ == Typ::kListe; }
  bool IstObjekt() const { return typ_ == Typ::kObjekt; }

  // ---- Lesen, mit Fehlermeldung statt stiller Umdeutung ----

  double Zahl() const {
    Erwarte(Typ::kZahl, "Zahl");
    return zahl_;
  }
  int Ganzzahl() const {
    const double z = Zahl();
    if (z != std::floor(z) || std::abs(z) > 2e9) {
      throw std::runtime_error("Ganzzahl erwartet, gefunden " + Format(z));
    }
    return static_cast<int>(z);
  }
  bool Bool() const {
    Erwarte(Typ::kBool, "true/false");
    return bool_;
  }
  const std::string& Text() const {
    Erwarte(Typ::kText, "Text");
    return text_;
  }
  const std::vector<Wert>& Elemente() const {
    Erwarte(Typ::kListe, "Liste");
    return liste_;
  }
  size_t Groesse() const {
    if (typ_ == Typ::kListe) return liste_.size();
    if (typ_ == Typ::kObjekt) return objekt_.size();
    return 0;
  }

  bool Enthaelt(const std::string& schluessel) const {
    if (typ_ != Typ::kObjekt) return false;
    for (const auto& [k, w] : objekt_) {
      if (k == schluessel) return true;
    }
    return false;
  }
  const Wert& An(const std::string& schluessel) const {
    Erwarte(Typ::kObjekt, "Objekt");
    for (const auto& [k, w] : objekt_) {
      if (k == schluessel) return w;
    }
    throw std::runtime_error("Schluessel fehlt: \"" + schluessel + "\"");
  }
  const Wert& An(size_t index) const {
    const std::vector<Wert>& e = Elemente();
    if (index >= e.size()) {
      throw std::runtime_error("Listenindex " + std::to_string(index) +
                               " ausserhalb (Laenge " +
                               std::to_string(e.size()) + ")");
    }
    return e[index];
  }

  std::vector<double> Zahlen() const {
    std::vector<double> ergebnis;
    for (const Wert& w : Elemente()) ergebnis.push_back(w.Zahl());
    return ergebnis;
  }
  std::vector<std::vector<double>> Matrix() const {
    std::vector<std::vector<double>> ergebnis;
    for (const Wert& w : Elemente()) ergebnis.push_back(w.Zahlen());
    return ergebnis;
  }
  std::vector<std::string> Texte() const {
    std::vector<std::string> ergebnis;
    for (const Wert& w : Elemente()) ergebnis.push_back(w.Text());
    return ergebnis;
  }

  // ---- Aufbauen ----

  Wert& Anhaengen(Wert w) {
    Erwarte(Typ::kListe, "Liste");
    liste_.push_back(std::move(w));
    return *this;
  }
  // Setzt einen Schluessel; ein vorhandener wird ersetzt.
  Wert& Setzen(const std::string& schluessel, Wert w) {
    Erwarte(Typ::kObjekt, "Objekt");
    for (auto& [k, alt] : objekt_) {
      if (k == schluessel) {
        alt = std::move(w);
        return *this;
      }
    }
    objekt_.emplace_back(schluessel, std::move(w));
    return *this;
  }

  // ---- Schreiben ----

  std::string Schreiben() const {
    std::string aus;
    SchreibeIn(aus);
    return aus;
  }

  static std::string Format(double z) {
    if (!std::isfinite(z)) {
      throw std::runtime_error("Nicht endliche Zahl laesst sich nicht als "
                               "JSON schreiben");
    }
    if (z == std::floor(z) && std::abs(z) < 1e15) {
      char puffer[32];
      std::snprintf(puffer, sizeof(puffer), "%.0f", z);
      // -0 als 0 schreiben waere eine stille Aenderung; %.0f gibt "-0"
      // aus, das ist gueltiges JSON und bleibt so.
      return puffer;
    }
    char puffer[32];
    std::snprintf(puffer, sizeof(puffer), "%.17g", z);
    return puffer;
  }

 private:
  void Erwarte(Typ t, const char* name) const {
    if (typ_ != t) {
      throw std::runtime_error(std::string(name) + " erwartet, gefunden " +
                               TypName(typ_));
    }
  }
  static const char* TypName(Typ t) {
    switch (t) {
      case Typ::kNull: return "null";
      case Typ::kBool: return "true/false";
      case Typ::kZahl: return "Zahl";
      case Typ::kText: return "Text";
      case Typ::kListe: return "Liste";
      case Typ::kObjekt: return "Objekt";
    }
    return "?";
  }
  static void SchreibeText(const std::string& t, std::string& aus) {
    aus.push_back('"');
    for (unsigned char c : t) {
      switch (c) {
        case '"': aus += "\\\""; break;
        case '\\': aus += "\\\\"; break;
        case '\n': aus += "\\n"; break;
        case '\r': aus += "\\r"; break;
        case '\t': aus += "\\t"; break;
        default:
          if (c < 0x20) {
            char puffer[8];
            std::snprintf(puffer, sizeof(puffer), "\\u%04x", c);
            aus += puffer;
          } else {
            aus.push_back(static_cast<char>(c));
          }
      }
    }
    aus.push_back('"');
  }
  void SchreibeIn(std::string& aus) const {
    switch (typ_) {
      case Typ::kNull: aus += "null"; break;
      case Typ::kBool: aus += bool_ ? "true" : "false"; break;
      case Typ::kZahl: aus += Format(zahl_); break;
      case Typ::kText: SchreibeText(text_, aus); break;
      case Typ::kListe:
        aus.push_back('[');
        for (size_t i = 0; i < liste_.size(); ++i) {
          if (i) aus.push_back(',');
          liste_[i].SchreibeIn(aus);
        }
        aus.push_back(']');
        break;
      case Typ::kObjekt:
        aus.push_back('{');
        for (size_t i = 0; i < objekt_.size(); ++i) {
          if (i) aus.push_back(',');
          SchreibeText(objekt_[i].first, aus);
          aus.push_back(':');
          objekt_[i].second.SchreibeIn(aus);
        }
        aus.push_back('}');
        break;
    }
  }

  Typ typ_ = Typ::kNull;
  bool bool_ = false;
  double zahl_ = 0.0;
  std::string text_;
  std::vector<Wert> liste_;
  std::vector<std::pair<std::string, Wert>> objekt_;

  friend class Leser;
};

// Rekursiv absteigender Leser nach RFC 8259. Fehler nennen die Stelle.
class Leser {
 public:
  static Wert Lies(const std::string& text) {
    Leser l(text);
    l.Leerraum();
    Wert w = l.Wert_(0);
    l.Leerraum();
    if (l.pos_ != text.size()) l.Fehler("Zeichen nach dem Ende");
    return w;
  }

 private:
  explicit Leser(const std::string& t) : t_(t) {}

  [[noreturn]] void Fehler(const std::string& was) const {
    size_t zeile = 1, spalte = 1;
    for (size_t i = 0; i < pos_ && i < t_.size(); ++i) {
      if (t_[i] == '\n') {
        ++zeile;
        spalte = 1;
      } else {
        ++spalte;
      }
    }
    throw std::runtime_error("JSON-Fehler in Zeile " + std::to_string(zeile) +
                             ", Spalte " + std::to_string(spalte) + ": " + was);
  }
  void Leerraum() {
    while (pos_ < t_.size() && (t_[pos_] == ' ' || t_[pos_] == '\n' ||
                                t_[pos_] == '\r' || t_[pos_] == '\t')) {
      ++pos_;
    }
  }
  char Zeichen() const { return pos_ < t_.size() ? t_[pos_] : '\0'; }
  void Erwarte(char c) {
    if (Zeichen() != c) Fehler(std::string("'") + c + "' erwartet");
    ++pos_;
  }
  bool Wort(const char* w) {
    const std::string s(w);
    if (t_.compare(pos_, s.size(), s) == 0) {
      pos_ += s.size();
      return true;
    }
    return false;
  }

  Wert Wert_(int tiefe) {
    if (tiefe > 200) Fehler("zu tief verschachtelt");
    const char c = Zeichen();
    if (c == '{') return Objekt_(tiefe);
    if (c == '[') return Liste_(tiefe);
    if (c == '"') return Wert(Text_());
    if (Wort("true")) return Wert(true);
    if (Wort("false")) return Wert(false);
    if (Wort("null")) return Wert();
    if (c == '-' || (c >= '0' && c <= '9')) return Wert(Zahl_());
    Fehler("Wert erwartet");
  }

  Wert Objekt_(int tiefe) {
    Erwarte('{');
    Wert o = Wert::Objekt();
    Leerraum();
    if (Zeichen() == '}') {
      ++pos_;
      return o;
    }
    while (true) {
      Leerraum();
      if (Zeichen() != '"') Fehler("Schluessel in \"...\" erwartet");
      std::string k = Text_();
      if (o.Enthaelt(k)) Fehler("Schluessel doppelt: \"" + k + "\"");
      Leerraum();
      Erwarte(':');
      Leerraum();
      o.objekt_.emplace_back(std::move(k), Wert_(tiefe + 1));
      Leerraum();
      if (Zeichen() == ',') {
        ++pos_;
        continue;
      }
      Erwarte('}');
      return o;
    }
  }

  Wert Liste_(int tiefe) {
    Erwarte('[');
    Wert l = Wert::Liste();
    Leerraum();
    if (Zeichen() == ']') {
      ++pos_;
      return l;
    }
    while (true) {
      Leerraum();
      l.liste_.push_back(Wert_(tiefe + 1));
      Leerraum();
      if (Zeichen() == ',') {
        ++pos_;
        continue;
      }
      Erwarte(']');
      return l;
    }
  }

  static void Utf8(unsigned cp, std::string& aus) {
    if (cp < 0x80) {
      aus.push_back(static_cast<char>(cp));
    } else if (cp < 0x800) {
      aus.push_back(static_cast<char>(0xC0 | (cp >> 6)));
      aus.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
    } else if (cp < 0x10000) {
      aus.push_back(static_cast<char>(0xE0 | (cp >> 12)));
      aus.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
      aus.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
    } else {
      aus.push_back(static_cast<char>(0xF0 | (cp >> 18)));
      aus.push_back(static_cast<char>(0x80 | ((cp >> 12) & 0x3F)));
      aus.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
      aus.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
    }
  }
  unsigned Hex4() {
    if (pos_ + 4 > t_.size()) Fehler("\\u unvollstaendig");
    unsigned wert = 0;
    for (int i = 0; i < 4; ++i) {
      const char c = t_[pos_++];
      wert <<= 4;
      if (c >= '0' && c <= '9') wert |= static_cast<unsigned>(c - '0');
      else if (c >= 'a' && c <= 'f') wert |= static_cast<unsigned>(c - 'a' + 10);
      else if (c >= 'A' && c <= 'F') wert |= static_cast<unsigned>(c - 'A' + 10);
      else Fehler("ungueltige Hexziffer in \\u");
    }
    return wert;
  }

  std::string Text_() {
    Erwarte('"');
    std::string aus;
    while (true) {
      if (pos_ >= t_.size()) Fehler("Text nicht abgeschlossen");
      const char c = t_[pos_++];
      if (c == '"') return aus;
      if (static_cast<unsigned char>(c) < 0x20) Fehler("Steuerzeichen im Text");
      if (c != '\\') {
        aus.push_back(c);
        continue;
      }
      if (pos_ >= t_.size()) Fehler("Text nicht abgeschlossen");
      const char e = t_[pos_++];
      switch (e) {
        case '"': aus.push_back('"'); break;
        case '\\': aus.push_back('\\'); break;
        case '/': aus.push_back('/'); break;
        case 'b': aus.push_back('\b'); break;
        case 'f': aus.push_back('\f'); break;
        case 'n': aus.push_back('\n'); break;
        case 'r': aus.push_back('\r'); break;
        case 't': aus.push_back('\t'); break;
        case 'u': {
          unsigned cp = Hex4();
          if (cp >= 0xD800 && cp <= 0xDBFF) {
            if (!Wort("\\u")) Fehler("Ersatzzeichenpaar unvollstaendig");
            const unsigned tief = Hex4();
            if (tief < 0xDC00 || tief > 0xDFFF) Fehler("Ersatzzeichenpaar ungueltig");
            cp = 0x10000 + ((cp - 0xD800) << 10) + (tief - 0xDC00);
          }
          Utf8(cp, aus);
          break;
        }
        default: Fehler("unbekannte Escape-Folge");
      }
    }
  }

  double Zahl_() {
    const size_t anfang = pos_;
    if (Zeichen() == '-') ++pos_;
    if (Zeichen() == '0') {
      ++pos_;
    } else if (Zeichen() >= '1' && Zeichen() <= '9') {
      while (Zeichen() >= '0' && Zeichen() <= '9') ++pos_;
    } else {
      Fehler("Ziffer erwartet");
    }
    if (Zeichen() == '.') {
      ++pos_;
      if (!(Zeichen() >= '0' && Zeichen() <= '9')) Fehler("Ziffer nach '.' erwartet");
      while (Zeichen() >= '0' && Zeichen() <= '9') ++pos_;
    }
    if (Zeichen() == 'e' || Zeichen() == 'E') {
      ++pos_;
      if (Zeichen() == '+' || Zeichen() == '-') ++pos_;
      if (!(Zeichen() >= '0' && Zeichen() <= '9')) Fehler("Ziffer im Exponenten erwartet");
      while (Zeichen() >= '0' && Zeichen() <= '9') ++pos_;
    }
    const std::string s = t_.substr(anfang, pos_ - anfang);
    char* ende = nullptr;
    const double z = std::strtod(s.c_str(), &ende);
    if (ende != s.c_str() + s.size()) Fehler("Zahl nicht lesbar: " + s);
    if (!std::isfinite(z)) Fehler("Zahl ausserhalb des double-Bereichs: " + s);
    return z;
  }

  const std::string& t_;
  size_t pos_ = 0;
};

inline Wert Lies(const std::string& text) { return Leser::Lies(text); }

}  // namespace json_klein

#endif  // INTRINSIC_TOPPRA_WRAPPER_JSON_KLEIN_H_
