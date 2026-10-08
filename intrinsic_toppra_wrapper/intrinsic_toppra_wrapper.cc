// intrinsic_toppra_wrapper -- eigener Wrapper um TOPP-RA aus Intrinsic Core.
//
// Hilfsprozess der Oberflaeche, Gegenstueck zu totg_rechner.py.
//
// Aufruf:
//     intrinsic_toppra_wrapper AUFTRAG.json ANTWORT.json
//     intrinsic_toppra_wrapper --pruefen
//
// Herkunft des Codes
//
//     Diese Datei ist vollstaendig selbst geschrieben. Sie enthaelt keinen
//     kopierten oder veraenderten Code aus Intrinsic Core. Aus Intrinsic
//     Core (Release 20260922.0, Apache 2.0, github.com/intrinsic-ai/
//     intrinsic-core) werden unveraendert aufgerufen -- jede Aufrufstelle
//     ist unten mit [INTRINSIC] markiert:
//
//       topp::ComputeRefinedPathForAccelerationLimitedTrajectory()
//           B-Spline-Pfadglaettung mit Ueberschleifen
//       topp::ToppOptionsWithJointAndCartesianConstraints()
//           Auswahl der aktiven Grenzen
//       topp::ToppRA::Solve()
//           zeitoptimale Parametrierung (TOPP-RA)
//       InterpolateJointTrajectoryInternalType()
//           kubische Interpolation der Rohausgabe
//     und fuer die LIN-Kette (Modi lin_kurve, lin_neuabtastung):
//       CartesianPolylineBSplinesCurve::Create(), ::SampleUniformly(),
//       ::Sample()
//           verrundete kartesische Kurve und ihre Abtastung
//       ComputeCartesianCurveMinimalSamplingStepsAndControlFrequency(),
//       NormalizedSamplingStepForCartesianCurve()
//           Abtastschritte wie im LIN-Planer
//       CreateUniformResamplingStepsForJointSpacePathFunction(),
//       ApproximateJointUniformSampling(), CreateUpsampledCurveParameters()
//           gleichabstaendige Neuabtastung im Gelenkraum
//       dazu die Datentypen PathSegment, PathSample, JointLimits,
//       JointTrajectoryPVA, Pose3d und die Konstante
//       kMaxJointAccelerationToppRa.
//
//     Eigene Anteile: JSON-Auftrag und -Antwort (json_klein.h, ohne
//     Fremdbibliothek), Pruefung der Eingaben,
//     Skalierung der Grenzen, Abschnitte und Verkettung, Abtastung auf
//     resample_dt, Grenzpruefung, Protokoll. Mit [NACHGEBILDET] markiert
//     sind Stellen, die Verhalten von Intrinsic mit eigenem Code
//     wiederholen (Beschleunigungskappung, Minimum an Start und Ende,
//     Meldung zu kleiner Rundungen, Aufwaertsfaktor 10).
//
// Modi (Feld "modus" im Auftrag, Standard "toppra")
//
//     toppra             Gelenkpunkte -> Bahn (wie bisher). Neu: statt
//                        wegpunkte+abschnitte auch "teilpfade", je Abschnitt
//                        eine eigene Punktliste. Ab Stand 3 mit "tcp_v_max"
//                        und "tcp_a_max" (siehe unten: Bahngeschwindigkeit,
//                        Bahnbeschleunigung).
//     toppra_proben      derselbe Auftrag wie toppra, aber nur die Glaettung:
//                        die Pfadproben (q, dq/ds, d2q/ds2) je Abschnitt,
//                        fuer die Bahngrenzen. Ab Stand 3.
//     lin_kurve          Posen -> verrundete kartesische Kurve, abgetastet
//     lin_neuabtastung   Kurvenparameter + IK-Loesungen -> dichte Posen fuer
//                        eine im Gelenkraum gleichabstaendige Abtastung
//
//     Die IK zwischen den LIN-Schritten rechnet MoveIt, nicht Intrinsic
//     (intrinsic_lin_rechner.py).
//
// Was gerechnet wird
//
//     Je Abschnitt ein PathSegment aus den Wegpunkten (nur aktive Achsen),
//     daraus mit ComputeRefinedPathForAccelerationLimitedTrajectory() den
//     B-Spline-Pfad (Grad 4, Ueberschleifen je Punkt) und mit topp::ToppRA
//     die zeitoptimale, beschleunigungsbegrenzte Parametrierung. Der
//     Solver wird direkt aufgerufen und nicht ueber die Komfortfunktion,
//     damit die Zahl seiner Iterationen in die Antwort kommt.
//
// Was nicht passiert
//
//     Keine Kollisionspruefung, keine kartesischen Grenzen von Intrinsic
//     (die braeuchten eine kinematics::Chain, und die kennt keine Mimic-
//     Gelenke), kein Ruck-Limit.
//     Nichts wird still nachgebessert: Alles, was der Wrapper oder der
//     Solver an den Vorgaben aendert, steht in "hinweise", und die Grenzen
//     werden an der Rohausgabe und an der abgetasteten Ausgabe nachgeprueft
//     ("grenzpruefung"). Liegt ein Wert ueber der Grenze, wird er gemeldet,
//     nicht korrigiert.
//
// Abschnitte
//
//     Wie bei totg_rechner.py beginnt und endet jeder Abschnitt im
//     Stillstand. Innerhalb eines Abschnitts wird an jedem inneren Wegpunkt
//     mit "ueberschleifen_rad" ueberschliffen. Erster und letzter Punkt
//     bekommen immer das Minimum 1e-5 rad, wie im Motion Planner von
//     Intrinsic.
//
// Bahngeschwindigkeit (ab Stand 3, eigener Code)
//
//     Der Pfad q(s) liegt nach der Glaettung fest. Fuer die Tip-Position x
//     gilt dann |dx/dt| = |dx/ds| * ds/dt. Die Grenze |dx/dt| <= v heisst
//     also ds/dt <= v / |dx/ds|. TOPP-RA begrenzt ds/dt ueber die
//     Gelenkgeschwindigkeit je Probe: ds/dt <= max_velocity_k / |dq_k/ds|.
//     Setzt man an einer Probe max_velocity_k = v * |dq_k/ds| / |dx/ds|,
//     ist das genau die Bahngrenze -- ohne Naeherung, ohne Kinematik hier.
//     Geaendert wird nur, wo die Bahngrenze strenger ist als die Achsen.
//
//     x, dx/ds und d2x/ds2 je Probe kommen im Auftrag ("tcp_x", "tcp_xp",
//     "tcp_xpp"), gerechnet vom Aufrufer mit seiner Vorwaertskinematik
//     (MoveIt, samt Mimic-Gelenken) an den Proben, die toppra_proben
//     geliefert hat. Dass es dieselben Proben sind, sichern Anzahl und
//     "tcp_pruefsummen".
//
// Bahnbeschleunigung (ab Stand 3, eigener Code)
//
//     a = dx/ds * d2s/dt2 + d2x/ds2 * (ds/dt)^2 ist nicht als Grenze der
//     echten Achsen zu schreiben -- die Koeffizienten sind andere als
//     dq/ds und d2q/ds2. Deshalb bekommt jede Probe 13 Zusatzachsen
//     y_j(s) = d_j . x(s) mit festen Einheitsrichtungen d_j (die drei
//     Achsen, sechs Flaechen- und vier Raumdiagonalen), Ableitungen
//     d_j . dx/ds und d_j . d2x/ds2, und als Beschleunigungsgrenze
//     c = A * kTcpPolyederFaktor, sonst unbegrenzt. TOPP-RA haelt dann
//     |d_j . a| <= c fuer alle j, und das umschliesst die Kugel |a| <= A
//     von innen: |a| <= A ist garantiert, in der ungunstigsten Richtung
//     wird aber nur 88,6 % von A ausgenutzt. Die Richtungen muessen fest
//     sein: TOPP-RA prueft die Beschleunigung danach als Differenz der
//     Geschwindigkeiten benachbarter Proben. Nach dem Loesen werden die
//     Zusatzachsen wieder abgeschnitten.
//
// Die Antwort wird erst in eine Nebendatei geschrieben und dann umbenannt.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <unistd.h>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/time/time.h"
#include "absl/types/span.h"
#include "intrinsic/eigenmath/types.h"
#include "intrinsic/kinematics/types/joint_limits.h"
#include "intrinsic/kinematics/types/joint_state.h"
#include "intrinsic/kinematics/types/joint_trajectories.h"
#include "intrinsic/math/pose3.h"
#include "intrinsic/motion_planning/path_planning/parametric_curves/cartesian_parametric_curve.h"
#include "intrinsic/motion_planning/path_planning/parametric_curves/cartesian_parametric_curve_utils.h"
#include "intrinsic/motion_planning/path_planning/parametric_curves/cartesian_polyline_bsplines_curve.h"
#include "intrinsic/motion_planning/path_planning/parametric_curves/parametric_curve_utils.h"
#include "intrinsic/motion_planning/path_planning/path_segment.h"
#include "intrinsic/motion_planning/path_planning/planners/joint_sampling_utils.h"
#include "intrinsic/motion_planning/trajectory_planning/interpolate_joint_trajectories.h"
#include "intrinsic/motion_planning/trajectory_planning/optimized_toppra.h"
#include "intrinsic/motion_planning/trajectory_planning/path_refinement/spline_based_path_refinement.h"
#include "intrinsic/motion_planning/trajectory_planning/topp/joint_optimization_options.pb.h"
#include "intrinsic/motion_planning/trajectory_planning/topp/path_sample.h"
#include "intrinsic/motion_planning/trajectory_planning/topp/refine_path_and_compute_acceleration_limited_trajectory.h"
#include "intrinsic/motion_planning/trajectory_planning/topp/topp_options.h"
#include "intrinsic_toppra_wrapper/json_klein.h"

namespace {

using json_klein::Wert;
using intrinsic::JointLimits;
using intrinsic::JointStatePVA;
using intrinsic::JointStatePVAJ;
using intrinsic::JointTrajectoryPVA;
using intrinsic::PathSegment;
using intrinsic::eigenmath::VectorNd;

// Stand dieses Wrappers. Ein Auftrag mit hoeherem Stand traegt Felder, die
// hier nicht verstanden wuerden -- er wird abgelehnt, nicht halb gerechnet.
// Seit 3: Bahngeschwindigkeit (tcp_v_max) und Modus toppra_proben.
constexpr int kStand = 3;

// Release von intrinsic-core, gegen den dieser Wrapper geschrieben ist.
constexpr char kIntrinsicRelease[] = "20260922.0";

// Minimum der Ueberschleif-Toleranz (wie kMinimumLowLevelJointBlending-
// DeviationRad in motion_planner.cc). Bedeutet: exakt durch den Punkt.
constexpr double kMinUeberschleifenRad = 1e-5;

// Unterhalb dieser Laenge (Euklid-Norm im Gelenkraum) hat ein Abschnitt keine
// Bewegung. Gleicher Wert wie kJointConfigEqualityMarginRad.
constexpr double kMinAbschnittLaengeRad = 5e-5;

// Feste Werte im Intrinsic-Code, die hier nur fuer das Protokoll stehen. Sie
// liegen dort in anonymen Namensraeumen und sind von aussen nicht lesbar --
// hier von Hand abgeschrieben (Stand 20260922.0). Aendert Intrinsic sie, muss
// man sie hier nachtragen. Sie beeinflussen die Rechnung nicht.
constexpr double kIntrinsicSicherheitsfaktor = 0.98;  // optimized_toppra.cc
constexpr int kIntrinsicMaxIterationen = 10;          // optimized_toppra.cc
constexpr int kIntrinsicSplineGrad = 4;               // spline_based_...cc
constexpr double kIntrinsicEckabstandRad = 0.3;       // spline_based_...h

// Kleine Toleranz fuer die Grenzpruefung (relativ), gegen reine
// Rundungsmeldungen. Alles darueber wird als Verletzung gezaehlt.
constexpr double kPruefToleranz = 1e-9;

// Feste Richtungen der Zusatzachsen fuer die Bahnbeschleunigung (werden
// beim Gebrauch normiert): drei Achsen, sechs Flaechen-, vier Raumdiagonalen.
constexpr int kTcpRichtungenAnzahl = 13;
constexpr double kTcpRichtungen[kTcpRichtungenAnzahl][3] = {
    {1, 0, 0}, {0, 1, 0},  {0, 0, 1},  {1, 1, 0},  {1, -1, 0},
    {1, 0, 1}, {1, 0, -1}, {0, 1, 1},  {0, 1, -1}, {1, 1, 1},
    {1, 1, -1}, {1, -1, 1}, {-1, 1, 1}};

// Groesster Faktor c/A, bei dem das Polyeder |d_j . a| <= c fuer die
// Richtungen oben ganz in der Kugel |a| <= A liegt: 1 / Umkugelradius bei
// c = 1, aus den Ecken gerechnet 0.886451886... Abgerundet.
constexpr double kTcpPolyederFaktor = 0.8864;

// Unter diesem |dx/ds| (m je Einheit s) steht der Tip auf der Bahn praktisch
// still -- an Start und Ende eines Abschnitts ist dq/ds null. Dort wirkt
// die Bahngeschwindigkeit nicht; die Achsgrenzen bleiben.
constexpr double kMinTcpAbleitung = 1e-9;

// Relative Toleranz, mit der die Pruefsumme der Proben aus toppra_proben
// wiedererkannt wird. Gleiche Eingaben ergeben bitgleiche Proben; die
// Toleranz deckt nur den Weg durch JSON ab (17 Stellen, verlustfrei).
constexpr double kPruefsummeToleranz = 1e-12;

// ---------------------------------------------------------------- Einlesen

std::vector<double> Zahlenliste(const Wert& j, const std::string& name,
                                size_t anzahl) {
  const std::vector<double> werte = j.An(name).Zahlen();
  if (werte.size() != anzahl) {
    throw std::runtime_error(absl::StrCat(name, " hat ", werte.size(),
                                          " Eintraege, erwartet ", anzahl));
  }
  for (double w : werte) {
    if (std::isnan(w)) {
      throw std::runtime_error(absl::StrCat(name, " enthaelt NaN"));
    }
  }
  return werte;
}

VectorNd ZuVektor(const std::vector<double>& werte) {
  if (werte.size() >
      static_cast<size_t>(intrinsic::eigenmath::MAX_EIGEN_VECTOR_SIZE)) {
    throw std::runtime_error(absl::StrCat(
        "Zu viele Achsen: ", werte.size(), ", Intrinsic kann hoechstens ",
        intrinsic::eigenmath::MAX_EIGEN_VECTOR_SIZE));
  }
  VectorNd v(static_cast<Eigen::Index>(werte.size()));
  for (size_t i = 0; i < werte.size(); ++i) {
    v[static_cast<Eigen::Index>(i)] = werte[i];
  }
  return v;
}

std::vector<double> ZuListe(const VectorNd& v) {
  std::vector<double> werte(static_cast<size_t>(v.size()));
  for (Eigen::Index i = 0; i < v.size(); ++i) {
    werte[static_cast<size_t>(i)] = v[i];
  }
  return werte;
}

double Sekunden(absl::Duration d) { return absl::ToDoubleSeconds(d); }

// ------------------------------------------------------------ Grenzpruefung

// Groesster Faktor |Wert| / Grenze ueber alle Punkte und Achsen, und die Zahl
// der Punkte, an denen er ueber 1 liegt. Unendliche Grenzen zaehlen nicht.
struct Pruefung {
  double v_faktor_max = 0.0;
  double a_faktor_max = 0.0;
  int v_verletzungen = 0;
  int a_verletzungen = 0;
  int punkte = 0;

  void Nimm(const VectorNd& v, const VectorNd& a, const JointLimits& g) {
    ++punkte;
    double fv = 0.0;
    double fa = 0.0;
    for (Eigen::Index i = 0; i < v.size(); ++i) {
      if (std::isfinite(g.max_velocity[i]) && g.max_velocity[i] > 0.0) {
        fv = std::max(fv, std::abs(v[i]) / g.max_velocity[i]);
      }
      if (std::isfinite(g.max_acceleration[i]) && g.max_acceleration[i] > 0.0) {
        fa = std::max(fa, std::abs(a[i]) / g.max_acceleration[i]);
      }
    }
    v_faktor_max = std::max(v_faktor_max, fv);
    a_faktor_max = std::max(a_faktor_max, fa);
    if (fv > 1.0 + kPruefToleranz) ++v_verletzungen;
    if (fa > 1.0 + kPruefToleranz) ++a_verletzungen;
  }

  bool Eingehalten() const { return v_verletzungen == 0 && a_verletzungen == 0; }

  Wert AlsJson() const {
    Wert o = Wert::Objekt();
    o.Setzen("punkte", punkte);
    o.Setzen("v_faktor_max", v_faktor_max);
    o.Setzen("a_faktor_max", a_faktor_max);
    o.Setzen("v_verletzungen", v_verletzungen);
    o.Setzen("a_verletzungen", a_verletzungen);
    o.Setzen("eingehalten", Eingehalten());
    return o;
  }
};

// ------------------------------------------------------------ Abschnitt

struct AbschnittErgebnis {
  JointTrajectoryPVA roh;
  int iterationen = 0;
  size_t pfadpunkte = 0;
  double rechenzeit_s = 0.0;
  // Proben, an denen die Bahngeschwindigkeit strenger war als die Achsen;
  // -1 ohne Bahngeschwindigkeit.
  int tcp_massgeblich = -1;
  std::vector<std::string> hinweise;
};

// Bahngeschwindigkeit und -beschleunigung eines Abschnitts, aus dem Auftrag.
struct TcpGrenze {
  double v_max = 0.0;  // m/s, schon mit velocity_scaling; 0: keine
  double a_max = 0.0;  // m/s^2, schon mit acceleration_scaling; 0: keine
  // Je Probe [x, y, z]: Tip-Position und ihre Ableitungen nach s.
  std::vector<std::vector<double>> x, xp, xpp;
  double pruefsumme = 0.0;  // aus toppra_proben, zum Wiedererkennen
};

// Die Pfadproben eines Abschnitts: B-Spline-Glaettung mit Ueberschleifen.
// toppra_proben und toppra rechnen sie aus denselben Eingaben; gleiche
// Eingaben ergeben dieselben Proben.
std::vector<intrinsic::topp::PathSample> Glaetten(
    const std::vector<std::vector<double>>& wegpunkte,
    const std::vector<double>& ueberschleifen, const JointLimits& grenzen,
    const std::string& name) {
  PathSegment segment;
  for (size_t i = 0; i < wegpunkte.size(); ++i) {
    segment.joint_configurations.push_back(ZuVektor(wegpunkte[i]));
    segment.joint_blending_parameter_rad.push_back(
        std::max(ueberschleifen[i], kMinUeberschleifenRad));
  }
  // [NACHGEBILDET] Wie im Motion Planner von Intrinsic (motion_planner.cc):
  // erster und letzter Punkt bekommen das Minimum, dort also kein
  // Ueberschleifen.
  segment.joint_blending_parameter_rad.front() = kMinUeberschleifenRad;
  segment.joint_blending_parameter_rad.back() = kMinUeberschleifenRad;
  segment.joint_limits = grenzen;
  segment.type = PathSegment::Type::kJoint;

  // [INTRINSIC] B-Spline-Pfadglaettung mit Ueberschleifen.
  absl::StatusOr<intrinsic::topp::SplineBasedPathRefinementResult> pfad =
      intrinsic::topp::ComputeRefinedPathForAccelerationLimitedTrajectory(
          absl::MakeConstSpan(&segment, 1));
  if (!pfad.ok()) {
    throw std::runtime_error(absl::StrCat("Pfadglaettung im Abschnitt ", name,
                                          ": ", pfad.status().ToString()));
  }
  return std::move(pfad->path_samples);
}

// Fingerabdruck der Proben: Summe aus s und allen q. Nur zum Wiedererkennen
// zwischen toppra_proben und toppra, keine Rechengroesse.
double Pruefsumme(absl::Span<const intrinsic::topp::PathSample> proben) {
  double summe = 0.0;
  for (const intrinsic::topp::PathSample& probe : proben) {
    summe += probe.s + probe.q.sum();
  }
  return summe;
}

// Passen die Tip-Werte im Auftrag zu diesen Proben? Wirft sonst.
void TcpPruefen(const TcpGrenze& tcp,
                absl::Span<const intrinsic::topp::PathSample> proben,
                const std::string& name) {
  for (const auto& [feld, werte] :
       {std::pair{"tcp_x", &tcp.x}, std::pair{"tcp_xp", &tcp.xp},
        std::pair{"tcp_xpp", &tcp.xpp}}) {
    if (werte->size() != proben.size()) {
      throw std::runtime_error(absl::StrCat(
          "Abschnitt ", name, ": ", feld, " hat ", werte->size(),
          " Werte, die Glaettung ", proben.size(),
          " Proben -- nicht aus diesem Auftrag (toppra_proben)?"));
    }
    for (size_t i = 0; i < werte->size(); ++i) {
      const std::vector<double>& w = (*werte)[i];
      if (w.size() != 3 || !std::isfinite(w[0]) || !std::isfinite(w[1]) ||
          !std::isfinite(w[2])) {
        throw std::runtime_error(absl::StrCat(
            "Abschnitt ", name, ": ", feld, "[", i,
            "] ist kein endlicher Vektor [x, y, z]"));
      }
    }
  }
  const double summe = Pruefsumme(proben);
  if (std::abs(summe - tcp.pruefsumme) >
      kPruefsummeToleranz * (1.0 + std::abs(summe))) {
    throw std::runtime_error(absl::StrCat(
        "Abschnitt ", name, ": Pruefsumme der Proben ", summe,
        " passt nicht zu tcp_pruefsummen ", tcp.pruefsumme,
        " -- nicht aus diesem Auftrag (toppra_proben)?"));
  }
}

double Betrag(const std::vector<double>& v) {
  return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

// Bahngeschwindigkeit als Gelenkgeschwindigkeitsgrenze je Probe (siehe
// Kopfkommentar). Eigener Code. Gibt die Zahl der Proben zurueck, an denen
// die Bahn strenger war als die Achsen und die Grenzen geaendert wurden.
int TcpGeschwindigkeitAnwenden(const TcpGrenze& tcp,
                               std::vector<intrinsic::topp::PathSample>& proben) {
  int massgeblich = 0;
  for (size_t i = 0; i < proben.size(); ++i) {
    const double ableitung = Betrag(tcp.xp[i]);
    if (ableitung <= kMinTcpAbleitung) continue;
    intrinsic::topp::PathSample& probe = proben[i];
    VectorNd& v_grenze = probe.joint_limits.max_velocity;
    // ds/dt, das die Achsen hier zulassen, und das, was die Bahn zulaesst.
    double s_punkt_achsen = std::numeric_limits<double>::infinity();
    for (Eigen::Index k = 0; k < probe.qp.size(); ++k) {
      if (probe.qp[k] != 0.0) {
        s_punkt_achsen =
            std::min(s_punkt_achsen, v_grenze[k] / std::abs(probe.qp[k]));
      }
    }
    const double s_punkt_bahn = tcp.v_max / ableitung;
    if (!(s_punkt_bahn < s_punkt_achsen)) continue;
    ++massgeblich;
    for (Eigen::Index k = 0; k < probe.qp.size(); ++k) {
      if (probe.qp[k] != 0.0) {
        v_grenze[k] =
            std::min(v_grenze[k], s_punkt_bahn * std::abs(probe.qp[k]));
      }
    }
  }
  return massgeblich;
}

// Haengt an einen Vektor die Werte f(j) fuer j = 0 .. kTcpRichtungenAnzahl-1.
template <typename F>
void Anhaengen(VectorNd& v, F f) {
  const Eigen::Index n = v.size();
  v.conservativeResize(n + kTcpRichtungenAnzahl);
  for (int j = 0; j < kTcpRichtungenAnzahl; ++j) v[n + j] = f(j);
}

// Bahnbeschleunigung ueber Zusatzachsen y_j = d_j . x (siehe Kopfkommentar).
// Eigener Code. a_grenze ist die Grenze je Zusatzachse.
void TcpAchsenAnhaengen(const TcpGrenze& tcp, double a_grenze,
                        std::vector<intrinsic::topp::PathSample>& proben) {
  double richtung[kTcpRichtungenAnzahl][3];
  for (int j = 0; j < kTcpRichtungenAnzahl; ++j) {
    const double* d = kTcpRichtungen[j];
    const double l = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
    for (int k = 0; k < 3; ++k) richtung[j][k] = d[k] / l;
  }
  auto projektion = [&richtung](const std::vector<double>& v) {
    return [&richtung, w = v.data()](int j) {
      return richtung[j][0] * w[0] + richtung[j][1] * w[1] +
             richtung[j][2] * w[2];
    };
  };
  const double unendlich = std::numeric_limits<double>::infinity();
  auto fest = [](double w) { return [w](int) { return w; }; };
  for (size_t i = 0; i < proben.size(); ++i) {
    intrinsic::topp::PathSample& p = proben[i];
    Anhaengen(p.q, projektion(tcp.x[i]));
    Anhaengen(p.qp, projektion(tcp.xp[i]));
    Anhaengen(p.qpp, projektion(tcp.xpp[i]));
    // Dritte Ableitung: nur fuer Ruckgrenzen, die hier aus sind.
    Anhaengen(p.qppp, fest(0.0));
    if (p.qppp_in.has_value()) Anhaengen(*p.qppp_in, fest(0.0));
    JointLimits& g = p.joint_limits;
    Anhaengen(g.min_position, fest(-unendlich));
    Anhaengen(g.max_position, fest(unendlich));
    Anhaengen(g.max_velocity, fest(unendlich));
    Anhaengen(g.max_acceleration, fest(a_grenze));
    Anhaengen(g.max_jerk, fest(unendlich));
    Anhaengen(g.max_torque, fest(unendlich));
  }
}

// Die Bahn ohne die Zusatzachsen: nur die ersten n Achsen.
JointTrajectoryPVA ZusatzachsenAbschneiden(const JointTrajectoryPVA& roh,
                                           Eigen::Index n,
                                           const std::string& name) {
  std::vector<JointStatePVA> zustaende;
  std::vector<absl::Duration> zeiten;
  zustaende.reserve(roh.size());
  zeiten.reserve(roh.size());
  for (size_t i = 0; i < roh.size(); ++i) {
    const JointStatePVA& z = roh.data()[i];
    JointStatePVA k;
    k.position = z.position.head(n);
    k.velocity = z.velocity.head(n);
    k.acceleration = z.acceleration.head(n);
    zustaende.push_back(std::move(k));
    zeiten.push_back(roh.time_stamps()[i]);
  }
  absl::StatusOr<JointTrajectoryPVA> neu = JointTrajectoryPVA::Create(
      std::move(zustaende), std::move(zeiten),
      roh.joint_dynamic_limits_check_mode(), roh.interpolation_type());
  if (!neu.ok()) {
    throw std::runtime_error(absl::StrCat(
        "Abschnitt ", name, ": Bahn ohne Zusatzachsen: ",
        neu.status().ToString()));
  }
  return *std::move(neu);
}

AbschnittErgebnis RechneAbschnitt(
    const std::vector<std::vector<double>>& wegpunkte,
    const std::vector<double>& ueberschleifen, const JointLimits& grenzen,
    const std::string& name, const TcpGrenze* tcp) {
  AbschnittErgebnis ergebnis;
  const auto beginn = std::chrono::steady_clock::now();

  std::vector<intrinsic::topp::PathSample> proben =
      Glaetten(wegpunkte, ueberschleifen, grenzen, name);
  ergebnis.pfadpunkte = proben.size();

  // [NACHGEBILDET] Dasselbe wie die interne Funktion
  // ComputeAccelerationLimitedTrajectory() in Intrinsic: die Beschleunigung
  // auf kMaxJointAccelerationToppRa kappen. Die Konstante stammt aus
  // Intrinsic, die Schleife ist eigener Code. Hier nicht still, sondern mit
  // Hinweis je Achse.
  const double a_kappe = intrinsic::topp::kMaxJointAccelerationToppRa;
  for (Eigen::Index i = 0; i < grenzen.max_acceleration.size(); ++i) {
    if (grenzen.max_acceleration[i] > a_kappe) {
      ergebnis.hinweise.push_back(absl::StrCat(
          "Abschnitt ", name, ": Achse ", i, " max_acceleration ",
          grenzen.max_acceleration[i], " rad/s^2 auf ", a_kappe,
          " gekappt (Grenze von Intrinsic TOPP-RA)"));
    }
  }
  for (intrinsic::topp::PathSample& probe : proben) {
    probe.joint_limits.max_acceleration =
        probe.joint_limits.max_acceleration.cwiseMin(a_kappe);
  }

  // Bahngeschwindigkeit und -beschleunigung, wenn verlangt (eigener Code).
  const Eigen::Index achsen = grenzen.max_velocity.size();
  bool zusatzachsen = false;
  if (tcp != nullptr) {
    TcpPruefen(*tcp, proben, name);
    if (tcp->v_max > 0.0) {
      ergebnis.tcp_massgeblich = TcpGeschwindigkeitAnwenden(*tcp, proben);
    }
    if (tcp->a_max > 0.0) {
      // Wie bei den Achsen: nicht ueber die Kappe von Intrinsic TOPP-RA.
      double a_grenze = tcp->a_max * kTcpPolyederFaktor;
      if (a_grenze > a_kappe) {
        ergebnis.hinweise.push_back(absl::StrCat(
            "Abschnitt ", name, ": Bahnbeschleunigung je Richtung ", a_grenze,
            " m/s^2 auf ", a_kappe,
            " gekappt (Grenze von Intrinsic TOPP-RA)"));
        a_grenze = a_kappe;
      }
      TcpAchsenAnhaengen(*tcp, a_grenze, proben);
      zusatzachsen = true;
    }
  }

  // [INTRINSIC] Aktive Grenzen: nur Gelenkgeschwindigkeit und
  // -beschleunigung.
  const intrinsic::OptimizationOptions optionen =
      intrinsic::topp::ToppOptionsWithJointAndCartesianConstraints(
          /*joint_acceleration=*/true, /*joint_jerk=*/false,
          /*cart_velocity=*/false, /*cart_acceleration=*/false);
  // [INTRINSIC] TOPP-RA-Solver, direkt aufgerufen (wegen num_iterations).
  intrinsic::topp::ToppRA loeser(/*chain=*/nullptr, /*dynamics=*/nullptr,
                                 optionen);
  absl::StatusOr<intrinsic::topp::ToppTrajectoryResult> bahn =
      loeser.Solve(proben);
  ergebnis.rechenzeit_s = std::chrono::duration<double>(
                              std::chrono::steady_clock::now() - beginn)
                              .count();
  if (!bahn.ok()) {
    throw std::runtime_error(absl::StrCat("TOPP-RA im Abschnitt ", name, ": ",
                                          bahn.status().ToString()));
  }
  ergebnis.iterationen = loeser.GetInternalData().num_iterations;
  ergebnis.roh = zusatzachsen
                     ? ZusatzachsenAbschneiden(bahn->trajectory, achsen, name)
                     : bahn->trajectory;
  return ergebnis;
}

// Tastet die Rohausgabe im Takt dt ab, mit derselben Interpolation, mit der
// ICON die Bahn abspielen wuerde (kCubicPolynomial -> kubisch mit stetiger
// Geschwindigkeit). Der letzte Punkt liegt genau auf der Dauer.
std::vector<std::pair<double, JointStatePVAJ>> Abtasten(
    const JointTrajectoryPVA& roh, double dt, const std::string& name) {
  const absl::Duration dauer = roh.Duration();
  const double dauer_s = Sekunden(dauer);
  std::vector<double> zeiten;
  const long anzahl = static_cast<long>(std::floor(dauer_s / dt + 1e-9));
  for (long k = 0; k <= anzahl; ++k) zeiten.push_back(static_cast<double>(k) * dt);
  if (zeiten.back() < dauer_s - 1e-12) zeiten.push_back(dauer_s);

  std::vector<std::pair<double, JointStatePVAJ>> punkte;
  punkte.reserve(zeiten.size());
  for (double t : zeiten) {
    const absl::Duration zeitpunkt = std::min(absl::Seconds(t), dauer);
    // [INTRINSIC] Interpolation, wie beim Abspielen in ICON.
    auto zustand =
        intrinsic::InterpolateJointTrajectoryInternalType(roh, zeitpunkt);
    if (!zustand.ok()) {
      throw std::runtime_error(absl::StrCat(
          "Abtastung im Abschnitt ", name, " bei t=", t, ": ",
          std::string(zustand.status().message())));
    }
    punkte.emplace_back(t, zustand.value());
  }
  return punkte;
}

double Pfadlaenge(const std::vector<std::vector<double>>& punkte) {
  double laenge = 0.0;
  for (size_t i = 1; i < punkte.size(); ++i) {
    double quadrat = 0.0;
    for (size_t k = 0; k < punkte[i].size(); ++k) {
      const double d = punkte[i][k] - punkte[i - 1][k];
      quadrat += d * d;
    }
    laenge += std::sqrt(quadrat);
  }
  return laenge;
}

// ---------------------------------------------------------------- Auftrag

// Liest einen Schluessel, wenn er da ist, sonst den Standardwert.
int GanzzahlOder(const Wert& o, const std::string& k, int standard) {
  return o.Enthaelt(k) ? o.An(k).Ganzzahl() : standard;
}
bool BoolOder(const Wert& o, const std::string& k, bool standard) {
  return o.Enthaelt(k) ? o.An(k).Bool() : standard;
}

// Punkt im Format von totg_rechner.py: [t, lagen, tempi].
Wert Punkt(double t, const VectorNd& lagen, const VectorNd& tempi) {
  Wert p = Wert::Liste();
  p.Anhaengen(t);
  p.Anhaengen(ZuListe(lagen));
  p.Anhaengen(ZuListe(tempi));
  return p;
}

// Ein Abschnitt, wie er in TOPP-RA geht: Gelenkpunkte und die Toleranz fuer
// das Ueberschleifen je Punkt. Start und Ende sind Stillstand.
struct Teil {
  std::string name;
  std::vector<std::vector<double>> punkte;
  std::vector<double> ueberschleifen;
};

// Ueberschleifen: eine Zahl fuer alle Punkte oder eine Liste je Punkt.
std::vector<double> UeberschleifenFuer(const Wert& u, size_t anzahl,
                                       const std::string& wo) {
  std::vector<double> werte;
  if (u.IstZahl()) {
    werte.assign(anzahl, u.Zahl());
  } else {
    werte = u.Zahlen();
    if (werte.size() != anzahl) {
      throw std::runtime_error(absl::StrCat(
          "ueberschleifen_rad ", wo, " hat ", werte.size(),
          " Eintraege, erwartet ", anzahl));
    }
  }
  for (double w : werte) {
    if (!(w > 0.0)) {
      throw std::runtime_error("ueberschleifen_rad muss > 0 sein");
    }
  }
  return werte;
}

// Die Teile eines Auftrags. Zwei Formen:
//   wegpunkte + abschnitte   wie totg_rechner.py (Indexbereiche, die sich an
//                            den Grenzen einen Punkt teilen)
//   teilpfade                je Abschnitt eine eigene Punktliste, fuer die
//                            dichten Gelenkpfade der LIN-Kette
std::vector<Teil> LiesTeile(const Wert& auftrag, size_t n) {
  std::vector<Teil> teile;
  const Wert& u = auftrag.An("ueberschleifen_rad");

  if (auftrag.Enthaelt("teilpfade")) {
    if (auftrag.Enthaelt("wegpunkte")) {
      throw std::runtime_error(
          "Auftrag hat wegpunkte und teilpfade -- nur eines von beiden");
    }
    const Wert& pfade = auftrag.An("teilpfade");
    std::vector<std::string> namen;
    if (auftrag.Enthaelt("namen")) namen = auftrag.An("namen").Texte();
    if (u.IstListe() && u.Groesse() != pfade.Groesse()) {
      throw std::runtime_error(
          "ueberschleifen_rad braucht bei teilpfade eine Zahl oder je "
          "Teilpfad eine Liste");
    }
    for (size_t i = 0; i < pfade.Groesse(); ++i) {
      Teil t;
      t.name = namen.size() == pfade.Groesse() ? namen[i]
                                               : absl::StrCat("teilpfad_", i);
      t.punkte = pfade.An(i).Matrix();
      const std::string wo = absl::StrCat("fuer ", t.name);
      t.ueberschleifen = UeberschleifenFuer(u.IstZahl() ? u : u.An(i),
                                            t.punkte.size(), wo);
      teile.push_back(std::move(t));
    }
  } else {
    const std::vector<std::vector<double>> wegpunkte =
        auftrag.An("wegpunkte").Matrix();
    std::vector<std::string> namen;
    if (auftrag.Enthaelt("namen")) namen = auftrag.An("namen").Texte();
    if (namen.size() != wegpunkte.size()) {
      namen.clear();
      for (size_t i = 0; i < wegpunkte.size(); ++i) {
        namen.push_back(std::to_string(i));
      }
    }
    const std::vector<double> ueberschleifen =
        UeberschleifenFuer(u, wegpunkte.size(), "");
    std::vector<std::pair<int, int>> stuecke;
    if (auftrag.Enthaelt("abschnitte") &&
        auftrag.An("abschnitte").Groesse() > 0) {
      for (const Wert& st : auftrag.An("abschnitte").Elemente()) {
        stuecke.emplace_back(st.An(0).Ganzzahl(), st.An(1).Ganzzahl());
      }
    } else {
      stuecke.emplace_back(0, static_cast<int>(wegpunkte.size()) - 1);
    }
    for (const auto& [anfang, ende] : stuecke) {
      if (anfang < 0 || ende >= static_cast<int>(wegpunkte.size()) ||
          ende <= anfang) {
        throw std::runtime_error(absl::StrCat(
            "Abschnitt [", anfang, ", ", ende, "] ist ungueltig"));
      }
      Teil t;
      t.name = absl::StrCat(namen[anfang], " -> ", namen[ende]);
      t.punkte.assign(wegpunkte.begin() + anfang, wegpunkte.begin() + ende + 1);
      t.ueberschleifen.assign(ueberschleifen.begin() + anfang,
                              ueberschleifen.begin() + ende + 1);
      teile.push_back(std::move(t));
    }
  }

  if (teile.empty()) throw std::runtime_error("Keine Abschnitte im Auftrag");
  for (const Teil& t : teile) {
    if (t.punkte.size() < 2) {
      throw std::runtime_error(
          absl::StrCat("Abschnitt ", t.name, " hat weniger als zwei Punkte"));
    }
    for (size_t i = 0; i < t.punkte.size(); ++i) {
      if (t.punkte[i].size() != n) {
        throw std::runtime_error(absl::StrCat(
            "Abschnitt ", t.name, " Punkt ", i, " hat ", t.punkte[i].size(),
            " Werte, erwartet ", n));
      }
    }
  }
  return teile;
}

// ------------------------------------------------------ Modus "toppra"

// Was toppra und toppra_proben gemeinsam lesen und pruefen. Beide muessen
// aus demselben Auftrag dieselben Proben bauen.
struct ToppraEingang {
  std::vector<std::string> gelenke;
  std::vector<Teil> teile;
  JointLimits grenzen;
  double v_skalierung = 1.0;
  double a_skalierung = 1.0;
  double resample_dt = 0.0;
};

ToppraEingang LiesToppraEingang(const Wert& auftrag) {
  const std::vector<std::string> gelenke = auftrag.An("gelenke").Texte();
  const size_t n = gelenke.size();
  if (n == 0) throw std::runtime_error("gelenke ist leer");

  const std::vector<Teil> teile = LiesTeile(auftrag, n);

  const double v_skalierung = auftrag.An("velocity_scaling").Zahl();
  const double a_skalierung = auftrag.An("acceleration_scaling").Zahl();
  if (!(v_skalierung > 0.0 && v_skalierung <= 1.0) ||
      !(a_skalierung > 0.0 && a_skalierung <= 1.0)) {
    throw std::runtime_error("velocity_scaling und acceleration_scaling "
                             "muessen in (0, 1] liegen");
  }
  const double resample_dt = auftrag.An("resample_dt").Zahl();
  if (!(resample_dt > 0.0)) {
    throw std::runtime_error("resample_dt muss > 0 sein");
  }

  // Grenzen: explizit im Auftrag, Positionen aus der URDF, v und a aus
  // joint_limits.yaml. Ruck und Moment werden nicht benutzt und stehen auf
  // unendlich (von Intrinsic gefordert, aber ohne Wirkung).
  const Wert& g = auftrag.An("grenzen");
  const std::vector<double> p_min = Zahlenliste(g, "min_position", n);
  const std::vector<double> p_max = Zahlenliste(g, "max_position", n);
  const std::vector<double> v_max = Zahlenliste(g, "max_velocity", n);
  const std::vector<double> a_max = Zahlenliste(g, "max_acceleration", n);

  auto unbegrenzt = JointLimits::Unlimited(n);
  if (!unbegrenzt.ok()) {
    throw std::runtime_error(std::string(unbegrenzt.status().message()));
  }
  JointLimits grenzen = unbegrenzt.value();
  grenzen.min_position = ZuVektor(p_min);
  grenzen.max_position = ZuVektor(p_max);
  grenzen.max_velocity = ZuVektor(v_max) * v_skalierung;
  grenzen.max_acceleration = ZuVektor(a_max) * a_skalierung;
  if (!grenzen.IsValid()) {
    throw std::runtime_error(
        "grenzen ungueltig (min > max oder negative Werte)");
  }
  for (size_t k = 0; k < n; ++k) {
    if (!(v_max[k] > 0.0) || !(a_max[k] > 0.0)) {
      throw std::runtime_error(absl::StrCat(
          "Achse ", gelenke[k],
          ": max_velocity und max_acceleration muessen > 0 sein"));
    }
  }
  for (const Teil& t : teile) {
    for (size_t i = 0; i < t.punkte.size(); ++i) {
      for (size_t k = 0; k < n; ++k) {
        if (t.punkte[i][k] < p_min[k] || t.punkte[i][k] > p_max[k]) {
          throw std::runtime_error(absl::StrCat(
              "Abschnitt ", t.name, " Punkt ", i, " Achse ", gelenke[k],
              " = ", t.punkte[i][k], " liegt ausserhalb [", p_min[k], ", ",
              p_max[k], "]"));
        }
      }
    }
    if (Pfadlaenge(t.punkte) < kMinAbschnittLaengeRad) {
      throw std::runtime_error(absl::StrCat(
          "Abschnitt ", t.name, " hat keine Bewegung (Laenge < ",
          kMinAbschnittLaengeRad, " rad)"));
    }
  }

  ToppraEingang eingang;
  eingang.gelenke = gelenke;
  eingang.teile = teile;
  eingang.grenzen = grenzen;
  eingang.v_skalierung = v_skalierung;
  eingang.a_skalierung = a_skalierung;
  eingang.resample_dt = resample_dt;
  return eingang;
}

// Eine Grenze > 0 aus dem Auftrag, 0 wenn der Schluessel fehlt.
double PositivOderNull(const Wert& auftrag, const std::string& k,
                       const char* einheit) {
  if (!auftrag.Enthaelt(k)) return 0.0;
  const double w = auftrag.An(k).Zahl();
  if (!std::isfinite(w) || !(w > 0.0)) {
    throw std::runtime_error(
        absl::StrCat(k, " muss eine Zahl > 0 sein (", einheit, ")"));
  }
  return w;
}

// Bahngrenzen aus dem Auftrag, je Teil eine. Leer ohne tcp_v_max und
// tcp_a_max.
std::vector<TcpGrenze> LiesTcpGrenzen(const Wert& auftrag,
                                      const ToppraEingang& eingang) {
  std::vector<TcpGrenze> grenzen;
  const double v = PositivOderNull(auftrag, "tcp_v_max", "m/s");
  const double a = PositivOderNull(auftrag, "tcp_a_max", "m/s^2");
  if (v == 0.0 && a == 0.0) return grenzen;
  const size_t teile = eingang.teile.size();
  const std::vector<double> summen = auftrag.An("tcp_pruefsummen").Zahlen();
  const Wert& x = auftrag.An("tcp_x");
  const Wert& xp = auftrag.An("tcp_xp");
  const Wert& xpp = auftrag.An("tcp_xpp");
  if (summen.size() != teile || x.Groesse() != teile ||
      xp.Groesse() != teile || xpp.Groesse() != teile) {
    throw std::runtime_error(absl::StrCat(
        "tcp_x, tcp_xp, tcp_xpp und tcp_pruefsummen brauchen je Abschnitt "
        "einen Eintrag (", teile, ")"));
  }
  if (a > 0.0 && eingang.gelenke.size() + kTcpRichtungenAnzahl >
                     static_cast<size_t>(
                         intrinsic::eigenmath::MAX_EIGEN_VECTOR_SIZE)) {
    throw std::runtime_error(absl::StrCat(
        "tcp_a_max braucht ", kTcpRichtungenAnzahl, " Zusatzachsen; mit ",
        eingang.gelenke.size(), " Achsen ueber dem Hoechstwert von Intrinsic (",
        intrinsic::eigenmath::MAX_EIGEN_VECTOR_SIZE, ")"));
  }
  for (size_t i = 0; i < teile; ++i) {
    TcpGrenze g;
    // Der Override gilt fuer die Bahn wie fuer die Achsen.
    g.v_max = v * eingang.v_skalierung;
    g.a_max = a * eingang.a_skalierung;
    g.x = x.An(i).Matrix();
    g.xp = xp.An(i).Matrix();
    g.xpp = xpp.An(i).Matrix();
    g.pruefsumme = summen[i];
    grenzen.push_back(std::move(g));
  }
  return grenzen;
}

Wert RechneToppra(const Wert& auftrag) {
  const ToppraEingang gelesen = LiesToppraEingang(auftrag);
  const std::vector<std::string>& gelenke = gelesen.gelenke;
  const std::vector<Teil>& teile = gelesen.teile;
  const JointLimits& grenzen = gelesen.grenzen;
  const double v_skalierung = gelesen.v_skalierung;
  const double a_skalierung = gelesen.a_skalierung;
  const double resample_dt = gelesen.resample_dt;
  const size_t n = gelenke.size();
  const std::vector<TcpGrenze> tcp_grenzen = LiesTcpGrenzen(auftrag, gelesen);

  const bool roh_ausgeben = BoolOder(auftrag, "roh_ausgeben", false);

  Wert punkte = Wert::Liste();
  Wert beschleunigungen = Wert::Liste();
  Wert teilbahnen = Wert::Liste();
  Wert abschnitte_info = Wert::Liste();
  Wert roh_alle = Wert::Liste();
  Wert hinweise = Wert::Liste();
  Pruefung pruefung_roh;
  Pruefung pruefung_abgetastet;
  double versatz = 0.0;
  double letzte_zeit = 0.0;
  double rechenzeit = 0.0;
  double groesster_uebergang = 0.0;
  size_t eingang = 0;

  for (size_t ti = 0; ti < teile.size(); ++ti) {
    const Teil& t = teile[ti];
    eingang += t.punkte.size();

    // Uebergang zum vorigen Teil. Bei wegpunkte+abschnitte ist er null, bei
    // teilpfaden liegen Ende und Anfang zwei getrennter IK-Ketten, die
    // denselben Wegpunkt treffen sollen. Der erste Punkt faellt unten weg;
    // wie weit er vom letzten des vorigen liegt, wird gemeldet.
    if (ti > 0) {
      const std::vector<double>& a = teile[ti - 1].punkte.back();
      const std::vector<double>& b = t.punkte.front();
      double sprung = 0.0;
      for (size_t k = 0; k < n; ++k) {
        sprung = std::max(sprung, std::abs(a[k] - b[k]));
      }
      groesster_uebergang = std::max(groesster_uebergang, sprung);
      if (sprung > 0.0) {
        hinweise.Anhaengen(absl::StrCat(
            "Uebergang vor ", t.name, ": Endpunkt des vorigen und Startpunkt "
            "dieses Abschnitts weichen um ", sprung,
            " rad (groesste Achse) ab; der Startpunkt entfaellt beim "
            "Verketten"));
      }
    }

    AbschnittErgebnis e = RechneAbschnitt(
        t.punkte, t.ueberschleifen, grenzen, t.name,
        tcp_grenzen.empty() ? nullptr : &tcp_grenzen[ti]);
    rechenzeit += e.rechenzeit_s;
    for (const std::string& h : e.hinweise) hinweise.Anhaengen(h);

    // Pruefung gegen die Grenzen des Auftrags (skaliert, ohne Kappung und
    // ohne Sicherheitsfaktor).
    Pruefung pr_teil;
    Wert roh_teil = Wert::Liste();
    for (size_t i = 0; i < e.roh.size(); ++i) {
      const JointStatePVA& zustand = e.roh.data()[i];
      pruefung_roh.Nimm(zustand.velocity, zustand.acceleration, grenzen);
      pr_teil.Nimm(zustand.velocity, zustand.acceleration, grenzen);
      if (roh_ausgeben) {
        Wert r = Wert::Liste();
        r.Anhaengen(Sekunden(e.roh.time_stamps()[i]));
        r.Anhaengen(ZuListe(zustand.position));
        r.Anhaengen(ZuListe(zustand.velocity));
        r.Anhaengen(ZuListe(zustand.acceleration));
        roh_teil.Anhaengen(std::move(r));
      }
    }
    if (!pr_teil.Eingehalten()) {
      hinweise.Anhaengen(absl::StrCat(
          "Abschnitt ", t.name, ": TOPP-RA-Ausgabe verletzt Grenzen (v-Faktor ",
          pr_teil.v_faktor_max, ", a-Faktor ", pr_teil.a_faktor_max,
          ") -- best-effort-Ergebnis von Intrinsic, nicht korrigiert"));
    }

    const auto abgetastet = Abtasten(e.roh, resample_dt, t.name);
    int uebernommen = 0;
    for (size_t i = 0; i < abgetastet.size(); ++i) {
      const auto& [zeit, zustand] = abgetastet[i];
      pruefung_abgetastet.Nimm(zustand.velocity, zustand.acceleration,
                               grenzen);
      // Erster Punkt eines Folgeabschnitts = letzter des vorigen (gleicher
      // Ort, Stillstand) -- faellt weg, wie in totg_rechner.verketten().
      if (punkte.Groesse() > 0 && i == 0) continue;
      punkte.Anhaengen(
          Punkt(zeit + versatz, zustand.position, zustand.velocity));
      beschleunigungen.Anhaengen(ZuListe(zustand.acceleration));
      letzte_zeit = zeit + versatz;
      ++uebernommen;
    }
    versatz = letzte_zeit;
    teilbahnen.Anhaengen(uebernommen);

    Wert info = Wert::Objekt();
    info.Setzen("name", t.name);
    info.Setzen("wegpunkte", t.punkte.size());
    info.Setzen("pfadpunkte", e.pfadpunkte);
    info.Setzen("rohpunkte", e.roh.size());
    info.Setzen("dauer_s", Sekunden(e.roh.Duration()));
    info.Setzen("toppra_iterationen", e.iterationen);
    info.Setzen("rechenzeit_s", e.rechenzeit_s);
    info.Setzen("grenzpruefung_roh", pr_teil.AlsJson());
    if (e.tcp_massgeblich >= 0) {
      info.Setzen("tcp_massgeblich_proben", e.tcp_massgeblich);
    }
    abschnitte_info.Anhaengen(std::move(info));
    if (roh_ausgeben) roh_alle.Anhaengen(std::move(roh_teil));
  }

  Wert herkunft = Wert::Objekt();
  herkunft.Setzen("rechnung",
                  "Intrinsic Core (Pfadglaettung, TOPP-RA, Interpolation), "
                  "unveraendert aufgerufen");
  herkunft.Setzen("wrapper",
                  "eigener Code (Auftrag, Skalierung, Abschnitte, Abtastung, "
                  "Grenzpruefung, Protokoll, JSON)");
  herkunft.Setzen(
      "nachgebildet",
      std::vector<std::string>{
          "Beschleunigungskappung auf kMaxJointAccelerationToppRa",
          "Ueberschleifen-Minimum an Start und Ende"});
  if (!tcp_grenzen.empty()) {
    herkunft.Setzen("tcp_grenze",
                    "eigener Code: |dx/ds| aus dem Auftrag als "
                    "Gelenkgeschwindigkeitsgrenze je Pfadprobe; "
                    "Bahnbeschleunigung ueber 13 Zusatzachsen d_j . x");
  }

  Wert pruefung = Wert::Objekt();
  pruefung.Setzen("roh", pruefung_roh.AlsJson());
  pruefung.Setzen("abgetastet", pruefung_abgetastet.AlsJson());

  Wert vorgaben = Wert::Objekt();
  if (auftrag.An("ueberschleifen_rad").IstZahl()) {
    vorgaben.Setzen("ueberschleifen_rad",
                    auftrag.An("ueberschleifen_rad").Zahl());
  } else {
    vorgaben.Setzen("ueberschleifen_rad", "je Punkt, siehe Auftrag");
  }
  vorgaben.Setzen("velocity_scaling", v_skalierung);
  vorgaben.Setzen("acceleration_scaling", a_skalierung);
  vorgaben.Setzen("resample_dt", resample_dt);
  if (!tcp_grenzen.empty()) {
    const TcpGrenze& g = tcp_grenzen.front();
    if (g.v_max > 0.0) {
      vorgaben.Setzen("tcp_v_max", auftrag.An("tcp_v_max").Zahl());
      vorgaben.Setzen("tcp_v_max_wirksam", g.v_max);
    }
    if (g.a_max > 0.0) {
      vorgaben.Setzen("tcp_a_max", auftrag.An("tcp_a_max").Zahl());
      vorgaben.Setzen("tcp_a_max_wirksam", g.a_max);
      vorgaben.Setzen("tcp_a_je_richtung",
                      std::min(g.a_max * kTcpPolyederFaktor,
                               intrinsic::topp::kMaxJointAccelerationToppRa));
      vorgaben.Setzen("tcp_a_polyeder_faktor", kTcpPolyederFaktor);
    }
  }

  Wert fest = Wert::Objekt();
  fest.Setzen("sicherheitsfaktor", kIntrinsicSicherheitsfaktor);
  fest.Setzen("max_iterationen", kIntrinsicMaxIterationen);
  fest.Setzen("a_max_kappe_rad_s2",
              intrinsic::topp::kMaxJointAccelerationToppRa);
  fest.Setzen("spline_grad", kIntrinsicSplineGrad);
  fest.Setzen("eckabstand_fuer_ueberschleifen_rad", kIntrinsicEckabstandRad);
  fest.Setzen("ruck_begrenzt", false);
  fest.Setzen("interpolation", "kubisch, stetige Geschwindigkeit");

  Wert antwort = Wert::Objekt();
  antwort.Setzen("ok", true);
  antwort.Setzen("rechner", "intrinsic_toppra_wrapper");
  antwort.Setzen("modus", "toppra");
  antwort.Setzen("herkunft", std::move(herkunft));
  antwort.Setzen("stand", kStand);
  antwort.Setzen("intrinsic_release", kIntrinsicRelease);
  antwort.Setzen("gelenke", gelenke);
  antwort.Setzen("punkte", std::move(punkte));
  antwort.Setzen("beschleunigungen", std::move(beschleunigungen));
  antwort.Setzen("dauer_s", letzte_zeit);
  antwort.Setzen("teilbahnen", std::move(teilbahnen));
  antwort.Setzen("eingang", eingang);
  antwort.Setzen("rechenzeit_s", rechenzeit);
  antwort.Setzen("uebergang_max_rad", groesster_uebergang);
  antwort.Setzen("abschnitte", std::move(abschnitte_info));
  antwort.Setzen("grenzpruefung", std::move(pruefung));
  antwort.Setzen("hinweise", std::move(hinweise));
  antwort.Setzen("vorgaben", std::move(vorgaben));
  antwort.Setzen("feste_werte_intrinsic", std::move(fest));
  if (roh_ausgeben) antwort.Setzen("roh", std::move(roh_alle));
  return antwort;
}

// ------------------------------------------------ Modus "toppra_proben"
//
// Derselbe Auftrag wie toppra, aber nur die Glaettung. Je Abschnitt die
// Proben q(s) und dq/ds, wie TOPP-RA sie bekaeme, und ihre Pruefsumme. Der
// Aufrufer rechnet daran |dx/ds| und gibt es mit tcp_v_max an toppra.

Wert RechneToppraProben(const Wert& auftrag) {
  const ToppraEingang eingang = LiesToppraEingang(auftrag);
  Wert teile = Wert::Liste();
  for (const Teil& t : eingang.teile) {
    // Dieselbe Glaettung wie in RechneAbschnitt. Die Beschleunigungs-
    // kappung dort aendert nur Grenzen, nicht q oder dq/ds.
    const std::vector<intrinsic::topp::PathSample> proben =
        Glaetten(t.punkte, t.ueberschleifen, eingang.grenzen, t.name);
    Wert q = Wert::Liste();
    Wert qp = Wert::Liste();
    Wert qpp = Wert::Liste();
    for (const intrinsic::topp::PathSample& probe : proben) {
      q.Anhaengen(ZuListe(probe.q));
      qp.Anhaengen(ZuListe(probe.qp));
      qpp.Anhaengen(ZuListe(probe.qpp));
    }
    Wert teil = Wert::Objekt();
    teil.Setzen("name", t.name);
    teil.Setzen("q", std::move(q));
    teil.Setzen("qp", std::move(qp));
    teil.Setzen("qpp", std::move(qpp));
    teil.Setzen("pruefsumme", Pruefsumme(proben));
    teile.Anhaengen(std::move(teil));
  }
  Wert antwort = Wert::Objekt();
  antwort.Setzen("ok", true);
  antwort.Setzen("rechner", "intrinsic_toppra_wrapper");
  antwort.Setzen("modus", "toppra_proben");
  antwort.Setzen("stand", kStand);
  antwort.Setzen("intrinsic_release", kIntrinsicRelease);
  antwort.Setzen("gelenke", eingang.gelenke);
  antwort.Setzen("teilpfade", std::move(teile));
  return antwort;
}

// ------------------------------------------------ Modi fuer die LIN-Kette
//
// Nachgebaut ist der Ablauf von LinearCartesianMotionPathPlanner und
// PathIkOnCartesianCurveAndResampleForUniformJointSampling() in Intrinsic,
// mit einem Unterschied: die Rueckwaertskinematik rechnet nicht Intrinsic,
// sondern MoveIt, im aufrufenden Prozess (intrinsic_lin_rechner.py). Der
// Wrapper liefert dafuer die Posen und nimmt die Gelenkloesungen zurueck:
//
//   lin_kurve          Posen der Eckpunkte -> verrundete kartesische Kurve,
//                      gleichmaessig abgetastet
//   (MoveIt-IK an allen Posen)
//   lin_neuabtastung   Kurvenparameter + IK-Loesungen -> neue, im Gelenkraum
//                      etwa gleichabstaendige Kurvenparameter, zehnfach
//                      verdichtet, mit den Posen dazu
//   (MoveIt-IK an den dichten Posen, Auswahl der behaltenen Indizes)
//   toppra             dichte Gelenkpfade als teilpfade

// Pose als [x, y, z, qx, qy, qz, qw].
intrinsic::Pose3d ZuPose(const std::vector<double>& p, size_t index) {
  if (p.size() != 7) {
    throw std::runtime_error(absl::StrCat(
        "Pose ", index, " hat ", p.size(),
        " Werte, erwartet 7 (x, y, z, qx, qy, qz, qw)"));
  }
  const double norm = std::sqrt(p[3] * p[3] + p[4] * p[4] + p[5] * p[5] +
                                p[6] * p[6]);
  // Normiert wird im Pose-Konstruktor von Intrinsic. Weicht die Norm merklich
  // von 1 ab, ist das kein Rundungsrest, sondern ein falscher Eingang.
  if (std::abs(norm - 1.0) > 1e-6) {
    throw std::runtime_error(absl::StrCat(
        "Pose ", index, ": Quaternion hat die Norm ", norm, ", erwartet 1"));
  }
  return intrinsic::Pose3d(
      intrinsic::eigenmath::Quaterniond(p[6], p[3], p[4], p[5]),
      intrinsic::eigenmath::Vector3d(p[0], p[1], p[2]));
}

std::vector<double> AusPose(const intrinsic::Pose3d& pose) {
  const auto& t = pose.translation();
  const auto& q = pose.quaternion();
  return {t.x(), t.y(), t.z(), q.x(), q.y(), q.z(), q.w()};
}

Wert PosenListe(const std::vector<intrinsic::Pose3d>& posen) {
  Wert liste = Wert::Liste();
  for (const intrinsic::Pose3d& p : posen) liste.Anhaengen(AusPose(p));
  return liste;
}

// Die verrundete Kurve und ihr Abtastschritt. Beide LIN-Modi bauen sie aus
// denselben Eingaben neu auf; gleiche Eingaben ergeben dieselbe Kurve.
struct LinKurve {
  std::unique_ptr<intrinsic::CartesianPolylineBSplinesCurve> kurve;
  intrinsic::CartesianCurveLength laenge;
  double schritt_translation_m = 0.0;
  double schritt_rotation_rad = 0.0;
  double pfad_ik_frequenz_hz = 0.0;
  double normierter_schritt = 0.0;
};

LinKurve BaueKurve(const Wert& auftrag, Wert& hinweise) {
  const std::vector<std::vector<double>> roh = auftrag.An("posen").Matrix();
  if (roh.size() < 2) {
    throw std::runtime_error("Mindestens zwei Posen noetig");
  }
  std::vector<intrinsic::Pose3d> posen;
  for (size_t i = 0; i < roh.size(); ++i) posen.push_back(ZuPose(roh[i], i));

  const double rundung_t = auftrag.An("rundung_translation_m").Zahl();
  const double rundung_r = auftrag.An("rundung_rotation_rad").Zahl();
  const double abtast_t = auftrag.An("abtastung_translation_m").Zahl();
  const double abtast_r = auftrag.An("abtastung_rotation_rad").Zahl();
  if (!(rundung_t > 0.0) || !(rundung_r > 0.0)) {
    throw std::runtime_error("rundung_translation_m und rundung_rotation_rad "
                             "muessen > 0 sein (Forderung von Intrinsic)");
  }
  // [NACHGEBILDET] Der Motion Planner von Intrinsic hebt kleinere Werte auf
  // 1 mm bzw. 0,01 rad an (motion_planner.cc). Hier wird nicht angehoben,
  // nur gemeldet.
  if (rundung_t < 0.001) {
    hinweise.Anhaengen(absl::StrCat(
        "rundung_translation_m = ", rundung_t, " liegt unter dem Minimum des "
        "Intrinsic Motion Planners (0.001); nicht angehoben"));
  }
  if (rundung_r < 0.01) {
    hinweise.Anhaengen(absl::StrCat(
        "rundung_rotation_rad = ", rundung_r, " liegt unter dem Minimum des "
        "Intrinsic Motion Planners (0.01); nicht angehoben"));
  }

  LinKurve k;
  // [INTRINSIC] Kartesische Kurve: B-Splines 4. Grades fuer Translation und
  // Rotation ueber dem Polygon der Posen, Ecken verrundet.
  auto kurve = intrinsic::CartesianPolylineBSplinesCurve::Create(
      /*rotational_blending_radius=*/rundung_r,
      /*translational_blending_radius=*/rundung_t,
      /*pose_waypoints=*/absl::MakeConstSpan(posen));
  if (!kurve.ok()) {
    throw std::runtime_error(
        absl::StrCat("Kurve: ", kurve.status().ToString()));
  }
  k.kurve = std::move(kurve).value();
  k.laenge = k.kurve->GetCurveLength();

  // [INTRINSIC] Abtastschritte und Frequenz, wie im LIN-Planer.
  auto schritte =
      intrinsic::ComputeCartesianCurveMinimalSamplingStepsAndControlFrequency(
          abtast_t, abtast_r, k.laenge);
  if (!schritte.ok()) {
    throw std::runtime_error(
        absl::StrCat("Abtastschritte: ", schritte.status().ToString()));
  }
  k.schritt_translation_m = schritte->sampling_step_translational_m;
  k.schritt_rotation_rad = schritte->sampling_step_rotational_rad;
  k.pfad_ik_frequenz_hz = schritte->path_ik_control_frequency_hz;

  // [INTRINSIC] Normierter Schritt auf dem Kurvenparameter [0, 1].
  auto normiert = intrinsic::NormalizedSamplingStepForCartesianCurve(
      k.schritt_translation_m, k.schritt_rotation_rad, k.laenge);
  if (!normiert.ok()) {
    throw std::runtime_error(
        absl::StrCat("Normierter Schritt: ", normiert.status().ToString()));
  }
  k.normierter_schritt = *normiert;
  return k;
}

Wert KurvenInfo(const LinKurve& k) {
  Wert info = Wert::Objekt();
  info.Setzen("laenge_translation_m", k.laenge.translation_m);
  info.Setzen("laenge_rotation_rad", k.laenge.rotation_rad);
  info.Setzen("abtastschritt_translation_m", k.schritt_translation_m);
  info.Setzen("abtastschritt_rotation_rad", k.schritt_rotation_rad);
  info.Setzen("normierter_schritt", k.normierter_schritt);
  info.Setzen("pfad_ik_frequenz_hz_intrinsic", k.pfad_ik_frequenz_hz);
  return info;
}

Wert AntwortKopf(const char* modus) {
  Wert antwort = Wert::Objekt();
  antwort.Setzen("ok", true);
  antwort.Setzen("rechner", "intrinsic_toppra_wrapper");
  antwort.Setzen("modus", modus);
  Wert herkunft = Wert::Objekt();
  herkunft.Setzen("rechnung",
                  "Intrinsic Core (CartesianPolylineBSplinesCurve, "
                  "Abtastschritte, Gelenkraum-Neuabtastung), unveraendert "
                  "aufgerufen");
  herkunft.Setzen("wrapper", "eigener Code (Auftrag, Pruefung, JSON)");
  herkunft.Setzen("ik", "nicht hier -- MoveIt im aufrufenden Prozess");
  antwort.Setzen("herkunft", std::move(herkunft));
  antwort.Setzen("stand", kStand);
  antwort.Setzen("intrinsic_release", kIntrinsicRelease);
  return antwort;
}

// ----------------------------------------------------- Modus "lin_kurve"

Wert RechneLinKurve(const Wert& auftrag) {
  Wert hinweise = Wert::Liste();
  const LinKurve k = BaueKurve(auftrag, hinweise);

  // [INTRINSIC] Gleichmaessig abtasten, ohne erzwungene ungerade Anzahl --
  // wie LinearCartesianMotionPathPlanner (force_odd_number_of_samples=false).
  auto abgetastet = k.kurve->SampleUniformly(k.normierter_schritt,
                                             /*force_odd_number_of_samples=*/
                                             false);
  if (!abgetastet.ok()) {
    throw std::runtime_error(
        absl::StrCat("Abtastung: ", abgetastet.status().ToString()));
  }

  Wert antwort = AntwortKopf("lin_kurve");
  antwort.Setzen("kurvenparameter",
                 abgetastet->normalized_curve_parameters);
  antwort.Setzen("posen", PosenListe(abgetastet->poses));
  antwort.Setzen("kurve", KurvenInfo(k));
  antwort.Setzen("hinweise", std::move(hinweise));
  return antwort;
}

// ----------------------------------------------- Modus "lin_neuabtastung"

// Aufwaertstastung wie in PathIkOnCartesianCurveAndResampleForUniform-
// JointSampling() (dort kUpsamplingFactor, lokal und nicht lesbar).
constexpr int kIntrinsicAufwaertsFaktor = 10;

Wert RechneLinNeuabtastung(const Wert& auftrag) {
  Wert hinweise = Wert::Liste();
  const LinKurve k = BaueKurve(auftrag, hinweise);

  const std::vector<double> parameter = auftrag.An("kurvenparameter").Zahlen();
  const std::vector<std::vector<double>> pfad =
      auftrag.An("gelenkpfad").Matrix();
  if (pfad.size() != parameter.size()) {
    throw std::runtime_error(absl::StrCat(
        "gelenkpfad hat ", pfad.size(), " Punkte, kurvenparameter ",
        parameter.size()));
  }
  std::vector<VectorNd> loesungen;
  for (const auto& p : pfad) loesungen.push_back(ZuVektor(p));

  const double abstand = auftrag.An("gelenk_abtastung_rad").Zahl();
  if (!(abstand > 0.0)) {
    throw std::runtime_error("gelenk_abtastung_rad muss > 0 sein");
  }

  // [INTRINSIC] Abtastschritt im Gelenkraum, wie im LIN-Planer ueber
  // CreateUniformResamplingStepsForJointSpacePathFunction(). Die Optionen
  // muessen den Aufruf ueberleben: die Funktion haelt sie per Referenz.
  intrinsic::JointSpaceUniformArcLengthPreservingSamplingOptions optionen;
  optionen.sampling_distance_rad = abstand;
  const intrinsic::UniformResamplingStepsForJointSpacePathFunction
      schritte_fn =
          intrinsic::CreateUniformResamplingStepsForJointSpacePathFunction(
              optionen);
  auto schritte = schritte_fn(absl::MakeConstSpan(loesungen));
  if (!schritte.ok()) {
    throw std::runtime_error(
        absl::StrCat("Gelenkschritt: ", schritte.status().ToString()));
  }
  if (!schritte->has_value()) {
    throw std::runtime_error("Intrinsic lieferte keinen Gelenkschritt");
  }

  // [INTRINSIC] Neue Kurvenparameter, die im Gelenkraum etwa gleich weit
  // auseinanderliegen (Standard: ungerade Anzahl erzwungen).
  auto neu = intrinsic::ApproximateJointUniformSampling(
      absl::MakeConstSpan(parameter), absl::MakeConstSpan(loesungen),
      absl::MakeConstSpan(**schritte));
  if (!neu.ok()) {
    throw std::runtime_error(
        absl::StrCat("Gleichabstaendige Abtastung: ", neu.status().ToString()));
  }

  // [INTRINSIC] Zehnfach verdichten; die IK laeuft ueber die dichten
  // Parameter, behalten werden nur die urspruenglichen.
  auto dicht = intrinsic::CreateUpsampledCurveParameters(
      absl::MakeConstSpan(*neu), kIntrinsicAufwaertsFaktor);
  if (!dicht.ok()) {
    throw std::runtime_error(
        absl::StrCat("Verdichtung: ", dicht.status().ToString()));
  }

  // [INTRINSIC] Posen an den dichten Parametern.
  auto posen = k.kurve->Sample(absl::MakeConstSpan(dicht->dense_curve_parameters));
  if (!posen.ok()) {
    throw std::runtime_error(
        absl::StrCat("Posen: ", posen.status().ToString()));
  }

  Wert indizes = Wert::Liste();
  for (int i : dicht->sparse_curve_parameters_indices_in_dense_vector) {
    indizes.Anhaengen(i);
  }

  Wert antwort = AntwortKopf("lin_neuabtastung");
  antwort.Setzen("gelenk_abtastschritt_rad", (**schritte).front());
  antwort.Setzen("kurvenparameter", *neu);
  antwort.Setzen("dichte_kurvenparameter", dicht->dense_curve_parameters);
  antwort.Setzen("posen", PosenListe(*posen));
  antwort.Setzen("behalten_indizes", std::move(indizes));
  antwort.Setzen("aufwaertsfaktor", kIntrinsicAufwaertsFaktor);
  antwort.Setzen("kurve", KurvenInfo(k));
  antwort.Setzen("hinweise", std::move(hinweise));
  return antwort;
}

// ------------------------------------------------------------- Verteiler

Wert Rechne(const Wert& auftrag) {
  const int stand = GanzzahlOder(auftrag, "stand", 1);
  if (stand > kStand) {
    throw std::runtime_error(absl::StrCat(
        "Auftrag hat Stand ", stand, ", intrinsic_toppra_wrapper kennt nur ",
        kStand, ". Wrapper neu bauen."));
  }
  const std::string modus =
      auftrag.Enthaelt("modus") ? auftrag.An("modus").Text() : "toppra";
  if (modus == "toppra") return RechneToppra(auftrag);
  if (modus == "toppra_proben") return RechneToppraProben(auftrag);
  if (modus == "lin_kurve") return RechneLinKurve(auftrag);
  if (modus == "lin_neuabtastung") return RechneLinNeuabtastung(auftrag);
  throw std::runtime_error(
      "Unbekannter modus: " + modus +
      " (toppra, toppra_proben, lin_kurve, lin_neuabtastung)");
}

void SchreibeAntwort(const std::string& pfad, const Wert& daten) {
  const std::filesystem::path ziel = std::filesystem::absolute(pfad);
  const std::filesystem::path zwischen =
      ziel.parent_path() / absl::StrCat(".antwort_", getpid(), ".tmp");
  {
    std::ofstream datei(zwischen);
    if (!datei) throw std::runtime_error("Antwort nicht schreibbar: " + pfad);
    datei << daten.Schreiben() << "\n";
  }
  std::filesystem::rename(zwischen, ziel);
}

}  // namespace

int main(int argc, char** argv) {
  if (argc == 2 && std::string(argv[1]) == "--pruefen") {
    std::cout << "intrinsic_toppra_wrapper Stand " << kStand << ", intrinsic-core "
              << kIntrinsicRelease << "\nok\n";
    return 0;
  }
  if (argc != 3) {
    std::cerr << "Aufruf: intrinsic_toppra_wrapper AUFTRAG.json ANTWORT.json\n"
              << "        intrinsic_toppra_wrapper --pruefen\n";
    return 2;
  }
  Wert antwort;
  bool ok = false;
  try {
    std::ifstream datei(argv[1]);
    if (!datei) {
      throw std::runtime_error(std::string("Auftrag fehlt: ") + argv[1]);
    }
    std::stringstream inhalt;
    inhalt << datei.rdbuf();
    const Wert auftrag = json_klein::Lies(inhalt.str());
    antwort = Rechne(auftrag);
    ok = true;
  } catch (const std::exception& fehler) {
    std::cerr << "Fehler: " << fehler.what() << "\n";
    antwort = Wert::Objekt();
    antwort.Setzen("ok", false);
    antwort.Setzen("fehler", std::string(fehler.what()));
  }
  try {
    SchreibeAntwort(argv[2], antwort);
  } catch (const std::exception& fehler) {
    std::cerr << "Fehler beim Schreiben: " << fehler.what() << "\n";
    return 1;
  }
  return ok ? 0 : 1;
}
