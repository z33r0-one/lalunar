// start_ks_v0929_midas_sim1r_flug49.ks — LaLunar Launch-Wrapper
// REFLIGHT von Flug 48 (Midas-SIM1). Core: lalunar_launch.ks (v0.9.29).
//
// ZWECK: Flug 48 lief auf dem alten Core (v0.9.22-24) und scheiterte an
// genau 3 Coast-Befunden (42./43. Sitzung, lalunar_launch_48.csv):
//   1. warpto war ein No-Op — beide COAST-Events trugen denselben
//      MET 486.7 s, es fand kein Zeitraffungs-Sprung statt.
//   2. Keine Zuend-Beweisfuehrung — kein IGNITION PROVEN Event, die
//      Transtage-Zuendung am Ap war nicht nachweisbar.
//   3. PE_PUSH drueckte RETROGRAD am Ap: Pe von -1047 km auf +2500 km
//      — falsche Richtung, das Manoever vernichtete orbitale Energie.
// Core v0.9.29 enthaelt die 3 Gegenmassnahmen:
//   1. warpToTime() — Rails-Warp mit gestuften Indizes bis Zielzeit
//   2. IGNITION PROVEN — Massenfluss-Beweis (ship:mass faellt um
//      IGN_PROOF_KG), Fehlen within IGN_MISS_WIN -> WARN + PE_PUSH blockiert
//   3. PE_PUSH prograd mit PPOFF_MAX-AoA-Deckel (core-default 15 deg)
// Dieser Flug validiert genau diese 3 Fixes. Craft und Wrapper-Profil
// sind identisch zu Flug 48 — EINE Variable: der Core.
//
// ERWARTUNGEN (Erfolgskriterien):
//   - COAST-Start und COAST-End im Log ~40 min Echtzeit auseinander
//   - Eigenstaendiger Warp-Rueckkehr aus Rails vor Zuendung
//   - EV IGNITION PROVEN im Log
//   - PE_PUSH prograd: Pe steigt monoton gegen 2500 km
//   - CUTOFF ~4000 x 2500 km, Inc ~90, Report im gruenen Bereich
//
// VERPASS-TEST (danach): GLEICHE Datei, nur LOG_NR 50 und
// MISSION_NAME "rlms-midas-sim2" — Zuendung am Ap UNTERLASSEN
// (Triebwerk nicht zuenden). Erwartung: WARN nach IGN_MISS_WIN,
// PE_PUSH blockiert, Safe-Hold. Dadurch verschiebt sich die alte
// Flug-49-Nummerierung um eins: 49 = dieser Reflight, 50 = Verpass.
//
// LOG-NUMMER: 49. Eine Log-Datei je Flug — LOG_NR 48 wuerde an die
// bereits analysierte lalunar_launch_48.csv anhaengen (kOS log() ist
// append). Deshalb neue Nummer.
//
// PARAMETER-QUELLEN:
//   log-bestaetigt (Flug-48-Log):  AZ 180 (Inc 89.72 erreicht),
//     PITCH_START 50 (EV "PITCH_START at 50 m/s"), STAGE_ACTIONS 2
//     (2x STAGE_CMD), HAS_FAIRING true (6 Fairings), TARGET_INC 90
//     (Fenster 85-95, erreicht 89.72), COAST_ENABLED true /
//     COAST_PRE_IGN 30 (41. Sitzung).
//   Werkbank-Defaults SL-1 (Mission Center v0.9.36, Generator):
//     PITCH_RATE 0.75, PITCH_FLOOR 30, EXO_PITCH_RATE 0 (aus),
//     EXO_PITCH_FLOOR 18, AP_LOFT_KM 260, VS_FLOOR 200,
//     FAIRING_SKIP_INTERSTAGE false, AIR_START_MET 0, SRM-Defaults.
//   Contract (Midas): 4000 x 2500 km polar.
//   EXO-Kalibrierung fuer dieses Profil: offen, unkritisch (42. Sitzung).

set MISSION_NAME   to "rlms-midas-sim1r".
set TARGET_AP_KM   to 4000.      // contract orbit
set TARGET_PE_KM   to 2500.
set TARGET_INC     to 90.        // deg, polar — Flug 48: erreicht 89.72
set INS_AP_KM      to 4000.      // insertion orbit = contract orbit
set INS_PE_KM      to 2500.
set LAUNCH_UT      to 0.         // UT seconds; 0 = launch NOW
set AZ             to 180.       // deg from north — Sued, in Flug 48 validiert
set PITCH_START    to 50.        // m/s surface speed — Flug-48-Log
set PITCH_RATE     to 0.75.      // deg/s — Werkbank-Default (v0.9.19)
set PITCH_FLOOR    to 30.        // v0.9.27: atmosphere pitch-program floor, deg
set EXO_PITCH_RATE to 0.         // 0 = exo program OFF (v0.9.27 behavior)
set EXO_PITCH_FLOOR to 18.       // nur relevant, wenn EXO_PITCH_RATE > 0
set Q_TRIGGER      to 10.        // kPa, pitch program -> guidance
set STAGE_ACTIONS  to 2.         // Flug-48-Log: 2 STAGE_CMD
set MANUAL_STAGING to false.
set ULLAGE_WAIT    to 2.         // s settle after stage command
set DEPLOY_Q_KPA   to 0.10.      // antenna/panel deploy threshold
set HAS_FAIRING    to true.      // Flug 48: 6 Fairings jettisoned
set AIR_START_MET  to 0.         // 0 = off — Bodenstart, keine SRM-Kette
set SRM_SEP_FRAC   to 0.5.       // Werkbank-Default (irrelevant, AIR_START 0)
set SRM_SEP_TIMEOUT to 40.       // Werkbank-Default (irrelevant, AIR_START 0)
set ROLL_MIN_MET   to 0.         // 0 = off
set PITCH_MIN_MET  to 0.         // 0 = off
set FAIRING_SKIP_INTERSTAGE to false.   // Werkbank-Default
set EXO_ALT_KM     to 70.       // guidance handover altitude
set ORB_TOL_KM     to 2.         // insertion tolerance
set THROTTLE_NEAR  to 0.40.      // terminal fine-trim only
set COAST_ENABLED  to true.      // DER Testgegenstand: Coast-Kette v0.9.29
set COAST_PRE_IGN  to 30.        // s vor Zuendung: RCS-Ausrichtfenster
set IGN_PROOF_KG   to 1.0.       // v0.9.29: Zuend-Beweis, kg Massenfluss
set IGN_MISS_WIN   to 30.        // v0.9.29: s past Ap ohne Beweis -> WARN
set AP_HOLD        to true.     // Ap am Ziel halten im Terminal
set AP_HOLD_GAIN   to 0.6.       // deg pro km Ap-Fehler
set AP_HOLD_MAX    to 45.        // deg, symmetrischer Deckel
set AP_LOFT_KM     to 260.      // Werkbank-Default; guide auto = 4000
set AP_GUIDE_KM    to -1.        // -1 = auto max(AP_LOFT, INS_AP) = 4000 km
set AP_CUT_KM      to -1.        // -1 = auto (AP_GUIDE + 500 km) = 4500 km
set VS_FLOOR       to 200.       // v0.9.26: climb VS schedule floor, m/s
set PLANE_HOLD     to true.     // Bahnebene halten ab Exo-Eintritt
set THROTTLEABLE   to false.    // Kette: full thrust, Praezision via Cutoff
set LOG_NR         to 49.       // log: lalunar_launch_49.csv
run lalunar_launch(true).
