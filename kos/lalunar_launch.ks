
// ============================================================================
// lalunar_launch.kos — LaLunar v0.9 core launch script
// English code per project convention.
// v0.9.29 — COAST-CHAIN REPAIR (flight-48 MIDAS-SIM1 debrief, Stefan's
// OK 30.10.): three fixes, all flight-48 findings. ML-1 non-coast
// flights are untouched — every new branch gates on COAST_ENABLED.
//   (1) RAILS WARP (finding 1) — warpto() is a NO-OP on kOS 1.6
//       (flight 48: both coast events logged the SAME MET 486.7; the
//       40-min coast ran real time, Stefan warped manually). New helper
//       warpToTime(): kuniverse:timewarp rails loop, warp index stepped
//       DOWN as the remaining seconds shrink, drop to real time at the
//       target. Also replaces the P0 launch-window warpto.
//   (2) IGNITION PROOF BY MASS FLOW (finding 2) — the old "wait until
//       availablethrust > 1" passed INSTANTLY: the engine reports
//       throttle-ready while ullage-blocked (no thrust, no mass flow),
//       so the WARN net never existed and terminal guidance + TVAL 1
//       engaged 40 min BEFORE the Ap (latent mission killer on any
//       ullage-capable engine). The script now takes throttle only
//       AFTER ship:mass drops by IGN_PROOF_KG (default 1 kg — SB-1
//       burns ~12 kg/s, RCS rotation is mass-neutral). Until proof:
//       orbital-prograde hold, WARN once at IGN_MISS_WIN (default
//       30 s) past the Ap, PE_PUSH blocked. Makes the Verpass-Test
//       (flight 49) valid for the first time.
//   (3) PROGRADE-PUSH BRANCH (finding 3, the mission killer). Root
//       cause, pinned during the fix build: the heading(azimuth, pitch)
//       frame is valid only near the launch site. Flight 48 flew a
//       POLAR orbit with a SOUTHERN azimuth (Stefan's correct
//       overwrite) — half an orbit later at the Ap the orbital
//       velocity is ANTIPARALLEL to the launch azimuth (south-launched
//       => north-flying at the Ap), so ANY heading(AZ, ..) command at
//       the Ap burns RETROGRADE by construction. FR: AoA ~100 deg
//       through the coast, 179.5 deg at the burn — after Stefan's
//       CORRECT manual ignition ~1.2 t went retrograde, Pe -1047 ->
//       -5044 km, Inc 89.72 -> 84.6 (revert). Second path in the same
//       envelope: vsref runs to the negative clamp when the vehicle is
//       above the guide altitude. Envelope guards, not cut criteria:
//       (3a) coast terminal steering (COAST_ENABLED + COAST_DONE +
//       alt > INS_PE) is built on the ORBITAL PROGRADE VECTOR with a
//       bounded AoA offset (PPOFF_MAX 15 deg — retrograde unreachable
//       by construction); (3b) vsref clamped to [0, VS_REF_MAX] while
//       above INS_PE — the push at the Ap HOLDS altitude and burns
//       horizontal. CONCEPT LESSON (the mirror image of the PVG dive):
//       a fixed control law does NOT stay valid outside its design
//       region — fixed laws need envelope guards, not just cut
//       criteria.
// File check: line 5 "v0.9.29", helpers contain "warpToTime", coast
// block contains "IGNITION PROVEN", P4 lock block contains "PPOFF_MAX",
// pepush law contains "hold-altitude clamp", HUD prints "LaLunar v0.9.29".
//
// v0.9.28 — EXO PITCH PROGRAM (ML-1 flight 43 debrief): above EXO_ALT
// the VS plan alone cannot hold the arc — VS_KP 0.02 caps the pitch
// offset at ~13 deg, so once the FPA decays the command follows
// prograde and the vehicle SAGS below its own Ap mid-burn (flight 43:
// VS 662 -> 205 by core-out at 115.7 km, then fell back into the
// atmosphere with S2 still burning). PVG instead holds a decaying
// ATTITUDE through this phase (pitch 33.7 at 150 s -> 24.9 at 310 s,
// ~0.05 deg/s, air-AoA +17-20 deg). NEW: at the VS-plan handover the
// exo program captures the atmosphere program command (pcmd) and
// decays it at EXO_PITCH_RATE toward EXO_PITCH_FLOOR; the pitch
// command is max(VS-plan pitch, exo program) while the lower stages
// burn — an attitude hold like PVG's, capped at EXO_AOA_CAP (20 deg)
// above prograde (PVG's own band). Gated off when PE_PUSH engages.
// Defaults keep the v0.9.27 behavior (EXO_PITCH_RATE 0 = off).
// File check: line 5 "v0.9.28", init contains "EXO_PITCH_RATE", VS-plan
// block contains "EXO_PITCH program engaged", HUD prints "LaLunar v0.9.28".
//
// v0.9.27 — ATMOSPHERE PITCH PROGRAM (ML-1 flight 40 debrief,
// Stefan's decision — "Kernaenderung zweitens").
//
// v0.9.27 (28.10., flight-40 debrief — every v0.9.26 mechanism fired
// correctly: VS_PLAN engaged 131.4 s at VS 1071.8, stage handover to
// PE_PUSH at 463.3 s, the VS law tracked its reference exactly (AoA
// -13.1 measured vs -13.4 computed). The flight STILL re-entered,
// and the FR data confirmed Stefan's hypothesis: the ATMOSPHERE
// phase is the bottleneck. The prograde lock (steer to
// surface velocity => AoA ~ 0) cannot flatten a steep arc —
// dFPA/dt = -g*cos(FPA)/v decays SLOWER at high FPA (cos feedback),
// so a steep early arc self-reinforces. Flight 40 vs PVG: pitch 78
// vs 53 deg at max-Q, SRM burnout FPA 63 vs 35 deg with VS 1071 vs
// ~590 m/s, core burnout 205 vs 97 km. The VS plan then engages at
// EXO_ALT with far too much vertical energy to work off (KP 0.02
// against core TWR 1.5-2.5).):
//   (1) ATMOSPHERE PITCH PROGRAM — below EXO_ALT the P4 loop now
//       CONTINUES the P3 pitch program instead of locking to
//       prograde: pcmd decays at PITCH_RATE down to PITCH_FLOOR
//       (wrapper param, default 30 deg), then the command is
//       clamped around the surface-prograde FPA with a
//       Q-dependent AoA cap: 3 deg while Q > 20 kPa, 6 deg while
//       Q > 2 kPa, free below (aero loads are gone by then). PVG
//       held air-AoA -2.3..+0.3 deg through max-Q, so a few deg
//       nose-below-prograde is aero-safe and bends the arc the
//       way PVG's shallower program does. The clamp cuts BOTH
//       ways: while the program is more aggressive than the cap
//       the vehicle flies prograde-minus-cap (bending down); once
//       the PITCH_FLOOR binds it flies prograde-plus-cap (keeps
//       some climb, prevents over-flattening). The handover from
//       P3 is a REDUCTION of AoA (P3 routinely flies ~-13 deg at
//       Q < 10 kPa where aero loads are tiny) — safe direction.
//       Prograde FPA from arcsin(verticalspeed/velocity:mag),
//       ratio clamped to [-1, 1] before arcsin (NaN guard).
//   (2) VS plan, PE_PUSH handover, stage gate: UNCHANGED (proven
//       in flight 40 — the law itself tracked perfectly; it was
//       handed 1071 m/s of vertical energy it could not spend).
//       With the flatter atmosphere arc it should engage at
//       ~600 m/s instead.
// File check: line 5 "v0.9.27", atmosphere block contains
// "ATM_PROGRAM" and "PITCH_FLOOR", climb block contains "VS_FLOOR",
// PE_PUSH block contains "tsactive" and "stage handover",
// both fairing blocks contain "AIRSTEP2", HUD prints "LaLunar v0.9.27".
//
// v0.9.26 — CLIMB VS SCHEDULE + PE_PUSH stage handover (ML-1 flight 39
// debrief, Option A — Stefan's decision).
//
// v0.9.26 (28.10., flight-39 debrief — v0.9.25 worked exactly as
// designed: fairing after SRM_SEP clean, AP_STALL deferred, PE_PUSH
// only after the last STAGE_CMD. The vehicle STILL died with the
// 36/37 signature, and the data pinned the remaining culprit: the
// ALTITUDE-based terminal pitch plan itself. pterm decays linearly
// to 0 deg AT the insertion apoapsis altitude — the vehicle flew
// HORIZONTALLY at its own apoapsis altitude with 120 s of S2 left
// (flight 39: VS = 0 at 342 s, 200.07 km = its own Ap, then fell to
// 159.5 km; TS ignition -574 m/s; the PVG reference ignites +290 m/s
// at 173 km). A pitch PLAN keyed to altitude cannot know how much
// burn time is left — it levels at the target altitude whenever the
// vehicle arrives there early.):
//   (1) CLIMB VS SCHEDULE — while lower stages burn (not tsactive),
//       the exo climb tracks a VS REFERENCE with a POSITIVE floor:
//       vsref = clamp(VS_KM*(AP_GUIDE-alt)/1000, VS_FLOOR,
//       VS_REF_MAX), pitch = FPA + VS_KP*(vsref - VS). Same law shape
//       as the proven PE_PUSH VS law, but the floor (VS_FLOOR,
//       default 200 m/s, wrapper-settable) makes levelling off
//       IMPOSSIBLE — the vehicle arrives at TS ignition still
//       climbing (the PVG-like continuous arc). Governs from
//       EXO_ALT, latches once ("VS_PLAN engaged"), replaces the
//       TERM_P0 plan and AP_HOLD during the lower-stage climb.
//       No-stage vehicles (tsactive from liftoff) are untouched.
//   (2) PE_PUSH STAGE HANDOVER — when the last stage command fires
//       while the VS plan is active, PE_PUSH engages IMMEDIATELY
//       ("stage handover"): the closed-loop VS law (floor VS_REF_MIN)
//       continues seamlessly — no TERM_P0 gap, no levelling moment
//       at the switch. The zoom/past-apogee/stall triggers remain
//       for every vehicle that never entered the VS plan (old paths
//       unchanged).
//   (3) AP_HOLD is now also gated on tsactive — during the
//       lower-stage climb the VS schedule owns the pitch; Ap
//       regulation near the guide is upper-stage work (flight 39:
//       Ap regulation MID-S2 = levelling off = the death spiral).
// File check: line 5 "v0.9.26", climb block contains "VS_FLOOR",
// PE_PUSH block contains "tsactive" and "stage handover",
// both fairing blocks contain "AIRSTEP2", HUD prints "LaLunar v0.9.26".
//
// v0.9.25 — PE_PUSH stage gate + fairing-after-SRM-sep rule (ML-1
// flights 36/37 + PVG reference debrief, flight 38).
//
// v0.9.25 (28.10., ML-1 endgame A/B proof — kOS flights 36/37 died with
// the SAME signature: the Ap-stall latch engaged MID-S2 (293/292.8 s),
// the vehicle flew HORIZONTAL under its Ap for ~150 s, the Transtage
// ignited FALLING, the atmosphere closed before Pe could rise. Gravity
// at MET ~658: kOS 1,117 m/s vs PVG 850 — the entire +267 m/s gap is
// gravity; steering and drag identical. PVG reference (flight 38, same
// vehicle) flew ONE continuous ascent arc and inserted at 200 km):
//   (1) PE_PUSH STAGE GATE (Stefan's decision A — root-cause repair) —
//       ALL PE_PUSH engagement triggers (zoom-predictive, past-apogee,
//       Ap-stall latch) are now gated on the LAST stage command: engage
//       only while tsactive (STAGE_ACTIONS < 1, i.e. the upper stage is
//       the active engine; manual staging decrements too). No-stage
//       vehicles (SL-1/Nimbus, STAGE_ACTIONS = 0) are open from
//       liftoff — old behavior unchanged. The Ap-stall WINDOW KEEPS
//       TRACKING during lower-stage burns: a stall there is LOGGED
//       once ("AP_STALL deferred") and the latch stays armed — the
//       vehicle keeps flying the loft arc (the PVG-like continuous
//       climb) and PE_PUSH engages only after the final STAGE_CMD,
//       when levelling off is actually correct. The latch was built
//       for Zoom-/Agena endgames (last stage active); it misfired when
//       a low-TWR stage could not reach the loft MID-FLIGHT.
//   (2) FAIRING AFTER SRM SEPARATION (flight 37: FAIRING_JETTISON at
//       118.6 s, a panel fell onto the SRM nosecone and destroyed it;
//       SRM_SEP came 10 s later — clean by luck. Stefan's rule, tested
//       in the PVG flight): at SRM carriers (AIR_START_MET > 0) the
//       fairing release waits until SRM_SEP has actually fired
//       (AIRSTEP2, including the timeout backstop). Sequence fix, not
//       an altitude rule — supersedes the FAIRING_MIN_ALT_KM backlog.
// File check: line 5 "v0.9.25", PE_PUSH block contains "tsactive",
// both fairing blocks contain "AIRSTEP2", HUD prints "LaLunar v0.9.25".
//
// v0.9.24 — roll program gate + shortest-direction roll + SRM-residual
// separation trigger (ML-1 SIM2 debrief, flight 34 runs 1-2).
//
// v0.9.24 (28.10., ML-1 SIM2 debrief — both flight-34 runs flew WITHOUT
// the v0.9.23 wrapper flags, so tower gate / fairing filter stayed
// untested; findings below are from lalunar_launch_34.csv):
//   (1) ROLL PROGRAM — the flown wrapper had PITCH_MIN_MET unset, so P0/P2
//       steered heading(AZ, 90) from the pad: the roll started right at
//       liftoff (Stefan's finding). NEW wrapper handle ROLL_MIN_MET (s,
//       default 0 = old behavior). If > 0: pad-attitude hold until MET >=
//       ROLL_MIN_MET (Stefan: minimum 5 s), then a dedicated roll phase
//       slews the commanded azimuth from the PAD azimuth to AZ along the
//       SHORTEST of the two possible directions (wrapped delta), staying
//       essentially vertical — heading(azCmd, 89.5) breaks the pitch-90
//       heading ambiguity. Slew rate constant ROLL_DEG_PER_SEC = 10 deg/s.
//       P3 then starts its pitch program from 89.5 deg when a roll phase
//       ran. Shuttle-class vehicles need a better roll steering concept
//       later — BACKLOG.
//   (2) SRM SEP TRIGGER — v0.9.22 measured TOTAL ship thrust against the
//       peak: with the core lit, availablethrust never falls below the
//       core level (~2.1 MN), so with SRM_SEP_FRAC 0.02 (threshold ~0.23
//       MN) the trigger could NEVER fire. Flight 34 evidence: no SRM_SEP
//       in the SRM phase (Stefan separated manually at < 4.0 / 3.5 MN
//       ship thrust — both clean, F3: no nozzle damage), and the
//       still-armed trigger fired LATE as a GHOST PRESS in the Transtage
//       phase (T+460/463 s, fraction 0.01) which DECOUPLED THE PAYLOAD
//       (mass 26 -> 12 t in both runs). NEW: the trigger measures the
//       SRM RESIDUAL = availablethrust - CORE_EST, with CORE_EST captured
//       from the thrust jump at the air-start press (thrust 2 s after
//       minus thrust just before; SRM tail-off during the settle makes
//       CORE_EST read LOW, so the residual reads HIGH — the trigger fires
//       LATER, the safe direction). Fires when residual < AIR_PEAK_TH *
//       SRM_SEP_FRAC. Window guard: presses allowed only until
//       SRM_SEP_TIMEOUT (default 40 s) after the air-start; at window end
//       the trigger disarms and a single backstop press + WARN event
//       fires instead (SRMs assumed burnt out). Manual-separation
//       detection: a mass jump > 30 t between P4 ticks sets AIRSTEP2
//       without a press (Stefan's manual staging is safe) — a ghost press
//       can never reach the payload group. ML-1 wrapper: SRM_SEP_FRAC
//       0.05 (~0.6 MN residual, clearly below Stefan's proven-safe
//       3.5 MN ship-thrust boundary).
// File check (v0.9.24): header contains "v0.9.24 — roll program gate",
// init contains "ROLL_MIN_MET", P2 contains
// "ROLL_START", air-start chain contains "CORE_EST".
//
// v0.9.23 (28.10., ML-1 SIM1 debrief, Stefans findings):
//   (1) ROLL/PITCH BEFORE TOWER CLEARING — the old P2 steered to
//       heading(AZ, 90) from liftoff, i.e. the roll to the launch azimuth
//       ran INSIDE the tower zone. ML-1 (635 t, tall stack) rotates there
//       long enough to matter. NEW wrapper handle PITCH_MIN_MET (s, 0 =
//       off, old behavior): if > 0, the vessel HOLDS its pad attitude
//       (set HOLD_ATT to ship:facing on the pad, lock steering to it —
//       no roll/yaw command), and the pitch kick is gated on BOTH speed
//       > PITCH_START and MET >= PITCH_MIN_MET. The roll to AZ then runs
//       TOGETHER with the pitch kick in P3 (real Titan: roll-and-pitch
//       program after tower clear). ML-1 wrapper: 15 s.
//   (2) INTERSTAGE FAIRINGS JETTISONED WITH THE PAYLOAD SHROUD — the Q <
//       0.5 kPa event scan (and the pre-stage variant, v0.9.16) fired
//       "Jettison Fairing" on EVERY part carrying the event: on ML-1 the
//       8 interstage shell panels went with it, the stages LOOKED
//       disconnected and one panel snagged between S1 and S2. Craft-file
//       verified: ALL panels are KzProcFairingSide, but the interstage
//       shells attach to parents named "...Interstage" (SSTUBase.
//       Interstage, RSBRibbedBase.Interstage) while the payload panels
//       attach to the base "RSBRibbedBase" (no "Interstage"). NEW wrapper
//       handle FAIRING_SKIP_INTERSTAGE (bool, default false = old
//       behavior): if true, tryJettison skips every fairing whose OWN name
//       or PARENT name contains "Interstage" — they stay attached and go
//       with their staging group (ML-1: istg 3 with the S2 press, istg 2
//       with the Transtage press — exactly the design rule "interstage
//       shells leave with the stage"). kOS suffixes doc-verified
//       (ksp-kos.github.io): Part:HASPARENT, Part:PARENT:NAME,
//       String:CONTAINS.
//   (3) The snagged panel between S1/S2 is the consequence of (2) — no
//       separate fix; re-check the craft in the VAB after the booster
//       decoupler rotation (Stefan's finding 4, VAB work, not script).
// File check: line 5 "v0.9.23", init contains "PITCH_MIN_MET" and
// "FAIRING_SKIP_INTERSTAGE", HUD prints "LaLunar v0.9.23", tryJettison
// contains "Interstage".
//
// v0.9.22 (28.10., ML-1 SIM1 prep): the staging logic staged only on thrust
// LOSS — impossible for an air-start vehicle: at the SRM flameout the core
// is already burning, so availablethrust never drops below 1 kN and the
// SRM separation would never fire; a timed core ignition cannot be mapped
// to any existing trigger. NEW wrapper handles (backward compatible —
// both default to the old behavior, old wrappers run unchanged):
//   AIR_START_MET  (s, 0 = off). If > 0: at MET >= AIR_START_MET, while
//       thrust is present, ONE stage press ignites the core while the SRMs
//       still burn (ullage is given by SRM thrust — the design overlap).
//   SRM_SEP_FRAC   (0.5 default). After the core ignition the script
//       tracks the peak available thrust; when it drops below this
//       fraction (SRM flameout — ML-1: ~2.1 MN of ~11.7 MN peak = 0.18)
//       ONE stage press fires the SRM separation group (sep motors are
//       part of that group). Confirmed via log thrust fraction.
//   STAGE_ACTIONS then counts the presses AFTER the SRM separation up to
//   the upper-stage ignition as before (ML-1: 2 = S2, Transtage).
//   FAILURE PATH: if the core does not ignite (TestFlight), thrust drops
//   below 1 at the SRM flameout and the OLD burnout branch takes over
//   (stage-until-thrust) — the WARN "no thrust after staging" in the log
//   marks that case; it is exactly what the KCT sim rehearses.
//   HOTFIX (28.10.): the init defaults first used "if not defined X" —
//   not parseable in kOS 1.6 (unexpected EOI at the brace). Now if/else
//   with a self-assignment no-op; see the init block comment.
// File check: line 5 "v0.9.22", init contains "AIR_START_MET", HUD prints
// "LaLunar v0.9.22", P4 loop contains "CORE_IGN_AIR" and "SRM_SEP".
//
// v0.9.21 (25.10., flight-32 reflight debrief): the reflight of v0.9.20
// auto-cut at MET 547.3 (the attempt-1 cutoff deadlock IS solved — pe
// closed to 398.1 km vs INS_PE 400) but with Ap 6374.9 km and inc
// 87.49 (window 90-100 missed; window Ap 800-1000 missed by 6.4x).
// THREE fixes, one failure class each:
//   (1) PLANE CAPTURE FIX — the old capture at the Q-trigger froze a
//       rotation-contaminated plane: at ~121 s / 1900 m/s / 71 km the
//       ~409 m/s Earth rotation dominates the inertial direction, the
//       captured plane read ~87 deg against the 95-deg target, and the
//       PI controller steered the WHOLE flight back into it (daz
//       saturated at +12 deg = PLANE_MAX, commanded azimuth ~363 deg,
//       final inc 87.49). At 33-deg launches the same error is only
//       ~1.5 deg (the documented undershoot of flights 30/31); at polar
//       azimuths ~5x. NEW CAPTURE, first match wins, latched:
//       (a) CONVERGED — surface speed > PLANE_CAP_MIN and the
//           osculating inc within PLANE_CAP_TOL of TARGET_INC (the
//           fixed-AZ trajectory converges toward the formula inc as
//           speed grows — freeze the plane exactly there);
//       (b) FALLBACK — surface speed > PLANE_CAP_FB (never converged,
//           e.g. a bad AZ: a LATE capture beats an early WRONG one; the
//           hold itself is flight-proven — the held inc varied
//           +-0.03 deg over 400 s in the reflight).
//   (2) AP_CUT CEILING — new wrapper handle AP_CUT_KM (km, -1 = auto:
//       AP_GUIDE + 500 km; old wrappers run unchanged on auto). If
//       apo > AP_CUT: hard cutoff + AP_CUT event, Pe trim at the
//       apogee via SB-1 (~30-60 m/s). Deterministic bound for every
//       runaway class — in BOTH flight-32 attempts the soft guard
//       never engaged, so the mission needs a hard limit that cannot
//       miss. Nimbus flies AP_CUT_KM 1100.
//   (3) GUARD ENERGY TRIGGER — the v0.9.20 point-perigee test
//       (alt - pe < 5 km) never fired in the reflight: the vehicle
//       crossed circular speed CLIMBING at +330 m/s, the osculating pe
//       trailed 23 km below the altitude all the way (min gap
//       22.8 km); between the circular crossing (MET 477, pe ~105 km)
//       and cutoff the Agena added ~1080 m/s, all into Ap (403 ->
//       6375 km). New branch (OR): altitude below the osculating SMA
//       (2*alt < apo + pe — equivalent to above-circular speed at the
//       current point, no new kOS suffixes), gated to the terminal
//       phase (pe within 10*ORB_TOL of INS_PE) so the healthy mid-push
//       overshoots of flights 30/31 stay untouched. Action unchanged.
// File check: line 4 "v0.9.21", file contains "APUSH_GUARD engaged" and
// "set AP_CUT to" (init), HUD prints "LaLunar v0.9.21".
//
// v0.9.20 (25.10., flight-32 debrief): Nimbus (LOG_NR 32, INS 400 x 400,
// Agena ~330 s burn) — the v0.9.19 guide regulation worked through the
// climb (AP_HOLD engaged at 398 km), but once the horizontal speed passed
// circular the CURRENT POINT became the perigee: the osculating pe tracked
// the vehicle altitude (~380 km, always just below INS_PE - ORB_TOL), the
// cutoff condition (apo >= INS_AP AND pe >= INS_PE) could therefore never
// fire, the burn continued at 100 % and the Ap ballooned to 3000 km
// (+2600 over target) until the manual abort with the Agena nearly dry
// (orbit 3000 x 379 km, dvExp 10,541 m/s). Flights 30/31 showed the same
// long-burn overshoot class benignly (+412/+443 km) — there the cutoff
// DID fire on pe because the drop-off INS_PE (300) sat well below the
// guide, so the point-perigee deadlock never bit. CHANGES — one error
// class, the terminal runaway of long-burn Pe-push phases:
//   (1) TERMINAL APO-RUNAWAY GUARD — during PE_PUSH, when the current
//       point IS the perigee (altitude within 5 km of pe — the
//       above-circular regime) and the osculating apo exceeds
//       AP_GUIDE + APUSH_TOL, add a ONE-SIDED pitch-up offset
//       (APUSH = APUSH_GAIN deg per km of apo excess, clamped to
//       APUSH_MAX). At above-circular speed the vehicle rides its own
//       arc upward: pe = altitude rises toward the target, the Ap growth
//       is arrested, the cutoff closes. One-sided (pitch up only) by
//       design — flight-14 lesson: an Ap-error pitch-DOWN behind the
//       apogee fed a re-entry dive; this guard never steers down, and it
//       only acts in the perigee regime, where the apo error IS the true
//       overshoot measure (in the climb regime AP_HOLD governs; in the
//       mid-arc drop-off regime (flights 15/27/28/31: pe far below
//       altitude) it stays inactive by construction).
//   (2) THROTTLE EXTENSION — on THROTTLEABLE stacks the THROTTLE_NEAR
//       fine-trim now ALSO engages while the guard is active (not only
//       when pe is within 10*ORB_TOL of target): a runaway burn drops
//       100 % -> THROTTLE_NEAR while the guard climb closes pe. Non-
//       throttleable stacks (SL-1/AJ10 class) keep the pitch-only guard.
// New constants are hardcoded in init (NO new wrapper params — old
// wrappers keep running unchanged): APUSH_TOL 10000 m deadband,
// APUSH_GAIN 0.3 deg/km, APUSH_MAX 20 deg. Log: APUSH_GUARD event
// at engage. FILE-CHECK before replacing the local copy: line 4 contains
// "v0.9.20", the file contains "AP_PUSH_GUARD engaged" and
// "set APUSH_GAIN" (init), "set AP_CUT to" (init), HUD prints
// "LaLunar v0.9.21".
//
// v0.9.19 (25.10., flight-29 debrief): flight 29 (SL-1, INS 350 x 300,
// inc-35 target) engaged PE_PUSH only at ~123.5 s, and AP_HOLD then
// regulated against the LOFT (260 km) while the climb carried the Ap to
// 523-528 km — insertion ended 301 x 100 km @ inc 32.3, and the Agena
// paid 39 m/s of Pe trim to fix it. Both the regulation target and the
// engage trigger ignored the insertion apogee and the climb zoom.
// CHANGES (Stefan's decisions, 25.10., variants a/b/c/d):
//   (a) GUIDE APOGEE — new wrapper handle AP_GUIDE_KM (km, default -1 =
//       auto: max(AP_LOFT, INS_AP); read via the DEFINED keyword so old
//       wrappers without the line keep running). AP_HOLD and the VS law
//       regulate against AP_GUIDE instead of the loft. Drop-off profiles
//       (loft above insertion) behave exactly as v0.9.18.
//   (b) ZOOM-PREDICTIVE ENGAGE — trigger (a) becomes
//       (apo + ZOOM_K*(VS/1000)*(VS/1000)) >= AP_GUIDE - ORB_TOL while
//       climbing (multiplication, no ^ operator). ZOOM_K default 60000 m
//       per (km/s)^2 of climb VS (calibration: flight 23 ~47k, flight 29
//       ~73k). Triggers (b) VS <= 0 and (c) the Ap-stall latch unchanged.
//       VS_OFF_MAX 30 -> 45 deg: the zoom arrest needs the authority
//       (VS_KP stays 0.02 — oscillation risk, authority first).
//   (c) PLANE_P 1.0 -> 2.5 (DAZ lag vs. great-circle rotation; PLANE_I
//       stays 0.015).
//   (d) PITCH_RATE wrapper default 0.5 -> 0.75 (flights 22-27 flew
//       0.75-1.0; the old 0.5 default comment was stale).
// Log verification: PE_PUSH event fires EARLIER (flight-29 profile
// ~55-70 s instead of 123.5 s), Ap peaks near AP_GUIDE (350, not
// 523 km), DAZ tracks to ~7-9 deg, INC final >= 34 at target 35.
//
// v0.9.18 (24.10., flights 26/27): FIRST ORBIT — flight 27 (PITCH_RATE 1.0,
// AP_LOFT 260) inserted 371.7 x 201.5 km inc 28.20, CUTOFF 526 s, and the
// terminal THROTTLE_NEAR fine-trim worked live for the first time (Pe +1.5 km
// on target). Remaining systematic error: inclination drift -0.33 to -0.35
// deg in EVERY flight since v0.9.14 — the plane-hold integral is too slow
// against the great-circle rotation: DAZ reaches only ~5 deg where the
// rotation needs ~9 deg azimuth-equivalent, so pint carries a permanent lag
// (steady-state perr) that keeps steering just short of the plane.
// SINGLE CHANGE: PLANE_I 0.005 -> 0.015 (lag shrinks by a factor 3;
// damping analysis from the v0.9.15 round: underdamped oscillation period
// ~1257 s exceeds the remaining post-capture window ~440 s, so the
// oscillation never develops in-flight; expected residual drift ~0.1-0.2
// deg). Log verification: DAZ_DEG should track higher (~7-9 deg), INC final
// >= 28.4. Everything else is byte-identical to v0.9.17.
//
// v0.9.17 (23.10., flight 22, lalunar_launch_22.csv): the C-retooled stack
// (LR87 SustainerUpgrade, SLT ~1.61) reached the Ap target EARLY — at only
// 118 km altitude with 1885 m/s vertical speed left. The "approach gently"
// throttle branch (Ap > INS_AP - 2*ORB_TOL -> THROTTLE_NEAR 40 %) then held
// the Agena at TWR < 1 for 248 s (S1 ignored the command — no throttle range,
// burned full until its rated burnout), and the later 314-s full-thrust
// PE_PUSH past the apogee was far from impulsive (Ap rose 318 -> 472 km
// instead of holding). Prop exhausted ~52 s full-thrust-equivalent short of
// orbit; no CUTOFF, no REPORT. Diagnosis: the VEHICLE is no longer the
// problem — the guidance wasted ~500-700 m/s in gravity losses.
// CHANGES (Stefan's decisions, 23.10.):
//   (1) PITCH REGULATES, ALWAYS — including through short settle throttling
//       before staging. Ap control never runs over throttle.
//   (2) THROTTLE POLICY — non-throttleable engines are the DEFAULT (RP-1
//       launch stacks: HG-3, J-2S/X, F-1, M-1 ...). Partial throttle has
//       exactly two legitimate uses: (a) a brief settle cut before staging,
//       (b) Max-Q reduction if Q ever becomes a problem (not implemented,
//       low priority). The minutes-long 40 % hold is GONE: on a throttleable
//       stage THROTTLE_NEAR now applies only as terminal fine-trim, when the
//       Pe is within 10*ORB_TOL of target.
//   (3) PE_PUSH CLOSED-LOOP VS LAW — replaces the "wait for VS <= 0, then
//       prograde + constant bias" architecture. Engage: as soon as Ap
//       reaches AP_LOFT - ORB_TOL while still climbing (or VS <= 0, or the
//       v0.9.16 Ap-stall latch — both kept). Law: a VS reference that decays
//       linearly to zero at the loft altitude (VS_KM m/s per km below
//       AP_LOFT, capped at VS_REF_MAX), tracked by a proportional pitch
//       offset around surface prograde (VS_KP deg per m/s of VS error,
//       clamped at VS_OFF_MAX). The vehicle now flies a near-horizontal arc
//       INTO the loft with the engine running at full thrust the whole way —
//       no more coasting up a 318-km arc at 40 % and falling back through
//       half an orbit before the push. PE_UP_BIAS is retired (kept as an
//       unused wrapper param so old start.ks files keep running).
// v0.9.16 — Ap-stall latch for the Pe push (flight-20 lesson) + fairing
// handling for staged fairing groups.
//
// v0.9.16 (23.10., flight 20, lalunar_launch_20.csv): the flight-19 reflight
// flew a clean ascent (48-t S1 avionics fixed the lock) but ended SUBORBITAL:
// S1 loft was much flatter than the AJ10 flights (Ap only 91 km at staging vs
// ~196 km before), so AP_HOLD never engaged (Ap never reached AP_LOFT 260 km)
// and PE_PUSH never engaged either — its trigger is VS <= 0, but the pitch
// plan (pterm ~19 deg at 116 km) forced a re-climb, VS bottomed at +35.8 m/s
// at ~400 s, and the script stayed in the loft branch the WHOLE S2 burn,
// spending prop climbing until fuel-out at ~510 s (Ap 158.7 km, Pe -1086 km).
// New failure class "AP-TEASE": the trigger architecture deadlocks whenever
// the stack's achievable apogee sits below AP_LOFT. FIX (1): Ap-stall latch —
// while in the loft branch, if the Ap gains less than AP_STALL_RISE m over
// AP_STALL_WIN s, the Pe push engages early (latched, same law as the
// apogee-past trigger) so the remaining prop goes into the Pe instead of an
// unreachable loft. FIX (2): fairing — flight 20's first STAGE_CMD consumed
// the fairing staging group at ~197 s (KSP physically jettisons the fairing
// on staging), and the later Q<0.5 event scan found no parts and logged a
// false "fairing event not found" warning. Now: tryJettison runs BEFORE each
// stage press once Q < 0.5 kPa (the group is then empty and the skip logic
// handles it), and after staging a read-only countFairings() distinguishes
// "already staged away" (info log) from "event name unknown" (real warning).
// v0.9.15 — plane-hold authority raise (PLANE_MAX 12) + integral anti-windup.
// v0.9.1 fix (23.10.): qkpa() rewritten — kOS 1.6 has no atm:density bound.
// v0.9.2 fixes (23.10., full suffix audit vs kOS 1.6 docs after 2nd crash):
//   ship:inclination -> ship:orbit:inclination (Vessel has no such suffix);
//   part-module scan via p:modules yields NAME STRINGS -> added getmodule();
//   coast ETA: ship:eta:apoapsis -> ship:obt:eta:apoapsis (documented form).
// v0.9.3 redesign (23.10., after first full flight + log analysis): exo
//   terminal replaces fixed 25-deg attitude with an altitude pitch plan
//   (TERM_P0 deg at EXO_ALT -> 0 deg at INS_AP altitude); v0.9.2 never
//   flattened the trajectory (flight 4: 25-deg FPA at Ap-target crossing,
//   Ap ballooned to 1,362 km, no insertion). Log mass column relabelled
//   MASS_T (kOS reports tonnes).
// v0.9.4 (23.10., event names confirmed in VAB by Stefan): "Jettison
//   Fairing" (was "Jettison", never matched) — now jettisons ALL fairings
//   (craft carries interstage + payload shroud); panels via "Extend Solar
//   Panel" (was "Extend Panels"/"Extend"); generic "Extend" dropped
//   (legs/gear risk); avionics RealAntennas antenna has no extend event,
//   stays untouched by design.
// v0.9.5 (23.10., Stefan's staging convention "every phase is a stage"):
//   staging now stages UNTIL THRUST — a single stage command could land on
//   an empty group (payload-fairing staging item left over after the
//   earlier event jettison) and never separate; real separation detected
//   via mass drop > 5 %, then no blind re-staging, 15 s ignition grace.
//   STAGE_ACTIONS now counts stage COMMANDS from liftoff to upper-stage
//   ignition (SL-1 convention: 2). HG-5 antenna deploys via "Extend
//   Antenna" (idempotent; "Toggle Antenna" rejected — state-dependent).
// v0.9.6 (23.10., flight-5 log analysis — FIRST INSERTION 539x202 km, but
//   Ap +339 km over target): terminal AP-HOLD — once Ap reaches the target
//   the terminal phase steers BELOW prograde proportional to overshoot
//   (flight 5 proved the altitude plan only ever tracks prograde: Ap
//   ballooned while S2 pushed Pe for ~400 s). PLANE-HOLD — freeze the
//   orbital plane at exo entry and steer along its horizon trace (fixes
//   the systematic inc drift 28.55 -> 27.89 deg, seen in flights 4 and 5).
//   Both via wrapper flags for separate log diagnosis. COAST stays
//   disabled for circular targets: S2 delta-v (~2.3 km/s) cannot
//   circularize from a ballistic apogee (~5.0 km/s needed) — the vehicle
//   would fall back. Flight-5 events all confirmed live: staging
//   stage-until-thrust, FAIRING_JETTISON (count = PARTS, procedural
//   fairings have several segments), DEPLOY, 40 % near-throttle, cutoff
//   on Pe.
// v0.9.7 (23.10., flight-6 post-mortem — SUBORBITAL, my bug): v0.9.6 used
//   VXCL as if it were the cross product. kOS: VXCL = VECTOREXCLUDE (a
//   projection); the cross product is VCRS. The plane normal came out as
//   the horizontal velocity and the "plane trace" as ~zenith, so the
//   commanded attitude was garbage: flight 6 left the plan right at exo
//   entry (FPA 33 -> 18 deg by 90 km vs planned 29 -> 26), Ap collapsed
//   144.8 -> 128 km, the Ap-target trigger never fired (AP_HOLD never
//   engaged, throttle stayed 100 %), S2 burned all propellant on a flat
//   trajectory, Pe peaked at -3,280 km, re-entry. FIX: vector steering
//   removed entirely — back to the flight-proven heading() primitive.
//   AP-HOLD as designed (pitch = prograde pitch - overshoot offset).
//   PLANE-HOLD rebuilt with VCRS + the standard kOS navball azimuth
//   idiom (north vector, east = vcrs(up, north), arctan2), with the
//   azimuth correction clamped to +/- PLANE_MAX (5 deg, hardened in
//   init) around the launch azimuth so a bad trace can never wreck an
//   ascent. No new wrapper parameters.
// v0.9.8 (23.10., flight-8 post-mortem — NaN crash at exo entry): the
//   v0.9.7 PLANE-HOLD azimuth used the KSLib navball idiom
//   (ship:north:vector / ship:up:vector / arctan2). In this install it
//   died right after plane capture with "Tried to push NaN into the
//   stack" at the arctan2 line — every preceding line provably ran with
//   finite values (the vdot one line earlier pushed a finite scalar),
//   so the NaN entered through the navball vectors: ship:north /
//   ship:up return NaN at ~70 km in this RSS/RO install (exact KSP/mod
//   degeneracy not provable from the log; dependency removed instead).
//   FIX: azimuth correction rebuilt WITHOUT any navball suffix — the
//   signed angle between the CAPTURED plane-trace direction (pdir0,
//   frozen at exo entry) and the CURRENT trace direction (pdir),
//   measured around the radial up vector (ship:position / mag —
//   flight-proven since v0.9.6):
//   daz = arctan2(vdot(vcrs(pdir0, pdir), upv), vdot(pdir0, pdir)),
//   clamped to +/- PLANE_MAX around the launch azimuth. Sign anchored to
//   the KSLib navball convention (east = vcrs(up, north)); VERIFY in
//   flight via the inc trend — if inc degrades faster than with a fixed
//   azimuth, the cross order needs flipping (one line). Normalization
//   guarded so every path is NaN-free. No new wrapper parameters.
// v0.9.9 (23.10., flight-9 post-mortem — NaN crash AGAIN, diagnosis
//   REVISED): v0.9.8 still crashed at the same phase ("Tried to push
//   NaN into the stack" at the sdaz line) although it contained NO
//   navball suffix — every input was provably finite (the flip-check
//   vdot one line earlier ran clean). Pattern across all nine flights:
//   every vdot/vcrs executed as its own statement with plain variables
//   ran clean; the only two NESTED vector-function expressions in
//   project history (v0.9.7: arctan2(vdot(..),vdot(..)); v0.9.8:
//   vdot(vcrs(..),..)) are the only two NaN crashes. The v0.9.8
//   "navball NaN" theory is therefore DISPROVEN; the trigger is the
//   NESTING itself (kOS 1.6.0.1 in this install corrupts vector
//   function calls nested inside another call's argument list).
//   FIX: flatten the azimuth correction — every intermediate in its
//   own set statement, zero nesting (KSLib style):
//   cvec = vcrs(pdirref, pdir); sdaz = vdot(cvec, upv);
//   cdaz = vdot(pdirref, pdir); daz = arctan2(sdaz, cdaz).
//   pdir0 renamed to pdirref. Math and sign convention unchanged.
//   Project rule from now on: NEVER nest vector functions inside
//   another call's arguments. Companion artifact nancheck.ks: manual
//   in-game diagnostic (run nancheck. from the kOS terminal right
//   after a crash) that recomputes the whole chain flat and prints
//   every intermediate — the first bad line names the culprit.
// v0.9.10 (23.10., flight-11 + nancheck live data — ROOT CAUSE FOUND,
//   nesting theory RETIRED): v0.9.9 crashed at the FLAT sdaz line, so
//   the v0.9.9 "nesting corrupts vector calls" theory is disproven.
//   Stefan ran nancheck in the PAUSED flight: ship:position = exactly
//   V(0,0,0), pdir mag = 0, upv = V(NaN,NaN,NaN), velocity normal —
//   nancheck crashed at the same vdot, reproducing the flight crash.
//   Mechanism: KSP 1.12's floating origin pins the world origin to the
//   ACTIVE vessel, and kOS :POSITION returns raw world coordinates, so
//   the active vessel's ship:position is a zero vector => position:mag
//   = 0 => position/mag = NaN. That one fact explains flight 6 (vxcl
//   divide-by-zero => NaN steering vector, the "garbage attitude") and
//   the NaN crashes of flights 8, 9 and 11. The flights-7/8 navball NaN
//   is a SEPARATE defect (elimination: with pdir = zero the v0.9.7 line
//   cannot produce NaN from finite navball values) — navball suffixes
//   stay out either way. FIX: radial up WITHOUT ship:position: the SOI
//   body's center is NOT at the floating origin, so body:position is a
//   valid raw coordinate, and with the active vessel at the origin
//   up = -(body:position)/mag. PLANE_N and pdir now build on upv (same
//   geometry: position was parallel to up). Degenerate fallback: if
//   body:position were ever zero, upv = zero vector and every guard
//   keeps daz = 0 (vdot of zero vectors is 0, not NaN) — worst case a
//   fixed-azimuth ascent, never a crash. PROJECT RULE: never build
//   steering geometry on ship:position of the active vessel in KSP
//   1.12 — this is floating-origin behavior, not install-specific;
//   applies to the hub code generator too. New companion artifact
//   nandiag.ks: reference-frame probe with CSV log (nandiag.csv),
//   safe-to-risky probe order, label printed before each probe so an
//   abort names the culprit, plus an up sanity check (vdot(upv, vel)
//   must match verticalspeed). Supersedes nancheck.ks.
//   Flight-12 postmortem: the crash recurred at line 496 with v0.9.9
//   line numbering — the file Stefan flew was a stale v0.9.9 download
//   (the v0.9.10 artifact publish had not persisted on my side; my
//   fault). v0.9.10 itself has never flown yet. Consequence: the P0
//   HUD now prints the exact version — do not launch unless the HUD
//   shows the expected version number.
// v0.9.11 (23.10., flight-13 analysis — FIRST FULL-LENGTH FLIGHT, no
//   crash; precision tuning round): v0.9.10 flew clean end-to-end
//   (PLANE_HOLD capture at 153.7 s, DEPLOY, AP_HOLD engaged at 172.8 s,
//   stage-until-thrust, CUTOFF 617.8 s, REPORT 2112.1 x 198.2 km inc
//   27.75 — the six-flight NaN saga is closed). Two precision problems
//   remain, both measured in the log:
//   (1) AP OVERSHOOT +1912 km: AP_HOLD engaged at only 91 km altitude
//   with VS 1295 m/s (lofted S1 gravity turn, FPA ~25 deg). Gain
//   0.15 deg/km with the 15 deg cap and the -10 deg pitch clamp gave
//   far too little down authority — VS stayed POSITIVE for 300 more
//   seconds, the trajectory peaked at 326 km, and the near-horizontal
//   S2 burn from there ballooned the osculating Ap to 2112 km.
//   FIX: SYMMETRIC Ap-error law — Ap above target steers below
//   prograde, Ap below target above prograde (the old one-sided
//   max(0,..) could never climb back); AP_HOLD_GAIN 1.5 deg/km,
//   AP_HOLD_MAX 45 deg (both wrapper params — UPDATE start.ks); pitch
//   clamp -30 deg.
//   (2) INC DRIFT -0.85 deg — WORSE than the fixed-azimuth baseline
//   (-0.66 deg, flight 5): the pre-registered flip criterion fired,
//   the azimuth correction sign was wrong. FIX: cvec = vcrs(pdir,
//   pdirref) — argument order swapped (one line). VERIFY again via
//   the inc trend in flight 14: it should now hold ~28.6.
// v0.9.12 (23.10., flight-14 analysis + Stefan's throttleability finding):
//   flight 14 (lalunar_launch_10.csv) CONFIRMED the v0.9.11 Ap fix —
//   Ap peaked 219.3 km (flight 13: 2112 km) and settled ~207 km — but
//   the flight re-entered. TWO root causes:
//   (A) STEFAN'S VAB CHECK: BOTH engines are NOT throttleable. The log
//   proves it: the 40 % command at 168.3 s changed nothing — S1 surface
//   speed kept rising ~53 m/s per second right to burnout. THR_PCT in
//   the log is the COMMANDED throttle, not engine state. So the
//   "40 % Pe push" reading of flight 14 was wrong: the S2 was at FULL
//   thrust the whole time. S2 TWR at push start is ~0.9 g (craft-
//   calibrated: ~23 kN on ~2.9 t), rising to ~1.6 g as prop depletes.
//   Near-target throttle-down was a design assumption that
//   does not hold in RP-1 launch stacks — and on engines that ARE
//   throttleable with min-throttle > THROTTLE_NEAR the command would
//   shut them down mid-burn (known RP-1 trap). FIX: new wrapper flag
//   THROTTLEABLE — false => the script never commands partial throttle
//   (TVAL stays 1; insertion precision comes from the cutoff loop).
//   (B) LOW-TWR PE PUSH NEEDS A LOFT: the vehicle coasted to its
//   trajectory apogee (207.3 km at 357 s, v_orb 5576 m/s, ~2.2 km/s
//   short of circular) and fell — and past apogee the Ap-error law kept
//   steering BELOW a downward prograde (error +7..+19 km => aoff
//   10..28 deg down), wasting ~1.5 km/s on downward thrust and feeding
//   a divergence (horizontal speed grew => Ap rose 207 -> 819 km =>
//   deeper dive). Prop exhausted at 557 s / 66 km: re-entry, cutoff
//   never fired. With a weak upper stage the Pe push is a DESCENDING
//   burn that needs an altitude cushion — flight 13's 326 km loft WAS
//   that cushion; flattening to 207 km removed it. FIXES:
//   (1) AP_HOLD steers against the LOFT apogee (new wrapper param
//   AP_LOFT_KM, default 260 km) instead of the insertion target during
//   the climb: engage at apo >= AP_LOFT - ORB_TOL. Free fall from
//   260 km buys ~205 s vs. the ~150-170 s push; trimming Ap 260 -> 200
//   at Pe later costs the payload only ~20 m/s. The insertion cutoff
//   still checks INS_AP/INS_PE — achieved orbit ~260x200, SB-1 trims
//   (drop-off logic unchanged).
//   (2) PE-PUSH phase: past the trajectory apogee (first VS <= 0 above
//   EXO_ALT, latched; gated off for coast profiles) the Ap-error law
//   is OFF — Ap is high BECAUSE the horizontal burn raises it, so Ap
//   error is the wrong variable past apogee. Steering = thrust NEAR
//   PROGRADE with a constant pitch-up bias PE_UP_BIAS 15 deg
//   (hardcoded A/B handle like TERM_P0): thrust along velocity wastes
//   the least delta-v, the bias stretches the arc against the fall. A
//   true altitude hold cannot equilibrate at push start (thrust accel
//   ~8 m/s2 vs gravity ~8.4 — margin comes only as prop depletes) —
//   the first-draft sink-rate law just saturates and was replaced.
//   Ap rises during the push: accepted, cutoff only needs apo >= INS_AP.
//   (3) THROTTLE per (A); the throttleable branch keeps full thrust
//   during the push (no 40 % Pe push ever again) with a THROTTLE_NEAR
//   precision window near the Pe target.
//   Inc note: the v0.9.11 sign flip helped (drift rate roughly HALVED,
//   -0.20 deg over the first 235 s past capture) but did not null it;
//   evaluate once an orbit is reached.
//   CRAFT-FILE VERIFICATION (23.10., sl-1.craft from Stefan's Google
//   Drive, 61 parts, staging fully parsed + Stefan's in-game PAW and
//   MechJeb-dV screenshots — replaces the VERIFY):
//   S1 = ROE-LR87.LH2, config "AJ23-65" / title LR87-LH2-TitanC
//   (Stefan's engine window; AJ23 = Aerojet factory designation of the
//   LR87 family), hydrolox, 33.2 t prop, 579-667 kN ASL-vac, Isp
//   350/403, 196.6 s burn, TWR 1.59, 1 ignition, min throttle 100 % —
//   not throttleable (PAW-confirmed). S2 = ROE-AJ10Adv config
//   "AJ10-133-Hylas" — HYDROLOX (RO config on GitHub), 26.7 kN, Isp
//   433 s vac, 2.64 t prop, dry 0.865 t, 9 ignitions, ullage-tagged,
//   min throttle 100 % — NOT throttleable (PAW-confirmed). MechJeb
//   rates S2 at 5942 m/s vac delta-v / 420.6 s full burn — the
//   earlier 3.37 km/s back-calc was TOO PESSIMISTIC (wrong Isp and
//   dry-mass assumptions). OPEN: flight 14 burned only ~358 s of the
//   ~420 s — LH2 boiloff during the ascent (Kerbalism) is the prime
//   suspect; VERIFY the S2 prop level at staging in the flight-15
//   log. Flight-15 tuning from this calibration: PE_UP_BIAS 15 (TWR
//   ~0.9 at push start needs the arc stretch), AP_HOLD_GAIN 0.6 deg/km
//   (flight-14 climb: 19 km error x 1.5 = 28 deg below prograde — the
//   gain itself fed the flattening dive), AP_LOFT_KM 260 and
//   AP_HOLD_MAX 45 unchanged.
// v0.9.13 (23.10., flight-15 SUCCESS — first orbital insertion via Pe-push;
//   diagnostic build for the LAST test flight): flight 15 (log 11)
//   confirmed all three v0.9.12 mechanisms — cutoff precise on Pe
//   (398.6x200.6 km), loft cushion held (apogee 286.7 km, VS min only
//   -65 m/s during the push vs. -1456 in flight 14), full tanks at
//   staging (394 s burn, margin left). REMAINING deficiency: INC —
//   -0.72 deg over the flight (28.6 -> 27.88), concentrated in the
//   descending push; the v0.9.11 sign flip reduced but did not null
//   the drift. SINGLE CHANGE, no steering modification: new log
//   column DAZ_DEG — the plane-hold azimuth correction (deg, clamped
//   +/- PLANE_MAX), initialized 0 in P0, logged at 1 Hz. Flight 16 is
//   the diagnosis flight: daz trend + inc trend decide the next
//   plane-hold fix (sign/gain/scope) on data, not guesswork. The Ap
//   overshoot (398 vs 200 target, ~57 m/s SB-1 trim at Pe) is ACCEPTED
//   for now — drop-off rule extension (insertion over contract) is a
//   Hub concern, not a script change.
// v0.9.14 (23.10., flight-16 log analysis — the DAZ_DEG column did its
//   job; Stefan approved BOTH changes, each separately verifiable in
//   the log):
//   (1) PLANE-HOLD REBUILT. The flight-16 DAZ column stayed 0.00-0.30
//   deg while inc drifted -0.69 deg (28.59 capture -> 27.90 final):
//   the old measurement was STRUCTURALLY ~zero — cvec =
//   vcrs(pdir, pdirref) runs parallel to the plane normal and upv
//   lies IN the plane, so the sin term vdot(cvec, upv) ~ 0 by
//   construction; the 0.29 deg at cutoff is an artifact (sin of the
//   arc angle x the out-of-plane drift angle), reproduced exactly in
//   post-flight math. The correction channel never moved. The drift
//   comes from the FIXED COMPASS AZIMUTH while the great-circle
//   azimuth rotates along the arc (empirically an azimuth
//   equivalent of ~1-2.5 deg, sideways accel 0.1-0.37 m/s2, worst in
//   the push). NEW LAW (no navball suffixes, every intermediate
//   flat): signed out-of-plane VELOCITY error perr = angle(horizontal
//   velocity -> in-plane trace pdir) around upv, plus an INTEGRAL
//   state pint that discovers the accumulated compass-to-plane
//   rotation — the absolute correction is not measurable against AZ
//   without north/east vectors:
//     daz = clamp(PLANE_P*perr + pint, +/- PLANE_MAX)
//     pint = pint + PLANE_I * perr * dt
//   PLANE_P 1.0, PLANE_I 0.015/s (damping analysis: velocity chases
//   the heading with tau ~ v/a ~ 700 s; an integral-only loop with
//   G >= 0.05/s is badly underdamped, 0.015 gives zeta ~ 0.16 but the
//   oscillation period ~1257 s never develops inside the flight window). Sign
//   check: pdir RIGHT of velocity => perr > 0 => steer right —
//   flight 16 needed exactly a right (azimuth-increasing)
//   correction, so daz should grow POSITIVE and inc should hold
//   ~28.5. Flip criterion unchanged: if inc degrades faster than the
//   fixed-azimuth baseline, flip the evec order (one line: evec =
//   vcrs(pdir, hn)). DAZ_DEG now logs the commanded correction.
//   (2) PITCH SLEW LIMITER. Stefan saw the flight-16 jolt live: at
//   the AP_HOLD -> PE_PUSH law switch (first VS <= 0, 414.0 s) pterm
//   jumps ~31 deg in one tick (prograde-16, i.e. 0.6 deg/km x 26.7
//   km Ap over loft, -> prograde+15). The lateral component of the
//   jolt is the compass-vs-plane offset (pitch rotations run in the
//   compass vertical plane). The transient costs almost nothing (the
//   sweep passes through prograde and the vessel caught within 1-2 s
//   — steering authority is fine), but a 31-deg single-tick command
//   is structural stress for no benefit and heavier payloads (the
//   agreed 1.0 t dummy counter-test) load it further. FIX: PT_SLEW
//   3 deg/s rate limit on the commanded pitch (pcmds), initialized
//   to the first exo pterm (nothing slews at the Q-trigger handover);
//   all normal phases command well under 3 deg/s, so only law-switch
//   jumps are smoothed. Azimuth stays un-slewed (daz is continuous
//   by construction).
//   Flight 17 (LOG_NR 13) expectations: profile identical to 15/16,
//   daz grows to ~1-3 deg positive, inc final ~28.4-28.6 (residual
//   <= 0.2 deg), PE_PUSH transition smooth over ~10 s, VS min
//   ~-65 m/s, cutoff ~593 s, orbit ~394x200.
// v0.9.15 (23.10., flight-17 log analysis — PI sign CONFIRMED, drift
//   -0.69 -> -0.46 deg, but PLANE_MAX 5 saturated from 561 s while inc
//   kept sliding: the controller wanted the ~8.8 deg azimuth
//   equivalent the flight-16 pre-analysis predicted. The clamp was a
//   safety from the retired trace-angle structure; with the
//   flight-proven PI sign the authority is raised to PLANE_MAX 12 deg.
//   Plus integral anti-windup: pint clamped to +/- PLANE_MAX (flight
//   17 showed pint growing unbounded while the output was clamped —
//   harmless there since cutoff followed, but a bloated integral
//   would delay the response after a sign flip). Flight 18 (LOG_NR
//   14) expectations: daz grows to ~8-9 deg late without flattening,
//   inc final ~28.4-28.6, otherwise the flight-15/16/17 profile
//   (VS min ~-65 to -90, cutoff ~591 s, orbit ~330-395x200, prop
//   margin ~0.2 t).
// Expect joint debugging in RP-1 sims; VERIFY markers flag items to confirm
// in the first runs (warpto argument, part-module event names).
// ============================================================================
//
// HOW TO RUN
//   The mission planner hub generates a wrapper per mission. Until the hub
//   exists, make one by hand, e.g. file ll_sl1_demo_001.kos:
//
//     set MISSION_NAME   to "DEMO-001".
//     set TARGET_AP_KM   to 200.        // contract orbit (for the report)
//     set TARGET_PE_KM   to 200.
//     set TARGET_INC     to 28.8.       // deg
//     set INS_AP_KM      to 200.        // insertion orbit (drop-off: lower)
//     set INS_PE_KM      to 200.
//     set LAUNCH_UT      to 0.          // UT seconds; 0 = launch NOW
//     set AZ             to 90.         // launch azimuth, deg from north
//     set PITCH_START    to 50.         // m/s surface speed for pitch start
//     set PITCH_RATE     to 0.75.       // deg/s (v0.9.19 default)
//     set PITCH_FLOOR    to 30.         // v0.9.27: atmosphere pitch-program
//                                       // floor, deg — stops the program
//                                       // decay, keeps some climb attitude
//     set EXO_PITCH_RATE to 0.05.       // v0.9.28: exo program attitude
//                                       // decay above EXO_ALT, deg/s;
//                                       // 0 = off (v0.9.27 behavior)
//     set EXO_PITCH_FLOOR to 18.        // v0.9.28: exo program terminal
//                                       // floor, deg (PVG-like hold)
//     set Q_TRIGGER      to 10.         // kPa, pitch program -> guidance
//     set STAGE_ACTIONS  to 2.          // stage presses from liftoff to upper
//                                       // ignition; empty groups (e.g. the
//                                       // fairing item after event jettison)
//                                       // count too — see v0.9.5 notes
//     set MANUAL_STAGING to false.
//     set ULLAGE_WAIT    to 2.          // s settle after stage command
//     set DEPLOY_Q_KPA   to 0.10.       // antenna/panel deploy threshold
//     set HAS_FAIRING    to true.
//     set EXO_ALT_KM     to 70.         // guidance handover altitude
//     set ORB_TOL_KM     to 2.          // insertion tolerance
//     set THROTTLE_NEAR  to 0.40.       // terminal fine-trim throttle ONLY
//                                       // (v0.9.17: applies in the last
//                                       // seconds when Pe is nearly closed;
//                                       // the long 40 % hold is retired)
//     set COAST_ENABLED  to false.      // coast to apoapsis before Pe push
//     set COAST_PRE_IGN  to 30.         // s real time before re-ignition
//                                       // (RCS alignment window)
//     set IGN_PROOF_KG   to 1.0.       // v0.9.29: coast ignition proof,
//                                       // kg mass flow (SB-1 ~12 kg/s)
//     set IGN_MISS_WIN   to 30.        // v0.9.29: s past Ap without
//                                       // proof -> WARN, PE_PUSH blocked
//     set AP_HOLD        to true.       // v0.9.6: hold Ap at target in
//                                       // terminal (steer below prograde)
//     set AP_HOLD_GAIN   to 0.6.        // deg per km Ap error (v0.9.12 final: 1.5 fed the climb-phase dive)
//     set AP_HOLD_MAX    to 45.         // deg, symmetric clamp (v0.9.11)
//     set PLANE_HOLD     to true.       // v0.9.6: hold exo-entry plane
//     set THROTTLEABLE    to false.     // v0.9.12: stack engines ignore throttle (RP-1 launch stack)
//     set AP_LOFT_KM     to 260.       // v0.9.12: terminal loft apogee, Pe-push cushion
//     set AP_GUIDE_KM    to -1.        // v0.9.19: explicit Ap regulation target, km;
//                                       // -1 = auto max(AP_LOFT_KM, INS_AP_KM)
//     set AP_CUT_KM      to 1400.      // v0.9.21: apoapsis ceiling, km; -1 = auto
//                                       // (AP_GUIDE + 500 km). Hard cutoff + report
//                                       // when exceeded — Pe trim at the apogee
//                                       // via SB-1 (~30-60 m/s).
//     set VS_FLOOR        to 200.       // v0.9.26: climb VS schedule floor
//                                       // while lower stages burn, m/s —
//                                       // keeps TS ignition CLIMBING
//     set LOG_NR         to 3.
//     run lalunar_launch(true).
//
//   No wrapper: run lalunar_launch(false). prints setup help.
//
// DESIGN NOTES
//   - P5 CUTOFF is script-owned (closes on actual orbit state). This is the
//     deliberate replacement for MechJeb PVG's terminal-phase bug.
//   - First-stage ignition stays manual (RP-1 convention: throttle 100 %,
//     ignite by staging). The script detects liftoff.
//   - Atmosphere phase: v0.9.27 CONTINUES the pitch program below
//     EXO_ALT with a Q-dependent AoA clamp around surface prograde
//     (see v0.9.27 header) — the v0.9.0 prograde lock could not
//     flatten a steep arc (flight 40).
// ============================================================================

declare parameter HAVE_PARAMS is false.

if not HAVE_PARAMS {
  print " ".
  print "lalunar_launch: NO PARAMETER BLOCK FOUND.".
  print "Type the set-commands from the header comment (or use a wrapper)".
  print "then: run lalunar_launch(true).".
  shutdown.
}

// ---------------------------------------------------------------------------
// init
// ---------------------------------------------------------------------------
set INS_AP     to INS_AP_KM * 1000.
set INS_PE     to INS_PE_KM * 1000.
set ORB_TOL    to ORB_TOL_KM * 1000.
set EXO_ALT    to EXO_ALT_KM * 1000.
set TERM_P0    to 30.         // terminal pitch plan: deg at EXO_ALT (A/B handle)
set PLANE_MAX  to 12.         // plane-hold azimuth clamp around launch AZ (v0.9.15: raised, 5 saturated in flight 17)
set daz to 0.                 // plane-hold azimuth correction, deg (v0.9.13: logged every second)
set PE_UP_BIAS to 15.         // RETIRED in v0.9.17 (kept so old wrappers keep running; VS law replaces it)
set AP_STALL_WIN to 20.       // Ap-stall window, s (v0.9.16, A/B handle)
set AP_STALL_RISE to 1000.     // Ap gain below this per window counts as stalled, m (v0.9.16, A/B handle)
set VS_KM to 20.               // v0.9.17: VS reference, m/s per km below AP_GUIDE (A/B handle)
set VS_REF_MAX to 400.         // v0.9.17: VS reference cap while climbing, m/s (A/B handle)
set VS_REF_MIN to -60.         // v0.9.17: VS reference floor — gentle descent if over the loft, m/s
// v0.9.29 coast-ignition params (flight-48 finding 2). Same flight-
// proven if/else no-op form as VS_FLOOR (never "if not defined" —
// kOS 1.6 parser).
if defined IGN_PROOF_KG {
  set IGN_PROOF_KG to IGN_PROOF_KG.  // wrapper value stands (no-op)
} else {
  set IGN_PROOF_KG to 1.0.    // kg mass flow that PROVES ignition (SB-1 ~12 kg/s at full)
}
if defined IGN_MISS_WIN {
  set IGN_MISS_WIN to IGN_MISS_WIN.  // wrapper value stands (no-op)
} else {
  set IGN_MISS_WIN to 30.     // s past apoapsis without proof -> WARN (Verpass-Test, flight 49)
}
if defined PPOFF_MAX {
  set PPOFF_MAX to PPOFF_MAX.         // wrapper value stands (no-op)
} else {
  set PPOFF_MAX to 15.       // v0.9.29: coast prograde-push AoA cap around orbital prograde, deg
}
// v0.9.26 FLIGHT-40 PAD-ABORT FIX (Stefan's parser report "unexpected
// token '}' at line 766, col 25"): the VS_FLOOR default first used
// "if not defined X" — NOT parseable in kOS 1.6 (the v0.9.22 hotfix
// rule, see the AIR_START_MET guard). Same if/else no-op form here.
if defined VS_FLOOR {
  set VS_FLOOR to VS_FLOOR.       // wrapper value stands (no-op)
} else {
  set VS_FLOOR to 200.            // v0.9.26: climb VS schedule floor while lower stages burn, m/s (A/B handle)
}
set VS_KP to 0.02.             // v0.9.17: pitch offset per m/s of VS error, deg/(m/s) (A/B handle)
// v0.9.27 atmosphere pitch program: PITCH_FLOOR default, if/else no-op
// form (kOS 1.6 CANNOT parse "if not defined X" — v0.9.22/flight-40 rule).
if defined PITCH_FLOOR {
  set PITCH_FLOOR to PITCH_FLOOR.  // wrapper value stands (no-op)
} else {
  set PITCH_FLOOR to 30.            // v0.9.27: atmosphere program floor, deg (A/B handle)
}
// v0.9.28 exo pitch program: defaults, if/else no-op form (kOS 1.6 rule —
// the parser breaks on "if not defined X")
if defined EXO_PITCH_RATE {
  set EXO_PITCH_RATE to EXO_PITCH_RATE.  // wrapper value stands (no-op)
} else {
  set EXO_PITCH_RATE to 0.            // 0 = program off (v0.9.27 behavior)
}
if defined EXO_PITCH_FLOOR {
  set EXO_PITCH_FLOOR to EXO_PITCH_FLOOR.  // wrapper value stands (no-op)
} else {
  set EXO_PITCH_FLOOR to 18.            // deg — terminal floor of the exo program
}
set EXO_AOA_CAP to 20.          // v0.9.28: exo program cap above prograde, deg (PVG air-AoA band)
set AOA_CAP_Q20 to 3.          // v0.9.27: max |AoA| vs surface prograde, deg, while Q > 20 kPa (max-Q)
set AOA_CAP_Q2  to 6.          // v0.9.27: same while Q > 2 kPa; free below 2 kPa (aero loads gone)
set VS_OFF_MAX to 45.          // v0.9.19: 30 -> 45 — zoom-arrest authority, deg (symmetric)
set PT_SLEW    to 3.          // pitch-command slew limit, deg/s (v0.9.14: law-switch jolt)
set PLANE_P    to 2.5.        // v0.9.19: 1.0 -> 2.5 — plane-hold P gain (DAZ lag, flights 27/29)
set PLANE_I    to 0.015.      // plane-hold integral gain, 1/s (v0.9.18: 0.005 -> 0.015, flight-27 inc drift -0.35)
set pint to 0.                // plane-hold azimuth integral state, deg (v0.9.14)
set AP_LOFT to AP_LOFT_KM * 1000.  // terminal loft apogee (v0.9.12): Pe-push cushion
set ZOOM_K to 60000.          // v0.9.19: zoom prediction, m apogee gain per (km/s)^2 of climb VS
set APUSH_TOL to 10000.       // v0.9.20: terminal guard deadband above AP_GUIDE, m
set APUSH_GAIN to 0.3.        // v0.9.20: guard pitch-up per km of Ap excess, deg/km
set APUSH_MAX to 20.          // v0.9.20: guard pitch-up clamp, deg (one-sided, climb only)
set PLANE_CAP_MIN to 3000.    // v0.9.21: plane capture min surface speed (converged branch), m/s
set PLANE_CAP_TOL to 1.0.     // v0.9.21: plane capture inc tolerance vs TARGET_INC, deg
set PLANE_CAP_FB to 6000.     // v0.9.21: plane capture fallback speed (never converged), m/s
set AP_GUIDE to max(AP_LOFT, INS_AP).  // v0.9.19: Ap regulation target (auto default)
if defined AP_GUIDE_KM {      // v0.9.19: explicit wrapper override (old wrappers: auto)
  if AP_GUIDE_KM > 0 {
    set AP_GUIDE to AP_GUIDE_KM * 1000.
  }
}
set AP_CUT to AP_GUIDE + 500000.  // v0.9.21: apoapsis ceiling (auto: guide + 500 km)
if defined AP_CUT_KM {            // v0.9.21: explicit wrapper override (old wrappers: auto)
  if AP_CUT_KM > 0 {
    set AP_CUT to AP_CUT_KM * 1000.
  }
}
// v0.9.22: air-start chain defaults (old wrappers keep running unchanged).
// HOTFIX (28.10., Stefans parser report "unexpected EOI at line 634, col 30"):
// "if not defined X" is NOT parseable in kOS 1.6 — the parser fails at the
// opening brace. Flight-proven grammar is bare "if defined X" (AP_GUIDE_KM
// idiom). RULE for the code generator: defined-guards always as if/else with
// a self-assignment no-op in the true branch, NEVER "not defined".
if defined AIR_START_MET {
  set AIR_START_MET to AIR_START_MET.   // wrapper value stands (no-op)
} else {
  set AIR_START_MET to 0.
}
if defined SRM_SEP_FRAC {
  set SRM_SEP_FRAC to SRM_SEP_FRAC.     // wrapper value stands (no-op)
} else {
  set SRM_SEP_FRAC to 0.5.
}
// v0.9.23: tower-clear gate + fairing filter defaults (same guard form)
if defined PITCH_MIN_MET {
  set PITCH_MIN_MET to PITCH_MIN_MET.   // wrapper value stands (no-op)
} else {
  set PITCH_MIN_MET to 0.
}
if defined FAIRING_SKIP_INTERSTAGE {
  set FAIRING_SKIP_INTERSTAGE to FAIRING_SKIP_INTERSTAGE.   // no-op
} else {
  set FAIRING_SKIP_INTERSTAGE to false.
}
// v0.9.24: roll program + sep window defaults (same guard form)
if defined ROLL_MIN_MET {
  set ROLL_MIN_MET to ROLL_MIN_MET.     // wrapper value stands (no-op)
} else {
  set ROLL_MIN_MET to 0.
}
if defined SRM_SEP_TIMEOUT {
  set SRM_SEP_TIMEOUT to SRM_SEP_TIMEOUT.   // no-op
} else {
  set SRM_SEP_TIMEOUT to 40.
}
// v0.9.24: roll slew rate for the dedicated roll phase (deg/s, constant —
// no wrapper handle needed yet)
set ROLL_DEG_PER_SEC to 10.
set LOGNAME    to "lalunar_launch_" + LOG_NR + ".csv".
set T0         to 0.
set PHASE      to "P0-INIT".
set EV         to list().            // event buffer for the log

// helpers -------------------------------------------------------------------
function qkpa {
  // dynamic pressure [kPa]. kOS 1.6 has NO atm:density bound (first live
  // test 23.10.: "undefined variable name 'atm'" killed the script at P1).
  // ship:q is kOS's own dynamic pressure, in atm — convert via constant.
  // (q is an alias for ship:dynamicpressure; scales fine in RSS/FAR.)
  return ship:q * constant:atmtokpa.
}

function lg { declare parameter s. log s to LOGNAME. }

function lgev {                        // log event with MET
  declare parameter txt.
  set line to "EV," + round(time:seconds - T0, 1) + "," + txt.
  lg(line).
  print "  [" + round(time:seconds - T0, 1) + "s] " + txt.
}

function warpToTime {
  // v0.9.29 FIX 1 (flight-48 finding 1): warpto() is a NO-OP on kOS 1.6
  // — both coast events logged the SAME MET 486.7 and the 40-minute coast
  // ran in real time (Stefan warped manually). Rails-warp loop instead:
  // request a warp index that shrinks with the remaining time, drop to
  // real time at the target. Call only with engines shut down and
  // throttle unlocked (KSP refuses rails warp under thrust anyway —
  // the same reason this loop can never fire an engine).
  // VERIFY in the flight-48 reflight: events ~40 min apart, script
  // returns to real time on its own.
  declare parameter tTarget.
  set wantWarp to 5.
  until time:seconds >= tTarget {
    set tRem to tTarget - time:seconds.
    if tRem > 1200 {
      set wantWarp to 5.
    } else if tRem > 300 {
      set wantWarp to 4.
    } else if tRem > 60 {
      set wantWarp to 3.
    } else {
      set wantWarp to 2.
    }
    if kuniverse:timewarp:warp < wantWarp {
      set kuniverse:timewarp:warp to wantWarp.
    }
    wait 0.05.
  }
  set kuniverse:timewarp:warp to 0.
  wait 0.2.   // let real-time state settle before control resumes
}

function tryJettison {
  // fairing release. Event name confirmed in VAB by Stefan (23.10.):
  // "Jettison Fairing" (Procedural Fairing recolorable). Returns count, not
  // boolean. kOS audit 23.10.: p:modules returns module NAME STRINGS, not
  // PartModule objects — hence the getmodule() lookup before hasevent/doevent.
  // v0.9.23: with FAIRING_SKIP_INTERSTAGE true, ONLY the payload shroud
  // jettisons at the Q event — a fairing whose OWN name or PARENT name
  // contains "Interstage" is an interstage shell and stays attached until
  // its staging group fires (it leaves with the stage, per design). ML-1
  // craft-file verified: payload panels parent = "RSBRibbedBase", shells
  // parent = "SSTUBase.Interstage" / "RSBRibbedBase.Interstage". Suffixes
  // doc-verified: HASPARENT, PARENT:NAME, NAME:CONTAINS.
  set nf to 0.
  for p in ship:parts {
    for mname in p:modules {
      set m to p:getmodule(mname).
      if m:hasevent("Jettison Fairing") {
        set skipme to false.
        if FAIRING_SKIP_INTERSTAGE {
          if p:name:contains("Interstage") {
            set skipme to true.
          } else {
            if p:hasparent {
              if p:parent:name:contains("Interstage") {
                set skipme to true.
              }
            }
          }
        }
        if not skipme {
          m:doevent("Jettison Fairing").
          set nf to nf + 1.
        }
      }
    }
  }
  return nf.
}

function countFairings {
  // v0.9.16: read-only fairing inventory — parts carrying a "Jettison
  // Fairing" event. Used after staging to tell "the fairing group was
  // already consumed by staging" (info log) from "parts still aboard but
  // the event name does not match" (real warning). Same scan idiom as
  // tryJettison (module NAME strings -> getmodule lookup).
  set nf to 0.
  for p in ship:parts {
    for mname in p:modules {
      set m to p:getmodule(mname).
      if m:hasevent("Jettison Fairing") {
        set nf to nf + 1.
      }
    }
  }
  return nf.
}

function deployPanelsAntennas {
  // Event names confirmed in VAB by Stefan (23.10.): panels "Extend Solar
  // Panel" (blanket solar panel); HG-5 parabolic antenna "Extend Antenna"
  // (deliberately NOT "Toggle Antenna" — toggle is state-dependent and
  // would retract an already-extended dish; extend is idempotent).
  // The avionics-integrated antenna is a RealAntennas module with no
  // extend event (active once the avionics has power; do not switch it
  // off) — intentionally untouched, per decision 22.10. Generic "Extend"
  // events deliberately NOT fired (legs/gear risk).
  for p in ship:parts {
    for mname in p:modules {
      set m to p:getmodule(mname).
      if m:hasevent("Extend Solar Panel") { m:doevent("Extend Solar Panel"). }
      if m:hasevent("Extend Antenna")     { m:doevent("Extend Antenna"). }
    }
  }
}

function logRow {
  set s to "".
  set s to s + round(time:seconds - T0, 1) + ",".
  set s to s + round(ship:altitude, 0) + ",".
  set s to s + round(ship:velocity:surface:mag, 1) + ",".
  set s to s + round(ship:velocity:orbit:mag, 1) + ",".
  set s to s + round(qkpa(), 3) + ",".
  set s to s + round(ship:verticalspeed, 1) + ",".
  set s to s + round(TVAL * 100, 0) + ",".
  set s to s + round(ship:mass, 0) + ",".
  set s to s + round(ship:apoapsis, 0) + ",".
  set s to s + round(ship:periapsis, 0) + ",".
  set s to s + round(ship:orbit:inclination, 2) + ",".
  set s to s + round(daz, 2) + ",".
  set s to s + PHASE.
  lg(s).
}

// log header ----------------------------------------------------------------
lg("MET,ALT_M,SPD_SRF,SPD_ORB,Q_KPA,VS_MPS,THR_PCT,MASS_T,AP_M,PE_M,INC_DEG,DAZ_DEG,PHASE").

// ---------------------------------------------------------------------------
// P0 INIT / VERIFY / WINDOW
// ---------------------------------------------------------------------------
print "==============================================". 
print "LaLunar v0.9.29 — lalunar_launch".
print "MISSION: " + MISSION_NAME.
print "TARGET   " + TARGET_AP_KM + " x " + TARGET_PE_KM + " km @ " + TARGET_INC + " deg".
print "INSERT   " + INS_AP_KM + " x " + INS_PE_KM + " km  (drop-off = differs from target)".
print "AZIMUTH  " + AZ + " deg".
print "AP-HOLD  " + AP_HOLD + " (gain " + AP_HOLD_GAIN + " deg/km, max " + AP_HOLD_MAX + " deg)".
print "PLANE-HOLD " + PLANE_HOLD + " (capture at inc convergence, v0.9.21)".
print "THROTTLEABLE " + THROTTLEABLE + " (false: throttle stays 1, cutoff-only precision)".
print "LOFT AP  " + AP_LOFT_KM + " km (Pe-push altitude cushion)".
print "GUIDE AP  " + round(AP_GUIDE / 1000, 1) + " km (Ap regulation target)".
print "AP-CUT   " + round(AP_CUT / 1000, 0) + " km (apoapsis ceiling — hard cutoff)".
if AIR_START_MET > 0 {
  print "AIR-START core ignition at MET " + AIR_START_MET + " s (SRM overlap, ullage by SRM thrust).".
  print "SRM-SEP at SRM residual < " + SRM_SEP_FRAC + " x SRM peak (residual trigger, backstop " + SRM_SEP_TIMEOUT + " s).".
}
if ROLL_MIN_MET > 0 {
  print "ROLL GATE: pad hold until MET " + ROLL_MIN_MET + " s, then shortest-direction roll to AZ (10 deg/s).".
}
if PITCH_MIN_MET > 0 {
  print "TOWER GATE: pad-attitude hold, pitch kick at MET " + PITCH_MIN_MET + " s (and " + PITCH_START + " m/s).".
}
if FAIRING_SKIP_INTERSTAGE {
  print "FAIRING: payload shroud only at Q event — interstage shells stay until staging.".
}
if COAST_ENABLED {
  print "COAST mode: rails warp to Ap, manual re-ignition, ignition proof by mass flow (v0.9.29).".
}
print "LOGFILE  " + LOGNAME.
print "----------------------------------------------".
print "REMINDER: press F5 (quicksave) now — manual, by design.".
if ship:control:mainthrottle < 0.99 {
  print "WARNING: main throttle < 100 %. Set it before ignition.".
}

set TVAL to 1.
lock throttle to TVAL.
// v0.9.24: tower zone — with PITCH_MIN_MET or ROLL_MIN_MET > 0 the vessel
// HOLDS its pad attitude (no roll/yaw command near the tower). With
// ROLL_MIN_MET > 0 the dedicated roll phase (P2, shortest direction,
// v0.9.24) follows at ROLL_MIN_MET; otherwise the roll runs together
// with the pitch kick in P3 (v0.9.23 behavior).
set ROLLDONE to false.
if (PITCH_MIN_MET > 0) or (ROLL_MIN_MET > 0) {
  set HOLD_ATT to ship:facing.
  lock steering to HOLD_ATT.
} else {
  lock steering to heading(AZ, 90).
}
if ROLL_MIN_MET > 0 {
  set PAD_AZ to ship:heading.   // pad compass heading, captured on the pad
}

if LAUNCH_UT > time:seconds + 20 {
  print "WINDOW in " + round(LAUNCH_UT - time:seconds, 0) + " s — warping to T-15 s (abort: Ctrl+C).".
  // v0.9.29: warpToTime() — warpto() is a NO-OP on kOS 1.6 (flight 48).
  warpToTime(LAUNCH_UT - 15).
  print "T-15 s — back to real time.".
  wait until time:seconds >= LAUNCH_UT.
} else if LAUNCH_UT > 0 {
  print "WINDOW shortly ahead — holding real time.".
  wait until time:seconds >= LAUNCH_UT.
}
print "LIFTOFF WINDOW REACHED — ignite stage 1 by staging (throttle is locked at 100 %).".

// ---------------------------------------------------------------------------
// P1 LIFTOFF (ignition manual, detection automatic)
// ---------------------------------------------------------------------------
set PHASE to "P1-LIFTOFF".
wait until ship:verticalspeed > 5.
set T0 to time:seconds.
lgev("LIFTOFF").
lgev("MISSION " + MISSION_NAME).

// ---------------------------------------------------------------------------
// P2 VERTICAL
// ---------------------------------------------------------------------------
set PHASE to "P2-VERTICAL".
// v0.9.24: hold the captured pad attitude through the tower zone; with
// ROLL_MIN_MET > 0 run a dedicated roll phase first (shortest of the two
// possible directions), then the pitch gate as before.
if (PITCH_MIN_MET > 0) or (ROLL_MIN_MET > 0) {
  lock steering to HOLD_ATT.
} else {
  lock steering to heading(AZ, 90).
}
set lastLog to T0.
if ROLL_MIN_MET > 0 {
  wait until (time:seconds - T0 >= ROLL_MIN_MET).
  // shortest of the two possible roll directions: wrapped to [-180, 180)
  set ROLL_DELTA to mod((AZ - PAD_AZ + 540), 360) - 180.
  set ROLL_DUR to (abs(ROLL_DELTA) / ROLL_DEG_PER_SEC).
  set ROLL_T0 to time:seconds.
  lgev("ROLL_START — slewing " + round(ROLL_DELTA, 1) + " deg to AZ " + round(AZ, 1) + " (shortest direction, " + round(ROLL_DUR, 1) + " s)").
  until (time:seconds - ROLL_T0 >= ROLL_DUR) {
    set ROLL_FRAC to min(1, (time:seconds - ROLL_T0) / max(ROLL_DUR, 0.1)).
    // 89.5 deg pitch breaks the pitch-90 heading ambiguity while rolling
    set ROLL_AZ to mod(((PAD_AZ + ROLL_DELTA * ROLL_FRAC) + 360), 360).
    lock steering to heading(ROLL_AZ, 89.5).
    if time:seconds - lastLog > 1 { logRow(). set lastLog to time:seconds. }
    wait 0.05.
  }
  lgev("ROLL_COMPLETE at AZ " + round(AZ, 1) + " — awaiting pitch gate").
  set ROLLDONE to true.
}
wait until (ship:velocity:surface:mag > PITCH_START) and (time:seconds - T0 >= PITCH_MIN_MET).
set pmsg to "PITCH_START at " + round(ship:velocity:surface:mag, 1) + " m/s".
if (PITCH_MIN_MET > 0) and (ROLLDONE) {
  set pmsg to pmsg + " (tower gate passed — pitch kick, roll complete)".
} else {
  if PITCH_MIN_MET > 0 {
    set pmsg to pmsg + " (tower gate passed — roll+pitch kick)".
  }
}
lgev(pmsg).

// ---------------------------------------------------------------------------
// P3 PITCH PROGRAM (until Q > Q_TRIGGER)
// ---------------------------------------------------------------------------
set PHASE to "P3-PITCH".
// v0.9.24: after a dedicated roll phase the vehicle is already at AZ with
// a defined heading — start the pitch program from 89.5 deg instead of 90
// (avoids the pitch-90 heading ambiguity at the P3 handover).
if ROLLDONE {
  set pcmd to 89.5.
} else {
  set pcmd to 90.
}
set tlast to time:seconds.
set lastLog to time:seconds.
until qkpa() > Q_TRIGGER {
  set dt to time:seconds - tlast.
  set pcmd to max(10, pcmd - PITCH_RATE * dt).
  set tlast to time:seconds.
  lock steering to heading(AZ, pcmd).
  if time:seconds - lastLog > 1 { logRow(). set lastLog to time:seconds. }
  wait 0.05.
}
lgev("GUIDANCE_ON at Q=" + round(qkpa(), 2) + " kPa").

// ---------------------------------------------------------------------------
// P4 GUIDANCE + P5 CUTOFF (single loop; staging, fairing, deploy included)
// ---------------------------------------------------------------------------
set PHASE to "P4-GUIDANCE".
set CUTOFF to false.
set fairing to HAS_FAIRING.
set deployed to false.
set COAST_DONE to false.
set IGN_OK to false.                  // v0.9.29: coast ignition proven by mass flow
set IGN_WARNED to false.              // v0.9.29: one-shot miss warning (Verpass-Test)
set ignMass to 0.                      // v0.9.29: mass reference for the ignition proof, kg
set t_ign to 0.                        // v0.9.29: predicted apoapsis arrival UT (coast flights)
set planeInit to false.
set apholdInit to false.
set pepushInit to false.
set apstallRef to -1.                  // v0.9.16: Ap-stall reference, m (unset = -1)
set apstallT to 0.                     // v0.9.16: Ap-stall window start, s
set pperi to false.                    // v0.9.20: current point is the perigee (terminal guard)
set apush to 0.                        // v0.9.20: terminal guard pitch-up offset, deg
set apushInit to false.                // v0.9.20: guard event-once flag
set APCUT to false.                    // v0.9.21: apoapsis-ceiling flag (P5 message branch)
set lastLog to time:seconds.
set thrLow to false.
set AIRSTEP1 to false.                 // v0.9.22: air-start chain flags
set AIRSTEP2 to false.
set AIR_PEAK_TH to 0.                  // v0.9.22: peak available thrust while SRMs burn, kN
set THR_BEFORE to 0.                   // v0.9.24: ship thrust just before the air-start press
set CORE_EST to 0.                     // v0.9.24: core thrust from the ignition jump
set CORE_EST_BAD to false.             // v0.9.24: estimation failed -> timeout fallback
set IGN_T to 0.                        // v0.9.24: air-start press time
set IGN_MASS to 0.                     // v0.9.24: ship mass at the air-start press
set M_LAST to 0.                       // v0.9.24: per-tick mass for manual-sep detection
set SRM_RESID to 0.                    // v0.9.24: SRM residual thrust
set tsteer to time:seconds.            // v0.9.14: per-tick dt source (plane integral + slew)
set ptermInit to false.               // v0.9.14: slew-limiter init flag
set TSGATE_WARN to false.             // v0.9.25: one-shot log for a deferred Ap stall
set climbInit to false.               // v0.9.26: climb VS schedule latched (event-once flag)
set exoInit to false.                 // v0.9.28: exo pitch program one-shot log flag
set exopitch to 0.                    // v0.9.28: exo program attitude, deg (captured at handover)

until CUTOFF {

  // -- per-tick dt for the plane integral + pitch slew limiter (v0.9.14) ----
  set sdt to time:seconds - tsteer.
  if sdt < 0.05 { set sdt to 0.05. }
  set tsteer to time:seconds.

  // -- v0.9.25 stage gate: the upper stage is the active engine when all
  // stage commands are consumed (auto mode decrements per press, manual
  // mode per confirmed burnout). No-stage vehicles (STAGE_ACTIONS = 0)
  // are open from liftoff = old behavior. PE_PUSH may engage only then.
  set tsactive to (STAGE_ACTIONS < 1).

  // -- air-start chain (v0.9.22: SRM boosters, core ignites in flight) ------
  // v0.9.24 REWORK (flight 34 debrief): the v0.9.22 trigger compared TOTAL
  // ship thrust against the SRM peak — with the core lit, availablethrust
  // never falls below the core level, so a low SRM_SEP_FRAC could never
  // fire and the armed trigger later GHOST-PRESSED in the Transtage phase
  // (decoupled the payload, mass 26 -> 12 t). Now: (a) SRM peak tracked
  // only while the SRMs burn alone, (b) CORE_EST from the ignition jump,
  // (c) trigger on the SRM RESIDUAL, (d) press window + backstop, (e)
  // manual-separation detection via mass jump. STAGE_ACTIONS untouched.
  if AIR_START_MET > 0 {
    if (not AIRSTEP1) and (ship:availablethrust > AIR_PEAK_TH) {
      set AIR_PEAK_TH to ship:availablethrust.
    }
    if (not AIRSTEP1) and (time:seconds - T0 >= AIR_START_MET) and (ship:availablethrust > 1) {
      set THR_BEFORE to ship:availablethrust.   // SRM level just before the press
      set IGN_MASS to ship:mass.
      set M_LAST to ship:mass.
      stage.   // core ignition while the SRMs still burn (ullage by SRM thrust)
      set AIRSTEP1 to true.
      set IGN_T to time:seconds.
      lgev("CORE_IGN_AIR at MET " + round(time:seconds - T0, 1) + " s (SRM overlap)").
    }
    // core thrust estimate from the ignition jump, 2 s settle. The SRM
    // tail-off during the settle makes CORE_EST read LOW -> the residual
    // reads HIGH -> the trigger fires LATER (the safe direction).
    if (AIRSTEP1) and (CORE_EST = 0) and (not CORE_EST_BAD) and (time:seconds - IGN_T >= 2) and (ship:availablethrust > 1) {
      set CORE_EST to ship:availablethrust - THR_BEFORE.
      if CORE_EST < 300 {
        set CORE_EST to 0.
        set CORE_EST_BAD to true.
        lgev("WARN — core thrust estimate failed, SRM sep falls back to the timeout backstop").
      } else {
        lgev("CORE_EST " + round(CORE_EST, 0) + " kN (ignition jump)").
      }
    }
    if (AIRSTEP1) and (not AIRSTEP2) {
      // manual-separation detection: a mass jump > 30 t between P4 ticks
      // means the boosters left (script press sets AIRSTEP2 first, so this
      // only catches Stefan's manual staging) — disarm without a press.
      if (M_LAST - ship:mass) > 30 {
        set AIRSTEP2 to true.
        lgev("SRM_SEP detected via mass jump — manual separation assumed, trigger disarmed").
      } else {
        set M_LAST to ship:mass.
      }
      if (not AIRSTEP2) {
        if (time:seconds - IGN_T >= SRM_SEP_TIMEOUT) {
          stage.   // backstop: SRMs assumed burnt out by now
          set AIRSTEP2 to true.
          lgev("SRM_SEP backstop — sep window timeout (" + SRM_SEP_TIMEOUT + " s after air-start, SRMs assumed burnt out)").
        } else {
          if (ship:availablethrust > 1) {
            if CORE_EST_BAD {
              set SRM_RESID to ship:availablethrust.   // old semantics (never fires early)
            } else {
              set SRM_RESID to ship:availablethrust - CORE_EST.
            }
            if (SRM_RESID < 0) { set SRM_RESID to 0. }
            if (SRM_RESID < AIR_PEAK_TH * SRM_SEP_FRAC) {
              stage.   // SRM residual low -> separation group (sep motors are in it)
              set AIRSTEP2 to true.
              lgev("SRM_SEP at SRM residual " + round(SRM_RESID, 0) + " kN (fraction " + round(SRM_RESID / max(AIR_PEAK_TH, 1), 2) + " of SRM peak) — F3 check for nozzle damage").
            }
          }
        }
      }
    }
  }

  // -- staging (script-owned, MANUAL_STAGING fallback) -----------------------
  if (STAGE_ACTIONS > 0) and (ship:availablethrust < 1) and (time:seconds - T0 > 10) {
    if MANUAL_STAGING {
      lgev("STAGE NOW — manual staging required (thrust lost)").
      wait until ship:availablethrust > 1.
      lgev("MANUAL_STAGE_CONFIRMED").
      // v0.9.25: a confirmed manual burnout consumes one stage action —
      // the stage gate (tsactive) needs the count to reach 0 here too.
      set STAGE_ACTIONS to STAGE_ACTIONS - 1.
      lgev("STAGE_CMD manual (remaining stage actions: " + STAGE_ACTIONS + ")").
    } else {
      lgev("BURNOUT — settling for ullage").
      wait ULLAGE_WAIT.
      // Stage UNTIL thrust (v0.9.5, Stefan's build convention: every phase
      // is its own staging item; e.g. the payload fairing is a staging item
      // that the script's earlier event jettison does NOT consume — the
      // first stage command can land on that empty group and would then
      // never separate the stage). Safety: after a REAL separation (mass
      // drop > 5 %) never stage blindly again — the next group could
      // decouple the payload; give the engine its ignition grace instead.
      set m0 to ship:mass.
      set sdone to false.
      until sdone or (STAGE_ACTIONS < 1) {
        // v0.9.16: event-jettison the fairing BEFORE a stage press can
        // consume the fairing group. Flight 20: the press physically
        // staged the fairing away at ~197 s and the later Q<0.5 scan
        // produced a false "event not found" warning. Below 0.5 kPa the
        // shrouds are non-structural, so release is safe here; once the
        // group is empty the skip logic below handles it as before.
        // v0.9.25: at SRM carriers wait for SRM_SEP (flight-37 rule) —
        // the gate is also in the main-loop fairing block below.
        if fairing and (qkpa() < 0.5) and ((AIR_START_MET <= 0) or AIRSTEP2) {
          set nfair to tryJettison().
          if nfair > 0 {
            lgev("FAIRING_JETTISON (" + nfair + " fairings)").
          }
          set fairing to false.
        }
        stage.
        set STAGE_ACTIONS to STAGE_ACTIONS - 1.
        lgev("STAGE_CMD (remaining auto-stage actions: " + STAGE_ACTIONS + ")").
        wait 2.
        if ship:availablethrust > 1 {
          set sdone to true.
        } else if ship:mass < m0 * 0.95 {
          lgev("SEP confirmed — engine ignition grace 15 s").
          set twait to time:seconds.
          until (ship:availablethrust > 1) or (time:seconds - twait > 15) { wait 0.1. }
          set sdone to true.
        }
        set m0 to ship:mass.
      }
      if ship:availablethrust < 1 {
        lgev("WARN: no thrust after staging — check ignition/ullage (tap RCS-H if needed)").
      }
      // v0.9.16: staging may have consumed the fairing group physically
      // (KSP jettisons procedural fairings on staging). If no fairing
      // parts remain aboard, the later Q<0.5 block must NOT warn.
      if fairing and (countFairings() < 1) {
        lgev("FAIRING already separated (staging consumed the group)").
        set fairing to false.
      }
    }
  }

  // -- fairing ---------------------------------------------------------------
  // v0.9.25 (flight 37): at SRM carriers the release waits for SRM_SEP —
  // a fairing panel fell onto the SRM nosecone at 118.6 s while the
  // boosters were still attached (SRM_SEP 10 s later). Sequence rule.
  if fairing and (qkpa() < 0.5) and ((AIR_START_MET <= 0) or AIRSTEP2) {
    set nfair to tryJettison().
    if nfair > 0 {
      lgev("FAIRING_JETTISON (" + nfair + " fairings)").
    } else {
      lgev("WARN: fairing event not found — jettison manually").
    }
    set fairing to false.
  }

  // -- panels + antennas (simple thresholds, per decision 22.10.) ------------
  if (not deployed) and (qkpa() < DEPLOY_Q_KPA) {
    deployPanelsAntennas().
    set deployed to true.
    lgev("DEPLOY (panels/antennas)").
  }

  // -- steering ---------------------------------------------------------------
  if ship:altitude < EXO_ALT {
    // v0.9.27 ATM_PROGRAM (flight-40 debrief): the prograde lock cannot
    // flatten a steep arc — cos(FPA) feedback keeps dFPA/dt small at high
    // FPA, so the gap to PVG compounds. Continue the P3 pitch program
    // (pcmd decays at PITCH_RATE down to PITCH_FLOOR), then clamp the
    // command to surface prograde +/- a Q-dependent AoA cap. While the
    // program is more aggressive than the cap the vehicle flies
    // prograde-MINUS-cap (bends the arc down, PVG-like); once the floor
    // binds it flies prograde-PLUS-cap (keeps some climb, no
    // over-flattening). PVG held air-AoA -2.3..+0.3 deg through max-Q —
    // a few deg below prograde is aero-safe.
    set qnow to qkpa().
    set aoacap to 999.
    if qnow > 20 {
      set aoacap to AOA_CAP_Q20.
    } else {
      if qnow > 2 {
        set aoacap to AOA_CAP_Q2.
      }
    }
    // Monotonic decay: P3 floors pcmd at 10 deg — never JUMP the program
    // back up to PITCH_FLOOR if P3 already decayed below it (slow vehicles).
    if pcmd > PITCH_FLOOR {
      set pcmd to max(PITCH_FLOOR, pcmd - PITCH_RATE * sdt).
    }
    set vspdnow to ship:verticalspeed.
    set sspdmag to ship:velocity:surface:mag.
    if sspdmag < 50 { set sspdmag to 50. }
    set vratio to vspdnow / sspdmag.
    if vratio > 1 { set vratio to 1. }
    if vratio < -1 { set vratio to -1. }
    set ppitch to arcsin(vratio).        // surface-prograde FPA, deg
    set pcmdc to pcmd.
    if pcmdc > ppitch + aoacap { set pcmdc to ppitch + aoacap. }
    if pcmdc < ppitch - aoacap { set pcmdc to ppitch - aoacap. }
    if pcmdc > 90 { set pcmdc to 90. }
    if pcmdc < 0 { set pcmdc to 0. }
    lock steering to heading(AZ, pcmdc). // ATM_PROGRAM: AoA capped around prograde
  } else {
    set apo to ship:apoapsis.
    set pe  to ship:periapsis.
    if COAST_ENABLED and (not COAST_DONE) and (apo >= INS_AP - ORB_TOL) {
      // coast to apoapsis; warp until 30 s before re-ignition (RCS align window).
      // Requires a restartable engine (>= 2 ignitions). RO re-ignition stays
      // manual: the player ignites inside the 30 s window (that is its purpose).
      set TVAL to 0.
      wait 1.                                     // let the engine shut down cleanly
      unlock throttle.
      lock steering to ship:velocity:orbit.       // attitude hold, SAS/RCS aligns
      set t_ign to time:seconds + ship:obt:eta:apoapsis.
      lgev("COAST — rails warp to T-" + COAST_PRE_IGN + " s before apoapsis re-ignition").
      // v0.9.29 FIX 1 (finding 1): warpto() is a NO-OP on kOS 1.6 — both
      // coast events logged the SAME MET 486.7 in flight 48 and the
      // 40-min coast ran real time. warpToTime() = kuniverse rails loop.
      warpToTime(t_ign - COAST_PRE_IGN).
      set COAST_DONE to true.
      set PHASE to "P4-COAST-ALIGN".
      // v0.9.29 FIX 2 (finding 2): the old "wait until availablethrust > 1"
      // passed INSTANTLY — the engine reports throttle-ready while
      // ullage-blocked (no thrust, no mass flow), so the WARN net never
      // existed and terminal guidance + TVAL 1 engaged 40 min BEFORE the
      // Ap. The script no longer takes the throttle here at all: the
      // ignition-watch branch below releases terminal guidance only
      // AFTER the ignition is PROVEN by mass flow.
      set ignMass to ship:mass.
      set IGN_OK to false.
      set IGN_WARNED to false.
      lgev("COAST T-" + COAST_PRE_IGN + " — RCS alignment + IGNITE NOW (manual: engine start + throttle up)").
    } else if COAST_ENABLED and COAST_DONE and (not IGN_OK) {
      // v0.9.29 FIX 2: ignition watch — non-blocking proof by MASS FLOW.
      // Steering stays on the orbital-prograde hold, throttle stays
      // with the player (they ignite manually per RO convention).
      // ship:mass must drop by IGN_PROOF_KG (default 1 kg: SB-1 burns
      // ~12 kg/s at full thrust — trips in ~0.1 s at 100 %, ~1 s at
      // 10 %; RCS rotation is mass-neutral). No proof by IGN_MISS_WIN
      // past the Ap -> WARN once, prograde hold continues, PE_PUSH
      // stays blocked (Verpass-Test, flight 49). Whenever the proof
      // fires — even minutes late — terminal guidance is released.
      lock steering to ship:velocity:orbit.
      if (ignMass - ship:mass) > IGN_PROOF_KG {
        set IGN_OK to true.
        set TVAL to 1.
        lock throttle to TVAL.
        lgev("IGNITION PROVEN (mass flow " + round(ignMass - ship:mass, 1) + " kg) — terminal guidance released").
      } else if (not IGN_WARNED) and (time:seconds > t_ign + IGN_MISS_WIN) {
        set IGN_WARNED to true.
        lgev("WARN: no ignition proof by T+" + IGN_MISS_WIN + " s past apoapsis — engine cold, prograde hold, PE_PUSH blocked. Ignite manually; guidance resumes on proof.").
      }
    } else {
      // Terminal mode (v0.9.7: heading() primitive + AP-HOLD + PLANE-HOLD).
      // Flight-6 lesson: v0.9.6 steered via vectors built with VXCL — but
      // VXCL is VECTOREXCLUDE (a projection), NOT the cross product (VCRS),
      // so the "plane" was garbage and the flight went suborbital. Steering
      // is back on the flight-proven heading(azimuth, pitch) primitive;
      // the v0.9.6 intent (AP-HOLD, PLANE-HOLD) is kept.
      // Pitch plan (v0.9.3): decays linearly from TERM_P0 at EXO_ALT to
      // 0 deg at the insertion apoapsis altitude. Endpoints are A/B
      // tuning handles.
      set pterm to TERM_P0 * (1 - (ship:altitude - EXO_ALT) / (INS_AP - EXO_ALT)).
      set pterm to max(0, min(TERM_P0, pterm)).
      // AP-HOLD (v0.9.6, flight-5 lesson: the plan only ever tracks
      // prograde — 25.3 deg commanded vs 25.9 deg actual FPA at 90 km —
      // so Ap ballooned +339 km past target while S2 pushed Pe for
      // ~400 s at 40 %). Once Ap reaches the target, steer BELOW
      // prograde proportional to overshoot: Pe rises, Ap stays put.
      // v0.9.26: AP_HOLD is upper-stage work — during the lower-stage
      // climb the VS schedule owns the pitch (flight-39 lesson: Ap
      // regulation near the guide altitude MID-S2 = levelling off =
      // the death spiral). tsactive vehicles are unchanged.
      if AP_HOLD and tsactive and (apo >= AP_GUIDE - ORB_TOL) {
        if not apholdInit {
          set apholdInit to true.
          lgev("AP_HOLD engaged at Ap " + round(apo / 1000, 1) + " km (guide target " + round(AP_GUIDE / 1000, 1) + " km)").
        }
        // v0.9.11 (flight-13 lesson: gain 0.15 deg/km with a 15 deg cap
        // could not kill VS 1295 m/s at engage — VS stayed positive for
        // 300 s, Ap ballooned +1912 km). SYMMETRIC Ap-error law: Ap
        // above target steers below prograde, Ap below target above
        // prograde (the old one-sided max(0,..) could never climb back).
        // New tuning lives in the wrapper: AP_HOLD_GAIN 0.6 deg/km
        // (v0.9.12 final, craft-calibrated — 1.5 drove up to 28 deg
        // below prograde during the flight-14 climb from a 19 km
        // error, feeding the flattening dive), AP_HOLD_MAX 45 deg.
        // v0.9.12: steering reference during the climb is the LOFT
        // apogee (AP_LOFT_KM), NOT the insertion target. A low-TWR
        // non-throttleable upper stage makes the Pe push a DESCENDING
        // burn — it needs the altitude cushion (flight 14: flattened
        // to a 207 km apogee and re-entered; flight 13's 326 km loft
        // is what made its push survivable). Free fall from 260 km
        // buys ~205 s vs. the ~150-170 s push; trimming the Ap down
        // to the contract later costs the payload only ~20 m/s.
        // v0.9.19 (a): regulation reference is AP_GUIDE — max(loft,
        // insertion) by auto default, or the explicit AP_GUIDE_KM. The
        // v0.9.12 loft-only reference let flight 29's zoom carry the Ap
        // to 523 km against a 350-km insertion target.
        set aperr to (apo - AP_GUIDE) / 1000.
        set aoff to AP_HOLD_GAIN * aperr.
        set aoff to max(-1 * AP_HOLD_MAX, min(AP_HOLD_MAX, aoff)).
        set vsurf to max(1, ship:velocity:surface:mag).
        set vpitch to arcsin(ship:verticalspeed / vsurf).   // kOS trig: deg out
        set pterm to vpitch - aoff.
      }
      // v0.9.26 CLIMB VS SCHEDULE (Option A, flight 39): while lower
      // stages burn the exo climb tracks a VS reference with a
      // POSITIVE floor — the vehicle can no longer level off at its
      // own apoapsis altitude with burn time left (flight 39: VS = 0
      // at 342 s, 200.07 km = own Ap, fell to 159.5 km, TS ignition
      // -574 m/s). Same law shape as the proven PE_PUSH VS law (VS_KM
      // m/s per km below the guide, capped at VS_REF_MAX), only the
      // floor differs: VS_FLOOR (default 200 m/s) instead of
      // VS_REF_MIN. Pitch = FPA + proportional VS offset — above
      // prograde when VS is below the schedule (never stop climbing),
      // below prograde when above it (energy goes horizontal early,
      // PVG-like). Engages at EXO_ALT, latches once; no-stage
      // vehicles (tsactive from liftoff) never see it.
      if (not tsactive) and (not pepushInit) and ((not COAST_ENABLED) or COAST_DONE) {
        set hkm to (AP_GUIDE - ship:altitude) / 1000.
        set vsref to VS_KM * hkm.
        set vsref to max(VS_FLOOR, min(VS_REF_MAX, vsref)).
        set vserr to vsref - ship:verticalspeed.
        set vsoff to VS_KP * vserr.
        set vsoff to max(-1 * VS_OFF_MAX, min(VS_OFF_MAX, vsoff)).
        set vsurf to max(1, ship:velocity:surface:mag).   // kOS trig: deg out
        set vpitch to arcsin(ship:verticalspeed / vsurf).
        set pterm to vpitch + vsoff.
        if not climbInit {
          set climbInit to true.
          set exopitch to pcmd.    // v0.9.28: capture the atmosphere program command at handover
          lgev("VS_PLAN engaged (climb VS schedule, floor " + VS_FLOOR + " m/s, guide " + round(AP_GUIDE / 1000, 1) + " km, VS " + round(ship:verticalspeed, 1) + " m/s — upper stage must ignite CLIMBING)").
        }
        // v0.9.28 EXO PITCH PROGRAM (flight-43 debrief): the VS plan
        // alone cannot hold the arc — VS_KP 0.02 caps the pitch offset
        // at ~13 deg, so once the FPA decays the command follows
        // prograde and the S2 vehicle sags below its own Ap mid-burn.
        // PVG holds a decaying ATTITUDE here (air-AoA +17-20 deg).
        // exopitch = atmosphere program value at handover, decays at
        // EXO_PITCH_RATE toward EXO_PITCH_FLOOR; the command is
        // max(VS-plan pitch, exo program), capped at EXO_AOA_CAP above
        // prograde. Off when EXO_PITCH_RATE = 0 (v0.9.27 behavior);
        // this block dies for good once PE_PUSH engages (tsactive).
        if EXO_PITCH_RATE > 0 {
          if exopitch > EXO_PITCH_FLOOR {
            set exopitch to max(EXO_PITCH_FLOOR, exopitch - EXO_PITCH_RATE * sdt).
          }
          set exocmd to exopitch.
          if exocmd > vpitch + EXO_AOA_CAP {
            set exocmd to vpitch + EXO_AOA_CAP.
          }
          if exocmd > pterm {
            set pterm to exocmd.
          }
          if not exoInit {
            set exoInit to true.
            lgev("EXO_PITCH program engaged (hold-attitude decay " + EXO_PITCH_RATE + " deg/s to floor " + EXO_PITCH_FLOOR + " deg, from " + round(exopitch, 1) + " deg, AoA cap " + EXO_AOA_CAP + " deg)").
          }
        }
      }
      // PE-PUSH (v0.9.17 CLOSED-LOOP VS LAW — flight-22 lesson: waiting for
      // VS <= 0 and then burning prograde past a 318-km apogee wasted half
      // the prop in gravity loss. The old trigger assumed the push is a
      // short impulsive burn; with this much burn time it never is.) NEW
      // TRIGGER, first match wins, latched — never re-evaluated:
      //   (a) v0.9.19: the PREDICTED zoom apogee (apo + ZOOM_K*(VS/1000)^2)
      //       reaches AP_GUIDE - ORB_TOL while still climbing: engage NOW
      //       and fly the near-horizontal arc toward the guide altitude
      //       under the VS law (the engine keeps pushing Pe during the
      //       climb arrest).
      //   (b) VS <= 0 above EXO_ALT: apogee already passed — engage late
      //       (loft unreachable profile).
      //   (c) v0.9.16 Ap-stall latch: Ap stalls below the loft — engage
      //       early, same law (the VS reference floor then keeps the nose
      //       near horizontal instead of climbing into an unreachable loft).
      // Gated off for coast profiles (transfer insertions coast through
      // VS <= 0 by design).
      if (not COAST_ENABLED or COAST_DONE) and (ship:altitude > EXO_ALT) {
        if not pepushInit {
          // v0.9.25 STAGE GATE (decision A, flights 36/37 vs PVG 38): PE_PUSH
          // may engage ONLY when the upper stage is active (tsactive = all
          // stage commands consumed). The latch was built for endgames with
          // the last stage burning; on low-TWR stages it levelled the
          // vehicle off UNDER its Ap mid-flight — the entire +267 m/s
          // gravity gap. While a lower stage still burns, a stall is only
          // LOGGED (once) and the loft arc keeps flying.
          // v0.9.19 (b): zoom-predictive engage — while climbing, the engine
          // keeps raising the FUTURE apogee, so the trigger fires on the
          // predicted zoom apogee (apo + ZOOM_K*(VS/1000)^2, computed by
          // multiplication — no ^ operator), not on the current apo
          // (flight 29: engage at 123.5 s, Ap then ballooned to 523 km).
          // v0.9.26 STAGE HANDOVER (flight 39): if the last stage
          // command fires while the climb VS schedule is active,
          // PE_PUSH engages IMMEDIATELY — the closed-loop VS law
          // (floor VS_REF_MIN) takes over seamlessly, no TERM_P0 gap
          // and no levelling moment at the switch. Vehicles that
          // never entered the VS plan (tsactive from liftoff) keep
          // the triggers below, unchanged.
          if climbInit and tsactive {
            set pepushInit to true.
            lgev("PE_PUSH engaged (stage handover from VS plan, VS " + round(ship:verticalspeed, 1) + " m/s, Ap " + round(apo / 1000, 1) + " km)").
          } else {
          set vsk to ship:verticalspeed / 1000.
          set zoomExtra to ZOOM_K * vsk * vsk.
          if (apo + zoomExtra) >= (AP_GUIDE - ORB_TOL) and (ship:verticalspeed > 0) {
            if tsactive {
              set pepushInit to true.
              lgev("PE_PUSH engaged (zoom-predictive, Ap " + round(apo / 1000, 1) + " km + zoom " + round(zoomExtra / 1000, 1) + " km, VS " + round(ship:verticalspeed, 1) + " m/s)").
            }
          } else if ship:verticalspeed <= 0 {
            if tsactive {
              set pepushInit to true.
              lgev("PE_PUSH engaged (past apogee, alt " + round(ship:altitude / 1000, 1) + " km)").
            }
          } else {
            // v0.9.16 Ap-stall latch (flight-20 lesson): with a flatter
            // S1 loft the achievable apogee sits far below AP_LOFT —
            // AP_HOLD never engages and VS never crosses zero, so the
            // old single trigger deadlocked and the whole S2 prop went
            // into an unreachable climb. While still climbing, track the
            // Ap over AP_STALL_WIN s windows: less than AP_STALL_RISE m
            // gained per window means the loft is stalling — engage the
            // Pe push early and latch it (same v0.9.17 VS law).
            // Guard: only when Ap is ALSO far below AP_LOFT — in normal
            // flights (15-18) the Ap plateaus near the loft target while
            // AP_HOLD is engaged; that is healthy regulation, not a stall.
            // v0.9.25: the window KEEPS TRACKING without tsactive; a
            // stall while a lower stage burns is logged once and the
            // latch stays armed for the upper-stage phase (continuous
            // loft arc instead of levelling off under the Ap).
            if apstallRef < 0 {
              set apstallRef to apo.
              set apstallT to time:seconds.
            } else if (time:seconds - apstallT) >= AP_STALL_WIN {
              if (apo - apstallRef) < AP_STALL_RISE and (apo < AP_LOFT - ORB_TOL) {
                if tsactive {
                  set pepushInit to true.
                  lgev("PE_PUSH engaged (Ap stall, Ap " + round(apo / 1000, 1) + " km — loft target unreachable)").
                } else if not TSGATE_WARN {
                  set TSGATE_WARN to true.
                  lgev("AP_STALL deferred (stage gate v0.9.25: upper stage not active, Ap " + round(apo / 1000, 1) + " km) — loft arc continues, PE_PUSH waits for the last STAGE_CMD").
                }
              }
              set apstallRef to apo.
              set apstallT to time:seconds.
            }
          }
          } // v0.9.26: closes the stage-handover else above
        }
        // v0.9.17 CLOSED-LOOP VS LAW (flight-22 lesson: the constant
        // PE_UP_BIAS prograde burn let the vehicle coast a 318-km arc at
        // 40 % and then fall back through half an orbit — Ap rose 318 ->
        // 472 km during the push, prop exhausted before Pe closed). The
        // new law tracks a VS REFERENCE that decays linearly to zero AT
        // the guide altitude: vsref = VS_KM * (AP_GUIDE - alt)/1000, capped
        // at VS_REF_MAX (climb) and VS_REF_MIN (gentle descent if the
        // vehicle overshoots the loft). Proportional pitch offset around
        // surface prograde: need more VS -> pitch above prograde, need
        // less -> below (thrust is full the whole time; PITCH regulates,
        // including through any short settle throttle). Effect: the arc
        // into the loft IS the Pe push — the engine runs near-horizontal
        // from engage to cutoff, VS dies to ~0 by the loft, Pe closes
        // while Ap stays put. PE_UP_BIAS is retired (param kept for
        // wrapper compatibility, unused).
        if pepushInit {
          set hkm to (AP_GUIDE - ship:altitude) / 1000.
          set vsref to VS_KM * hkm.
          set vsref to max(VS_REF_MIN, min(VS_REF_MAX, vsref)).
          // v0.9.29 FIX 3b (flight-48 finding 3): coast-mission envelope
          // guard — hold-altitude clamp. Above the target Pe the VS
          // reference can never be negative: the push at the Ap HOLDS the
          // altitude and burns horizontal, it never descends. Without
          // the clamp vsref runs to the negative floor as soon as the
          // vehicle sits above the guide altitude.
          if COAST_ENABLED and COAST_DONE and (ship:altitude > INS_PE) {
            set vsref to max(0, vsref).
            set vsref to min(VS_REF_MAX, vsref).
          }
          set vserr to vsref - ship:verticalspeed.
          set vsoff to VS_KP * vserr.
          set vsoff to max(-1 * VS_OFF_MAX, min(VS_OFF_MAX, vsoff)).
          set vsurf to max(1, ship:velocity:surface:mag).   // kOS trig: deg out
          set vpitch to arcsin(ship:verticalspeed / vsurf).
          // v0.9.20 TERMINAL APO-RUNAWAY GUARD (flight 32, Nimbus: the
          // long Agena burn pushed past circular speed near the guide
          // altitude — the current point became the perigee, pe tracked
          // the altitude just below target, the cutoff could never fire
          // and the Ap ran to 3000 km at 100 % until the manual abort).
          // v0.9.21 TRIGGER FIX (flight-32 reflight: the v0.9.20 point-
          // perigee test NEVER fired — the vehicle crossed circular
          // speed CLIMBING at +330 m/s, the osculating pe trailed 23 km
          // below the altitude all the way to cutoff (min gap 22.8 km
          // vs the 5 km trigger); between the circular crossing and the
          // cutoff the Agena added ~1080 m/s, all into Ap: 403 -> 6375
          // km). TWO-branch test:
          //   (a) GEOMETRY — point IS the perigee (altitude within 5 km
          //       of the osculating pe), the attempt-1 pinned regime.
          //   (b) ENERGY — altitude BELOW the osculating semi-major
          //       axis (2*alt < apo + pe, equivalent to above-circular
          //       speed at the current point — reflight terminal:
          //       2*421 < 6375 + 398 fires), gated to the TERMINAL
          //       phase (pe within 10*ORB_TOL of INS_PE — the healthy
          //       mid-push overshoots of flights 30/31 with pe far
          //       below target stay untouched).
          // Action unchanged: ONE-SIDED pitch-up offset proportional
          // to the Ap excess — the vehicle rides its own arc up,
          // pe = altitude rises through the target and the cutoff
          // closes at bounded Ap. Never steers down (flight-14
          // lesson: a pitch-down behind the apogee fed the re-entry
          // dive). The hard backstop for every runaway class is the
          // v0.9.21 AP_CUT ceiling — the guard is the soft layer
          // before it.
          set pperi to false.
          if (ship:altitude - pe) < 5000 { set pperi to true. }
          if (not pperi) and (2 * ship:altitude < apo + pe) and (pe > INS_PE - 10 * ORB_TOL) {
            set pperi to true.
          }
          set apush to 0.
          if pperi and (apo > AP_GUIDE + APUSH_TOL) {
            set apush to APUSH_GAIN * (apo - AP_GUIDE - APUSH_TOL) / 1000.
            set apush to max(0, min(APUSH_MAX, apush)).
            if not apushInit {
              set apushInit to true.
              lgev("APUSH_GUARD engaged (terminal runaway, Ap " + round(apo / 1000, 1) + " km vs guide " + round(AP_GUIDE / 1000, 1) + " km — climbing to close Pe)").
            }
          }
          set pterm to vpitch + vsoff + apush.
        }
      }
      set pterm to max(-30, min(89, pterm)).
      // Azimuth: default the launch azimuth AZ; PLANE-HOLD adds the
      // bounded PI correction daz (v0.9.14 — see the PLANE_HOLD block
      // below; the v0.9.8-v0.9.11 trace-angle approach measured a
      // structurally ~zero angle and is retired).
      set azc to AZ.
      if PLANE_HOLD {
        // Radial up WITHOUT ship:position (v0.9.10). Flight-11 + nancheck
        // live data proved the root cause of the NaN crashes: in KSP
        // 1.12 the floating origin pins the world origin to the ACTIVE
        // vessel and kOS :POSITION returns raw world coordinates, so
        // ship:position = V(0,0,0) exactly => position/mag = NaN (this
        // killed flights 8, 9 and 11, and fed garbage to flight 6).
        // Replacement: the SOI body's center is NOT at the floating
        // origin, so body:position is a valid raw coordinate, and with
        // the active vessel AT the origin: up = -(body:position)/mag
        // (same vector whether kOS reports raw or ship-relative
        // coordinates). Degenerate fallback: upv = zero vector — every
        // guard below then keeps daz at 0 deg (vdot of zero vectors is
        // 0, not NaN), worst case a fixed-azimuth ascent, never a crash.
        set bpos to body:position.
        set bmag to bpos:mag.
        if bmag > 0 {
          set upv to (-1) * bpos / bmag.
        } else {
          set upv to V(0, 0, 0).
        }
        if not planeInit {
          // v0.9.21 CAPTURE FIX (flight-32 reflight lesson): the old
          // capture at the Q-trigger fired ~121 s / ~1900 m/s / 71 km,
          // where the ~409 m/s Earth rotation dominates the inertial
          // direction (inertial heading ~3 deg EAST of north for a
          // 351-deg surface AZ) — the frozen plane read ~87 deg against
          // the 95-deg target and the PI controller steered the WHOLE
          // flight back into it (daz saturated at +12 deg = PLANE_MAX,
          // commanded azimuth ~363 deg ~ due north, final inc 87.49).
          // At 33-deg launches the same capture error is only ~1.5 deg
          // (the documented undershoot of flights 30/31); at polar
          // azimuths it is ~5x. NEW CAPTURE, first match wins, latched:
          //   (a) CONVERGED — surface speed above PLANE_CAP_MIN and the
          //       osculating inc within PLANE_CAP_TOL of TARGET_INC:
          //       the fixed-AZ trajectory converges toward the formula
          //       inc as speed grows — freeze the plane exactly there.
          //   (b) FALLBACK — surface speed above PLANE_CAP_FB (never
          //       converged, e.g. a bad AZ: a LATE capture beats an
          //       early WRONG one. The hold itself is flight-proven —
          //       the reflight held its captured inc to +-0.03 deg over
          //       400 s).
          set pcapv to ship:velocity:surface:mag.
          set pcapok to false.
          if pcapv > PLANE_CAP_FB { set pcapok to true. }
          if (not pcapok) and (pcapv > PLANE_CAP_MIN) {
            if defined TARGET_INC {
              if abs(ship:orbit:inclination - TARGET_INC) < PLANE_CAP_TOL {
                set pcapok to true.
              }
            }
          }
          if pcapok {
            set PLANE_N to vcrs(upv, ship:velocity:orbit).
            set pdir to vcrs(PLANE_N, upv).
            if vdot(pdir, ship:velocity:orbit) < 0 { set pdir to (-1) * pdir. }
            if pdir:mag > 0 { set pdir to pdir / pdir:mag. }
            set planeInit to true.
            lgev("PLANE_HOLD — entry plane captured (speed " + round(pcapv, 0) + " m/s, inc " + round(ship:orbit:inclination, 2) + " deg)").
          }
        }
        if planeInit {
          // v0.9.21: everything below needs a captured plane — before
          // capture the azimuth stays at the plain launch AZ (the
          // fixed-AZ ascent does the plane convergence for free).
          // current trace direction: in-plane (perp to N) and horizontal
          // (perp to up) — the plane's trace across the local
          // horizon, flight direction. Normalization guarded: a zero
          // vector stays zero (vdots then read 0, arctan2(0,0)=0 in
          // .NET — correction falls back to 0 deg, no NaN possible).
          set pdir to vcrs(PLANE_N, upv).
          if vdot(pdir, ship:velocity:orbit) < 0 { set pdir to (-1) * pdir. }
          if pdir:mag > 0 { set pdir to pdir / pdir:mag. }
          // v0.9.14 PLANE-HOLD PI CONTROLLER (flight-16 lesson: the old
          // pdirref angle was structurally ~zero — cvec runs parallel
          // to the plane normal while upv lies IN the plane, so the sin
          // term vanished by construction and the correction channel
          // never moved while inc drifted -0.69 deg). New control
          // variable: the signed OUT-OF-PLANE VELOCITY ERROR — the
          // angle between the horizontal velocity and the captured
          // in-plane trace direction pdir, measured around upv — plus
          // an integral state that discovers the accumulated
          // compass-to-plane rotation. Flat ops, no nesting, no
          // navball suffixes. Flip criterion (pre-registered): if inc
          // degrades faster than the fixed-azimuth baseline, flip to
          // evec = vcrs(pdir, hn).
          set tvec to vcrs(ship:velocity:orbit, upv).
          set hvel to vcrs(upv, tvec).
          set hmag to hvel:mag.
          set perr to 0.
          if hmag > 0.01 {
            set hn to hvel / hmag.
            set evec to vcrs(hn, pdir).
            set serr to vdot(evec, upv).
            set cerr to vdot(hn, pdir).
            set perr to arctan2(serr, cerr).
          }
          set pint to pint + PLANE_I * perr * sdt.
          set pint to max(-1 * PLANE_MAX, min(PLANE_MAX, pint)).
          set daz to PLANE_P * perr + pint.
          set daz to max(-1 * PLANE_MAX, min(PLANE_MAX, daz)).
          set azc to AZ + daz.
        }
      }
      // v0.9.14: pitch-command slew limiter (flight-16 jolt — at the
      // AP_HOLD -> PE_PUSH law switch pterm jumped ~31 deg in one
      // tick; recovery was fast, but the single-tick command is
      // structural stress for no benefit). pcmds is the SLEWED
      // command: initialized once to the first exo pterm (nothing
      // slews at the Q-trigger handover), then rate-limited to
      // PT_SLEW deg/s. Normal phases command well under PT_SLEW —
      // only law-switch jumps are smoothed. Azimuth stays un-slewed
      // (daz is continuous by construction).
      if not ptermInit {
        set pcmds to pterm.
        set ptermInit to true.
      }
      set pdif to pterm - pcmds.
      set pmax to PT_SLEW * sdt.
      set pdif to max(-1 * pmax, min(pmax, pdif)).
      set pcmds to pcmds + pdif.
      if COAST_ENABLED and COAST_DONE and (ship:altitude > INS_PE) {
        // v0.9.29 FIX 3a (flight-48 finding 3, MISSION KILLER): the
        // heading(azimuth, pitch) frame is valid only near the launch
        // site. Flight 48 flew a POLAR orbit with a SOUTHERN azimuth
        // (Stefan's correct overwrite) — half an orbit later at the Ap
        // the orbital velocity is ANTIPARALLEL to the launch azimuth
        // (south-launched => north-flying at the Ap), so ANY
        // heading(AZ, ..) command there is RETROGRADE by construction
        // (FR: AoA ~100 deg through the coast, 179.5 deg at the burn).
        // Envelope guard: coast terminal steering is built on the
        // ORBITAL PROGRADE VECTOR with a bounded AoA offset —
        // retrograde is unreachable by construction. The offset reuses
        // the pepush VS-law output (vsoff + apush, deg above prograde);
        // before PE_PUSH engages it is 0 = pure prograde, which is
        // exactly right at the Ap (burn horizontal, Pe rises).
        if pepushInit {
          set ppoff to vsoff + apush.
        } else {
          set ppoff to 0.
        }
        set ppoff to max(-1 * PPOFF_MAX, min(PPOFF_MAX, ppoff)).
        set ppv to ship:velocity:orbit.
        set ppmag to ppv:mag.
        if ppmag > 1 {
          set ppu to ppv / ppmag.
          // radial up WITHOUT ship:position (v0.9.10 floating-origin rule)
          set bpos to body:position.
          set bmag to bpos:mag.
          if bmag > 0 {
            set upu to (-1) * bpos / bmag.
            // kOS trig: degrees in — rotate prograde toward up by ppoff
            set svec to ppu * cos(ppoff) + upu * sin(ppoff).
            if svec:mag > 0.001 {
              lock steering to svec.
            }
          }
        }
      } else {
        lock steering to heading(azc, pcmds).
      }
      if not THROTTLEABLE {
        // v0.9.12 (Stefan's VAB check + flight-14 log: BOTH engines are
        // NOT throttleable — the 40 % command at 168.3 s changed
        // nothing, S1 surface speed kept rising ~53 m/s per second to
        // burnout; THR_PCT in the log is the COMMANDED value). v0.9.17
        // (Stefan): non-throttleable engines are the DEFAULT (RP-1
        // launch stacks: HG-3, J-2S/X, F-1, M-1 ...). Never command
        // partial throttle on such a stack: the engines ignore it, and
        // on engines that ARE throttleable with a min-throttle above
        // THROTTLE_NEAR it would shut them down mid-burn (known RP-1
        // trap). Ap control runs over PITCH, never over throttle.
        // Throttle reduction has exactly two legitimate uses (settle
        // before staging; Max-Q relief if Q ever becomes critical —
        // not implemented, low priority). Insertion precision comes
        // from the cutoff loop.
        set TVAL to 1.
      } else if pepushInit and ((pe > INS_PE - 10 * ORB_TOL) or (pperi and (apo > AP_GUIDE + APUSH_TOL))) {
        // v0.9.17 TERMINAL FINE-TRIM ONLY — the only remaining use of
        // THROTTLE_NEAR on a throttleable stage: the Pe is within
        // 10*ORB_TOL of target, the cutoff loop is about to close, and
        // full thrust would overshoot the insertion. The flight-22
        // failure (248 s at 40 % with TWR < 1 from the "approach
        // gently" branch, ~500-700 m/s of gravity loss) is retired.
        // v0.9.20 EXTENSION: the fine-trim ALSO engages while the
        // terminal apo-runaway guard is active (point is perigee, apo
        // beyond the guide deadband) — flight 32 burned 330 s at 100 %
        // while the Ap ran to 3000 km; a throttleable stage drops to
        // THROTTLE_NEAR while the guard climb closes pe.
        set TVAL to THROTTLE_NEAR.
      } else {
        set TVAL to 1.
      }
    }
    if (apo >= INS_AP - ORB_TOL) and (pe >= INS_PE - ORB_TOL) {
      set CUTOFF to true.
    }
    // v0.9.21 AP_CUT CEILING (flight-32 class: guidance-bound runaway
    // burns — in BOTH flight-32 attempts the soft guard never
    // engaged; a bounded mission needs a hard limit that cannot miss).
    // apo above the ceiling = cutoff NOW; the Pe trim happens at the
    // apogee via SB-1 (~30-60 m/s — closing pe at the apogee is ~30x
    // cheaper than at the perigee, the flight-32 lesson in one line).
    if apo > AP_CUT {
      if not APCUT {
        set APCUT to true.
        lgev("AP_CUT — apoapsis ceiling reached (Ap " + round(apo / 1000, 1) + " km > " + round(AP_CUT / 1000, 0) + " km) — cutoff, Pe trim at the apogee via SB-1").
      }
      set CUTOFF to true.
    }
  }

  if time:seconds - lastLog > 1 { logRow(). set lastLog to time:seconds. }
  wait 0.05.
}

// ---------------------------------------------------------------------------
// P5 CUTOFF — script-owned shutoff (the PVG-bug fix)
// ---------------------------------------------------------------------------
set PHASE to "P5-CUTOFF".
set TVAL to 0.
wait 1.                       // let the command settle before unlocking
unlock throttle.
unlock steering.
set ship:control:pitch to 0.  // neutral input; SAS stays to the player
if APCUT {
  lgev("CUTOFF — apoapsis ceiling, insertion partial (see AP_CUT event)").
} else {
  lgev("CUTOFF — insertion reached").
}

// ---------------------------------------------------------------------------
// P6 POST-INSERTION REPORT
// ---------------------------------------------------------------------------
set PHASE to "P6-POST".
set apo to ship:apoapsis.
set pe  to ship:periapsis.
set inc to ship:orbit:inclination.
set lastLog to time:seconds.
logRow().

print "==============================================".
print "INSERTION REPORT — " + MISSION_NAME.
print "ACHIEVED   " + round(apo/1000, 1) + " x " + round(pe/1000, 1) + " km @ " + round(inc, 2) + " deg".
print "INSERTION  " + INS_AP_KM + " x " + INS_PE_KM + " km (tol " + ORB_TOL_KM + " km)".
print "CONTRACT   " + TARGET_AP_KM + " x " + TARGET_PE_KM + " km @ " + TARGET_INC + " deg".
if (pe < 0) {
  print "WARNING: periapsis still negative — payload must finish orbit raise.".
}
print "Payload trim: use SB-1 engine / MechJeb node. Log: " + LOGNAME.
print "==============================================".

lgev("REPORT achieved " + round(apo/1000, 1) + "x" + round(pe/1000, 1) + " km inc " + round(inc, 2)).
print "lalunar_launch: DONE.".
shutdown.
