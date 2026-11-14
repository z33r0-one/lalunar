# LaLunar Mission Center

Telemetrie-Auswertung und kOS-Missionsscripte fuer eine RP-1-Karriere in Kerbal Space Program
(Realism Overhaul / Real Solar System). LaLunar ist das komplette Missions-Software-Paket dieser
Karriere: Autopilot-Scripte fuer die Missionsphasen plus ein Web-Tool, das die geflogenen Logs
auswertet und die Entwurfsentscheidungen mit Referenzwerten unterlegt.

## Web-Tool: index.html

Eine einzige HTML-Datei, keine Installation, kein Server. index.html im Browser oeffnen
(lokal oder via GitHub Pages). Benoetigt beim Laden nur Internet fuer die Chart-Bibliothek
(Chart.js via CDN); alle Missionsdaten bleiben lokal im Browser - nichts wird zu einem
Server uebertragen.

### Aufbau

Registerkarten je Missionsphase: Launch (aktiv), Landing (aktiv), TLI (Platzhalter),
Ascent & Rendezvous (Platzhalter). Jeder Tab folgt der gleichen Struktur:

- Erklaerkarten oben: Script-Aufruf und Ablauf der Phase in Kuerze.
- Upload A/B: zwei CSV-Dateien parallel (Ist-Flug gegen Referenz- oder Vergleichsflug).
- 2x2-Metriken: die vier Kennzahlen der Phase auf einen Blick.
- Charts: volle Breite, Hoehen-/Geschwindigkeits-/Q-/Beschleunigungsprofile und Phasenverlauf.
- Benchmark: Einordnung gegen historische Referenzen (Launch-Tab: klassische Traegerraketen).
- Design-Limits-Block: 5-10 Richtwerte der Phase als Checkliste fuer den naechsten Entwurf.

### Launch-Tab

Delta-v total, Verluste einzeln ausgewiesen (Gravity, Steering, Drag), time-to-orbit,
Max G, Max Q und Rest-delta-v-Budget im Orbit (das Budget, das die Nutzlast noch
einsammeln kann - fuer Vertragsmissionen der entscheidende Wert). Benchmark gegen
historische Traegerraketen.

### Landing-Tab

Treibstoffverbrauch, Effizienzrating, total delta-v expended (toolseitig aus den Massen-
und Isp-Spalten berechnet: Isp * g0 * ln(m0/m1), RCS-Verbrauch eingeschlossen) und Max G.
Design-Limits u.a.: TWR (Mond) 1,8-4,0 mit descentStart-Regeln, Landemotoren-Drosselgrenze,
mindestens 2 Zuendungen fuer Ullage, Zielorbit 50-100 km, Skriptstart etwa eine halbe
Umlaufbahn plus 500 km vor dem Ziel, Hover 100 m mit Slides unter 3 m/s, Sinkratenprofil
mit Cutoff bei 25 cm, Treibstoffreserve ueber 25 s.

### Dateneingabe

CSV-Logs der kOS-Scripte (1 Hz Zustandszeilen plus Event-Zeilen) und des Flight-Recorders.
Log-Namenskonvention: lalunar_<phase>_<NN>.csv (launch / tli / land / ascent).

## kOS-Scripte: kos/

### lalunar_launch.ks (Launch-Kernscript v0.9.29)

HUB-Prinzip: Der Kern enthaelt die komplette Logik einmal; je Mission existiert ein kleiner
Wrapper (nur set-Zeilen), der die Missionsparameter setzt und den Kern aufruft. Das loest das
Problem mehrerer Script-Varianten ueber den Dateinamen.

Phasen P0-P6: Warpen zum Startfenster, Liftoff, Pitch-/Azimut-Fuehrung auf Zielinklination und
Zielorbit (Azimut aus Zielinklination und Pad-Breite, Warnung bei unmoeglicher Inklination),
Staging mit Ullage-Settle und Schubnachweis, Fairing-Jettison unter 0,5 kPa, Antennen- und
Solar-Deploy per Modul-Scan, script-eigener Closed-Loop-Cutoff auf den Soll-Orbit,
Insertion auch unter dem Vertragorbit (DROP-OFF-Modus, Nutzlast bringt eigenes delta-v mit),
Coast-Phasen mit Warp bis 30 s vor der Wiederzuendung (30 s RCS-Ausrichtungsfenster,
Zuendung manuell wie in Realism Overhaul ueblich, Script wartet auf Schubnachweis).
1-Hz-CSV-Log parallel fuer das Web-Tool.

### lalunar_land.ks (Mondlande-Autopilot v6.8 / LaLunar 0.8)

Deorbit-Burn, Coast (RCS aus, 40 s vor Bremsbeginn wieder an), Descent-Burn ab 15 km:
Sinkrate wird ueber ein Hoehe-Profil gehalten, die Restkraft bremst die seitliche Bewegung.
Hover bei 100 m mit Positions-Halt auf Zielkoordinaten (targetLat/targetLng, Querregelung
auf unter 2 m/s), Landeschenkel-Check mit Auto-Ausfahren bei 1000 m, Scheinwerfer ab 15 km,
finaler Sinkflug manuell (AG5, Sinkrate per AG7/AG8), Cutoff bei 25 cm, Zielabweichung
wird als TARGET_MISS-Ereignis geloggt.

### start_ks_v0929_midas_sim1r_flug49.ks (Wrapper-Beispiel)

Beispiel fuer einen Missions-Wrappers (Midas-49-Wiederholungsflug): zeigt, welche Parameter
der Kern erwartet und wie ein Wrapper aussieht. Neue Mission = dieser Block mit neuen Werten,
Kern unangetastet.

### Installation

kOS-Archiv in Realism Overhaul ist das Verzeichnis Ships/Script der KSP-Installation.
Script-Dateien muessen auf .ks enden (nicht .kos). Scriptstart im kOS-Terminal mit run.

## Projektstand und Roadmap

- Landing v0.8: abgeschlossen (Script v6.8, in Simulationsfluegen erprobt).
- Launch v0.9.x: aktiv, bislang 46 Kartenfluege der Karriere im Tool ausgewertet;
  aktuelle Entwicklungsstufe ExoPitch v0.9.36.
- TLI v0.9.5 (geplant): Inklination via TLI-Aim, LOI/Capture, Braking-Aimpoint.
- Ascent & Rendezvous v1.0 (geplant): Lander-Aufstieg, Rendezvous mit der TLI-Stufe.
- Ab v1.x: echte explizite Guidance (PEG/E-Guidance-Familie, wie Apollo-LM) fuer
  Meter-genaues Targeting; Web-Tool generiert Missions-Wrapper direkt ins Spiel.

## Rechte

Alle Inhalte dieses Repositories sind geistiges Eigentum des Autors. Es ist keine
Open-Source-Lizenz erteilt; Nutzung, Veroeffentlichung oder Weitergabe ausserhalb
dieses Repositories nur mit ausdruecklicher Zustimmung des Autors.
