// LUNA RSS/RO MOON ONLY v6.8 - LaLunar 0.8 (Datei: lalunar_land.kos)
// Ablauf: Deorbit-Burn -> Coast (RCS aus, 40s vor Bremsbeginn wieder an)
//         -> Descent-Burn ab 15km (descentStart). Bremsung mit Retrograde-Anteil:
//            Die Sinkrate wird dabei ueber ein Profil gehalten (sinkFrac der Hoehe,
//            geklemmt zwischen sinkEnd unten und sinkStart oben), die Restkraft
//            bremst die seitliche Bewegung, bis sie unter horizStop liegt.
//         -> Hover bei 100m (Sideslide, Radar-Avoid, Zielkreuz)
//         -> Finaler Sinkflug per AG5 (Sinkrate per AG7/AG8), Cutoff bei 25cm.
// HINWEIS: 6000m reichen physikalisch nicht, um Orbitalgeschwindigkeit (~1650 m/s)
//          abzubauen - die Bremsung braucht mehrere Minuten und ~500 km Downrange.
//          Deshalb startet der Descent-Burn hoeher (descentStart = 15000).
// Auto: Scheinwerfer ab 15km Hoehe, Landeschenkel-Check/Ausfahren bei 1000m.
// Logs: lalunar_land.csv (Volume 0) bzw. lalunar_land_00X.csv (Archiv-Kopie).
// WICHTIG: Triebwerke muessen VOR dem Bremsbeginn gezundet/gestaged sein,
//          dieses Script zuendet nicht selbst (RO-Ignitions sind begrenzt).
CLEARSCREEN.
SET CONFIG:IPU TO 2000.
SAS OFF. RCS ON.

LOCK g TO SHIP:BODY:MU / (SHIP:BODY:RADIUS + SHIP:ALTITUDE)^2.
LOCK maxThrust TO SHIP:AVAILABLETHRUSTAT(0).
LOCK shipMass TO SHIP:MASS.
LOCK dryMass TO SHIP:DRYMASS.
LOCK fuelMass TO shipMass - dryMass.
LOCK twr TO maxThrust / (shipMass * MAX(0.1,g)).
LOCK horizSpd TO SQRT(SHIP:VELOCITY:SURFACE:NORTH^2 + SHIP:VELOCITY:SURFACE:EAST^2).
LOCK trueAlt TO SHIP:ALTITUDE.
LOCK radarAlt TO ALT:RADAR.

SET targetHov TO 100.
SET maxSpeed TO 3.0.
SET minSpeed TO 0.5.
SET rampTime TO 30.0.
SET slideActive TO FALSE.
SET slideDir TO "NONE".
SET slideStartTime TO 0.
SET targetVelN TO 0.
SET targetVelE TO 0.
SET totalStartMass TO shipMass.
SET lastRadarAlt TO radarAlt.
SET sinkRate TO -1.5.
SET sinkStart TO 30.0.
SET sinkEnd TO 1.5.
SET sinkFrac TO 0.01.
SET descentStart TO 15000.
SET horizStop TO 2.0.
SET attUp TO FALSE.
SET landCmd TO FALSE.
SET timeToHover TO 999.
SET restHoverTime TO 0.
SET lastLogTime TO TIME:SECONDS.
SET lastTerrainTime TO TIME:SECONDS.
SET calibrated TO FALSE.
SET enginesWarned TO FALSE.
SET desiredVS TO 0.
SET lastHudTime TO 0.
SET distN TO 0.
SET distE TO 0.
SET distToTarget TO 0.
// Zielkoordinaten der Landezone - hier anpassen (Fein-Targeting im Hover, max 2 m/s)
SET targetLat TO 0.68.
SET targetLng TO 23.47.

PRINT "BODY: "+SHIP:BODY:NAME+" MU: "+ROUND(SHIP:BODY:MU)+" g: "+ROUND(g,3).
PRINT "=== RO CALIBRATION MOON ===".
SET engList TO LIST().
LIST ENGINES IN engList.
SET GLOBAL_MAX_THRUST TO 0.
SET GLOBAL_MIN_THROTTLE TO 0.35.
SET GLOBAL_ISP TO 310.

FUNCTION calibrateEngines {
  LOCAL tMax IS 0.
  LOCAL tMin IS 1.0.
  LOCAL ispSum IS 0.
  LOCAL n IS 0.
  FOR eng IN engList {
    IF eng:IGNITION {
      SET tMax TO tMax + eng:AVAILABLETHRUST.
      LOCAL eMin IS eng:MINTHRUST / MAX(0.01, eng:MAXTHRUST).
      IF eMin < tMin { SET tMin TO eMin. }
      SET ispSum TO ispSum + eng:ISPAT(0).
      SET n TO n + 1.
    }
  }
  IF n > 0 {
    SET GLOBAL_MIN_THROTTLE TO MAX(0.25, tMin + 0.05).
    SET GLOBAL_MAX_THRUST TO tMax.
    SET GLOBAL_ISP TO ispSum / n.
    PRINT "Kalibriert ("+n+" Triebwerke): Thrust "+ROUND(tMax)+" Min "+ROUND(GLOBAL_MIN_THROTTLE*100)+"% Isp "+ROUND(GLOBAL_ISP).
  }
}

// Erster Versuch: falls Triebwerke schon brennen, sonst spaeter im Loop.
calibrateEngines().
IF GLOBAL_MAX_THRUST > 0 { SET calibrated TO TRUE. }

PRINT "Thrust "+ROUND(GLOBAL_MAX_THRUST)+" Min "+ROUND(GLOBAL_MIN_THROTTLE*100)+"% Isp "+ROUND(GLOBAL_ISP)+" TWR "+ROUND(maxThrust/(shipMass*MAX(0.1,g)),2).
WAIT 1.

SET logFile TO "0:lalunar_land.csv".
IF EXISTS(logFile) {
  DELETEPATH(logFile).
}
LOG "time,phase,event,mass_total,mass_fuel,twr,vertSpd,horizSpd,orbSpd,trueAlt,radarAlt,targetVelN,targetVelE,slideDir,throttle,isp,timeToHover,restHoverTime,sinkRate,lat,lng" TO logFile.

FUNCTION logData {
  PARAMETER phase, event, tth, rht.
  LOG ROUND(TIME:SECONDS,2)+","+phase+","+event+","+ROUND(shipMass,2)+","+ROUND(fuelMass,2)+","+ROUND(twr,3)+","+ROUND(SHIP:VERTICALSPEED,2)+","+ROUND(horizSpd,2)+","+ROUND(SHIP:VELOCITY:ORBIT:MAG,1)+","+ROUND(trueAlt,1)+","+ROUND(radarAlt,1)+","+ROUND(targetVelN,2)+","+ROUND(targetVelE,2)+","+slideDir+","+ROUND(THROTTLE,3)+","+ROUND(GLOBAL_ISP,1)+","+ROUND(tth,1)+","+ROUND(rht,1)+","+ROUND(sinkRate,2)+","+ROUND(SHIP:GEOPOSITION:LAT,5)+","+ROUND(SHIP:GEOPOSITION:LNG,5) TO logFile.
}

FUNCTION saveLogIncremental {
  LOCAL n IS 1.
  LOCAL fname IS "".
  UNTIL FALSE {
    LOCAL numStr IS ""+n.
    IF n < 10 { SET numStr TO "00"+n. }
    ELSE IF n < 100 { SET numStr TO "0"+n. }
    SET fname TO "0:lalunar_land_"+numStr+".csv".
    IF NOT EXISTS(fname) { BREAK. }
    SET n TO n+1.
    IF n > 999 {
      SET fname TO "0:lalunar_land_"+ROUND(TIME:SECONDS,0)+".csv".
      BREAK.
    }
  }
  IF VOLUME(0):EXISTS("lalunar_land.csv") {
    COPYPATH("0:lalunar_land.csv", fname).
    PRINT "Log gespeichert: "+fname.
  }
  RETURN fname.
}

FUNCTION getSlideSpeed {
  IF NOT slideActive {
    RETURN 0.
  }
  RETURN MIN(maxSpeed, minSpeed + (maxSpeed - minSpeed) * MIN(TIME:SECONDS - slideStartTime, rampTime) / rampTime).
}

FUNCTION startSlide {
  PARAMETER dir.
  IF landCmd {
    RETURN.
  }
  IF slideDir = dir {
    SET slideActive TO FALSE.
    SET slideDir TO "NONE".
    SET targetVelN TO 0.
    SET targetVelE TO 0.
    logData("P4_HOVER","SLIDE_STOP_"+dir,0,restHoverTime).
  } ELSE {
    SET slideActive TO TRUE.
    SET slideDir TO dir.
    SET slideStartTime TO TIME:SECONDS.
    logData("P4_HOVER","SLIDE_START_"+dir,0,restHoverTime).
  }
}

// Radar-Bodenvermessung: echte Vergangenheitsdeltas, kein festes Intervall.
FUNCTION checkTerrainAvoid {
  LOCAL dt IS TIME:SECONDS - lastTerrainTime.
  IF dt < 0.1 { RETURN FALSE. }
  LOCAL riseRate IS (lastRadarAlt - radarAlt) / dt.
  IF riseRate > 12 AND radarAlt < 120 AND NOT landCmd {
    SET targetHov TO targetHov + 40.
    logData("P4_HOVER","AUTO_AVOID_UP_"+ROUND(targetHov),0,restHoverTime).
    SET lastRadarAlt TO radarAlt.
    SET lastTerrainTime TO TIME:SECONDS.
    RETURN TRUE.
  }
  IF radarAlt < 50 AND slideActive {
    SET slideActive TO FALSE.
    SET targetVelN TO 0.
    SET targetVelE TO 0.
    SET targetHov TO 120.
    logData("P4_HOVER","AUTO_AVOID_STOP",0,restHoverTime).
    SET lastRadarAlt TO radarAlt.
    SET lastTerrainTime TO TIME:SECONDS.
    RETURN TRUE.
  }
  SET lastRadarAlt TO radarAlt.
  SET lastTerrainTime TO TIME:SECONDS.
  RETURN FALSE.
}

// HUD im kOS-Terminal: 4 Statuszeilen, alle 0.5s aktualisiert.
FUNCTION hudUpdate {
  LOCAL ph IS "INIT".
  IF runmode = 1 { SET ph TO "P1_DEORBIT". }
  IF runmode = 2 { SET ph TO "P2_COAST". }
  IF runmode = 3 { SET ph TO "P3_BRAKE". }
  IF runmode = 4 { SET ph TO "P4_HOVER". }
  IF runmode = 5 { SET ph TO "P5_FINAL". }
  PRINT "T+"+ROUND(TIME:SECONDS,0)+"s  "+ph+"  ALT "+ROUND(radarAlt)+"m" AT (0,10).
  PRINT "VS "+ROUND(SHIP:VERTICALSPEED,1)+" (Soll "+ROUND(desiredVS,1)+")  HSPD "+ROUND(horizSpd,1)+" m/s" AT (0,11).
  PRINT "THR "+ROUND(THROTTLE*100)+"%  TWR "+ROUND(twr,2)+"  Fuel "+ROUND(fuelMass)+"kg  Rest "+ROUND(restHoverTime)+"s" AT (0,12).
  PRINT "Ziel "+ROUND(targetLat,3)+"/"+ROUND(targetLng,3)+"  DTG "+ROUND(distToTarget)+"m" AT (0,13).
}

SET runmode TO 1.

UNTIL runmode = 0 {
  IF AG1 {
    startSlide("NORTH").
    AG1 OFF.
    WAIT 0.2.
  }
  IF AG2 {
    startSlide("SOUTH").
    AG2 OFF.
    WAIT 0.2.
  }
  IF AG3 {
    startSlide("WEST").
    AG3 OFF.
    WAIT 0.2.
  }
  IF AG4 {
    startSlide("EAST").
    AG4 OFF.
    WAIT 0.2.
  }
  IF AG7 {
    IF landCmd {
      SET sinkRate TO sinkRate - 0.25.
      IF sinkRate < -3.0 {
        SET sinkRate TO -3.0.
      }
      logData("P5_FINAL","SINK_FASTER_"+ROUND(sinkRate,2),0,restHoverTime).
    } ELSE {
      SET targetHov TO targetHov + 25.
      logData("P4_HOVER","H_+25_"+targetHov,0,restHoverTime).
    }
    AG7 OFF.
    WAIT 0.2.
  }
  IF AG8 {
    IF landCmd {
      SET sinkRate TO sinkRate + 0.25.
      IF sinkRate > -0.1 {
        SET sinkRate TO -0.1.
      }
      logData("P5_FINAL","SINK_SLOWER_"+ROUND(sinkRate,2),0,restHoverTime).
    } ELSE {
      SET targetHov TO MAX(20, targetHov - 25).
      logData("P4_HOVER","H_-25_"+targetHov,0,restHoverTime).
    }
    AG8 OFF.
    WAIT 0.2.
  }
  IF AG5 {
    SET landCmd TO TRUE.
    IF NOT LEGS {
      LEGS ON.
      logData("P4_HOVER","LEGS_CHECK_ON",0,restHoverTime).
    }
    SET runmode TO 5.
    AG5 OFF.
    logData("P4_HOVER","LAND_CMD",0,restHoverTime).
    SET lastLogTime TO TIME:SECONDS.
  }
  IF AG6 {
    logData("ABORT","ABORT_AG6",0,restHoverTime).
    AG6 ON.
    RCS ON.
    LOCK THROTTLE TO 1.
    LOCK STEERING TO UP.
    WAIT 5.
    LOCK STEERING TO HEADING(90,60).
    WAIT UNTIL SHIP:APOAPSIS > 25000.
    LOCK THROTTLE TO 0.
    AG6 OFF.
    SHUTDOWN.
  }

  // Nachkalibrierung sobald Triebwerke tatsaechlich Schub liefern.
  IF NOT calibrated AND maxThrust > 0 {
    calibrateEngines().
    IF GLOBAL_MAX_THRUST > 0 {
      SET calibrated TO TRUE.
      logData("INIT","ENGINE_CALIBRATED",0,restHoverTime).
      SET lastLogTime TO TIME:SECONDS.
    }
  }

  // Scheinwerfer ab 15km, Landeschenkel ab 1000m (automatisch)
  IF NOT LIGHTS AND radarAlt < 15000 {
    LIGHTS ON.
    logData("SYSTEM","LIGHTS_ON_15KM",0,restHoverTime).
  }
  IF NOT LEGS AND radarAlt < 1000 {
    LEGS ON.
    logData("SYSTEM","LEGS_AUTO_1000M",0,restHoverTime).
  }

  // Distanz zur Landezone (fuer HUD, Hover-Fein-Targeting und Miss-Log)
  SET lngDiff TO targetLng - SHIP:GEOPOSITION:LNG.
  IF lngDiff > 180 { SET lngDiff TO lngDiff - 360. }
  IF lngDiff < -180 { SET lngDiff TO lngDiff + 360. }
  SET distN TO (targetLat - SHIP:GEOPOSITION:LAT) * 30400.
  SET distE TO lngDiff * 30400 * COS(SHIP:GEOPOSITION:LAT).
  SET distToTarget TO SQRT(distN*distN + distE*distE).

  IF TIME:SECONDS - lastHudTime >= 0.5 {
    hudUpdate().
    SET lastHudTime TO TIME:SECONDS.
  }

  SET currSpeed TO getSlideSpeed().
  IF slideActive {
    IF slideDir = "NORTH" {
      SET targetVelN TO currSpeed.
      SET targetVelE TO 0.
    }
    IF slideDir = "SOUTH" {
      SET targetVelN TO -currSpeed.
      SET targetVelE TO 0.
    }
    IF slideDir = "WEST" {
      SET targetVelN TO 0.
      SET targetVelE TO -currSpeed.
    }
    IF slideDir = "EAST" {
      SET targetVelN TO 0.
      SET targetVelE TO currSpeed.
    }
  } ELSE {
    // Kein Slide: Im Hover Zielposition halten (Fein-Targeting, max 2 m/s).
    IF runmode = 4 AND NOT landCmd {
      SET targetVelN TO MAX(-2, MIN(2, distN * 0.02)).
      SET targetVelE TO MAX(-2, MIN(2, distE * 0.02)).
    } ELSE {
      SET targetVelN TO 0.
      SET targetVelE TO 0.
    }
  }

  // Rest-Schwebezeit: Treibstoff / Verbrauch im Schwebezustand (Gewicht/(Isp*g0)).
  SET hoverFlow TO (shipMass * MAX(0.01, g)) / MAX(1, GLOBAL_ISP * 9.81).
  IF hoverFlow > 0.001 {
    SET restHoverTime TO MAX(0, fuelMass) / hoverFlow.
  } ELSE {
    SET restHoverTime TO 0.
  }
  IF SHIP:VERTICALSPEED < -0.1 {
    SET timeToHover TO (radarAlt - targetHov) / -SHIP:VERTICALSPEED.
    IF timeToHover < 0 {
      SET timeToHover TO 0.
    }
  } ELSE {
    SET timeToHover TO 0.
  }

  IF runmode = 1 {
    LOCK STEERING TO RETROGRADE.
    LOCK THROTTLE TO 0.5.
    IF SHIP:PERIAPSIS < 1000 {
      LOCK THROTTLE TO 0.
      RCS OFF.
      SET runmode TO 2.
      logData("P1_DEORBIT","DONE",timeToHover,restHoverTime).
      SET lastLogTime TO TIME:SECONDS.
    }
    IF TIME:SECONDS - lastLogTime >= 1 {
      logData("P1_DEORBIT","-",timeToHover,restHoverTime).
      SET lastLogTime TO TIME:SECONDS.
    }
  }
  IF runmode = 2 {
    // RCS frueh genug wieder aktivieren, damit der Lander zum Bremsen ausrichten kann.
    IF NOT RCS {
      SET vsDown TO -SHIP:VERTICALSPEED.
      SET tToBurn TO 9999.
      IF vsDown > 1 {
        SET tToBurn TO (radarAlt - descentStart) / vsDown.
      }
      IF tToBurn < 40 OR radarAlt < descentStart + 500 {
        RCS ON.
        logData("P2_COAST","RCS_ON_ALIGN",timeToHover,restHoverTime).
        SET lastLogTime TO TIME:SECONDS.
      }
    }
    IF radarAlt < descentStart {
      SET runmode TO 3.
      logData("P2_COAST","DONE",timeToHover,restHoverTime).
      SET lastLogTime TO TIME:SECONDS.
    }
    IF TIME:SECONDS - lastLogTime >= 1 {
      logData("P2_COAST","-",timeToHover,restHoverTime).
      SET lastLogTime TO TIME:SECONDS.
    }
  }
  IF runmode = 3 {
    IF maxThrust = 0 AND NOT enginesWarned {
      SET enginesWarned TO TRUE.
      PRINT "WARNUNG: Kein Schub - Triebwerke zaenden/stagen!" AT (0,14).
      logData("P3_POWERED","NO_THRUST_WARNING",timeToHover,restHoverTime).
    }
    // Sinkprofil: sinkFrac der Hoehe, geklemmt zwischen sinkEnd (unten) und sinkStart (oben).
    SET desiredVS TO -(MAX(sinkEnd, MIN(sinkStart, radarAlt * sinkFrac))).
    // Vertikal-Befehl: Gewicht halten + Sinkraten-Fehler.
    SET vAccCmd TO (shipMass * g) + (shipMass * (desiredVS - SHIP:VERTICALSPEED) * 0.05).
    SET vFrac TO MIN(0.95, MAX(0.1, vAccCmd / MAX(1, maxThrust))).
    IF horizSpd > horizStop {
      // Retrograde-Anteil: Schub schraeg gegen die seitliche Bewegung, gerade genug
      // Aufwaertsanteil, um die Sinkrate am Profil zu halten. Restkraft bremst seitlich.
      SET steerUp TO horizSpd * vFrac / SQRT(1 - vFrac*vFrac).
      SET hvN TO SHIP:VELOCITY:SURFACE:NORTH.
      SET hvE TO SHIP:VELOCITY:SURFACE:EAST.
      SET steerVec TO (UP:VECTOR * steerUp) - (NORTH:VECTOR * hvN) - (VCRS(UP:VECTOR, NORTH:VECTOR) * hvE).
      LOCK STEERING TO LOOKDIRUP(steerVec, UP:VECTOR).
      SET attUp TO FALSE.
    } ELSE {
      LOCK STEERING TO UP.
      SET vFrac TO 1.0.
      IF NOT attUp {
        SET attUp TO TRUE.
        logData("P3_POWERED","ATTITUDE_UP",timeToHover,restHoverTime).
      }
    }
    LOCK THROTTLE TO MIN(1.0, MAX(GLOBAL_MIN_THROTTLE, vAccCmd / MAX(1, maxThrust * vFrac))).
    IF (radarAlt <= targetHov AND horizSpd < horizStop) OR radarAlt < 60 {
      SET runmode TO 4.
      logData("P3_POWERED","DONE",timeToHover,restHoverTime).
      SET lastLogTime TO TIME:SECONDS.
    }
    IF TIME:SECONDS - lastLogTime >= 1 {
      logData("P3_POWERED","-",timeToHover,restHoverTime).
      SET lastLogTime TO TIME:SECONDS.
    }
  }
  IF runmode = 4 {
    SET desiredVS TO 0.
    IF TIME:SECONDS - lastTerrainTime >= 0.5 {
      checkTerrainAvoid().
    }
    SET currVelN TO SHIP:VELOCITY:SURFACE:NORTH.
    SET currVelE TO SHIP:VELOCITY:SURFACE:EAST.
    SET errN TO targetVelN - currVelN.
    SET errE TO targetVelE - currVelE.
    SET tiltNorth TO MAX(-6, MIN(6, errN * 2.0)).
    SET tiltEast TO MAX(-6, MIN(6, errE * 2.0)).
    SET tiltVec TO UP:VECTOR + (NORTH:VECTOR * tiltNorth * 0.01745) + (VCRS(UP:VECTOR, NORTH:VECTOR) * tiltEast * 0.01745).
    LOCK STEERING TO LOOKDIRUP(tiltVec, UP:VECTOR).
    SET altErr TO targetHov - radarAlt.
    SET baseThrott TO (shipMass * g) / MAX(1, maxThrust).
    SET hoverThrott TO baseThrott + (altErr * 0.015) - (SHIP:VERTICALSPEED * 0.08).
    LOCK THROTTLE TO MIN(1.0, MAX(GLOBAL_MIN_THROTTLE, hoverThrott)).
    SET predLat TO SHIP:GEOPOSITION:LAT + (targetVelN * 10 / 30400).
    SET predLng TO SHIP:GEOPOSITION:LNG + (targetVelE * 10 / (30400 * COS(SHIP:GEOPOSITION:LAT))).
    SET dLatM TO (predLat - SHIP:GEOPOSITION:LAT) * 30400.
    SET dLngM TO (predLng - SHIP:GEOPOSITION:LNG) * 30400 * COS(SHIP:GEOPOSITION:LAT).
    SET distToPred TO SQRT(dLatM^2 + dLngM^2).
    PRINT "Slide "+slideDir+" "+ROUND(currSpeed,2)+"m/s | Cross10s "+ROUND(distToPred)+"m -> "+ROUND(predLat,4)+"/"+ROUND(predLng,4)+" Fuel "+ROUND(fuelMass)+"kg" AT (0,14).
    IF TIME:SECONDS - lastLogTime >= 1 {
      logData("P4_HOVER","HOVER",timeToHover,restHoverTime).
      SET lastLogTime TO TIME:SECONDS.
    }
  }
  IF runmode = 5 {
    SET slideActive TO FALSE.
    SET targetVelN TO 0.
    SET targetVelE TO 0.
    SET desiredVS TO sinkRate.
    IF radarAlt < 25 AND sinkRate < -1.0 {
      SET desiredVS TO -1.0.
    }
    IF radarAlt < 5 AND sinkRate < -0.5 {
      SET desiredVS TO -0.5.
    }
    IF radarAlt < 0.25 {
      LOCK THROTTLE TO 0.
      logData("P5_FINAL","DUST_CUT_0.25M",0,restHoverTime).
      SET lastLogTime TO TIME:SECONDS.
      WAIT 0.2.
      LOCK STEERING TO UP.
      SAS ON.
      RCS OFF.
      SET runmode TO 0.
    } ELSE {
      SET vsErr TO desiredVS - SHIP:VERTICALSPEED.
      SET baseThrott TO (shipMass * g) / MAX(1, maxThrust).
      SET finalThrott TO baseThrott + (vsErr * 0.1).
      LOCK THROTTLE TO MIN(1.0, MAX(GLOBAL_MIN_THROTTLE, finalThrott)).
    }
    IF TIME:SECONDS - lastLogTime >= 1 {
      logData("P5_FINAL","DESCENT",0,restHoverTime).
      SET lastLogTime TO TIME:SECONDS.
    }
  }
  WAIT 0.05.
}

logData("LANDED","TOUCHDOWN",0,restHoverTime).
logData("LANDED","TARGET_MISS_"+ROUND(distToTarget,0),0,restHoverTime).
PRINT "LANDED Moon - Verbrauch: "+ROUND(totalStartMass - SHIP:MASS,1)+"kg | Ziel-Abweichung: "+ROUND(distToTarget,0)+"m".
PRINT "Speichere Log...".
saveLogIncremental().
LOCK THROTTLE TO 0.
SAS ON.
SHUTDOWN.