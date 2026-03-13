// ####################################################################################################################
// 
// Code part of CarCluster project by Andrej Rolih. See .ino file more details

// re-edit by Xiaobai151 when 2026
// 
// ####################################################################################################################

#include "BMWFSeriesCluster.h"

BMWFSeriesCluster::BMWFSeriesCluster(MCP_CAN& CAN, bool isCarMini): CAN(CAN) {
  this->isCarMini = isCarMini;

  if (isCarMini) {
    inFuelRange[0] = 0; inFuelRange[1] = 50; inFuelRange[2] = 100;
    outFuelRange[0] = 22; outFuelRange[1] = 7; outFuelRange[2] = 3;
  } else {
    inFuelRange[0] = 0; inFuelRange[1] = 50; inFuelRange[2] = 100;
    outFuelRange[0] = 37; outFuelRange[1] = 18; outFuelRange[2] = 4;
  }
  crc8Calculator.begin();
}

uint8_t BMWFSeriesCluster::mapGenericGearToLocalGear(GearState inputGear) {
  // The gear that the car is in: 0 = clear, 1-9 = M1-M9, 10 = P, 11 = R, 12 = N, 13 = D
  switch(inputGear) {
    case GearState_Manual_1: return 1;
    case GearState_Manual_2: return 2;
    case GearState_Manual_3: return 3;
    case GearState_Manual_4: return 4;
    case GearState_Manual_5: return 5;
    case GearState_Manual_6: return 6;
    case GearState_Manual_7: return 7;
    case GearState_Manual_8: return 8;
    case GearState_Manual_9: return 9;

    case GearState_Auto_P: return 10;
    case GearState_Auto_R: return 11;
    case GearState_Auto_N: return 12;

    // D and S both map to base auto gear (cluster mode handled elsewhere)
    case GearState_Auto_D:
    case GearState_Auto_S:
      return 13;

    default:
      return 0;   // clear / unknown
  }
}

int BMWFSeriesCluster::mapSpeed(GameState& game) {
  int scaledSpeed = game.speed * game.configuration.speedCorrectionFactor;
  if (scaledSpeed > game.configuration.maximumSpeedValue) {
    return game.configuration.maximumSpeedValue;
  } else {
    return scaledSpeed;
  }
}

int BMWFSeriesCluster::mapRPM(GameState& game) {
  int scaledRPM = game.rpm * game.configuration.rpmCorrectionFactor;
  if (scaledRPM > game.configuration.maximumRPMValue) {
    return game.configuration.maximumRPMValue;
  } else {
    return scaledRPM;
  }
}


int BMWFSeriesCluster::mapCoolantTemperature(GameState& game) {
  if (game.coolantTemperature < game.configuration.minimumCoolantTemperature) { return game.configuration.minimumCoolantTemperature; }
  if (game.coolantTemperature > game.configuration.maximumCoolantTemperature) { return game.configuration.maximumCoolantTemperature; }
  return game.coolantTemperature;
}

void BMWFSeriesCluster::updateWithGame(GameState& game) {
  sendFixedLIM();

  // ============================
  // Immediate steering wheel button processing
  // Process button events without waiting for 1s dashboard loop
  // ============================
  if (game.buttonEventToProcess != 0) {
    sendSteeringWheelButton(game.buttonEventToProcess);
    game.buttonEventToProcess = 0;
  }

  // Re-send language/units aggressively for a short window after ignition ON.
  // Many F-series clusters only latch 0x291 during startup.
  static bool lastIgnitionForSettings = false;
  static unsigned long settingsBurstStart = 0;

  if (game.ignition && !lastIgnitionForSettings) {
    settingsBurstStart = millis();
  }
  lastIgnitionForSettings = game.ignition;

  // ============================================
  // Traction monitor → force DSC OFF if active >2s
  // Using offroadLight as traction indicator proxy
  // ============================================
  static unsigned long tractionStartTime = 0;
  static bool tractionTriggered = false;
  static bool dscForced = false;

  bool tractionNow = game.offroadLight;

  if (tractionNow) {

    if (!tractionTriggered) {
      tractionStartTime = millis();
      tractionTriggered = true;
    }

    if (!dscForced && (millis() - tractionStartTime >= 2000)) {
      dscForced = true;
    }

  } else {

    tractionTriggered = false;
    tractionStartTime = 0;

    // If traction flashed briefly (<2s) treat as intervention event
    if (tractionTriggered && !dscForced) {
      uint8_t msg184[] = { 0x40, 184, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5C0, 0, 8, msg184);
    }

    // 如果防滑灯刚刚熄灭 → 立即清除DSC状态
    if (dscForced) {

      uint8_t ids[] = {36, 42, 215, 35, 204};

      for (uint8_t i = 0; i < sizeof(ids); i++) {

        uint8_t message[] = {
          0x40,
          ids[i],
          0x00,
          0x28,   // OFF
          0xFF,
          0xFF,
          0xFF,
          0xFF
        };

        CAN.sendMsgBuf(0x5C0, 0, 8, message);
      }
    }

    dscForced = false;
  }
  

  static unsigned long engineStartTime = 0;
  static bool engineWasRunning = false;
  static bool can46Sent = false;
  if (millis() - lastDashboardUpdateTime >= dashboardUpdateTime100) {



    // This should probably be done using a more sophisticated method like a
    // scheduler, but for now this seems to work.

    sendIgnitionStatus(game.ignition);
    // ============================================
    // AUTO HOLD (CC-ID 58) – cyclic while active
    // Requirement:
    // - ignition ON
    // - speed == 0 for >= 2 seconds
    // - if speed changes (or ignition OFF) -> clear immediately
    // NOTE: We do NOT use CC-ID 48 at all.
    // ============================================

    static unsigned long zeroSpeedStartTime = 0;
    static bool autoHoldActive = false;

    bool wantAutoHold = false;

    if (game.ignition) {
      if (game.speed == 0) {
        if (zeroSpeedStartTime == 0) {
          zeroSpeedStartTime = millis();
        }
        if (millis() - zeroSpeedStartTime >= 2000) {
          wantAutoHold = true;
        }
      } else {
        // Speed changed -> reset timer and request OFF
        zeroSpeedStartTime = 0;
        wantAutoHold = false;
      }
    } else {
      // Ignition OFF -> reset timer and request OFF
      zeroSpeedStartTime = 0;
      wantAutoHold = false;
    }

    if (wantAutoHold) {
      // Keep-alive: must be sent cyclic or cluster will drop the icon
      uint8_t msg58_on[] = { 0x40, 58, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5C0, 0, 8, msg58_on);
      autoHoldActive = true;
    } else {
      // Send OFF once on state change
      if (autoHoldActive) {
        uint8_t msg58_off[] = { 0x40, 58, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
        CAN.sendMsgBuf(0x5C0, 0, 8, msg58_off);
        autoHoldActive = false;
      }
    }
    // =====================================================
    // Auto High Beam (0x36A) – cyclic
    // Auto Start/Stop (0x30B) – cyclic
    // Must be sent continuously or cluster will drop icon
    // =====================================================

    // ---------- 0x36A Automatic High Beam ----------
    {
      uint8_t ahbFrame[8];
      uint8_t ahbVal = game.highBeam ? 0x02 : 0x01;   // 0x02 = ON only when high beam active

      for (int i = 0; i < 8; i++) {
        ahbFrame[i] = ahbVal;
      }

      CAN.sendMsgBuf(0x36A, 0, 8, ahbFrame);
    }

    // ---------- 0x30B Auto Start/Stop ----------
    {
      uint8_t assFrame[8];
      uint8_t assVal = game.ignition ? 0x1A : 0xE6;   // adjust if capture differs

      for (int i = 0; i < 8; i++) {
        assFrame[i] = assVal;
      }

      CAN.sendMsgBuf(0x30B, 0, 8, assFrame);
    }
    // ===============================
    // Engine start detection (ONLY ignition based)
    // ===============================
    if (game.ignition) {
      if (!engineWasRunning) {
        engineStartTime = millis();
        engineWasRunning = true;
        can46Sent = false;
      }

      if (!can46Sent && millis() - engineStartTime >= 10000) {
        uint8_t msgSet[] = { 0x40, 91, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
        CAN.sendMsgBuf(0x5C0, 0, 8, msgSet);

        // delay(50);

        uint8_t msgClear[] = { 0x40, 91, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
        CAN.sendMsgBuf(0x5C0, 0, 8, msgClear);

        // CAN 53 (engine start related)
        uint8_t msg53_set[] = { 0x40, 53, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
        CAN.sendMsgBuf(0x5C0, 0, 8, msg53_set);

        // delay(50);

        uint8_t msg53_clear[] = { 0x40, 53, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
        CAN.sendMsgBuf(0x5C0, 0, 8, msg53_clear);

        // CAN 181 (constant after engine start)
        uint8_t msg181_const[] = { 0x40, 181, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
        CAN.sendMsgBuf(0x5C0, 0, 8, msg181_const);

        // ===============================
        // Seatbelt self-check after engine start (10s trigger)
        // Front seatbelt (example CC-ID 71)
        // Rear seatbelt (example CC-ID 72)
        // Adjust IDs if your cluster uses different ones
        // ===============================

        uint8_t seatbeltFront_on[]  = { 0x40, 71, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
        CAN.sendMsgBuf(0x5C0, 0, 8, seatbeltFront_on);

        uint8_t seatbeltRear_on[]   = { 0x40, 72, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
        CAN.sendMsgBuf(0x5C0, 0, 8, seatbeltRear_on);

        // Small delay between ON and OFF simulation
        delay(50);

        uint8_t seatbeltFront_off[] = { 0x40, 71, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
        CAN.sendMsgBuf(0x5C0, 0, 8, seatbeltFront_off);

        uint8_t seatbeltRear_off[]  = { 0x40, 72, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
        CAN.sendMsgBuf(0x5C0, 0, 8, seatbeltRear_off);

        can46Sent = true;
      }
    } else {
      engineWasRunning = false;
      can46Sent = false;
    }
    // ===============================
    // Engine start warning (ID 40)
    // Ignition ON but RPM == 0 → show
    // Otherwise → clear
    // ===============================
    if (game.ignition && game.rpm < 10) {
      uint8_t msg40[] = { 0x40, 40, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5C0, 0, 8, msg40);
    } else {
      uint8_t msg40[] = { 0x40, 40, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5C0, 0, 8, msg40);
    }
    // CAN 41 (same logic as ID 40)
    if (game.ignition && game.rpm < 10) {
      uint8_t msg41[] = { 0x40, 41, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5C0, 0, 8, msg41);
    } else {
      uint8_t msg41[] = { 0x40, 41, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5C0, 0, 8, msg41);
    }
    // ==============================
    // CRUISE CONTROL (Simple mode)
    // ==============================

    static bool lastCruiseActive = false;
    static int lastCruiseSpeed = 0;

    if (game.cruiseControlActive) {

      uint8_t cruiseOn[] = { 0x40, 0x2F, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5C0, 0, 8, cruiseOn);

    } else {

      uint8_t cruiseOff[] = { 0x40, 0x2F, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5C0, 0, 8, cruiseOff);

      // Cruise cancelled while vehicle slowing → trigger message 59
      if (lastCruiseActive && game.speed < lastCruiseSpeed) {
        uint8_t msg59[] = { 0x40, 59, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
        CAN.sendMsgBuf(0x5C0, 0, 8, msg59);
      }
    }

    lastCruiseActive = game.cruiseControlActive;
    lastCruiseSpeed = game.speed;

 

    // ===============================
    // Ignition ON but engine not started logic (with hysteresis + state cache)
    // ===============================
    static bool lastEngineStoppedState = false;

    // Hysteresis thresholds
    bool engineStoppedNow = false;

    if (game.ignition) {
      if (game.rpm < 50) {
        engineStoppedNow = true;
      } else if (game.rpm > 150) {
        engineStoppedNow = false;
      } else {
        // Between 50–150 rpm → keep previous state (hysteresis zone)
        engineStoppedNow = lastEngineStoppedState;
      }
    } else {
      engineStoppedNow = false;
    }

    if (engineStoppedNow != lastEngineStoppedState) {

      uint8_t ids[] = {213, 220, 21, 24, 30, 175, 206, 255};

      for (uint8_t i = 0; i < sizeof(ids); i++) {
        uint8_t msg[] = { 
          0x40, 
          ids[i], 
          0x00, 
          engineStoppedNow ? 0x29 : 0x28, 
          0xFF, 
          0xFF, 
          0xFF, 
          0xFF 
        };
        CAN.sendMsgBuf(0x5C0, 0, 8, msg);
      }

      lastEngineStoppedState = engineStoppedNow;
    }
    sendSpeed(mapSpeed(game));





    sendRPM(mapRPM(game), mapGenericGearToLocalGear(game.gear));
    sendBasicDriveInfo(game, game.oilTemperature);
    // -------------------------------------------------
    // Base ECU keep‑alive frames (BDC / Gateway / Vehicle status)
    // These frames simulate missing modules required for a stable
    // F‑series cluster environment.
    // -------------------------------------------------
    {
      // Vehicle status ECU
      unsigned char vehicleStatus[8] = {0xFF, 0xFF, 0xC0, 0xFF, 0xFF, 0xFF, 0xF0, (uint8_t)random(0xFC,0xFD)};
      CAN.sendMsgBuf(0x3A0, 0, 8, vehicleStatus);

      // Body controller (BDC / FEM)
      unsigned char bodyController[8] = {0x00, count, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      CAN.sendMsgBuf(0xB68, 0, 8, bodyController);

      // Gateway keep‑alive
      unsigned char gatewayFrame[2] = {0x79, 0x20};
      CAN.sendMsgBuf(0x381, 0, 2, gatewayFrame);
    }

    // -------------------------------------------------
    // Additional chassis modules required by many F-series clusters
    // ICM (Integrated Chassis Management), Steering Angle,
    // Wheel Speed broadcast and Power/Battery ECU.
    // -------------------------------------------------

    // ----- ICM (Integrated Chassis Management) -----
    {
      uint8_t icmFrame[8] = {
        (uint8_t)(0xF0 | counter4Bit),
        0x00,
        0x00,
        0x80,
        0x00,
        0x00,
        0x00,
        0x00
      };
      CAN.sendMsgBuf(0x130, 0, 8, icmFrame);
    }

    // ----- Steering Angle ECU (SZL simulation) -----
    {
      uint8_t steeringAngleFrame[8] = {
        (uint8_t)(0xF0 | counter4Bit),
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00
      };
      CAN.sendMsgBuf(0x0C4, 0, 8, steeringAngleFrame);
    }

    // ----- Wheel speed broadcast (DSC dependent modules expect it) -----
    {
      uint8_t wheelSpeedFrame[8] = {
        (uint8_t)(0xF0 | counter4Bit),
        0x10,
        0x10,
        0x10,
        0x10,
        0x00,
        0x00,
        0x00
      };
      CAN.sendMsgBuf(0x0AA, 0, 8, wheelSpeedFrame);
    }

    // ----- Power / Battery management ECU -----
    {
      uint8_t batteryFrame[8] = {
        (uint8_t)(0xF0 | counter4Bit),
        0x64,
        0x64,
        0x64,
        0x64,
        0x64,
        0x64,
        0x64
      };
      CAN.sendMsgBuf(0x3D0, 0, 8, batteryFrame);
    }
    sendAutomaticTransmission(game.gear, game.gearIndex);
    // ===============================
    // N gear cyclic frame (ID 0x178)
    // Must be sent continuously while in N
    // ===============================
    if (game.gear == GearState_Auto_N) {
      unsigned char neutralWithoutCRC[] = { 0xF0 | counter4Bit, 0x60, 0xFC, 0xFF };
      unsigned char neutralWithCRC[] = {
        crc8Calculator.get_crc8(neutralWithoutCRC, 4, 0x5A),
        neutralWithoutCRC[0],
        neutralWithoutCRC[1],
        neutralWithoutCRC[2],
        neutralWithoutCRC[3]
      };
      CAN.sendMsgBuf(0x178, 0, 5, neutralWithCRC);

      // CC-ID 169 (N gear related)
      uint8_t msg169_on[] = { 0x40, 169, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5C0, 0, 8, msg169_on);

      // CC-ID 203 (N gear related)
      uint8_t msg203_on[] = { 0x40, 203, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5C0, 0, 8, msg203_on);
    }
    else {
      uint8_t msg169_off[] = { 0x40, 169, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5C0, 0, 8, msg169_off);

      uint8_t msg203_off[] = { 0x40, 203, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5C0, 0, 8, msg203_off);
    }
    // ===============================
    // Fuel gauge integration (0–100%)
    // Convert BeamNG 0.0–1.0 fuel ratio to 0–100 percentage
    // ===============================
    // BeamNG fuelQuantity is 0.0–1.0 (1.0 = full tank)
    sendFuel(game.fuelQuantity, inFuelRange, outFuelRange, isCarMini);
    sendParkBrake(game.handbrake);
    sendDistanceTravelled(mapSpeed(game));
    // ===============================
    // Overspeed >220 km/h → force doorOpen trigger
    // ===============================
    if (game.speed > 220) {
      game.doorOpen = true;
    }
    sendAlerts(game, game.offroadLight, game.handbrake, isCarMini);

    // ===============================
    // DSC / Stability Control (OEM style)
    // ===============================
    // 2) DSC fully disabled → use driveMode = 6 (DSC OFF lamp via 0x3A7)


    // ===============================
    // Parking brake extra alerts
    // NOTE: Do NOT send CCID 48 here (reserved for AutoHold logic above)
    // ===============================
    // (kept empty intentionally)

    // ===============================
    // Overspeed >120km/h → CAN62
    // 如果想模拟Beamng里的超速提示 把这个解除注释可以显示 当速度超过120则显示超速
    // ===============================
    // if (game.speed > 120) {
    //   uint8_t msg62_on[] = { 0x40, 62, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    //   CAN.sendMsgBuf(0x5C0, 0, 8, msg62_on);
    // } else {
    //   uint8_t msg62_off[] = { 0x40, 62, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    //   CAN.sendMsgBuf(0x5C0, 0, 8, msg62_off);
    // }

    // ===============================
    // Engine warning lamp → CC-ID 34 / 30 / 22 / 50
    // ===============================
    if (game.engineLight) {
      uint8_t msg34_on[] = { 0x40, 34, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5C0, 0, 8, msg34_on);

      uint8_t msg30_on[] = { 0x40, 30, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5C0, 0, 8, msg30_on);

      uint8_t msg22_on[] = { 0x40, 22, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5C0, 0, 8, msg22_on);

      uint8_t msg50_on[] = { 0x40, 50, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5C0, 0, 8, msg50_on);

      uint8_t msg213_on[] = { 0x40, 213, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5C0, 0, 8, msg213_on);
    } else {
      uint8_t msg34_off[] = { 0x40, 34, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5C0, 0, 8, msg34_off);

      uint8_t msg30_off[] = { 0x40, 30, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5C0, 0, 8, msg30_off);

      uint8_t msg22_off[] = { 0x40, 22, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5C0, 0, 8, msg22_off);

      uint8_t msg50_off[] = { 0x40, 50, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5C0, 0, 8, msg50_off);

      uint8_t msg213_off[] = { 0x40, 213, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5C0, 0, 8, msg213_off);
    }
    // Force ACC ECU alive (required for 0x289 parsing)
    // sendAcc();
    // TPMS ECU keep‑alive
    {
      unsigned char tpmsWithoutCRC[] = { 0xF0 | counter4Bit, 0xA2, 0xA0, 0xA0 };
      unsigned char tpmsWithCRC[] = {
        crc8Calculator.get_crc8(tpmsWithoutCRC, 4, 0xC5),
        tpmsWithoutCRC[0],
        tpmsWithoutCRC[1],
        tpmsWithoutCRC[2],
        tpmsWithoutCRC[3]
      };
      CAN.sendMsgBuf(0x369, 0, 5, tpmsWithCRC);
    }
    // Startup burst: send 0x291 every 100 ms for the first 3 seconds after ignition ON.
    if (game.ignition && (millis() - settingsBurstStart) < 3000) {
      updateLanguageAndUnits();
    }

    counter4Bit++;
    if (counter4Bit > 14) {
      counter4Bit = 0;
    }

    count++;
    if (count >= 254) { count = 0; } // Needs to be reset at 254 not 255

    lastDashboardUpdateTime = millis();
  }

  // =======================
  // Manual alert injection from WebDashboard
  // =======================
  if (game.alertStart) {
    uint8_t msg[] = { 0x40, game.alertId, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg);
    game.alertStart = false;
  }

  if (game.alertClear) {
    uint8_t msg[] = { 0x40, game.alertId, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg);
    game.alertClear = false;
  }

  // =======================
  // F15 Cluster specific logic
  // =======================
  // Send ignition and temperature frame for F15 cluster
  // Use oil temperature (WTemp) instead of coolant temperature
  {
    unsigned char ingandtemp[8] = {0x0, count, count, 0x00, 0x00, count, count, count}; // ignition for F15 cluster
    // Use oil temperature (from game.oilTemperature) instead of coolant temperature
    ingandtemp[5] = int((0.983607 * game.oilTemperature) + 51.3169);
    CAN.sendMsgBuf(0x3f9, 0, 8, ingandtemp);
  }

  // Oil temperature overheat warning (based on game.oilTemperature)
  if(game.oilTemperature >= 130) {
      uint8_t engine_overheated[] = { 0x40, 39, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5c0, 0, 8, engine_overheated);
  } else {
      uint8_t engine_overheated1[] = { 0x40, 39, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5c0, 0, 8, engine_overheated1);
  }

  if (millis() - lastDashboardUpdateTime1000ms >= dashboardUpdateTime1000) {

    // TEST: force BC button every 1s to verify 0x1EE works
    // game.buttonEventToProcess = 1;

    sendLights(game.mainLights, game.highBeam, game.rearFogLight, game.frontFogLight);
    sendBlinkers(game.leftTurningIndicator, game.rightTurningIndicator);
    sendBacklightBrightness(game.backlightBrightness);
    // =========================================
    // Gear-based automatic drive mode override
    // D  -> Comfort (2)
    // S  -> Sport+ (5)
    // Otherwise use game.driveMode
    // =========================================
    uint8_t driveModeToSend = game.driveMode;

    if (game.gear == GearState_Auto_D) {
      driveModeToSend = 2;   // Comfort
    }
    else if (game.gear == GearState_Auto_S) {
      driveModeToSend = 5;   // Sport+
    }

    // Force DSC OFF if traction intervention lasted >2s
    if (dscForced) {
      driveModeToSend = 6;   // DSC OFF

      uint8_t ids[] = {36, 42, 215, 35, 204};

      for (uint8_t i = 0; i < sizeof(ids); i++) {
        uint8_t message[] = {
          0x40,
          ids[i],
          0x00,
          0x29,
          0xFF,
          0xFF,
          0xFF,
          0xFF
        };

        CAN.sendMsgBuf(0x5C0, 0, 8, message);
      }
    }

    sendDriveMode(driveModeToSend);

    // ===============================
    // Game Time → Cluster Clock (HH:MM)
    // Using game.time (milliseconds since simulation start)
    // ===============================
    sendOutsideTemperature(game.outdoorTemperature);

    // ===============================
    // 仪表盘时间显示 (0x39E)
    // ===============================
    // 将游戏内的毫秒运行时间转化为小时和分钟 (你也可以改成 ESP32 的真实时间)
    unsigned long totalSeconds = game.time / 1000;
    uint8_t hours = (totalSeconds / 3600) % 24;
    uint8_t minutes = (totalSeconds / 60) % 60;

    sendTime(hours, minutes);

    // Example BMW style time broadcast frame (adjust ID if needed for your cluster)
    unsigned char timeFrame[] = { 
      0xF0 | counter4Bit, 
      hours, 
      minutes, 
      0x00, 
      0x00 
    };

    unsigned char timeFrameWithCRC[] = {
      crc8Calculator.get_crc8(timeFrame, 5, 0xA5),
      timeFrame[0],
      timeFrame[1],
      timeFrame[2],
      timeFrame[3],
      timeFrame[4]
    };

    CAN.sendMsgBuf(0x3F1, 0, 6, timeFrameWithCRC);

    // (button event handling moved to immediate processing above)

    // Keep sending 0x291 at low rate after startup so the cluster can re-sync if needed.
    updateLanguageAndUnits();

    lastDashboardUpdateTime1000ms = millis();
  }
  // Serial.println(game.fuelQuantity);


}

void BMWFSeriesCluster::sendFixedLIM()
{
  /*
   * CarCluster-style fixed LIM sender
   *
   * Instead of using 0x289 in mr_goofy format, this sends the LIM / SLI
   * speed frame on 0x287, which is the format used by the newer logic.
   *
   * Byte layout:
   * [0] = 0x05
   * [1] = LIM value encoded in 5 km/h steps
   * [2] = 0x10
   * [3] = 0x10
   * [4] = 0x00
   * [5] = 0xFF
   * [6] = 0xFF
   * [7] = 0xFF
   */

  const uint8_t fixedLimSpeedKmh = 110;
  uint8_t limValue = fixedLimSpeedKmh / 5;

  if (limValue < 1 || limValue > 30) {
    return;
  }

  unsigned char limFrame[] = {
    0x05,
    limValue,
    0x10,
    0x10,
    0x00,
    0xFF,
    0xFF,
    0xFF
  };

  CAN.sendMsgBuf(0x287, 0, 8, limFrame);
}

void BMWFSeriesCluster::sendIgnitionStatus(bool ignition) {
  uint8_t ignitionStatus = ignition ? 0x8A : 0x8;
  unsigned char ignitionWithoutCRC[] = { 0x80|counter4Bit, ignitionStatus, 0xDD, 0xF1, 0x01, 0x30, 0x06 };
  unsigned char ignitionWithCRC[] = { crc8Calculator.get_crc8(ignitionWithoutCRC, 7, 0x44), ignitionWithoutCRC[0], ignitionWithoutCRC[1], ignitionWithoutCRC[2], ignitionWithoutCRC[3], ignitionWithoutCRC[4], ignitionWithoutCRC[5], ignitionWithoutCRC[6] };
  CAN.sendMsgBuf(0x12F, 0, 8, ignitionWithCRC);
}

void BMWFSeriesCluster::sendSpeed(int speed) {
  uint16_t calculatedSpeed = (double)speed * 64.01;
  unsigned char speedWithoutCRC[] = { 0xC0|counter4Bit, lo8(calculatedSpeed), hi8(calculatedSpeed), (speed == 0 ? 0x81 : 0x91) };
  unsigned char speedWithCRC[] = { crc8Calculator.get_crc8(speedWithoutCRC, 4, 0xA9), speedWithoutCRC[0], speedWithoutCRC[1], speedWithoutCRC[2], speedWithoutCRC[3] };
  uint32_t t1 = micros();
  byte status = CAN.sendMsgBuf(0x1A1, 0, 5, speedWithCRC);
  uint32_t t2 = micros();

  if (t2 - t1 > 2000) {
    Serial.print("[CAN BLOCK][SPEED] time(us): ");
    Serial.println(t2 - t1);
  }

  if (status != CAN_OK) {
    Serial.print("[CAN ERROR][SPEED] status: ");
    Serial.println(status);
  }
}

void BMWFSeriesCluster::sendRPM(int rpm, int manualGear) {

  // ===== High refresh F3x-style logic =====
  if (rpm < 0) rpm = 0;
  if (rpm > 7500) rpm = 7500;

  // Map gear similar to original logic
  int calculatedGear = 0;
  switch (manualGear) {
    case 0: calculatedGear = 0; break;
    case 1 ... 9: calculatedGear = manualGear + 4; break;
    case 11: calculatedGear = 2; break;
    case 12: calculatedGear = 1; break;
    default: calculatedGear = 0; break;
  }

  // BMW F3x scaling factor
  // Cluster expects scaled value ≈ rpm * 1.557
  float rpmScaledFloat = rpm * 1.557f;
  uint16_t rpmScaled = (uint16_t)rpmScaledFloat;

  // Slight offset for dual-frame smoothing
  uint16_t rpmScaledPlus  = (uint16_t)((rpm + 4) * 1.557f);
  uint16_t rpmScaledMinus = (uint16_t)((rpm) * 1.557f);

  // ===== Frame template =====
  uint8_t rpmFrame[8] = {
    0x00, // CRC placeholder
    0x00, // LSB RPM
    0x00, // MSB RPM
    0xC0,
    0xF0,
    (uint8_t)calculatedGear,
    0xFF,
    0xFF
  };

  // ---------- First frame (rpm + small offset) ----------
  rpmFrame[1] = lo8(rpmScaledPlus);
  rpmFrame[2] = hi8(rpmScaledPlus);

  uint8_t crc1 = crc8Calculator.get_crc8(&rpmFrame[1], 7, 0x7A);
  rpmFrame[0] = crc1;

  CAN.sendMsgBuf(0x0F3, 0, 8, rpmFrame);

  // ---------- Second frame (real rpm) ----------
  rpmFrame[1] = lo8(rpmScaledMinus);
  rpmFrame[2] = hi8(rpmScaledMinus);

  uint8_t crc2 = crc8Calculator.get_crc8(&rpmFrame[1], 7, 0x7A);
  rpmFrame[0] = crc2;

  CAN.sendMsgBuf(0x0F3, 0, 8, rpmFrame);
}

void BMWFSeriesCluster::sendAutomaticTransmission(GearState gear, uint8_t gearIndex) {

  // 0 = clear
  // 1-9 = gear index (used for D1-D8 / S1-S8 / M1-M8)
  // 10 = P
  // 11 = R
  // 12 = N
  // 13 = D (auto, index must be provided externally)

  uint8_t selectedGear = 0x00;
  uint8_t manualByte   = counter4Bit;

  // ----- D / S / M -----
  // NOTE:
  // 0x80 = D
  // 0x81 = S
  // 0x82 = M

  if (gear == GearState_Auto_D) {
    selectedGear = 0x80;   // D
    if (gearIndex > 0)
      manualByte = (gearIndex << 4) | counter4Bit;
  }
  else if (gear == GearState_Auto_S) {
    selectedGear = 0x81;   // S
    if (gearIndex > 0)
      manualByte = (gearIndex << 4) | counter4Bit;
  }
  else if (gear >= GearState_Manual_1 && gear <= GearState_Manual_8) {
    selectedGear = 0x82;   // M
    if (gearIndex > 0)
      manualByte = (gearIndex << 4) | counter4Bit;
  }
  else {
    // P / R / N switch section (do not modify)
    switch (gear) {
      case GearState_Auto_P: selectedGear = 0x20; break; // P
      case GearState_Auto_R: selectedGear = 0x40; break; // R
      case GearState_Auto_N: selectedGear = 0x60; break; // N
      default: selectedGear = 0x00; break;
    }
    manualByte = counter4Bit;
  }

  unsigned char transmissionWithoutCRC[] = {
    manualByte,
    selectedGear,
    0xFC,
    0xFF
  };

  unsigned char transmissionWithCRC[] = {
    crc8Calculator.get_crc8(transmissionWithoutCRC, 4, 0xD6),
    transmissionWithoutCRC[0],
    transmissionWithoutCRC[1],
    transmissionWithoutCRC[2],
    transmissionWithoutCRC[3]
  };

  // ===== DEBUG: Print current transmission state =====
//   Serial.print("[GEAR DEBUG] gear enum=");
//   Serial.print((int)gear);
//   Serial.print(" gearIndex=");
//   Serial.print((int)gearIndex);
//   Serial.print(" selectedGear=0x");
//   Serial.println(selectedGear, HEX);
  CAN.sendMsgBuf(0x3FD, 0, 5, transmissionWithCRC);
}

void BMWFSeriesCluster::sendBasicDriveInfo(GameState& game, int oilTemperature) {

    // ABS alive frame (cluster requires module online)
    uint8_t absStatusByte = 0x14;
    unsigned char abs1WithoutCRC[] = { (uint8_t)(0xF0 | counter4Bit), 0xFE, 0xFF, absStatusByte };
    unsigned char abs1WithCRC[] = { crc8Calculator.get_crc8(abs1WithoutCRC, 4, 0xD8), abs1WithoutCRC[0], abs1WithoutCRC[1], abs1WithoutCRC[2], abs1WithoutCRC[3] };
    CAN.sendMsgBuf(0x36E, 0, 5, abs1WithCRC);

    // ABS secondary frame
    unsigned char absSecondary[8] = { counter4Bit, counter4Bit, counter4Bit, counter4Bit, counter4Bit, counter4Bit, counter4Bit, counter4Bit };
    CAN.sendMsgBuf(0xB6E, 0, 8, absSecondary);

    // Alive counter safety
    unsigned char aliveCounterSafetyWithoutCRC[] = { count, 0xFF };
    CAN.sendMsgBuf(0xD7, 0, 2, aliveCounterSafetyWithoutCRC);

    // Power Steering keep-alive (0x2A7)
    unsigned char steeringColumnWithoutCRC[] = { (uint8_t)(0xF0 | counter4Bit), 0xFE, 0xFF, 0x14 };
    unsigned char steeringColumnWithCRC[] = { crc8Calculator.get_crc8(steeringColumnWithoutCRC, 4, 0x9E), steeringColumnWithoutCRC[0], steeringColumnWithoutCRC[1], steeringColumnWithoutCRC[2], steeringColumnWithoutCRC[3] };
    CAN.sendMsgBuf(0x2A7, 0, 5, steeringColumnWithCRC);

    // Restraint system (0x19B)
    unsigned char restraintWithoutCRC[] = { (uint8_t)(0x40|counter4Bit), 0x40, 0x55, 0xFD, 0xFF, 0xFF, 0xFF };
    unsigned char restraintWithCRC[] = { crc8Calculator.get_crc8(restraintWithoutCRC, 7, 0xFF), restraintWithoutCRC[0], restraintWithoutCRC[1], restraintWithoutCRC[2], restraintWithoutCRC[3], restraintWithoutCRC[4], restraintWithoutCRC[5], restraintWithoutCRC[6] };
    CAN.sendMsgBuf(0x19B, 0, 8, restraintWithCRC);

    // EHC Signal (0x26A)
    unsigned char EHCWithoutCRC[] = { (uint8_t)(0x40|counter4Bit), 0x40, 0x55, 0xFD, 0xFF, 0xFF, 0xFF };
    unsigned char EHCWithCRC[] = { crc8Calculator.get_crc8(EHCWithoutCRC, 7, 0xFF), EHCWithoutCRC[0], EHCWithoutCRC[1], EHCWithoutCRC[2], EHCWithoutCRC[3], EHCWithoutCRC[4], EHCWithoutCRC[5], EHCWithoutCRC[6] };
    CAN.sendMsgBuf(0x26A, 0, 8, EHCWithCRC);

    // Restraint system 2 (0x297)
    unsigned char restraint2WithoutCRC[] = { (uint8_t)(0xE0|counter4Bit), 0xF1, 0xF0, 0xF2, 0xF2, 0xFE };
    unsigned char restraint2WithCRC[] = { crc8Calculator.get_crc8(restraint2WithoutCRC, 6, 0x28), restraint2WithoutCRC[0], restraint2WithoutCRC[1], restraint2WithoutCRC[2], restraint2WithoutCRC[3], restraint2WithoutCRC[4], restraint2WithoutCRC[5] };
    CAN.sendMsgBuf(0x297, 0, 7, restraint2WithCRC);


    static unsigned long rpmStableStartTime = 0;
    static bool engineStable = false;

    if (game.ignition && game.rpm >= 400) {
        if (rpmStableStartTime == 0) {
            rpmStableStartTime = millis();
        }
        if (millis() - rpmStableStartTime >= 500) {
            engineStable = true;
        }
    } else {
        rpmStableStartTime = 0;
        engineStable = false;
    }

    if (!engineStable) {
        // Clear traction lamp
        uint8_t clear42[] = { 0x40, 42, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
        CAN.sendMsgBuf(0x5C0, 0, 8, clear42);

        // Clear DSC related IDs
        uint8_t clearIds[] = {184,215,237,236};
        for (uint8_t i = 0; i < sizeof(clearIds); i++) {
            uint8_t msg[] = {
                0x40,
                clearIds[i],
                0x00,
                0x28,
                0xFF,
                0xFF,
                0xFF,
                0xFF
            };
            CAN.sendMsgBuf(0x5C0, 0, 8, msg);
        }
        return; 
    }


    // ===============================
    // TPMS – F-series CC-ID (correct decimal IDs + state change only)
    // 139 = FL
    // 143 = FR
    // 141 = RL
    // 140 = RR
    // 142 = Global
    // ===============================

    static bool lastFL = false;
    static bool lastFR = false;
    static bool lastRL = false;
    static bool lastRR = false;
    static bool lastGlobal = false;

    // Front Left (139)
    if (game.tireDefFL != lastFL) {
      uint8_t msg[] = { 0x40, 139, 0x00, game.tireDefFL ? 0x29 : 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5C0, 0, 8, msg);
      lastFL = game.tireDefFL;
    }

    // Front Right (143)
    if (game.tireDefFR != lastFR) {
      uint8_t msg[] = { 0x40, 143, 0x00, game.tireDefFR ? 0x29 : 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5C0, 0, 8, msg);
      lastFR = game.tireDefFR;
    }

    // Rear Left (141)
    if (game.tireDefRL != lastRL) {
      uint8_t msg[] = { 0x40, 141, 0x00, game.tireDefRL ? 0x29 : 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5C0, 0, 8, msg);
      lastRL = game.tireDefRL;
    }

    // Rear Right (140)
    if (game.tireDefRR != lastRR) {
      uint8_t msg[] = { 0x40, 140, 0x00, game.tireDefRR ? 0x29 : 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5C0, 0, 8, msg);
      lastRR = game.tireDefRR;
    }

    bool anyTireDeflated =
      game.tireDefFL ||
      game.tireDefFR ||
      game.tireDefRL ||
      game.tireDefRR;

    if (anyTireDeflated != lastGlobal) {
      uint8_t globalMsg[] = { 0x40, 142, 0x00, anyTireDeflated ? 0x29 : 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5C0, 0, 8, globalMsg);
      lastGlobal = anyTireDeflated;
    }

  // Unknown (makes RPM steady)
  // Also engine temp on diesel? Range 100 - 200
  unsigned char oilWithoutCRC[] = { 0x10|counter4Bit, 0x82, 0x4E, 0x7E, oilTemperature + 50, 0x05, 0x89 };
  unsigned char oilWithCRC[] = { crc8Calculator.get_crc8(oilWithoutCRC, 7, 0xF1), oilWithoutCRC[0], oilWithoutCRC[1], oilWithoutCRC[2], oilWithoutCRC[3], oilWithoutCRC[4], oilWithoutCRC[5], oilWithoutCRC[6] };
  CAN.sendMsgBuf(0x3F9, 0, 8, oilWithCRC);

  // ===============================
  // Oil / Coolant Overheat → CC-ID 39
  // ===============================
  if (oilTemperature > 130 || game.coolantTemperature > 115) {
    uint8_t msg39_on[] = { 0x40, 39, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg39_on);
  } else {
    uint8_t msg39_off[] = { 0x40, 39, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg39_off);
  }

  // ===============================
  // Gearbox Overheat logic (103 / 104 / 105)
  // Based on oil temp + rpm + speed
  // ===============================
  if (oilTemperature > 120 && game.rpm > 3500) {
    uint8_t msg103[] = { 0x40, 103, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg103);
  }

  if (oilTemperature > 130 && game.speed > 80) {
    uint8_t msg104[] = { 0x40, 104, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg104);
  }

  if (oilTemperature > 140) {
    uint8_t msg105[] = { 0x40, 105, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg105);
  }
}

void BMWFSeriesCluster::sendParkBrake(bool handbrakeActive) {
  unsigned char abs3WithoutCRC[] = { 0xF0|counter4Bit, 0x38, 0, handbrakeActive ? 0x15 : 0x14 };
  unsigned char abs3WithCRC[] = { crc8Calculator.get_crc8(abs3WithoutCRC, 4, 0x17), abs3WithoutCRC[0], abs3WithoutCRC[1], abs3WithoutCRC[2], abs3WithoutCRC[3] };
  CAN.sendMsgBuf(0x36F, 0, 5, abs3WithCRC);
}

void BMWFSeriesCluster::sendFuel(float fuelRatio, uint8_t inFuelRange[], uint8_t outFuelRange[], bool isCarMini) {

  // BeamNG fuelRatio = 0.0 – 1.0
  if (fuelRatio < 0.0f) fuelRatio = 0.0f;
  if (fuelRatio > 1.0f) fuelRatio = 1.0f;

  int fuelPercent = (int)(fuelRatio * 100.0f);

  uint8_t fuelQuantityLiters = multiMap<uint8_t>(
      fuelPercent,
      inFuelRange,
      outFuelRange,
      3
  );

  unsigned char fuelWithoutCRC[] = {
    (isCarMini ? 0 : hi8(fuelQuantityLiters)),
    (isCarMini ? 0 : lo8(fuelQuantityLiters)),
    hi8(fuelQuantityLiters),
    lo8(fuelQuantityLiters),
    0x00
  };

  CAN.sendMsgBuf(0x349, 0, 5, fuelWithoutCRC);
}

void BMWFSeriesCluster::sendDistanceTravelled(int speed)
{
  // ============================================================
  // Realistic instantaneous fuel model (speed + throttle + load)
  // Uses:
  //   - game speed (passed as parameter)
  //   - engine load approximation via rpm influence
  // This creates dynamic MPG behavior instead of fixed ratio
  // ============================================================

  // --- Static accumulator keeps continuity ---
  static float virtualDistanceAccumulator = 0.0f;

  // ---- 0x2C4 (MPG base frame - keeps bar alive) ----
  unsigned char mpgWithoutCRC[] = { count, 0xFF, 0x64, 0x64, 0x64, 0x01, 0xF1 };
  unsigned char mpgWithCRC[] = {
    crc8Calculator.get_crc8(mpgWithoutCRC, 7, 0xC6),
    mpgWithoutCRC[0],
    mpgWithoutCRC[1],
    mpgWithoutCRC[2],
    mpgWithoutCRC[3],
    mpgWithoutCRC[4],
    mpgWithoutCRC[5],
    mpgWithoutCRC[6]
  };
  CAN.sendMsgBuf(0x2C4, 0, 8, mpgWithCRC);

  // ============================================================
  // Fuel consumption approximation logic
  //
  // Idea:
  // High RPM + high speed → more fuel burn → smaller Δdistance
  // Low RPM cruise        → efficient     → larger Δdistance
  //
  // Cluster calculates MPG from:
  //   speed / delta(distance)
  // So we manipulate delta(distance) dynamically
  // ============================================================

  // Estimate load factor from RPM (normalized 0–1)
  float rpmFactor = (float)mapRPMValueForFuelModel(speed);
  if (rpmFactor < 0.1f) rpmFactor = 0.1f;

  // Base consumption coefficient
  float baseBurn = 0.8f;

  // Dynamic burn increases with rpmFactor
  float fuelBurnRate = baseBurn + (rpmFactor * 2.5f);

  // Prevent division anomalies at very low speed
  if (speed < 3) {
    fuelBurnRate *= 3.0f;  // idling is inefficient
  }

  // Convert burn rate into "distance delta"
  float distanceDelta = (float)speed / fuelBurnRate;

  virtualDistanceAccumulator += distanceDelta;

  if (virtualDistanceAccumulator > 65535.0f)
    virtualDistanceAccumulator = 0.0f;

  distanceTravelledCounter = (uint16_t)virtualDistanceAccumulator;

  // ---- 0x2BB (distance frame that moves MPG bar) ----
  unsigned char mpg2WithoutCRC[] = {
    0xF0 | counter4Bit,
    lo8(distanceTravelledCounter),
    hi8(distanceTravelledCounter),
    0xF2
  };

  unsigned char mpg2WithCRC[] = {
    crc8Calculator.get_crc8(mpg2WithoutCRC, 4, 0xDE),
    mpg2WithoutCRC[0],
    mpg2WithoutCRC[1],
    mpg2WithoutCRC[2],
    mpg2WithoutCRC[3]
  };

  CAN.sendMsgBuf(0x2BB, 0, 5, mpg2WithCRC);
}


void BMWFSeriesCluster::sendBlinkers(bool leftTurningIndicator, bool rightTurningIndicator) {
  //Blinkers
  uint8_t blinkerStatus = (leftTurningIndicator == 0 && rightTurningIndicator == 0) ? 0x80 : (0x81 | leftTurningIndicator << 4 | rightTurningIndicator << 5);
  unsigned char blinkersWithoutCRC[] = { blinkerStatus, 0xF0 };
  CAN.sendMsgBuf(0x1F6, 0, 2, blinkersWithoutCRC);
}

void BMWFSeriesCluster::sendLights(bool mainLights, bool highBeam, bool rearFogLight, bool frontFogLight) {
  //Lights
  // If high beam is active, force main lights ON (BMW logic: high beam requires low beam)
  if (highBeam) {
    mainLights = true;
  }
  //32 = front fog light, 64 = rear fog light, 2 = high beam, 4 = main lights
  uint8_t lightStatus = highBeam << 1 | mainLights << 2 | frontFogLight << 5 | rearFogLight << 6;
  unsigned char lightsWithoutCRC[] = { lightStatus, 0xC0, 0xF7 };
  CAN.sendMsgBuf(0x21A, 0, 3, lightsWithoutCRC);
}

void BMWFSeriesCluster::sendBacklightBrightness(uint8_t brightness) {
  // Backlight brightness
  uint8_t mappedBrightness = map(brightness, 0, 100, 0, 253);
  unsigned char backlightBrightnessWithoutCRC[] = { mappedBrightness, 0xFF };
  CAN.sendMsgBuf(0x202, 0, 2, backlightBrightnessWithoutCRC);
}

void BMWFSeriesCluster::sendAlerts(GameState& game, bool offroad, bool handbrake, bool isCarMini) {
  // static bool lastDoorState = false;
  static bool lastDoorFL = false;
  static bool lastDoorFR = false;
  static bool lastDoorRL = false;
  static bool lastDoorRR = false;
  // ===============================
  // Individual door CC-ID mapping (real per-door state)
  // 14 = Front Right
  // 15 = Front Left
  // 16 = Rear Left
  // 17 = Rear Right
  // ===============================

  bool fl = game.doorFL;
  bool fr = game.doorFR;
  bool rl = game.doorRL;
  bool rr = game.doorRR;

  if (fr != lastDoorFR) {
    uint8_t msg[] = { 0x40, 14, 0x00, fr ? 0x29 : 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg);
    lastDoorFR = fr;
  }

  if (fl != lastDoorFL) {
    uint8_t msg[] = { 0x40, 15, 0x00, fl ? 0x29 : 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg);
    lastDoorFL = fl;
  }

  if (rl != lastDoorRL) {
    uint8_t msg[] = { 0x40, 16, 0x00, rl ? 0x29 : 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg);
    lastDoorRL = rl;
  }

  if (rr != lastDoorRR) {
    uint8_t msg[] = { 0x40, 17, 0x00, rr ? 0x29 : 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg);
    lastDoorRR = rr;
  }
  if (offroad) {
    uint8_t message[] = { 0x40, 215, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5c0, 0, 8, message);
  } else {
    uint8_t message[] = { 0x40, 215, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5c0, 0, 8, message);
  }

  if (isCarMini) {
    if (handbrake) {
      uint8_t message[] = { 0x40, 71, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5c0, 0, 8, message);
    } else {
      uint8_t message[] = { 0x40, 71, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5c0, 0, 8, message);
    }
  }
}

void BMWFSeriesCluster::sendSteeringWheelButton(int buttonEvent) {
  // BMW F-series BC/menu button on 0x1EE needs a press + release edge.
  // buttonEvent == 1 -> BC / menu cycle
  uint8_t pressedValue = 0x00;

  switch (buttonEvent) {

    case 1:
      pressedValue = 0x4C;   // BC / menu cycle
      break;

    case 2:
      pressedValue = 0x44;   // SET / LIM
      break;

    case 3:
      pressedValue = 0x48;   // RES / Resume
      break;

    case 4:
      pressedValue = 0x40;   // CANCEL
      break;

    case 5:
      pressedValue = 0x50;   // LIM toggle
      break;

    default:
      return;
  }

  uint8_t pressFrame[2] = { pressedValue, 0xFF };
  CAN.sendMsgBuf(0x1EE, 0, 2, pressFrame);

  delay(40);

  uint8_t releaseFrame[2] = { 0x00, 0xFF };
  CAN.sendMsgBuf(0x1EE, 0, 2, releaseFrame);
}

void BMWFSeriesCluster::updateLanguageAndUnits() {

  uint8_t language = 0x01;   // English
  uint8_t byte2 = 18;        // Celsius
  uint8_t byte3 = 89;        // l/100km + km

  uint8_t frame[8] = { 
    language,
    byte2,
    byte3,
    0x00,
    0x00,
    0x00,
    0x00,
    0x00
  };

  CAN.sendMsgBuf(0x291, 0, 8, frame);
}

void BMWFSeriesCluster::sendDriveMode(uint8_t driveMode) {
  //1= Traction, 2= Comfort, 4= Sport, 5= Sport+, 6= DSC off, 7= Eco pro 
  unsigned char modeWithoutCRC[] = { 0xF0|counter4Bit, 0, 0, driveMode, 0x11, 0xC0 };
  unsigned char modeWithCRC[] = { crc8Calculator.get_crc8(modeWithoutCRC, 6, 0x4a), modeWithoutCRC[0], modeWithoutCRC[1], modeWithoutCRC[2], modeWithoutCRC[3], modeWithoutCRC[4], modeWithoutCRC[5] };
  CAN.sendMsgBuf(0x3A7, 0, 7, modeWithCRC);
}

void BMWFSeriesCluster::sendAcc() {
  unsigned char accWithoutCrc[] = {
    0xF0 | accCounter,
    0x5C,
    0x70,
    0x01,   // ACC active flag
    0x00
  };

  unsigned char accWithCrc[] = {
    crc8Calculator.get_crc8(accWithoutCrc, 5, 0x6b),
    accWithoutCrc[0],
    accWithoutCrc[1],
    accWithoutCrc[2],
    accWithoutCrc[3],
    accWithoutCrc[4]
  };

  CAN.sendMsgBuf(0x33B, 0, 6, accWithCrc);

  accCounter += 4;
  if (accCounter > 0x0E) {
    accCounter = accCounter - 0x0F;
  }
}
float BMWFSeriesCluster::mapRPMValueForFuelModel(int speed)
{
  // Approximate RPM influence based on vehicle speed
  // This avoids needing direct throttle access
  // Creates realistic curve:
  //   low speed → high consumption
  //   cruise    → efficient
  //   high speed→ moderate consumption

  float normalized = speed / 200.0f;   // assume 200km/h max range

  if (normalized > 1.0f) normalized = 1.0f;
  if (normalized < 0.0f) normalized = 0.0f;

  return normalized;
}
void BMWFSeriesCluster::sendOutsideTemperature(int temperature) {
  
  uint8_t tempByte = (temperature * 2) + 80;
  
  unsigned char tempFrame[2] = { tempByte, 0xFF }; 
  CAN.sendMsgBuf(0x2CA, 0, 2, tempFrame);
}

void BMWFSeriesCluster::sendTime(uint8_t hours, uint8_t minutes) {
  
  unsigned char timeFrame[8] = {
    hours,    
    minutes,  
    0x00,     
    0x01,     
    0x01,     
    0xDF,     
    0x07,     
    0xF2      
  };
  
  CAN.sendMsgBuf(0x39E, 0, 8, timeFrame);
}
