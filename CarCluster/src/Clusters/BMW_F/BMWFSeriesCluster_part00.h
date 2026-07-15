// ####################################################################################################################
// 
// Code part of CarCluster project by Andrej Rolih. See .ino file more details

// Enhanced and optimized for the BMW F10-only build.
// Project: https://github.com/JackieZ123430/CarCluster-F10-Enhanced
// Better_CAN: https://github.com/JackieZ123430/Better_CAN
//
// ####################################################################################################################

#include "BMWFSeriesCluster.h"

BMWFSeriesCluster::BMWFSeriesCluster(MCP_CAN& CAN) : CAN(CAN) {
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
  if (scaledSpeed < 0) return 0;
  if (scaledSpeed > game.configuration.maximumSpeedValue) {
    return game.configuration.maximumSpeedValue;
  }
  return scaledSpeed;
}

int BMWFSeriesCluster::mapRPM(GameState& game) {
  int scaledRPM = game.rpm * game.configuration.rpmCorrectionFactor;
  if (scaledRPM < 0) return 0;
  if (scaledRPM > game.configuration.maximumRPMValue) {
    return game.configuration.maximumRPMValue;
  }
  return scaledRPM;
}


int BMWFSeriesCluster::mapCoolantTemperature(GameState& game) {
  if (game.coolantTemperature < game.configuration.minimumCoolantTemperature) { return game.configuration.minimumCoolantTemperature; }
  if (game.coolantTemperature > game.configuration.maximumCoolantTemperature) { return game.configuration.maximumCoolantTemperature; }
  return game.coolantTemperature;
}

void BMWFSeriesCluster::updateWithGame(GameState& game) {
  // The previously removed LIM/ACC implementation is intentionally not called here.
  // Keeping a declaration or call without an implementation caused a link failure.

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

  // ESC/TCS intervention and DSC-off mode are separate states.
  // Better_CAN supplies both explicitly, so a long intervention must not force DSC OFF.

  static unsigned long engineStartTime = 0;
  static bool engineWasRunning = false;
  static bool can46Sent = false;
  if (millis() - lastDashboardUpdateTime >= dashboardUpdateTimeFast) {



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
    const bool engineRunningNow = game.engineRunning || game.rpm >= 400;
    if (game.ignition && engineRunningNow) {
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

