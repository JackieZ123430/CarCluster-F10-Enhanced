// ####################################################################################################################
// 
// Code part of CarCluster project by Andrej Rolih. See .ino file more details
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
    case GearState_Manual_1: return 1; break;
    case GearState_Manual_2: return 2; break;
    case GearState_Manual_3: return 3; break;
    case GearState_Manual_4: return 4; break;
    case GearState_Manual_5: return 5; break;
    case GearState_Manual_6: return 6; break;
    case GearState_Manual_7: return 7; break;
    case GearState_Manual_8: return 8; break;
    case GearState_Manual_9: return 9; break;
    case GearState_Manual_10: return 13; break;
    case GearState_Auto_P: return 10; break;
    case GearState_Auto_R: return 11; break;
    case GearState_Auto_N: return 12; break;
    case GearState_Auto_D: return 13; break;
    case GearState_Auto_S: return 13; break;
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
  int scaledRPM = game.rpm * game.configuration.speedCorrectionFactor;
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
  static unsigned long engineStartTime = 0;
  static bool engineWasRunning = false;
  static bool can46Sent = false;
  if (millis() - lastDashboardUpdateTime >= dashboardUpdateTime100) {
    // This should probably be done using a more sophisticated method like a
    // scheduler, but for now this seems to work.

    sendIgnitionStatus(game.ignition);
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

        delay(50);

        uint8_t msgClear[] = { 0x40, 91, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
        CAN.sendMsgBuf(0x5C0, 0, 8, msgClear);

        // CAN 53 (engine start related)
        uint8_t msg53_set[] = { 0x40, 53, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
        CAN.sendMsgBuf(0x5C0, 0, 8, msg53_set);

        delay(50);

        uint8_t msg53_clear[] = { 0x40, 53, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
        CAN.sendMsgBuf(0x5C0, 0, 8, msg53_clear);

        // CAN 181 (constant after engine start)
        uint8_t msg181_const[] = { 0x40, 181, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
        CAN.sendMsgBuf(0x5C0, 0, 8, msg181_const);

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

    // ===============================
    // Ignition ON but engine not started logic
    // If ignition ON and rpm == 0 → open multiple CC-IDs
    // If rpm >= 200 but engine not successfully running → add extra CC-IDs
    // ===============================
    if (game.ignition && game.rpm == 0) {

      uint8_t ids_on[] = {213, 220, 21, 24, 30, 175, 206, 255};

      for (uint8_t i = 0; i < sizeof(ids_on); i++) {
        uint8_t msg[] = { 0x40, ids_on[i], 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
        CAN.sendMsgBuf(0x5C0, 0, 8, msg);
      }

    } else {

      uint8_t ids_off[] = {213, 220, 21, 24, 30, 175, 206, 255};

      for (uint8_t i = 0; i < sizeof(ids_off); i++) {
        uint8_t msg[] = { 0x40, ids_off[i], 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
        CAN.sendMsgBuf(0x5C0, 0, 8, msg);
      }
    }

    // Additional failure detection:
    // RPM >= 200 but still considered not started (no stable idle)
    // Since GameState has no engineRunning flag,
    // treat rpm between 200 and 600 as failed start region
    if (game.ignition && game.rpm >= 200 && game.rpm < 600) {

      uint8_t extra_ids_on[] = {186, 22};

      for (uint8_t i = 0; i < sizeof(extra_ids_on); i++) {
        uint8_t msg[] = { 0x40, extra_ids_on[i], 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
        CAN.sendMsgBuf(0x5C0, 0, 8, msg);
      }

    } else {

      uint8_t extra_ids_off[] = {186, 22};

      for (uint8_t i = 0; i < sizeof(extra_ids_off); i++) {
        uint8_t msg[] = { 0x40, extra_ids_off[i], 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
        CAN.sendMsgBuf(0x5C0, 0, 8, msg);
      }
    }
    sendSpeed(mapSpeed(game));
    sendRPM(mapRPM(game), mapGenericGearToLocalGear(game.gear));
    sendBasicDriveInfo(game, game.oilTemperature);
    sendAutomaticTransmission(mapGenericGearToLocalGear(game.gear));
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
    int fuelPercent = (int)(game.fuelQuantity * 100.0f);
    if (fuelPercent < 0) fuelPercent = 0;
    if (fuelPercent > 100) fuelPercent = 100;

    sendFuel(fuelPercent, inFuelRange, outFuelRange, isCarMini);
    sendParkBrake(game.handbrake);
    sendDistanceTravelled(mapSpeed(game));
    // ===============================
    // Overspeed >220 km/h → force doorOpen trigger
    // ===============================
    if (game.speed > 220) {
      game.doorOpen = true;
    }
    sendAlerts(game.offroadLight, game.doorOpen, game.handbrake, isCarMini);

    // ===============================
    // DSC / Stability Control Logic (corrected)
    // ===============================

    if (game.escActive) {
      // DSC actively intervening → flash yellow DSC (35) + traction (215)
      uint8_t msg35_on[] = { 0x40, 35, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5C0, 0, 8, msg35_on);

      uint8_t msg215_on[] = { 0x40, 215, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5C0, 0, 8, msg215_on);

      // Ensure DSC OFF and fault cleared
      uint8_t msg36_off[] = { 0x40, 36, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5C0, 0, 8, msg36_off);

      uint8_t msg42_off[] = { 0x40, 42, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5C0, 0, 8, msg42_off);
    }
    else if (!game.hasESC) {
      // DSC completely disabled → DSC OFF + fault style
      uint8_t msg36_on[] = { 0x40, 36, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5C0, 0, 8, msg36_on);

      uint8_t msg35_on[] = { 0x40, 35, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5C0, 0, 8, msg35_on);

      uint8_t msg42_on[] = { 0x40, 42, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5C0, 0, 8, msg42_on);

      // Clear traction
      uint8_t msg215_off[] = { 0x40, 215, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5C0, 0, 8, msg215_off);
    }
    else {
      // Normal driving → clear all DSC related warnings
      uint8_t msg35_off[] = { 0x40, 35, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5C0, 0, 8, msg35_off);

      uint8_t msg215_off[] = { 0x40, 215, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5C0, 0, 8, msg215_off);

      uint8_t msg36_off[] = { 0x40, 36, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5C0, 0, 8, msg36_off);

      uint8_t msg42_off[] = { 0x40, 42, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5C0, 0, 8, msg42_off);
    }

    // ===============================
    // Parking brake extra alerts
    // 1) If handbrake pulled while speed >1 → show 48 & 55
    // 2) If handbrake pulled (independent) → show 55
    // ===============================
    if (game.handbrake && game.speed > 1) {
      uint8_t msg48_on[] = { 0x40, 48, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5C0, 0, 8, msg48_on);
    } else {
      uint8_t msg48_off[] = { 0x40, 48, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5C0, 0, 8, msg48_off);
    }

    // Independent CAN 55 (only handbrake state)
    if (game.handbrake) {
      uint8_t msg55_on[] = { 0x40, 55, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5C0, 0, 8, msg55_on);
    } else {
      uint8_t msg55_off[] = { 0x40, 55, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5C0, 0, 8, msg55_off);
    }

    // ===============================
    // Overspeed >120km/h → CAN62
    // ===============================
    if (game.speed > 120) {
      uint8_t msg62_on[] = { 0x40, 62, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5C0, 0, 8, msg62_on);
    } else {
      uint8_t msg62_off[] = { 0x40, 62, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
      CAN.sendMsgBuf(0x5C0, 0, 8, msg62_off);
    }

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
    sendAcc();
    // updateLanguageAndUnits();

    counter4Bit++;
    if (counter4Bit >= 14) { counter4Bit = 0; }

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

  if (millis() - lastDashboardUpdateTime1000ms >= dashboardUpdateTime1000) {
    sendLights(game.mainLights, game.highBeam, game.rearFogLight, game.frontFogLight);
    sendBlinkers(game.leftTurningIndicator, game.rightTurningIndicator);
    sendBacklightBrightness(game.backlightBrightness);
    sendDriveMode(game.driveMode);

    // ===============================
    // Game Time → Cluster Clock (HH:MM)
    // Using game.time (milliseconds since simulation start)
    // ===============================
    unsigned long totalSeconds = game.time / 1000;
    uint8_t hours = (totalSeconds / 3600) % 24;
    uint8_t minutes = (totalSeconds / 60) % 60;

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

    sendSteeringWheelButton(game.buttonEventToProcess);
    if (game.buttonEventToProcess != 0) {
      game.buttonEventToProcess = 0;
    }

    lastDashboardUpdateTime1000ms = millis();
  }
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
  CAN.sendMsgBuf(0x1A1, 0, 5, speedWithCRC);
}

void BMWFSeriesCluster::sendRPM(int rpm, int manualGear) {
  // The gear that the car is in: 0 = clear, 1-9 = M1-M9, 10 = P, 11 = R, 12 = N, 13 = D
  int calculatedGear = 0;
  switch (manualGear) {
    case 0: calculatedGear = 0; break; // Empty
    case 1 ... 9: calculatedGear = manualGear + 4; break; // 1-9
    case 11: calculatedGear = 2; break; // Reverse
    case 12: calculatedGear = 1; break; // Neutral
  }
  int rpmValue =  map(rpm, 0, 6900, 0x00, 0x2B);
  unsigned char rpmWithoutCRC[] = { 0x60|counter4Bit, rpmValue, 0xC0, 0xF0, calculatedGear, 0xFF, 0xFF };
  unsigned char rpmWithCRC[] = { crc8Calculator.get_crc8(rpmWithoutCRC, 7, 0x7A), rpmWithoutCRC[0], rpmWithoutCRC[1], rpmWithoutCRC[2], rpmWithoutCRC[3], rpmWithoutCRC[4], rpmWithoutCRC[5], rpmWithoutCRC[6] };
  CAN.sendMsgBuf(0x0F3, 0, 8, rpmWithCRC);
}

void BMWFSeriesCluster::sendAutomaticTransmission(int gear) {
  // The gear that the car is in: 0 = clear, 1-9 = M1-M9, 10 = P, 11 = R, 12 = N, 13 = D
  uint8_t selectedGear = 0;
  switch (gear) {
    case 1 ... 9: selectedGear = 0x81; break; // DS
    case 10: selectedGear = 0x20; break; // P
    case 11: selectedGear = 0x40; break; // R
    case 12: selectedGear = 0x60; break; // N
    case 13: selectedGear = 0x80; break; // D
  }
  unsigned char transmissionWithoutCRC[] = { counter4Bit, selectedGear, 0xFC, 0xFF }; //0x20= P, 0x40= R, 0x60= N, 0x80= D, 0x81= DS
  unsigned char transmissionWithCRC[] = { crc8Calculator.get_crc8(transmissionWithoutCRC, 4, 0xD6), transmissionWithoutCRC[0], transmissionWithoutCRC[1], transmissionWithoutCRC[2], transmissionWithoutCRC[3] };
  CAN.sendMsgBuf(0x3FD, 0, 5, transmissionWithCRC);
}

void BMWFSeriesCluster::sendBasicDriveInfo(GameState& game, int oilTemperature) {
    // ABS
    unsigned char abs1WithoutCRC[] = { 0xF0|counter4Bit, 0xFE, 0xFF, 0x14 };
    unsigned char abs1WithCRC[] = { crc8Calculator.get_crc8(abs1WithoutCRC, 4, 0xD8), abs1WithoutCRC[0], abs1WithoutCRC[1], abs1WithoutCRC[2], abs1WithoutCRC[3] };
    CAN.sendMsgBuf(0x36E, 0, 5, abs1WithCRC);

    //Alive counter safety
    unsigned char aliveCounterSafetyWithoutCRC[] = { count, 0xFF };
    CAN.sendMsgBuf(0xD7, 0, 2, aliveCounterSafetyWithoutCRC);

    //Power Steering
    unsigned char steeringColumnWithoutCRC[] = { 0xF0|counter4Bit, 0xFE, 0xFF, 0x14 };
    unsigned char steeringColumnWithCRC[] = { crc8Calculator.get_crc8(steeringColumnWithoutCRC, 4, 0x9E), steeringColumnWithoutCRC[0], steeringColumnWithoutCRC[1], steeringColumnWithoutCRC[2], steeringColumnWithoutCRC[3] };
    CAN.sendMsgBuf(0x2A7, 0, 5, steeringColumnWithCRC);

    //Cruise control (dynamic activation)
    uint8_t cruiseStateByte = game.cruiseControlActive ? 0xE3 : 0xE0;

    unsigned char cruiseWithoutCRC[] = { 
      0xF0|counter4Bit, 
      0xE0, 
      0xE0, 
      cruiseStateByte, 
      0x00, 
      0xEC, 
      0x01 
    };

    unsigned char cruiseWithCRC[] = { 
      crc8Calculator.get_crc8(cruiseWithoutCRC, 7, 0x82), 
      cruiseWithoutCRC[0], 
      cruiseWithoutCRC[1], 
      cruiseWithoutCRC[2], 
      cruiseWithoutCRC[3], 
      cruiseWithoutCRC[4], 
      cruiseWithoutCRC[5], 
      cruiseWithoutCRC[6] 
    };

    CAN.sendMsgBuf(0x289, 0, 8, cruiseWithCRC);
    
    //Restraint system (airbag?)
    unsigned char restraintWithoutCRC[] = { 0x40|counter4Bit, 0x40, 0x55, 0xFD, 0xFF, 0xFF, 0xFF };
    unsigned char restraintWithCRC[] = { crc8Calculator.get_crc8(restraintWithoutCRC, 7, 0xFF), restraintWithoutCRC[0], restraintWithoutCRC[1], restraintWithoutCRC[2], restraintWithoutCRC[3], restraintWithoutCRC[4], restraintWithoutCRC[5], restraintWithoutCRC[6] };
    CAN.sendMsgBuf(0x19B, 0, 8, restraintWithCRC);

    //Restraint system (seatbelt?)
    unsigned char restraint2WithoutCRC[] = { 0xE0|counter4Bit, 0xF1, 0xF0, 0xF2, 0xF2, 0xFE };
    unsigned char restraint2WithCRC[] = { crc8Calculator.get_crc8(restraint2WithoutCRC, 6, 0x28), restraint2WithoutCRC[0], restraint2WithoutCRC[1], restraint2WithoutCRC[2], restraint2WithoutCRC[3], restraint2WithoutCRC[4], restraint2WithoutCRC[5] };
    CAN.sendMsgBuf(0x297, 0, 7, restraint2WithCRC);

    //TPMS
    unsigned char TPMSWithoutCRC[] = { 0xF0|counter4Bit, 0xA2, 0xA0, 0xA0 };
    unsigned char TPMSWithCRC[] = { crc8Calculator.get_crc8(TPMSWithoutCRC, 4, 0xC5), TPMSWithoutCRC[0], TPMSWithoutCRC[1], TPMSWithoutCRC[2], TPMSWithoutCRC[3] };
    CAN.sendMsgBuf(0x369, 0, 5, TPMSWithCRC);

  // Unknown (makes RPM steady)
  // Also engine temp on diesel? Range 100 - 200
  unsigned char oilWithoutCRC[] = { 0x10|counter4Bit, 0x82, 0x4E, 0x7E, oilTemperature + 50, 0x05, 0x89 };
  unsigned char oilWithCRC[] = { crc8Calculator.get_crc8(oilWithoutCRC, 7, 0xF1), oilWithoutCRC[0], oilWithoutCRC[1], oilWithoutCRC[2], oilWithoutCRC[3], oilWithoutCRC[4], oilWithoutCRC[5], oilWithoutCRC[6] };
  CAN.sendMsgBuf(0x3F9, 0, 8, oilWithCRC);

  // Engine temperature
  // range: 0 - 200
  // CRC calculation for this one is weird... there is no counter present but scans show something like CRC
  unsigned char engineTempWithoutCRC[] = { 0x3e, oilTemperature, 0x64, 0x64, 0x64, 0x01, 0xF1 };
  unsigned char engineTempWithCRC[] = { crc8Calculator.get_crc8(engineTempWithoutCRC, 7, 0xB2), engineTempWithoutCRC[0], engineTempWithoutCRC[1], engineTempWithoutCRC[2], engineTempWithoutCRC[3], engineTempWithoutCRC[4], engineTempWithoutCRC[5], engineTempWithoutCRC[6] };
  CAN.sendMsgBuf(0x2C4, 0, 8, engineTempWithCRC);

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

void BMWFSeriesCluster::sendFuel(int fuelQuantity, uint8_t inFuelRange[], uint8_t outFuelRange[], bool isCarMini) {
  //Fuel
  uint8_t fuelQuantityLiters = multiMap<uint8_t>(fuelQuantity, inFuelRange, outFuelRange, 3);
  unsigned char fuelWithoutCRC[] = { (isCarMini ? 0 : hi8(fuelQuantityLiters)), (isCarMini ? 0 : lo8(fuelQuantityLiters)), hi8(fuelQuantityLiters), lo8(fuelQuantityLiters), 0x00 };
  CAN.sendMsgBuf(0x349, 0, 5, fuelWithoutCRC);
}

void BMWFSeriesCluster::sendDistanceTravelled(int speed) {
  // MPG bar
  unsigned char mpgWithoutCRC[] = { count, 0xFF, 0x64, 0x64, 0x64, 0x01, 0xF1 };
  unsigned char mpgWithCRC[] = { crc8Calculator.get_crc8(mpgWithoutCRC, 7, 0xC6), mpgWithoutCRC[0], mpgWithoutCRC[1], mpgWithoutCRC[2], mpgWithoutCRC[3], mpgWithoutCRC[4], mpgWithoutCRC[5], mpgWithoutCRC[6] };
  CAN.sendMsgBuf(0x2C4, 0, 8, mpgWithCRC);

  // MPG bar 2 (this one actually moves the bar)
  // The distance travelled counter is used to calculate the travelled distance shown in the cluster.
  unsigned char mpg2WithoutCRC[] = { 0xF0|counter4Bit, lo8(distanceTravelledCounter), hi8(distanceTravelledCounter), 0xF2 };
  unsigned char mpg2WithCRC[] = { crc8Calculator.get_crc8(mpg2WithoutCRC, 4, 0xde), mpg2WithoutCRC[0], mpg2WithoutCRC[1], mpg2WithoutCRC[2], mpg2WithoutCRC[3], mpg2WithoutCRC[4] };
  CAN.sendMsgBuf(0x2BB, 0, 5, mpg2WithCRC);
  distanceTravelledCounter += speed*2.9;
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

void BMWFSeriesCluster::sendAlerts(bool offroad, bool doorOpen, bool handbrake, bool isCarMini) {
  static bool lastDoorState = false;
  static bool lastDoorFL = false;
  static bool lastDoorFR = false;
  static bool lastDoorRL = false;
  static bool lastDoorRR = false;
  // ===============================
  // Individual door CC-ID mapping
  // 14 = Front Right
  // 15 = Front Left
  // 16 = Rear Left
  // 17 = Rear Right
  // ===============================

  // Front Right (CC-ID 14)
  if (doorOpen && !lastDoorFR) {
    uint8_t msg[] = { 0x40, 14, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg);
  } else if (!doorOpen && lastDoorFR) {
    uint8_t msg[] = { 0x40, 14, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg);
  }

  // Front Left (CC-ID 15)
  if (doorOpen && !lastDoorFL) {
    uint8_t msg[] = { 0x40, 15, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg);
  } else if (!doorOpen && lastDoorFL) {
    uint8_t msg[] = { 0x40, 15, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg);
  }

  // Rear Left (CC-ID 16)
  if (doorOpen && !lastDoorRL) {
    uint8_t msg[] = { 0x40, 16, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg);
  } else if (!doorOpen && lastDoorRL) {
    uint8_t msg[] = { 0x40, 16, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg);
  }

  // Rear Right (CC-ID 17)
  if (doorOpen && !lastDoorRR) {
    uint8_t msg[] = { 0x40, 17, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg);
  } else if (!doorOpen && lastDoorRR) {
    uint8_t msg[] = { 0x40, 17, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg);
  }

  // Check control messages
  // These are complicated since the same CAN ID is used to show variety of messages
  // Sending 0x29 on byte 4 sets the alert for the ID in byte 2, while sending 0x28 clears that message
  // Known IDs:
  // 34: check engine
  // 35, 215: DSC
  // 36: DSC OFF
  // 24: Park brake error (yellow)
  // 71: Park brake error (red)
  // 77 Seat belt indicator
    if (doorOpen) {
    // Door open 本身
    // uint8_t msg1[] = { 0x40, 0x0F, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    // CAN.sendMsgBuf(0x5C0, 0, 8, msg1);

    // Check engine
    uint8_t msg2[] = { 0x40, 34, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg2);

    // DSC
    uint8_t msg3[] = { 0x40, 212, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg3);

    // DSC OFF
    uint8_t msg4[] = { 0x40, 39, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg4);

    // Park brake yellow
    uint8_t msg5[] = { 0x40, 24, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg5);

    // Park brake red
    uint8_t msg6[] = { 0x40, 71, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg6);

    // Seat belt
    uint8_t msg7[] = { 0x40, 77, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg7);

    // ID 37
    uint8_t msg8[] = { 0x40, 37, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg8);

    // ID 75
    uint8_t msg9[] = { 0x40, 75, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg9);

    // ID 88
    uint8_t msg11[] = { 0x40, 88, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg11);

    // ID 91
    uint8_t msg12[] = { 0x40, 91, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg12);

    // ID 54
    uint8_t msg13[] = { 0x40, 54, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg13);

    // ID 87
    uint8_t msg14[] = { 0x40, 87, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg14);

    // ID 30
    uint8_t msg15[] = { 0x40, 30, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg15);




    // ===== EXTRA CC-ID (manual inject) =====

    uint8_t msg16[] = { 0x40, 50, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg16);

    uint8_t msg17[] = { 0x40, 63, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg17);

    uint8_t msg18[] = { 0x40, 21, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg18);

    uint8_t msg19[] = { 0x40, 22, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg19);

    uint8_t msg20[] = { 0x40, 26, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg20);

    uint8_t msg21[] = { 0x40, 28, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg21);

    uint8_t msg22[] = { 0x40, 29, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg22);

    uint8_t msg23[] = { 0x40, 31, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg23);

    uint8_t msg24[] = { 0x40, 33, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg24);

    uint8_t msg25[] = { 0x40, 41, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg25);

    uint8_t msg26[] = { 0x40, 51, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg26);

    uint8_t msg27[] = { 0x40, 103, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg27);

    uint8_t msg28[] = { 0x40, 108, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg28);

    uint8_t msg29[] = { 0x40, 255, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg29);

    // ===== EXTRA CC-ID (manual inject) - additional SET messages =====
    uint8_t msg30[] = { 0x40, 97, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg30);

    uint8_t msg31[] = { 0x40, 60, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg31);

    uint8_t msg32[] = { 0x40, 51, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg32);

    uint8_t msg33[] = { 0x40, 52, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg33);

    uint8_t msg34[] = { 0x40, 40, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg34);

    uint8_t msg35[] = { 0x40, 85, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg35);

    uint8_t msg36[] = { 0x40, 22, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg36);

    uint8_t msg37[] = { 0x40, 147, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg37);

    uint8_t msg38[] = { 0x40, 26, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg38);

    uint8_t msg39[] = { 0x40, 49, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg39);
      // ===== EXTRA CC-ID SET (new added) =====

    uint8_t msg40[] = { 0x40, 158, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg40);

    uint8_t msg41[] = { 0x40, 42, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg41);

    uint8_t msg42[] = { 0x40, 76, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg42);

    uint8_t msg43[] = { 0x40, 93, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg43);

    uint8_t msg44[] = { 0x40, 217, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg44);

    uint8_t msg45[] = { 0x40, 216, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg45);

    uint8_t msg46[] = { 0x40, 209, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg46);

    uint8_t msg47[] = { 0x40, 205, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg47);

    uint8_t msg48[] = { 0x40, 184, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg48);

    uint8_t msg49[] = { 0x40, 220, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg49);

    uint8_t msg50[] = { 0x40, 229, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg50);
  } else if (!doorOpen && lastDoorState) {
    // // Door open clear
    // uint8_t msg1[] = { 0x40, 0x0F, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    // CAN.sendMsgBuf(0x5C0, 0, 8, msg1);

    // Clear all others
    uint8_t msg2[] = { 0x40, 34, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg2);

    uint8_t msg3[] = { 0x40, 212, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg3);

    uint8_t msg4[] = { 0x40, 39, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg4);

    uint8_t msg5[] = { 0x40, 24, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg5);

    uint8_t msg6[] = { 0x40, 71, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg6);

    uint8_t msg7[] = { 0x40, 77, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg7);

     // Clear ID 37
    uint8_t msg8[] = { 0x40, 37, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg8);

    // Clear ID 75
    uint8_t msg9[] = { 0x40, 75, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg9);


    // Clear ID 88
    uint8_t msg11[] = { 0x40, 88, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg11);

    // Clear ID 91
    uint8_t msg12[] = { 0x40, 91, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg12);

    // Clear ID 54
    uint8_t msg13[] = { 0x40, 54, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg13);

    // Clear ID 87
    uint8_t msg14[] = { 0x40, 87, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg14);

    // Clear ID 30
    uint8_t msg15[] = { 0x40, 30, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg15);


        // ===== EXTRA CC-ID clear =====

    uint8_t msg16[] = { 0x40, 50, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg16);

    uint8_t msg17[] = { 0x40, 63, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg17);

    uint8_t msg18[] = { 0x40, 21, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg18);

    uint8_t msg19[] = { 0x40, 22, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg19);

    uint8_t msg20[] = { 0x40, 26, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg20);

    uint8_t msg21[] = { 0x40, 28, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg21);

    uint8_t msg22[] = { 0x40, 29, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg22);

    uint8_t msg23[] = { 0x40, 31, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg23);

    uint8_t msg24[] = { 0x40, 33, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg24);

    uint8_t msg25[] = { 0x40, 41, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg25);

    uint8_t msg26[] = { 0x40, 51, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg26);

    uint8_t msg27[] = { 0x40, 103, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg27);

    uint8_t msg28[] = { 0x40, 108, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg28);

    uint8_t msg29[] = { 0x40, 255, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg29);

    // ===== EXTRA CC-ID clear - additional CLEAR messages =====
    uint8_t msg30[] = { 0x40, 97, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg30);

    uint8_t msg31[] = { 0x40, 60, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg31);

    uint8_t msg32[] = { 0x40, 51, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg32);

    uint8_t msg33[] = { 0x40, 52, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg33);

    uint8_t msg34[] = { 0x40, 40, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg34);

    uint8_t msg35[] = { 0x40, 85, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg35);

    uint8_t msg36[] = { 0x40, 22, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg36);

    uint8_t msg37[] = { 0x40, 147, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg37);

    uint8_t msg38[] = { 0x40, 26, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg38);

    uint8_t msg39[] = { 0x40, 49, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg39);

    // ===== EXTRA CC-ID CLEAR (new added) =====

    uint8_t msg40[] = { 0x40, 158, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg40);

    uint8_t msg41[] = { 0x40, 42, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg41);

    uint8_t msg42[] = { 0x40, 76, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg42);

    uint8_t msg43[] = { 0x40, 93, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg43);

    uint8_t msg44[] = { 0x40, 217, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg44);

    uint8_t msg45[] = { 0x40, 216, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg45);

    uint8_t msg46[] = { 0x40, 209, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg46);

    uint8_t msg47[] = { 0x40, 205, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg47);

    uint8_t msg48[] = { 0x40, 184, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg48);

    uint8_t msg49[] = { 0x40, 220, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg49);

    uint8_t msg50[] = { 0x40, 229, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg50);
  }
  lastDoorFL = doorOpen;
  lastDoorFR = doorOpen;
  lastDoorRL = doorOpen;
  lastDoorRR = doorOpen;
  lastDoorState = doorOpen;
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
  //Menu buttons
  uint8_t buttonMessage[] = { 0x00, 0xFF };
  if (buttonEvent == 1) {
    buttonMessage[0] = 76;
  }
  CAN.sendMsgBuf(0x1EE, 0, 2, buttonMessage);
}

void BMWFSeriesCluster::updateLanguageAndUnits() {
  //language and units
  //CanSend(0x291, 2, 18, 0, 0x00, 0x00, 0x00, 0x00, 0x00);
  //Byte:1 language
  //Byte 2: Clock and Temp layout,format eg celius,fainhieght and am,pm
  //Byte 3: miles or km
  //Above is english, mpg
  //CanSend(0x291, 2, 18, 1, 0x00, 0x00, 0x00, 0x00, 0x00);
  //Aboove is english, l/100km
  //   uint8_t msg[] = { 
  //     0x12,   // language = English (假设值)
  //     0x01,   // 12h format + Celsius
  //     0x00,   // km
  //     0x00, 
  //     0x00,
  //     0x00,
  //     0x00,
  //     0x00
  // };

  // CAN.sendMsgBuf(0x291, 0, 8, msg);
}

void BMWFSeriesCluster::sendDriveMode(uint8_t driveMode) {
  //1= Traction, 2= Comfort, 4= Sport, 5= Sport+, 6= DSC off, 7= Eco pro 
  unsigned char modeWithoutCRC[] = { 0xF0|counter4Bit, 0, 0, driveMode, 0x11, 0xC0 };
  unsigned char modeWithCRC[] = { crc8Calculator.get_crc8(modeWithoutCRC, 6, 0x4a), modeWithoutCRC[0], modeWithoutCRC[1], modeWithoutCRC[2], modeWithoutCRC[3], modeWithoutCRC[4], modeWithoutCRC[5] };
  CAN.sendMsgBuf(0x3A7, 0, 7, modeWithCRC);
}

void BMWFSeriesCluster::sendAcc() {
  unsigned char accWithoutCrc[] = { 0xF0|accCounter, 0x5C, 0x70, 0x00, 0x00 };
  unsigned char accWithCrc[] = { crc8Calculator.get_crc8(accWithoutCrc, 5, 0x6b), accWithoutCrc[0], accWithoutCrc[1], accWithoutCrc[2], accWithoutCrc[3], accWithoutCrc[4] };
  CAN.sendMsgBuf(0x33b, 0, 6, accWithCrc);
      
  accCounter += 4;
  if (accCounter > 0x0E) {
    accCounter = accCounter - 0x0F;
  }
}