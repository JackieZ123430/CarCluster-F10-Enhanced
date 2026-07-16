// ####################################################################################################################
//
// BMW F10/F-series cluster implementation
// Author / maintainer: JackieZ123430
// Project: https://github.com/JackieZ123430/CarCluster-F10-Enhanced
// Better_CAN: https://github.com/JackieZ123430/Better_CAN
// Original upstream project: https://github.com/r00li/CarCluster
//
// 仅供个人学习、研究及非商业用途。禁止倒卖、付费分发或包装成收费产品。
// 如果你通过第三方付费获得本项目，请及时申请退款，并保留商品页面和付款记录后举报卖家。
// Personal learning, research and non-commercial use only. Unauthorized resale or paid redistribution is prohibited.
// Keep the author name and project links in permitted copies and forks.
// CAN-bus and instrument-cluster experiments can damage hardware when wired or powered incorrectly; use at your own risk.
//
// Runtime notes
// - CC-ID 67 is sent once after ignition has remained on for five seconds.
// - CC-ID 78 follows the speed threshold state and is active above 160 km/h.
// - Warning messages use state caching where possible to avoid repeated activation/clear frames.
// - The steering-wheel output accepts the BC/menu action only.
//
// ####################################################################################################################

#include "BMWFSeriesCluster.h"

BMWFSeriesCluster::BMWFSeriesCluster(MCP_CAN& CAN) : CAN(CAN) {
  crc8Calculator.begin();
}

uint8_t BMWFSeriesCluster::mapGenericGearToLocalGear(GearState inputGear) {
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
    case GearState_Auto_D:
    case GearState_Auto_S:
      return 13;
    default:
      return 0;
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
  if (game.coolantTemperature < game.configuration.minimumCoolantTemperature) {
    return game.configuration.minimumCoolantTemperature;
  }
  if (game.coolantTemperature > game.configuration.maximumCoolantTemperature) {
    return game.configuration.maximumCoolantTemperature;
  }
  return game.coolantTemperature;
}

void BMWFSeriesCluster::updateWithGame(GameState& game) {
  if (game.buttonEventToProcess != 0) {
    sendSteeringWheelButton(game.buttonEventToProcess);
    game.buttonEventToProcess = 0;
  }

  static bool lastIgnition = false;
  static unsigned long ignitionOnTime = 0;
  static bool cc67Sent = false;

  if (game.ignition && !lastIgnition) {
    ignitionOnTime = millis();
    cc67Sent = false;
  }

  if (!game.ignition && lastIgnition) {
    if (cc67Sent) {
      uint8_t msg67_off[] = {0x40, 67, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF};
      CAN.sendMsgBuf(0x5C0, 0, 8, msg67_off);
    }
    cc67Sent = false;
  }

  lastIgnition = game.ignition;

  if (millis() - lastDashboardUpdateTime >= dashboardUpdateTimeFast) {
    sendIgnitionStatus(game.ignition);

    // CC-ID 58 parking-brake indication after two seconds at zero speed.
    static unsigned long zeroSpeedStartTime = 0;
    static bool autoHoldActive = false;
    bool wantAutoHold = false;

    if (game.ignition && game.speed == 0) {
      if (zeroSpeedStartTime == 0) zeroSpeedStartTime = millis();
      if (millis() - zeroSpeedStartTime >= 2000) wantAutoHold = true;
    } else {
      zeroSpeedStartTime = 0;
    }

    if (wantAutoHold) {
      uint8_t msg58_on[] = {0x40, 58, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF};
      CAN.sendMsgBuf(0x5C0, 0, 8, msg58_on);
      autoHoldActive = true;
    } else if (autoHoldActive) {
      uint8_t msg58_off[] = {0x40, 58, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF};
      CAN.sendMsgBuf(0x5C0, 0, 8, msg58_off);
      autoHoldActive = false;
    }

    // Automatic high-beam keep-alive.
    {
      uint8_t ahbFrame[8];
      const uint8_t ahbVal = game.highBeam ? 0x02 : 0x01;
      for (uint8_t i = 0; i < 8; i++) ahbFrame[i] = ahbVal;
      CAN.sendMsgBuf(0x36A, 0, 8, ahbFrame);
    }

    // Automatic start/stop keep-alive.
    {
      uint8_t assFrame[8];
      const uint8_t assVal = game.ignition ? 0x1A : 0xE6;
      for (uint8_t i = 0; i < 8; i++) assFrame[i] = assVal;
      CAN.sendMsgBuf(0x30B, 0, 8, assFrame);
    }

    // CC-ID 67: Remote Control/Key Battery Discharged.
    if (game.ignition && !cc67Sent && millis() - ignitionOnTime >= 5000) {
      uint8_t msg67_on[] = {0x40, 67, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF};
      CAN.sendMsgBuf(0x5C0, 0, 8, msg67_on);
      cc67Sent = true;
    }

    // CC-ID 40: Press Brake to Start.
    static bool lastPressBrakeState = false;
    const bool pressBrakeState = game.ignition && game.rpm < 10;
    if (pressBrakeState != lastPressBrakeState) {
      uint8_t msg40[] = {
        0x40, 40, 0x00, static_cast<uint8_t>(pressBrakeState ? 0x29 : 0x28),
        0xFF, 0xFF, 0xFF, 0xFF
      };
      CAN.sendMsgBuf(0x5C0, 0, 8, msg40);
      lastPressBrakeState = pressBrakeState;
    }

    // Engine-stopped warning state with hysteresis.
    static bool lastEngineStoppedState = false;
    bool engineStoppedNow = false;

    if (game.ignition) {
      if (game.rpm < 50) {
        engineStoppedNow = true;
      } else if (game.rpm > 150) {
        engineStoppedNow = false;
      } else {
        engineStoppedNow = lastEngineStoppedState;
      }
    }

    if (engineStoppedNow != lastEngineStoppedState) {
      const uint8_t engineStoppedIds[] = {21, 30};
      for (uint8_t i = 0; i < sizeof(engineStoppedIds); i++) {
        uint8_t msg[] = {
          0x40,
          engineStoppedIds[i],
          0x00,
          static_cast<uint8_t>(engineStoppedNow ? 0x29 : 0x28),
          0xFF, 0xFF, 0xFF, 0xFF
        };
        CAN.sendMsgBuf(0x5C0, 0, 8, msg);
      }
      lastEngineStoppedState = engineStoppedNow;
    }

    sendSpeed(mapSpeed(game));
    sendRPM(mapRPM(game), mapGenericGearToLocalGear(game.gear));
    sendBasicDriveInfo(game, game.oilTemperature);

    // Gateway, body and chassis keep-alive frames for standalone cluster operation.
    {
      unsigned char vehicleStatus[8] = {
        0xFF, 0xFF, 0xC0, 0xFF, 0xFF, 0xFF, 0xF0,
        static_cast<uint8_t>(random(0xFC, 0xFD))
      };
      CAN.sendMsgBuf(0x3A0, 0, 8, vehicleStatus);

      unsigned char bodyController[8] = {0x00, count, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
      CAN.sendMsgBuf(0xB68, 0, 8, bodyController);

      unsigned char gatewayFrame[2] = {0x79, 0x20};
      CAN.sendMsgBuf(0x381, 0, 2, gatewayFrame);
    }

    {
      uint8_t icmFrame[8] = {
        static_cast<uint8_t>(0xF0 | counter4Bit), 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00
      };
      CAN.sendMsgBuf(0x130, 0, 8, icmFrame);

      uint8_t steeringAngleFrame[8] = {
        static_cast<uint8_t>(0xF0 | counter4Bit), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
      };
      CAN.sendMsgBuf(0x0C4, 0, 8, steeringAngleFrame);

      uint8_t wheelSpeedFrame[8] = {
        static_cast<uint8_t>(0xF0 | counter4Bit), 0x10, 0x10, 0x10, 0x10, 0x00, 0x00, 0x00
      };
      CAN.sendMsgBuf(0x0AA, 0, 8, wheelSpeedFrame);

      uint8_t batteryFrame[8] = {
        static_cast<uint8_t>(0xF0 | counter4Bit), 0x64, 0x64, 0x64, 0x64, 0x64, 0x64, 0x64
      };
      CAN.sendMsgBuf(0x3D0, 0, 8, batteryFrame);
    }

    sendAutomaticTransmission(game.gear, game.gearIndex);

    if (game.gear == GearState_Auto_N) {
      unsigned char neutralWithoutCRC[] = {
        static_cast<uint8_t>(0xF0 | counter4Bit), 0x60, 0xFC, 0xFF
      };
      unsigned char neutralWithCRC[] = {
        crc8Calculator.get_crc8(neutralWithoutCRC, 4, 0x5A),
        neutralWithoutCRC[0], neutralWithoutCRC[1], neutralWithoutCRC[2], neutralWithoutCRC[3]
      };
      CAN.sendMsgBuf(0x178, 0, 5, neutralWithCRC);

      uint8_t msg169_on[] = {0x40, 169, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF};
      uint8_t msg203_on[] = {0x40, 203, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF};
      CAN.sendMsgBuf(0x5C0, 0, 8, msg169_on);
      CAN.sendMsgBuf(0x5C0, 0, 8, msg203_on);
    } else {
      uint8_t msg169_off[] = {0x40, 169, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF};
      uint8_t msg203_off[] = {0x40, 203, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF};
      CAN.sendMsgBuf(0x5C0, 0, 8, msg169_off);
      CAN.sendMsgBuf(0x5C0, 0, 8, msg203_off);
    }

    sendFuel(game.fuelQuantity);
    sendParkBrake(game.handbrake);
    sendDistanceTravelled(mapSpeed(game));
    sendAlerts(game, game.offroadLight);

    // CC-ID 78: Vehicle Speed Limit Exceeded.
    static bool lastOver160 = false;
    const bool over160 = game.speed > 160;
    if (over160 != lastOver160) {
      uint8_t msg78[] = {
        0x40, 78, 0x00, static_cast<uint8_t>(over160 ? 0x29 : 0x28),
        0xFF, 0xFF, 0xFF, 0xFF
      };
      CAN.sendMsgBuf(0x5C0, 0, 8, msg78);
      lastOver160 = over160;
    }

    // Engine warning output uses state-change-only transmission.
    static bool lastEngineLightState = false;
    if (game.engineLight != lastEngineLightState) {
      uint8_t msg50[] = {
        0x40, 50, 0x00, static_cast<uint8_t>(game.engineLight ? 0x29 : 0x28),
        0xFF, 0xFF, 0xFF, 0xFF
      };
      CAN.sendMsgBuf(0x5C0, 0, 8, msg50);
      lastEngineLightState = game.engineLight;
    }

    // TPMS ECU keep-alive.
    {
      unsigned char tpmsWithoutCRC[] = {
        static_cast<uint8_t>(0xF0 | counter4Bit), 0xA2, 0xA0, 0xA0
      };
      unsigned char tpmsWithCRC[] = {
        crc8Calculator.get_crc8(tpmsWithoutCRC, 4, 0xC5),
        tpmsWithoutCRC[0], tpmsWithoutCRC[1], tpmsWithoutCRC[2], tpmsWithoutCRC[3]
      };
      CAN.sendMsgBuf(0x369, 0, 5, tpmsWithCRC);
    }

    if (game.ignition && millis() - ignitionOnTime < 3000) {
      updateLanguageAndUnits();
    }

    counter4Bit++;
    if (counter4Bit > 14) counter4Bit = 0;

    count++;
    if (count >= 254) count = 0;

    lastDashboardUpdateTime = millis();
  }

  // Manual CC-ID injection for bench testing.
  if (game.alertStart) {
    uint8_t msg[] = {0x40, game.alertId, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF};
    CAN.sendMsgBuf(0x5C0, 0, 8, msg);
    game.alertStart = false;
  }

  if (game.alertClear) {
    uint8_t msg[] = {0x40, game.alertId, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF};
    CAN.sendMsgBuf(0x5C0, 0, 8, msg);
    game.alertClear = false;
  }

  if (millis() - lastDashboardUpdateTimeLights >= dashboardUpdateTimeLights) {
    sendLights(game.mainLights, game.highBeam, game.rearFogLight, game.frontFogLight);
    sendBlinkers(game.leftTurningIndicator, game.rightTurningIndicator);
    lastDashboardUpdateTimeLights = millis();
  }

  if (millis() - lastDashboardUpdateTime1000ms >= dashboardUpdateTimeSlow) {
    sendBacklightBrightness(game.backlightBrightness);

    uint8_t driveModeToSend = game.driveMode;
    if (driveModeToSend != 1 && driveModeToSend != 2 && driveModeToSend != 4 &&
        driveModeToSend != 5 && driveModeToSend != 6 && driveModeToSend != 7) {
      driveModeToSend = 2;
    }
    sendDriveMode(driveModeToSend);

    sendOutsideTemperature(game.outdoorTemperature);
    const unsigned long totalSeconds = game.time / 1000UL;
    const uint8_t hours = static_cast<uint8_t>((totalSeconds / 3600UL) % 24UL);
    const uint8_t minutes = static_cast<uint8_t>((totalSeconds / 60UL) % 60UL);
    sendTime(hours, minutes);

    updateLanguageAndUnits();
    lastDashboardUpdateTime1000ms = millis();
  }
}
