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

// ####################################################################################################################
// BMW F10 output helpers
// Author / maintainer: JackieZ123430
// Project: https://github.com/JackieZ123430/CarCluster-F10-Enhanced
// Better_CAN: https://github.com/JackieZ123430/Better_CAN
//
// 仅供个人学习、研究及非商业用途。禁止倒卖、付费分发或包装成收费产品。
// 如果你通过第三方付费获得本项目，请及时申请退款，并保留商品页面和付款记录后举报卖家。
// Personal learning, research and non-commercial use only. Preserve author and project attribution.
// ####################################################################################################################

void BMWFSeriesCluster::sendBlinkers(bool leftTurningIndicator, bool rightTurningIndicator) {
  const uint8_t blinkerStatus =
      (!leftTurningIndicator && !rightTurningIndicator)
          ? 0x80
          : static_cast<uint8_t>(0x81 | (leftTurningIndicator << 4) | (rightTurningIndicator << 5));
  unsigned char blinkersWithoutCRC[] = {blinkerStatus, 0xF0};
  CAN.sendMsgBuf(0x1F6, 0, 2, blinkersWithoutCRC);
}

void BMWFSeriesCluster::sendLights(bool mainLights, bool highBeam, bool rearFogLight, bool frontFogLight) {
  if (highBeam) mainLights = true;

  const uint8_t lightStatus = static_cast<uint8_t>(
      (highBeam << 1) | (mainLights << 2) | (frontFogLight << 5) | (rearFogLight << 6));
  unsigned char lightsWithoutCRC[] = {lightStatus, 0xC0, 0xF7};
  CAN.sendMsgBuf(0x21A, 0, 3, lightsWithoutCRC);
}

void BMWFSeriesCluster::sendBacklightBrightness(uint8_t brightness) {
  if (brightness > 100) brightness = 100;
  const uint8_t mappedBrightness = map(brightness, 0, 100, 0, 253);
  unsigned char backlightBrightnessWithoutCRC[] = {mappedBrightness, 0xFF};
  CAN.sendMsgBuf(0x202, 0, 2, backlightBrightnessWithoutCRC);
}

void BMWFSeriesCluster::sendAlerts(GameState& game, bool stabilityIntervention) {
  static bool lastDoorFL = false;
  static bool lastDoorFR = false;
  static bool lastDoorRL = false;
  static bool lastDoorRR = false;

  if (game.doorFR != lastDoorFR) {
    uint8_t msg[] = {0x40, 14, 0x00, static_cast<uint8_t>(game.doorFR ? 0x29 : 0x28), 0xFF, 0xFF, 0xFF, 0xFF};
    CAN.sendMsgBuf(0x5C0, 0, 8, msg);
    lastDoorFR = game.doorFR;
  }

  if (game.doorFL != lastDoorFL) {
    uint8_t msg[] = {0x40, 15, 0x00, static_cast<uint8_t>(game.doorFL ? 0x29 : 0x28), 0xFF, 0xFF, 0xFF, 0xFF};
    CAN.sendMsgBuf(0x5C0, 0, 8, msg);
    lastDoorFL = game.doorFL;
  }

  if (game.doorRL != lastDoorRL) {
    uint8_t msg[] = {0x40, 16, 0x00, static_cast<uint8_t>(game.doorRL ? 0x29 : 0x28), 0xFF, 0xFF, 0xFF, 0xFF};
    CAN.sendMsgBuf(0x5C0, 0, 8, msg);
    lastDoorRL = game.doorRL;
  }

  if (game.doorRR != lastDoorRR) {
    uint8_t msg[] = {0x40, 17, 0x00, static_cast<uint8_t>(game.doorRR ? 0x29 : 0x28), 0xFF, 0xFF, 0xFF, 0xFF};
    CAN.sendMsgBuf(0x5C0, 0, 8, msg);
    lastDoorRR = game.doorRR;
  }

  uint8_t stabilityMessage[] = {
    0x40, 215, 0x00, static_cast<uint8_t>(stabilityIntervention ? 0x29 : 0x28),
    0xFF, 0xFF, 0xFF, 0xFF
  };
  CAN.sendMsgBuf(0x5C0, 0, 8, stabilityMessage);
}

void BMWFSeriesCluster::sendSteeringWheelButton(int buttonEvent) {
  // BC/menu action only.
  if (buttonEvent != 1) return;

  uint8_t pressFrame[2] = {0x4C, 0xFF};
  CAN.sendMsgBuf(0x1EE, 0, 2, pressFrame);

  delay(40);

  uint8_t releaseFrame[2] = {0x00, 0xFF};
  CAN.sendMsgBuf(0x1EE, 0, 2, releaseFrame);
}

void BMWFSeriesCluster::updateLanguageAndUnits() {
  const uint8_t language = 0x01;
  const uint8_t byte2 = 18;
  const uint8_t byte3 = 89;

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
  unsigned char modeWithoutCRC[] = {
    static_cast<uint8_t>(0xF0 | counter4Bit), 0, 0, driveMode, 0x11, 0xC0
  };
  unsigned char modeWithCRC[] = {
    crc8Calculator.get_crc8(modeWithoutCRC, 6, 0x4A),
    modeWithoutCRC[0], modeWithoutCRC[1], modeWithoutCRC[2],
    modeWithoutCRC[3], modeWithoutCRC[4], modeWithoutCRC[5]
  };
  CAN.sendMsgBuf(0x3A7, 0, 7, modeWithCRC);
}

float BMWFSeriesCluster::mapRPMValueForFuelModel(int speed) {
  float normalized = speed / 200.0f;
  if (normalized > 1.0f) normalized = 1.0f;
  if (normalized < 0.0f) normalized = 0.0f;
  return normalized;
}

void BMWFSeriesCluster::sendOutsideTemperature(int temperature) {
  if (temperature < -40) temperature = -40;
  if (temperature > 87) temperature = 87;

  const uint8_t tempByte = static_cast<uint8_t>((temperature * 2) + 80);
  unsigned char tempFrame[2] = {tempByte, 0xFF};
  CAN.sendMsgBuf(0x2CA, 0, 2, tempFrame);
}

void BMWFSeriesCluster::sendTime(uint8_t hours, uint8_t minutes) {
  if (hours > 23) hours = 0;
  if (minutes > 59) minutes = 0;

  unsigned char timeFrame[8] = {
    hours, minutes, 0x00, 0x01, 0x01, 0xDF, 0x07, 0xF2
  };
  CAN.sendMsgBuf(0x39E, 0, 8, timeFrame);
}
