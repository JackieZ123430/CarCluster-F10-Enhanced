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
  if (brightness > 100) brightness = 100;
  uint8_t mappedBrightness = map(brightness, 0, 100, 0, 253);
  unsigned char backlightBrightnessWithoutCRC[] = { mappedBrightness, 0xFF };
  CAN.sendMsgBuf(0x202, 0, 2, backlightBrightnessWithoutCRC);
}

void BMWFSeriesCluster::sendAlerts(GameState& game, bool stabilityIntervention) {
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
    uint8_t msg[] = { 0x40, 14, 0x00, (uint8_t)(fr ? 0x29 : 0x28), 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg);
    lastDoorFR = fr;
  }

  if (fl != lastDoorFL) {
    uint8_t msg[] = { 0x40, 15, 0x00, (uint8_t)(fl ? 0x29 : 0x28), 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg);
    lastDoorFL = fl;
  }

  if (rl != lastDoorRL) {
    uint8_t msg[] = { 0x40, 16, 0x00, (uint8_t)(rl ? 0x29 : 0x28), 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg);
    lastDoorRL = rl;
  }

  if (rr != lastDoorRR) {
    uint8_t msg[] = { 0x40, 17, 0x00, (uint8_t)(rr ? 0x29 : 0x28), 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5C0, 0, 8, msg);
    lastDoorRR = rr;
  }
  if (stabilityIntervention) {
    uint8_t message[] = { 0x40, 215, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5c0, 0, 8, message);
  } else {
    uint8_t message[] = { 0x40, 215, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    CAN.sendMsgBuf(0x5c0, 0, 8, message);
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
  unsigned char modeWithoutCRC[] = { (uint8_t)(0xF0|counter4Bit), 0, 0, driveMode, 0x11, 0xC0 };
  unsigned char modeWithCRC[] = { crc8Calculator.get_crc8(modeWithoutCRC, 6, 0x4a), modeWithoutCRC[0], modeWithoutCRC[1], modeWithoutCRC[2], modeWithoutCRC[3], modeWithoutCRC[4], modeWithoutCRC[5] };
  CAN.sendMsgBuf(0x3A7, 0, 7, modeWithCRC);
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
      hours, minutes, 0x00, 0x01, 0x01, 0xDF, 0x07, 0xF2};
  CAN.sendMsgBuf(0x39E, 0, 8, timeFrame);
}
