// ####################################################################################################################
// BMW F10 implementation continuation
// Author / maintainer: JackieZ123430
// Project: https://github.com/JackieZ123430/CarCluster-F10-Enhanced
//
// Personal learning, research and non-commercial use only.
// Unauthorized resale or paid redistribution is prohibited. If this project was sold to you, request a refund and
// report the seller with the listing and evidence.
// ####################################################################################################################

  // TPMS CC-ID mapping. These messages are sent only when the corresponding tyre state changes.
  // 139 = front left, 143 = front right, 141 = rear left, 140 = rear right, 142 = global tyre-pressure warning.
  static bool lastFL = false;
  static bool lastFR = false;
  static bool lastRL = false;
  static bool lastRR = false;
  static bool lastGlobal = false;

  if (game.tireDefFL != lastFL) {
    uint8_t msg[] = {0x40, 139, 0x00, static_cast<uint8_t>(game.tireDefFL ? 0x29 : 0x28), 0xFF, 0xFF, 0xFF, 0xFF};
    CAN.sendMsgBuf(0x5C0, 0, 8, msg);
    lastFL = game.tireDefFL;
  }

  if (game.tireDefFR != lastFR) {
    uint8_t msg[] = {0x40, 143, 0x00, static_cast<uint8_t>(game.tireDefFR ? 0x29 : 0x28), 0xFF, 0xFF, 0xFF, 0xFF};
    CAN.sendMsgBuf(0x5C0, 0, 8, msg);
    lastFR = game.tireDefFR;
  }

  if (game.tireDefRL != lastRL) {
    uint8_t msg[] = {0x40, 141, 0x00, static_cast<uint8_t>(game.tireDefRL ? 0x29 : 0x28), 0xFF, 0xFF, 0xFF, 0xFF};
    CAN.sendMsgBuf(0x5C0, 0, 8, msg);
    lastRL = game.tireDefRL;
  }

  if (game.tireDefRR != lastRR) {
    uint8_t msg[] = {0x40, 140, 0x00, static_cast<uint8_t>(game.tireDefRR ? 0x29 : 0x28), 0xFF, 0xFF, 0xFF, 0xFF};
    CAN.sendMsgBuf(0x5C0, 0, 8, msg);
    lastRR = game.tireDefRR;
  }

  const bool anyTireDeflated =
      game.tireDefFL || game.tireDefFR || game.tireDefRL || game.tireDefRR;

  if (anyTireDeflated != lastGlobal) {
    uint8_t globalMsg[] = {0x40, 142, 0x00, static_cast<uint8_t>(anyTireDeflated ? 0x29 : 0x28), 0xFF, 0xFF, 0xFF, 0xFF};
    CAN.sendMsgBuf(0x5C0, 0, 8, globalMsg);
    lastGlobal = anyTireDeflated;
  }

  // Oil-temperature frame used by the F-series cluster gauge path.
  int encodedOilTemperature = oilTemperature + 50;
  if (encodedOilTemperature < 0) encodedOilTemperature = 0;
  if (encodedOilTemperature > 255) encodedOilTemperature = 255;

  unsigned char oilWithoutCRC[] = {
    static_cast<uint8_t>(0x10 | counter4Bit), 0x82, 0x4E, 0x7E,
    static_cast<uint8_t>(encodedOilTemperature), 0x05, 0x89
  };
  unsigned char oilWithCRC[] = {
    crc8Calculator.get_crc8(oilWithoutCRC, 7, 0xF1),
    oilWithoutCRC[0], oilWithoutCRC[1], oilWithoutCRC[2], oilWithoutCRC[3],
    oilWithoutCRC[4], oilWithoutCRC[5], oilWithoutCRC[6]
  };
  CAN.sendMsgBuf(0x3F9, 0, 8, oilWithCRC);

  // CC-ID 39: engine/coolant overheat. This remains active because it matches the measured temperature conditions.
  if (oilTemperature > 130 || game.coolantTemperature > 115) {
    uint8_t msg39_on[] = {0x40, 39, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF};
    CAN.sendMsgBuf(0x5C0, 0, 8, msg39_on);
  } else {
    uint8_t msg39_off[] = {0x40, 39, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF};
    CAN.sendMsgBuf(0x5C0, 0, 8, msg39_off);
  }

  // Gearbox-temperature warnings retained from the existing build.
  if (oilTemperature > 120 && game.rpm > 3500) {
    uint8_t msg103[] = {0x40, 103, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF};
    CAN.sendMsgBuf(0x5C0, 0, 8, msg103);
  }

  if (oilTemperature > 130 && game.speed > 80) {
    uint8_t msg104[] = {0x40, 104, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF};
    CAN.sendMsgBuf(0x5C0, 0, 8, msg104);
  }

  if (oilTemperature > 140) {
    uint8_t msg105[] = {0x40, 105, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF};
    CAN.sendMsgBuf(0x5C0, 0, 8, msg105);
  }
}

void BMWFSeriesCluster::sendParkBrake(bool handbrakeActive) {
  unsigned char abs3WithoutCRC[] = {
    static_cast<uint8_t>(0xF0 | counter4Bit), 0x38, 0,
    static_cast<uint8_t>(handbrakeActive ? 0x15 : 0x14)
  };
  unsigned char abs3WithCRC[] = {
    crc8Calculator.get_crc8(abs3WithoutCRC, 4, 0x17),
    abs3WithoutCRC[0], abs3WithoutCRC[1], abs3WithoutCRC[2], abs3WithoutCRC[3]
  };
  CAN.sendMsgBuf(0x36F, 0, 5, abs3WithCRC);
}

void BMWFSeriesCluster::sendFuel(float fuelPercent) {
  // Better_CAN, SimHub and WebDashboard all use a normalized 0-100 percentage.
  if (fuelPercent < 0.0f) fuelPercent = 0.0f;
  if (fuelPercent > 100.0f) fuelPercent = 100.0f;

  const uint8_t mappedFuel = multiMap<uint8_t>(
      static_cast<uint8_t>(fuelPercent),
      inFuelRange,
      outFuelRange,
      3);

  unsigned char fuelFrame[5] = {
      hi8(mappedFuel),
      lo8(mappedFuel),
      hi8(mappedFuel),
      lo8(mappedFuel),
      0x00};

  CAN.sendMsgBuf(0x349, 0, 5, fuelFrame);
}

void BMWFSeriesCluster::sendDistanceTravelled(int speed) {
  // Approximate instantaneous fuel-consumption model used to animate the cluster's MPG display.
  static float virtualDistanceAccumulator = 0.0f;

  unsigned char mpgWithoutCRC[] = {count, 0xFF, 0x64, 0x64, 0x64, 0x01, 0xF1};
  unsigned char mpgWithCRC[] = {
    crc8Calculator.get_crc8(mpgWithoutCRC, 7, 0xC6),
    mpgWithoutCRC[0], mpgWithoutCRC[1], mpgWithoutCRC[2], mpgWithoutCRC[3],
    mpgWithoutCRC[4], mpgWithoutCRC[5], mpgWithoutCRC[6]
  };
  CAN.sendMsgBuf(0x2C4, 0, 8, mpgWithCRC);

  float rpmFactor = static_cast<float>(mapRPMValueForFuelModel(speed));
  if (rpmFactor < 0.1f) rpmFactor = 0.1f;

  float fuelBurnRate = 0.8f + (rpmFactor * 2.5f);
  if (speed < 3) fuelBurnRate *= 3.0f;

  const float distanceDelta = static_cast<float>(speed) / fuelBurnRate;
  virtualDistanceAccumulator += distanceDelta;

  if (virtualDistanceAccumulator > 65535.0f) virtualDistanceAccumulator = 0.0f;
  distanceTravelledCounter = static_cast<uint16_t>(virtualDistanceAccumulator);

  unsigned char mpg2WithoutCRC[] = {
    static_cast<uint8_t>(0xF0 | counter4Bit),
