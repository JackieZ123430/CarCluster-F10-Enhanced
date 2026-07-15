  uint8_t ignitionStatus = ignition ? 0x8A : 0x8;
  unsigned char ignitionWithoutCRC[] = { (uint8_t)(0x80|counter4Bit), ignitionStatus, 0xDD, 0xF1, 0x01, 0x30, 0x06 };
  unsigned char ignitionWithCRC[] = { crc8Calculator.get_crc8(ignitionWithoutCRC, 7, 0x44), ignitionWithoutCRC[0], ignitionWithoutCRC[1], ignitionWithoutCRC[2], ignitionWithoutCRC[3], ignitionWithoutCRC[4], ignitionWithoutCRC[5], ignitionWithoutCRC[6] };
  CAN.sendMsgBuf(0x12F, 0, 8, ignitionWithCRC);
}

void BMWFSeriesCluster::sendSpeed(int speed) {
  uint16_t calculatedSpeed = (double)speed * 64.01;
  unsigned char speedWithoutCRC[] = { (uint8_t)(0xC0|counter4Bit), lo8(calculatedSpeed), hi8(calculatedSpeed), (uint8_t)((speed == 0 ? 0x81 : 0x91)) };
  unsigned char speedWithCRC[] = { crc8Calculator.get_crc8(speedWithoutCRC, 4, 0xA9), speedWithoutCRC[0], speedWithoutCRC[1], speedWithoutCRC[2], speedWithoutCRC[3] };
  CAN.sendMsgBuf(0x1A1, 0, 5, speedWithCRC);
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

  // BMW F3x scaling factor: cluster expects approximately rpm * 1.557.
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
