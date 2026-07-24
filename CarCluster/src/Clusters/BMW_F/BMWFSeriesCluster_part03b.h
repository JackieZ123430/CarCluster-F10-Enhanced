// ####################################################################################################################
// BMW F10 implementation continuation
// Author / maintainer: JackieZ123430
// Project: https://github.com/JackieZ123430/CarCluster-F10-Enhanced
// Better_CAN: https://github.com/JackieZ123430/Better_CAN
//
// 仅供个人学习、研究及非商业用途。禁止倒卖或付费分发。
// 如果你通过第三方付费获得本项目，请及时申请退款并举报卖家。
// ####################################################################################################################

    switch (gear) {
      case GearState_Auto_P: selectedGear = 0x20; break;
      case GearState_Auto_R: selectedGear = 0x40; break;
      case GearState_Auto_N: selectedGear = 0x60; break;
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

  CAN.sendMsgBuf(0x3FD, 0, 5, transmissionWithCRC);
}

void BMWFSeriesCluster::sendBasicDriveInfo(GameState& game, int oilTemperature) {
  uint8_t absStatusByte = 0x14;
  unsigned char abs1WithoutCRC[] = {
    static_cast<uint8_t>(0xF0 | counter4Bit), 0xFE, 0xFF, absStatusByte
  };
  unsigned char abs1WithCRC[] = {
    crc8Calculator.get_crc8(abs1WithoutCRC, 4, 0xD8),
    abs1WithoutCRC[0], abs1WithoutCRC[1], abs1WithoutCRC[2], abs1WithoutCRC[3]
  };
  CAN.sendMsgBuf(0x36E, 0, 5, abs1WithCRC);

  unsigned char absSecondary[8] = {
    counter4Bit, counter4Bit, counter4Bit, counter4Bit,
    counter4Bit, counter4Bit, counter4Bit, counter4Bit
  };
  CAN.sendMsgBuf(0xB6E, 0, 8, absSecondary);

  unsigned char aliveCounterSafetyWithoutCRC[] = {count, 0xFF};
  CAN.sendMsgBuf(0xD7, 0, 2, aliveCounterSafetyWithoutCRC);

  unsigned char steeringColumnWithoutCRC[] = {
    static_cast<uint8_t>(0xF0 | counter4Bit), 0xFE, 0xFF, 0x14
  };
  unsigned char steeringColumnWithCRC[] = {
    crc8Calculator.get_crc8(steeringColumnWithoutCRC, 4, 0x9E),
    steeringColumnWithoutCRC[0], steeringColumnWithoutCRC[1],
    steeringColumnWithoutCRC[2], steeringColumnWithoutCRC[3]
  };
  CAN.sendMsgBuf(0x2A7, 0, 5, steeringColumnWithCRC);

  unsigned char restraintWithoutCRC[] = {
    static_cast<uint8_t>(0x40 | counter4Bit), 0x40, 0x55, 0xFD, 0xFF, 0xFF, 0xFF
  };
  unsigned char restraintWithCRC[] = {
    crc8Calculator.get_crc8(restraintWithoutCRC, 7, 0xFF),
    restraintWithoutCRC[0], restraintWithoutCRC[1], restraintWithoutCRC[2], restraintWithoutCRC[3],
    restraintWithoutCRC[4], restraintWithoutCRC[5], restraintWithoutCRC[6]
  };
  CAN.sendMsgBuf(0x19B, 0, 8, restraintWithCRC);

  unsigned char restraint2WithoutCRC[] = {
    static_cast<uint8_t>(0xE0 | counter4Bit), 0xF1, 0xF0, 0xF2, 0xF2, 0xFE
  };
  unsigned char restraint2WithCRC[] = {
    crc8Calculator.get_crc8(restraint2WithoutCRC, 6, 0x28),
    restraint2WithoutCRC[0], restraint2WithoutCRC[1], restraint2WithoutCRC[2],
    restraint2WithoutCRC[3], restraint2WithoutCRC[4], restraint2WithoutCRC[5]
  };
  CAN.sendMsgBuf(0x297, 0, 7, restraint2WithCRC);

  // Keep the stability-control warning state cleared until the engine signal has been stable for 500 ms.
  static unsigned long rpmStableStartTime = 0;
  static bool engineStable = false;

  if (game.ignition && (game.engineRunning || game.rpm >= 400)) {
    if (rpmStableStartTime == 0) rpmStableStartTime = millis();
    if (millis() - rpmStableStartTime >= 500) engineStable = true;
  } else {
    rpmStableStartTime = 0;
    engineStable = false;
  }

  if (!engineStable) {
    uint8_t clear42[] = {0x40, 42, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF};
    CAN.sendMsgBuf(0x5C0, 0, 8, clear42);

    const uint8_t clearIds[] = {184, 215, 237, 236};
    for (uint8_t i = 0; i < sizeof(clearIds); i++) {
      uint8_t msg[] = {0x40, clearIds[i], 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF};
      CAN.sendMsgBuf(0x5C0, 0, 8, msg);
    }
    return;
  }
