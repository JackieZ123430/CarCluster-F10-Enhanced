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
        (uint8_t)((uint8_t)(0xF0 | counter4Bit)),
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
      unsigned char neutralWithoutCRC[] = { (uint8_t)(0xF0 | counter4Bit), 0x60, 0xFC, 0xFF };
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
    // Fuel gauge input is standardized to 0-100 percent across all retained integrations.
    sendFuel(game.fuelQuantity);
    sendParkBrake(game.handbrake);
    sendDistanceTravelled(mapSpeed(game));
    sendAlerts(game, game.offroadLight);

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
    // TPMS ECU keep‑alive
    {
      unsigned char tpmsWithoutCRC[] = { (uint8_t)(0xF0 | counter4Bit), 0xA2, 0xA0, 0xA0 };
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

  // Lighting needs faster response than language, clock and backlight settings.
  if (millis() - lastDashboardUpdateTimeLights >= dashboardUpdateTimeLights) {
    sendLights(game.mainLights, game.highBeam, game.rearFogLight, game.frontFogLight);
    sendBlinkers(game.leftTurningIndicator, game.rightTurningIndicator);
    lastDashboardUpdateTimeLights = millis();
  }

  if (millis() - lastDashboardUpdateTime1000ms >= dashboardUpdateTimeSlow) {
    sendBacklightBrightness(game.backlightBrightness);
    // Better_CAN and the Wi-Fi dashboard provide the requested drive mode directly.
    // Accept only values understood by the F10 cluster.
    uint8_t driveModeToSend = game.driveMode;
    if (driveModeToSend != 1 && driveModeToSend != 2 && driveModeToSend != 4 &&
        driveModeToSend != 5 && driveModeToSend != 6 && driveModeToSend != 7) {
      driveModeToSend = 2;
    }
    sendDriveMode(driveModeToSend);

    // Game simulation time -> cluster clock.
    sendOutsideTemperature(game.outdoorTemperature);
    const unsigned long totalSeconds = game.time / 1000UL;
    const uint8_t hours = static_cast<uint8_t>((totalSeconds / 3600UL) % 24UL);
    const uint8_t minutes = static_cast<uint8_t>((totalSeconds / 60UL) % 60UL);
    sendTime(hours, minutes);

    // (button event handling moved to immediate processing above)

    // Keep sending 0x291 at low rate after startup so the cluster can re-sync if needed.
    updateLanguageAndUnits();

    lastDashboardUpdateTime1000ms = millis();
  }
  // Serial.println(game.fuelQuantity);


}


void BMWFSeriesCluster::sendIgnitionStatus(bool ignition) {
