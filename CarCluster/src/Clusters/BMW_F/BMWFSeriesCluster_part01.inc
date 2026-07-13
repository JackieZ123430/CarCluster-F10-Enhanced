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

    // static bool lastCruiseActive = false;
    // static int lastCruiseSpeed = 0;

    // if (game.cruiseControlActive) {

    //   uint8_t cruiseOn[] = { 0x40, 0x2F, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    //   CAN.sendMsgBuf(0x5C0, 0, 8, cruiseOn);

    // } else {

    //   uint8_t cruiseOff[] = { 0x40, 0x2F, 0x00, 0x28, 0xFF, 0xFF, 0xFF, 0xFF };
    //   CAN.sendMsgBuf(0x5C0, 0, 8, cruiseOff);

    //   // Cruise cancelled while vehicle slowing → trigger message 59
    //   if (lastCruiseActive && game.speed < lastCruiseSpeed) {
    //     uint8_t msg59[] = { 0x40, 59, 0x00, 0x29, 0xFF, 0xFF, 0xFF, 0xFF };
    //     CAN.sendMsgBuf(0x5C0, 0, 8, msg59);
    //   }
    // }

    // lastCruiseActive = game.cruiseControlActive;
    // lastCruiseSpeed = game.speed;

 

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
          (uint8_t)(engineStoppedNow ? 0x29 : 0x28), 
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
        (uint8_t)((uint8_t)(0xF0 | counter4Bit)),
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
        (uint8_t)((uint8_t)(0xF0 | counter4Bit)),
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
        (uint8_t)((uint8_t)(0xF0 | counter4Bit)),
        0x10,
        0x10,
