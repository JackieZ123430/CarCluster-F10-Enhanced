// ####################################################################################################################
// BMW F10 cluster implementation
// Author / maintainer: JackieZ123430
// Project: https://github.com/JackieZ123430/CarCluster-F10-Enhanced
// Better_CAN: https://github.com/JackieZ123430/Better_CAN
// Original upstream project: https://github.com/r00li/CarCluster
//
// 仅供个人学习、研究及非商业用途。禁止倒卖、付费分发或包装成收费产品。
// 如果你通过第三方付费获得本项目，请及时申请退款，并保留商品页面和付款记录后举报卖家。
// Personal learning, research and non-commercial use only. Unauthorized resale or paid redistribution is prohibited.
// Preserve author and project attribution in permitted copies and forks.
// ####################################################################################################################

#ifndef BMW_F10_CLUSTER_H
#define BMW_F10_CLUSTER_H

#include "../../Libs/MultiMap/MultiMap.h"
#include "../../Libs/MCP_CAN/mcp_can.h"
#include "CRC8.h"
#include "../Cluster.h"

#define lo8(x) (uint8_t)((x) & 0xFF)
#define hi8(x) (uint8_t)(((x) >> 8) & 0xFF)

class BMWFSeriesCluster : public Cluster {
 public:
  static ClusterConfiguration clusterConfig() {
    ClusterConfiguration config;
    config.minimumCoolantTemperature = 50;
    config.maximumCoolantTemperature = 150;
    config.maximumRPMValue = 7500;
    config.maximumSpeedValue = 260;
    return config;
  }

  explicit BMWFSeriesCluster(MCP_CAN& CAN);
  void updateWithGame(GameState& game) override;
  void updateLanguageAndUnits();

 private:
  MCP_CAN& CAN;
  CRC8 crc8Calculator;

  unsigned long dashboardUpdateTimeFast = 20;
  unsigned long dashboardUpdateTimeLights = 100;
  unsigned long dashboardUpdateTimeSlow = 1000;
  unsigned long lastDashboardUpdateTime = 0;
  unsigned long lastDashboardUpdateTimeLights = 0;
  unsigned long lastDashboardUpdateTime1000ms = 0;

  uint8_t counter4Bit = 0;
  uint8_t count = 0;
  uint16_t distanceTravelledCounter = 0;

  uint8_t inFuelRange[3] = {0, 50, 100};
  uint8_t outFuelRange[3] = {37, 18, 4};

  void sendIgnitionStatus(bool ignition);
  void sendSpeed(int speed);
  void sendRPM(int rpm, int manualGear);
  void sendAutomaticTransmission(GearState gear, uint8_t gearIndex);
  void sendBasicDriveInfo(GameState& game, int oilTemperature);
  void sendParkBrake(bool handbrakeActive);
  void sendFuel(float fuelPercent);
  void sendDistanceTravelled(int speed);
  void sendBlinkers(bool leftTurningIndicator, bool rightTurningIndicator);
  void sendLights(bool mainLights, bool highBeam, bool rearFogLight, bool frontFogLight);
  void sendBacklightBrightness(uint8_t brightness);
  void sendAlerts(GameState& game, bool stabilityIntervention);
  void sendSteeringWheelButton(int buttonEvent);
  void sendDriveMode(uint8_t driveMode);
  void sendOutsideTemperature(int temperature);
  void sendTime(uint8_t hours, uint8_t minutes);

  uint8_t mapGenericGearToLocalGear(GearState inputGear);
  int mapSpeed(GameState& game);
  int mapRPM(GameState& game);
  int mapCoolantTemperature(GameState& game);
  float mapRPMValueForFuelModel(int speed);
};

#endif
