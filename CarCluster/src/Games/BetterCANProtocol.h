// ####################################################################################################################
// Better_CAN UDP wire protocol (148 bytes)
// Author / maintainer: JackieZ123430
// Source project: https://github.com/JackieZ123430/Better_CAN
// Consumer project: https://github.com/JackieZ123430/CarCluster-F10-Enhanced
// Original upstream project: https://github.com/r00li/CarCluster
//
// 仅供个人学习、研究及非商业用途。禁止倒卖、付费分发或包装成收费产品。
// 如果你通过第三方付费获得本项目，请及时申请退款，并保留商品页面和付款记录后举报卖家。
// Personal learning, research and non-commercial use only. Preserve author and project attribution.
//
// The packet size and field offsets are kept stable for compatibility with existing Better_CAN senders.
// Reserved storage is ignored by the F10 receiver and must not be repurposed without versioning both endpoints.
// ####################################################################################################################

#ifndef BETTER_CAN_PROTOCOL_H
#define BETTER_CAN_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

struct __attribute__((packed)) BetterCANPacket {
  uint32_t time;
  float speedKmh;
  float rpm;
  char gearLetter;
  uint8_t gearIndex;

  uint8_t ignition;
  uint8_t engineRunning;

  uint8_t doorFL;
  uint8_t doorFR;
  uint8_t doorRL;
  uint8_t doorRR;
  uint8_t trunkOpen;
  uint8_t hoodOpen;

  uint8_t parkingBrake;

  uint8_t absAvailable;
  uint8_t absActive;
  uint8_t escAvailable;
  uint8_t escActive;
  uint8_t tcsAvailable;
  uint8_t tcsActive;

  uint8_t abs;
  uint8_t isABSBrakeActive;
  uint8_t hasABS;
  uint8_t hasESC;
  uint8_t hasTCS;
  uint8_t esc;
  uint8_t tcs;
  uint8_t isTCBrakeActive;
  uint8_t isYCBrakeActive;

  uint8_t highBeam;
  uint8_t lowBeam;
  uint8_t fog;
  uint8_t signalL;
  uint8_t signalR;
  uint8_t hazard;
  uint8_t brakelights;
  uint8_t battery;
  uint8_t oil;
  uint8_t checkengine;
  uint8_t lowfuel;

  uint8_t reservedControlByte;
  uint8_t reservedAlignControl[2];
  float reservedControlValue;

  float fuel;
  float waterTemp;
  float oilTemp;

  uint8_t tireDefFL;
  uint8_t tireDefFR;
  uint8_t tireDefRL;
  uint8_t tireDefRR;

  uint8_t throttleInput;
  uint8_t brakeInput;
  uint8_t engineLoad;
  uint8_t driveMode;
  float airspeedKmh;

  uint8_t engineImpactDamage;
  uint8_t radiatorLeak;
  uint8_t oilpanLeak;
  uint8_t oilRadiatorLeak;
  uint8_t exhaustBroken;

  uint8_t mainEngineBroken;
  uint8_t gearboxBroken;
  uint8_t transfercaseBroken;
  uint8_t driveshaftBroken;
  uint8_t differentialRBroken;
  uint8_t spindleFLBroken;
  uint8_t spindleFRBroken;
  uint8_t spindleRLBroken;
  uint8_t spindleRRBroken;
  uint8_t wheelaxleRLBroken;
  uint8_t wheelaxleRRBroken;
  uint8_t torsionReactorRBroken;

  uint8_t reservedAlignBrakeHeat[3];
  float brakeOverHeatFL;
  float brakeOverHeatFR;
  float brakeOverHeatRL;
  float brakeOverHeatRR;

  uint8_t engineDisabled;
  uint8_t engineLockedUp;
  uint8_t engineReducedTorque;
  uint8_t engineHydrolocked;
  uint8_t engineIsHydrolocking;
  uint8_t headGasketDamaged;
  uint8_t pistonRingsDamaged;
  uint8_t rodBearingsDamaged;
  uint8_t blockMelted;
  uint8_t cylinderWallsMelted;

  uint8_t coolantOverheating;
  uint8_t oilOverheating;
  uint8_t oilLevelCritical;
  uint8_t oilLevelTooHigh;
  uint8_t starvedOfOil;

  uint8_t overRevDanger;
  uint8_t mildOverrevDamage;
  uint8_t catastrophicOverrevDamage;
  uint8_t overTorqueDanger;
  uint8_t catastrophicOverTorqueDamage;

  uint8_t headlightFL;
  uint8_t headlightFR;
  uint8_t turnFL;
  uint8_t turnFR;
  uint8_t turnRL;
  uint8_t turnRR;
  uint8_t taillightOuterL;
  uint8_t taillightOuterR;
  uint8_t taillightInnerL;
  uint8_t taillightInnerR;

  uint8_t reservedTail[2];
};

static_assert(sizeof(BetterCANPacket) == 148, "Better_CAN packet size changed");
static_assert(offsetof(BetterCANPacket, reservedControlValue) == 52, "Better_CAN reserved-control offset changed");
static_assert(offsetof(BetterCANPacket, airspeedKmh) == 76, "Better_CAN airspeed offset changed");
static_assert(offsetof(BetterCANPacket, brakeOverHeatFL) == 100, "Better_CAN brake heat offset changed");
static_assert(offsetof(BetterCANPacket, taillightInnerR) == 145, "Better_CAN tail offset changed");

#endif
