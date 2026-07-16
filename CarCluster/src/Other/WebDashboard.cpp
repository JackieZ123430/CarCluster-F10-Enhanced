// ####################################################################################################################
// CarCluster-F10-Enhanced web dashboard bridge
// Author / maintainer: JackieZ123430
// Project: https://github.com/JackieZ123430/CarCluster-F10-Enhanced
// Better_CAN: https://github.com/JackieZ123430/Better_CAN
// Original upstream project: https://github.com/r00li/CarCluster
//
// 仅供个人学习、研究及非商业用途。禁止倒卖、付费分发或包装成收费产品。
// 如果你通过第三方付费获得本项目，请及时申请退款，并保留商品页面和付款记录后举报卖家。
// Personal learning, research and non-commercial use only. Unauthorized resale or paid redistribution is prohibited.
// Keep the author name and project links in permitted copies and forks.
// ####################################################################################################################

#include "WebDashboard.h"

WebDashboard::WebDashboard(GameState &game, unsigned long webDashboardUpdateInterval): gameState(game) {
  this->webDashboardUpdateInterval = webDashboardUpdateInterval;
}

void WebDashboard::getState(struct state *data) {
  data->speed = gameState.speed;
  data->maximumSpeed = gameState.configuration.maximumSpeedValue;
  data->rpm = gameState.rpm;
  data->maximumRPM = gameState.configuration.maximumRPMValue;
  data->fuel = gameState.fuelQuantity;
  data->high_beam = gameState.highBeam;
  data->fog_rear = gameState.rearFogLight;
  data->fog_front = gameState.frontFogLight;
  data->left_indicator = gameState.leftTurningIndicator;
  data->right_indicator = gameState.rightTurningIndicator;
  data->main_lights = gameState.mainLights;
  data->door_open = gameState.doorOpen;
  data->dsc = gameState.offroadLight;
  data->abs = gameState.absLight;
  strcpy(data->gear, mapGenericGearToLocalGear(gameState.gear));
  data->backlight = gameState.backlightBrightness;
  data->coolant_temp = gameState.coolantTemperature;
  data->minimumCoolantTemp = gameState.configuration.minimumCoolantTemperature;
  data->maximumCoolantTemp = gameState.configuration.maximumCoolantTemperature;
  data->handbrake = gameState.handbrake;
  data->ignition = gameState.ignition;
  strcpy(data->drive_mode, mapGenericDriveModeToLocalDriveMode(gameState.driveMode));
  data->outdoor_temp = gameState.outdoorTemperature;
  data->indicators_blink = gameState.turningIndicatorsBlinking;
}

void WebDashboard::setState(struct state *data) {
  gameState.speed = data->speed;
  gameState.rpm = data->rpm;
  gameState.fuelQuantity = data->fuel;
  gameState.highBeam = data->high_beam;
  gameState.rearFogLight = data->fog_rear;
  gameState.frontFogLight = data->fog_front;
  gameState.leftTurningIndicator = data->left_indicator;
  gameState.rightTurningIndicator = data->right_indicator;
  gameState.mainLights = data->main_lights;
  gameState.doorOpen = data->door_open;
  gameState.doorFL = data->door_open;
  gameState.doorFR = false;
  gameState.doorRL = false;
  gameState.doorRR = false;
  gameState.offroadLight = data->dsc;
  gameState.absLight = data->abs;
  gameState.gear = mapLocalGearToGenericGear(data->gear);
  gameState.backlightBrightness = data->backlight;
  gameState.coolantTemperature = data->coolant_temp;
  gameState.oilTemperature = data->coolant_temp;
  gameState.handbrake = data->handbrake;
  gameState.ignition = data->ignition;
  gameState.driveMode = mapLocalDriveModeToGenericDriveMode(data->drive_mode);
  gameState.outdoorTemperature = data->outdoor_temp;
  gameState.turningIndicatorsBlinking = data->indicators_blink;
}

void WebDashboard::steeringWheelAction(struct mg_str params) {
  if (params.len < 1) return;

  const int requestedAction = params.buf[0] - '0';
  // The dashboard exposes only the BC/menu action to the cluster output layer.
  gameState.buttonEventToProcess = requestedAction == 1 ? 1 : 0;
}

void WebDashboard::alertStart(struct mg_str params) {
  (void)params;
  gameState.alertStart = true;
}

void WebDashboard::alertClear(struct mg_str params) {
  (void)params;
  gameState.alertClear = true;
}

const char* WebDashboard::mapGenericGearToLocalGear(GearState inputGear) {
  switch(inputGear) {
    case GearState_Manual_1: return "1";
    case GearState_Manual_2: return "2";
    case GearState_Manual_3: return "3";
    case GearState_Manual_4: return "4";
    case GearState_Manual_5: return "5";
    case GearState_Manual_6: return "6";
    case GearState_Manual_7: return "7";
    case GearState_Manual_8: return "8";
    case GearState_Manual_9: return "9";
    case GearState_Manual_10: return "10";
    case GearState_Auto_P: return "P";
    case GearState_Auto_R: return "R";
    case GearState_Auto_N: return "N";
    case GearState_Auto_D: return "D";
    case GearState_Auto_S: return "S";
    default: return "P";
  }
}

GearState WebDashboard::mapLocalGearToGenericGear(const char *gear) {
  if (!gear || !gear[0]) return GearState_Auto_P;
  if (strcmp(gear, "10") == 0) return GearState_Manual_10;

  switch (gear[0]) {
    case '1': return GearState_Manual_1;
    case '2': return GearState_Manual_2;
    case '3': return GearState_Manual_3;
    case '4': return GearState_Manual_4;
    case '5': return GearState_Manual_5;
    case '6': return GearState_Manual_6;
    case '7': return GearState_Manual_7;
    case '8': return GearState_Manual_8;
    case '9': return GearState_Manual_9;
    case 'P': return GearState_Auto_P;
    case 'R': return GearState_Auto_R;
    case 'N': return GearState_Auto_N;
    case 'D': return GearState_Auto_D;
    case 'S': return GearState_Auto_S;
    default: return GearState_Auto_P;
  }
}

const char* WebDashboard::mapGenericDriveModeToLocalDriveMode(uint8_t driveMode) {
  switch(driveMode) {
    case 1: return "Traction";
    case 2: return "Comfort";
    case 4: return "Sport";
    case 5: return "Sport+";
    case 6: return "DSC off";
    case 7: return "Eco pro";
    default: return "Comfort";
  }
}

uint8_t WebDashboard::mapLocalDriveModeToGenericDriveMode(const char *driveMode) {
  if (!driveMode || !driveMode[0]) return 2;
  if (strcmp(driveMode, "Traction") == 0) return 1;
  if (strcmp(driveMode, "Comfort") == 0) return 2;
  if (strcmp(driveMode, "Sport") == 0) return 4;
  if (strcmp(driveMode, "Sport+") == 0) return 5;
  if (strcmp(driveMode, "DSC off") == 0) return 6;
  if (strcmp(driveMode, "Eco pro") == 0) return 7;
  return 2;
}

void WebDashboard::update() {
  if (millis() - lastWebDashboardUpdateTime >= webDashboardUpdateInterval) {
    glue_update_state();
    lastWebDashboardUpdateTime = millis();
  }
}
