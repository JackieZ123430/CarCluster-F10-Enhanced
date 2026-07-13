-- Better_CAN optimized BeamNG UDP sender
-- Project: https://github.com/JackieZ123430/Better_CAN
-- Personal, educational, research and non-commercial use only.
-- Commercial use or resale requires explicit permission.
local M = {}
local gameTime = 0
local debugTimer = 0
local accum = 0
local interval = 0.02
local signalL_latched = 0
local signalR_latched = 0
local function init()
end
local function reset()
gameTime = 0
debugTimer = 0
accum = 0
signalL_latched = 0
signalR_latched = 0
end
local targetAddress = "192.168.31.61"
local function getAddress()
return targetAddress
end
local function getPort()
return 4444
end
local function getMaxUpdateRate()
return 60
end
local function onPhysicsStep(dt)
accum = accum + dt
if accum >= interval then
accum = 0
sendData()
end
end
local function isPhysicsStepUsed()
return true
end
-- Better_CAN protocol layout. Keep field order and types unchanged.
-- https://github.com/JackieZ123430/Better_CAN - non-commercial use only.
local function getStructDefinition()
return [[
unsigned       time;
float          speedKmh;
float          rpm;
char           gearLetter;   // P R N D S M
unsigned char  gearIndex;       // 0-8 only
unsigned char  ignition;
unsigned char  engineRunning;
unsigned char  doorFL;
unsigned char  doorFR;
unsigned char  doorRL;
unsigned char  doorRR;
unsigned char  trunkOpen;
unsigned char  hoodOpen;
unsigned char  parkingBrake;
unsigned char  absAvailable;
unsigned char  absActive;
unsigned char  escAvailable;
unsigned char  escActive;
unsigned char  tcsAvailable;
unsigned char  tcsActive;
unsigned char  abs;
unsigned char  isABSBrakeActive;
unsigned char  hasABS;
unsigned char  hasESC;
unsigned char  hasTCS;
unsigned char  esc;
unsigned char  tcs;
unsigned char  isTCBrakeActive;
unsigned char  isYCBrakeActive;
unsigned char  highBeam;
unsigned char  lowBeam;
unsigned char  fog;
unsigned char  signalL;
unsigned char  signalR;
unsigned char  hazard;
unsigned char  brakelights;
unsigned char  battery;
unsigned char  oil;
unsigned char  checkengine;
unsigned char  lowfuel;
unsigned char  cruiseControlActive;
float          cruiseControlTarget;
float          fuel;
float          waterTemp;
float          oilTemp;
unsigned char  tireDefFL;
unsigned char  tireDefFR;
unsigned char  tireDefRL;
unsigned char  tireDefRR;
unsigned char throttleInput;
unsigned char brakeInput;
unsigned char engineLoad;
unsigned char  driveMode;
float          airspeedKmh;       // 空气速度
unsigned char  engineImpactDamage;
unsigned char  radiatorLeak;
unsigned char  oilpanLeak;
unsigned char  oilRadiatorLeak;
unsigned char  exhaustBroken;
unsigned char  mainEngineBroken;
unsigned char  gearboxBroken;
unsigned char  transfercaseBroken;
unsigned char  driveshaftBroken;
unsigned char  differentialRBroken;
unsigned char  spindleFLBroken;
unsigned char  spindleFRBroken;
unsigned char  spindleRLBroken;
unsigned char  spindleRRBroken;
unsigned char  wheelaxleRLBroken;
unsigned char  wheelaxleRRBroken;
unsigned char  torsionReactorRBroken;
float          brakeOverHeatFL;
float          brakeOverHeatFR;
float          brakeOverHeatRL;
float          brakeOverHeatRR;
unsigned char  engineDisabled;
unsigned char  engineLockedUp;
unsigned char  engineReducedTorque;
unsigned char  engineHydrolocked;
unsigned char  engineIsHydrolocking;
unsigned char  headGasketDamaged;
unsigned char  pistonRingsDamaged;
unsigned char  rodBearingsDamaged;
unsigned char  blockMelted;
unsigned char  cylinderWallsMelted;
unsigned char  coolantOverheating;
unsigned char  oilOverheating;
unsigned char  oilLevelCritical;
unsigned char  oilLevelTooHigh;
unsigned char  starvedOfOil;
unsigned char  overRevDanger;
unsigned char  mildOverrevDamage;
unsigned char  catastrophicOverrevDamage;
unsigned char  overTorqueDanger;
unsigned char  catastrophicOverTorqueDamage;
unsigned char headlightFL;
unsigned char headlightFR;
unsigned char turnFL;
unsigned char turnFR;
unsigned char turnRL;
unsigned char turnRR;
unsigned char taillightOuterL;
unsigned char taillightOuterR;
unsigned char taillightInnerL;
unsigned char taillightInnerR;
]]
end
local function toFlag01(v)
if v == nil then return 0 end
if type(v) == "boolean" then
return v and 1 or 0
end
if type(v) == "number" then
return (v ~= 0) and 1 or 0
end
return 0
end
local function fillStruct(o, dtSim)
if not electrics or not electrics.values then return end
local e = electrics.values
gameTime = gameTime + dtSim
debugTimer = debugTimer + dtSim
o.time = math.floor(gameTime * 1000)
o.speedKmh = (e.wheelspeed or 0) * 3.6
local rpm = e.rpm or 0
if rpm < 10 then rpm = 0 end
o.rpm = rpm
local gearStr = tostring(e.gear or "N")
local gearIndex = tonumber(e.gearIndex or 0)
o.gearIndex = gearIndex
local firstChar = gearStr:sub(1,1)
if firstChar == "P" then
o.gearLetter = string.byte("P")
elseif firstChar == "R" then
o.gearLetter = string.byte("R")
elseif firstChar == "N" then
o.gearLetter = string.byte("N")
elseif firstChar == "S" then
o.gearLetter = string.byte("S")
elseif firstChar == "M" then
o.gearLetter = string.byte("M")
else
o.gearLetter = string.byte("D")
end
o.ignition = math.floor(e.ignitionLevel or 0)
o.engineRunning = toFlag01(e.engineRunning)
local function anyElectricsOpen(...)
for i = 1, select("#", ...) do
local key = select(i, ...)
local v = e[key]
if v ~= nil and v ~= 0 then
return 1
end
end
return 0
end
o.doorFL = anyElectricsOpen(
"door_FL_coupler_notAttached",
"door_L_coupler_notAttached",
"doorFLCoupler_notAttached",
"doorLCoupler_notAttached"
)
o.doorFR = anyElectricsOpen(
"door_FR_coupler_notAttached",
"door_R_coupler_notAttached",
"doorFRCoupler_notAttached",
"doorRCoupler_notAttached"
)
o.doorRL = anyElectricsOpen(
"door_RL_coupler_notAttached",
"doorRLCoupler_notAttached"
)
o.doorRR = anyElectricsOpen(
"door_RR_coupler_notAttached",
"doorRRCoupler_notAttached"
)
o.trunkOpen = anyElectricsOpen(
"tailgateCoupler_notAttached",
"trunkCoupler_notAttached"
)
o.hoodOpen = anyElectricsOpen(
"hoodLatchCoupler_notAttached"
)
o.parkingBrake = toFlag01(e.parkingbrake)
local escVal = (e.esc and e.esc ~= 0) and 1 or 0
local tcsVal = (e.tcs and e.tcs ~= 0) and 1 or 0
local absVal = (e.abs and e.abs ~= 0) and 1 or 0
o.hasABS = 1
o.absAvailable = 1
o.absActive = absVal
o.abs = absVal
o.isABSBrakeActive = absVal
o.hasESC = 1
o.escAvailable = 1
o.escActive = escVal
o.esc = escVal
o.hasTCS = 1
o.tcsAvailable = 1
o.tcsActive = tcsVal
o.tcs = tcsVal
o.isTCBrakeActive = tcsVal
o.isYCBrakeActive = escVal
o.highBeam = toFlag01(e.highbeam)
o.lowBeam = toFlag01(e.lowbeam)
o.fog = toFlag01(e.fog)
local sigL = e.signal_left_input or 0
local sigR = e.signal_right_input or 0
if toFlag01(e.hazard) == 1 then
signalL_latched = 1
signalR_latched = 1
else
if sigL ~= 0 then
signalL_latched = 1
elseif sigL == 0 and sigR == 0 then
signalL_latched = 0
end
if sigR ~= 0 then
signalR_latched = 1
elseif sigL == 0 and sigR == 0 then
signalR_latched = 0
end
end
o.signalL = signalL_latched
o.signalR = signalR_latched
o.hazard = toFlag01(e.hazard)
o.brakelights = toFlag01(e.brakelights)
o.battery = 0
o.oil = toFlag01(e.oil)
o.checkengine = toFlag01(e.checkengine)
o.lowfuel = toFlag01(e.lowfuel)
if e.cruiseControlActive ~= nil then
o.cruiseControlActive =
(e.cruiseControlActive ~= 0) and 1 or 0
else
o.cruiseControlActive = 0
end
if e.cruiseControlTarget ~= nil then
o.cruiseControlTarget = e.cruiseControlTarget * 3.6
else
o.cruiseControlTarget = 0
end
o.fuel = (e.fuel or 0) * 100.0
o.waterTemp = e.watertemp or 0
o.oilTemp = e.oiltemp or 0
o.throttleInput = 0
o.brakeInput = 0
o.engineLoad = 0
o.airspeedKmh = 0.0
local mode = ""
local dm = controller.getController("driveModes")
if dm and dm.getCurrentDriveModeKey then
local key = dm:getCurrentDriveModeKey()
if key then
mode = string.lower(tostring(key))
end
end
if mode == "comfort" then
o.driveMode = 1
elseif mode == "sport" or mode == "ttsport" or mode == "race" then
o.driveMode = 2
elseif mode == "sport+" or mode == "race+" or mode == "ttsport+" then
o.driveMode = 3
elseif mode == "off" then
o.driveMode = 4
elseif mode == "2WD" or mode == "2wd" then
o.driveMode = 5
elseif mode == "drift" then
o.driveMode = 6
else
o.driveMode = 0
end
local wheelInfo = nil
if obj and obj.getWheelInfo then
wheelInfo = obj:getWheelInfo()
end
local function defFromElectrics(key)
local v = e[key]
if v == nil then return nil end
return (v ~= 0) and 1 or 0
end
local function defFromWheelInfo(idx)
if not wheelInfo then return nil end
local w = wheelInfo[idx] or wheelInfo[idx + 1]
if not w then return nil end
if w.isTireDeflated ~= nil then
return (w.isTireDeflated and 1 or 0)
end
if w.tireDeflated ~= nil then
return (w.tireDeflated and 1 or 0)
end
if w.damage and w.damage.isTireDeflated ~= nil then
return (w.damage.isTireDeflated and 1 or 0)
end
return nil
end
local function defFromWheelRotator(idx)
if not wheels or not wheels.wheelRotators then return nil end
local r = wheels.wheelRotators[idx]
if r and r.isTireDeflated ~= nil then
return (r.isTireDeflated and 1 or 0)
end
return nil
end
local function getDef(idx, eKey)
local v = defFromElectrics(eKey)
if v ~= nil then return v end
v = defFromWheelInfo(idx)
if v ~= nil then return v end
v = defFromWheelRotator(idx)
if v ~= nil then return v end
return 0
end
local function dmgBool(group, name)
if not damageTracker or not damageTracker.getDamage then
return 0
end
local v = damageTracker.getDamage(group, name)
if type(v) == "boolean" then
return v and 1 or 0
end
if type(v) == "number" then
return (v ~= 0) and 1 or 0
end
return 0
end
o.tireDefFL = getDef(0, "tireDeflatedFL")
o.tireDefFR = getDef(1, "tireDeflatedFR")
o.tireDefRL = getDef(2, "tireDeflatedRL")
o.tireDefRR = getDef(3, "tireDeflatedRR")
o.engineImpactDamage = dmgBool("engine", "impactDamage")
o.radiatorLeak = dmgBool("engine", "radiatorLeak")
o.oilpanLeak = dmgBool("engine", "oilpanLeak")
o.oilRadiatorLeak = dmgBool("engine", "oilRadiatorLeak")
o.exhaustBroken = dmgBool("engine", "exhaustBroken")
o.mainEngineBroken = dmgBool("powertrain", "mainEngine")
o.gearboxBroken = dmgBool("powertrain", "gearbox")
o.transfercaseBroken = 0
o.driveshaftBroken = 0
o.differentialRBroken = 0
o.spindleFLBroken = 0
o.spindleFRBroken = 0
o.spindleRLBroken = 0
o.spindleRRBroken = 0
o.wheelaxleRLBroken = 0
o.wheelaxleRRBroken = 0
o.torsionReactorRBroken = 0
o.brakeOverHeatFL = 0.0
o.brakeOverHeatFR = 0.0
o.brakeOverHeatRL = 0.0
o.brakeOverHeatRR = 0.0
o.engineDisabled = dmgBool("engine", "engineDisabled")
o.engineLockedUp = dmgBool("engine", "engineLockedUp")
o.engineReducedTorque = dmgBool("engine", "engineReducedTorque")
o.engineHydrolocked = 0
o.engineIsHydrolocking = 0
o.headGasketDamaged = 0
o.pistonRingsDamaged = 0
o.rodBearingsDamaged = 0
o.blockMelted = 0
o.cylinderWallsMelted = 0
o.coolantOverheating = dmgBool("engine", "coolantOverheating")
o.oilOverheating = dmgBool("engine", "oilOverheating")
o.oilLevelCritical = dmgBool("engine", "oilLevelCritical")
o.oilLevelTooHigh = dmgBool("engine", "oilLevelTooHigh")
o.starvedOfOil = dmgBool("engine", "starvedOfOil")
o.overRevDanger = 0
o.mildOverrevDamage = 0
o.catastrophicOverrevDamage = 0
o.overTorqueDanger = 0
o.catastrophicOverTorqueDamage = 0
o.headlightFL = 0
o.headlightFR = 0
o.turnFL = 0
o.turnFR = 0
o.turnRL = 0
o.turnRR = 0
o.taillightOuterL = 0
o.taillightOuterR = 0
o.taillightInnerL = 0
o.taillightInnerR = 0
if debugTimer >= 1 then
debugTimer = 0
if not _G.__clusterOutsideCanTpmsKeyDumped then
_G.__clusterOutsideCanTpmsKeyDumped = true
log("I", "BETTER_CAN_TPMS_KEYS",
string.format(
"tireDeflatedFL=%s tireDeflatedFR=%s tireDeflatedRL=%s tireDeflatedRR=%s",
tostring(e.tireDeflatedFL),
tostring(e.tireDeflatedFR),
tostring(e.tireDeflatedRL),
tostring(e.tireDeflatedRR)
)
)
end
log("I", "BETTER_CAN_DEBUG",
string.format(
"SPD=%.1f RPM=%.0f | TPMS FL=%d FR=%d RL=%d RR=%d | MODE=%d | CRUISE=%d/%.1f",
o.speedKmh or 0,
o.rpm or 0,
o.tireDefFL or 0,
o.tireDefFR or 0,
o.tireDefRL or 0,
o.tireDefRR or 0,
o.driveMode or 0,
o.cruiseControlActive or 0,
o.cruiseControlTarget or 0
)
)
end
end
-- Better_CAN module exports. Project attribution and non-commercial restriction retained.
-- https://github.com/JackieZ123430/Better_CAN
M.init = init
M.reset = reset
M.getAddress = getAddress
M.getPort = getPort
M.getMaxUpdateRate = getMaxUpdateRate
M.getStructDefinition = getStructDefinition
M.fillStruct = fillStruct
M.isPhysicsStepUsed = isPhysicsStepUsed
M.onPhysicsStep = onPhysicsStep
return M
