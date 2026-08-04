#include "Control/GpsRescue.h"
#include "Utils/Math.hpp"
#include <cmath>
#include <algorithm>

namespace Espfc::Control {

GpsRescue::GpsRescue(Model& model): _model(model) {}

int GpsRescue::begin()
{
  reset();
  return 1;
}

void GpsRescue::reset()
{
  auto& r = _model.state.gpsRescue;
  r.phase = GPS_RESCUE_PHASE_IDLE;
  r.abortReason = GPS_RESCUE_ABORT_NONE;
  r.active = false;
  r.targetClimbRate = 0;
  r.targetLeanAngle = 0;
}

int GpsRescue::update()
{
  auto& r = _model.state.gpsRescue;
  const bool wantActive = _model.isModeActive(MODE_GPS_RESCUE) && _model.isModeActive(MODE_ARMED);

  if(!wantActive)
  {
    if(r.active) reset();
    return 0;
  }

  if(!r.active)
  {
    r.active = true;
    r.phase = GPS_RESCUE_PHASE_INIT;
    r.abortReason = GPS_RESCUE_ABORT_NONE;
    r.engageTimeMs = millis();
    r.distanceToHome = _model.state.gps.distanceToHome;
    r.distanceToHomePrev = r.distanceToHome;
  }

  if(checkAborts())
  {
    r.phase = GPS_RESCUE_PHASE_ABORTED;
    logStatus();
    return 0;
  }

  // Only recompute geometry on a fresh GPS fix — GPS runs at 5-25Hz vs this
  // update's 50Hz actuator-tier schedule (see Espfc::update()). Between
  // fixes we just re-log the last computed targets rather than run stale
  // math every cycle.
  if(_model.state.gps.lastMsgTs != r.lastGpsMsgTs)
  {
    r.lastGpsMsgTs = _model.state.gps.lastMsgTs;
    r.distanceToHomePrev = r.distanceToHome;
    r.distanceToHome = _model.state.gps.distanceToHome;

    switch(r.phase)
    {
      case GPS_RESCUE_PHASE_INIT:    updateInit();    break;
      case GPS_RESCUE_PHASE_CLIMB:   updateClimb();   break;
      case GPS_RESCUE_PHASE_RETURN:  updateReturn();  break;
      case GPS_RESCUE_PHASE_DESCEND: updateDescend(); break;
      case GPS_RESCUE_PHASE_HOVER:   updateHover();   break;
      default: break;
    }
  }

  r.lastUpdateMs = millis();
  logStatus();

  return 1;
}

bool GpsRescue::checkAborts()
{
  auto& r = _model.state.gpsRescue;
  const auto& cfg = _model.config.gpsRescue;
  const auto& gps = _model.state.gps;

  if(!_model.isModeActive(MODE_ARMED))
  {
    r.abortReason = GPS_RESCUE_ABORT_DISARMED;
    return true;
  }
  if(!gps.isHomeValid())
  {
    r.abortReason = GPS_RESCUE_ABORT_HOME_INVALID;
    return true;
  }
  if(gps.numSats < cfg.minSats)
  {
    r.abortReason = GPS_RESCUE_ABORT_GPS_SATS_LOW;
    return true;
  }
  if(!isGpsDataFresh())
  {
    r.abortReason = GPS_RESCUE_ABORT_GPS_STALE;
    return true;
  }
  if(cfg.allowManualOverride && isPilotOverriding())
  {
    r.abortReason = GPS_RESCUE_ABORT_PILOT_OVERRIDE;
    return true;
  }
  if(r.phase == GPS_RESCUE_PHASE_INIT && gps.distanceToHome < cfg.minStartDistM)
  {
    r.abortReason = GPS_RESCUE_ABORT_TOO_CLOSE;
    return true;
  }
  if(cfg.sanityChecks && r.phase == GPS_RESCUE_PHASE_RETURN)
  {
    // Heuristic only: distance-to-home should trend down during RETURN.
    // Small margin to tolerate GPS noise between fixes; not tuned/validated.
    constexpr float hysteresisM = 2.0f;
    if(r.distanceToHomePrev > 0.0f && r.distanceToHome > r.distanceToHomePrev + hysteresisM)
    {
      r.abortReason = GPS_RESCUE_ABORT_SANITY_DISTANCE;
      return true;
    }
  }

  r.abortReason = GPS_RESCUE_ABORT_NONE;
  return false;
}

bool GpsRescue::isPilotOverriding() const
{
  const auto& cfg = _model.config.gpsRescue;
  const float threshold = cfg.stickOverrideDeflectionPct / 100.0f;
  for(size_t i = 0; i < AXIS_COUNT_RPY; i++)
  {
    if(std::fabs(_model.state.input.ch[i]) > threshold) return true;
  }
  return false;
}

bool GpsRescue::isGpsDataFresh() const
{
  const auto& cfg = _model.config.gpsRescue;
  const uint32_t timeoutUs = (uint32_t)cfg.staleDataTimeoutMs10 * 10000u; // x10ms -> us
  return (micros() - _model.state.gps.lastMsgTs) < timeoutUs;
}

float GpsRescue::computeBearingToHome() const
{
  // gps.directionToHome is the bearing FROM home TO the current position
  // (GpsSensor::calculateHomeVector() calls calculateDistanceAndBearing(home, current)).
  // We need the reciprocal: bearing FROM the current position TO home.
  float b = _model.state.gps.directionToHome + Utils::pi();
  if(b >= Utils::twoPi()) b -= Utils::twoPi();
  return b;
}

float GpsRescue::computeTargetLeanAngle(float distanceM, float speedErrorCms) const
{
  const auto& cfg = _model.config.gpsRescue;

  // Placeholder P-controller: NOT tuned or validated. Exists only to
  // produce a bounded, observable number for this log-only pass. Must be
  // replaced with a real velocity/position controller before this is ever
  // wired into the mixer.
  constexpr float kP = 0.02f; // rad per cm/s of speed error
  float angle = speedErrorCms * kP;

  // taper toward zero as distance-to-home shrinks inside the descent radius
  if(cfg.descentDistanceM > 0 && distanceM < cfg.descentDistanceM)
  {
    angle *= std::clamp(distanceM / (float)cfg.descentDistanceM, 0.0f, 1.0f);
  }

  const float maxAngle = Utils::toRad(cfg.maxAngleDecidegrees * 0.1f);
  return std::clamp(angle, -maxAngle, maxAngle);
}

void GpsRescue::updateInit()
{
  auto& r = _model.state.gpsRescue;
  const auto& gps = _model.state.gps;
  const auto& cfg = _model.config.gpsRescue;

  r.startLat = gps.location.raw.lat;
  r.startLon = gps.location.raw.lon;
  r.startAltitude = _model.state.altitude.height;
  r.targetAltitude = r.startAltitude + cfg.returnAltitudeM;
  r.targetHeading = _model.state.attitude.euler[AXIS_YAW];
  r.headingError = 0;
  r.targetLeanAngle = 0;
  r.targetClimbRate = 0;

  r.phase = GPS_RESCUE_PHASE_CLIMB;
}

void GpsRescue::updateClimb()
{
  auto& r = _model.state.gpsRescue;
  const auto& cfg = _model.config.gpsRescue;

  // climb in place first, matching the user's requested climb-before-return
  // sequencing (a deliberate simplification vs. real Betaflight's blended
  // climb+return default).
  r.targetLeanAngle = 0;
  r.targetHeading = _model.state.attitude.euler[AXIS_YAW];
  r.headingError = 0;

  const float remaining = r.targetAltitude - _model.state.altitude.height;
  if(remaining > 0.5f)
  {
    r.targetClimbRate = cfg.climbRateCms / 100.0f;
  }
  else
  {
    r.targetClimbRate = 0;
    r.phase = GPS_RESCUE_PHASE_RETURN;
  }
}

void GpsRescue::updateReturn()
{
  auto& r = _model.state.gpsRescue;
  const auto& cfg = _model.config.gpsRescue;

  r.targetHeading = computeBearingToHome();
  float error = r.targetHeading - _model.state.attitude.euler[AXIS_YAW];
  while(error > Utils::pi()) error -= Utils::twoPi();
  while(error < -Utils::pi()) error += Utils::twoPi();
  r.headingError = error;

  // gentle P-hold on return altitude
  const float altError = r.targetAltitude - _model.state.altitude.height;
  r.targetClimbRate = std::clamp(altError * 0.5f, -1.0f, 1.0f);

  const float groundSpeedCms = _model.state.gps.velocity.raw.groundSpeed / 10.0f;
  const float speedErrorCms = cfg.returnSpeedCms - groundSpeedCms;
  r.targetLeanAngle = computeTargetLeanAngle(r.distanceToHome, speedErrorCms);

  if(r.distanceToHome <= cfg.descentDistanceM)
  {
    r.phase = GPS_RESCUE_PHASE_DESCEND;
  }
}

void GpsRescue::updateDescend()
{
  auto& r = _model.state.gpsRescue;
  const auto& cfg = _model.config.gpsRescue;

  r.targetHeading = computeBearingToHome();
  float error = r.targetHeading - _model.state.attitude.euler[AXIS_YAW];
  while(error > Utils::pi()) error -= Utils::twoPi();
  while(error < -Utils::pi()) error += Utils::twoPi();
  r.headingError = error;

  r.targetClimbRate = -cfg.descendRateCms / 100.0f;

  // tapered, slower final approach
  const float groundSpeedCms = _model.state.gps.velocity.raw.groundSpeed / 10.0f;
  const float speedErrorCms = (cfg.returnSpeedCms * 0.5f) - groundSpeedCms;
  r.targetLeanAngle = computeTargetLeanAngle(r.distanceToHome, speedErrorCms) * 0.5f;

  if(_model.state.altitude.height - r.startAltitude <= cfg.landingAltitudeM)
  {
    r.phase = GPS_RESCUE_PHASE_HOVER;
  }
}

void GpsRescue::updateHover()
{
  auto& r = _model.state.gpsRescue;
  r.targetClimbRate = 0;
  r.targetLeanAngle = 0;
  // heading held from previous phase. Terminal for this pass — real
  // land/disarm behavior is explicitly out of scope until the future
  // flight-enabled pass.
}

void GpsRescue::logStatus() const
{
  const auto& r = _model.state.gpsRescue;
  _model.setDebug(DEBUG_GPS_RESCUE, 0, (int16_t)r.phase);
  _model.setDebug(DEBUG_GPS_RESCUE, 1, (int16_t)lrintf(Utils::toDeg(r.targetHeading) * 10.0f));
  _model.setDebug(DEBUG_GPS_RESCUE, 2, (int16_t)lrintf(Utils::toDeg(r.targetLeanAngle) * 10.0f));
  _model.setDebug(DEBUG_GPS_RESCUE, 3, (int16_t)lrintf(r.targetClimbRate * 100.0f));
  _model.setDebug(DEBUG_GPS_RESCUE, 4, (int16_t)lrintf(r.distanceToHome));
  _model.setDebug(DEBUG_GPS_RESCUE, 5, (int16_t)lrintf(r.targetAltitude * 100.0f));
  _model.setDebug(DEBUG_GPS_RESCUE, 6, (int16_t)r.abortReason);
  _model.setDebug(DEBUG_GPS_RESCUE, 7, (int16_t)lrintf(Utils::toDeg(r.headingError) * 10.0f));
}

}
