#pragma once

#include "Model.h"

namespace Espfc::Control {

// GPS Rescue foundation: computes what a return-to-home guidance state
// machine WOULD do (phase, target heading/lean/climb-rate), and only logs
// it via debug[]/CLI. It does not write into setpoint.rate/setpoint.angle
// and is never consumed by Controller.cpp or the mixer. Wiring this into
// real flight control is an explicitly separate, future pass.
class GpsRescue
{
  public:
    GpsRescue(Model& model);

    int begin();
    int update();

  #ifndef UNIT_TEST
  private:
  #endif

    void reset();
    void updateInit();
    void updateClimb();
    void updateReturn();
    void updateDescend();
    void updateHover();

    bool checkAborts();
    bool isPilotOverriding() const;
    bool isGpsDataFresh() const;

    float computeBearingToHome() const;
    float computeTargetLeanAngle(float distanceM, float speedErrorCms) const;

    void logStatus() const;

    Model& _model;
};

}
