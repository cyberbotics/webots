#include "Wrapper.hpp"
#include "entry_points.hpp"

bool wbr_init(WbrInterface *ri) {
  Wrapper::init();
  ri->mandatory.wbr_start = Wrapper::start;
  ri->mandatory.wbr_stop = Wrapper::stop;
  ri->mandatory.wbr_has_failed = Wrapper::hasFailed;
  ri->mandatory.wbr_robot_step = Wrapper::robotStep;
  ri->mandatory.wbr_stop_actuators = Wrapper::stopActuators;
  ri->wbr_custom_function = Wrapper::callCustomFunction;
  ri->wbr_set_sampling_period = Wrapper::setSamplingPeriod;
  ri->wbr_led_set = Wrapper::ledSet;
  ri->wbr_motor_set_velocity = Wrapper::motorSetVelocity;
  ri->wbr_motor_set_torque_sampling_period = Wrapper::motorSetTorqueSamplingPeriod;
  return true;
}

void wbr_cleanup() {
  Wrapper::cleanup();
}
