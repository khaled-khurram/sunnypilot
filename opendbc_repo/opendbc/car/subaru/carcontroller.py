import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus, make_tester_present_msg
from opendbc.car.lateral import apply_driver_steer_torque_limits, common_fault_avoidance
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.subaru import subarucan
from opendbc.car.subaru.values import DBC, GLOBAL_ES_ADDR, CanBus, CarControllerParams, SubaruFlags

from opendbc.sunnypilot.car.subaru.stop_and_go import SnGCarController

# --- Phase 3 read-and-gate, self-contained (2026-07-24) ---
# Deliberately NOT imported from openpilot.sunnypilot.selfdrive.controls.lib.phase3_shared:
# opendbc is a lower-level car-interface library with zero existing precedent anywhere in
# this tree (checked the whole opendbc/ tree, including sunnypilot's own extension
# namespace) for importing from openpilot's planning stack - that would be a new, one-off
# layering violation in the single most safety-critical file in the codebase. Duplicated
# instead. COMMAND_FILE and _PHASE3_STALENESS_S MUST match phase3_shared.py's
# COMMAND_FILE/COMMAND_STALENESS_S exactly - if one changes, change both.
import time as _time

_PHASE3_COMMAND_FILE = "/data/phase3_button_command"
_PHASE3_STALENESS_S = 0.3

# Observability-only (2026-07-24): EyeSight's own real, unmodified Car_Follow/
# Close_Distance from the incoming ES_Distance message - the same message this file
# already re-transmits every 5 frames with only the button field overridden. Written
# here (not read here) so phase3_shared.py's shadow log can record whether EyeSight
# already had its own lead lock at the moment any Phase 3 feature acted - answers "did
# Phase 3 duplicate a job EyeSight was already doing" with real data instead of a guess.
# No gating/behavior effect - nothing in this file reads this back.
_PHASE3_EYESIGHT_STATE_FILE = "/data/phase3_eyesight_state.txt"


def _phase3_read_command_if_safe(gas_pressed: bool, brake_pressed: bool, steering_pressed: bool,
                                  real_button_pressed: bool) -> int | None:
  """Independent final gate before any real cruise_button override - re-checks the
  override guarantee here too, not just trusting plannerd's upstream decision
  (research/phase3_controller_design.md §3: "the override check must be the single
  first gate wrapping the entire 'maybe send a command' block"). Returns None (fall
  back to plain relay of CS.cruise_button, today's exact shipped behavior) unless the
  file exists, is fresh, and nothing overrides."""
  if gas_pressed or brake_pressed or steering_pressed or real_button_pressed:
    return None
  try:
    with open(_PHASE3_COMMAND_FILE) as f:
      raw = f.read().strip().split()
    value, ts = int(raw[0]), float(raw[1])
  except (FileNotFoundError, ValueError, IndexError, OSError):
    return None
  if _time.time() - ts > _PHASE3_STALENESS_S:
    return None
  return value

# FIXME: These limits aren't exact. The real limit is more than likely over a larger time period and
# involves the total steering angle change rather than rate, but these limits work well for now
MAX_STEER_RATE = 25  # deg/s
MAX_STEER_RATE_FRAMES = 7  # tx control frames needed before torque can be cut


class CarController(CarControllerBase, SnGCarController):
  def __init__(self, dbc_names, CP, CP_SP):
    CarControllerBase.__init__(self, dbc_names, CP, CP_SP)
    SnGCarController.__init__(self, CP, CP_SP)
    self.apply_torque_last = 0

    self.cruise_button_prev = 0
    self.steer_rate_counter = 0

    self.p = CarControllerParams(CP)
    self.packer = CANPacker(DBC[CP.carFingerprint][Bus.pt])

  def update(self, CC, CC_SP, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl
    pcm_cancel_cmd = CC.cruiseControl.cancel

    can_sends = []

    # *** steering ***
    if (self.frame % self.p.STEER_STEP) == 0:
      apply_torque = int(round(actuators.torque * self.p.STEER_MAX))

      # limits due to driver torque

      new_torque = int(round(apply_torque))
      apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorque, self.p)

      if not CC.latActive:
        apply_torque = 0

      if self.CP.flags & SubaruFlags.PREGLOBAL:
        can_sends.append(subarucan.create_preglobal_steering_control(self.packer, self.frame // self.p.STEER_STEP, apply_torque, CC.latActive))
      else:
        apply_steer_req = CC.latActive

        if self.CP.flags & SubaruFlags.STEER_RATE_LIMITED:
          # Steering rate fault prevention
          self.steer_rate_counter, apply_steer_req = \
            common_fault_avoidance(abs(CS.out.steeringRateDeg) > MAX_STEER_RATE, apply_steer_req,
                                   self.steer_rate_counter, MAX_STEER_RATE_FRAMES)

        can_sends.append(subarucan.create_steering_control(self.packer, apply_torque, apply_steer_req))

      self.apply_torque_last = apply_torque

    # *** longitudinal ***

    if CC.longActive:
      apply_throttle = int(round(np.interp(actuators.accel, CarControllerParams.THROTTLE_LOOKUP_BP, CarControllerParams.THROTTLE_LOOKUP_V)))
      apply_rpm = int(round(np.interp(actuators.accel, CarControllerParams.RPM_LOOKUP_BP, CarControllerParams.RPM_LOOKUP_V)))
      apply_brake = int(round(np.interp(actuators.accel, CarControllerParams.BRAKE_LOOKUP_BP, CarControllerParams.BRAKE_LOOKUP_V)))

      # limit min and max values
      cruise_throttle = np.clip(apply_throttle, CarControllerParams.THROTTLE_MIN, CarControllerParams.THROTTLE_MAX)
      cruise_rpm = np.clip(apply_rpm, CarControllerParams.RPM_MIN, CarControllerParams.RPM_MAX)
      cruise_brake = np.clip(apply_brake, CarControllerParams.BRAKE_MIN, CarControllerParams.BRAKE_MAX)
    else:
      cruise_throttle = CarControllerParams.THROTTLE_INACTIVE
      cruise_rpm = CarControllerParams.RPM_MIN
      cruise_brake = CarControllerParams.BRAKE_MIN

    # *** alerts and pcm cancel ***
    if self.CP.flags & SubaruFlags.PREGLOBAL:
      if self.frame % 5 == 0:
        # 1 = main, 2 = set shallow, 3 = set deep, 4 = resume shallow, 5 = resume deep
        # disengage ACC when OP is disengaged
        if pcm_cancel_cmd:
          cruise_button = 1
        # turn main on if off and past start-up state
        elif not CS.out.cruiseState.available and CS.ready:
          cruise_button = 1
        else:
          phase3_button = _phase3_read_command_if_safe(CS.out.gasPressed, CS.out.brakePressed,
                                                         CS.out.steeringPressed, CS.cruise_button != 0)
          cruise_button = phase3_button if phase3_button is not None else CS.cruise_button

        # unstick previous mocked button press
        if cruise_button == 1 and self.cruise_button_prev == 1:
          cruise_button = 0
        self.cruise_button_prev = cruise_button

        can_sends.append(subarucan.create_preglobal_es_distance(self.packer, cruise_button, CS.es_distance_msg))

        try:
          with open(_PHASE3_EYESIGHT_STATE_FILE, "w") as f:
            f.write(f"{int(CS.es_distance_msg.get('Car_Follow', 0))} "
                    f"{CS.es_distance_msg.get('Close_Distance', 0.0)} {_time.time()}\n")
        except OSError:
          pass

    else:
      if self.frame % 10 == 0:
        can_sends.append(subarucan.create_es_dashstatus(self.packer, self.frame // 10, CS.es_dashstatus_msg, CC.enabled,
                                                        self.CP.openpilotLongitudinalControl, CC.longActive, hud_control.leadVisible))

        can_sends.append(subarucan.create_es_lkas_state(self.packer, self.frame // 10, CS.es_lkas_state_msg, CC.enabled, hud_control.visualAlert,
                                                        hud_control.leftLaneVisible, hud_control.rightLaneVisible,
                                                        hud_control.leftLaneDepart, hud_control.rightLaneDepart))

        if self.CP.flags & SubaruFlags.SEND_INFOTAINMENT:
          can_sends.append(subarucan.create_es_infotainment(self.packer, self.frame // 10, CS.es_infotainment_msg, hud_control.visualAlert))

      if self.CP.openpilotLongitudinalControl:
        if self.frame % 5 == 0:
          can_sends.append(subarucan.create_es_status(self.packer, self.frame // 5, CS.es_status_msg,
                                                      self.CP.openpilotLongitudinalControl, CC.longActive, cruise_rpm))

          can_sends.append(subarucan.create_es_brake(self.packer, self.frame // 5, CS.es_brake_msg,
                                                     self.CP.openpilotLongitudinalControl, CC.longActive, cruise_brake))

          can_sends.append(subarucan.create_es_distance(self.packer, self.frame // 5, CS.es_distance_msg, 0, pcm_cancel_cmd,
                                                        self.CP.openpilotLongitudinalControl, cruise_brake > 0, cruise_throttle))
      else:
        if pcm_cancel_cmd:
          if not (self.CP.flags & SubaruFlags.HYBRID):
            bus = CanBus.alt if self.CP.flags & SubaruFlags.GLOBAL_GEN2 else CanBus.main
            can_sends.append(subarucan.create_es_distance(self.packer, CS.es_distance_msg["COUNTER"] + 1, CS.es_distance_msg, bus, pcm_cancel_cmd))

      if self.CP.flags & SubaruFlags.DISABLE_EYESIGHT:
        # Tester present (keeps eyesight disabled)
        if self.frame % 100 == 0:
          can_sends.append(make_tester_present_msg(GLOBAL_ES_ADDR, CanBus.camera, suppress_response=True))

        # Create all of the other eyesight messages to keep the rest of the car happy when eyesight is disabled
        if self.frame % 5 == 0:
          can_sends.append(subarucan.create_es_highbeamassist(self.packer))

        if self.frame % 10 == 0:
          can_sends.append(subarucan.create_es_static_1(self.packer))

        if self.frame % 2 == 0:
          can_sends.append(subarucan.create_es_static_2(self.packer))

    can_sends.extend(SnGCarController.create_stop_and_go(self, self.packer, CC, CS, self.frame))

    new_actuators = actuators.as_builder()
    new_actuators.torque = self.apply_torque_last / self.p.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last

    self.frame += 1
    return new_actuators, can_sends
