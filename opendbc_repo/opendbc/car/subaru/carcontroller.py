import os
import time
import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus, make_tester_present_msg
from opendbc.car.lateral import apply_driver_steer_torque_limits, common_fault_avoidance
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.subaru import subarucan
from opendbc.car.subaru.values import DBC, GLOBAL_ES_ADDR, CanBus, CarControllerParams, SubaruFlags

from opendbc.sunnypilot.car.subaru.stop_and_go import SnGCarController

# FIXME: These limits aren't exact. The real limit is more than likely over a larger time period and
# involves the total steering angle change rather than rate, but these limits work well for now
MAX_STEER_RATE = 25  # deg/s
MAX_STEER_RATE_FRAMES = 7  # tx control frames needed before torque can be cut

# DRAFT v3 (not enabled by default): three independently-armed one-shot bench tests for
# the ES_Distance Cruise_Button question, follow-ups to v2's confirmed SET-shallow-while-
# not-engaged success (progress.md Q6). See research/es_distance_live_test_protocol_v3.md.
# Each test is armed via its own flag file (deliberately not a new Params key — same
# UnknownKeyName crash-class reasoning as v2). Only ONE should ever be armed per drive —
# if more than one flag is present simultaneously, all three are treated as unarmed and a
# warning is logged, rather than guessing which one to honor.
ES_DISTANCE_TEST_LOG = "/data/es_distance_button_test_v3.log"
ES_DISTANCE_TEST_ARM_MAX_AGE_S = 30 * 60  # auto-expire a forgotten arm

ES_DISTANCE_RESUME_FLAG = "/data/es_distance_test_resume"
ES_DISTANCE_RESUME_MIN_VEGO = 5.0  # m/s (~11mph) — still clearly driving after a disengage

ES_DISTANCE_DEEP_FLAG = "/data/es_distance_test_deep"
ES_DISTANCE_DEEP_STEADY_TOLERANCE = 1.0  # m/s, |vEgo - cruiseState.speed| under this = steady-state

ES_DISTANCE_BURST_FLAG = "/data/es_distance_test_burst"
ES_DISTANCE_BURST_COUNT = 3
ES_DISTANCE_BURST_SPACING_CYCLES = 5  # 5 * 50ms (this block's cadence) = ~250ms between presses

ES_DISTANCE_SUSTAIN_FRAMES = 40  # ~2s at this block's 20Hz cadence, before any test may fire
ES_DISTANCE_DEEP_SUSTAIN_FRAMES = 60  # ~3s — deep test wants extra confidence of true steady-state


class CarController(CarControllerBase, SnGCarController):
  def __init__(self, dbc_names, CP, CP_SP):
    CarControllerBase.__init__(self, dbc_names, CP, CP_SP)
    SnGCarController.__init__(self, CP, CP_SP)
    self.apply_torque_last = 0

    self.cruise_button_prev = 0
    self.steer_rate_counter = 0

    self.es_distance_resume_fired = False
    self.es_distance_resume_was_engaged = False
    self.es_distance_resume_sustain = 0

    self.es_distance_deep_fired = False
    self.es_distance_deep_sustain = 0

    self.es_distance_burst_fired = False
    self.es_distance_burst_sustain = 0
    self.es_distance_burst_sent = 0
    self.es_distance_burst_next_frame = 0

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
          cruise_button = CS.cruise_button

        cruise_button = self._es_distance_v3_test_hook(cruise_button, CS)

        # unstick previous mocked button press
        if cruise_button == 1 and self.cruise_button_prev == 1:
          cruise_button = 0
        self.cruise_button_prev = cruise_button

        can_sends.append(subarucan.create_preglobal_es_distance(self.packer, cruise_button, CS.es_distance_msg))

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

  def _es_distance_v3_test_hook(self, cruise_button, CS):
    """DRAFT v3, not enabled by default: see research/es_distance_live_test_protocol_v3.md.
    Three independently-armed one-shot bench tests (RESUME / deep-SET / burst). Only one
    flag should ever be armed per drive; if more than one is present, all are disabled."""
    resume_armed = os.path.exists(ES_DISTANCE_RESUME_FLAG)
    deep_armed = os.path.exists(ES_DISTANCE_DEEP_FLAG)
    burst_armed = os.path.exists(ES_DISTANCE_BURST_FLAG)

    if sum([resume_armed, deep_armed, burst_armed]) > 1:
      if not getattr(self, "_es_distance_v3_multi_arm_warned", False):
        self._es_distance_v3_multi_arm_warned = True
        with open(ES_DISTANCE_TEST_LOG, "a") as f:
          f.write(f"{time.time()} WARNING multiple v3 test flags armed simultaneously - all disabled\n")
      return cruise_button

    def _fresh(flag_path):
      try:
        return (time.time() - os.path.getmtime(flag_path)) < ES_DISTANCE_TEST_ARM_MAX_AGE_S
      except OSError:
        return False

    no_real_press = CS.cruise_button == 0

    # --- RESUME test: requires a REAL engagement to have happened earlier this arm window,
    # so there's an actual stored set-speed to resume to - deliberately not assumed by
    # analogy to v2's SET result. Real Subaru RESUME recalls a previously stored target;
    # SET captures the current speed. Different code path, different precondition. ---
    if resume_armed and _fresh(ES_DISTANCE_RESUME_FLAG) and not self.es_distance_resume_fired:
      if CS.out.cruiseState.enabled:
        self.es_distance_resume_was_engaged = True  # latch: only a real engagement counts

      ready = (self.es_distance_resume_was_engaged
               and bool(CS.out.cruiseState.available) and not CS.out.cruiseState.enabled
               and CS.out.vEgo > ES_DISTANCE_RESUME_MIN_VEGO and no_real_press)
      self.es_distance_resume_sustain = self.es_distance_resume_sustain + 1 if ready else 0

      if self.es_distance_resume_sustain >= ES_DISTANCE_SUSTAIN_FRAMES:
        real_cruise_button = cruise_button
        cruise_button = 4  # RESUME shallow
        self.es_distance_resume_fired = True
        try:
          os.remove(ES_DISTANCE_RESUME_FLAG)
        except OSError:
          pass
        with open(ES_DISTANCE_TEST_LOG, "a") as f:
          f.write(f"{time.time()} FIRED TEST=resume cruise_button=4 (real={real_cruise_button}) "
                  f"frame={self.frame} vEgo={CS.out.vEgo} cruiseSpeed={CS.out.cruiseState.speed}\n")
      return cruise_button

    # --- Deep-SET test: requires cruise ALREADY engaged and steady, so what's measured is
    # the decrement applied to an existing set-speed - a different behavior than v2 tested
    # (which was "engage at current speed", not "adjust an existing target"). ---
    if deep_armed and _fresh(ES_DISTANCE_DEEP_FLAG) and not self.es_distance_deep_fired:
      steady = abs(CS.out.vEgo - CS.out.cruiseState.speed) < ES_DISTANCE_DEEP_STEADY_TOLERANCE
      ready = CS.out.cruiseState.enabled and steady and no_real_press
      self.es_distance_deep_sustain = self.es_distance_deep_sustain + 1 if ready else 0

      if self.es_distance_deep_sustain >= ES_DISTANCE_DEEP_SUSTAIN_FRAMES:
        real_cruise_button = cruise_button
        cruise_button = 3  # SET deep - decrease direction, safer than a deep resume/increase
        self.es_distance_deep_fired = True
        try:
          os.remove(ES_DISTANCE_DEEP_FLAG)
        except OSError:
          pass
        with open(ES_DISTANCE_TEST_LOG, "a") as f:
          f.write(f"{time.time()} FIRED TEST=deep cruise_button=3 (real={real_cruise_button}) "
                  f"frame={self.frame} vEgo={CS.out.vEgo} cruiseSpeed={CS.out.cruiseState.speed}\n")
      return cruise_button

    # --- Burst test: up to 3 commanded shallow-SET presses ~250ms apart while already
    # engaged and steady - tests whether closely-spaced commanded presses fault the ECU or
    # break counter/checksum continuity. Decrease direction only (bounded, ~3mph total).
    # Aborts (not just pauses) the instant steady-state is lost mid-sequence, rather than
    # trying to resume a stale plan later. ---
    if burst_armed and _fresh(ES_DISTANCE_BURST_FLAG) and not self.es_distance_burst_fired:
      steady = abs(CS.out.vEgo - CS.out.cruiseState.speed) < ES_DISTANCE_DEEP_STEADY_TOLERANCE
      ready = CS.out.cruiseState.enabled and steady and no_real_press
      self.es_distance_burst_sustain = self.es_distance_burst_sustain + 1 if ready else 0

      started = self.es_distance_burst_sent > 0
      may_send_next = started and self.frame >= self.es_distance_burst_next_frame

      if (not started and self.es_distance_burst_sustain >= ES_DISTANCE_SUSTAIN_FRAMES) or (started and may_send_next and ready):
        real_cruise_button = cruise_button
        cruise_button = 2  # SET shallow, repeated
        self.es_distance_burst_sent += 1
        self.es_distance_burst_next_frame = self.frame + ES_DISTANCE_BURST_SPACING_CYCLES
        with open(ES_DISTANCE_TEST_LOG, "a") as f:
          f.write(f"{time.time()} FIRED TEST=burst seq={self.es_distance_burst_sent}/{ES_DISTANCE_BURST_COUNT} "
                  f"cruise_button=2 (real={real_cruise_button}) frame={self.frame} vEgo={CS.out.vEgo} "
                  f"cruiseSpeed={CS.out.cruiseState.speed}\n")
        if self.es_distance_burst_sent >= ES_DISTANCE_BURST_COUNT:
          self.es_distance_burst_fired = True
          try:
            os.remove(ES_DISTANCE_BURST_FLAG)
          except OSError:
            pass
      elif started and not ready:
        self.es_distance_burst_fired = True
        try:
          os.remove(ES_DISTANCE_BURST_FLAG)
        except OSError:
          pass
        with open(ES_DISTANCE_TEST_LOG, "a") as f:
          f.write(f"{time.time()} ABORTED TEST=burst after {self.es_distance_burst_sent}/{ES_DISTANCE_BURST_COUNT} "
                  f"sent - lost steady-state or real press intervened\n")

    return cruise_button
