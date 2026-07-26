"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import cereal.messaging as messaging
from cereal import log, car, custom
from openpilot.common.constants import CV
from openpilot.common.params import Params, UnknownKeyName
from openpilot.sunnypilot.selfdrive.selfdrived.events_base import EventsBase, Priority, ET, Alert, \
  NoEntryAlert, ImmediateDisableAlert, EngagementAlert, NormalPermanentAlert, AlertCallbackType, wrong_car_mode_alert
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit import PCM_LONG_REQUIRED_MAX_SET_SPEED, CONFIRM_SPEED_THRESHOLD
from openpilot.sunnypilot.selfdrive.controls.lib.phase3_shared import read_ui_status
from openpilot.system.hardware import HARDWARE

AlertSize = log.SelfdriveState.AlertSize
AlertStatus = log.SelfdriveState.AlertStatus
VisualAlert = car.CarControl.HUDControl.VisualAlert
AudibleAlert = car.CarControl.HUDControl.AudibleAlert
AudibleAlertSP = custom.SelfdriveStateSP.AudibleAlert
EventNameSP = custom.OnroadEventSP.EventName


# get event name from enum
EVENT_NAME_SP = {v: k for k, v in EventNameSP.schema.enumerants.items()}

IS_MICI = HARDWARE.get_device_type() == 'mici'


def speed_limit_adjust_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int, personality) -> Alert:
  speedLimit = sm['longitudinalPlanSP'].speedLimit.resolver.speedLimit
  speed = round(speedLimit * (CV.MS_TO_KPH if metric else CV.MS_TO_MPH))
  message = f'Adjusting to {speed} {"km/h" if metric else "mph"} speed limit'
  return Alert(
    message,
    "",
    AlertStatus.normal, AlertSize.small,
    Priority.LOW, VisualAlert.none, AudibleAlert.none, 4.)


def speed_limit_pre_active_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int, personality) -> Alert:
  speed_conv = CV.MS_TO_KPH if metric else CV.MS_TO_MPH
  v_cruise_cluster = CS.vCruiseCluster
  set_speed = sm['controlsState'].deprecated.vCruise if v_cruise_cluster == 0.0 else v_cruise_cluster
  set_speed_conv = round(set_speed * speed_conv)

  speed_limit_final_last = sm['longitudinalPlanSP'].speedLimit.resolver.speedLimitFinalLast
  speed_limit_final_last_conv = round(speed_limit_final_last * speed_conv)
  alert_1_str = ""
  alert_size = AlertSize.small

  if CP.openpilotLongitudinalControl and CP.pcmCruise:
    # PCM long
    cst_low, cst_high = PCM_LONG_REQUIRED_MAX_SET_SPEED[metric]
    pcm_long_required_max = cst_low if speed_limit_final_last_conv < CONFIRM_SPEED_THRESHOLD[metric] else cst_high
    pcm_long_required_max_set_speed_conv = round(pcm_long_required_max * speed_conv)
    speed_unit = "km/h" if metric else "mph"

    alert_1_str = f"Speed Limit Assist: set to {pcm_long_required_max_set_speed_conv} {speed_unit} to engage"
  else:
    if IS_MICI:
      if set_speed_conv < speed_limit_final_last_conv:
        alert_1_str = "Press + to confirm speed limit"
      elif set_speed_conv > speed_limit_final_last_conv:
        alert_1_str = "Press - to confirm speed limit"
    else:
      alert_size = AlertSize.none

  return Alert(
    alert_1_str,
    "",
    AlertStatus.normal, alert_size,
    Priority.LOW, VisualAlert.none, AudibleAlertSP.promptSingleLow, .1)


def _phase3_curve_armed() -> bool:
  # Shadow-mode/actuation wording switch - see research/phase3_controller_design.md §7
  # ("Alert during auto-actuation" row). Reads Params directly (unlike sibling alert
  # functions in this file, which only read sm) since Phase3Armed isn't published into
  # any capnp message - adding one would need a schema change/rebuild, not available on
  # this prebuilt branch (no SConstruct). A plain Params() read has no such constraint.
  try:
    return Params().get_bool("Phase3Armed")
  except UnknownKeyName:
    return False  # same compiled-allowlist landmine as CurveSpeedAdvisory; defaults off


def curve_speed_advisory_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int, personality) -> Alert:
  scc_map = sm['longitudinalPlanSP'].smartCruiseControl.map
  speed = round(scc_map.vTarget * (CV.MS_TO_KPH if metric else CV.MS_TO_MPH))
  speed_unit = "km/h" if metric else "mph"

  if metric:
    dist_str = f"{round(scc_map.distance)} m"
  else:
    dist_str = f"{round(scc_map.distance * 3.28084)} ft"

  if _phase3_curve_armed():
    # Informational, not advisory - the driver doesn't need to act, Phase 3 is already
    # walking the target down on its own. Still shown (not suppressed) for situational
    # awareness - silently changing speed with zero indicator would erode trust.
    return Alert(
      "Curve Ahead",
      f"auto-adjusting to {speed} {speed_unit} in {dist_str}",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlertSP.promptSingleHigh, 4.)

  return Alert(
    "Curve Ahead",
    f"reduce to {speed} {speed_unit} in {dist_str}",
    AlertStatus.normal, AlertSize.mid,
    Priority.LOW, VisualAlert.none, AudibleAlertSP.promptSingleHigh, 4.)


def lead_closing_advisory_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int, personality) -> Alert:
  return Alert(
    "Traffic Ahead",
    "vehicle ahead may be slowing",
    AlertStatus.normal, AlertSize.mid,
    Priority.LOW, VisualAlert.none, AudibleAlertSP.promptSingleHigh, 4.)


# Repurposed 2026-07-26 (see lead_closing_test_guidance_helper.py's own docstring):
# reused EventNameSP.leadClosingTestGuidance slot, now the Phase 3 override-trip
# one-shot alert instead of the original closing-lead validation-tool prompt. Reads
# /data/phase3_ui_status.json directly rather than a capnp field - trip_reason is a
# string with no home in any existing schema, and adding one needs capnp codegen this
# build can't do (same landmine _phase3_curve_armed()'s docstring above describes).
_TRIP_REASON_TEXT = {
  "gas": "gas pedal",
  "brake": "brake pedal",
  "steering": "steering wheel",
  "gas+brake": "gas + brake",
  "gas+steering": "gas + steering",
  "brake+steering": "brake + steering",
  "gas+brake+steering": "gas + brake + steering",
}


def lead_closing_test_guidance_alert(CP: car.CarParams, CS: car.CarState, sm: messaging.SubMaster, metric: bool, soft_disable_time: int, personality) -> Alert:
  status = read_ui_status()
  reason = _TRIP_REASON_TEXT.get(status.get("trip_reason") if status else None, "override detected")

  return Alert(
    "Phase 3 Off",
    reason,
    AlertStatus.normal, AlertSize.mid,
    Priority.LOW, VisualAlert.none, AudibleAlertSP.promptSingleHigh, 4.)


class EventsSP(EventsBase):
  def __init__(self):
    super().__init__()
    self.event_counters = dict.fromkeys(EVENTS_SP.keys(), 0)

  def get_events_mapping(self) -> dict[int, dict[str, Alert | AlertCallbackType]]:
    return EVENTS_SP

  def get_event_name(self, event: int):
    return EVENT_NAME_SP[event]

  def get_event_msg_type(self):
    return custom.OnroadEventSP.Event


EVENTS_SP: dict[int, dict[str, Alert | AlertCallbackType]] = {
  # sunnypilot
  EventNameSP.lkasEnable: {
    ET.ENABLE: EngagementAlert(AudibleAlert.engage),
  },

  EventNameSP.lkasDisable: {
    ET.USER_DISABLE: EngagementAlert(AudibleAlert.disengage),
  },

  EventNameSP.manualSteeringRequired: {
    ET.USER_DISABLE: Alert(
      "Automatic Lane Centering is OFF",
      "Manual Steering Required",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.disengage, 1.),
  },

  EventNameSP.manualLongitudinalRequired: {
    ET.WARNING: Alert(
      "Smart/Adaptive Cruise Control: OFF",
      "Manual Speed Control Required",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, 1.),
  },

  EventNameSP.silentLkasEnable: {
    ET.ENABLE: EngagementAlert(AudibleAlert.none),
  },

  EventNameSP.silentLkasDisable: {
    ET.USER_DISABLE: EngagementAlert(AudibleAlert.none),
  },

  EventNameSP.silentBrakeHold: {
    ET.WARNING: EngagementAlert(AudibleAlert.none),
    ET.NO_ENTRY: NoEntryAlert("Brake Hold Active"),
  },

  EventNameSP.silentWrongGear: {
    ET.WARNING: Alert(
      "",
      "",
      AlertStatus.normal, AlertSize.none,
      Priority.LOWEST, VisualAlert.none, AudibleAlert.none, 0.),
    ET.NO_ENTRY: Alert(
      "Gear not D",
      "openpilot Unavailable",
      AlertStatus.normal, AlertSize.mid,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, 0.),
  },

  EventNameSP.silentReverseGear: {
    ET.PERMANENT: Alert(
      "Reverse\nGear",
      "",
      AlertStatus.normal, AlertSize.full,
      Priority.LOWEST, VisualAlert.none, AudibleAlert.none, .2, creation_delay=0.5),
    ET.NO_ENTRY: NoEntryAlert("Reverse Gear"),
  },

  EventNameSP.silentDoorOpen: {
    ET.WARNING: Alert(
      "",
      "",
      AlertStatus.normal, AlertSize.none,
      Priority.LOWEST, VisualAlert.none, AudibleAlert.none, 0.),
    ET.NO_ENTRY: NoEntryAlert("Door Open"),
  },

  EventNameSP.silentSeatbeltNotLatched: {
    ET.WARNING: Alert(
      "",
      "",
      AlertStatus.normal, AlertSize.none,
      Priority.LOWEST, VisualAlert.none, AudibleAlert.none, 0.),
    ET.NO_ENTRY: NoEntryAlert("Seatbelt Unlatched"),
  },

  EventNameSP.silentParkBrake: {
    ET.WARNING: Alert(
      "",
      "",
      AlertStatus.normal, AlertSize.none,
      Priority.LOWEST, VisualAlert.none, AudibleAlert.none, 0.),
    ET.NO_ENTRY: NoEntryAlert("Parking Brake Engaged"),
  },

  EventNameSP.controlsMismatchLateral: {
    ET.IMMEDIATE_DISABLE: ImmediateDisableAlert("Controls Mismatch: Lateral"),
    ET.NO_ENTRY: NoEntryAlert("Controls Mismatch: Lateral"),
  },

  EventNameSP.experimentalModeSwitched: {
    ET.WARNING: NormalPermanentAlert("Experimental Mode Switched", duration=1.5)
  },

  EventNameSP.wrongCarModeAlertOnly: {
    ET.WARNING: wrong_car_mode_alert,
  },

  EventNameSP.pedalPressedAlertOnly: {
    ET.WARNING: NoEntryAlert("Pedal Pressed")
  },

  EventNameSP.laneTurnLeft: {
    ET.WARNING: Alert(
      "Turning Left",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, 1.),
  },

  EventNameSP.laneTurnRight: {
    ET.WARNING: Alert(
      "Turning Right",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlert.none, 1.),
  },

  EventNameSP.speedLimitActive: {
    ET.WARNING: Alert(
      "Auto adjusting to speed limit",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlertSP.promptSingleHigh, 5.),
  },

  EventNameSP.speedLimitChanged: {
    ET.WARNING: Alert(
      "Set speed changed",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlertSP.promptSingleHigh, 5.),
  },

  EventNameSP.speedLimitPreActive: {
    ET.WARNING: speed_limit_pre_active_alert,
  },

  EventNameSP.speedLimitPending: {
    ET.WARNING: Alert(
      "Auto adjusting to last speed limit",
      "",
      AlertStatus.normal, AlertSize.small,
      Priority.LOW, VisualAlert.none, AudibleAlertSP.promptSingleHigh, 5.),
  },

  EventNameSP.e2eChime: {
    ET.PERMANENT: Alert(
      "",
      "",
      AlertStatus.normal, AlertSize.none,
      Priority.MID, VisualAlert.none, AudibleAlert.prompt, 3.),
  },

  EventNameSP.curveSpeedAdvisory: {
    ET.WARNING: curve_speed_advisory_alert,
  },

  EventNameSP.leadClosingAdvisory: {
    ET.WARNING: lead_closing_advisory_alert,
  },

  EventNameSP.leadClosingTestGuidance: {
    ET.WARNING: lead_closing_test_guidance_alert,
  },
}
