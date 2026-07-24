"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from cereal import messaging, custom
from opendbc.car import structs
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX
from openpilot.sunnypilot.selfdrive.controls.lib.curve_advisory_helper import CurveAdvisoryHelper
from openpilot.sunnypilot.selfdrive.controls.lib.phase3_shared import Phase3OverrideLatch, Phase3CommandArbiter
from openpilot.sunnypilot.selfdrive.controls.lib.phase3_curve_controller import Phase3CurveController
from openpilot.sunnypilot.selfdrive.controls.lib.phase3_lead_controller import Phase3LeadController
from openpilot.sunnypilot.selfdrive.controls.lib.phase3_slf_controller import Phase3SlfController
from openpilot.sunnypilot.selfdrive.controls.lib.lead_closing_advisory_helper import LeadClosingAdvisoryHelper
from openpilot.sunnypilot.selfdrive.controls.lib.lead_closing_test_guidance_helper import LeadClosingTestGuidanceHelper
from openpilot.sunnypilot.selfdrive.controls.lib.dec.dec import DynamicExperimentalController
from openpilot.sunnypilot.selfdrive.controls.lib.e2e_alerts_helper import E2EAlertsHelper
from openpilot.sunnypilot.selfdrive.controls.lib.smart_cruise_control.smart_cruise_control import SmartCruiseControl
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_assist import SpeedLimitAssist
from openpilot.sunnypilot.selfdrive.controls.lib.speed_limit.speed_limit_resolver import SpeedLimitResolver
from openpilot.sunnypilot.selfdrive.selfdrived.events import EventsSP
from openpilot.sunnypilot.models.helpers import get_active_bundle

DecState = custom.LongitudinalPlanSP.DynamicExperimentalControl.DynamicExperimentalControlState
LongitudinalPlanSource = custom.LongitudinalPlanSP.LongitudinalPlanSource


class LongitudinalPlannerSP:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP, mpc):
    self.events_sp = EventsSP()
    self.resolver = SpeedLimitResolver()
    self.dec = DynamicExperimentalController(CP, mpc)
    self.scc = SmartCruiseControl()
    self.curve_advisory = CurveAdvisoryHelper()
    # Shared across every Phase 3 actuation feature - one override event latches all of
    # them off together (see phase3_shared.Phase3OverrideLatch docstring).
    self.phase3_override_latch = Phase3OverrideLatch()
    self.phase3_was_long_enabled = False  # for the latch's clear_on_reengage() edge detection
    # Shared arbiter: only one real button command can be written per planner cycle -
    # curve controller is called first below and wins ties on purpose (see
    # Phase3CommandArbiter's own docstring). Also enforces the whole-drive
    # SESSION_COMMAND_CAP backstop across both features combined.
    self.phase3_command_arbiter = Phase3CommandArbiter()
    self.phase3_curve_controller = Phase3CurveController(self.phase3_override_latch, self.phase3_command_arbiter)
    self.phase3_lead_controller = Phase3LeadController(self.phase3_override_latch, self.phase3_command_arbiter)
    # Third Phase 3 feature (2026-07-24) - speed-limit-following. Called last, after
    # curve/lead, both so it naturally loses arbiter ties to them (§6's priority order)
    # and so its own context-gated button routing can read curve/lead's CURRENT-frame
    # was_active/in_episode state, not last frame's.
    self.phase3_slf_controller = Phase3SlfController(self.phase3_override_latch, self.phase3_command_arbiter)
    self.lead_closing_advisory = LeadClosingAdvisoryHelper()
    self.lead_closing_test_guidance = LeadClosingTestGuidanceHelper()
    self.resolver = SpeedLimitResolver()
    self.sla = SpeedLimitAssist(CP, CP_SP)
    self.generation = int(model_bundle.generation) if (model_bundle := get_active_bundle()) else None
    self.source = LongitudinalPlanSource.cruise
    self.e2e_alerts_helper = E2EAlertsHelper()

    self.output_v_target = 0.
    self.output_a_target = 0.

  def is_e2e(self, sm: messaging.SubMaster) -> bool:
    experimental_mode = sm['selfdriveState'].experimentalMode
    if not self.dec.active():
      return experimental_mode

    return experimental_mode and self.dec.mode() == "blended"

  def update_targets(self, sm: messaging.SubMaster, v_ego: float, a_ego: float, v_cruise: float) -> tuple[float, float]:
    CS = sm['carState']
    v_cruise_cluster_kph = min(CS.vCruiseCluster, V_CRUISE_MAX)
    v_cruise_cluster = v_cruise_cluster_kph * CV.KPH_TO_MS

    long_enabled = sm['carControl'].enabled
    long_override = sm['carControl'].cruiseControl.override

    # Rising edge of cruise-enabled ("set the cruise again") clears the shared Phase 3
    # override latch - see Phase3OverrideLatch.clear_on_reengage()'s own docstring for
    # why this replaced the original whole-drive-lockout behavior. Checked once here,
    # shared across curve/lead/SLF, not duplicated per controller.
    if long_enabled and not self.phase3_was_long_enabled:
      self.phase3_override_latch.clear_on_reengage()
    self.phase3_was_long_enabled = long_enabled

    # Smart Cruise Control
    self.scc.update(sm, long_enabled, long_override, v_ego, a_ego, v_cruise)
    self.curve_advisory.update(self.scc.map.state, long_enabled, v_ego, self.events_sp)
    self.phase3_command_arbiter.new_cycle(DT_MDL)  # reset the one-write-per-cycle gate, tick the shared clock
    # NOTE: CS.cruise_button is deliberately NOT passed here (2026-07-24 postmortem,
    # crashed plannerd outright: "struct has no such member; name = cruise_button" -
    # that field only exists on opendbc's raw CarState object inside carcontroller.py,
    # a different object than this capnp-published sm['carState']/CS). Not a safety
    # gap - see Phase3OverrideLatch.check()'s own docstring for why.
    self.phase3_curve_controller.update(self.scc.map.state, self.scc.map.distance, self.scc.map.output_v_target,
                                         long_enabled, v_ego, v_cruise,
                                         CS.gasPressed, CS.brakePressed, CS.steeringPressed)
    self.phase3_lead_controller.update(sm['radarState'].leadOne, long_enabled, v_ego, v_cruise,
                                        CS.gasPressed, CS.brakePressed, CS.steeringPressed)
    self.lead_closing_advisory.update(sm['radarState'].leadOne, long_enabled, v_ego,
                                       CS.gasPressed, CS.brakePressed, self.events_sp)
    self.lead_closing_test_guidance.update(sm['radarState'].leadOne, long_enabled, v_ego, v_cruise_cluster,
                                            CS.gasPressed, CS.brakePressed, self.events_sp)

    # Speed Limit Resolver
    self.resolver.update(v_ego, sm)

    # Phase 3 speed-limit-following (2026-07-24) - own, self-contained implementation,
    # deliberately not built on top of SpeedLimitAssist above: that class is a large,
    # unfamiliar state machine whose button-based path may itself depend on
    # CS.buttonEvents, the exact field already confirmed empty on this preglobal car -
    # not audited under today's time constraint, worth a real look separately rather
    # than risking an unverified adaptation. Reuses the resolver's already-computed
    # speed_limit (m/s) exactly like curve reuses MTSC's output_v_target - no duplicate
    # computation. Called after curve/lead so it naturally loses arbiter ties to both
    # (§6 priority: curve > lead > slf) and so its context-gated button routing reads
    # curve/lead's current-frame was_active/in_episode.
    slf_limit_mph = self.resolver.speed_limit * CV.MS_TO_MPH if self.resolver.speed_limit_valid else None
    self.phase3_slf_controller.update(slf_limit_mph, long_enabled, v_ego, v_cruise,
                                       CS.gasPressed, CS.brakePressed, CS.steeringPressed,
                                       self.phase3_curve_controller.was_active, self.phase3_lead_controller.in_episode)

    # Speed Limit Assist
    has_speed_limit = self.resolver.speed_limit_valid or self.resolver.speed_limit_last_valid
    self.sla.update(long_enabled, long_override, v_ego, a_ego, v_cruise_cluster, self.resolver.speed_limit,
                    self.resolver.speed_limit_final_last, has_speed_limit, self.resolver.distance, self.events_sp)

    targets = {
      LongitudinalPlanSource.cruise: (v_cruise, a_ego),
      LongitudinalPlanSource.sccVision: (self.scc.vision.output_v_target, self.scc.vision.output_a_target),
      LongitudinalPlanSource.sccMap: (self.scc.map.output_v_target, self.scc.map.output_a_target),
      LongitudinalPlanSource.speedLimitAssist: (self.sla.output_v_target, self.sla.output_a_target),
    }

    self.source = min(targets, key=lambda k: targets[k][0])
    self.output_v_target, self.output_a_target = targets[self.source]
    return self.output_v_target, self.output_a_target

  def update(self, sm: messaging.SubMaster) -> None:
    self.events_sp.clear()
    self.dec.update(sm)
    self.e2e_alerts_helper.update(sm, self.events_sp)

  def publish_longitudinal_plan_sp(self, sm: messaging.SubMaster, pm: messaging.PubMaster) -> None:
    plan_sp_send = messaging.new_message('longitudinalPlanSP')

    plan_sp_send.valid = sm.all_checks(service_list=['carState', 'controlsState'])

    longitudinalPlanSP = plan_sp_send.longitudinalPlanSP
    longitudinalPlanSP.longitudinalPlanSource = self.source
    longitudinalPlanSP.vTarget = float(self.output_v_target)
    longitudinalPlanSP.aTarget = float(self.output_a_target)
    longitudinalPlanSP.events = self.events_sp.to_msg()

    # Dynamic Experimental Control
    dec = longitudinalPlanSP.dec
    dec.state = DecState.blended if self.dec.mode() == 'blended' else DecState.acc
    dec.enabled = self.dec.enabled()
    dec.active = self.dec.active()

    # Smart Cruise Control
    smartCruiseControl = longitudinalPlanSP.smartCruiseControl
    # Vision Control
    sccVision = smartCruiseControl.vision
    sccVision.state = self.scc.vision.state
    sccVision.vTarget = float(self.scc.vision.output_v_target)
    sccVision.aTarget = float(self.scc.vision.output_a_target)
    sccVision.currentLateralAccel = float(self.scc.vision.current_lat_acc)
    sccVision.maxPredictedLateralAccel = float(self.scc.vision.max_pred_lat_acc)
    sccVision.enabled = self.scc.vision.is_enabled
    sccVision.active = self.scc.vision.is_active
    # Map Control
    sccMap = smartCruiseControl.map
    sccMap.state = self.scc.map.state
    sccMap.vTarget = float(self.scc.map.output_v_target)
    sccMap.aTarget = float(self.scc.map.output_a_target)
    sccMap.enabled = self.scc.map.is_enabled
    sccMap.active = self.scc.map.is_active
    sccMap.distance = float(self.scc.map.distance)

    # Speed Limit
    speedLimit = longitudinalPlanSP.speedLimit
    resolver = speedLimit.resolver
    resolver.speedLimit = float(self.resolver.speed_limit)
    resolver.speedLimitLast = float(self.resolver.speed_limit_last)
    resolver.speedLimitFinal = float(self.resolver.speed_limit_final)
    resolver.speedLimitFinalLast = float(self.resolver.speed_limit_final_last)
    resolver.speedLimitValid = self.resolver.speed_limit_valid
    resolver.speedLimitLastValid = self.resolver.speed_limit_last_valid
    resolver.speedLimitOffset = float(self.resolver.speed_limit_offset)
    resolver.distToSpeedLimit = float(self.resolver.distance)
    resolver.source = self.resolver.source
    assist = speedLimit.assist
    assist.state = self.sla.state
    assist.enabled = self.sla.is_enabled
    assist.active = self.sla.is_active
    assist.vTarget = float(self.sla.output_v_target)
    assist.aTarget = float(self.sla.output_a_target)

    # Lead-closing test guidance (validation tool, opt-in, off by default)
    leadClosingTest = longitudinalPlanSP.leadClosingTest
    leadClosingTest.vTarget = float(self.lead_closing_test_guidance.v_target)
    leadClosingTest.active = self.lead_closing_test_guidance.active

    # E2E Alerts
    e2eAlerts = longitudinalPlanSP.e2eAlerts
    e2eAlerts.greenLightAlert = self.e2e_alerts_helper.green_light_alert
    e2eAlerts.leadDepartAlert = self.e2e_alerts_helper.lead_depart_alert

    pm.send('longitudinalPlanSP', plan_sp_send)
