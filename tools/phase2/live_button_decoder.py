#!/usr/bin/env python3
"""
Phase 2 recon tool — NOT part of the shipped sunnypilot build, never imported
by anything else. Passive/RX-only: subscribes to the 'can' message and watches
for the empirically-confirmed ACC button bits on CruiseControl (0x144). Never
sends anything on the bus.

Usage: run standalone over SSH while the car is on (ignition on is enough,
doesn't need to be moving). Press the steering wheel SET/RES buttons in a
known pattern and confirm the printed detections match what you actually
pressed, in real time. Logs to both stdout and a timestamped file so it can
be reviewed after driving instead of watched while driving.

Confirmed 2026-07-22 via correlation against 412 real button-press events in
locally archived rlogs (see progress.md Q4):
  SET_BUTTON (speed down) = byte 0, bit 3
  RES_BUTTON (speed up)   = byte 0, bit 4
"""
import datetime
import os

import cereal.messaging as messaging

CRUISE_CONTROL_ADDR = 0x144  # 324 decimal

SET_BIT = 3
RES_BIT = 4

LOG_PATH = "/data/phase2_button_log.txt"


def bit(byte0: int, n: int) -> int:
  return (byte0 >> n) & 1


def main():
  sock = messaging.sub_sock('can')
  prev_byte0: dict[int, int] = {}  # src (bus) -> last byte0 seen

  log_f = open(LOG_PATH, "a", buffering=1)

  def emit(line: str):
    ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
    full = f"[{ts}] {line}"
    print(full, flush=True)
    log_f.write(full + "\n")

  emit(f"--- live_button_decoder starting, pid={os.getpid()}, logging to {LOG_PATH} ---")
  emit("Press SET/RES on the steering wheel now and watch for detections below.")

  frame_count = 0
  while True:
    msgs = messaging.drain_sock(sock, wait_for_one=True)
    for msg in msgs:
      for can in msg.can:
        if can.address != CRUISE_CONTROL_ADDR or len(can.dat) < 1:
          continue

        frame_count += 1
        src = can.src
        byte0 = can.dat[0]
        prev = prev_byte0.get(src)
        prev_byte0[src] = byte0

        if prev is None:
          continue  # first frame on this bus, nothing to compare against yet

        if bit(prev, SET_BIT) == 0 and bit(byte0, SET_BIT) == 1:
          emit(f"SET pressed   (bus={src}, byte0 {prev:#04x} -> {byte0:#04x})")

        if bit(prev, RES_BIT) == 0 and bit(byte0, RES_BIT) == 1:
          emit(f"RES pressed   (bus={src}, byte0 {prev:#04x} -> {byte0:#04x})")

        # heartbeat every ~1000 frames (~50s at 20Hz) so it's obvious the script is alive
        if frame_count % 1000 == 0:
          emit(f"... alive, {frame_count} CruiseControl frames seen so far (bus={src}, current byte0={byte0:#04x})")


if __name__ == "__main__":
  try:
    main()
  except KeyboardInterrupt:
    pass
