#!/usr/bin/env python3
"""
Phase 2 recon tool — NOT part of the shipped sunnypilot build, never imported
by anything else. Passive/RX-only: subscribes to the 'can' message and watches
for ANY bit transition on CruiseControl (0x144) byte0 and byte1, labeling the
already-confirmed bits and flagging a specific candidate for the physical
main ACC on/off button. Never sends anything on the bus.

Motivation: opendbc's own DBC (_subaru_preglobal_2015.dbc) labels byte0 bit2
as "OnOffButton", adjacent to SET_BUTTON (bit3) and RES_BUTTON (bit4) which
were both independently confirmed correct against 412 real archived events
plus a live real-time test (see progress.md Q4). Same DBC, same message,
same byte — strong prior, but unconfirmed for this specific bit, so this
script watches ALL byte0/byte1 bits (not just bit2) to catch the real answer
even if the DBC's OnOffButton guess turns out wrong, same discipline that
caught the DBC's own wrong "bits 48/49" claim being corrected in the past.

Usage: run standalone over SSH with the car key at least in ACC/ON position
(engine doesn't need to run — the steering wheel/BCM module just needs power
to transmit on the bus at all). Press the physical MAIN/power ACC button in
a deliberate, spaced-out pattern and confirm which bit(s) flip each time.
"""
import datetime
import os

import cereal.messaging as messaging

CRUISE_CONTROL_ADDR = 0x144  # 324 decimal

KNOWN_BITS = {
  2: "OnOffButton (DBC label, UNCONFIRMED candidate for MAIN button)",
  3: "SET_BUTTON (confirmed)",
  4: "RES_BUTTON (confirmed)",
}

LOG_PATH = "/data/phase2_main_button_log.txt"


def changed_bits(prev: int, cur: int) -> list[int]:
  diff = prev ^ cur
  return [n for n in range(8) if (diff >> n) & 1]


def main():
  sock = messaging.sub_sock('can')
  prev_bytes: dict[int, tuple[int, int]] = {}  # src -> (byte0, byte1)

  log_f = open(LOG_PATH, "a", buffering=1)

  def emit(line: str):
    ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
    full = f"[{ts}] {line}"
    print(full, flush=True)
    log_f.write(full + "\n")

  emit(f"--- live_main_button_decoder starting, pid={os.getpid()}, logging to {LOG_PATH} ---")
  emit("Watching ALL byte0/byte1 bit transitions on CruiseControl (0x144).")
  emit("Press the physical MAIN/power ACC button now, one press at a time, with a pause between.")

  frame_count = 0
  while True:
    msgs = messaging.drain_sock(sock, wait_for_one=True)
    for msg in msgs:
      for can in msg.can:
        if can.address != CRUISE_CONTROL_ADDR or len(can.dat) < 2:
          continue

        frame_count += 1
        src = can.src
        byte0, byte1 = can.dat[0], can.dat[1]
        prev = prev_bytes.get(src)
        prev_bytes[src] = (byte0, byte1)

        if prev is None:
          continue  # first frame on this bus, nothing to compare against yet

        prev_byte0, prev_byte1 = prev

        for n in changed_bits(prev_byte0, byte0):
          rising = ((byte0 >> n) & 1) == 1
          label = KNOWN_BITS.get(n, "byte0 UNKNOWN bit")
          edge = "RISING (0->1)" if rising else "falling (1->0)"
          emit(f"byte0 bit{n} {edge}  [{label}]  (bus={src}, {prev_byte0:#04x} -> {byte0:#04x})")

        for n in changed_bits(prev_byte1, byte1):
          if n == 0:
            continue  # byte1 bit0 (DBC bit8) = known rolling counter/heartbeat, not a button (see Q4) - pure noise here
          rising = ((byte1 >> n) & 1) == 1
          edge = "RISING (0->1)" if rising else "falling (1->0)"
          emit(f"byte1 bit{n} {edge}  [byte1 UNKNOWN bit, bit13-8={n+8} in DBC numbering]  (bus={src}, {prev_byte1:#04x} -> {byte1:#04x})")

        if frame_count % 1000 == 0:
          emit(f"... alive, {frame_count} CruiseControl frames seen so far (bus={src}, byte0={byte0:#04x} byte1={byte1:#04x})")


if __name__ == "__main__":
  try:
    main()
  except KeyboardInterrupt:
    pass
