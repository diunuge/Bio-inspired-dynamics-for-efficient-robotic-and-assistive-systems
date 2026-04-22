#!/usr/bin/env python3
"""
jump_sweep.py  —  Capstan Drive Jump Frequency Sweep Data Collection
=====================================================================
Collects synchronized Leptrino force sensor data during jumpFrequency()
sweep driven by Teensy serial phase markers.



Output: Data/jump_speed_X.XXmps.csv
CSV columns: time_s, speed_mps, rep, phase, Fx, Fy, Fz, Mx, My, Mz
"""
import csv
import struct
import threading
import time
from collections import deque
from pathlib import Path

import serial

# ── Configuration ──────────────────────────────────────────────────────────────
TEENSY_PORT = "COM11"
TEENSY_BAUD = 115200

LEP_PORT = "COM5"
LEP_BAUD = 9600
LEP_HZ   = 100          # Leptrino hardware rate (100 Hz = 10 ms/sample)

DATA_DIR = Path("Data")
DATA_DIR.mkdir(exist_ok=True)

SERIAL_BOOT_DELAY_S  = 2.0
CLOSED_LOOP_WAIT_S   = 4.0   # wait after 'c' command before sending 'f'
SWEEP_TIMEOUT_S      = 300.0  # 5-minute hard cap for the whole sweep

# ── CSV field order ────────────────────────────────────────────────────────────
CSV_FIELDS = ["time_s", "speed_mps", "rep", "phase",
              "Fx", "Fy", "Fz", "Mx", "My", "Mz"]

# ══════════════════════════════════════════════════════════════════════════════
# Leptrino Client
# ══════════════════════════════════════════════════════════════════════════════
class LeptrinoClient:
    """
    Low-level driver for the Leptrino force/torque sensor.
    Samples are accumulated in a thread-safe deque; call drain_samples()
    to retrieve and clear all samples collected since the last drain.
    """
    DLE = 0x10
    STX = 0x02
    ETX = 0x03

    def __init__(self, port: str = LEP_PORT, baud: int = LEP_BAUD):
        self.ser = serial.Serial(
            port=port, baudrate=baud,
            bytesize=8, parity="N", stopbits=1,
            timeout=0,
        )
        time.sleep(0.2)
        self._rx_buf     = bytearray()
        # Thread-safe sample queue: each item is (perf_time, Fx, Fy, Fz, Mx, My, Mz)
        self._samples    = deque()
        self._hardware_reset()
        self.rated_values = self._get_rated_values()

    # ── Public API ─────────────────────────────────────────────────────────────
    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    def drain_samples(self) -> list:
        """Return all accumulated (perf_time, Fx…Mz) tuples and clear the buffer."""
        out = []
        try:
            while True:
                out.append(self._samples.popleft())
        except IndexError:
            pass
        return out

    def run_background(self, stop_event: threading.Event, hz: int = LEP_HZ):
        """
        Background thread: sample at `hz` Hz, push (perf_time, Fx..Mz) to deque.
        Uses perf_counter for sub-millisecond timing resolution.
        """
        period = 1.0 / max(1, hz)
        while not stop_event.is_set():
            t_start = time.perf_counter()
            try:
                vals = self.read_sample()
                ts   = time.perf_counter()
                self._samples.append((ts, *vals))
            except Exception:
                pass
            elapsed = time.perf_counter() - t_start
            rem = period - elapsed
            if rem > 0:
                time.sleep(rem)

    # ── Frame I/O ──────────────────────────────────────────────────────────────
    def _send_payload(self, payload: bytes):
        framed = bytearray([self.DLE, self.STX])
        bcc = 0
        for byte in payload:
            if byte == self.DLE:
                framed.append(self.DLE)
            framed.append(byte)
            bcc ^= byte
        framed.extend([self.DLE, self.ETX, bcc ^ self.ETX])
        self.ser.write(framed)

    def _read_frames(self) -> list:
        chunk = self.ser.read(256)
        if chunk:
            self._rx_buf.extend(chunk)
        frames = []
        buf = self._rx_buf
        while True:
            stx = buf.find(bytes([self.DLE, self.STX]))
            if stx < 0:
                if len(buf) > 4096:
                    del buf[:-64]
                break
            etx = buf.find(bytes([self.DLE, self.ETX]), stx + 2)
            if etx < 0 or etx + 2 >= len(buf):
                break
            escaped = buf[stx + 2:etx]
            recv_bcc = buf[etx + 2]
            payload = bytearray()
            idx = 0
            while idx < len(escaped):
                b = escaped[idx]
                if b == self.DLE and idx + 1 < len(escaped) and escaped[idx + 1] == self.DLE:
                    payload.append(self.DLE)
                    idx += 2
                else:
                    payload.append(b)
                    idx += 1
            bcc = 0
            for b in payload:
                bcc ^= b
            bcc ^= self.ETX
            frames.append((bytes(payload), bcc == recv_bcc))
            del buf[: etx + 3]
        return frames

    def _receive_payload(self, timeout: float = 0.5) -> bytes:
        deadline = time.time() + timeout
        while time.time() < deadline:
            for payload, ok in self._read_frames():
                if ok:
                    return payload
        raise TimeoutError("Leptrino: no valid frame received")

    def _hardware_reset(self):
        self._send_payload(bytes([0x08, 0xFF, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00]))
        time.sleep(1.5)
        self.ser.reset_input_buffer()

    def _get_rated_values(self):
        self._send_payload(struct.pack("<4B", 4, 0xFF, 0x2B, 0x00))
        response = self._receive_payload(timeout=1.0)
        if len(response) < 28:
            raise ValueError(f"Leptrino rated-values reply too short ({len(response)} bytes)")
        return struct.unpack("<6f", response[4:28])

    def read_sample(self) -> list:
        self._send_payload(struct.pack("<4B", 4, 0xFF, 0x30, 0x00))
        response = self._receive_payload(timeout=0.5)
        if len(response) < 16:
            raise ValueError(f"Leptrino sample reply too short ({len(response)} bytes)")
        raw = struct.unpack("<6h", response[4:16])
        return [raw[i] * self.rated_values[i] / 10000.0 for i in range(6)]


# ══════════════════════════════════════════════════════════════════════════════
# Phase-aware CSV writer
# ══════════════════════════════════════════════════════════════════════════════
def _assign_phase(sample_ts: float, events: list) -> tuple:
    """
    Binary-search the phase_events list to find the last event before sample_ts.
    Returns (phase_str, rep_int).
    events must be sorted ascending by timestamp.
    """
    phase = "idle"
    rep   = 0
    for (evt_t, evt_phase, evt_rep) in events:
        if evt_t <= sample_ts:
            phase = evt_phase
            rep   = evt_rep
        else:
            break
    return phase, rep


def save_speed_csv(speed_mps: float, phase_events: list, samples: list) -> int:
    """
    Assign phase labels to every Leptrino sample and write Data/jump_speed_X.XXmps.csv.
    Returns number of rows written.
    """
    if not samples:
        print(f"  [WARN] speed={speed_mps:.2f} m/s — no force samples collected")
        return 0

    path = DATA_DIR / f"jump_speed_{speed_mps:.2f}mps.csv"

    # reference time = first sample's perf_counter
    t0 = samples[0][0]

    # sort events (should already be sorted, but be safe)
    events_sorted = sorted(phase_events, key=lambda e: e[0])

    rows = []
    for sample in samples:
        ts, Fx, Fy, Fz, Mx, My, Mz = sample
        phase, rep = _assign_phase(ts, events_sorted)
        rows.append({
            "time_s":   round(ts - t0, 6),
            "speed_mps": speed_mps,
            "rep":      rep,
            "phase":    phase,
            "Fx": round(Fx, 4),
            "Fy": round(Fy, 4),
            "Fz": round(Fz, 4),
            "Mx": round(Mx, 4),
            "My": round(My, 4),
            "Mz": round(Mz, 4),
        })

    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        w.writeheader()
        w.writerows(rows)

    return len(rows)


# ══════════════════════════════════════════════════════════════════════════════
# Main sweep collection loop
# ══════════════════════════════════════════════════════════════════════════════
def run_sweep(teensy_ser: serial.Serial, leptrino) -> list:
    """
    Read Teensy phase markers, accumulate Leptrino samples per speed level,
    and save one CSV per level.  Returns list of completed speed_mps values.
    """
    print("\n" + "=" * 60)
    print(" SWEEP: listening for Teensy phase markers ...")
    print("=" * 60)

    current_delay  = None
    phase_events   = []        # [(perf_time, phase_str, rep), ...]
    completed      = []

    deadline = time.perf_counter() + SWEEP_TIMEOUT_S

    while time.perf_counter() < deadline:
        raw = teensy_ser.readline()
        if not raw:
            continue
        line = raw.decode("utf-8", errors="ignore").strip()
        if not line:
            continue

        recv_t = time.perf_counter()
        print(f"  [T] {line}")

        try:
            # ── #START speed_mps ──────────────────────────────────────────
            if line.startswith("#START"):
                speed_mps     = float(line.split()[1])
                current_delay = speed_mps
                phase_events  = [(recv_t, "start", 0)]
                if leptrino:
                    leptrino.drain_samples()   # discard inter-level samples
                print(f"\n  [LEVEL] Speed = {speed_mps:.2f} m/s  ▶ collecting ...")

            # ── #DOWN rep delay_ms ────────────────────────────────────────
            elif line.startswith("#DOWN"):
                parts = line.split()
                rep   = int(parts[1])
                # delay_ms = int(parts[2])  # redundant — trusts current_delay
                phase_events.append((recv_t, "compress", rep))

            # ── #HOLD rep delay_ms ────────────────────────────────────────
            elif line.startswith("#HOLD"):
                rep = int(line.split()[1])
                phase_events.append((recv_t, "hold", rep))

            # ── #UP rep delay_ms ──────────────────────────────────────────
            elif line.startswith("#UP"):
                rep = int(line.split()[1])
                phase_events.append((recv_t, "extend", rep))

            # ── #REP_END rep delay_ms ─────────────────────────────────────
            elif line.startswith("#REP_END"):
                rep = int(line.split()[1])
                phase_events.append((recv_t, "idle", rep))

            # ── #DONE speed_mps ────────────────────────────────────────────
            elif line.startswith("#DONE"):
                speed_mps = float(line.split()[1])
                if current_delay is None or abs(current_delay - speed_mps) > 0.01:
                    print(f"  [WARN] #DONE mismatch (expected {current_delay}, got {speed_mps})")

                samples = leptrino.drain_samples() if leptrino else []
                n = save_speed_csv(speed_mps, phase_events, samples)
                completed.append(speed_mps)
                print(f"  [SAVED] speed={speed_mps:.2f}m/s → {n} samples  "
                      f"({len(completed)}/10 levels done)")

                current_delay = None
                phase_events  = []

            # ── #SWEEP_COMPLETE ───────────────────────────────────────────
            elif line.strip() == "#SWEEP_COMPLETE":
                print(f"\n  [SWEEP COMPLETE] {len(completed)} levels saved.")
                break

        except (ValueError, IndexError) as exc:
            print(f"  [WARN] Could not parse '{line}': {exc}")

    return completed


# ══════════════════════════════════════════════════════════════════════════════
# Entry point
# ══════════════════════════════════════════════════════════════════════════════
def main():
    stop_event = threading.Event()
    leptrino   = None

    # ── 1. Connect Leptrino ─────────────────────────────────────────────────
    print(f"[1/3] Connecting to Leptrino on {LEP_PORT} ...")
    try:
        leptrino = LeptrinoClient(port=LEP_PORT, baud=LEP_BAUD)
        lep_thread = threading.Thread(
            target=leptrino.run_background,
            args=(stop_event,),
            daemon=True,
            name="LeptrinoSampler",
        )
        lep_thread.start()
        print(f"  Leptrino OK — sampling at {LEP_HZ} Hz in background")
    except Exception as exc:
        leptrino = None
        print(f"  [WARN] Leptrino unavailable: {exc}")
        print("  Force channels (Fx..Mz) will be ABSENT from CSV.")

    # ── 2. Connect Teensy ──────────────────────────────────────────────────
    print(f"\n[2/3] Connecting to Teensy on {TEENSY_PORT} ...")
    try:
        ser = serial.Serial(TEENSY_PORT, TEENSY_BAUD, timeout=0.5, write_timeout=2)
    except serial.SerialException as exc:
        print(f"  ERROR: {exc}")
        stop_event.set()
        if leptrino:
            leptrino.close()
        return

    try:
        with ser:
            # Let Teensy finish booting
            time.sleep(SERIAL_BOOT_DELAY_S)
            ser.reset_input_buffer()

            # ── 3. Enter closed-loop control ────────────────────────────────
            print(f"\n[3/3] Entering closed-loop control ...")
            ser.write(b"c")
            time.sleep(CLOSED_LOOP_WAIT_S)
            ser.reset_input_buffer()
            print("  Motors in closed-loop control — ready.")

            # ── 4. Start frequency sweep ────────────────────────────────────
            print("\n" + "=" * 60)
            print(" Sending 'f' → jumpFrequency() starts on Teensy")
            print(" Sweep: speed = 0.45 m/s to 1.26 m/s  ×3 reps each")
            print("=" * 60)
            ser.write(b"f")

            # ── 5. Collect data ─────────────────────────────────────────────
            completed = run_sweep(ser, leptrino)

            # ── 6. Summary ──────────────────────────────────────────────────
            print("\n" + "=" * 60)
            print(f"  Sweep finished.  {len(completed)}/10 speed levels saved.")
            for d in completed:
                p = DATA_DIR / f"jump_speed_{d:.2f}mps.csv"
                sz = p.stat().st_size if p.exists() else 0
                print(f"    speed={d:.2f}m/s → {p.name}  ({sz} bytes)")
            print("=" * 60)

    finally:
        stop_event.set()
        if leptrino:
            leptrino.close()


if __name__ == "__main__":
    main()
