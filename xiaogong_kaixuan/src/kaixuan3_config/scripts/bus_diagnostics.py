#!/usr/bin/env python3
"""Decode read-only bus probes; no ROS or hardware access in this module."""

WIDTH = 77
HEADER = ["sequence", "request_steady_ns", "completion_steady_ns", "slave", "kind",
          "index", "subindex", "expected_bytes", "actual_bytes", "outcome",
          "api_result_or_state", "disabled_until_reconfigure", "ring_dropped_records"]
OUTCOMES = {0: "success", -1: "request_failed", -2: "unexpected_size",
            -3: "schedule_failed", -5: "still_busy_after_1s"}
NAMES = {
    0x2070: "pn070_lost_frame_tolerance", 0x2200: "drive_temperature_raw",
    0x100A: "software_version_optional", 0x2002: "pn002_monitor_selector_not_value",
    0x2031: "current_p", 0x2032: "current_i", 0x2033: "speed_p",
    0x2034: "speed_i", 0x2035: "speed_feedforward", 0x2063: "position_p_fine",
    0x2064: "position_feedforward", 0x2065: "position_p_coarse",
    0x2066: "position_command_filter", 0x2067: "position_output_filter",
    0x2025: "pwm_off_delay",
}
SYNC_NAMES = {1: "synchronization_type", 2: "cycle_time_ns",
              0x0B: "sm_event_missed", 0x0C: "cycle_exceeded", 0x20: "sync_error"}
SCHEMA = {
    "version": 1, "columns": HEADER, "payload": "up to 64 raw bytes, padded to width 77",
    "outcomes": OUTCOMES, "max_request_starts_per_second_per_master": 10,
    "esc_counter_and_dc_nominal_period_s": 3, "temperature_nominal_period_s": 10,
    "parameter_nominal_period_s": 60, "optional_sync_counter_nominal_period_s": 30,
    "software_version_nominal_period_s": 300,
    "period_note": "lower bounds; startup, queueing and busy requests may lengthen intervals",
    "dc_note": "0x092C sign-magnitude filtered clock difference, not PDO arrival/SYNC0 phase",
    "failure_note": "failed reads are null, not zero; optional failures disable that object for this session",
    "unavailable": ["Monitor 64 uses separate monitor64 log and temporary selector writes; monitor 63 not sampled",
                    "IgH request API does not expose CoE abort code; inspect node/kernel logs",
                    "drive internal PDO-consumption timestamp and brake state are not exposed here"],
}


def decode(wire):
    if len(wire) < 3 or list(wire[:3:2]) != [1, WIDTH] or wire[1] not in (0, 1):
        raise ValueError("invalid bus probe header")
    if (len(wire) - 3) % WIDTH:
        raise ValueError("truncated bus probe packet")
    master = int(wire[1])
    result = []
    for offset in range(3, len(wire), WIDTH):
        row = wire[offset:offset + WIDTH]
        item = dict(zip(HEADER, map(int, row[:len(HEADER)])))
        if not 0 <= item["slave"] < 7 or item["kind"] not in (0, 1):
            raise ValueError("invalid probe target")
        size = item["actual_bytes"]
        if size < 0 or item["outcome"] not in OUTCOMES:
            raise ValueError("invalid probe result")
        item.update(master=master, joint=f"{'right' if master == 0 else 'left'}_joint{item['slave'] + 1}",
                    transport="esc_register" if item["kind"] == 0 else "coe_upload",
                    result=OUTCOMES[item["outcome"]], value=None, raw_hex=None)
        item["request_duration_ms"] = (item["completion_steady_ns"] - item["request_steady_ns"]) / 1e6
        index, sub = item["index"], item["subindex"]
        item["name"] = (NAMES.get(index, f"object_0x{index:04x}_{sub:02x}") if item["kind"] else
                        {0x0300: "esc_error_counters", 0x092C: "dc_system_time_difference_ns"}.get(index, "esc_raw"))
        if index in (0x1C32, 0x1C33):
            item["name"] = f"{'output' if index == 0x1C32 else 'input'}_{SYNC_NAMES.get(sub, 'unknown')}_optional"
        if item["outcome"] == 0:
            if not 0 < size <= 64 or (index != 0x100A and size != item["expected_bytes"]):
                raise ValueError("invalid successful probe size")
            raw = bytes(int(v) for v in row[len(HEADER):len(HEADER) + size])
            item["raw_hex"] = raw.hex()
            if item["kind"] == 0 and index == 0x0300:
                if size != 20:
                    raise ValueError("invalid ESC counter size")
                values = {}
                for port in range(4):
                    values[f"port{port}_invalid_frames"] = raw[2 * port]
                    values[f"port{port}_rx_errors"] = raw[2 * port + 1]
                    values[f"port{port}_forwarded_errors"] = raw[8 + port]
                    values[f"port{port}_lost_links"] = raw[16 + port]
                values.update(ecat_processing_errors=raw[12], pdi_errors=raw[13])
                item["value"] = values
            elif item["kind"] == 0 and index == 0x092C:
                if size != 4:
                    raise ValueError("invalid DC size")
                number = int.from_bytes(raw, "little")
                item["value"] = -(number & 0x7FFFFFFF) if number & 0x80000000 else number
            elif index == 0x100A:
                item["value"] = raw.rstrip(b"\x00").decode("utf-8", errors="replace")
            else:
                item["value"] = int.from_bytes(raw, "little")
        result.append(item)
    return result


class BusEvidence:
    def __init__(self, emit):
        self.emit = emit
        self.previous = {}
        self.last_sequence = {}
        self.ring_drops = {}
        self.gaps = 0

    def receive(self, wire):
        for item in decode(wire):
            master, seq = item["master"], item["sequence"]
            previous_seq = self.last_sequence.get(master)
            if previous_seq is not None and seq != previous_seq + 1:
                self.gaps += 1
                self.emit("events", "bus_probe_gap", {"master": master, "previous": previous_seq, "current": seq})
            self.last_sequence[master] = seq
            dropped = item["ring_dropped_records"]
            if dropped != self.ring_drops.get(master, 0):
                self.emit("events", "bus_probe_ring_drops", {"master": master, "total": dropped})
            self.ring_drops[master] = dropped
            key = (master, item["slave"], item["kind"], item["index"], item["subindex"])
            previous = self.previous.get(key)
            if previous and item["value"] is not None:
                item["previous_success_steady_ns"] = previous["completion_steady_ns"]
                item["elapsed_since_previous_success_s"] = (item["completion_steady_ns"] - previous["completion_steady_ns"]) / 1e9
                if isinstance(item["value"], dict):
                    item["counter_changes"] = {k: {"before": previous["value"][k], "after": v,
                        "direction": "increased" if v > previous["value"][k] else "decreased_reset_or_wrap"}
                        for k, v in item["value"].items() if v != previous["value"][k]}
                    if item["counter_changes"]:
                        self.emit("events", "esc_counter_change", item)
                elif item["kind"] == 1 and item["value"] != previous["value"] and item["index"] != 0x2200:
                    self.emit("events", "drive_object_change", {**item, "previous_value": previous["value"]})
            if item["outcome"] != 0:
                self.emit("events", "bus_probe_failure", item)
            else:
                self.previous[key] = item.copy()
            self.emit("bus", "bus_probe", item)
