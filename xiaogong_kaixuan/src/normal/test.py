#!/usr/bin/env python3

import argparse
import os
from typing import Optional


STATE_DIR = "/tmp/ethercat_op_csp"
MASTER_IDS = (0, 1)
SLAVES_PER_MASTER = 7


def hold_log_path(master_id: int, state_dir: str) -> str:
	return os.path.join(state_dir, f"master{master_id}.log")


def parse_int(value: str) -> int:
	return int(value, 0)


def decode_cia402_state(status_word: int) -> str:
	masked_6f = status_word & 0x006F
	masked_4f = status_word & 0x004F

	if masked_4f == 0x0040:
		return "Switch on disabled"
	if masked_6f == 0x0021:
		return "Ready to switch on"
	if masked_6f == 0x0023:
		return "Switched on"
	if masked_6f == 0x0027:
		return "Operation enabled"
	if masked_6f == 0x0007:
		return "Quick stop active"
	if masked_4f == 0x000F:
		return "Fault reaction active"
	if masked_4f == 0x0008:
		return "Fault"
	if masked_4f == 0x0000:
		return "Not ready to switch on"
	return "Unknown"


def is_operation_enabled(status_word: int) -> bool:
	return (status_word & 0x006F) == 0x0027


def read_latest_snapshot(log_path: str, master_id: int) -> Optional[tuple[int, dict[int, tuple[int, int, int]]]]:
	try:
		with open(log_path, "r", encoding="utf-8", errors="replace") as handle:
			lines = handle.readlines()
	except OSError:
		return None

	prefix = f"master={master_id} snapshot="
	for index in range(len(lines) - 1, -1, -1):
		header = lines[index].strip()
		if not header.startswith(prefix):
			continue

		try:
			snapshot = int(header.split("snapshot=", 1)[1].split()[0])
		except (IndexError, ValueError):
			continue

		axes: dict[int, tuple[int, int, int]] = {}
		for raw_line in lines[index + 1:]:
			line = raw_line.strip()
			if not line or line == "axis slave offset mode status position":
				continue
			if line.startswith("master="):
				break

			parts = line.split()
			if len(parts) != 6 or not parts[0].isdigit():
				continue

			try:
				slave_pos = int(parts[1])
				mode_display = int(parts[3])
				status_word = int(parts[4])
				actual_position = int(parts[5])
			except ValueError:
				continue

			axes[slave_pos] = (mode_display, status_word, actual_position)

		if len(axes) >= SLAVES_PER_MASTER:
			return snapshot, axes

	return None


def collect_helper_rows(state_dir: str) -> list[tuple[int, int, int, int, int]]:
	rows: list[tuple[int, int, int, int, int]] = []
	for master_id in MASTER_IDS:
		snapshot = read_latest_snapshot(hold_log_path(master_id, state_dir), master_id)
		if snapshot is None:
			raise RuntimeError(
				f"No valid snapshot found for master{master_id}. Expected log: {hold_log_path(master_id, state_dir)}"
			)

		_snapshot_index, axes = snapshot
		for slave_pos in range(SLAVES_PER_MASTER):
			axis_data = axes.get(slave_pos)
			if axis_data is None:
				raise RuntimeError(f"master{master_id} snapshot is missing slave {slave_pos}")
			mode_display, status_word, actual_position = axis_data
			rows.append((master_id, slave_pos, mode_display, status_word, actual_position))

	return rows


def print_decoded_rows(rows: list[tuple[int, int, int, int, int]]) -> None:
	enabled_count = 0
	print("master slave 6061   6041    enabled state                     6064")
	print("------ ----- ---- -------- ------- ------------------------- ----------")
	for master_id, slave_pos, mode_display, status_word, actual_position in rows:
		enabled = is_operation_enabled(status_word)
		if enabled:
			enabled_count += 1
		state = decode_cia402_state(status_word)
		print(
			f"{master_id:>6} {slave_pos:>5} {mode_display:>4} 0x{status_word:04x} "
			f"{('YES' if enabled else 'NO'):>7} {state:<25} {actual_position:>10}"
		)

	print(f"\noperation enabled axes: {enabled_count}/{len(rows)}")


def print_decoded_statuswords(status_words: list[int]) -> None:
	print("6041    enabled state")
	print("-------- ------- -------------------------")
	for status_word in status_words:
		enabled = is_operation_enabled(status_word)
		state = decode_cia402_state(status_word)
		print(f"0x{status_word:04x} {('YES' if enabled else 'NO'):>7} {state}")


def main() -> None:
	parser = argparse.ArgumentParser(description="Decode CiA402 statusword 0x6041 from helper logs or manual values")
	parser.add_argument(
		"statusword",
		nargs="*",
		type=parse_int,
		help="Optional 0x6041 value(s), supports decimal or hex such as 0x27",
	)
	parser.add_argument(
		"--state-dir",
		default=STATE_DIR,
		help=f"Directory containing master logs (default: {STATE_DIR})",
	)
	args = parser.parse_args()

	if args.statusword:
		print_decoded_statuswords(args.statusword)
		return

	rows = collect_helper_rows(args.state_dir)
	print_decoded_rows(rows)


if __name__ == "__main__":
	main()
