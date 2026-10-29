#!/usr/bin/env python3
"""Load every bundled KGU model and exercise its control/torque path."""

from __future__ import annotations

from kaixuan_bam import load_kgu_model


MODELS = ("kgu20_c_48v", "kgu17_b_48v", "kgu11_c_48v")


def main() -> None:
    for name in MODELS:
        model = load_kgu_model(name)
        actuator = model.actuator
        current = actuator.compute_control(q_target=0.1, q=0.0, dq=0.0, dt=0.002)
        torque = actuator.compute_torque(
            current, torque_enable=True, q=0.0, dq=0.0
        )
        print(
            f"{name}: current={float(current):.6f} Aeq, "
            f"torque={float(torque):.6f} Nm, "
            f"armature={actuator.get_extra_inertia():.9f} kg.m^2"
        )


if __name__ == "__main__":
    main()
