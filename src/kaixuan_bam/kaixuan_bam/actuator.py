"""Output-side equivalent model for a KGU integrated joint module.

This is deliberately an identification-friendly model.  The JSON parameter
files contain the effective electrical constants, position-loop gains and
friction values, so the same class can represent every KGU size and ratio.
"""

from __future__ import annotations

from bam.actuator import ArrayLike, DCMotorActuator
from bam.parameter import Parameter
from bam.testbench import Testbench


class KguActuator(DCMotorActuator):
    """Position-controlled KGU represented at the gearbox output shaft.

    ``kt`` and ``R`` are *effective output-side* constants.  They already
    include the gearbox, so MuJoCo actuators using this model must use
    ``gear=1``.
    """

    def __init__(self, testbench_class: Testbench):
        super().__init__(testbench_class, vin=48.0, kp=1.0)

    def initialize(self) -> None:
        # Output-side effective DC constants.
        self.model.kt = Parameter(10.0, 0.1, 100.0)
        self.model.R = Parameter(3.0, 0.01, 100.0)

        # Reflected rotor/gear inertia added to the MuJoCo DOF.
        self.model.armature = Parameter(0.001, 0.0, 5.0)

        # Output position-loop model.  Torque command is
        # kp_joint * position_error - kd_joint * joint_velocity.
        self.model.kp_joint = Parameter(800.0, 0.0, 20000.0)
        self.model.kd_joint = Parameter(10.0, 0.0, 2000.0)
        self.model.current_limit = Parameter(10.0, 0.0, 200.0)

        # KGU friction can be tens of Nm for the larger units.  BAM's generic
        # defaults target small servos, so widen the identification bounds.
        self.model.max_friction_base = 100.0
        self.model.max_load_friction = 3.0
        self.model.max_viscous_friction = 100.0

    def control_unit(self) -> str:
        return "effective output amps"

    def compute_control(
        self, q_target: ArrayLike, q: ArrayLike, dq: ArrayLike, dt: float
    ) -> ArrayLike:
        del dt

        kt = self.model.kt.value
        resistance = self.model.R.value
        torque_request = (
            self.model.kp_joint.value * (q_target - q)
            - self.model.kd_joint.value * dq
        )
        current = torque_request / kt

        # Back-EMF/48 V torque-speed envelope.
        current_low = (-self.vin - kt * dq) / resistance
        current_high = (self.vin - kt * dq) / resistance
        current = self.backend.clamp(current, current_low, current_high)
        current = self.backend.clamp(
            current,
            -self.model.current_limit.value,
            self.model.current_limit.value,
        )

        # Used by BAM's optional shared-bus voltage-drop calculation.
        duty = (kt * dq + resistance * current) / self.vin
        self.duty_cycle = self.backend.clamp(duty, -1.0, 1.0)
        return current

    def compute_torque(
        self,
        control: ArrayLike | None,
        torque_enable: bool,
        q: ArrayLike,
        dq: ArrayLike,
    ) -> ArrayLike:
        del q, dq
        if control is None:
            return 0.0
        return self.model.kt.value * control * torque_enable

    def get_extra_inertia(self) -> float:
        return self.model.armature.value
