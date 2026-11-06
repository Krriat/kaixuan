"""Saved scientific figures for the Cartesian trajectory experiment."""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


COLORS = {"joint_pd": "#d97706", "computed_torque_pd": "#16836b", "tvlqr": "#2563eb"}
LABELS = {"joint_pd": "Independent joint PD + reference feedforward",
          "computed_torque_pd": "Computed torque + PD", "tvlqr": "Time-varying LQR"}


def shade(ax, settings):
    pulse = settings["disturbance"]
    ax.axvspan(pulse["start_s"], pulse["start_s"]+pulse["duration_s"], color="#ef4444", alpha=.13)
    ax.grid(alpha=.2)


def plot_results(output, runs, reference, design, settings, sweep, strong_runs):
    fig = plt.figure(figsize=(13, 9), constrained_layout=True)
    ax = fig.add_subplot(2, 2, 1, projection="3d")
    p0 = reference.start_position
    desired = (reference.positions-p0)*1000
    ax.plot(*desired.T, "k--", lw=2, label="Desired line")
    ax.set(xlabel="World X from start (mm)", ylabel="World Y from start (mm)", zlabel="World Z from start (mm)")
    axes = [fig.add_subplot(2, 2, i) for i in (2, 3, 4)]
    for name in COLORS:
        _, trace = runs[f"push_{name}"]
        p = (trace["ee_position_m"]-p0)*1000
        error = (trace["ee_position_m"]-trace["desired_position_m"])*1000
        norm = np.linalg.norm(error, axis=1)
        t = trace["time_s"]
        label, color = LABELS[name], COLORS[name]
        ax.plot(*p[::5].T, color=color, label=label)
        axes[0].plot(t, norm, color=color, label=label)
        axes[1].plot(t, error[:, 1], color=color, label=label)
        axes[2].plot(t, np.rad2deg(trace["orientation_error_rad"]), color=color, label=label)
    axes[0].set(ylabel="Position error norm (mm)", xlabel="Time (s)")
    axes[1].set(ylabel="World Y tracking error (mm)", xlabel="Time (s)")
    axes[2].set(ylabel="Tool orientation error (deg)", xlabel="Time (s)")
    for axis in axes:
        shade(axis, settings)
    _, fast = strong_runs["push"]
    fast_error = 1000*(fast["ee_position_m"]-fast["desired_position_m"])
    fast_label = f"Computed torque PD, w={settings['strong_pd_frequency_rad_s']:g} rad/s"
    ax.plot(*((fast["ee_position_m"]-p0)*1000)[::5].T, color="#7c3aed", ls=":")
    for axis, values in zip(axes, (np.linalg.norm(fast_error, axis=1), fast_error[:, 1],
                                  np.rad2deg(fast["orientation_error_rad"]))):
        axis.plot(fast["time_s"], values, color="#7c3aed", ls=":", lw=1.8, label=fast_label)
    axes[0].axhline(1000*settings["acceptance"]["recovery_band_m"], color="gray", ls=":", label="Recovery band")
    axes[0].legend(fontsize=8)
    fig.suptitle("Seven active joints: Cartesian line + a prescribed end-effector force pulse")
    fig.savefig(output / "tracking.png", dpi=160)
    plt.close(fig)

    nrows = int(np.ceil(len(settings["scenarios"])/2))
    fig, axes = plt.subplots(nrows, 2, figsize=(13, 3.8*nrows), constrained_layout=True)
    for scenario, axis in zip(settings["scenarios"], axes.flat):
        for name in COLORS:
            metrics, trace = runs[f"{scenario}_{name}"]
            error = 1000*np.linalg.norm(trace["ee_position_m"]-trace["desired_position_m"], axis=1)
            axis.plot(trace["time_s"], error, color=COLORS[name], label=LABELS[name])
            if metrics["status"] != "completed":
                axis.plot(trace["time_s"][-1], error[-1], "rx")
        metric, fast = strong_runs[scenario]
        error = 1000*np.linalg.norm(fast["ee_position_m"]-fast["desired_position_m"], axis=1)
        axis.plot(fast["time_s"], error, color="#7c3aed", ls=":", label=fast_label)
        if metric["status"] != "completed":
            axis.plot(fast["time_s"][-1], error[-1], "rx", ms=8)
        axis.set(title=scenario.replace("_", " "), ylabel="Position error norm (mm)", xlabel="Time (s)")
        shade(axis, settings)
    for axis in list(axes.flat)[len(settings["scenarios"]):]:
        axis.set_visible(False)
    axes[0, 0].legend(fontsize=8)
    fig.suptitle("Identical references and force pulses; delay/filter affect only the physical plant")
    fig.savefig(output / "scenarios.png", dpi=160)
    plt.close(fig)

    fig, axes = plt.subplots(7, 2, figsize=(13, 16), sharex=True, constrained_layout=True)
    for j in range(7):
        axes[j, 0].plot(reference.times, np.rad2deg(reference.q[:, j]), "k--", lw=1.5, label="Reference")
        for name in COLORS:
            _, trace = runs[f"push_{name}"]
            axes[j, 0].plot(trace["time_s"], np.rad2deg(trace["q_rad"][:, j]), color=COLORS[name], label=LABELS[name])
            axes[j, 1].plot(trace["time_s"][:-1], trace["applied_nm"][:, j], color=COLORS[name], label=LABELS[name])
        axes[j, 0].set_ylabel(f"J{j+1} angle (deg)")
        axes[j, 1].set_ylabel(f"J{j+1} motor torque (Nm)")
        for axis in axes[j]:
            shade(axis, settings)
    axes[0, 0].legend(fontsize=7)
    axes[-1, 0].set_xlabel("Time (s)")
    axes[-1, 1].set_xlabel("Time (s)")
    fig.suptitle("Push scenario: joint motion and TOTAL applied motor torque")
    fig.savefig(output / "joints_and_torques.png", dpi=140)
    plt.close(fig)

    fig, axes = plt.subplots(1, 2, figsize=(12, 5), constrained_layout=True)
    k = min(int(settings["disturbance"]["start_s"]/settings["control_period_s"]), len(design.gains)-1)
    for axis, gain, label in zip(axes, (design.gains[k, :, :7], design.gains[k, :, 7:]),
                                 ("Position feedback (Nm/rad)", "Velocity feedback (Nm/(rad/s))")):
        vmax = float(np.max(np.abs(gain)))
        img = axis.imshow(gain, cmap="RdBu_r", vmin=-vmax, vmax=vmax)
        axis.set(title=label, xlabel="Measured joint error", ylabel="Commanded motor torque")
        axis.set_xticks(range(7), range(1, 8))
        axis.set_yticks(range(7), range(1, 8))
        fig.colorbar(img, ax=axis, shrink=.8)
    fig.suptitle("Coupled TVLQR gain at the start of the push (off-diagonal entries coordinate joints)")
    fig.savefig(output / "feedback_gains.png", dpi=150)
    plt.close(fig)

    fig, axes = plt.subplots(1, 2, figsize=(12, 5), constrained_layout=True)
    for name in ("joint_pd", "computed_torque_pd"):
        cases = sorted((s for s in sweep.values() if s["controller"] == name and s["scenario_name"] == "push"),
                       key=lambda s: s["pd_frequency_rad_s"])
        good = [s for s in cases if s["status"] == "completed"]
        for axis, metric in zip(axes, ("peak_position_error_mm", "common_sampled_cost")):
            axis.plot([s["pd_frequency_rad_s"] for s in good], [s[metric] for s in good],
                "o-", color=COLORS[name], label=LABELS[name])
            axis.axhline(runs["push_tvlqr"][0][metric], color=COLORS["tvlqr"], ls="--", alpha=.7)
            axis.grid(alpha=.2)
            axis.set_xlabel("PD natural frequency (rad/s); damping ratio = 1")
            for stopped in (s for s in cases if s["status"] != "completed"):
                axis.plot(stopped["pd_frequency_rad_s"], 0, "rx", ms=8)
    axes[0].set_ylabel("Peak position error (mm)")
    axes[1].set_ylabel("Common sampled quadratic cost (complete runs only)")
    axes[0].legend(fontsize=8)
    fig.suptitle("PD gain sensitivity; blue dashed = TVLQR; red x at zero = guard stop (no comparable full cost)")
    fig.savefig(output / "pd_gain_sweep.png", dpi=150)
    plt.close(fig)
