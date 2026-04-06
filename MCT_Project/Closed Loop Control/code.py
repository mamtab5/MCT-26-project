import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from scipy.linalg import solve_continuous_are
from pathlib import Path

# ============================================================
# Part 3 Code: Planar Quadrotor Closed-Loop Control (LQR)
# ============================================================

np.set_printoptions(precision=4, suppress=True)
plt.style.use("seaborn-v0_8-whitegrid")


# ------------------------------------------------------------
# 1. Nominal physical parameters
# ------------------------------------------------------------
params = {
    "m": 0.50,      # mass [kg]
    "I": 0.0023,    # pitch inertia [kg m^2]
    "l": 0.25,      # half-arm length [m]
    "g": 9.81,      # gravity [m/s^2]
    "fmax": 10.0    # max thrust per rotor [N]
}


# ------------------------------------------------------------
# 2. Linearized hover model
# State: [x, x_dot, z, z_dot, theta, theta_dot]^T
# Input deviation: [delta_u1, delta_u2]^T
# ------------------------------------------------------------
def get_linear_matrices(m, I, l, g):
    A = np.array([
        [0, 1, 0, 0, 0, 0],
        [0, 0, 0, 0, -g, 0],
        [0, 0, 0, 1, 0, 0],
        [0, 0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 1],
        [0, 0, 0, 0, 0, 0]
    ], dtype=float)

    B = np.array([
        [0,   0],
        [0,   0],
        [0,   0],
        [1/m, 0],
        [0,   0],
        [0,   l/I]
    ], dtype=float)

    C = np.array([
        [1, 0, 0, 0, 0, 0],  # x
        [0, 0, 1, 0, 0, 0],  # z
        [0, 0, 0, 0, 1, 0]   # theta
    ], dtype=float)

    D = np.zeros((3, 2), dtype=float)

    return A, B, C, D


# ------------------------------------------------------------
# 3. LQR design
# ------------------------------------------------------------
def lqr(A, B, Q, R):
    P = solve_continuous_are(A, B, Q, R)
    K = np.linalg.solve(R, B.T @ P)
    eig_cl = np.linalg.eigvals(A - B @ K)
    return K, P, eig_cl


# ------------------------------------------------------------
# 4. Reference signals
# ------------------------------------------------------------
def ref_hover_origin(t):
    return np.array([0.0, 0.0], dtype=float)  # [x_ref, z_ref]

def ref_step(t, t_step=2.0, x_cmd=1.0, z_cmd=1.0):
    if t < t_step:
        return np.array([0.0, 0.0], dtype=float)
    return np.array([x_cmd, z_cmd], dtype=float)


# ------------------------------------------------------------
# 5. Error state and controller
# e = [x-x_ref, x_dot, z-z_ref, z_dot, theta, theta_dot]^T
# delta_u = -K e
# u1 = mg + delta_u1, u2 = delta_u2
# ------------------------------------------------------------
def error_state(X, ref):
    return np.array([
        X[0] - ref[0],  # x error
        X[1],           # x_dot
        X[2] - ref[1],  # z error
        X[3],           # z_dot
        X[4],           # theta
        X[5]            # theta_dot
    ], dtype=float)

def controller(X, ref, K, params, saturate=False):
    m = params["m"]
    g = params["g"]
    fmax = params["fmax"]

    e = error_state(X, ref)
    delta_u = -K @ e

    u1 = m * g + delta_u[0]
    u2 = delta_u[1]

    # Recover rotor thrusts
    f1 = 0.5 * (u1 - u2)
    f2 = 0.5 * (u1 + u2)

    # Optional saturation on rotor thrusts
    if saturate:
        f1 = np.clip(f1, 0.0, fmax)
        f2 = np.clip(f2, 0.0, fmax)
        u1 = f1 + f2
        u2 = f2 - f1
        delta_u = np.array([u1 - m * g, u2], dtype=float)

    return e, delta_u, u1, u2, f1, f2


# ------------------------------------------------------------
# 6. Dynamics
# ------------------------------------------------------------
def dyn_linear_open(t, X, params):
    g = params["g"]
    return np.array([
        X[1],
        -g * X[4],
        X[3],
        0.0,
        X[5],
        0.0
    ], dtype=float)

def dyn_linear_closed(t, X, params, K, ref_fun):
    m = params["m"]
    I = params["I"]
    l = params["l"]
    g = params["g"]

    ref = ref_fun(t)
    _, delta_u, _, _, _, _ = controller(X, ref, K, params, saturate=False)
    delta_u1, delta_u2 = delta_u

    return np.array([
        X[1],
        -g * X[4],
        X[3],
        delta_u1 / m,
        X[5],
        (l / I) * delta_u2
    ], dtype=float)

def dyn_nonlinear_open(t, X, params):
    m = params["m"]
    I = params["I"]
    l = params["l"]
    g = params["g"]

    u1 = m * g
    u2 = 0.0
    theta = X[4]

    return np.array([
        X[1],
        -(u1 / m) * np.sin(theta),
        X[3],
        (u1 / m) * np.cos(theta) - g,
        X[5],
        (l / I) * u2
    ], dtype=float)

def dyn_nonlinear_closed(t, X, params, K, ref_fun, saturate=True):
    m = params["m"]
    I = params["I"]
    l = params["l"]
    g = params["g"]

    ref = ref_fun(t)
    _, _, u1, u2, _, _ = controller(X, ref, K, params, saturate=saturate)
    theta = X[4]

    return np.array([
        X[1],
        -(u1 / m) * np.sin(theta),
        X[3],
        (u1 / m) * np.cos(theta) - g,
        X[5],
        (l / I) * u2
    ], dtype=float)


# ------------------------------------------------------------
# 7. Simulation helper
# ------------------------------------------------------------
def simulate(rhs, x0, t_final, dt=0.01):
    t_eval = np.arange(0.0, t_final + dt, dt)
    sol = solve_ivp(
        rhs,
        (0.0, t_final),
        x0,
        t_eval=t_eval,
        method="RK45",
        rtol=1e-8,
        atol=1e-9
    )
    if not sol.success:
        raise RuntimeError(f"Simulation failed: {sol.message}")
    return sol.t, sol.y


# ------------------------------------------------------------
# 8. Control reconstruction for plotting
# ------------------------------------------------------------
def get_control_history(t, X, params, K, ref_fun, saturate=False):
    N = len(t)
    E = np.zeros((6, N))
    DU = np.zeros((2, N))
    U = np.zeros((4, N))   # [u1, u2, f1, f2]
    REF = np.zeros((2, N)) # [x_ref, z_ref]

    for i, ti in enumerate(t):
        ref = ref_fun(ti)
        e, delta_u, u1, u2, f1, f2 = controller(X[:, i], ref, K, params, saturate=saturate)

        E[:, i] = e
        DU[:, i] = delta_u
        U[:, i] = np.array([u1, u2, f1, f2], dtype=float)
        REF[:, i] = ref

    return E, DU, U, REF


# ------------------------------------------------------------
# 9. Performance metrics
# ------------------------------------------------------------
def settling_time(t, y, y_ref, tol):
    err = np.abs(y - y_ref)
    idx = np.where(err > tol)[0]

    if len(idx) == 0:
        return 0.0
    if idx[-1] == len(t) - 1:
        return np.nan
    return t[idx[-1] + 1]

def overshoot_percent(y, y0, yref):
    amp = yref - y0
    if np.isclose(amp, 0.0):
        return 0.0

    if amp > 0:
        peak = np.max(y)
        os = (peak - yref) / amp * 100.0
    else:
        peak = np.min(y)
        os = (yref - peak) / (-amp) * 100.0

    return max(0.0, os)

def compute_hover_metrics(t, X, U):
    metrics = {}
    metrics["settling_x_s"] = settling_time(t, X[0], 0.0, 0.02)
    metrics["settling_z_s"] = settling_time(t, X[2], 0.0, 0.02)
    metrics["settling_theta_s"] = settling_time(t, np.rad2deg(X[4]), 0.0, 1.0)

    metrics["peak_abs_theta_deg"] = np.max(np.abs(np.rad2deg(X[4])))
    metrics["peak_abs_xdot_mps"] = np.max(np.abs(X[1]))
    metrics["peak_abs_zdot_mps"] = np.max(np.abs(X[3]))
    metrics["peak_u1_N"] = np.max(U[0])
    metrics["peak_abs_u2_N"] = np.max(np.abs(U[1]))
    metrics["peak_f1_N"] = np.max(U[2])
    metrics["peak_f2_N"] = np.max(U[3])
    metrics["min_f1_N"] = np.min(U[2])
    metrics["min_f2_N"] = np.min(U[3])

    return metrics

def compute_step_metrics(t, X, U, t_step=2.0, x_cmd=1.0, z_cmd=1.0):
    metrics = {}
    mask = t >= t_step

    tp = t[mask] - t_step
    xp = X[0, mask]
    zp = X[2, mask]
    thetap_deg = np.rad2deg(X[4, mask])

    x0 = xp[0]
    z0 = zp[0]

    tol_x = max(0.02 * abs(x_cmd), 0.02)
    tol_z = max(0.02 * abs(z_cmd), 0.02)

    metrics["settling_x_s_after_step"] = settling_time(tp, xp, x_cmd, tol_x)
    metrics["settling_z_s_after_step"] = settling_time(tp, zp, z_cmd, tol_z)
    metrics["overshoot_x_percent"] = overshoot_percent(xp, x0, x_cmd)
    metrics["overshoot_z_percent"] = overshoot_percent(zp, z0, z_cmd)
    metrics["peak_abs_theta_deg_after_step"] = np.max(np.abs(thetap_deg))
    metrics["peak_u1_N_after_step"] = np.max(U[0, mask])
    metrics["peak_abs_u2_N_after_step"] = np.max(np.abs(U[1, mask]))
    metrics["peak_f1_N_after_step"] = np.max(U[2, mask])
    metrics["peak_f2_N_after_step"] = np.max(U[3, mask])

    return metrics


# ------------------------------------------------------------
# 10. Plotting utilities
# ------------------------------------------------------------
def plot_closed_loop_poles(eigs, savepath):
    fig, ax = plt.subplots(figsize=(6, 5))
    ax.scatter(np.real(eigs), np.imag(eigs), s=70, color="crimson", label="Closed-loop poles")

    for lam in eigs:
        ax.annotate(
            f"{lam.real:.2f}{lam.imag:+.2f}j",
            (lam.real, lam.imag),
            textcoords="offset points",
            xytext=(5, 5),
            fontsize=8
        )

    ax.axvline(0, color="k", linewidth=1)
    ax.axhline(0, color="k", linewidth=1)
    ax.set_xlabel("Real part")
    ax.set_ylabel("Imaginary part")
    ax.set_title("Closed-Loop Poles of $A-BK$")
    ax.grid(True, linestyle="--", alpha=0.6)
    ax.legend()
    fig.tight_layout()
    fig.savefig(savepath, dpi=300)
    plt.close(fig)

def plot_compare_open_closed_linear(t, X_open, X_closed, savepath):
    fig, axes = plt.subplots(3, 1, figsize=(8, 9), sharex=True)

    y_open = [X_open[0], X_open[2], np.rad2deg(X_open[4])]
    y_closed = [X_closed[0], X_closed[2], np.rad2deg(X_closed[4])]
    labels = ["Horizontal Position $x$ (m)",
              "Altitude $z$ (m)",
              "Pitch Angle $\\theta$ (deg)"]

    for ax, yo, yc, lab in zip(axes, y_open, y_closed, labels):
        ax.plot(t, yo, label="Open-loop", linewidth=2, color="tab:red")
        ax.plot(t, yc, label="Closed-loop (LQR)", linewidth=2, color="tab:blue")
        ax.axhline(0.0, color="k", linestyle="--", linewidth=1)
        ax.set_ylabel(lab)
        ax.grid(True, linestyle="--", alpha=0.6)

    axes[0].legend(loc="best")
    axes[-1].set_xlabel("Time (s)")
    fig.suptitle("Linear Model: Open-Loop vs Closed-Loop Response", fontsize=14)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(savepath, dpi=300)
    plt.close(fig)

def plot_states(t, X, REF, title, savepath, t_step=None):
    fig, axes = plt.subplots(3, 2, figsize=(12, 10), sharex=True)

    state_series = [
        X[0], X[1], X[2], X[3], np.rad2deg(X[4]), np.rad2deg(X[5])
    ]
    ref_series = [
        REF[0],
        np.zeros_like(t),
        REF[1],
        np.zeros_like(t),
        np.zeros_like(t),
        np.zeros_like(t)
    ]
    labels = [
        "Horizontal Position $x$ (m)",
        "Horizontal Velocity $\\dot{x}$ (m/s)",
        "Altitude $z$ (m)",
        "Vertical Velocity $\\dot{z}$ (m/s)",
        "Pitch Angle $\\theta$ (deg)",
        "Pitch Rate $\\dot{\\theta}$ (deg/s)"
    ]

    for ax, y, r, lab in zip(axes.flatten(), state_series, ref_series, labels):
        ax.plot(t, y, linewidth=2, label="State")
        ax.plot(t, r, "--", linewidth=1.5, label="Reference")
        if t_step is not None:
            ax.axvline(t_step, color="k", linestyle=":", linewidth=1.5)
        ax.set_ylabel(lab)
        ax.grid(True, linestyle="--", alpha=0.6)

    axes[0, 0].legend(loc="best")
    axes[-1, 0].set_xlabel("Time (s)")
    axes[-1, 1].set_xlabel("Time (s)")
    fig.suptitle(title, fontsize=14)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(savepath, dpi=300)
    plt.close(fig)

def plot_controls(t, U, params, title, savepath, t_step=None):
    fig, axes = plt.subplots(2, 2, figsize=(11, 8), sharex=True)

    m = params["m"]
    g = params["g"]
    hover_u1 = m * g
    hover_f = hover_u1 / 2.0

    control_series = [U[0], U[1], U[2], U[3]]
    ref_series = [
        hover_u1 * np.ones_like(t),
        np.zeros_like(t),
        hover_f * np.ones_like(t),
        hover_f * np.ones_like(t)
    ]
    labels = [
        "Total Thrust $u_1$ (N)",
        "Differential Thrust $u_2$ (N)",
        "Left Rotor Thrust $f_1$ (N)",
        "Right Rotor Thrust $f_2$ (N)"
    ]

    for ax, y, r, lab in zip(axes.flatten(), control_series, ref_series, labels):
        ax.plot(t, y, linewidth=2, label="Control")
        ax.plot(t, r, "--", linewidth=1.5, label="Hover value")
        if t_step is not None:
            ax.axvline(t_step, color="k", linestyle=":", linewidth=1.5)
        ax.set_ylabel(lab)
        ax.grid(True, linestyle="--", alpha=0.6)

    axes[0, 0].legend(loc="best")
    axes[-1, 0].set_xlabel("Time (s)")
    axes[-1, 1].set_xlabel("Time (s)")
    fig.suptitle(title, fontsize=14)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(savepath, dpi=300)
    plt.close(fig)

def plot_xz_path(X, REF, title, savepath):
    fig, ax = plt.subplots(figsize=(7, 6))
    ax.plot(X[0], X[2], linewidth=2.5, label="Trajectory")
    ax.plot(X[0, 0], X[2, 0], "go", markersize=8, label="Start")
    ax.plot(REF[0, -1], REF[1, -1], "r*", markersize=14, label="Final reference")
    ax.set_xlabel("Horizontal Position $x$ (m)")
    ax.set_ylabel("Altitude $z$ (m)")
    ax.set_title(title)
    ax.grid(True, linestyle="--", alpha=0.6)
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(savepath, dpi=300)
    plt.close(fig)


# ------------------------------------------------------------
# 11. Save summary file
# ------------------------------------------------------------
def array_to_str(arr):
    return np.array2string(arr, precision=4, suppress_small=True)

def save_summary(filepath, A, B, Q, R, K, P, eigs, hover_metrics, step_metrics, params):
    lines = []
    lines.append("===================================================")
    lines.append("PART 3 SUMMARY: PLANAR QUADROTOR CLOSED-LOOP CONTROL")
    lines.append("===================================================\n")

    lines.append("Nominal parameters:")
    for k, v in params.items():
        lines.append(f"  {k} = {v}")
    lines.append("")

    lines.append("A matrix:")
    lines.append(array_to_str(A))
    lines.append("")

    lines.append("B matrix:")
    lines.append(array_to_str(B))
    lines.append("")

    lines.append("Q matrix:")
    lines.append(array_to_str(Q))
    lines.append("")

    lines.append("R matrix:")
    lines.append(array_to_str(R))
    lines.append("")

    lines.append("LQR gain K:")
    lines.append(array_to_str(K))
    lines.append("")

    lines.append("Riccati solution P:")
    lines.append(array_to_str(P))
    lines.append("")

    lines.append("Closed-loop eigenvalues of A-BK:")
    lines.append(array_to_str(eigs))
    lines.append("")

    lines.append("Hover stabilization metrics (nonlinear plant):")
    for k, v in hover_metrics.items():
        if isinstance(v, float) and np.isnan(v):
            lines.append(f"  {k}: not settled within simulation horizon")
        else:
            lines.append(f"  {k}: {v:.4f}")
    lines.append("")

    lines.append("Setpoint tracking metrics (nonlinear plant):")
    for k, v in step_metrics.items():
        if isinstance(v, float) and np.isnan(v):
            lines.append(f"  {k}: not settled within simulation horizon")
        else:
            lines.append(f"  {k}: {v:.4f}")
    lines.append("")

    filepath.write_text("\n".join(lines))


# ------------------------------------------------------------
# 12. Main execution
# ------------------------------------------------------------
def main():
    figs_dir = Path("figures_part3")
    results_dir = Path("results_part3")
    figs_dir.mkdir(exist_ok=True)
    results_dir.mkdir(exist_ok=True)

    m = params["m"]
    I = params["I"]
    l = params["l"]
    g = params["g"]

    # Build linear model
    A, B, C, D = get_linear_matrices(m, I, l, g)

    # --------------------------------------------------------
    # LQR tuning
    # --------------------------------------------------------
    # Balanced default tuning
    Q = np.diag([80.0, 15.0, 120.0, 20.0, 600.0, 40.0])
    R = np.diag([1.0, 2.0])

    # If response is too aggressive, try uncommenting:
    # R = np.diag([2.0, 5.0])

    K, P, eig_cl = lqr(A, B, Q, R)

    print("\nA =\n", A)
    print("\nB =\n", B)
    print("\nQ =\n", Q)
    print("\nR =\n", R)
    print("\nLQR gain K =\n", K)
    print("\nClosed-loop eigenvalues of A-BK =\n", eig_cl)

    # Pole plot
    plot_closed_loop_poles(eig_cl, figs_dir / "fig00_closed_loop_poles.png")

    # --------------------------------------------------------
    # Scenario 1: Linear open-loop vs closed-loop
    # --------------------------------------------------------
    x0_lin = np.array([
        0.40,                 # x
        0.15,                 # x_dot
        -0.25,                # z
        0.10,                 # z_dot
        np.deg2rad(6.0),      # theta
        np.deg2rad(-4.0)      # theta_dot
    ], dtype=float)

    t_lin_ol, X_lin_ol = simulate(
        lambda t, x: dyn_linear_open(t, x, params),
        x0_lin,
        t_final=6.0,
        dt=0.01
    )

    t_lin_cl, X_lin_cl = simulate(
        lambda t, x: dyn_linear_closed(t, x, params, K, ref_hover_origin),
        x0_lin,
        t_final=6.0,
        dt=0.01
    )

    plot_compare_open_closed_linear(
        t_lin_ol,
        X_lin_ol,
        X_lin_cl,
        figs_dir / "fig01_linear_open_vs_closed.png"
    )

    # --------------------------------------------------------
    # Scenario 2: Nonlinear hover stabilization
    # --------------------------------------------------------
    x0_hover = np.array([
        0.40,                 # x
        0.15,                 # x_dot
        -0.25,                # z
        0.10,                 # z_dot
        np.deg2rad(6.0),      # theta
        np.deg2rad(-4.0)      # theta_dot
    ], dtype=float)

    t_hover, X_hover = simulate(
        lambda t, x: dyn_nonlinear_closed(t, x, params, K, ref_hover_origin, True),
        x0_hover,
        t_final=8.0,
        dt=0.01
    )

    E_hover, DU_hover, U_hover, REF_hover = get_control_history(
        t_hover, X_hover, params, K, ref_hover_origin, saturate=True
    )

    plot_states(
        t_hover,
        X_hover,
        REF_hover,
        "Nonlinear Closed-Loop Hover Stabilization",
        figs_dir / "fig02_nonlinear_hover_states.png"
    )

    plot_controls(
        t_hover,
        U_hover,
        params,
        "Control Inputs During Nonlinear Hover Stabilization",
        figs_dir / "fig03_nonlinear_hover_controls.png"
    )

    plot_xz_path(
        X_hover,
        REF_hover,
        "XZ Trajectory: Hover Stabilization",
        figs_dir / "fig04_hover_xz_path.png"
    )

    hover_metrics = compute_hover_metrics(t_hover, X_hover, U_hover)

    # --------------------------------------------------------
    # Scenario 3: Nonlinear setpoint regulation
    # --------------------------------------------------------
    t_step_cmd = 2.0
    x_cmd = 1.0
    z_cmd = 1.0

    ref_fun_step = lambda t: ref_step(t, t_step=t_step_cmd, x_cmd=x_cmd, z_cmd=z_cmd)

    x0_track = np.zeros(6, dtype=float)

    t_track, X_track = simulate(
        lambda t, x: dyn_nonlinear_closed(t, x, params, K, ref_fun_step, True),
        x0_track,
        t_final=12.0,
        dt=0.01
    )

    E_track, DU_track, U_track, REF_track = get_control_history(
        t_track, X_track, params, K, ref_fun_step, saturate=True
    )

    plot_states(
        t_track,
        X_track,
        REF_track,
        "Nonlinear Closed-Loop Setpoint Tracking",
        figs_dir / "fig05_nonlinear_setpoint_states.png",
        t_step=t_step_cmd
    )

    plot_controls(
        t_track,
        U_track,
        params,
        "Control Inputs During Nonlinear Setpoint Tracking",
        figs_dir / "fig06_nonlinear_setpoint_controls.png",
        t_step=t_step_cmd
    )

    plot_xz_path(
        X_track,
        REF_track,
        "XZ Trajectory: Setpoint Tracking",
        figs_dir / "fig07_setpoint_xz_path.png"
    )

    step_metrics = compute_step_metrics(
        t_track, X_track, U_track,
        t_step=t_step_cmd, x_cmd=x_cmd, z_cmd=z_cmd
    )

    # --------------------------------------------------------
    # Save summary text file
    # --------------------------------------------------------
    save_summary(
        results_dir / "part3_summary.txt",
        A, B, Q, R, K, P, eig_cl,
        hover_metrics, step_metrics, params
    )

    # Also save K and poles separately
    np.savetxt(results_dir / "K_gain_matrix.csv", K, delimiter=",")
    np.savetxt(results_dir / "closed_loop_eigenvalues.csv",
               np.column_stack((np.real(eig_cl), np.imag(eig_cl))),
               delimiter=",",
               header="real_part,imag_part",
               comments="")

    print("\n--- Hover stabilization metrics (nonlinear plant) ---")
    for k, v in hover_metrics.items():
        print(f"{k}: {v}")

    print("\n--- Setpoint tracking metrics (nonlinear plant) ---")
    for k, v in step_metrics.items():
        print(f"{k}: {v}")

    print("\nDone.")
    print(f"Figures saved in: {figs_dir.resolve()}")
    print(f"Results saved in: {results_dir.resolve()}")


if __name__ == "__main__":
    main()