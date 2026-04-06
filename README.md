# Closed-Loop Control of a Quadrotor UAV

## Project Overview

This project focuses on the **modeling, analysis, and control of a quadrotor UAV** with the goal of improving **hover stability and translational motion**.

The system is inherently:

* **Multi-Input Multi-Output (MIMO)**
* **6 Degrees of Freedom (6-DOF)**
* Controlled using **4 inputs** (typical quadrotor thrust and torques)

Initial analysis revealed that the **open-loop system is unstable**, motivating the design and implementation of a **closed-loop control system**.

---

## Objectives

* Model quadrotor dynamics using **state-space representation**
* Analyze **stability and controllability**
* Design a **closed-loop controller** to stabilize the system
* Evaluate system performance using **simulation plots**

---

## System Description

### Degrees of Freedom

* Position: x, y, z
* Orientation: roll (ϕ), pitch (θ), yaw (ψ)

### Control Inputs

* Total thrust
* Roll torque
* Pitch torque
* Yaw torque

---

## Methodology

### 1. System Modeling

* Derived nonlinear dynamic equations
* Linearized around hover condition
* Represented system using:

  * State-space matrices (A, B, C, D)
  * Block diagrams

### 2. Stability Analysis

* Open-loop system analyzed using **eigenvalues**
* Stability assessed from system matrix

### 3. Controllability

* Checked using controllability matrix
* Ensured full-state control capability

### 4. Controller Design

* Implemented **state-feedback control**
* Gain matrix (K) computed for stabilization

### 5. Simulation

* Compared:

  * Open-loop vs Closed-loop response
  * Linear vs Nonlinear system behavior

---

## Results

Generated outputs include:

### Figures (`figures_part3/`)

* Closed-loop pole plots
* Open vs closed-loop comparisons
* Hover state responses
* Control input evolution
* Setpoint tracking performance
* X-Z path trajectories
  
<img width="1800" height="1500" alt="fig00_closed_loop_poles" src="https://github.com/user-attachments/assets/4acd5878-9538-4ea5-9676-fcd5375ef6e8" />

<img width="2400" height="2700" alt="fig01_linear_open_vs_closed" src="https://github.com/user-attachments/assets/beedd19b-9699-471d-ad5a-05c5d6c110e6" />

<img width="3600" height="3000" alt="fig02_nonlinear_hover_states" src="https://github.com/user-attachments/assets/160f97dd-0a32-4f84-8b05-7d41b9c9a54d" />

<img width="3300" height="2400" alt="fig03_nonlinear_hover_controls" src="https://github.com/user-attachments/assets/c3688614-5cb0-4433-b545-bd66d4bff00f" />

<img width="2100" height="1800" alt="fig04_hover_xz_path" src="https://github.com/user-attachments/assets/3a875b56-e199-4e12-a3d8-a7179fe5ed9a" />

<img width="3600" height="3000" alt="fig05_nonlinear_setpoint_states" src="https://github.com/user-attachments/assets/44592bb2-14e6-40f2-9188-2f59dbb5a8e4" />

<img width="3300" height="2400" alt="fig06_nonlinear_setpoint_controls" src="https://github.com/user-attachments/assets/729102b6-25e8-4081-b293-8d192f3a5b2b" />

<img width="2100" height="1800" alt="fig07_setpoint_xz_path" src="https://github.com/user-attachments/assets/255db43a-3a5d-4847-a0ea-d120ac1c02b2" />

### Data (`results_part3/`)

* `closed_loop_eigenvalues.csv`
* `K_gain_matrix.csv`
* `part3_summary.txt`

## Closed Loop Eigenvalues

| Real Part | Imaginary Part |
|----------|----------------|
| -486.0863 | 0.0000 |
| -3.6577   | 0.0000 |
| -1.5173   | ±1.2211 |
| -2.5561   | 0.0000 |
| -8.5713   | 0.0000 |

### Stability Analysis
All eigenvalues have negative real parts ⇒ the system is **stable**.

## K Gain Matrix

|     | x1 | x2 | x3 | x4 | x5 | x6 |
|-----|----|----|----|----|----|----|
| u1  | 0 | 0 | 10.9545 | 5.5637 | 0 | 0 |
| u2  | -6.3246 | -6.8019 | 0 | 0 | 30.0645 | 4.5336 |

### Interpretation
- Control input **u1** mainly affects states x3 and x4  
- Control input **u2** affects x1, x2, x5, x6

## SUMMARY: PLANAR QUADROTOR CLOSED-LOOP CONTROL
### Nominal System Parameters

- **Mass (m):** 0.5 kg  
- **Moment of Inertia (I):** 0.0023 kg·m²  
- **Arm Length (l):** 0.25 m  
- **Gravity (g):** 9.81 m/s²  
- **Maximum Thrust (fmax):** 10 N  

[View Full Report](results_part3/summary.txt)

---

## Project Structure

```
Closed Loop Control/
│
├── figures_part3/              # Output plots from simulations
├── results_part3/              # Numerical results (eigenvalues, gain matrix, summary)
├── code.py                     # Main simulation and control implementation


└── block_diagram.png          # Flowchart of loop stabilization
└── quadrotor_schematic.png    # Simple schematic of planar quadrotor
```

---

## ▶️ How to Run

### 1. Install Dependencies

Make sure you have Python installed, then install required libraries:

```bash
pip install numpy matplotlib scipy pandas
```

### 2. Run the Simulation

```bash
python code.py
```

### 3. View Outputs

* Generated plots will appear in `figures_part3/`
* Numerical results stored in `results_part3/`

---

## Key Features

* Open-loop system is **unstable**
* Closed-loop control significantly improves:

  * Stability - Stable hover using LQR 
  * Tracking performance - Low overshoot tracking  
* System is **fully controllable** despite fewer inputs than DOFs - Efficient control effort
* Works on nonlinear system

---







