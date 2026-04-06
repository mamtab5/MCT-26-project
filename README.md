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

### Data (`results_part3/`)

* `closed_loop_eigenvalues.csv`
* `K_gain_matrix.csv`
* `part3_summary.txt`

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

## Key Insights

* Open-loop system is **unstable**
* Closed-loop control significantly improves:

  * Stability
  * Tracking performance
* System is **fully controllable** despite fewer inputs than DOFs

---
