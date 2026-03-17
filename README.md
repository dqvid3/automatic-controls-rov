# Automatic Controls — ROV Controller Design

**Course:** Controlli Automatici (Automatic Controls) — BEng in Computer Engineering, University of Palermo
**Academic Year:** 2022/2023
**Group:** CA22-21 (6 members)

---

## Overview

This project covers the full control design pipeline for a **remotely operated underwater vehicle (ROV)** subject to buoyancy, longitudinal thrust, and bow thruster forces. The goal is to stabilize the bow pitch angle and achieve perfect tracking of constant reference signals.

The system is described by a nonlinear state-space model with states: depth $z$, pitch angle $\vartheta$, and their derivatives. The control input is the bow thruster force $F$.

---

## Project Structure

### Symbolic Analysis
- Equilibrium configurations derived analytically (H1: $\bar\vartheta = \pi/6$ rad)
- Linearization around equilibrium via small-perturbation approximation
- Free evolution expressed in modal form $M(A, x_0)\, m(t)$
- Asymptotic stability analysis of the linearized modes
- Transfer function $G(s)$ derivation, pole/zero analysis

### Numerical Analysis
- Numerical evaluation of $G(s)$ with physical parameters
- Translation of control requirements (R1) into static/dynamic specs
- Bode form representation of the open-loop plant
- Lead-lag controller design $C(s)$ to meet specs:
  - Perfect tracking of constant references
  - Rise time $\leq 45$ s, overshoot $\leq 5\%$
- MATLAB/Simulink closed-loop simulation (linear model)
- MATLAB/Simulink closed-loop simulation (nonlinear model)

### Theoretical Section
- Additional theoretical question on control systems

---

## System Parameters

| Parameter | Description | Value |
|-----------|-------------|-------|
| $M$ | Vehicle mass | 580 kg |
| $J$ | Lateral inertia | 560 Nms² |
| $B$ | Buoyancy force | 20 N |
| $b$ | CoM–bow distance | 1.2 m |
| $k_z$, $k_r$ | Viscous friction coefficients | 133 Ns/m, 168 Nms |
| $k_\vartheta$ | Lever constant | 143 Nm |

---

## Controller

A lead-lag controller was designed via Bode diagram synthesis:

$$C(s) = -11.4 \cdot \frac{1 + s/0.274}{1 + s/0.902}$$

Verified via closed-loop simulation in both the linearized and original nonlinear model.

---

## Files

```
├── def/
│   ├── main.tex              # Main LaTeX document
│   ├── afstyle.tex           # Exam template style
│   └── theme-sottomarino.tex # ROV theme: model, parameters, requirements
├── support/
│   ├── runme.m               # MATLAB script: analysis & controller design
│   ├── simulazione.slx       # Simulink model
│   └── Sisotool.mat          # SISOTOOL session data
├── img/                      # Figures and images
├── 1A.tex – 1F.tex           # Symbolic analysis solutions
├── 2A.tex – 2F.tex           # Numerical analysis solutions
└── 3A.tex                    # Theoretical answer
```

---

## Tools

- **LaTeX** — written report
- **MATLAB / Simulink** — numerical analysis, controller design, simulation
