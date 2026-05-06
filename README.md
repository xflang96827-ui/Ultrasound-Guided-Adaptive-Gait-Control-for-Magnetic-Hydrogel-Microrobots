# Ultrasound-Guided Closed-Loop Control of Magnetic Hydrogel Microrobots

<div align="center">

### Adaptive Multimodal Navigation in Tissue-Mimicking Environments

![Python](https://img.shields.io/badge/Python-3.10+-blue.svg)
![Research](https://img.shields.io/badge/Research-Soft%20Robotics-red.svg)
![Control](https://img.shields.io/badge/Control-MPC%20%2B%20EKF-orange.svg)
![Status](https://img.shields.io/badge/Status-Research%20Prototype-lightgrey.svg)

</div>

---

## Overview

This repository provides a research-oriented implementation of an **ultrasound-guided closed-loop control framework for magnetic hydrogel microrobots** with adaptive gait switching.

The project integrates:

- ultrasound-based localization,
- correlated-noise Extended Kalman Filtering,
- RRT* path planning,
- Model Predictive Control for swimming,
- PID control for rolling,
- and stability-aware adaptive gait switching.

The system is inspired by the locomotion strategy of the **crown-of-thorns starfish**, where different locomotion modes are used depending on local environmental conditions.

---

## Research Motivation

Soft hydrogel microrobots are attractive for biomedical applications because of their mechanical compliance, biocompatibility, safe interaction with soft tissue, and ability to deform in confined environments.

However, precise autonomous navigation remains difficult because soft robots operate under weak ultrasound echoes, uncertain magnetic actuation, nonlinear deformation, fluid–structure coupling, and incomplete environmental information.

This repository treats the problem as a **closed-loop sensing–estimation–planning–control system**, rather than a purely mechanical actuation problem.

---

## System Architecture

```text
                Ultrasound Imaging
                        |
                        v
          Inter-Frame Motion Detection
                        |
                        v
        Correlated-Noise EKF Estimation
                        |
                        v
               Environment Mapping
                        |
                        v
                 RRT* Path Planner
                        |
                        v
            Adaptive Gait Scheduler
             |                     |
             v                     v
      MPC Swimming Control    PID Rolling Control
             |                     |
             +----------+----------+
                        v
              Magnetic Field Commands
                        |
                        v
            Hydrogel Microrobot Motion
```

---

## Key Features

### Ultrasound-Guided Localization

The robot is localized from B-mode ultrasound images using inter-frame differencing and background subtraction. The tracked centroid is then filtered using a correlated-noise EKF.

### Correlated-Noise EKF

Instead of assuming independent process and measurement noise, this project explicitly considers the possibility that magnetic actuation uncertainty and ultrasound measurement noise are weakly correlated.

### Multimodal Locomotion

The system supports two locomotion modes:

- **Swimming mode**: useful for vertical traversal and obstacle crossing.
- **Rolling mode**: useful for stable surface-conforming motion.

### Hierarchical Control

The framework uses **MPC** for swimming control, **PID** for rolling control, and an adaptive gait scheduler for mode switching.

### Stability-Aware Gait Switching

The switching strategy is motivated by Lyapunov stability and Average Dwell Time analysis, ensuring that transitions between locomotion modes do not destabilize the closed-loop system.

---

## Repository Structure

```text
hydrogel-microrobot/
|
├── README.md
├── FORMULAS.md
├── requirements.txt
├── setup.py
├── LICENSE
|
├── configs/
│   ├── controller.yaml
│   ├── planner.yaml
│   └── ultrasound.yaml
|
├── examples/
│   ├── run_demo.py
│   ├── obstacle_crossing.py
│   └── gait_switching.py
|
├── microrobot/
│   ├── control/
│   ├── estimation/
│   ├── planning/
│   ├── locomotion/
│   ├── simulation/
│   └── utils/
|
├── docs/
│   ├── theory/
│   ├── figures/
│   └── notes/
|
└── tests/
```

---

## Installation

```bash
git clone https://github.com/YOUR_USERNAME/hydrogel-microrobot.git
cd hydrogel-microrobot
```

```bash
conda create -n hydrogel_robot python=3.10
conda activate hydrogel_robot
```

```bash
pip install -r requirements.txt
```

---

## Quick Start

```bash
python examples/run_demo.py
```

```bash
python examples/obstacle_crossing.py
```

```bash
python examples/gait_switching.py
```

---

## Mathematical Foundation

All mathematical formulas are placed in:

```text
FORMULAS.md
```

This avoids overcrowding the README while keeping the repository mathematically rigorous.

The theoretical framework includes inter-frame ultrasound localization, RRT* path planning, OBB/SAT collision detection, correlated-noise EKF, Lie group modeling on SE(2), swimming and rolling kinematics, MPC and PID control, hybrid gait switching, deformation-aware propulsion modeling, and physics-informed learning extensions.

---

## Research Direction

This repository sits at the intersection of soft robotics, biomedical microrobotics, control theory, dynamical systems, uncertainty-aware estimation, computational imaging, scientific machine learning, and physics-informed learning.

The long-term goal is to develop physically grounded intelligent robotic systems that combine sensing, estimation, control, learning, and adaptive decision-making.

---

## Citation

```bibtex
@article{lang2026hydrogel,
  title={Ultrasound-Guided Closed-Loop Control of Magnetic Hydrogel Microrobots with Adaptive Gait Switching},
  author={Lang, Xiaofeng and Chu, Lang and Chi, Lei and Qi, Cheng and Kong, Tiantian and Liu, Zhou},
  journal={Under Review},
  year={2026}
}
```

---

## Maintainer

**Your Name**

Research interests:

- Physics-Informed Learning
- Dynamical Systems
- Intelligent Control
- Soft Robotics
- Biomedical Microrobotics
- Scientific Machine Learning
- Hybrid Systems
- Embodied AI
