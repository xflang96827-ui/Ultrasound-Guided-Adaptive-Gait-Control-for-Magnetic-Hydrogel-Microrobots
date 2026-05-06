# Ultrasound-Guided Closed-Loop Control of Magnetic Hydrogel Microrobots

<div align="center">

### Adaptive Multimodal Navigation in Tissue-Mimicking Environments

</div>

---

## Overview

This repository provides an open research-oriented implementation of an **ultrasound-guided closed-loop control framework for magnetic hydrogel microrobots** capable of adaptive multimodal locomotion in tissue-like environments.

The project integrates:

* **Ultrasound-based localization** using inter-frame differencing
* **Correlated-noise Extended Kalman Filtering (EKF)**
* **RRT*** path planning under environmental constraints
* **Model Predictive Control (MPC)** for swimming locomotion
* **PID-based regulation** for rolling locomotion
* **Adaptive gait switching** with stability-aware control logic

Inspired by the locomotion strategies of the *crown-of-thorns starfish*, the system autonomously transitions between:

* **Swimming mode** for obstacle traversal and vertical mobility
* **Rolling mode** for stable surface-conforming navigation

The repository is designed for:

* researchers in **soft robotics**,
* scientists working on **biomedical microrobotics**,
* and engineers interested in **closed-loop autonomous systems**.

---

# Scientific Motivation

Soft microrobots possess major advantages for biomedical applications:

* mechanical compliance,
* biocompatibility,
* safe tissue interaction,
* and adaptability in confined environments.

However, achieving **reliable closed-loop navigation** inside tissue-like media remains extremely challenging due to:

1. weak ultrasound contrast,
2. nonlinear hydrogel deformation,
3. uncertain magnetic actuation,
4. coupled fluid–structure dynamics,
5. and incomplete environmental information.

This project addresses these challenges through a unified sensing–planning–control architecture.

Unlike many existing open-source robotics repositories that focus purely on simulation or kinematics, this framework explicitly combines:

* imaging,
* estimation,
* control,
* path planning,
* and locomotion adaptation

within a single modular system.

---

# System Architecture

```text
                Ultrasound Imaging
                        │
                        ▼
          Inter-frame Motion Detection
                        │
                        ▼
        Correlated-Noise EKF Estimation
                        │
                        ▼
               Environment Mapping
                        │
                        ▼
                 RRT* Path Planner
                        │
                        ▼
            Adaptive Gait Scheduler
             │                     │
             ▼                     ▼
      MPC Swimming Control    PID Rolling Control
             │                     │
             └──────────┬──────────┘
                        ▼
              Magnetic Field Commands
                        │
                        ▼
            Hydrogel Microrobot Motion
```

---

# Key Contributions

## 1. Ultrasound-Based Real-Time Localization

The repository implements a lightweight yet robust localization strategy combining:

* inter-frame differencing,
* adaptive thresholding,
* connected-component analysis,
* and EKF-based smoothing.

This enables stable localization even under:

* weak echoes,
* deformable morphologies,
* and noisy ultrasound conditions.

---

## 2. Correlated-Noise Extended Kalman Filter

Instead of assuming independent process and measurement noise, the framework explicitly models:

[
Cov(w_k, v_k) \neq 0
]

This is particularly important in soft robotic systems where:

* actuation uncertainty,
* fluid interaction,
* and sensing noise

are physically coupled.

The EKF implementation therefore incorporates:

* cross-covariance estimation,
* modified Kalman gain updates,
* and stabilized covariance propagation.

This provides substantially smoother trajectory estimation compared with standard filtering methods.

---

## 3. Multimodal Locomotion

Two biologically inspired locomotion modes are implemented.

### Swimming Mode

Used for:

* vertical obstacle traversal,
* free-fluid navigation,
* and non-contact movement.

Controlled using:

* Model Predictive Control (MPC)

Advantages:

* nonlinear constraint handling,
* predictive optimization,
* and trajectory tracking.

---

### Rolling Mode

Used for:

* stable surface locomotion,
* low-variance tracking,
* and rapid planar motion.

Controlled using:

* PID feedback control

Advantages:

* simplicity,
* robustness,
* and computational efficiency.

---

## 4. Adaptive Gait Switching

The controller autonomously selects locomotion modes based on:

* environmental geometry,
* obstacle interaction,
* and motion feasibility.

The switching framework is stability-aware and inspired by:

* Average Dwell Time (ADT) analysis,
* Lyapunov stability theory,
* and hybrid dynamical systems.

---

# Repository Structure

```text
hydrogel-microrobot/
│
├── README.md
├── requirements.txt
├── setup.py
├── LICENSE
│
├── configs/
│   ├── controller.yaml
│   ├── planner.yaml
│   └── ultrasound.yaml
│
├── data/
│   ├── ultrasound_frames/
│   └── trajectories/
│
├── docs/
│   ├── figures/
│   ├── architecture/
│   └── theory/
│
├── examples/
│   ├── run_demo.py
│   ├── obstacle_crossing.py
│   └── gait_switching.py
│
├── microrobot/
│   ├── control/
│   │   ├── mpc_controller.py
│   │   ├── pid_controller.py
│   │   └── gait_scheduler.py
│   │
│   ├── estimation/
│   │   ├── ekf.py
│   │   └── ultrasound_tracker.py
│   │
│   ├── planning/
│   │   ├── rrt_star.py
│   │   └── collision.py
│   │
│   ├── locomotion/
│   │   ├── swimming.py
│   │   ├── rolling.py
│   │   └── dynamics.py
│   │
│   ├── simulation/
│   │   ├── environment.py
│   │   └── phantom.py
│   │
│   └── utils/
│
└── tests/
```

---

# Installation

## Clone Repository

```bash
git clone https://github.com/YOUR_USERNAME/hydrogel-microrobot.git

cd hydrogel-microrobot
```

---

## Create Environment

```bash
conda create -n hydrogel_robot python=3.10
conda activate hydrogel_robot
```

---

## Install Dependencies

```bash
pip install -r requirements.txt
```

---

# Quick Start

## Run Closed-Loop Navigation Demo

```bash
python examples/run_demo.py
```

---

## Run Obstacle Crossing Experiment

```bash
python examples/obstacle_crossing.py
```

---

## Run Adaptive Gait Switching

```bash
python examples/gait_switching.py
```

---

# Mathematical Formulation

## State Representation

The robot pose is represented on the Lie group:

[
SE(2)
]

with state:

[
X = (x, y, \theta)
]

where:

* (x,y) denote planar position,
* (\theta) denotes heading angle.

---

## Swimming Dynamics

[
\dot{x} = v_y \tan(\alpha)\cos\theta - v_y\sin\theta
]

[
\dot{y} = v_y \tan(\alpha)\sin\theta + v_y\cos\theta
]

---

## Rolling Dynamics

[
\dot{x} = \mu_r \omega_r
]

[
\dot{\theta} = \omega_r
]

---

## MPC Objective

[
J = \sum_{k=0}^{N-1}(e_k^TQe_k + u_k^TRu_k) + e_N^TPe_N
]

The controller minimizes:

* tracking error,
* control effort,
* and trajectory deviation.

---

# Research Extensions

This repository is intentionally designed to be extensible.

Possible future directions include:

## Physics-Informed Learning

Potential integration:

* PINNs,
* differentiable fluid models,
* learned hydrodynamics,
* neural observers,
* and adaptive residual dynamics.

---

## Reinforcement Learning

Potential integration:

* hybrid locomotion policies,
* autonomous gait discovery,
* adaptive obstacle negotiation,
* and exploration under uncertainty.

---

## Real Biomedical Translation

Potential future applications:

* targeted drug delivery,
* minimally invasive surgery,
* localized hyperthermia,
* vascular intervention,
* and in vivo autonomous navigation.

---

# Why This Repository Matters

Most open robotics projects fall into one of two categories:

1. purely theoretical control implementations,
2. or visually appealing simulations without rigorous estimation.

This repository instead attempts to bridge:

* control theory,
* medical imaging,
* soft robotics,
* and autonomous decision-making.

The goal is not merely to reproduce figures from a paper.

The goal is to provide a foundation for:

* next-generation intelligent microrobotic systems.

---

# Citation

If you use this repository in your research, please cite:

```bibtex
@article{lang2026hydrogel,
  title={Ultrasound-Guided Closed-Loop Control of Magnetic Hydrogel Microrobots with Adaptive Gait Switching},
  author={Lang, Xiaofeng and Chu, Lang and Chi, Lei and Qi, Cheng and Kong, Tiantian and Liu, Zhou},
  journal={Under Review},
  year={2026}
}
```

---

# Acknowledgements

This repository was inspired by recent advances in:

* soft microrobotics,
* ultrasound-guided intervention,
* hybrid locomotion,
* and adaptive autonomous control.

Special appreciation is extended to the broader soft robotics and biomedical engineering communities whose foundational work continues to shape the future of intelligent medical systems.

---

# Research Directions

This repository sits at the intersection of:

* Physics-Informed Learning
* Dynamical Systems
* Intelligent Control
* Soft Robotics
* Biomedical Robotics
* Hybrid Systems
* Computational Imaging
* Autonomous Navigation
* Magnetic Actuation
* Medical Microrobotics
* Scientific Machine Learning
* State Estimation
* Nonlinear Control
* Robot Learning
* Human-Centered Robotics

---

# Research Philosophy

The future of intelligent biomedical robotics will not emerge from isolated disciplines.

It will emerge from the fusion of:

* physics,
* learning,
* control,
* sensing,
* and embodied intelligence.

Modern robotic systems increasingly require:

* physically grounded learning,
* adaptive decision-making,
* uncertainty-aware estimation,
* and real-time autonomous control.

This repository therefore intentionally combines:

| Domain                    | Role in This Project                               |
| ------------------------- | -------------------------------------------------- |
| Soft Robotics             | Compliant interaction with biological environments |
| Dynamical Systems         | Mathematical modeling of multimodal locomotion     |
| Control Theory            | MPC/PID closed-loop regulation                     |
| Estimation Theory         | Correlated-noise EKF localization                  |
| Computational Imaging     | Ultrasound-based environmental perception          |
| Physics-Informed Learning | Future differentiable modeling extensions          |
| Hybrid Systems            | Stable switching between locomotion modes          |
| Intelligent Robotics      | Autonomous exploration and adaptation              |

Rather than viewing robotics as a purely mechanical problem, this project treats robotic intelligence as a coupled dynamical process involving:

* embodiment,
* sensing,
* uncertainty,
* control,
* and adaptation.

---

# Long-Term Research Vision

The broader research vision behind this project includes:

## Physics-Informed Intelligent Microrobots

Future microrobots should not only move.

They should:

* infer environmental structure,
* estimate hidden dynamics,
* adapt to uncertainty,
* and learn physically consistent behaviors.

This motivates future integration with:

* PINNs (Physics-Informed Neural Networks),
* Neural Operators,
* Koopman-based dynamics,
* differentiable simulation,
* and scientific foundation models.

---

## Embodied AI for Biomedical Systems

Biomedical robots operate under:

* uncertainty,
* partial observability,
* constrained sensing,
* and strong safety requirements.

Therefore, future intelligent systems must combine:

* perception,
* control,
* and physical reasoning

inside a unified embodied framework.

This repository is intended as an early step toward:

* embodied scientific intelligence,
* physically grounded autonomy,
* and safe adaptive biomedical robotics.

---

## Learning + Control + Physics

A major future direction is the integration of:

[
\text{Physics} + \text{Learning} + \text{Control}
]

within closed-loop robotic systems.

Potential future modules include:

* learned residual dynamics,
* adaptive MPC,
* uncertainty-aware planners,
* neural state observers,
* differentiable fluid models,
* and reinforcement learning under physical constraints.

---

# Technical Highlights

## Closed-Loop Autonomy

The robot continuously:

1. senses through ultrasound,
2. estimates state uncertainty,
3. updates environmental understanding,
4. plans feasible trajectories,
5. regulates locomotion,
6. and adaptively switches gait.

This creates a fully closed-loop perception–planning–control pipeline.

---

## Hybrid Dynamical Systems

The system naturally forms a hybrid dynamical process:

* continuous state evolution,
* discrete gait transitions,
* and environment-triggered mode switching.

This is highly relevant to modern research in:

* hybrid robotics,
* autonomous systems,
* and embodied intelligence.

---

## Research-Level Mathematical Structure

The repository explicitly incorporates:

* Lie groups (SE(2)),
* nonlinear kinematics,
* constrained optimization,
* Lyapunov stability,
* Average Dwell Time analysis,
* and correlated-noise estimation.

---

# Academic Positioning

This repository reflects a research profile positioned around:

* intelligent physical systems,
* mathematically grounded robotics,
* and next-generation biomedical autonomy.

* embodied intelligence,
* scientific machine learning,
* adaptive control,
* and soft robotic systems.

---

# Maintainer

**Xiaofeng Lang**

Research Interests:

* Physics-Informed Learning
* Dynamical Systems
* Intelligent Control
* Soft Robotics
* Biomedical Microrobotics
* Scientific Machine Learning
* Hybrid Dynamical Systems
* Embodied AI
* Adaptive Autonomous Systems

Current Focus:

Developing physically grounded intelligent robotic systems that combine:

* sensing,
* control,
* learning,
* and adaptive reasoning

for next-generation biomedical and autonomous applications.

---

# Final Remark

The long-term vision of this project is not simply to build a controllable hydrogel robot.

It is to move toward a future where soft autonomous agents can:

* sense,
* reason,
* adapt,
* and safely interact

inside complex biological environments.

That future will require the fusion of:

* mechanics,
* control,
* learning,
* imaging,
* and intelligence.

This repository is one small step toward that direction.
