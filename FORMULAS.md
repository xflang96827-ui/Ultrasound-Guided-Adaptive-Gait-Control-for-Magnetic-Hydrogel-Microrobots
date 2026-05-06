# Mathematical Formulas and Theoretical Notes

This file collects the main mathematical formulas used in the project.

> GitHub Markdown supports block equations using `$$ ... $$`.  
> Do **not** use `\[ ... \]` in GitHub README files, because GitHub may display them as plain text.

---

# 1. Ultrasound Localization

## Inter-Frame Difference Map

The ultrasound localization module uses a difference map combining frame-to-frame motion and background subtraction:

$$
D_t(x,y)
=
\alpha
\left|
I_t(x,y)-I_{t-1}(x,y)
\right|
+
\beta
\left|
I_t(x,y)-I_0(x,y)
\right|
$$

with:

$$
\alpha+\beta=1
$$

where $I_t(x,y)$ is the current ultrasound frame, $I_{t-1}(x,y)$ is the previous frame, and $I_0(x,y)$ is the initial background frame.

---

# 2. RRT* Path Planning

## State Space

The environment is divided into the admissible state space:

$$
\chi
$$

obstacle space:

$$
\chi_{obs}
$$

and contact-permissive boundary regions:

$$
\chi_{boundary}
$$

The start and goal states are:

$$
\chi_{start}, \quad \chi_{goal}
$$

The resulting path is denoted by:

$$
\sigma
$$

---

## Tree Initialization

The RRT* tree is defined as:

$$
T=(V,E)
$$

with initial condition:

$$
V=\{\chi_{start}\}
$$

$$
E=\varnothing
$$

---

## Random Sampling

$$
x_{rand}
\sim
\mathrm{Uniform}(\chi)
$$

---

## Nearest Node Search

$$
x_{near}
=
\arg\min_{x\in V}
\left\|
x-x_{rand}
\right\|
$$

---

## Node Expansion Distance

$$
d
=
\left\|
x_{rand}-x_{near}
\right\|
$$

---

## New Node Generation

$$
x_{new}
=
\begin{cases}
x_{rand}, & d<\eta \\
x_{near}
+
\dfrac{x_{rand}-x_{near}}{d}\eta,
& \text{otherwise}
\end{cases}
$$

where $\eta$ is the expansion step size.

---

## Collision-Free Condition

$$
\mathrm{ObstacleFree}(x_{near},x_{new})
=
\begin{cases}
\mathrm{True}, & \text{if no collision occurs} \\
\mathrm{False}, & \text{if collision occurs}
\end{cases}
$$

---

## Neighbor Set

$$
X_{near}
=
\left\{
x\in V
\mid
\left\|
x-x_{new}
\right\|<r
\right\}
$$

---

## Search Radius

$$
r
=
\gamma
\left(
\frac{\log |V|}{|V|}
\right)^{1/d_s}
$$

where $\gamma$ is a constant and $d_s$ is the state-space dimension.

---

## Parent Selection

$$
x_{min}
=
\arg\min_{x\in X_{near}}
\left(
\mathrm{Cost}(x)
+
\left\|
x-x_{new}
\right\|
\right)
$$

---

## Path Cost

$$
\mathrm{Cost}(x)
=
\sum_{i=1}^{n}
\left\|
x_i-x_{i-1}
\right\|
$$

---

## Tree Update

$$
V
=
V\cup \{x_{new}\}
$$

$$
E
=
E\cup \{(x_{min},x_{new})\}
$$

---

## Rewiring Condition

If:

$$
\mathrm{Cost}(x_{new})
+
\left\|
x_{new}-x
\right\|
<
\mathrm{Cost}(x)
$$

then:

$$
\mathrm{Parent}(x)
=
x_{new}
$$

and:

$$
\mathrm{Cost}(x)
=
\mathrm{Cost}(x_{new})
+
\left\|
x_{new}-x
\right\|
$$

---

# 3. OBB/SAT Collision Detection

## OBB Representation

An oriented bounding box is represented by:

$$
C=(c_x,c_y,c_z)
$$

$$
H=(h_x,h_y,h_z)
$$

$$
R=[u,v,w]
$$

For obstacle $i$:

$$
R_i=[u_i,v_i,w_i]
$$

---

## Projection Interval

For a separating axis $L$, the projection interval is:

$$
\mathrm{Projection}_i
=
\left[
C_i\cdot L-r_i,
\;
C_i\cdot L+r_i
\right]
$$

---

## Projection Radius

$$
r_i
=
h_{x_i}|u_i\cdot L|
+
h_{y_i}|v_i\cdot L|
+
h_{z_i}|w_i\cdot L|
$$

---

## SAT Non-Intersection Condition

Two OBBs do not intersect if:

$$
C_i\cdot L+r_i
<
C_j\cdot L-r_j
$$

or:

$$
C_j\cdot L+r_j
<
C_i\cdot L-r_i
$$

---

# 4. Path Optimization and Gait Assignment

## Vertical Path Optimization Objective

$$
\min
\sum_{i=1}^{n-1}
|\Delta y_i|
$$

---

## Rolling Segment Constraint

For rolling segments:

$$
y_{i+1}^{new}
=
y_i^{new}
$$

---

## Vertical Increment Constraint

$$
y_{i+1}^{new}
=
y_i^{new}
+
\Delta y_i
$$

$$
|\Delta y_i|
\le
\Delta y_{max}
$$

---

## Collision-Free Path Constraint

$$
\mathrm{ObstacleFree}
\left(
x_{i+1}^{new},
x_i^{new}
\right)
=
\mathrm{True}
$$

---

## Updated Node Position

$$
y_{i+1}^{new}
=
y_i
+
\sum_{j=1}^{i-1}
\Delta y_j^\ast
$$

---

## New Path Cost

$$
\mathrm{Cost}_{new}
=
\sum_{i=0}^{n-1}
\left\|
x_{i+1}^{new}
-
x_i^{new}
\right\|
$$

---

## Iterative Optimization Condition

$$
\exists i:
\neg
\mathrm{ObstacleFree}
\left(
x_{i+1}^{new},
x_i^{new}
\right)
\Rightarrow
\left|
y_{i+1}^{new}
-
y_i^{new}
\right|
\le
\epsilon
$$

---

# 5. State-Space Model and EKF

## Linear State-Space Model

$$
x_{k+1}
=
A x_k
+
B u_k
+
w_k
$$

$$
y_k
=
C x_k
+
v_k
$$

The state may include velocity components and motion angle:

$$
x_k
=
\begin{bmatrix}
V_x \\
V_y \\
\varphi
\end{bmatrix}
$$

with:

$$
\varphi
=
\arctan
\left(
\frac{V_x}{V_y}
\right)
$$

---

## EKF Prediction

$$
\hat{x}_{k|k-1}
=
A_k
\hat{x}_{k-1|k-1}
+
B_k u_k
$$

---

## Covariance Prediction

$$
P_{k|k-1}
=
A_k
P_{k-1|k-1}
A_k^T
+
Q_k
$$

---

## Standard Kalman Gain

$$
K_k
=
P_{k|k-1}
C_k^T
\left(
C_k
P_{k|k-1}
C_k^T
+
R_k
\right)^{-1}
$$

---

## State Update

$$
\hat{x}_{k|k}
=
\hat{x}_{k|k-1}
+
K_k
\left(
y_k
-
C_k\hat{x}_{k|k-1}
\right)
$$

---

## Standard Covariance Update

$$
P_{k|k}
=
(I-K_kC_k)
P_{k|k-1}
$$

---

# 6. Correlated-Noise EKF

## Correlated Noise Assumption

The standard Kalman filter assumes:

$$
\mathrm{Cov}(w_k,v_k)=0
$$

In this system, the process noise and measurement noise may be weakly correlated:

$$
\mathrm{Cov}(w_k,v_k)\neq 0
$$

The cross-covariance is denoted as:

$$
S_k
$$

---

## Correlated-Noise Kalman Gain

$$
K_k
=
\left(
P_{k|k-1}C_k^T
+
S_k
\right)
\left(
C_kP_{k|k-1}C_k^T
+
R_k
+
H_kS_k
+
S_k^TH_k^T
\right)^{-1}
$$

---

## Correlated-Noise Covariance Update

$$
P_{k|k}
=
(I-K_kC_k)
P_{k|k-1}
(I-K_kC_k)^T
+
K_kR_kK_k^T
-
K_kS_k^T
-
S_kK_k^T
$$

---

## Bayesian Filtering Interpretation

$$
p(x_k\mid y_{1:k})
$$

A Bayesian update can be written as:

$$
p(x_k\mid y_{1:k})
\propto
p(y_k\mid x_k)
p(x_k\mid y_{1:k-1})
$$

---

# 7. Lie Group Formulation

## Pose on SE(2)

$$
X
=
\begin{bmatrix}
\cos\theta & -\sin\theta & x \\
\sin\theta & \cos\theta & y \\
0 & 0 & 1
\end{bmatrix}
\in
SE(2)
$$

---

## Lie Algebra Basis

$$
\hat{e}_1
=
\begin{bmatrix}
0 & 0 & 1 \\
0 & 0 & 0 \\
0 & 0 & 0
\end{bmatrix}
$$

$$
\hat{e}_2
=
\begin{bmatrix}
0 & 0 & 0 \\
0 & 0 & 1 \\
0 & 0 & 0
\end{bmatrix}
$$

$$
\hat{e}_3
=
\begin{bmatrix}
0 & -1 & 0 \\
1 & 0 & 0 \\
0 & 0 & 0
\end{bmatrix}
$$

---

## Lie Algebra Element

$$
\hat{\xi}
=
v_x\hat{e}_1
+
v_y\hat{e}_2
+
\omega\hat{e}_3
\in
\mathfrak{se}(2)
$$

---

## General Exponential Map Update

$$
X_{k+1}
=
X_k
\exp
\left(
\Delta t\hat{\xi}
\right)
$$

---

# 8. Swimming Dynamics

## Continuous-Time Swimming Dynamics

$$
\dot{x}
=
v_y\tan\alpha\cos\theta
-
v_y\sin\theta
$$

$$
\dot{y}
=
v_y\tan\alpha\sin\theta
+
v_y\cos\theta
$$

$$
\dot{\theta}
=
0
$$

---

## Normalized Discrete-Time Swimming Dynamics

The supplementary material writes the Euler-discretized normalized form as:

$$
x_{k+1}
=
x_k
+
\Delta t
\left(
\tan\alpha\cos\theta_k
-
\sin\theta_k
\right)
$$

$$
y_{k+1}
=
y_k
+
\Delta t
\left(
\tan\alpha\sin\theta_k
-
\cos\theta_k
\right)
$$

$$
\theta_{k+1}
=
\theta_k
$$

A dimensional version retaining $v_y$ is:

$$
x_{k+1}
=
x_k
+
\Delta t
\left(
v_y\tan\alpha\cos\theta_k
-
v_y\sin\theta_k
\right)
$$

$$
y_{k+1}
=
y_k
+
\Delta t
\left(
v_y\tan\alpha\sin\theta_k
+
v_y\cos\theta_k
\right)
$$

$$
\theta_{k+1}
=
\theta_k
$$

---

## Swimming Body Velocity

$$
\xi_{swim}
=
\begin{bmatrix}
v_x \\
v_y \\
0
\end{bmatrix}
=
\begin{bmatrix}
v_y\tan\alpha \\
v_y \\
0
\end{bmatrix}
$$

---

## Swimming Lie Algebra Element

$$
\hat{\xi}_{swim}
=
\begin{bmatrix}
0 & 0 & v_y\tan\alpha \\
0 & 0 & v_y \\
0 & 0 & 0
\end{bmatrix}
\in
\mathfrak{se}(2)
$$

---

## Swimming Exponential Map

$$
X_{k+1}
=
X_k
\exp
\left(
\Delta t\hat{\xi}_{swim}
\right)
$$

Expanded form:

$$
X_{k+1}
=
X_k
\begin{bmatrix}
1 & 0 & \Delta t\,v_y\tan\alpha \\
0 & 1 & \Delta t\,v_y \\
0 & 0 & 1
\end{bmatrix}
$$

---

# 9. Rolling Dynamics

## Continuous-Time Rolling Dynamics

$$
\dot{x}
=
\mu_r\omega_r
$$

$$
\dot{y}
=
0
$$

$$
\dot{\theta}
=
\omega_r
$$

---

## Discrete-Time Rolling Dynamics

$$
x_{k+1}
=
x_k
+
\Delta t\,\mu_r\omega_r
$$

$$
y_{k+1}
=
y_k
$$

$$
\theta_{k+1}
=
\theta_k
+
\Delta t\,\omega_r
$$

---

## Rolling Body Velocity

$$
\xi_{roll}
=
\begin{bmatrix}
\mu_r\omega_r \\
0 \\
\omega_r
\end{bmatrix}
$$

---

## Rolling Lie Algebra Element

$$
\hat{\xi}_{roll}
=
\begin{bmatrix}
0 & -\omega_r & \mu_r\omega_r \\
\omega_r & 0 & 0 \\
0 & 0 & 0
\end{bmatrix}
\in
\mathfrak{se}(2)
$$

---

## Rolling Exponential Map

$$
X_{k+1}
=
X_k
\exp
\left(
\Delta t\hat{\xi}_{roll}
\right)
$$

Expanded form:

$$
X_{k+1}
=
X_k
\begin{bmatrix}
\cos(\omega_r\Delta t)
&
-\sin(\omega_r\Delta t)
&
\mu_r\omega_r
\\
\sin(\omega_r\Delta t)
&
\cos(\omega_r\Delta t)
&
0
\\
0 & 0 & 1
\end{bmatrix}
$$

---

# 10. Geometric Tracking Error

## Relative Pose

$$
X_{rel}
=
X_k^{-1}
X_k^{ref}
$$

with:

$$
X_{rel}
=
\begin{bmatrix}
R_{rel} & t_{rel} \\
0 & 1
\end{bmatrix}
$$

---

## Rotation Error

$$
\omega
=
\operatorname{atan2}
\left(
R_{rel,21},
R_{rel,11}
\right)
$$

---

## Translation Error

$$
t_{rel}
=
\frac{\sin\omega}{\omega}p
+
\frac{1-\cos\omega}{\omega^2}
p^\perp
$$

where:

$$
p=
\begin{bmatrix}
v_x \\
v_y
\end{bmatrix}
$$

and:

$$
p^\perp
=
\begin{bmatrix}
-v_y \\
v_x
\end{bmatrix}
$$

---

## Error Vector

$$
e_k
=
\log
\left(
X_{rel}
\right)^\vee
=
\begin{bmatrix}
t_{rel,x} \\
t_{rel,y} \\
\omega
\end{bmatrix}
\in
\mathbb{R}^3
$$

---

# 11. Model Predictive Control

## MPC Objective

$$
J
=
\sum_{k=0}^{N-1}
\left(
e_k^TQe_k
+
u_k^TRu_k
\right)
+
e_N^TPe_N
$$

where:

$$
Q
=
\operatorname{diag}
(q_x,q_y,q_\theta)
\succ 0
$$

$$
R
=
\operatorname{diag}
(r_x,r_y)
\succ 0
$$

$$
P
\succ 0
$$

---

## Dynamic Constraint

$$
X_{k+1}
=
X_k
\exp
\left(
\Delta t\hat{\xi}_k
\right)
$$

---

## Input Constraints

$$
v_y^{min}
\le
v_y
\le
v_y^{max}
$$

$$
|\alpha|
\le
\alpha_{max}
$$

---

## Convexity Conditions

$$
\nabla_e^2
\left(
e^TQe
\right)
\succ
0
$$

$$
\nabla_u^2
\left(
u^TRu
\right)
\succ
0
$$

$$
\nabla_{e_N}^2
\left(
e_N^TPe_N
\right)
\succ
0
$$

---

## Jacobian of Control-to-Velocity Map

$$
J_k^{(i)}
=
\frac{\partial \xi_k}{\partial u_k}
=
\begin{bmatrix}
\dfrac{\partial v_x}{\partial v_y}
&
\dfrac{\partial v_x}{\partial \alpha}
\\
\dfrac{\partial v_y}{\partial v_y}
&
\dfrac{\partial v_y}{\partial \alpha}
\\
0 & 0
\end{bmatrix}
=
\begin{bmatrix}
\tan\alpha & v_y\sec^2\alpha \\
1 & 0 \\
0 & 0
\end{bmatrix}
$$

---

## Linearized Dynamics

$$
\xi_k
\approx
\xi_k^{(i)}
+
J_k^{(i)}
\left(
u_k-u_k^{(i)}
\right)
$$

---

## KKT Conditions

$$
\begin{cases}
\nabla_u J
+
\sum_i \lambda_i \nabla_u g_i
+
\sum_j \mu_j \nabla_u h_j
=
0
\\
g_i(u)\le 0,\quad \lambda_i\ge 0,\quad \lambda_i g_i(u)=0
\\
h_j(u)=0
\end{cases}
$$

---

# 12. PID Rolling Control

## PID Control Law

$$
\omega_r
=
K_p e_\theta
+
K_i
\int e_\theta dt
+
K_d
\frac{de_\theta}{dt}
+
K_{p,x}e_x
$$

---

# 13. Hybrid Switching and Stability

## Switching Signal

$$
\sigma(t)
\in
\{swim,roll\}
$$

---

## Hybrid System Representation

$$
\mathcal{H}
=
(\mathcal{C},F,\mathcal{D},G)
$$

---

## Average Dwell Time Condition

$$
\tau_d
>
\tau_d^\star
$$

A common switched-system form is:

$$
N_\sigma(t_0,t)
\le
N_0
+
\frac{t-t_0}{\tau_d}
$$

---

## Lyapunov Decrease Condition

For mode $i$:

$$
\dot{V}_i(x)
\le
-\lambda_i V_i(x)
$$

At switching instants:

$$
V_j(x)
\le
\mu V_i(x)
$$

A sufficient dwell-time condition is:

$$
\tau_d
>
\frac{\ln\mu}{\lambda}
$$

---

# 14. Soft-Robot Deformation and Propulsion Factor

## Magnetic Torque

$$
\tau_m
=
VMB\sin\psi
$$

---

## Quasi-Static Deformation Approximation

When:

$$
\dot{\varepsilon}
\gg
\dot{v}
$$

then:

$$
\varepsilon(t)
\approx
\frac{
VMB\sin\psi
}{
EI/L+\gamma\eta\omega
}
$$

---

## Propulsive Force

$$
F_{push}
=
\alpha(\varepsilon)f
$$

---

## Velocity–Input Relation

$$
v(t)
=
\alpha(\varepsilon(t))u(t)
$$

---

## Propulsion Efficiency With Energy Dissipation

$$
\alpha(\varepsilon)
=
\alpha_0
\left(
1-\beta\varepsilon^2
\right)
$$

---

## High-Dimensional Deformation-Aware Dynamics

$$
\dot{x}
=
f(x,u,\varepsilon)
$$

$$
\dot{\varepsilon}
=
g(u,\varepsilon)
$$

---

## Nominal Linear Model

$$
x_{k+1}
=
A x_k
+
B u_k
$$

with:

$$
B
=
\begin{bmatrix}
\overline{\alpha(\varepsilon)}\Delta t \\
0
\end{bmatrix}
$$

---

## Deformation-Dependent System

$$
x_{k+1}
=
A(\varepsilon)x_k
+
B(\varepsilon)u_k
$$

---

## MPC Nominal Prediction

$$
\hat{x}_{k+i|k}
=
A_0^i x_k
+
\sum_{j=0}^{i-1}
A_0^{i-1-j}
B_0u_{k+j}
$$

---

## Actual Deformation-Dependent Trajectory

$$
x_{k+i}
=
\left(
\prod_{j=0}^{i-1}
A(\varepsilon_{k+j})
\right)
x_k
+
\sum_{j=0}^{i-1}
\left(
\prod_{l=j+1}^{i-1}
A(\varepsilon_{k+l})
\right)
B(\varepsilon_{k+j})
u_{k+j}
$$

---

## Prediction Error

$$
e_{k+i}
=
x_{k+i}
-
\hat{x}_{k+i|k}
$$

---

## First-Order Error Bound

For small deformation:

$$
\left\|
e_{k+i}
\right\|
\le
\sum_{j=0}^{i-1}
\left\|
A_0^{i-1-j}
\right\|
\left(
\left\|A_1\right\|
\left\|x\right\|
+
\left\|B_1\right\|
\left\|u\right\|
\right)
\left\|
\varepsilon_{k+i}
\right\|
$$

---

## Deformation Influence in PID Control

$$
e
=
v_{ref}
-
\alpha(\varepsilon)u
$$

This expresses deformation as an equivalent gain attenuation in the rolling-mode PID control framework.

---

# 15. Physics-Informed Learning Extension

This section is not a direct formula from the supplementary material. It is included as a natural future extension for a research repository focused on physics-informed learning, dynamical systems, and control.

## Physics-Informed Loss

$$
\mathcal{L}
=
\mathcal{L}_{data}
+
\lambda_p\mathcal{L}_{physics}
+
\lambda_b\mathcal{L}_{boundary}
+
\lambda_c\mathcal{L}_{control}
$$

---

## PDE-Constrained Dynamics

$$
\frac{\partial z}{\partial t}
+
\mathcal{N}(z)
=
0
$$

---

## Learned Residual Dynamics

$$
x_{k+1}
=
f_{nominal}(x_k,u_k)
+
f_\theta(x_k,u_k)
$$

---

## Physics-Regularized Control Objective

$$
\min_{\theta,u}
\mathcal{L}_{data}
+
\lambda_p
\left\|
\frac{\partial z}{\partial t}
+
\mathcal{N}(z)
\right\|^2
+
\lambda_u
\sum_{k=0}^{N-1}
\left\|
u_k
\right\|^2
$$
