# SRV02 and ROTFLEX Control Systems

<p align="center">
  <img src="assets/diagrams/project-overview.svg" alt="SRV02 and ROTFLEX control-system overview" width="920">
</p>

MATLAB and Simulink implementations for position control of the Quanser SRV02
rotary servo and discrete control of the SRV02-ROTFLEX flexible joint. The
repository connects control theory to controller design, Simulink models, and
recorded experimental behavior.

The project covers LQR design, Ziegler-Nichols reaction-curve tuning, relay
auto-tuning, discrete pole placement, integral action, numerical state
estimation, and the effect of sampling time on closed-loop performance.

## Highlights

- Three position-controller tuning workflows applied to the SRV02 plant.
- A four-state physical model of the SRV02-ROTFLEX mechanism.
- Zero-order-hold discretization and direct digital pole placement.
- Numerical angular-velocity estimation from measured positions.
- Recorded step, ramp, disturbance, control-voltage, and sampling-time results.
- QUARC-ready Simulink models for Q2-USB hardware interfacing.
- A detailed [technical notebook](notebooks/srv02-rotflex-control.ipynb) that
  follows a theory - code - visual result - interpretation sequence.

## System overview

The repository contains two related control studies.

### 1. SRV02 position control

The load-speed model is approximated by

$$
\frac{\Omega_l(s)}{V_m(s)}=\frac{K}{\tau s+1},
$$

and the corresponding position model includes an additional integrator. Three
controller-design approaches are compared:

- Linear Quadratic Regulator (LQR).
- Ziegler-Nichols tuning from the step-response tangent.
- Relay auto-tuning from the critical gain and period.

### 2. SRV02-ROTFLEX digital control

The flexible-joint state is

$$
x^T=\begin{bmatrix}\theta & \alpha & \dot{\theta} & \dot{\alpha}\end{bmatrix},
$$

where $\theta$ is the SRV02 load angle and $\alpha$ is the flexible-joint
deflection. The continuous model is discretized with a zero-order hold, the
controller is designed by pole placement, and the unmeasured angular
velocities are estimated numerically.

<p align="center">
  <img src="assets/diagrams/discrete-controller-model.jpg" alt="Discrete pole-placement and integral-control Simulink model" width="920">
</p>

The block labeled `High-Gain Observer` in the stored model is a legacy name.
In `Q_MDL_DISC.mdl`, its active path uses discrete numerical derivatives; in
`Q_CONTROL_Discret.mdl`, filtered derivatives are used.

## Representative recorded results

The figures below are recorded experiment or model-run captures retained from
the project. They are not regenerated synthetic data.

| LQR step response | Tuned Ziegler-Nichols step response |
|:--:|:--:|
| ![LQR step response](assets/results/position-control/09-lqr-step-response.jpg) | ![Tuned Ziegler-Nichols response](assets/results/position-control/21-zn-attempt-4-step-response.jpg) |

| Digital control at 2 ms | Loss of control at 100 ms |
|:--:|:--:|
| ![Discrete response at 2 ms](assets/results/flexible-joint/36-discrete-motor-angle-ts-0p002.jpg) | ![Unstable response at 100 ms](assets/results/flexible-joint/42-discrete-response-ts-0p1.jpg) |

### Summary

| Study | Recorded outcome |
|---|---|
| LQR position control | 0.174 s rise time, approximately 0.758 s settling time, and low control effort |
| Tuned Ziegler-Nichols | 0.092 s rise time after iterative adjustment to avoid saturation |
| Tuned relay controller | 0.072 s rise time and 1.125% overshoot, with less voltage margin |
| Digital control, $T_s=0.002$ s | Stable response comparable to the continuous controller |
| Digital control, $T_s=0.05$ s | Stable but slower and more oscillatory |
| Digital control, $T_s=0.1$ s | Recorded loss of control |

See the [technical notebook](notebooks/srv02-rotflex-control.ipynb) for the
equations, controller parameters, complete result sequence, and engineering
interpretation.

## Hardware and software

### Core software

- MATLAB R2021a or a release compatible with the installed QUARC version.
- Simulink.
- Control System Toolbox.
- Signal Processing Toolbox for `findpeaks` in the relay-tuning script.

### Hardware workflow

- Quanser SRV02 rotary servo.
- Quanser ROTFLEX or ROTFLEX-E flexible-joint module.
- Quanser Q2-USB data-acquisition device.
- VoltPAQ or a compatible UPM amplifier, according to the selected setup.
- Quanser QUARC Targets and the associated Simulink libraries.

The hardware models were saved with analog output disabled. Before enabling
hardware output, verify the selected amplifier, sensor channels, encoder
direction, emergency stop, mechanical travel, and the configured voltage
limits. Hardware operation requires direct supervision.

## Quick start

Clone the repository and start MATLAB from the repository root:

```bash
git clone https://github.com/yuvalMARMOR/control-systems-srv02-rotflex.git
cd control-systems-srv02-rotflex
```

### Position-control design

```matlab
cd('SRV02-Position-Control')
run('LQR_Controller.m')
run('Q2_ZN.m')
run('Q2_RELAY.m')
```

`Q2_ZN.m` can use recorded `theta_l` and `t` vectors when they are available.
Without them it runs a model-based illustration and reports that the output is
not a reconstruction of the recorded experiment.

### Flexible-joint controller design

```matlab
cd('SRV02-Observer-Control')
run('model3.m')
open_system('Q_MDL_DISC')
```

`model3.m` adds the module path, initializes the SRV02-ROTFLEX configuration,
forms the continuous and discrete state-space models, and computes the nominal
2 ms pole-placement gain.

Offline comparison scripts are also provided:

```matlab
run('sampling_time_comparison.m')
run('observer_estimator_comparison.m')
```

These scripts produce explanatory simulations. They are deliberately kept
separate from the recorded experimental figures.

## Repository structure

```text
.
|-- assets/
|   |-- diagrams/                      # Architecture and Simulink diagrams
|   `-- results/                       # Recorded experiment/model-run figures
|-- notebooks/
|   `-- srv02-rotflex-control.ipynb    # Main technical walkthrough
|-- SRV02-Position-Control/
|   |-- LQR_Controller.m
|   |-- Q2_RELAY.m
|   |-- Q2_ZN.m
|   `-- RELAY_TUNING.slx
|-- SRV02-Observer-Control/
|   |-- model3.m
|   |-- sampling_time_comparison.m
|   |-- observer_estimator_comparison.m
|   |-- Q_CONTROL_Discret.mdl
|   |-- Q_MDL_DISC.mdl
|   `-- Modules/                       # Quanser setup and plant support files
|-- LICENSE
`-- THIRD_PARTY_NOTICES.md
```

## Known limitations

- Raw experiment logs are not available; recorded figures and reported
  numerical metrics are preserved as evidence, but cannot all be recomputed.
- The hardware-linked models require the matching QUARC installation and may
  show unresolved library links on systems without Quanser software.
- The two `.mdl` controller variants contain different derivative filtering
  and integral gains; `Q_MDL_DISC.mdl` is the model aligned with the nominal
  numerical-derivative design presented in the notebook.
- MATLAB and QUARC execution is not currently covered by automated CI.
- The supplemental estimator-comparison script uses explicitly labeled
  synthetic measurement noise and is not an experimental result.

## License

Original project material is released under the [MIT License](LICENSE).
Quanser support files and QUARC-linked material retain their existing notices;
see [Third-Party Notices](THIRD_PARTY_NOTICES.md).
