# SRV02 Rotflex Control Systems

MATLAB & Simulink projects for SRV02 position control and flexible joint control.  
Includes implementation of several classic control strategies:

- **LQR Controller**  
- **Ziegler–Nichols Relay Auto-Tuning**  
- **Observer-based control**

---

## 📂 Repository Structure

```
control-systems-srv02-rotflex/
│
├── SRV02-Position-Control/        # Project 1: Position control
│   ├── LQR_Controller.m
│   ├── Q2_RELAY.m
│   ├── Q2_ZN.m
│   ├── RELAY_TUNING.slx
│   └── RELAY_TUNING.slxc
│
├── SRV02-Observer-Control/        # Project 2: Observer-based control
│   ├── COMPLATE.m
│   ├── Q_CONTROL_Discret.mdl
│   ├── Q_MDL_DISC.mdl
│   ├── model3.m
│   ├── observer_based-control.m
│   └── MODULE/                    # Support functions & constants
│       ├── SRV02_ROTFLEX_ABCD_eqns.m
│       ├── calc_conversion_constants.m
│       ├── config_rotflex.m
│       ├── config_srv02.m
│       ├── plot_power_spectrum.m
│       ├── q_srv02_rotflex.mdl
│       ├── q_srv02_rotflex_stiffness.mdl
│       └── setup_srv02_exp05_rotflex.m
│
├── docs/                          # Reports & documentation
│   └── SRV02-Projects-Summary-Report.pdf
│
└── README.md                      # Project documentation (this file)
```

---

## ⚙️ Requirements

- MATLAB (R2021a or later recommended)  
- Simulink toolbox  
- Control System Toolbox  

---

## 🚀 How to Run

1. Clone the repository:
   ```bash
   git clone https://github.com/<your-username>/control-systems-srv02-rotflex.git
   cd control-systems-srv02-rotflex
   ```

2. Open MATLAB in the project directory.

3. For **Position Control**:
   - Open `SRV02-Position-Control/RELAY_TUNING.slx` or run scripts like `Q2_RELAY.m`.

4. For **Observer-Based Control**:
   - Open `SRV02-Observer-Control/model3.m` or the Simulink models in the `MODULE/` folder.

---

## 📑 Documentation

Full summary report is available under:  
[`docs/SRV02-Projects-Summary-Report.pdf`](docs/SRV02-Projects-Summary-Report.pdf)

---

## 👥 Authors

Project developed as part of the academic track in Control, Mechatronics, and Robotics.  
Contributors: **Yuval Marmor**, **Rabea Hawa**.  
