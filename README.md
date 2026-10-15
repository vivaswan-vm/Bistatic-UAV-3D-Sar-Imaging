# Bistatic UAV-Based 3D SAR Imaging Using ISAC Transceivers

Official MATLAB simulation code for:

> **Bistatic UAV-Based 3D SAR Imaging Using Integrated Sensing and Communications Transceivers**  
> Akanksha Sneh, Krishna Ayyagari\*, Vivaswan Nawani\*, and Shobha Sundar Ram  
> *\* Krishna Ayyagari and Vivaswan Nawani are joint second authors.*  
> Submitted to **IEEE Transactions on Aerospace and Electronic Systems (T-AES)**.

---

## Description

This repository implements a bistatic UAV tomographic SAR framework using IEEE 802.11ad ISAC waveforms for joint communication and radar sensing. It includes:
- Communication-assisted synchronization (timing, CFO, phase offset) for a passive receiver UAV.
- Two-stage motion compensation (coarse geometric compensation and fine autofocus).
- 3D target localization via sphere-plane intersection using monostatic and bistatic range profiles.
- Benchmark comparison against conventional InSAR height estimation.

---

## Requirements

- MATLAB (tested on R2024b / R2025a)
- Toolboxes:
  - WLAN Toolbox
  - Communications Toolbox
  - Phased Array System Toolbox
  - Signal Processing Toolbox
  - Statistics and Machine Learning Toolbox

---

## Repository Structure

```
├── data/
│   ├── car_rcs.mat                      # 3D facet RCS data of automotive target
│   ├── car_x.mat, car_y.mat, car_z.mat  # Target scatterer coordinates
│   └── ISAC_waveform_v2.mat             # IEEE 802.11ad baseband waveform
├── results/
│   ├── Combined_RMSE_Results.mat        # Pre-computed Monte Carlo results
│   └── RMSE_X_Y_X_Plots.png             # Generated RMSE plot (Fig. 5 in paper)
├── src/
│   ├── comms/                           # 802.11ad framing, channel, synchronization & BER
│   ├── experiments/                     # Exp 1 (Bistatic 3D SAR) & Exp 2 (InSAR)
│   ├── init/                            # Radar, platform, and scenario initializations
│   ├── plotting_code/                   # Results plotting
│   ├── sar/                             # Monostatic/bistatic SAR focusing & InSAR
│   └── utils/                           # Terrain model, clutter generation, utilities
├── master_code.m                        # Monte Carlo simulation script
├── LICENSE                              # Apache 2.0 License
└── README.md
```

---

## Usage

### 1. Plot Pre-computed Results
To plot the 3D localization RMSE curves (Fig. 5 in the paper) using the saved Monte Carlo data:
```matlab
addpath(genpath('src'));
plot_results;
```

### 2. Run Full Simulation
To run the full Monte Carlo simulation:
```matlab
master_code;
```
*Note: A full sweep across 120 iterations and 6 velocity perturbations runs 2,880 SAR executions. Set `Number_of_Iterations` in `master_code.m` to a smaller value for faster test runs.*

---

## Citation

```bibtex
@article{sneh2026bistatic,
  title={Bistatic UAV-Based 3D SAR Imaging Using Integrated Sensing and Communications Transceivers},
  author={Akanksha Sneh and Krishna Ayyagari and Vivaswan Nawani and Shobha Sundar Ram},
  journal={IEEE Transactions on Aerospace and Electronic Systems},
  year={2026},
  note={Submitted}
}
```

---

## License

This project is licensed under the Apache License 2.0. See [LICENSE](LICENSE) for details.
