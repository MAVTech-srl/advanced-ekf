# Advanced EKF for UAVs

This repo is the development environment for the advanced-ekf ROS2 package. The EKF is used for state estimation and it uses the full dynamical system of a UAV, plus the input forces and torques given by the propellers. The advanced EKF leverages on the [IKFoM Toolkit](https://github.com/hku-mars/IKFoM) by the University of Hong Kong. Hopefully, it can achieve higher accuracy in state estimation.

## Table of Contents
- [Installation](#installation)
- [Usage](#usage)
- [Logging](#logging)
- [Contact](#contact)

## Installation
Clone this repository and all its submodules.
```bash
# Clone the repository
git clone https://github.com/MAVTech-srl/advanced-ekf
cd advanced-ekf

# Clone submodules
git submodule update --init --recursive
```

## Usage
This project uses a Dev Container as development environment. The Dev Container is compatible for both AMD64 and ARM64 CPU architectures. To build and open the container, create the *ad-hoc* `devcontainer.json` running the provided `bash` script in the `.devcontainer` folder:
```bash
# This creates the devcontainer.json file used by VS Code
bash setup_devcontainer.sh
```
Then open VS Code and open the project in the container. The correct `Dockerfile` will automatically be selected based on the host capabilities.

## Logging
The advanced-ekf ROS2 package comes with two data visualization tools:
- Plotjuggler (self-contained in Docker) to visualize both `.csv` files and ROS2 messages in real time
- `plot_csv.py` to visualize the `.csv` file created by the advanced-ekf node when `save_log` parameter in `src/advanced-ekf/config/default.yaml` file is set to `true`. This script runs a Dash-Plotly webapp at `localhost:8050` in which it is possible to open the log files in the `logs/` folder.
Before using `plot_csv.py`, create a `venv` and install the pip packages listed in `requirements.txt`.

# Contact
- [Davide](https://github.com/DavideCarminati)