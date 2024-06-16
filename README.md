### README.md

# Long-Range Communication for Unmanned Surface Vessels (USVs)

## Overview

This repository contains the resources and tools used for the research and implementation of long-range communication solutions for Unmanned Surface Vessels (USVs) in marine environments. The project focused on enhancing WiFi connectivity using private networks, point-to-point WiFi connections with directional antennas, and LoRa communication.

## Directory Structure

```plaintext
├── antenna_tracking
├── calibrate_magneto
├── gps_tx
├── schematics
├── simulation_results
├── Presentation USV.pptx
└── USV Final Report.docx
```

### Folders and Files

- **antenna_tracking**: Contains code and resources for the dual-axis gimbal system used to control the directional antenna.
- **calibrate_magneto**: Includes scripts and tools for calibrating the magnetometer used in the IMU.
- **gps_tx**: Contains resources related to GPS transmission and processing.
- **schematics**: Detailed schematics of the hardware setup for both the base station and the USV.
- **simulation_results**: Results from simulations conducted using the Longley Rice ITM model and other simulation tools.
- **Presentation USV.pptx**: A presentation summarizing the project, methodology, and findings.
- **USV Final Report.docx**: The final report documenting the research, implementation, and evaluation of the communication solutions.

## Getting Started

### Prerequisites

- **Arduino IDE**: Required for programming the Adafruit Feather M0 microcontrollers.
- **Python**: For running custom scripts and data analysis tools.
- **RadioMobile Software**: For simulating communication performance using the Longley Rice ITM model.

### Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/Deathwalker9959/tracking-antenna.git
   ```
2. Navigate to the desired directory:
   ```bash
   cd tracking-antenna/[directory_name]
   ```

### Usage

#### Antenna Tracking

Navigate to the `antenna_tracking` directory and follow the instructions in the README file to set up and run the dual-axis gimbal system.

#### Magnetometer Calibration

In the `calibrate_magneto` directory, you will find scripts for calibrating the magnetometer. Follow the steps provided in the README file within that directory.

#### GPS Transmission

The `gps_tx` directory contains resources and scripts for handling GPS transmission. Detailed usage instructions are included in the README file inside the directory.

#### Schematics

Detailed schematics for the hardware setup can be found in the `schematics` directory. Open the files to view the component layouts and connections.

#### Simulation Results

The `simulation_results` directory contains results from various simulations. Review these files to understand the theoretical performance and limitations of the communication solutions.

## Documentation

- **System Schematics**: Detailed diagrams and component layouts for the hardware setup.
- **Test Procedures**: Step-by-step procedures for simulations, lab tests, and field trials.

## Contributions

Contributions are welcome! Please fork the repository and submit a pull request with your improvements.

## License

This project is licensed under the MIT License. See the `LICENSE` file for details.

## Contact

For any questions or further information, please contact Andreas Malathouras at [your email].

---

Thank you for exploring our project on enhancing long-range communication for Unmanned Surface Vessels (USVs)!