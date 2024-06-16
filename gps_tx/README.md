### README.md

# GPS Transmission Module

## Overview

This directory contains the code and resources for handling GPS transmission in the antenna tracking system. The GPS transmission module ensures accurate and reliable transmission of GPS data from the Unmanned Surface Vessel (USV) to the base station.

## Directory Structure

```plaintext
├── .vscode
├── gps_tx.ino
└── packages.microsoft.gpg
```

### Folders and Files

- **.vscode**: Configuration files for the Visual Studio Code IDE.
- **gps_tx.ino**: The main Arduino sketch for handling GPS data transmission.
- **packages.microsoft.gpg**: GPG keys for package verification (specific to Microsoft packages).

## Getting Started

### Prerequisites

- **Arduino IDE**: Required for programming the microcontroller.
- **GPS Module**: Ensure you have a compatible GPS module connected to your microcontroller.
- **LoRa Module**: Required for long-range communication of GPS data.

### Installation

1. **Clone the repository:**
   ```bash
   git clone https://github.com/Deathwalker9959/tracking-antenna.git
   cd tracking-antenna/gps_tx
   ```

2. **Open the Arduino sketch:**
   - Open `gps_tx.ino` in the Arduino IDE.

3. **Configure the Serial Port:**
   - Ensure the correct serial port is selected in the Arduino IDE.

4. **Upload the code:**
   - Connect your microcontroller to your computer.
   - Select the appropriate board and port in the Arduino IDE.
   - Upload the `gps_tx.ino` sketch to the microcontroller.

### Usage

1. **Initialize the System:**
   - Power on the microcontroller with the GPS module connected.
   - Open the serial monitor in the Arduino IDE to view GPS data output.

2. **Transmit GPS Data:**
   - The GPS data will be transmitted via the connected LoRa module.
   - Ensure the base station is set up to receive and process the incoming GPS data.

3. **Monitor Output:**
   - Check the serial monitor for real-time GPS data and transmission status.
   - Verify that the data is being correctly transmitted and received.

### Files Description

- **gps_tx.ino**: Contains the main logic for initializing the GPS module, reading GPS data, and transmitting it via the LoRa module.
- **packages.microsoft.gpg**: GPG keys for package verification, typically used for securing package installations.

## Contribution

Contributions are welcome! Please fork the repository and submit a pull request with your improvements.

## License

This project is licensed under the MIT License. See the `LICENSE` file for details.

## Contact

For any questions or further information, please contact Andreas Malathouras at [your email].

---

Thank you for using the GPS Transmission Module to enhance the accuracy and reliability of your antenna tracking system!