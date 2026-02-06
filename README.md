# ROS 2 Interface Guide for RFT Series (UART Model)


This guide provides instructions for interfacing the **RFT sensor**, which uses **UART communication**, within a **ROS 2** environment.

Steps 2 through 4 require a total of **three terminals**.
Each step must be executed in a **separate terminal**.


---

## System Requirements

* **ROS 2** must be installed
* Verified in a **virtual environment** using **Ubuntu 24.04** with **ROS 2 Jazzy**

---



## 1. Serial Port Low Latency Setting

For stable and responsive communication with the sensor, the serial port should be configured with the **low_latency** option.

---



### 1-1. Identify the Serial Device

First, identify the serial device assigned to the sensor.

Use the following command to list connected serial devices:

```bash
ls /dev/ttyUSB*
ls /dev/ttyACM*
```

Determine which device corresponds to the sensor (for example: `/dev/ttyUSB0`).

---

### 1-2. Check Serial Port Status

Check the current status of the identified serial device.

> You must use the actual device name instead of /dev/ttyUSB0.

```bash
setserial -a /dev/ttyUSB0
```

Check the **`Flags`** field in the output.

* If **`low_latency`** is already present, no further action is required.
* If **`low_latency`** is **not present**, proceed to the next step.

---

### 1-3. Install `setserial` (If Not Installed)

If the `setserial` command is not available, install it using the following command:

```bash
sudo apt update
sudo apt install setserial
```

After installation, recheck the serial port status:

```bash
setserial -a /dev/ttyUSB0
```

---

### 1-4. Enable `low_latency`

Apply the **`low_latency`** option to the serial device.

> You must use the actual device name instead of /dev/ttyUSB0.

```bash
sudo setserial /dev/ttyUSB0 low_latency
```

---

### 1-5. Verify Configuration

Verify that the configuration has been successfully applied:

```bash
setserial -a /dev/ttyUSB0
```

Confirm that **`low_latency`** appears in the **`Flags`** field.

---

### 1-6. Notes

Enabling **`low_latency`** improves serial communication responsiveness.
Device names such as `ttyUSB*` or `ttyACM*` may vary depending on the system and USB chipset.
The configuration may be reset after a system reboot.
If communication issues occur again, reapply the above steps.


---

## 2. ROS 2 Workspace Setup

---

### 2-1. Create your workspace

Create a directory that will contain your ROS 2 workspace.

```bash
mkdir -p ~/ {your workspace}
cd ~/ {your workspace}
```

---

### 2-2. Create the 'src' folder

The 'src' folder is not generated automaticaly. You need to create it manually to store ROS 2 packages.

```bash
mkdir src
```

---

### 2-3. Unzip your ROS 2 package into the 'src' folder

Copy 'rft_sensor_serial.zip' into 'src' and extract it.

```bash
cp ~/downloads/rft_sensor_serial.zip ~/{your workspace}/src/
cd ~/{your workspace}/src
unzip rft_sensor_serial.zip
```

---

### 2-4. Build the workspace

Build all packages in the 'src' folder using 'colcon'.

```bash
cd ~/{your workspace}
colcon build
```

After the build completes, verify that the following directories are created:

* build/

* install/

* log/

---

### 2-5. Source the ROS 2 Environment

Source the ROS 2 setup script to load the core environment, including required paths and command-line tools.

```bash
source /opt/ros/<your_ros2_distro>/setup.bash
```

---

### 2-6. Source the workspace

To use the newly built packages, source the setup script.

```bash
source ./install/setup.bash
```

---

### 2-7. Launches the 'rft_sensor_serial' package

Launch the sensor node and begin publishing RFT sensor data to ROS 2 topics.

```bash
ros2 launch rft_sensor_serial rft_sensor_launch.py
```

### 2-8. Notes

To change the sensor USB port, edit the YAML file located in the config directory of the 'rft_sensor_serial' package.
Update the port value to match the device path where the sensor is detected.

---

## For a detailed usage guide, please refer to the provided PDF documentation.
