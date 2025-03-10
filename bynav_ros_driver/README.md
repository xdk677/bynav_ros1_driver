# Install Dependencies
Run the script file `install_dependencies.sh`.
1. Online Installation
```shell
./install_dependencies.sh online
```
2. Offline Installation
```shell
./install_dependencies.sh offline
```

# Compilation
Run the script file bash build.sh

# Load Environment
```shell
. devel/setup.bash
```

# Run
Supports two communication methods: serial port and network port.
1. Serial Port Connection
```shell
roslaunch bynav_ros_driver connect_tty.launch
```
2. Serial Port Connection
```shell
roslaunch bynav_ros_driver connect_net.launch
```

You can modify the serial port and network port connection information in the launch file.
You can enable NTRIP and configure NTRIP-related parameters in the `ntrip_parameters.yaml` file under the `config` folder.

**Note:**
1. If you need to use the relevant data of the IMU, you need to configure the IMU frequency and the scale of gyro and acc in the `std_driver_config.xml` file according to the specific IMU model before use.
2. If the script file cannot be executed, you can add executable permissions through the following commands:
```shell
chmod +x build.sh
chmod +x install_dependencies.sh
```