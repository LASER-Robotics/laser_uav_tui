# Laser UAV TUI

*The main TUI dashboard displaying real-time telemetry, hardware diagnostics, and topic monitoring for a single UAV in a ROS 2 environment.*

**Laser UAV TUI** is a lightweight, `curses`-based Terminal User Interface (TUI) for monitoring and controlling multiple unmanned aerial vehicles (UAVs) in a ROS 2 environment. It provides real-time system and telemetry monitoring, diagnostic checks, and a command interface directly from your terminal.

![Image of the laser_uav_tui in operation.](https://github.com/LASER-Robotics/laser_uav_tui/blob/dev/increase_view/images/main_interface.png)


## Features

* **Auto-Discovery:** Automatically detects and monitors up to 3 UAVs running on the network by scanning for specific `estimation` topics.
* **Real-Time Telemetry:** Displays current Pose (X, Y, Z, Heading), publishing rates (Hz), and speeds.
* **Hardware & API Diagnostics:** Monitors PX4 satellite counts, RF jamming status, and preflight check validations (UI blinks if checks fail).
* **Control Interface:** Provides an interactive menu to send commands directly to the UAVs.
* **Dynamic Topic Monitoring:** Monitor the frequency (Hz) of arbitrary ROS 2 topics by specifying them in a YAML configuration file.
* **System Stats:** Tracks host CPU and RAM usage in real-time.


## Usage

Run the TUI node using the standard ROS 2 run command. To enable dynamic topic monitoring, pass the path to your `config.yaml` file as an argument.

```bash
ros2 run laser_uav_tui tui.py --config $(ros2 pkg prefix laser_uav_tui)/share/laser_uav_tui/config/config.yaml

```

**Keyboard Controls:**

![laser_uav_tui system menu image ](https://github.com/LASER-Robotics/laser_uav_tui/blob/dev/increase_view/images/menu_interface.png)

* **`UP` / `DOWN` Arrows**: Select a different UAV from the list.
* **`M` or `m**`: Open the Action Menu for the selected UAV.
* **`Enter`**: Execute the selected action or confirm a Go-To coordinate.
* **`ESC`**: Exit the current menu or cancel coordinate input.
* **`Ctrl + C`**: Gracefully exit the application.

**Interactive Control Menus:**

Pressing **`M`** opens the action menu for the selected UAV. From here, you can trigger specific service calls (Arm, Takeoff, Land, Disarm) or navigate to the "GoTo" submenu to send specific coordinates.

![laser_uav_tui system goto menu image ](https://github.com/LASER-Robotics/laser_uav_tui/blob/dev/increase_view/images/goto_interface.png)

## Configuration (`config.yaml`):

You can monitor the publishing rates of extra topics by adding them to the `config.yaml` file. The TUI will automatically resolve the message types and display their Hz in the "Topic Monitor" box shown in the main interface image.

Example `config/config.yaml`:

```yaml
topics:
  - "/uav1/camera/image_raw"
  - "/uav2/lidar/scan"
  - "/uav1/battery_status"

```

*Note: The script dynamically maps these topics to the respective UAV based on the namespace (e.g., `uav1`, `uav2`).*

