# Simulation Logging Plugin for CoppeliaSim

This ROS2 Humble package implements a plugin that logs simulation data from Gazebo Classic. The plugin records data such as simulation time, system time, frame rate, and scene object information (including pose and collisions) into a CSV file during the simulation.

## Features
- **Logs simulation data to CSV**: The plugin records the following data for each frame during the simulation:
  - Frame number
  - Simulation time (in milliseconds)
  - System time (in milliseconds)
  - Frames per second (FPS)
  - Active scene objects (including pose information)
  - Collision count
  - Simulation step size (in milliseconds)

- **CSV File Naming**: The CSV file is automatically named based on the current date and time when the simulation starts.

## Getting Started

### Prerequisites
- **ROS2 Humble**: The package is designed to run in a ROS2 Humble environment.
- **Gazebo Classic**: The plugin requires Gazebo classic.
- **CMake**: For building the project manually.

### Installation via ROS2 Workspace

1. Clone this repository into your ROS2 workspace:
    ```bash
    cd ~/ros2_ws/src
    git clone <repo-url>
    ```

2. Build the package using `colcon`:
    ```bash
    cd ~/ros2_ws
    colcon build
    ```

3. Source the workspace:
    ```bash
    source ~/ros2_ws/install/setup.bash
    ```

### Installation via CMake

If you prefer to build the package with `CMake` outside of a ROS2 workspace, follow these steps:

1. Clone the repository:
    ```bash
    git clone <repo-url>
    cd <repo-directory>
    ```

2. Create a `build` directory and navigate to it:
    ```bash
    mkdir build
    cd build
    ```

3. Configure and build the project using CMake:
    ```bash
    cmake ..
    make
    ```

4. After building, you can run the plugin as described in the **Running the Plugin** section.

### Running the Plugin

1. Start a CoppeliaSim simulation.

2. The plugin will automatically begin logging data when the simulation starts. A CSV file will be created in the same directory, named with the current date and time, e.g., `gazeboLogger_2024-09-16_12-30-45.csv`.

3. The CSV file will log:
   - **Frame number**: Sequential count of frames.
   - **Simulation Time**: Time elapsed in the simulation (in milliseconds).
   - **System Time**: Time elapsed since the plugin was initialized (in milliseconds).
   - **FPS**: Frames per second.
   - **Active Objects**: JSON-formatted list of active objects in the scene, including object aliases and poses.
   - **Collision Count**: Number of collisions detected with the `/Floor` object.
   - **Step Size**: The size of the current simulation step (in milliseconds).

### Example CSV Output
Here’s an example of the data logged in the CSV file:

```
Frame;Simulation Time (ms);System Time (ms);FPS (Hz);Active Objects;Collision Count;Step Size (ms)
1;500;502;60;[{ "alias": "Robot", "pose": [0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 1.0]}];0;50
2;1000;1005;60;[{ "alias": "Robot", "pose": [0.1, 0.0, 0.1, 0.0, 0.0, 0.0, 1.0]}];0;50
```

### Clean Up

When the simulation ends, the plugin will automatically close the CSV file and reset internal counters.

### Configuration

You can customize the logging behavior by modifying the source code to adjust:
- Collision detection for specific objects.
- Custom logging parameters and formats.

## Troubleshooting

- **CSV file not created**: Ensure that the plugin is successfully loaded into CoppeliaSim and that the simulation is running.
- **File permissions**: Ensure that the plugin has write permissions to the directory where the simulation is running.

## Contributing

If you'd like to contribute to this project, feel free to submit issues or pull requests via GitHub.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.