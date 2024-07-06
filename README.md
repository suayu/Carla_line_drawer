# Carla Line Drawing Tool

## Introduction

This plugin allows you to control an overhead camera in the Carla simulator, draw lines in the Carla simulator, and save reference points.

## Installation

### Import Conda Environment

To install the required environment, run:
```
pip3 install -r requirements.txt
```

### Copy the Script

Copy `cameras.py` to the `Carla/PythonAPI/examples` directory.

### Start Carla

To start the Carla simulator, run:
```
./CarlaUE4.sh
```

## Usage

### Start the Tool

To start the line tool, run:
```
python cameras.py
```

After starting the tool, it will load a map in the Carla simulator (default is Town01).   A new window will open and display a camera view.  The default camera height is `z=50`, and the default window size is `800×800`.

### Camera Controls

- Use `W`, `A`, `S`, `D` keys to control the camera's horizontal position.
- Use the mouse wheel to control the camera height.

### Drawing Lines

In the new window, you can draw lines with the mouse.  The tool will draw red lines and a series of reference points in the Carla simulator at the corresponding x and y coordinates.  When drawing multiple lines, new lines will start from the last point of the previous line.

### Keyboard Shortcuts

- Press `P` to clear the lines and reference points.
- Press `C` to save the list of reference points, both in the simulator's coordinates and relative to the ego vehicle's position.
- Press `H` to reset the camera height to the default value.
- Press `Y` to calculate yaw of ego car at the reference point
- Press `B` to smooth the curve