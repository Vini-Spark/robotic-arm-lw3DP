# Robotic Arm Controller - Python GUI

A Python application with a graphical interface to control an ESP32-based robotic arm via serial communication.

## Features

- **Movement Modes**: Choose between Instant (immediate) or Smooth (gradual) movement
- **4 Servo Sliders**: Control Base, Shoulder, Elbow, and Gripper angles (0-180°)
- **Real-time Mode**: Move servos as you drag the sliders (with debounce)
- **Manual Mode**: Set angles and send with a button
- **Serial Port Selection**: Dropdown to select available COM ports
- **Communication Log**: View sent and received messages for debugging

## Requirements

- Python 3.9+
- Poetry (for dependency management)

## Installation

1. Navigate to the `python_app` folder:
   ```bash
   cd python_app
   ```

2. Install dependencies with Poetry:
   ```bash
   poetry install
   ```

3. Activate the virtual environment:
   ```bash
   poetry shell
   ```

## Running the Application

### Option 1: Using Poetry
```bash
poetry run python robotic_arm_controller.py
```

### Option 2: After activating the shell
```bash
poetry shell
python robotic_arm_controller.py
```

## Usage

1. **Connect**: Select your ESP32's COM port from the dropdown and click "Connect"
2. **Set Movement Mode**: Choose "Instant" or "Smooth" using the radio buttons
3. **Control Servos**: 
   - Move the sliders to set desired angles
   - Click "Send Command" to move the arm (Manual mode)
   - OR enable "Real-time Mode" to move as you drag sliders
4. **Special Commands**:
   - **Home (90°)**: Reset all servos to 90°
   - **Query Position (?)**: Ask ESP32 for current positions

## Serial Protocol

| Command | Format | Example |
|---------|--------|---------|
| Instant Move | `I:base,shoulder,elbow,gripper` | `I:90,45,120,60` |
| Smooth Move | `S:base,shoulder,elbow,gripper` | `S:90,45,120,60` |
| Home | `H` | `H` |
| Query | `?` | `?` |

## Responses from ESP32

| Response | Meaning |
|----------|---------|
| `OK:90,45,120,60` | Command executed successfully |
| `OK:HOME,90,90,90,90` | Home command executed |
| `POS:90,45,120,60` | Response to position query |
| `ERR:message` | Error occurred |
