"""
Robotic Arm Controller - Python GUI Application

This application provides a graphical interface to control an ESP32-based
robotic arm with 4 servo motors via serial communication.

Features:
- Select between Instant and Smooth movement modes
- Four sliders for controlling each servo (Base, Shoulder, Elbow, Gripper)
- Real-time mode: Move servos as sliders change
- Manual mode: Set angles and send with a button
- Serial communication log for debugging

Serial Command Format:
- Instant movement: "I:base,shoulder,elbow,gripper"
- Smooth movement:  "S:base,shoulder,elbow,gripper"

Author: Your Name
Date: 2024
"""

import tkinter as tk
from tkinter import ttk
import serial
import serial.tools.list_ports
import threading
import queue
import time
from typing import Optional


class SerialManager:
    """
    Handles serial communication with the ESP32.
    
    This class manages the serial connection, sending commands,
    and receiving responses in a thread-safe manner.
    """
    
    def __init__(self, log_callback):
        """
        Initialize the SerialManager.
        
        Args:
            log_callback: Function to call for logging messages to the UI
        """
        self.serial_port: Optional[serial.Serial] = None
        self.log_callback = log_callback
        self.read_thread: Optional[threading.Thread] = None
        self.running = False
        self.rx_queue = queue.Queue()
    
    def get_available_ports(self) -> list:
        """
        Get a list of available COM ports.
        
        Returns:
            List of tuples (port_name, description)
        """
        ports = serial.tools.list_ports.comports()
        return [(port.device, port.description) for port in ports]
    
    def connect(self, port: str, baudrate: int = 115200) -> bool:
        """
        Connect to the specified serial port.
        
        Args:
            port: COM port name (e.g., "COM3")
            baudrate: Baud rate (default: 115200)
            
        Returns:
            True if connection successful, False otherwise
        """
        try:
            # Close existing connection if any
            self.disconnect()
            
            # Open new connection
            self.serial_port = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=0.1,  # 100ms read timeout
                write_timeout=1.0
            )
            
            # Start the read thread
            self.running = True
            self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self.read_thread.start()
            
            self.log_callback(f"Connected to {port} at {baudrate} baud")
            return True
            
        except serial.SerialException as e:
            self.log_callback(f"Connection error: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from the serial port."""
        self.running = False
        
        if self.read_thread and self.read_thread.is_alive():
            self.read_thread.join(timeout=1.0)
        
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.log_callback("Disconnected")
        
        self.serial_port = None
    
    def is_connected(self) -> bool:
        """Check if serial port is connected and open."""
        return self.serial_port is not None and self.serial_port.is_open
    
    def send_command(self, command: str) -> bool:
        """
        Send a command to the ESP32.
        
        Args:
            command: Command string (e.g., "I:90,90,90,90")
            
        Returns:
            True if sent successfully, False otherwise
        """
        if not self.is_connected():
            self.log_callback("Error: Not connected")
            return False
        
        try:
            # Add newline and encode to bytes
            full_command = command + "\n"
            self.serial_port.write(full_command.encode('utf-8'))
            self.log_callback(f"TX: {command}")
            return True
            
        except serial.SerialException as e:
            self.log_callback(f"Send error: {e}")
            return False
    
    def _read_loop(self):
        """
        Background thread that continuously reads from serial port.
        Received lines are put in the rx_queue for the main thread to process.
        """
        while self.running and self.serial_port:
            try:
                if self.serial_port.in_waiting > 0:
                    line = self.serial_port.readline().decode('utf-8').strip()
                    if line:
                        self.rx_queue.put(line)
            except serial.SerialException:
                break
            except Exception:
                pass
            time.sleep(0.01)  # Small delay to prevent CPU hogging


class RoboticArmControllerApp:
    """
    Main application class for the Robotic Arm Controller GUI.
    
    This class creates and manages the Tkinter GUI, handling user
    interactions and coordinating with the SerialManager.
    """
    
    # Servo names for display
    SERVO_NAMES = ["Base", "Shoulder", "Elbow", "Gripper"]
    
    # Debounce delay for real-time mode (milliseconds)
    DEBOUNCE_DELAY_MS = 150  # Increased from 100ms to reduce command frequency
    
    def __init__(self, root: tk.Tk):
        """
        Initialize the application.
        
        Args:
            root: The Tkinter root window
        """
        self.root = root
        self.root.title("Robotic Arm Controller")
        self.root.geometry("600x700")
        self.root.resizable(True, True)
        
        # Serial manager instance
        self.serial_manager = SerialManager(self.log_message)
        
        # Variables for UI state
        self.movement_mode = tk.StringVar(value="instant")  # "instant" or "smooth"
        self.realtime_mode = tk.BooleanVar(value=False)     # Real-time toggle
        self.selected_port = tk.StringVar()
        
        # Slider variables (IntVar for integer angles)
        self.servo_angles = [tk.IntVar(value=90) for _ in range(4)]
        
        # Track last sent angles to avoid sending redundant commands
        self.last_sent_angles = [90, 90, 90, 90]
        
        # Debounce timer ID
        self.debounce_timer_id = None
        
        # Build the UI
        self._create_widgets()
        
        # Start polling for serial responses
        self._poll_serial_rx()
        
        # Refresh available ports
        self._refresh_ports()
        
        # Handle window close
        self.root.protocol("WM_DELETE_WINDOW", self._on_closing)
    
    def _create_widgets(self):
        """Create all UI widgets."""
        
        # Main container with padding
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky="nsew")
        
        # Configure grid weights for resizing
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(0, weight=1)
        
        # === Serial Connection Section ===
        self._create_connection_section(main_frame)
        
        # === Movement Mode Section ===
        self._create_movement_mode_section(main_frame)
        
        # === Servo Sliders Section ===
        self._create_sliders_section(main_frame)
        
        # === Control Buttons Section ===
        self._create_control_section(main_frame)
        
        # === Log Section ===
        self._create_log_section(main_frame)
    
    def _create_connection_section(self, parent):
        """Create the serial connection controls."""
        
        # Frame for connection controls
        conn_frame = ttk.LabelFrame(parent, text="Serial Connection", padding="5")
        conn_frame.grid(row=0, column=0, sticky="ew", pady=(0, 10))
        conn_frame.columnconfigure(1, weight=1)
        
        # Port selection dropdown
        ttk.Label(conn_frame, text="Port:").grid(row=0, column=0, padx=(0, 5))
        
        self.port_combo = ttk.Combobox(
            conn_frame, 
            textvariable=self.selected_port,
            state="readonly",
            width=30
        )
        self.port_combo.grid(row=0, column=1, sticky="ew", padx=5)
        
        # Refresh ports button
        self.refresh_btn = ttk.Button(
            conn_frame, 
            text="↻", 
            width=3,
            command=self._refresh_ports
        )
        self.refresh_btn.grid(row=0, column=2, padx=2)
        
        # Connect/Disconnect button
        self.connect_btn = ttk.Button(
            conn_frame, 
            text="Connect",
            command=self._toggle_connection
        )
        self.connect_btn.grid(row=0, column=3, padx=(5, 0))
        
        # Connection status label
        self.status_label = ttk.Label(
            conn_frame, 
            text="● Disconnected",
            foreground="red"
        )
        self.status_label.grid(row=1, column=0, columnspan=4, pady=(5, 0))
    
    def _create_movement_mode_section(self, parent):
        """Create the movement mode selection."""
        
        mode_frame = ttk.LabelFrame(parent, text="Movement Mode", padding="5")
        mode_frame.grid(row=1, column=0, sticky="ew", pady=(0, 10))
        
        # Radio buttons for movement mode
        self.instant_radio = ttk.Radiobutton(
            mode_frame,
            text="Instant (I:) - Jump directly to position",
            variable=self.movement_mode,
            value="instant"
        )
        self.instant_radio.grid(row=0, column=0, sticky="w", padx=10)
        
        self.smooth_radio = ttk.Radiobutton(
            mode_frame,
            text="Smooth (S:) - Gradual movement",
            variable=self.movement_mode,
            value="smooth"
        )
        self.smooth_radio.grid(row=1, column=0, sticky="w", padx=10)
        
        # Real-time mode toggle
        ttk.Separator(mode_frame, orient="horizontal").grid(
            row=2, column=0, sticky="ew", pady=10
        )
        
        self.realtime_check = ttk.Checkbutton(
            mode_frame,
            text="Real-time Mode (send commands as sliders move)",
            variable=self.realtime_mode,
            command=self._on_realtime_toggle
        )
        self.realtime_check.grid(row=3, column=0, sticky="w", padx=10)
    
    def _create_sliders_section(self, parent):
        """Create the servo angle sliders."""
        
        sliders_frame = ttk.LabelFrame(parent, text="Servo Angles", padding="10")
        sliders_frame.grid(row=2, column=0, sticky="ew", pady=(0, 10))
        sliders_frame.columnconfigure(1, weight=1)
        
        self.sliders = []
        self.angle_labels = []
        
        for i, name in enumerate(self.SERVO_NAMES):
            # Servo name label
            ttk.Label(
                sliders_frame, 
                text=f"{name}:",
                width=10
            ).grid(row=i, column=0, sticky="w")
            
            # Slider (Scale widget)
            slider = ttk.Scale(
                sliders_frame,
                from_=0,
                to=180,
                orient="horizontal",
                variable=self.servo_angles[i],
                command=lambda val, idx=i: self._on_slider_change(idx, val)
            )
            slider.grid(row=i, column=1, sticky="ew", padx=10, pady=5)
            self.sliders.append(slider)
            
            # Current angle value label
            angle_label = ttk.Label(
                sliders_frame, 
                text="90°",
                width=5
            )
            angle_label.grid(row=i, column=2, sticky="e")
            self.angle_labels.append(angle_label)
            
            # Bind variable trace to update label
            self.servo_angles[i].trace_add(
                "write", 
                lambda *args, idx=i: self._update_angle_label(idx)
            )
    
    def _create_control_section(self, parent):
        """Create the control buttons."""
        
        control_frame = ttk.Frame(parent)
        control_frame.grid(row=3, column=0, sticky="ew", pady=(0, 10))
        control_frame.columnconfigure(0, weight=1)
        control_frame.columnconfigure(1, weight=1)
        control_frame.columnconfigure(2, weight=1)
        
        # Send Command button (force=True to always send when manually clicked)
        self.send_btn = ttk.Button(
            control_frame,
            text="Send Command",
            command=lambda: self._send_command(force=True)
        )
        self.send_btn.grid(row=0, column=0, padx=5, sticky="ew")
        
        # Home position button
        self.home_btn = ttk.Button(
            control_frame,
            text="Home (90°)",
            command=self._go_home
        )
        self.home_btn.grid(row=0, column=1, padx=5, sticky="ew")
        
        # Query position button
        self.query_btn = ttk.Button(
            control_frame,
            text="Query Position (?)",
            command=self._query_position
        )
        self.query_btn.grid(row=0, column=2, padx=5, sticky="ew")
    
    def _create_log_section(self, parent):
        """Create the communication log area."""
        
        log_frame = ttk.LabelFrame(parent, text="Communication Log", padding="5")
        log_frame.grid(row=4, column=0, sticky="nsew", pady=(0, 10))
        log_frame.columnconfigure(0, weight=1)
        log_frame.rowconfigure(0, weight=1)
        
        # Configure parent to expand log section
        parent.rowconfigure(4, weight=1)
        
        # Text widget for log with scrollbar
        self.log_text = tk.Text(
            log_frame,
            height=10,
            state="disabled",
            wrap="word",
            font=("Consolas", 9)
        )
        self.log_text.grid(row=0, column=0, sticky="nsew")
        
        scrollbar = ttk.Scrollbar(
            log_frame, 
            orient="vertical",
            command=self.log_text.yview
        )
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.log_text.configure(yscrollcommand=scrollbar.set)
        
        # Clear log button
        clear_btn = ttk.Button(
            log_frame,
            text="Clear Log",
            command=self._clear_log
        )
        clear_btn.grid(row=1, column=0, columnspan=2, pady=(5, 0))
    
    # ==================== Event Handlers ====================
    
    def _refresh_ports(self):
        """Refresh the list of available COM ports."""
        ports = self.serial_manager.get_available_ports()
        
        if ports:
            # Format: "COM3 - USB Serial Device"
            port_strings = [f"{port} - {desc}" for port, desc in ports]
            self.port_combo['values'] = port_strings
            
            # Select first port if none selected
            if not self.selected_port.get() and port_strings:
                self.port_combo.current(0)
                
            self.log_message(f"Found {len(ports)} serial port(s)")
        else:
            self.port_combo['values'] = []
            self.log_message("No serial ports found")
    
    def _toggle_connection(self):
        """Connect or disconnect from serial port."""
        if self.serial_manager.is_connected():
            self.serial_manager.disconnect()
            self._update_connection_status(False)
        else:
            # Extract port name from combo selection (e.g., "COM3 - Description")
            selection = self.selected_port.get()
            if selection:
                port = selection.split(" - ")[0]
                if self.serial_manager.connect(port):
                    self._update_connection_status(True)
            else:
                self.log_message("Please select a port first")
    
    def _update_connection_status(self, connected: bool):
        """Update UI elements based on connection status."""
        if connected:
            self.status_label.configure(text="● Connected", foreground="green")
            self.connect_btn.configure(text="Disconnect")
        else:
            self.status_label.configure(text="● Disconnected", foreground="red")
            self.connect_btn.configure(text="Connect")
    
    def _on_realtime_toggle(self):
        """Handle real-time mode toggle."""
        if self.realtime_mode.get():
            self.log_message("Real-time mode enabled - commands sent as sliders move")
            # Optionally disable the send button visual (but keep functional)
            self.send_btn.configure(state="disabled")
        else:
            self.log_message("Real-time mode disabled - use Send button")
            self.send_btn.configure(state="normal")
    
    def _on_slider_change(self, servo_index: int, value: str):
        """
        Handle slider value change.
        
        In real-time mode, this triggers a debounced command send.
        
        Args:
            servo_index: Which servo slider changed (0-3)
            value: New value as string (from Scale widget)
        """
        # Update the angle label
        self._update_angle_label(servo_index)
        
        # If real-time mode is enabled, schedule a command send
        if self.realtime_mode.get():
            # Cancel any pending debounce timer
            if self.debounce_timer_id:
                self.root.after_cancel(self.debounce_timer_id)
            
            # Schedule new send after debounce delay
            self.debounce_timer_id = self.root.after(
                self.DEBOUNCE_DELAY_MS,
                self._send_command
            )
    
    def _update_angle_label(self, servo_index: int):
        """Update the angle display label for a servo."""
        angle = self.servo_angles[servo_index].get()
        self.angle_labels[servo_index].configure(text=f"{angle}°")
    
    def _send_command(self, force: bool = False):
        """
        Build and send the servo command.
        
        Args:
            force: If True, send even if angles haven't changed (used by manual button)
        """
        # Get current angles from all sliders
        angles = [var.get() for var in self.servo_angles]
        
        # Check if angles have changed (skip if unchanged to prevent jitter)
        if not force and angles == self.last_sent_angles:
            # Angles unchanged, skip sending to prevent servo jitter
            return
        
        # Determine command prefix based on movement mode
        if self.movement_mode.get() == "instant":
            prefix = "I"
        else:
            prefix = "S"
        
        # Build command string: "I:90,90,90,90" or "S:90,90,90,90"
        command = f"{prefix}:{angles[0]},{angles[1]},{angles[2]},{angles[3]}"
        
        # Send via serial
        if self.serial_manager.send_command(command):
            # Update last sent angles only if send was successful
            self.last_sent_angles = angles.copy()
    
    def _go_home(self):
        """Set all sliders to 90° and send home command."""
        for var in self.servo_angles:
            var.set(90)
        
        # Update last sent angles
        self.last_sent_angles = [90, 90, 90, 90]
        
        # Send home command directly
        self.serial_manager.send_command("H")
    
    def _query_position(self):
        """Query current servo positions from ESP32."""
        self.serial_manager.send_command("?")
    
    def _poll_serial_rx(self):
        """
        Poll the serial receive queue and display received messages.
        This runs periodically via Tkinter's after() mechanism.
        """
        try:
            while True:
                # Non-blocking get from queue
                line = self.serial_manager.rx_queue.get_nowait()
                self.log_message(f"RX: {line}")
        except queue.Empty:
            pass
        
        # Schedule next poll (every 50ms)
        self.root.after(50, self._poll_serial_rx)
    
    def log_message(self, message: str):
        """
        Add a message to the log text widget.
        
        Args:
            message: Message string to display
        """
        self.log_text.configure(state="normal")
        
        # Add timestamp
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.insert("end", f"[{timestamp}] {message}\n")
        
        # Auto-scroll to bottom
        self.log_text.see("end")
        
        self.log_text.configure(state="disabled")
    
    def _clear_log(self):
        """Clear the log text widget."""
        self.log_text.configure(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.configure(state="disabled")
    
    def _on_closing(self):
        """Handle application close."""
        self.serial_manager.disconnect()
        self.root.destroy()


def main():
    """Application entry point."""
    root = tk.Tk()
    app = RoboticArmControllerApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
