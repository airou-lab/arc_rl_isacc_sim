import omni.ui as ui
import torch

class TelemetryWindow:
    def __init__(self, window_name="Robot Telemetry"):
        print(f"Initializing Telemetry Window: {window_name}")
        self.window = ui.Window(window_name, width=350, height=450)
        self.speed = 0.0
        self.steering = 0.0
        self.lat_err = "N/A"
        self.step = 0
        self.jv_drive = [0.0, 0.0, 0.0, 0.0]
        
        with self.window.frame:
            with ui.VStack(spacing=10, height=0):
                ui.Label("F1Tenth GSD Telemetry", style={"font_size": 20, "color": 0xFF00FFFF})
                
                with ui.HStack(height=0):
                    ui.Label("Step:", width=100)
                    self.step_label = ui.Label("0")
                
                with ui.HStack(height=0):
                    ui.Label("Linear Velocity:", width=100)
                    self.speed_label = ui.Label("0.00 m/s")
                
                with ui.HStack(height=0):
                    ui.Label("Steer Command:", width=100)
                    self.steering_label = ui.Label("0.000 rad")
                
                with ui.HStack(height=0):
                    ui.Label("Lateral Error:", width=100)
                    self.lat_err_label = ui.Label("N/A")
                
                ui.Spacer(height=10)
                ui.Label("Wheel Speeds (rad/s):", style={"font_size": 14})
                
                with ui.HStack(height=0):
                    ui.Label("FL:", width=50)
                    self.jv_fl_label = ui.Label("0.0")
                    ui.Label("FR:", width=50)
                    self.jv_fr_label = ui.Label("0.0")
                
                with ui.HStack(height=0):
                    ui.Label("RL:", width=50)
                    self.jv_rl_label = ui.Label("0.0")
                    ui.Label("RR:", width=50)
                    self.jv_rr_label = ui.Label("0.0")
                
                ui.Spacer(height=15)
                ui.Label("GSD Phase 5-02 Status:", style={"font_size": 12, "color": 0xFF00FF00})
                ui.Label("- Protocol: 12-Float (Legacy)", style={"font_size": 10})
                ui.Label("- Drive: FWD (FL/FR)", style={"font_size": 10})
                ui.Label("- Reward: Original Composite", style={"font_size": 10})
                ui.Label("- Metric Scale: 1.0x", style={"font_size": 10})

    def update(self, step, speed, steering, lat_err, jv_drive):
        self.step_label.text = str(step)
        self.speed_label.text = f"{speed:.2f} m/s"
        self.steering_label.text = f"{steering:.3f} rad"
        self.lat_err_label.text = str(lat_err)
        
        self.jv_fl_label.text = f"{jv_drive[0]:.1f}"
        self.jv_fr_label.text = f"{jv_drive[1]:.1f}"
        self.jv_rl_label.text = f"{jv_drive[2]:.1f}"
        self.jv_rr_label.text = f"{jv_drive[3]:.1f}"
