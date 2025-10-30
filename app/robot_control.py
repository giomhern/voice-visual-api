# robot_control.py
import os

class RobotManager:
    def __init__(self):
        self.robot = None
        self.started = False
        self.last_error = None

    def startup(self):
        """
        Initialize the Stretch robot & enable motors (no motion).
        Returns (ok, message, meta)
        """
        if self.started:
            return True, "Robot already started.", {"already_started": True}

        try:
            # Import lazily so audio-only runs don’t require the SDK
            import stretch_body.robot as hello_robot
            self.robot = hello_robot.Robot()
            if not self.robot.startup():
                self.last_error = "robot.startup() returned False"
                return False, "Failed to startup Stretch SDK.", {"error": self.last_error}

            # Check for E-Stop / runstop
            # If runstop is pressed, motors can't be enabled.
            if hasattr(self.robot, "pimu"):
                self.robot.pimu.get_status()
                if getattr(self.robot.pimu.status, "runstop_event", 0) == 1:
                    self.last_error = "E-Stop / runstop is engaged"
                    return False, "Runstop is pressed; release it and try again.", {"runstop": True}

            # Enable motors (no motion commands here)
            if hasattr(self.robot, "pimu"):
                self.robot.pimu.enable_motors()

            self.started = True
            return True, "Robot initialized and motors enabled.", {"motors_enabled": True}
        except Exception as e:
            self.last_error = str(e)
            self.robot = None
            return False, f"Startup error: {e}", {"exception": self.last_error}

    def shutdown(self):
        """
        Disable motors and shut down the SDK.
        Returns (ok, message, meta)
        """
        if not self.started:
            return True, "Robot already stopped.", {"already_stopped": True}
        try:
            if self.robot:
                try:
                    # Best-effort: disable motors if possible
                    if hasattr(self.robot, "pimu"):
                        self.robot.pimu.disable_motors()
                except Exception:
                    pass
                self.robot.stop()
                self.robot = None
            self.started = False
            return True, "Robot shut down.", {}
        except Exception as e:
            self.last_error = str(e)
            return False, f"Shutdown error: {e}", {"exception": self.last_error}

    # OPTIONAL: Home/stow example (commented out to avoid accidental motion)
    def home(self):
        if not self.started or not self.robot:
            return False, "Robot not started.", {}
        try:
            # Example: stow the robot (safe compact pose)
            # Many Stretch images include helper methods; if not, issue explicit joint moves.
            self.robot.stow()  # WARNING: this moves the robot
            return True, "Robot stowed.", {}
        except Exception as e:
            self.last_error = str(e)
            return False, f"Home/Stow error: {e}", {"exception": self.last_error}