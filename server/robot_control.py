# robot_control.py
import grp
import getpass
import time

# ------------------------------
# PIMU & SDK compatibility helpers
# ------------------------------
def _pimu_pull_status(pimu):
    """Refresh PIMU status across SDK versions."""
    if hasattr(pimu, "pull_status"):
        return pimu.pull_status()
    if hasattr(pimu, "get_status"):
        return pimu.get_status()
    # Some versions keep status live; nothing to do.
    return None

def _pimu_status_field(pimu, key, default=None):
    """Read a status field whether status is an object or a dict."""
    st = getattr(pimu, "status", None)
    if st is None:
        return default
    if isinstance(st, dict):
        return st.get(key, default)
    return getattr(st, key, default)

def _motors_enable(pimu, on=True):
    """Enable/disable motors across SDK variants."""
    # Newer APIs
    if hasattr(pimu, "enable_motors") and hasattr(pimu, "disable_motors"):
        return pimu.enable_motors() if on else pimu.disable_motors()
    # Older APIs
    if hasattr(pimu, "set_motors_power"):
        return pimu.set_motors_power(bool(on))
    # If neither exists, skip (base/arm may still function post-startup).
    return None

def _runstop_pressed(robot):
    """Return True if runstop/estop is active, False otherwise (best-effort)."""
    try:
        pimu = getattr(robot, "pimu", None)
        if not pimu:
            return False
        _pimu_pull_status(pimu)
        return bool(_pimu_status_field(pimu, "runstop_event", 0) == 1)
    except Exception:
        return False


def _push_cmd(robot):
    # prefer robot.push_command(); fall back to base.push_command()
    if hasattr(robot, "push_command"):
        robot.push_command()
    elif hasattr(getattr(robot, "base", None), "push_command"):
        robot.base.push_command()

def _base_is_moving(b):
    # Try common APIs/fields in order
    try:
        if hasattr(b, "is_busy"):
            return bool(b.is_busy())
        if hasattr(b, "is_moving"):
            return bool(b.is_moving())
        st = getattr(b, "status", None)
        if isinstance(st, dict):
            # names vary by SDK—these are common ones
            for k in ("is_moving", "motion_active", "motion", "busy"):
                v = st.get(k, None)
                if isinstance(v, dict) and "is_moving" in v:
                    return bool(v["is_moving"])
                if isinstance(v, bool):
                    return v
    except Exception:
        pass
    # If we can't tell, be conservative and say "moving" only briefly
    return False

# ------------------------------
# Robot manager
# ------------------------------
class RobotManager:
    def __init__(self):
        self.robot = None
        self.started = False
        self.last_error = None

    def _is_user_in_dialout(self):
        user = getpass.getuser()
        return any(user in g.gr_mem for g in grp.getgrall() if g.gr_name == "dialout")

    # ---------- Diagnostics ----------
    def diagnostics(self):
        info = {
            "started": self.started,
            "last_error": self.last_error,
            "stretch_body_import_ok": False,
            "user_in_dialout": self._is_user_in_dialout(),
        }
        try:
            import stretch_body.robot as hello_robot
            info["stretch_body_import_ok"] = True
            r = hello_robot.Robot()
            if hasattr(r, "pimu"):
                try:
                    _pimu_pull_status(r.pimu)
                    info["runstop_pressed"] = bool(_pimu_status_field(r.pimu, "runstop_event", 0) == 1)
                except Exception as e:
                    info["runstop_error"] = str(e)
            try:
                r.stop()
            except Exception:
                pass
        except Exception as e:
            info["import_error"] = str(e)
        return info

    # ---------- Lifecycle ----------
    def startup(self):
        if self.started:
            return True, "Robot already started.", {"already_started": True}
        try:
            import stretch_body.robot as hello_robot
            self.robot = hello_robot.Robot()

            ok = self.robot.startup()
            if not ok:
                # Try to surface a runstop hint if available
                meta = {"startup_ok": False}
                try:
                    if hasattr(self.robot, "pimu"):
                        _pimu_pull_status(self.robot.pimu)
                        meta["runstop_pressed"] = bool(_pimu_status_field(self.robot.pimu, "runstop_event", 0) == 1)
                except Exception as e:
                    meta["pimu_status_error"] = str(e)
                self.last_error = "robot.startup() returned False"
                self.robot = None
                return False, "Failed to startup Stretch SDK.", meta

            # Immediately check runstop/estop
            if _runstop_pressed(self.robot):
                self.last_error = "Runstop is pressed"
                # Safely stop what we started
                try:
                    self.robot.stop()
                except Exception:
                    pass
                self.robot = None
                return False, "Runstop is pressed; release and retry.", {"runstop": True}

            # Optional: enable motors (version-safe)
            try:
                if hasattr(self.robot, "pimu"):
                    _motors_enable(self.robot.pimu, True)
            except Exception:
                pass

            self.started = True
            return True, "Robot initialized.", {}
        except Exception as e:
            self.last_error = str(e)
            self.robot = None
            return False, f"Startup error: {e}", {"exception": self.last_error}

    def shutdown(self):
        if not self.started:
            return True, "Robot already stopped.", {"already_stopped": True}
        try:
            if self.robot:
                # Best-effort: disable motors, then stop()
                try:
                    if hasattr(self.robot, "pimu"):
                        _motors_enable(self.robot.pimu, False)
                except Exception:
                    pass
                try:
                    self.robot.stop()
                except Exception:
                    pass
                self.robot = None
            self.started = False
            return True, "Robot shut down.", {}
        except Exception as e:
            self.last_error = str(e)
            return False, f"Shutdown error: {e}", {"exception": self.last_error}

    # ---------- Motion ----------
    def move_linear(self, distance_m: float, velocity_mps: float = 0.10, timeout_s: float = 10.0):
        if not self.started or not self.robot:
            return False, "Robot not started. Call /robot/start first.", {}

        # Safety clamps
        try:
            distance_m = float(distance_m)
            velocity_mps = float(velocity_mps)
        except Exception:
            return False, "Invalid distance or velocity.", {}
        if not (-1.0 <= distance_m <= 1.0):
            return False, "distance_m must be between -1.0 and 1.0 meters.", {"distance_m": distance_m}
        if not (0.02 <= velocity_mps <= 0.30):
            return False, "velocity_mps must be between 0.02 and 0.30 m/s.", {"velocity_mps": velocity_mps}

        # Run-stop check
        if _runstop_pressed(self.robot):
            return False, "Runstop is pressed; release and retry.", {"runstop": True}

        try:
            b = getattr(self.robot, "base", None)
            if b is None:
                return False, "Robot base not available on this model.", {}

            # Set velocity if supported
            if hasattr(b, "set_velocity"):
                try:
                    b.set_velocity(abs(velocity_mps))
                except Exception:
                    pass

            # Queue the motion and PUSH it to the robot
            b.translate_by(distance_m)
            _push_cmd(self.robot)

            # Wait until motion completes (best-effort)
            start = time.time()
            time.sleep(0.05)  # give firmware a tick
            while time.time() - start < timeout_s:
                if not _base_is_moving(b):
                    break
                time.sleep(0.05)

            return True, f"Moved {distance_m:.3f} m at ~{velocity_mps:.2f} m/s", {
                "distance_m": distance_m,
                "velocity_mps": velocity_mps
            }
        except Exception as e:
            return False, f"Move error: {e}", {}

    def move_angular(self, angle_rad: float, angular_rate_rps: float = 0.5, timeout_s: float = 10.0):
        """
        Rotate the base in place: +CCW / -CW (radians).
        """
        if not self.started or not self.robot:
            return False, "Robot not started. Call /robot/start first.", {}
        try:
            angle_rad = float(angle_rad)
            angular_rate_rps = float(angular_rate_rps)
        except Exception:
            return False, "Invalid angle or angular rate.", {}
        if not (-1.57 <= angle_rad <= 1.57):  # ±90°
            return False, "angle_rad must be between -1.57 and 1.57 rad.", {"angle_rad": angle_rad}
        if not (0.1 <= angular_rate_rps <= 1.5):
            return False, "angular_rate_rps must be between 0.1 and 1.5 rad/s.", {"angular_rate_rps": angular_rate_rps}

        if _runstop_pressed(self.robot):
            return False, "Runstop is pressed; release and retry.", {"runstop": True}

        try:
            b = getattr(self.robot, "base", None)
            if b is None:
                return False, "Robot base not available on this model.", {}

            if hasattr(b, "set_ang_vel"):
                try:
                    b.set_ang_vel(abs(angular_rate_rps))
                except Exception:
                    pass

            b.rotate_by(angle_rad)
            _push_cmd(self.robot)

            start = time.time()
            time.sleep(0.05)
            while time.time() - start < timeout_s:
                if not _base_is_moving(b):
                    break
                time.sleep(0.05)

            return True, f"Rotated {angle_rad:.3f} rad at ~{angular_rate_rps:.2f} rad/s", {
                "angle_rad": angle_rad,
                "angular_rate_rps": angular_rate_rps
            }
        except Exception as e:
            return False, f"Rotate error: {e}", {}

    def halt_motion(self):
        """Best-effort immediate halt."""
        try:
            if self.robot:
                if hasattr(self.robot, "base"):
                    try:
                        self.robot.base.stop()
                    except Exception:
                        pass
                # Disabling motors helps ensure no residual motion
                try:
                    if hasattr(self.robot, "pimu"):
                        _motors_enable(self.robot.pimu, False)
                except Exception:
                    pass
            return True, "Motion halted.", {}
        except Exception as e:
            return False, f"Halt error: {e}", {}

    # ---------- Higher-level actions ----------
    def home(self):
        """
        Home the robot if supported by this SDK version. Some stacks expose robot.home().
        """
        if not self.started or not self.robot:
            return False, "Robot not started. Call /robot/start first.", {}
        try:
            if hasattr(self.robot, "home"):
                self.robot.home()
                return True, "Homed robot.", {}
            # If no aggregate home(), try homing subsystems that exist.
            did_any = False
            if hasattr(self.robot, "lift") and hasattr(self.robot.lift, "home"):
                self.robot.lift.home(); did_any = True
            if hasattr(self.robot, "arm") and hasattr(self.robot.arm, "home"):
                self.robot.arm.home(); did_any = True
            if hasattr(self.robot, "head") and hasattr(self.robot.head, "home"):
                self.robot.head.home(); did_any = True
            if did_any:
                return True, "Homed available subsystems.", {}
            return False, "Home not supported on this SDK.", {}
        except Exception as e:
            return False, f"Home error: {e}", {}