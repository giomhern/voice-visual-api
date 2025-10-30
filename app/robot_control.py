# robot_control.py
import glob, grp, getpass

class RobotManager:
    def __init__(self):
        self.robot = None
        self.started = False
        self.last_error = None

    def _is_user_in_dialout(self):
        user = getpass.getuser()
        return any(user in g.gr_mem for g in grp.getgrall() if g.gr_name == "dialout")

    def diagnostics(self):
        info = {
            "user_in_dialout": self._is_user_in_dialout(),
            "ttyUSB_present": bool(glob.glob("/dev/ttyUSB*")),
            "hello_symlinks_present": bool(glob.glob("/dev/hello*")),
            "ttyUSB_list": glob.glob("/dev/ttyUSB*"),
            "hello_symlink_list": glob.glob("/dev/hello*"),
            "started": self.started,
            "last_error": self.last_error,
        }
        try:
            import stretch_body.robot as hello_robot
            info["stretch_body_import_ok"] = True
            r = hello_robot.Robot()
            # Don't enable motors here—just probe PIMU & runstop
            if hasattr(r, "pimu"):
                r.pimu.get_status()
                info["runstop_pressed"] = bool(getattr(r.pimu.status, "runstop_event", 0) == 1)
            else:
                info["runstop_pressed"] = None
            try:
                r.stop()
            except Exception:
                pass
        except Exception as e:
            info["stretch_body_import_ok"] = False
            info["import_error"] = str(e)
        return info

    def startup(self):
        if self.started:
            return True, "Robot already started.", {"already_started": True}
        try:
            import stretch_body.robot as hello_robot
            self.robot = hello_robot.Robot()
            ok = self.robot.startup()
            if not ok:
                self.last_error = "robot.startup() returned False"
                # Try to surface runstop immediately
                meta = {"startup_ok": False}
                if hasattr(self.robot, "pimu"):
                    try:
                        self.robot.pimu.get_status()
                        meta["runstop_pressed"] = bool(getattr(self.robot.pimu.status, "runstop_event", 0) == 1)
                    except Exception as e:
                        meta["pimu_status_error"] = str(e)
                return False, "Failed to startup Stretch SDK.", meta

            # Check runstop after startup
            if hasattr(self.robot, "pimu"):
                self.robot.pimu.get_status()
                if getattr(self.robot.pimu.status, "runstop_event", 0) == 1:
                    self.last_error = "Runstop is pressed"
                    return False, "Runstop is pressed; release and retry.", {"runstop": True}

            # Enable motors, but no motion
            if hasattr(self.robot, "pimu"):
                self.robot.pimu.enable_motors()

            self.started = True
            return True, "Robot initialized and motors enabled.", {"motors_enabled": True}
        except Exception as e:
            self.last_error = str(e)
            self.robot = None
            return False, f"Startup error: {e}", {"exception": self.last_error}

    def shutdown(self):
        if not self.started:
            return True, "Robot already stopped.", {"already_stopped": True}
        try:
            if self.robot:
                try:
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