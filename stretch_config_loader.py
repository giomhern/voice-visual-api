#!/usr/bin/env python3
"""
stretch_config_loader.py
Configuration loader for Stretch robot - parses JSON files from voice assistant.

Usage:
    from stretch_config_loader import StretchConfigLoader

    loader = StretchConfigLoader()
    config = loader.load_all()

    print(config['general'].movement_speed)
    print(config['desk'].cleaning_thoroughness)
"""

import json
import glob
import os
import subprocess
import requests
from dataclasses import dataclass, asdict
from typing import Optional, Dict, Any


# =============================================================================
# Data Classes for Settings
# =============================================================================

@dataclass
class GeneralSettings:
    """General settings from Section 1."""
    voice_gender: str = "female"        # male, female
    personality: str = "polite"         # casual, meticulous, polite
    explainability_level: str = "short" # none, short, full
    confirmation_style: bool = True     # True = ask for confirmation

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class DeskSettings:
    """Desk station settings from Section 2."""
    movement_speed: str = "medium"          # slow, medium, fast
    voice_volume: str = "medium"            # low, medium, high
    social_distance: str = "medium"         # close, medium, far
    cleaning_thoroughness: str = "wipe once"  # wipe once, wipe twice, wipe thoroughly

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    def get_wipe_count(self) -> int:
        """Convert cleaning_thoroughness to number of wipes."""
        mapping = {
            "wipe once": 1,
            "wipe twice": 2,
            "wipe thoroughly": 3,
        }
        return mapping.get(self.cleaning_thoroughness, 1)


@dataclass
class BedSettings:
    """Bed station settings from Section 2."""
    movement_speed: str = "medium"          # slow, medium, fast
    voice_volume: str = "medium"            # low, medium, high
    social_distance: str = "medium"         # close, medium, far
    pillow_arrangement: str = "center"      # center, top

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class KitchenSettings:
    """Kitchen station settings from Section 2."""
    movement_speed: str = "medium"          # slow, medium, fast
    voice_volume: str = "medium"            # low, medium, high
    social_distance: str = "medium"         # close, medium, far
    preferred_snack: str = "Doritos"        # Doritos, Cheetos

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


# =============================================================================
# Value Mappings for Robot Actions
# =============================================================================

class ValueMappings:
    """Convert setting values to robot-usable numbers."""

    # Movement speed -> velocity in m/s
    SPEED_TO_VELOCITY = {
        "slow": 0.1,
        "medium": 0.3,
        "fast": 0.5,
    }

    # Social distance -> distance in meters
    DISTANCE_TO_METERS = {
        "close": 0.76,    # ~2.5 feet
        "medium": 1.22,   # ~4 feet
        "far": 1.68,      # ~5.5 feet
    }

    # Room voice volume -> multiplier (0-1)
    VOLUME_TO_MULTIPLIER = {
        "low": 0.3,
        "medium": 0.6,
        "high": 1.0,
    }

    # Cleaning thoroughness -> wipe count
    THOROUGHNESS_TO_WIPES = {
        "wipe once": 1,
        "wipe twice": 2,
        "wipe thoroughly": 3,
    }

    @classmethod
    def get_velocity(cls, speed: str) -> float:
        """Convert speed setting to velocity in m/s."""
        return cls.SPEED_TO_VELOCITY.get(speed, 0.3)

    @classmethod
    def get_distance_meters(cls, distance: str) -> float:
        """Convert social distance to meters."""
        return cls.DISTANCE_TO_METERS.get(distance, 1.22)

    @classmethod
    def get_volume_multiplier(cls, volume: str) -> float:
        """Convert volume setting to multiplier."""
        return cls.VOLUME_TO_MULTIPLIER.get(volume, 0.6)

    @classmethod
    def get_wipe_count(cls, thoroughness: str) -> int:
        """Convert cleaning thoroughness to wipe count."""
        return cls.THOROUGHNESS_TO_WIPES.get(thoroughness, 1)


# =============================================================================
# Main Config Loader Class
# =============================================================================

class StretchConfigLoader:
    """Load configuration from voice assistant JSON files or remote Flask server."""

    def __init__(self, config_dir: str = "experiment_record", remote_url: str = None):
        """
        Initialize the config loader.

        Args:
            config_dir: Directory containing the JSON config files (local mode)
            remote_url: Base URL of the voice assistant Flask server (e.g. "http://192.168.1.10:5050")
        """
        self.config_dir = config_dir
        self.remote_url = remote_url.rstrip("/") if remote_url else None

    def _get_latest_file(self, pattern: str) -> Optional[str]:
        """Get the most recent file matching pattern."""
        search_path = os.path.join(self.config_dir, pattern)
        files = glob.glob(search_path)
        if not files:
            return None
        return max(files, key=os.path.getmtime)

    def _load_json(self, filepath: str) -> Optional[Dict]:
        """Load and parse a JSON file."""
        try:
            with open(filepath, 'r') as f:
                return json.load(f)
        except (json.JSONDecodeError, IOError) as e:
            print(f"Error loading {filepath}: {e}")
            return None

    def _fetch_remote_station(self, station_name: str) -> Optional[Dict]:
        """Fetch station settings from the remote Flask server."""
        if not self.remote_url:
            return None
        url = f"{self.remote_url}/api/config/station/{station_name}"
        try:
            resp = requests.get(url, timeout=5)
            if resp.status_code == 404:
                print(f"[ConfigLoader] Remote: station '{station_name}' not configured yet")
                return None
            resp.raise_for_status()
            data = resp.json()
            print(f"[ConfigLoader] Loaded {station_name} from remote: {self.remote_url}")
            return data.get("extracted_settings", {})
        except requests.exceptions.ConnectionError:
            print(f"[ConfigLoader] Cannot connect to remote: {self.remote_url}")
            return None
        except Exception as e:
            print(f"[ConfigLoader] Remote fetch error for {station_name}: {e}")
            return None

    def wait_for_station(self, station_name: str, poll_interval: float = 2.0, timeout: float = 300.0) -> Optional[Dict]:
        """Poll the remote server until a station's config becomes available."""
        if not self.remote_url:
            return None
        import time
        start = time.time()
        print(f"[ConfigLoader] Waiting for {station_name} config from {self.remote_url}...")
        while time.time() - start < timeout:
            result = self._fetch_remote_station(station_name)
            if result is not None:
                return result
            time.sleep(poll_interval)
        print(f"[ConfigLoader] Timeout waiting for {station_name} config after {timeout:.0f}s")
        return None

    def _fetch_remote_general(self) -> Optional[Dict]:
        """Fetch general settings from the remote Flask server."""
        if not self.remote_url:
            return None
        url = f"{self.remote_url}/api/config/general"
        try:
            resp = requests.get(url, timeout=5)
            if resp.status_code == 404:
                print("[ConfigLoader] Remote: general settings not configured yet")
                return None
            resp.raise_for_status()
            data = resp.json()
            print(f"[ConfigLoader] Loaded general settings from remote: {self.remote_url}")
            return data.get("applied_settings", {})
        except requests.exceptions.ConnectionError:
            print(f"[ConfigLoader] Cannot connect to remote: {self.remote_url}")
            return None
        except Exception as e:
            print(f"[ConfigLoader] Remote fetch error for general: {e}")
            return None

    def load_general_settings(self) -> GeneralSettings:
        """Load general settings from remote server or local file."""
        applied = None

        # Try remote first
        if self.remote_url:
            applied = self._fetch_remote_general()

        # Fallback to local file
        if applied is None:
            filepath = self._get_latest_file("section1_general_settings_*.json")
            if not filepath:
                print("[ConfigLoader] No Section 1 config found, using defaults")
                return GeneralSettings()
            data = self._load_json(filepath)
            if not data:
                return GeneralSettings()
            print(f"[ConfigLoader] Loaded general settings from: {filepath}")
            applied = data.get("applied_settings", {})

        return GeneralSettings(
            voice_gender=applied.get("voice_gender", "female"),
            personality=applied.get("personality", "polite"),
            explainability_level=applied.get("explainability_level", "short"),
            confirmation_style=applied.get("confirmation_style", True),
        )

    def load_desk_settings(self) -> DeskSettings:
        """Load desk station settings from remote server or local file."""
        desk = None

        # Try remote first
        if self.remote_url:
            desk = self._fetch_remote_station("desk")

        # Fallback to local file
        if desk is None:
            filepath = self._get_latest_file("section2_all_stations_*.json")
            if not filepath:
                print("[ConfigLoader] No desk config found, using defaults")
                return DeskSettings()
            data = self._load_json(filepath)
            if not data:
                return DeskSettings()
            desk = data.get("stations", {}).get("desk", {}).get("extracted_settings", {})

        return DeskSettings(
            movement_speed=desk.get("room_movement_speed", "medium"),
            voice_volume=desk.get("room_voice_volume", "medium"),
            social_distance=desk.get("room_social_distance", "medium"),
            cleaning_thoroughness=desk.get("cleaning_thoroughness", "wipe once"),
        )

    def load_bed_settings(self) -> BedSettings:
        """Load bed station settings from remote server or local file."""
        bed = None

        if self.remote_url:
            bed = self._fetch_remote_station("bed")

        if bed is None:
            filepath = self._get_latest_file("section2_all_stations_*.json")
            if not filepath:
                print("[ConfigLoader] No bed config found, using defaults")
                return BedSettings()
            data = self._load_json(filepath)
            if not data:
                return BedSettings()
            bed = data.get("stations", {}).get("bed", {}).get("extracted_settings", {})

        return BedSettings(
            movement_speed=bed.get("room_movement_speed", "medium"),
            voice_volume=bed.get("room_voice_volume", "medium"),
            social_distance=bed.get("room_social_distance", "medium"),
            pillow_arrangement=bed.get("pillow_arrangement", "center"),
        )

    def load_kitchen_settings(self) -> KitchenSettings:
        """Load kitchen station settings from remote server or local file."""
        kitchen = None

        if self.remote_url:
            kitchen = self._fetch_remote_station("kitchen")

        if kitchen is None:
            filepath = self._get_latest_file("section2_all_stations_*.json")
            if not filepath:
                print("[ConfigLoader] No kitchen config found, using defaults")
                return KitchenSettings()
            data = self._load_json(filepath)
            if not data:
                return KitchenSettings()
            kitchen = data.get("stations", {}).get("kitchen", {}).get("extracted_settings", {})

        return KitchenSettings(
            movement_speed=kitchen.get("room_movement_speed", "medium"),
            voice_volume=kitchen.get("room_voice_volume", "medium"),
            social_distance=kitchen.get("room_social_distance", "medium"),
            preferred_snack=kitchen.get("preferred_snack", "Doritos"),
        )

    def load_all(self) -> Dict[str, Any]:
        """
        Load all settings from both Section 1 and Section 2.

        Returns:
            Dict with keys: 'general', 'desk', 'bed', 'kitchen'
        """
        return {
            "general": self.load_general_settings(),
            "desk": self.load_desk_settings(),
            "bed": self.load_bed_settings(),
            "kitchen": self.load_kitchen_settings(),
        }

    # =========================================================================
    # ROS2 Publishing
    # =========================================================================

    def _ros2_pub(self, topic: str, msg_type: str, data: dict):
        """Publish a single message to a ROS2 topic via subprocess."""
        msg_json = json.dumps(data)
        cmd = ["ros2", "topic", "pub", "--once", topic, msg_type, msg_json]
        print(f"  [ROS2] Publishing to {topic}")
        try:
            subprocess.run(cmd, check=True, timeout=10)
        except FileNotFoundError:
            print(f"  [ROS2] ERROR: 'ros2' command not found. Is ROS2 sourced?")
        except subprocess.TimeoutExpired:
            print(f"  [ROS2] ERROR: Publish to {topic} timed out")
        except subprocess.CalledProcessError as e:
            print(f"  [ROS2] ERROR: Publish to {topic} failed: {e}")

    def publish_commands(self):
        """Load config and publish all robot commands as ROS2 topics."""
        params = self.get_robot_params()

        print("\n=== Publishing ROS2 Commands ===")
        print(f"  General: personality={params['general']['personality']}, explainability={params['general']['explainability']}")
        print(f"  Desk: velocity={params['desk']['velocity']}, wipes={params['desk']['wipe_count']}")
        print(f"  Bed: velocity={params['bed']['velocity']}, pillow={params['bed']['pillow_position']}")
        print(f"  Kitchen: velocity={params['kitchen']['velocity']}, snack={params['kitchen']['snack']}")
        print("=== Done ===")

    def get_robot_params(self) -> Dict[str, Any]:
        """
        Get settings converted to robot-usable parameters.

        Returns:
            Dict with numeric values ready for robot control.
        """
        config = self.load_all()
        general = config["general"]
        desk = config["desk"]
        bed = config["bed"]
        kitchen = config["kitchen"]

        return {
            "general": {
                "personality": general.personality,
                "explainability": general.explainability_level,
            },
            "desk": {
                "velocity": ValueMappings.get_velocity(desk.movement_speed),
                "voice_volume": ValueMappings.get_volume_multiplier(desk.voice_volume),
                "social_distance_m": ValueMappings.get_distance_meters(desk.social_distance),
                "wipe_count": ValueMappings.get_wipe_count(desk.cleaning_thoroughness),
            },
            "bed": {
                "velocity": ValueMappings.get_velocity(bed.movement_speed),
                "voice_volume": ValueMappings.get_volume_multiplier(bed.voice_volume),
                "social_distance_m": ValueMappings.get_distance_meters(bed.social_distance),
                "pillow_position": bed.pillow_arrangement,
            },
            "kitchen": {
                "velocity": ValueMappings.get_velocity(kitchen.movement_speed),
                "voice_volume": ValueMappings.get_volume_multiplier(kitchen.voice_volume),
                "social_distance_m": ValueMappings.get_distance_meters(kitchen.social_distance),
                "snack": kitchen.preferred_snack,
            },
        }


# =============================================================================
# Main (for testing)
# =============================================================================

if __name__ == "__main__":
    loader = StretchConfigLoader()

    print("=" * 60)
    print("STRETCH CONFIGURATION LOADER")
    print("=" * 60)

    # Load all settings
    config = loader.load_all()

    print("\n=== General Settings ===")
    general = config["general"]
    print(f"  Voice Gender: {general.voice_gender}")
    print(f"  Personality: {general.personality}")
    print(f"  Explainability: {general.explainability_level}")
    print(f"  Confirmation Style: {general.confirmation_style}")

    print("\n=== Desk Settings ===")
    desk = config["desk"]
    print(f"  Movement Speed: {desk.movement_speed}")
    print(f"  Voice Volume: {desk.voice_volume}")
    print(f"  Cleaning: {desk.cleaning_thoroughness} ({desk.get_wipe_count()} wipes)")

    print("\n=== Bed Settings ===")
    bed = config["bed"]
    print(f"  Movement Speed: {bed.movement_speed}")
    print(f"  Pillow: {bed.pillow_arrangement}")

    print("\n=== Kitchen Settings ===")
    kitchen = config["kitchen"]
    print(f"  Movement Speed: {kitchen.movement_speed}")
    print(f"  Snack: {kitchen.preferred_snack}")

    print("\n=== Robot Parameters (converted) ===")
    params = loader.get_robot_params()
    print(f"  General: personality={params['general']['personality']}, explainability={params['general']['explainability']}")
    print(f"  Desk velocity: {params['desk']['velocity']} m/s, wipes: {params['desk']['wipe_count']}")
    print(f"  Bed velocity: {params['bed']['velocity']} m/s, pillow: {params['bed']['pillow_position']}")
    print(f"  Kitchen velocity: {params['kitchen']['velocity']} m/s, snack: {params['kitchen']['snack']}")

    # Publish to ROS2 topics
    loader.publish_commands()
