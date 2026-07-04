"""Companion computer hardware diagnostics for CPU temperature monitoring."""

import logging

logger = logging.getLogger(__name__)

# Thermal zone path on Raspberry Pi and most ARM Linux SBCs
_THERMAL_PATH = "/sys/class/thermal/thermal_zone0/temp"

# Thresholds
_TEMP_CRITICAL_MC = 85000  # 85 °C in millicelsius — throttling risk above this


class SystemMonitor:
    """Monitors companion computer health via Linux sysfs / procfs interfaces.

    Designed to run inside the FSM tick loop (called every tick). Reads are
    cheap filesystem reads; no external dependencies required.
    """

    def __init__(self, config: dict):
        self.config = config
        self._temp_unavailable_warned = False  # Suppress repeated warnings on non-Pi hosts

    def read_cpu_temp_mc(self) -> int:
        """Read CPU temperature from sysfs thermal zone in millicelsius.

        Returns:
            Temperature in millicelsius, or -1 if the file is not available
            (e.g. development machine, Docker, VM).
        """
        try:
            with open(_THERMAL_PATH, "r") as f:
                return int(f.read().strip())
        except (FileNotFoundError, OSError):
            return -1

    def check_health(self) -> bool:
        """Perform companion computer self-diagnostics.

        Checks:
            - CPU temperature: fails if > 85 °C

        Returns:
            True if all checks pass, False if any threshold is exceeded.
        """
        temp_mc = self.read_cpu_temp_mc()

        if temp_mc == -1:
            # Thermal sensor unavailable — log once and treat as healthy
            # (allows code to run on dev machines without Pi hardware)
            if not self._temp_unavailable_warned:
                logger.warning(
                    "SystemMonitor: CPU thermal sensor not found at %s — "
                    "temperature monitoring disabled.", _THERMAL_PATH
                )
                self._temp_unavailable_warned = True
            return True

        temp_c = temp_mc / 1000.0

        if temp_mc >= _TEMP_CRITICAL_MC:
            logger.critical(
                "SystemMonitor: CPU temperature %.1f°C exceeds critical threshold (85°C)! "
                "Triggering failsafe.", temp_c
            )
            return False

        return True
