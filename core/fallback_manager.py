"""Placeholder fallback manager for hardware fail-safes."""

import logging

logger = logging.getLogger(__name__)


class FallbackManager:
    """Manages trigger abort routines when telemetry links or hardware fails."""

    def __init__(self, flight_control):
        self.fc = flight_control

    def handle_fail(self, reason: str):
        """Command safe landing or return to launch based on failures."""
        logger.warning(f"Fail-safe triggered: {reason}")
        self.fc.rtl()
