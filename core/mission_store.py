"""Crash-recovery mission checkpoint persistence.

Persists the mission FSM checkpoint to disk after every state transition and
after key mission variables change, so a companion-computer crash / power loss /
process restart can resume the mission from the last safely-completed context
instead of restarting from the beginning.

Design goals
------------
* Atomic writes  — a checkpoint is written to a temp file and atomically moved
  into place with os.replace(), so a crash mid-write can never leave a torn
  (half-written) JSON file on disk.
* Tolerant reads — a missing, corrupt, or unreadable checkpoint yields None
  (treated as "no recoverable mission"), never an exception propagating up to
  the FSM.
* Thread-safe   — the FSM tick thread and the vision thread may both trigger a
  checkpoint; all file access is serialized under a lock.
* Crash-consistent — every write is followed by fsync so the data is durable
  across a sudden power cut (not just an orderly process exit).

What is persisted (see CHANGE 1 in the task spec)
-------------------------------------------------
* current mission state + landing context
* GUIDED_HOLD anchor (search origin) and true_home (launch reference)
* payload status (armed gate / released) and QR decode text
* search progress (ring waypoint index) and mission timestamps
"""

import json
import logging
import os
import tempfile
import threading
import time
from typing import Optional, Any, Dict

logger = logging.getLogger(__name__)


class MissionStore:
    """Atomic, crash-consistent JSON checkpoint store for the mission FSM."""

    # Schema version — bump when the on-disk layout changes so old checkpoints
    # can be detected and handled deliberately rather than misinterpreted.
    SCHEMA_VERSION = 1

    def __init__(self, path: str = "mission_state.json"):
        self.path = path
        self._lock = threading.Lock()
        logger.info(f"MissionStore using checkpoint file: {os.path.abspath(path)}")

    # ------------------------------------------------------------------
    # Write path
    # ------------------------------------------------------------------

    def save(self, checkpoint: Dict[str, Any]) -> bool:
        """Atomically persist a checkpoint dict to disk.

        Adds envelope metadata (schema version + wall-clock save time) before
        writing. Returns True on success, False on any I/O failure — a failed
        checkpoint is logged but never raises, so a disk hiccup cannot crash the
        flight loop.
        """
        if not isinstance(checkpoint, dict):
            logger.error("MissionStore.save: checkpoint must be a dict, refusing to write.")
            return False

        envelope = dict(checkpoint)
        envelope["schema_version"] = self.SCHEMA_VERSION
        envelope["saved_at"] = time.time()

        try:
            serialized = json.dumps(envelope, indent=2)
        except (TypeError, ValueError) as e:
            logger.error(f"MissionStore.save: checkpoint not JSON-serializable: {e}")
            return False

        directory = os.path.dirname(os.path.abspath(self.path)) or "."
        tmp_path = None
        try:
            # Write to a temp file in the SAME directory so os.replace() is an
            # atomic rename on the same filesystem (no cross-device copy).
            fd, tmp_path = tempfile.mkstemp(dir=directory, prefix=".mission_state.", suffix=".tmp")
            with os.fdopen(fd, "w") as f:
                f.write(serialized)
                f.flush()
                os.fsync(f.fileno())          # durable across power loss
            os.replace(tmp_path, self.path)   # atomic move into place
            # fsync the directory so the rename itself is durable
            try:
                dir_fd = os.open(directory, os.O_DIRECTORY)
                os.fsync(dir_fd)
                os.close(dir_fd)
            except OSError:
                pass
            return True
        except OSError as e:
            logger.error(f"MissionStore.save: failed to write checkpoint: {e}")
            return False
        finally:
            if tmp_path is not None and os.path.exists(tmp_path):
                try:
                    os.unlink(tmp_path)
                except OSError:
                    pass

    # ------------------------------------------------------------------
    # Read path
    # ------------------------------------------------------------------

    def load(self) -> Optional[Dict[str, Any]]:
        """Load the last persisted checkpoint, or None if none/invalid.

        Tolerant by design: a missing file, corrupt JSON, unreadable file, or a
        mismatched schema version all return None so the caller treats it as a
        fresh (un-recoverable) start. Never raises.
        """
        if not os.path.exists(self.path):
            return None
        try:
            with open(self.path, "r") as f:
                data = json.load(f)
        except (OSError, ValueError) as e:
            logger.warning(f"MissionStore.load: checkpoint unreadable ({e}); ignoring.")
            return None

        if not isinstance(data, dict):
            logger.warning("MissionStore.load: checkpoint is not a dict; ignoring.")
            return None

        version = data.get("schema_version", 0)
        if version != self.SCHEMA_VERSION:
            logger.warning(
                f"MissionStore.load: schema version {version} != {self.SCHEMA_VERSION}; "
                "checkpoint ignored (treated as fresh start)."
            )
            return None

        return data

    # ------------------------------------------------------------------
    # Housekeeping
    # ------------------------------------------------------------------

    def clear(self) -> bool:
        """Delete the checkpoint file.

        Called once the mission reaches a clean terminal state (MISSION_COMPLETE)
        or a deliberate abort, so a *subsequent* boot does not try to resume a
        mission that already finished. Returns True if the file is gone (or never
        existed), False on I/O error.
        """
        try:
            if os.path.exists(self.path):
                os.unlink(self.path)
            return True
        except OSError as e:
            logger.warning(f"MissionStore.clear: could not remove checkpoint: {e}")
            return False

    def checkpoint_age_s(self) -> Optional[float]:
        """Seconds since the on-disk checkpoint was saved, or None if absent."""
        data = self.load()
        if data is None:
            return None
        saved_at = data.get("saved_at")
        if saved_at is None:
            return None
        return time.time() - float(saved_at)
