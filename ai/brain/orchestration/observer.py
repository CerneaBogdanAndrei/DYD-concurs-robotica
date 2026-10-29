"""LocationObserver: brațul în poza de observare → așezare → captură → viziune, cu reîncercări.

Imaginile (cadrul brut și analiza adnotată) sunt publicate în `PreviewHub`, ca sarcina să poată fi urmărită
live în browser: camera nu poate fi deschisă de două procese, deci previzualizarea rulează în același proces.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Any, Optional

from brain.configuration.models import SystemConfig
from brain.domain.errors import BrainError, PerceptionInputError
from brain.domain.observation import GridObservation
from brain.logging_setup import log_event
from brain.orchestration.session import ArmSession, DurationJournal
from brain.ports.sensors import Clock, FrameSource

logger = logging.getLogger("brain.orchestration.observer")


class ObservationFailed(BrainError):
    """Locația nu a putut fi observată acționabil după toate încercările."""


@dataclass
class PerceptionResult:
    observation: GridObservation
    raw: Any = None  # ultimul cadru BGR (pentru previzualizare), dacă există
    annotated: Any = None


class CameraPerception:
    """Captură de la o sursă de cadre + pipeline-ul de viziune."""

    def __init__(self, camera: FrameSource, pipeline: Any, frames: int) -> None:
        self.camera = camera
        self.pipeline = pipeline
        self.frames = frames

    def perceive(self, location: str) -> PerceptionResult:
        from brain.perception.annotate import render_annotation

        frames = self.camera.grab(self.frames)
        result = self.pipeline.analyze(frames, location)
        gripper = getattr(getattr(self.pipeline, "system", None), "gripper", None)
        return PerceptionResult(result.observation, frames[-1] if frames else None, render_annotation(result, gripper))


class LocationObserver:
    def __init__(
        self,
        system: SystemConfig,
        perception: Any,
        clock: Clock,
        journal: DurationJournal,
        session: Optional[ArmSession] = None,
        hub: Any = None,
        move_arm: bool = True,
        task: str = "",
    ) -> None:
        self.system = system
        self.perception = perception
        self.clock = clock
        self.journal = journal
        self.session = session
        self.hub = hub
        self.move_arm = move_arm
        self.task = task
        self.current_pose: Optional[str] = None  # poza „văzută” de camera din fișiere în modul fără mișcare
        self.observations = 0

    def observe(self, location: str) -> GridObservation:
        loc = self.system.location(location)
        attempts = self.system.planning.max_observe_attempts
        reasons: tuple = ()
        for attempt in range(1, attempts + 1):
            started = self.clock.monotonic()
            if self.move_arm:
                if self.session is None or loc.observe_pose is None:
                    raise ObservationFailed("locația {} nu are poză de observare (observe_pose)".format(location))
                self.session.go_pose(loc.observe_pose)
                self.clock.sleep(self.system.arm.settle_s)
            self.current_pose = loc.observe_pose
            try:
                result = self.perception.perceive(location)
            except PerceptionInputError as exc:
                self.journal.add("observe", self.clock.monotonic() - started, "error", location)
                log_event(logger, "observe", level=logging.WARNING, task=self.task, state="OBSERVE", action=location,
                          result="error", error=str(exc), attempt=attempt)
                reasons = (str(exc),)
                continue
            observation = result.observation
            self.observations += 1
            duration = self.clock.monotonic() - started
            self.journal.add("observe", duration, "actionable" if observation.actionable else "rejected", location)
            self._publish(location, result)
            log_event(logger, "observe", task=self.task, state="OBSERVE", action=location, attempt=attempt,
                      duration_ms=round(duration * 1000, 1), result="actionable" if observation.actionable else "rejected",
                      reasons=list(observation.reasons))
            if observation.actionable:
                return observation
            reasons = observation.reasons
        raise ObservationFailed("{}: neacționabil după {} încercări ({})".format(location, attempts, ", ".join(reasons)))

    def _publish(self, location: str, result: PerceptionResult) -> None:
        if self.hub is None:
            return
        from brain.cli.preview import encode_jpeg, summarize

        quality = self.system.preview.jpeg_quality
        self.hub.location = location
        try:
            if result.raw is not None:
                self.hub.publish_raw(encode_jpeg(result.raw, quality))
            annotated = encode_jpeg(result.annotated, quality) if result.annotated is not None else None
            self.hub.publish_analysis(annotated, summarize(result.observation, self.system), result.observation)
        except PerceptionInputError as exc:  # previzualizarea nu are voie să oprească sarcina
            log_event(logger, "preview_publish", level=logging.WARNING, result="error", error=str(exc))
