import numpy as np
import pytest

from brain.domain.errors import PerceptionInputError
from brain.perception.photometry import (
    FLAG_LOW_CONTRAST,
    FLAG_OVEREXPOSED,
    FLAG_UNDEREXPOSED,
    aggregate_frames,
    apply_gains,
    assess_frame,
    estimate_white_balance,
    validate_frames,
    white_reference_mask,
)


def _scene(value=180):
    rng = np.random.default_rng(0)
    image = np.full((120, 160, 3), value, dtype=np.uint8)
    image[40:80, 20:140] = 30  # contrast: o bandă întunecată
    return np.clip(image + rng.normal(0, 2, image.shape), 0, 255).astype(np.uint8)


def test_validate_frames_errors():
    good = _scene()
    with pytest.raises(PerceptionInputError):
        validate_frames([])
    with pytest.raises(PerceptionInputError):
        validate_frames([good.astype(np.float32)])
    with pytest.raises(PerceptionInputError):
        validate_frames([good[..., 0]])
    with pytest.raises(PerceptionInputError):
        validate_frames([good, good[:100]])
    assert len(validate_frames([good, good])) == 2


def test_quality_flags(app_config):
    thresholds = app_config.system.vision.quality
    assert assess_frame(_scene(), thresholds).ok
    assert FLAG_OVEREXPOSED in assess_frame(np.full((120, 160, 3), 255, np.uint8), thresholds).flags
    assert FLAG_UNDEREXPOSED in assess_frame(np.full((120, 160, 3), 5, np.uint8), thresholds).flags
    assert FLAG_LOW_CONTRAST in assess_frame(np.full((120, 160, 3), 128, np.uint8), thresholds).flags


def test_median_removes_transient_occluder():
    base = _scene()
    frames = [base.copy() for _ in range(5)]
    frames[1][10:30, 10:30] = (0, 0, 255)  # obiect tranzitoriu în 2 din 5 cadre
    frames[3][10:30, 10:30] = (0, 255, 0)
    aggregated = aggregate_frames(frames)
    assert np.abs(aggregated.astype(int) - base.astype(int)).max() <= 1


def test_white_balance_neutralizes_cast():
    image = np.full((100, 100, 3), (200, 180, 160), dtype=np.uint8)  # alb cu dominantă caldă (BGR)
    gains = estimate_white_balance(image, white_reference_mask(image), max_gain=1.6, min_pixels=100)
    assert gains is not None
    balanced = apply_gains(image, gains)
    channel_means = balanced.reshape(-1, 3).mean(axis=0)
    assert channel_means.max() - channel_means.min() < 2
    assert estimate_white_balance(image, np.zeros((100, 100), np.uint8), 1.6, 100) is None
