import numpy as np
import pytest

from brain.domain.cube import Color
from brain.perception.color import REASON_AMBIGUOUS_HUE, REASON_FEW_PIXELS, REASON_LOW_SATURATION, HueClassifier


def _pixels(hue, sat=200, val=200, n=400, spread=2, seed=0):
    rng = np.random.default_rng(seed)
    h = (hue + rng.integers(-spread, spread + 1, n)) % 180
    return np.column_stack([h, np.full(n, sat), np.full(n, val)]).astype(np.uint8)


@pytest.fixture(scope="module")
def classifier(app_config):
    return HueClassifier(app_config.colors)


@pytest.mark.parametrize("hue, color", [(0, Color.RED), (177, Color.RED), (27, Color.YELLOW), (62, Color.GREEN), (110, Color.BLUE)])
def test_known_colors(classifier, hue, color):
    vote = classifier.vote(_pixels(hue), min_pixels=60)
    assert vote.color is color
    assert vote.confidence > 0.9


def test_red_wraparound_mixed(classifier):
    pixels = np.vstack([_pixels(2, n=200), _pixels(176, n=200, seed=1)])
    assert classifier.vote(pixels, 60).color is Color.RED


def test_orange_is_ambiguous(classifier):
    vote = classifier.vote(_pixels(14, spread=1), 60)
    assert vote.color is Color.UNKNOWN
    assert REASON_AMBIGUOUS_HUE in vote.reasons
    assert vote.confidence == 0.0


def test_too_few_pixels(classifier):
    vote = classifier.vote(_pixels(110, n=10), 60)
    assert vote.color is Color.UNKNOWN and REASON_FEW_PIXELS in vote.reasons


def test_low_saturation_reduces_confidence(classifier):
    vote = classifier.vote(_pixels(110, sat=80), 60)
    assert vote.color is Color.BLUE
    assert REASON_LOW_SATURATION in vote.reasons
    assert vote.confidence <= 0.8
