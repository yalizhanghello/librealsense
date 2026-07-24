# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2026 RealSense, Inc. All Rights Reserved.

# D5x5 HKR-new Triggered Calibration (TAC v2) -- health-gated flow.
#
# Validate the frozen protocol contract end-to-end via the typed
# rs.auto_calibrated_device.run_triggered_calibration() API --
#   - RUN / DRY_RUN stop at HEALTH_CHECK with a valid health payload.
#   - Nothing is persisted to flash until the host issues an explicit COMMIT.
#   - CANCEL discards the candidate and returns the device to IDLE.

import pytest
import pyrealsense2 as rs
import logging

log = logging.getLogger(__name__)

pytestmark = [
    pytest.mark.device_each("D585"),
    pytest.mark.context("nightly"),
]

TIMEOUT_MS = 120000


@pytest.fixture
def calib_dev(test_device):
    dev, _ = test_device
    return rs.auto_calibrated_device(dev)


def test_dry_run_reaches_health_check_then_cancels(calib_dev):
    status = calib_dev.run_triggered_calibration(
        rs.triggered_calibration_mode.dry_run, timeout_ms=TIMEOUT_MS)
    assert status.state == rs.triggered_calibration_state.health_check
    assert status.result in (rs.triggered_calibration_result.success,
                             rs.triggered_calibration_result.failed_to_converge,
                             rs.triggered_calibration_result.failed_to_run)
    assert status.health_valid and status.candidate_valid

    cancel_status = calib_dev.run_triggered_calibration(
        rs.triggered_calibration_mode.cancel, timeout_ms=TIMEOUT_MS)
    assert cancel_status.state == rs.triggered_calibration_state.idle


def test_run_reaches_health_check_then_cancels(calib_dev):
    status = calib_dev.run_triggered_calibration(
        rs.triggered_calibration_mode.run, timeout_ms=TIMEOUT_MS)
    assert status.state == rs.triggered_calibration_state.health_check
    assert status.health_valid and status.candidate_valid

    cancel_status = calib_dev.run_triggered_calibration(
        rs.triggered_calibration_mode.cancel, timeout_ms=TIMEOUT_MS)
    assert cancel_status.state == rs.triggered_calibration_state.idle


def test_dry_run_commit_is_rejected(calib_dev):
    """DRY_RUN candidates must never be committable, and flash must stay untouched."""
    before = bytes(calib_dev.get_calibration_table())

    status = calib_dev.run_triggered_calibration(
        rs.triggered_calibration_mode.dry_run, timeout_ms=TIMEOUT_MS)
    assert status.state == rs.triggered_calibration_state.health_check

    with pytest.raises(RuntimeError):
        calib_dev.run_triggered_calibration(
            rs.triggered_calibration_mode.commit, timeout_ms=TIMEOUT_MS)

    cancel_status = calib_dev.run_triggered_calibration(
        rs.triggered_calibration_mode.cancel, timeout_ms=TIMEOUT_MS)
    assert cancel_status.state == rs.triggered_calibration_state.idle
    assert bytes(calib_dev.get_calibration_table()) == before


def test_run_cancel_leaves_flash_unchanged(calib_dev):
    """CANCEL from HEALTH_CHECK must discard the candidate; flash must stay untouched."""
    before = bytes(calib_dev.get_calibration_table())

    status = calib_dev.run_triggered_calibration(
        rs.triggered_calibration_mode.run, timeout_ms=TIMEOUT_MS)
    assert status.state == rs.triggered_calibration_state.health_check
    assert status.health_valid and status.candidate_valid

    cancel_status = calib_dev.run_triggered_calibration(
        rs.triggered_calibration_mode.cancel, timeout_ms=TIMEOUT_MS)
    assert cancel_status.state == rs.triggered_calibration_state.idle
    assert bytes(calib_dev.get_calibration_table()) == before


def test_repeated_run_cancel_cycles(calib_dev):
    """Repeated RUN/CANCEL cycles must not leak state, hang, or touch flash."""
    before = bytes(calib_dev.get_calibration_table())

    for cycle in range(3):
        log.info(f"RUN/CANCEL cycle {cycle + 1}/3")
        status = calib_dev.run_triggered_calibration(
            rs.triggered_calibration_mode.run, timeout_ms=TIMEOUT_MS)
        assert status.state == rs.triggered_calibration_state.health_check
        assert status.health_valid and status.candidate_valid

        cancel_status = calib_dev.run_triggered_calibration(
            rs.triggered_calibration_mode.cancel, timeout_ms=TIMEOUT_MS)
        assert cancel_status.state == rs.triggered_calibration_state.idle

    assert bytes(calib_dev.get_calibration_table()) == before
