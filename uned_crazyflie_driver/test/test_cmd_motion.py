# Copyright 2015 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from uned_crazyflie_driver.cmd_motion import CMD_Motion


class _FakeLogger:
    def __init__(self):
        self.errors = []
        self.warnings = []
        self.infos = []
        self.debugs = []

    def error(self, msg):
        self.errors.append(msg)

    def warning(self, msg):
        self.warnings.append(msg)

    def info(self, msg):
        self.infos.append(msg)

    def debug(self, msg):
        self.debugs.append(msg)


def test_pose_within_soft_limit_is_left_untouched():
    logger = _FakeLogger()
    cmd = CMD_Motion(logger, xy_lim=10)
    cmd.x, cmd.y = 1.0, -1.0  # well below 0.9 * xy_lim

    cmd.ckeck_pose()

    assert cmd.x == 1.0
    assert cmd.y == -1.0
    assert not logger.errors and not logger.warnings


def test_pose_beyond_soft_limit_only_warns():
    logger = _FakeLogger()
    cmd = CMD_Motion(logger, xy_lim=10)
    cmd.x = 9.5  # > 0.9 * xy_lim (9.0) but <= xy_lim (10)

    cmd.ckeck_pose()

    assert cmd.x == 9.5  # untouched, no hard clamp yet
    assert not logger.errors
    assert logger.warnings  # soft-limit warning was logged


def test_pose_beyond_hard_limit_is_clamped_to_85_percent():
    logger = _FakeLogger()
    cmd = CMD_Motion(logger, xy_lim=10)
    cmd.x = 15.0  # beyond xy_lim

    cmd.ckeck_pose()

    assert cmd.x == 0.85 * 10
    assert logger.errors  # hard-limit error was logged


def test_negative_pose_beyond_hard_limit_is_clamped_to_negative_85_percent():
    logger = _FakeLogger()
    cmd = CMD_Motion(logger, xy_lim=10)
    cmd.y = -15.0

    cmd.ckeck_pose()

    assert cmd.y == -0.85 * 10


def test_z_is_never_touched_by_ckeck_pose():
    # ckeck_pose() only clamps X/Y -- Z altitude is out of its scope.
    logger = _FakeLogger()
    cmd = CMD_Motion(logger, xy_lim=10)
    cmd.z = 999.0

    cmd.ckeck_pose()

    assert cmd.z == 999.0
