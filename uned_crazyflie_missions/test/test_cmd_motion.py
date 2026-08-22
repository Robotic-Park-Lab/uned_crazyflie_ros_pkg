# Copyright 2026 Robotic Park Lab
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Robotic Park Lab nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Tests for uned_crazyflie_missions.cmd_motion.CMD_Motion.

CMD_Motion only needs a logger-like object (get_logger()-compatible
.error()/.warning() methods), so it is tested here with a small fake
logger instead of a real rclpy node -- no ROS init required.
"""

from uned_crazyflie_missions.cmd_motion import CMD_Motion, xy_lim, xy_warn


class FakeLogger:
    def __init__(self):
        self.errors = []
        self.warnings = []
        self.infos = []

    def error(self, msg):
        self.errors.append(msg)

    def warning(self, msg):
        self.warnings.append(msg)

    def info(self, msg):
        self.infos.append(msg)


class FakeCommander:
    def __init__(self):
        self.calls = []

    def go_to(self, *args):
        self.calls.append(('go_to', args))

    def send_setpoint(self, *args):
        self.calls.append(('send_setpoint', args))


class FakeCrazyflie:
    def __init__(self):
        self.high_level_commander = FakeCommander()
        self.commander = FakeCommander()


def test_within_warn_threshold_is_left_untouched():
    cmd = CMD_Motion(FakeLogger())
    cmd.x = xy_warn - 0.1
    cmd.y = xy_warn - 0.1
    cmd.ckeck_pose()
    assert cmd.x == xy_warn - 0.1
    assert cmd.y == xy_warn - 0.1
    assert cmd.logger.errors == []
    assert cmd.logger.warnings == []


def test_between_warn_and_limit_only_warns_and_keeps_the_value():
    cmd = CMD_Motion(FakeLogger())
    cmd.x = (xy_warn + xy_lim) / 2
    cmd.ckeck_pose()
    assert cmd.x == (xy_warn + xy_lim) / 2
    assert cmd.logger.errors == []
    assert len(cmd.logger.warnings) == 1


def test_beyond_limit_clamps_positive_x_to_ninety_five_percent_of_warn():
    cmd = CMD_Motion(FakeLogger())
    cmd.x = xy_lim + 1.0
    cmd.ckeck_pose()
    assert cmd.x == 0.95 * xy_warn
    assert len(cmd.logger.errors) == 1


def test_beyond_limit_clamps_negative_x_to_minus_ninety_five_percent_of_warn():
    cmd = CMD_Motion(FakeLogger())
    cmd.x = -(xy_lim + 1.0)
    cmd.ckeck_pose()
    assert cmd.x == -0.95 * xy_warn
    assert len(cmd.logger.errors) == 1


def test_beyond_limit_clamps_y_independently_of_x():
    cmd = CMD_Motion(FakeLogger())
    cmd.x = 0.0
    cmd.y = xy_lim + 1.0
    cmd.ckeck_pose()
    assert cmd.x == 0.0
    assert cmd.y == 0.95 * xy_warn


def test_send_pose_data_default_matches_the_pre_refactor_leader_follower_call():
    # Before the dedup, leader_follower.py always called go_to(..., 0.5)
    # with no relative flag; send_pose_data_ must reproduce that exact
    # behaviour when called with its default argument.
    cmd = CMD_Motion(FakeLogger())
    cmd.x, cmd.y, cmd.z, cmd.yaw = 1.0, 2.0, 3.0, 0.5
    cf = FakeCrazyflie()
    cmd.send_pose_data_(cf)
    assert cf.high_level_commander.calls == [('go_to', (1.0, 2.0, 3.0, 0.5, 0.5))]


def test_send_pose_data_relative_uses_a_different_velocity():
    cmd = CMD_Motion(FakeLogger())
    cmd.x, cmd.y, cmd.z, cmd.yaw = 1.0, 2.0, 3.0, 0.5
    cf = FakeCrazyflie()
    cmd.send_pose_data_(cf, relative_pose=True)
    assert cf.high_level_commander.calls == [('go_to', (1.0, 2.0, 3.0, 0.5, 0.1))]


def test_send_offboard_setpoint_forwards_roll_pitch_yaw_thrust():
    cmd = CMD_Motion(FakeLogger())
    cmd.roll, cmd.pitch, cmd.yaw, cmd.thrust = 0.1, 0.2, 0.3, 40000
    cf = FakeCrazyflie()
    cmd.send_offboard_setpoint_(cf)
    assert cf.commander.calls == [('send_setpoint', (0.1, 0.2, 0.3, 40000))]
