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

from types import SimpleNamespace

from uned_crazyflie_driver.pid_params import apply_controller_params


def _msg(id_, kp=1.0, ki=2.0, kd=3.0, co=4.0, ai=5.0, upperlimit=6.0, lowerlimit=-6.0, nd=7.0):
    return SimpleNamespace(
        id=id_, kp=kp, ki=ki, kd=kd, co=co, ai=ai,
        upperlimit=upperlimit, lowerlimit=lowerlimit, nd=nd)


def _recorder():
    calls = []
    return calls, (lambda path, value: calls.append((path, value)))


def test_position_axis_continuous_uses_ctlpid_group_no_co_ai():
    calls, set_value = _recorder()
    msg = _msg('x')
    apply_controller_params(set_value, lambda s: None, msg, event_based=False)

    paths = dict(calls)
    assert paths['posCtlPid.xKp'] == msg.kp
    assert paths['posCtlPid.xKi'] == msg.ki
    assert paths['posCtlPid.xKd'] == msg.kd
    assert 'posCtlPid.xCo' not in paths and 'posCtlPid.xAi' not in paths
    # x axis' velocity-limit key: id + 'yVelMax' -> 'xyVelMax'
    assert paths['posCtlPid.xyVelMax'] == msg.upperlimit


def test_position_axis_event_based_uses_ebctlpid_group_with_co_ai():
    calls, set_value = _recorder()
    msg = _msg('y')
    apply_controller_params(set_value, lambda s: None, msg, event_based=True)

    paths = dict(calls)
    assert paths['posEbCtlPid.yKp'] == msg.kp
    assert paths['posEbCtlPid.yCo'] == msg.co
    assert paths['posEbCtlPid.yAi'] == msg.ai
    # y axis' velocity-limit key: 'x' + id + 'VelMax' -> 'xyVelMax'
    # (x and y share the same real firmware parameter, by design).
    assert paths['posEbCtlPid.xyVelMax'] == msg.upperlimit


def test_z_axis_velmax_key_is_its_own_axis_not_shared():
    calls, set_value = _recorder()
    msg = _msg('z')
    apply_controller_params(set_value, lambda s: None, msg, event_based=False)

    paths = dict(calls)
    assert paths['posCtlPid.zVelMax'] == msg.upperlimit


def test_velocity_axis_has_no_velmax_key():
    calls, set_value = _recorder()
    msg = _msg('vx')
    apply_controller_params(set_value, lambda s: None, msg, event_based=False)

    paths = dict(calls)
    assert paths['velCtlPid.vxKp'] == msg.kp
    assert not any(k.endswith('VelMax') for k in paths)


def test_attitude_axis_same_group_regardless_of_event_based():
    for event_based in (False, True):
        calls, set_value = _recorder()
        msg = _msg('roll')
        apply_controller_params(set_value, lambda s: None, msg, event_based=event_based)

        paths = dict(calls)
        assert paths['pid_attitude.roll_kp'] == msg.kp
        assert paths['pid_attitude.roll_ki'] == msg.ki
        assert paths['pid_attitude.roll_kd'] == msg.kd


def test_rate_axis_strips_leading_d_from_firmware_param_name():
    calls, set_value = _recorder()
    msg = _msg('droll')
    apply_controller_params(set_value, lambda s: None, msg, event_based=False)

    paths = dict(calls)
    # message id is 'droll' but the real firmware group uses 'roll_kp', not 'droll_kp'.
    assert paths['pid_rate.roll_kp'] == msg.kp
    assert 'pid_rate.droll_kp' not in paths


def test_log_info_called_once_with_summary():
    calls, set_value = _recorder()
    msg = _msg('x')
    logged = []
    apply_controller_params(set_value, logged.append, msg, event_based=False)

    assert len(logged) == 1
    assert 'Kp' in logged[0] and 'Ki' in logged[0] and 'Kd' in logged[0]
