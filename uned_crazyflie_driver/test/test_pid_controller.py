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

from uned_crazyflie_driver.pid_controller import PIDController


def test_pure_proportional_response():
    # Ki=Kd=0: only the proportional term should contribute.
    pid = PIDController(
        Kp=2.0, Ki=0.0, Kd=0.0, Td=1.0, Nd=1.0,
        UpperLimit=0.0, LowerLimit=0.0, ai=0.0, co=0.0)
    pid.error[0] = 5.0
    out = pid.update(dt=0.1)
    assert out == 10.0


def test_upper_and_lower_limit_saturate_output():
    pid = PIDController(
        Kp=10.0, Ki=0.0, Kd=0.0, Td=1.0, Nd=1.0,
        UpperLimit=3.0, LowerLimit=-3.0, ai=0.0, co=0.0)
    pid.error[0] = 5.0  # raw P output would be 50, well above UpperLimit
    assert pid.update(dt=0.1) == 3.0

    pid.error[0] = -5.0
    assert pid.update(dt=0.1) == -3.0


def test_zero_upper_limit_disables_saturation():
    # `if not self.UpperLimit == 0.0` in update(): UpperLimit == 0.0 means
    # "no limiting", not "clamp to zero" -- real behaviour of the code,
    # not a bug to "fix" in the test.
    pid = PIDController(
        Kp=10.0, Ki=0.0, Kd=0.0, Td=1.0, Nd=1.0,
        UpperLimit=0.0, LowerLimit=0.0, ai=0.0, co=0.0)
    pid.error[0] = 5.0
    assert pid.update(dt=0.1) == 50.0


def test_integral_uses_previous_error_not_current():
    # integral += Ki * self.error[1] * dt, and error[1] is only updated to
    # the current error[0] at the *end* of update() -- so the integral
    # term is always one step behind the error you just set. Real,
    # deliberate behaviour of this controller; the test documents it.
    pid = PIDController(
        Kp=0.0, Ki=1.0, Kd=0.0, Td=1.0, Nd=1.0,
        UpperLimit=0.0, LowerLimit=0.0, ai=0.0, co=0.0)

    pid.error[0] = 4.0
    out1 = pid.update(dt=0.5)
    # error[1] was still 0 (initial value) when this call ran.
    assert out1 == 0.0
    assert pid.integral == 0.0

    pid.error[0] = 2.0
    out2 = pid.update(dt=0.5)
    # This call used error[1] == 4.0 (the *previous* error[0]).
    assert out2 == 2.0
    assert pid.integral == 2.0


def test_eval_threshold_triggers_on_large_enough_delta():
    pid = PIDController(
        Kp=0.0, Ki=0.0, Kd=0.0, Td=1.0, Nd=1.0,
        UpperLimit=0.0, LowerLimit=0.0, ai=0.0, co=1.0)
    # First call: trigger_last_signal starts at 0, so any nonzero error
    # bigger than the fixed threshold `co` should trigger.
    assert pid.eval_threshold(signal=0.0, ref=5.0) is True
    # Right after triggering, the delta from the just-stored signal is 0,
    # so it should not trigger again immediately.
    assert pid.eval_threshold(signal=0.0, ref=5.0) is False


def test_rele_update_switches_sign_outside_range():
    pid = PIDController(
        Kp=0.0, Ki=0.0, Kd=0.0, Td=1.0, Nd=1.0,
        UpperLimit=0.0, LowerLimit=0.0, ai=0.0, co=0.0)
    pid.range = 1.0
    pid.cmd = 5.0

    pid.error[0] = 2.0  # above range, relay not yet active -> +cmd, arms it
    assert pid.rele_update(dt=0.1) == 5.0
    assert pid.rele is True

    pid.error[0] = 2.0  # still above -range while armed -> +cmd stays
    assert pid.rele_update(dt=0.1) == 5.0

    pid.error[0] = -2.0  # below -range while armed -> switches to -cmd
    assert pid.rele_update(dt=0.1) == -5.0
    assert pid.rele is False
