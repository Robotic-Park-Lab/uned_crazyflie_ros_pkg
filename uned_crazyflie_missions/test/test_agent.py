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
Tests for uned_crazyflie_missions.agent.Agent.

Agent only touches its `parent` through two duck-typed calls
(get_logger() and create_subscription()), so it is tested here with a
minimal fake parent instead of a real rclpy node -- no ROS init required.
geometry_msgs.msg.Pose itself is a real ROS message type, which is fine
to import without initializing ROS.
"""

from geometry_msgs.msg import Pose

from uned_crazyflie_missions.agent import Agent


class FakeLogger:
    def info(self, msg):
        pass


class FakeParent:
    def __init__(self):
        self.subscriptions = []

    def get_logger(self):
        return FakeLogger()

    def create_subscription(self, msg_type, topic, callback, qos):
        self.subscriptions.append((msg_type, topic, callback, qos))
        return object()


def test_constructor_stores_id_and_position():
    parent = FakeParent()
    agent = Agent(parent, x=1.0, y=2.0, z=3.0, id='dron02')
    assert agent.id == 'dron02'
    assert (agent.x, agent.y, agent.z) == (1.0, 2.0, 3.0)


def test_constructor_subscribes_to_the_agent_pose_topic():
    parent = FakeParent()
    Agent(parent, x=0.0, y=0.0, z=0.0, id='dron03')
    assert len(parent.subscriptions) == 1
    msg_type, topic, _callback, _qos = parent.subscriptions[0]
    assert msg_type is Pose
    assert topic == 'dron03/pose'


def test_str_includes_id_and_coordinates():
    agent = Agent(FakeParent(), x=1.5, y=-2.5, z=0.0, id='dron01')
    text = agent.str_()
    assert 'dron01' in text
    assert '1.5' in text
    assert '-2.5' in text


def test_gtpose_callback_stores_the_received_pose():
    agent = Agent(FakeParent(), x=0.0, y=0.0, z=0.0, id='dron01')
    msg = Pose()
    msg.position.x = 9.0
    agent.gtpose_callback(msg)
    assert agent.pose is msg
    assert agent.pose.position.x == 9.0
