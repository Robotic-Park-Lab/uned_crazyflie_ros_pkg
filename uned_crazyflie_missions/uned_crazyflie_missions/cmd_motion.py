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
CMD_Motion compartido entre leader_follower.py y shape_based_formation_control.py.

Antes de esta refactorización estaba duplicado byte a byte entre ambos
ficheros salvo por `send_pose_data_`: la versión de
`shape_based_formation_control.py` (la que se usa aquí) añadía un
parámetro `relative_pose` con un valor por defecto que reproduce
exactamente el comportamiento que tenía la versión de
`leader_follower.py` cuando se llama sin ese argumento -- no hay cambio
de comportamiento para ninguno de los dos nodos.
"""

xy_warn = 1.3
xy_lim = 1.5


class CMD_Motion():
    def __init__(self, logger):
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0
        self.thrust = 0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.logger = logger
        self.flight_time = 1.0

    def ckeck_pose(self):
        # X Check
        if abs(self.x) > xy_warn:
            if abs(self.x) > xy_lim:
                self.logger.error('X: Error')
                if self.x > 0:
                    self.x = 0.95 * xy_warn
                else:
                    self.x = -0.95 * xy_warn
                self.logger.warning('New Point: %s' % self.pose_str_())
            else:
                self.logger.warning('X: Warning')
        # Y Check
        if abs(self.y) > xy_warn:
            if abs(self.y) > xy_lim:
                self.logger.error('Y: Error')
                if self.y > 0:
                    self.y = 0.95 * xy_warn
                else:
                    self.y = -0.95 * xy_warn
                self.logger.warning('New Point: %s' % self.pose_str_())
            else:
                self.logger.warning('Y: Warning')

    def str_(self):
        return ('Thrust: ' + str(self.thrust) + ' Roll: ' + str(self.roll) +
                ' Pitch: ' + str(self.pitch) + ' Yaw: ' + str(self.yaw))

    def pose_str_(self):
        return ('X: ' + str(self.x) + ' Y: ' + str(self.y) +
                ' Z: ' + str(self.z) + ' Yaw: ' + str(self.yaw))

    def send_pose_data_(self, cf, relative_pose=False):
        if (relative_pose):
            # cf.high_level_commander.go_to(self.x, self.y, self.z, self.yaw, 0.5,
            # relative=relative_pose)
            self.logger.info('Goal Pose: %s' % (self.pose_str_()))
            cf.high_level_commander.go_to(self.x, self.y, self.z, self.yaw, 0.1)
        else:
            self.logger.info('Goal Pose: %s' % (self.pose_str_()))
            # cf.commander.send_position_setpoint(self.x, self.y, self.z, self.yaw)
            cf.high_level_commander.go_to(self.x, self.y, self.z, self.yaw, 0.5)

    def send_offboard_setpoint_(self, cf):
        self.logger.info('Command: %s' % self.str_())
        cf.commander.send_setpoint(self.roll, self.pitch, self.yaw,
                                   self.thrust)
