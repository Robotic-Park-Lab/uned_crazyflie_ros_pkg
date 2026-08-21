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


import rclpy
import sys
import os

from rclpy.node import Node

from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5 import uic


class MainWindow(QMainWindow):
    def __init__(self, path):
        # Qt Stuff..
        super(MainWindow, self).__init__()
        # Load the .ui file made from Qt designer
        uic.loadUi(path, self)

        print("Development in progress ...")

# ^^^^^^^^^^^^^^^^^^^^^
# Node
# ^^^^^^^^^^^^^^^^^^^^^


class InterfaceGUI(Node):
    def __init__(self, aux):
        super().__init__('interface')
        self.initialize(aux)

    def initialize(self, aux):
        self.get_logger().info('InterfaceGUI::inicialize() ok.')
        app = QApplication(aux)
        path = os.path.dirname(os.path.abspath(__file__))
        path = path.replace('\\', '/') + '/main.ui'
        interfaz = MainWindow(path)
        interfaz.show()
        sys.exit(app.exec_())


def main(args=None):
    rclpy.init(args=args)
    interface_node = InterfaceGUI(sys.argv)
    rclpy.spin(interface_node)

    interface_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
