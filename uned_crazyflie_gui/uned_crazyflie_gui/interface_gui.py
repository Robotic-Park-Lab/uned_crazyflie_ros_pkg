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
Interfaz gráfica de escritorio para el crazyflie 2.X.

Versión básica y conectada como entry point real (`interface_node`):
muestra la ventana diseñada en Qt Designer (`main.ui`) tal cual, sin
paneles embebidos de rqt_plot/rqt_graph -- ver la rama `doc`
(FUTURO_interface_gui.md) para el alcance completo pendiente.
"""
import os
import sys

import rclpy

from PyQt5.QtWidgets import QApplication, QLabel, QMainWindow
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QIcon
from PyQt5 import uic

from uned_crazyflie_gui import logo_rc  # noqa: F401  (recursos Qt: iconos/logo)


class MainWindow(QMainWindow):

    def __init__(self, node):
        super(MainWindow, self).__init__()
        self.node = node
        # Load the .ui file made from Qt designer
        ui_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'main.ui')
        uic.loadUi(ui_path, self)

        self.setWindowIcon(QIcon(':/logo/figs/LogoRoboticPark.png'))
        self.setWindowTitle('Robotic Park. Crazyflie 2.X')

        # Los paneles de rqt_plot/rqt_graph de la versión original no se
        # han conectado en esta versión básica (dependían de embeber
        # ventanas X11 externas vía xdotool, poco fiable). Placeholder
        # informativo en su lugar -- ver FUTURO_interface_gui.md.
        placeholder = QLabel(
            "Gráficas en tiempo real: pendiente (ver rama doc,\n"
            "FUTURO_interface_gui.md).")
        placeholder.setAlignment(Qt.AlignCenter)
        self.GraphsWidget.addTab(placeholder, 'Plots')

        self.Close_action.triggered.connect(self.cerrar)
        self.Close_action.setShortcut('Ctrl+W')

        self.node.get_logger().info('MainWindow::inicialize() ok.')

    def cerrar(self):
        QApplication.quit()


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('uned_cf_interface')

    app = QApplication(sys.argv)
    window = MainWindow(node)
    window.show()
    exit_code = app.exec_()

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
