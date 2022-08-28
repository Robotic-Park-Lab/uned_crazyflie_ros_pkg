import logging
import time
import rclpy
import numpy as np
import math
import sys
import os

from rclpy.node import Node

from qt_gui.settings import Settings
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5 import uic

class MainWindow(QMainWindow):
	def __init__(self, path):
		#Qt Stuff..
		super(MainWindow, self).__init__()
		# Load the .ui file made from Qt designer
		uic.loadUi(path, self)

		print("Development in progress ...")

## ^^^^^^^^^^^^^^^^^^^^^
## Node
## ^^^^^^^^^^^^^^^^^^^^^
class InterfaceGUI(Node):
    def __init__(self, aux):
        super().__init__('interface')
        self.initialize(aux)

    def initialize(self, aux):
        self.get_logger().info('InterfaceGUI::inicialize() ok.')
        app = QApplication(aux)
        path = os.path.dirname(os.path.abspath(__file__))
        path = path.replace('\\', '/')+'/main.ui'
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