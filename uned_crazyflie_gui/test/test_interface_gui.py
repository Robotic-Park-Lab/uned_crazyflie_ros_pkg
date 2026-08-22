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
Headless smoke test for MainWindow.

Forces QT_QPA_PLATFORM=offscreen (no real display needed) and checks that
the window builds from main.ui, loads its Qt resources (logo_rc) and
closes cleanly -- not a UI/interaction test, just that interface_node does
not crash on startup, which is the kind of regression a plain
flake8/pep257 pass would never catch (e.g. the real bug this exact class
had before commit a233491 of this refactor: main_ui.py doing a bare
`import logo_rc` that broke once the file moved into a proper Python
package).
"""

import os

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

import rclpy  # noqa: E402
from PyQt5.QtWidgets import QApplication  # noqa: E402

from uned_crazyflie_gui.interface_gui import MainWindow  # noqa: E402


def test_main_window_builds_and_closes_without_crashing():
    rclpy.init()
    node = rclpy.create_node('test_interface_gui')
    # A QApplication instance must exist before creating any QWidget, and
    # a reference to it must be kept alive for the whole test: if the
    # object returned here isn't held anywhere, Python garbage-collects
    # it immediately (nothing else references it), which tears down the
    # underlying Qt application and crashes the very next QWidget created.
    app = QApplication.instance() or QApplication([])

    window = MainWindow(node)
    try:
        assert app is not None
        assert window.windowTitle() == 'Robotic Park. Crazyflie 2.X'
        assert window.GraphsWidget.count() >= 1
    finally:
        window.close()
        node.destroy_node()
        rclpy.shutdown()
