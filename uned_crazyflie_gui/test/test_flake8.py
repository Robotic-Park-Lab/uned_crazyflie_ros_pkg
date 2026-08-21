# Copyright 2017 Open Source Robotics Foundation, Inc.
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

from ament_flake8.main import main_with_errors
import pytest

# main_ui.py y logo_rc.py son generados por pyuic5/pyrcc5. interface_gui.py
# es una implementación alternativa sin conectar a ningún entry point,
# rota (import absoluto a un módulo inexistente) y pendiente de decidir si
# se arregla o se retira -- ver AUDIT.md en la rama doc, punto complejo #7.
GENERATED_OR_PENDING = [
    '--exclude', 'main_ui.py', 'logo_rc.py', 'interface_gui.py']


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    rc, errors = main_with_errors(argv=GENERATED_OR_PENDING)
    assert rc == 0, \
        'Found %d code style errors / warnings:\n' % len(errors) + \
        '\n'.join(errors)
