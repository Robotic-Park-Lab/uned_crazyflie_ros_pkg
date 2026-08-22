import os
from setuptools import setup

package_name = 'uned_crazyflie_webots'

data_files = [
    ('share/ament_index/resource_index/packages',
     ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]


def package_files(data_files, directory_list):
    paths_dict = {}
    for directory in directory_list:
        for (path, directories, filenames) in os.walk(directory):
            for filename in filenames:
                file_path = os.path.join(path, filename)
                install_path = os.path.join('share', package_name, path)
                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)
                else:
                    paths_dict[install_path] = [file_path]

    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))

    return data_files


setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=package_files(data_files, ['resources/', 'launch/', 'worlds/']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Francisco José Mañas Álvarez',
    maintainer_email='fjmanas@dia.uned.es',
    description='Driver y controlador ROS 2 para simular el crazyflie 2.1 en Webots',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    # Sin entry_points: ninguno de los dos drivers de este paquete se
    # ejecuta con 'ros2 run'. webots_ros2_driver los carga en tiempo de
    # simulación por ruta de clase, vía la etiqueta <plugin type="..."/>
    # de resources/crazyflie.urdf (CrazyflieWebotsDriver, controlador
    # propio) y resources/crazyflie_firmware.urdf (CrazyflieWebotsDriver
    # de crazyflie_driver_firmware.py, firmware real / gemelo digital).
    # Antes había aquí un entry_point 'crazyflie_driver' que apuntaba a
    # crazyflie_driver:main -- esa función nunca existió en el módulo,
    # así que 'ros2 run uned_crazyflie_webots crazyflie_driver' ya
    # fallaba antes de este cambio; era vestigial, no una entrada real.
)
