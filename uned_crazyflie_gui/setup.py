from setuptools import setup

package_name = 'uned_crazyflie_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/rqt', ['rqt/crazyflie.perspective']),
        ('share/' + package_name + '/rviz', ['rviz/crazyflie.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Francisco José Mañas Álvarez',
    maintainer_email='fjmanas@dia.uned.es',
    description='Interfaz gráfica PyQt para el manejo individual del crazyflie 2.1, con una'
    ' perspectiva de RQT y un fichero de RViz genéricos para visualizar cualquier Crazyflie',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'interface_node = uned_crazyflie_gui.interface_gui:main'
        ],
    },
)
