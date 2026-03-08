from setuptools import setup

package_name = 'kilin_panel'

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    py_modules=[
        'kilin_panel',
        'mainwindow',
        'module_panel',
        'module_widget',
        'motor_node',
        'power_node',
        'trigger_node'
    ],
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resources/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ian920201',
    maintainer_email='ian20030201@gmail.com',
    description='Kilin robot PyQt5 control panel for power and motor management',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kilin_panel = kilin_panel:main',
        ],
    },
)
