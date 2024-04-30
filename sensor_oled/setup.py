from setuptools import setup

package_name = 'sensor_oled'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tegra',
    maintainer_email='tegra@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor = sensor_oled.sensor:main',
            'oled = sensor_oled.oled:main',
            'trackbar = sensor_oled.trackbar:main'
        ],
    },
)
