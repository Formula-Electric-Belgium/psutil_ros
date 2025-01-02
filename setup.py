from setuptools import find_packages, setup

package_name = 'psutil_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'psutil', 'rclpy'],
    zip_safe=True,
    maintainer='tiesto1306',
    maintainer_email='tiestotjolle@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'psutil_ros_node = scripts.psutil_ros_node:main',
        ],
    },
)
