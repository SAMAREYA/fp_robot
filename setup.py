from setuptools import find_packages, setup

package_name = 'fp_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/main_launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='henray',
    maintainer_email='henryalifian13@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'udp_node = fp_robot.main:main',
            'direction_pub = fp_robot.direction_pub:main',
            'maze_node = fp_robot.brain:main',
        ],
    },
)
