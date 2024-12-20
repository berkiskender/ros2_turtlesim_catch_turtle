from setuptools import find_packages, setup

package_name = 'my_turtlesim_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='berk',
    maintainer_email='berk@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "turtle_spawn_node = my_turtlesim_project.turtle_spawn_node:main",
            "turtle_controller_node = my_turtlesim_project.turtle_controller_node:main",
        ],
    },
)
