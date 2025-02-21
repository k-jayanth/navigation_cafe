import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join("share", package_name, "launch/"), glob("./launch/*")),
        (os.path.join("share", package_name, "config/"), glob("./config/*")),
        (os.path.join("share", package_name, "params/"), glob("./params/*")),
        (
            os.path.join("share", package_name, "worlds/"),
            list(glob("./worlds/**/*.world", recursive=True)),
        ),
        (
            os.path.join("share", package_name, "worlds/"),
            list(glob("./worlds/**/*.model", recursive=True)),
        ),
        (
            os.path.join("share", package_name, "maps/"),
            list(glob("./maps/**/*.pgm", recursive=True)),
        ),
        (
            os.path.join("share", package_name, "maps/"),
            list(glob("./maps/**/*.yaml", recursive=True)),
        ),
        (
            os.path.join("share", package_name, "behavior_trees/"),
            glob("./behavior_trees/*"),
        ),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jayanth',
    maintainer_email='jayanth@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
