import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'kratos_maze'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'models/sample'), glob('models/sample/*')),
        (os.path.join('share', package_name, 'models/maze10x10'), glob('models/maze10x10/*')),
        (os.path.join('share', package_name, 'models/maze5x5'), glob('models/maze5x5/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='soham',
    maintainer_email='soham@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dfs = src.dfs:main',
            'wall_follower = src.wall_follower:main'
            # add your nodes here
        ],
    },
)
