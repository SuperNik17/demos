from setuptools import find_packages, setup
import os
from glob import glob
from setuptools import find_packages

package_name = 'turtlebot_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),

        
    ],
    
    
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pasq-sti',
    maintainer_email='pasq-sti@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'move_turtlebot = turtlebot_pkg.move_forward:main',     
        'aruco_nav_node = turtlebot_pkg.aruco_nav_node:main',    
        'aruco_nav_node2_SMC = turtlebot_pkg.aruco_nav_node2_SMC:main',
        'aruco_nav_node2_PID = turtlebot_pkg.aruco_nav_node2_PID:main',  
        'gazebo_sim = turtlebot_pkg.gazebo:main', 
        'gazebo_sim2 = turtlebot_pkg.gazebo2:main',
        'gazebo_sim3 = turtlebot_pkg.gazebo3:main',
        'move_tes_robo_ostacolo_real= turtlebot_pkg.move_tes_robo_ostacolo_real:main',
        'move_doc_rob_intorno_ostacolo = turtlebot_pkg.move_doc_rob_intorno_ostacolo:main',
        'move_tes_robo_ostacolo_gazebo = turtlebot_pkg.move_tes_robo_ostacolo_gazebo:main',
        ],
    },
)
