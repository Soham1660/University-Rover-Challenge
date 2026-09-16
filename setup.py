from setuptools import find_packages, setup

package_name = 'urc_gui'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='soham',
    maintainer_email='aggarwalsoham2008@gmail.com',
    description='Operator GUI and rover simulator for the URC robotics club challenge',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rover_sim_node = urc_gui.rover_sim_node:main',
            'operator_gui_node = urc_gui.operator_gui_node:main',
        ],
    },
)
