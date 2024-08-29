from setuptools import setup

package_name = 'particle_filter'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/triton_gmapping.launch.py', 'launch/particle_filter.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seu_nome',
    maintainer_email='thxssio@gmail.com',
    description='Pacote para implementação de filtro de partículas em ROS 2.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'particle_filter_node = particle_filter.particle_filter_node:main',
            'position_publisher = particle_filter.position_publisher:main',
            'teleop_particle_filter = particle_filter.teleop_particle_filter:main',
        ],
    },
)
