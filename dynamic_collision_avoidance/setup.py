from setuptools import setup

package_name = 'dynamic_collision_avoidance'

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
    maintainer='root',
    maintainer_email='brsrikrishna@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dynamic_collision_avoidance_sub = dynamic_collision_avoidance.dynamic_collision_avoidance_sub:main',
            'dynamic_collision_avoidance_pub = dynamic_collision_avoidance.dynamic_collision_avoidance_pub:main',
        ],
    },
)
