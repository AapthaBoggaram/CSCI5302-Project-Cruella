from setuptools import setup

package_name = 'stop_sign'

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
        'stop_sign_sub = stop_sign.stop_sign_subscriber:main',
        'stop_sign_pub = stop_sign.stop_sign_publisher:main',
        ],
    },
)
