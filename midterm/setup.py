from setuptools import setup

package_name = 'midterm'

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
    maintainer='deepracer',
    maintainer_email='deepracer@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'sub = midterm.run_midterm_subscriber:main',
        	'pub = midterm.run_midterm_publisher:main',
        ],
    },
)
