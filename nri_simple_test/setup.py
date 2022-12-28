from setuptools import setup

package_name = 'nri_simple_test'

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
    maintainer='nvidia',
    maintainer_email='nvidia@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'gpswaypoint_test = nri_simple_test.gpswaypoint_test:main',
             'xyzwaypoint_test = nri_simple_test.xyzwaypoint_test:main',
             'gpswaypointmulti_test = nri_simple_test.gpswaypointmulti_test:main',
             'xyzwaypointmulti_test = nri_simple_test.xyzwaypointmulti_test:main',             
        ],
    },
)
