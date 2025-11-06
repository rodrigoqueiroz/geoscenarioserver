from setuptools import setup

package_name = 'geoscenario_server'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Michal Antkiewicz, Spencer Delcore',
    maintainer_email='michal.antkiewicz@uwaterloo.ca, sdelcore@uwaterloo.ca',
    description='Python server for GeoScenarioServer',
    license='MIT',
    extras_require={
        'testing': [
            'pytest'
        ]
    },
    entry_points={
        'console_scripts': [
            'geoscenario_server = geoscenario_server.geoscenario_server:main'
        ],
    },
)
