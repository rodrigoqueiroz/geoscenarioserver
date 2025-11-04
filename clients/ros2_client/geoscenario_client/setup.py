from setuptools import setup

package_name = 'geoscenario_client'

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
    maintainer='Ian Colwell',
    maintainer_email='ian.colwell@hexagon.com',
    description='Python client for GeoScenarioServer',
    license='MIT',
    extras_require={
        'testing': [
            'pytest'
        ]
    },
    entry_points={
        'console_scripts': [
            'mock_co_simulator = geoscenario_client.mock_co_simulator:main'
        ],
    },
)
