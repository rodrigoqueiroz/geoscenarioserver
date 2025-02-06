from setuptools import setup

package_name = 'geoscenario_client'

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
    maintainer='Ian',
    maintainer_email='ian.colwell@hexagon.com',
    description='Python client for GeoScenarioServer',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'geoscenario_client = geoscenario_client.geoscenario_client:main'
        ],
    },
)
