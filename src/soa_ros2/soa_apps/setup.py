from setuptools import find_packages, setup

package_name = 'soa_apps'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='42076119+htchr@users.noreply.github.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'go_to_joint_states = soa_apps.go_to_joint_states:main',
            'go_to_joint_states_bimanual = soa_apps.go_to_joint_states_bimanual:main',
            'orchestrator = soa_apps.orchestrator:main',
        ],
    },
)
