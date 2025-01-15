from setuptools import find_packages
from setuptools import setup

package_name = 'ament_skilllint'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + "/schema", ['schema/skill.schema.json']),
    ],
    install_requires=['setuptools'],
    package_data={'': [
        'schema/skill.schema.json',
    ]},
    zip_safe=True,
    author='Séverin Lemaignan',
    author_email='severin.lemaignan@pal-robotics.com',
    maintainer='Séverin Lemaignan',
    maintainer_email='severin.lemaignan@pal-robotics.com',
    # url='https://github.com/ament/ament_lint',
    # download_url='https://github.com/ament/ament_lint/releases',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Check the manifest of ROS skills for conformance.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ament_skilllint = ament_skilllint.main:main',
        ],
        'pytest11': [
            'ament_skilllint = ament_skilllint.pytest_marker',
        ],
    },
)
