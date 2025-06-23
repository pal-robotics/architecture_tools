from setuptools import find_packages
from setuptools import setup

package_name = 'ament_archlint'

setup(
    name=package_name,
    version='1.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
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
            'ament_archlint = ament_archlint.main:main',
        ],
        'pytest11': [
            'ament_archlint = ament_archlint.pytest_marker',
        ],
    },
)
