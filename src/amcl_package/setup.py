from setuptools import setup

package_name = 'amcl_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thxssio',
    maintainer_email='thxssio@gmail.com',
    description='AMCL package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'amcl_launch = amcl_package.amcl_launch:main',
        ],
    },
)
