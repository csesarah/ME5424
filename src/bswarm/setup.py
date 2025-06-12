from setuptools import setup

package_name = 'bswarm'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='FirstName LastName',
    maintainer_email='null@email.com',
    description='lorem ipsum',
    license='Apache-2.0',
    tests_require=['pytest'],
)