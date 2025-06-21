from setuptools import setup

package_name = 'tracking'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],  # no packages since scripts folder not a Python package
    scripts=['scripts/target_publisher.py'],  # install script as executable
    install_requires=['setuptools'],
    zip_safe=True,
    author='Bernardo Martinez',
    author_email='jbmm@ugr.es',
    description='Tracking package',
    license='Apache License 2.0',
)
