from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['franka_tut_reasoning'],
    package_dir={'': 'src'},
    install_requires=[
          'breathe',
      ]
    )

setup(**d)
