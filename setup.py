
#! /usr/bin/env python

descr = "Teaching semantics and skills for human robot collaboration"

from distutils.core import setup

setup(
    name='cogrob-tut-hri',
    version='1.0',
    maintainer='Alexandre Angleraud',
    maintainer_email='alexandre.angleraud@tut.fi',
    packages=['franka_tut_reasoning'],
    package_dir={'': 'franka_tut_reasoning/src'},
    license='LICENSE',
    description= descr,
    long_description=open('README.md').read(),
)
