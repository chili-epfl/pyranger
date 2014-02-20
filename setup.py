 #!/usr/bin/env python
 # -*- coding: utf-8 -*-

import os
from distutils.core import setup

setup(name='ranger2',
      version='0.1',
      license='ISC',
      description='A collection of Python scripts to interface with EPFL Ranger2 robot',
      author='Séverin Lemaignan',
      author_email='severin.lemaignan@epfl.ch',
      package_dir = {'': 'src'},
      packages=['ranger', 'ranger.lowlevel', 'ranger.actions', 'ranger.helpers'],
      scripts=['bin/ranger_monitor', 'bin/autonomous_ranger'], 
      #data_files=[
      #            ('share/doc/pyrobots', ['AUTHORS', 'LICENSE', 'README'])]
      )
