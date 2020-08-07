from setuptools import setup, Extension

module = Extension('virxrlcu', sources=['virxrlcu.c'])

setup(name='VirxERLU CLib',
      version='1.1',
      description='C modules for VirxERLU',
      long_description="""# VirxERLU

Big thanks to ddthj/GoslingUtils for the basis of VirxERLU. This version, however, improves upon many things, including pathfinding, finding shots, and aerials.
VirxERLU-CLib is the C library for VirxERLU.
""",
      ext_modules=[module],
      license="Unlicense",
      author='VirxEC',
      author_email='virx@virxcase.dev',
      url="https://github.com/VirxEC/VirxERLU"
      )
