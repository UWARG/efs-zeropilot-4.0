from setuptools import setup, Extension
import os
import glob
import platform

zeropilot_root = '../zeropilot4.0'

# 1. Handle OS-specific compiler and linker settings
if platform.system() == "Windows":
    # MSVC Flags: /std:c++20 is needed for designated initializers
    # /D_USE_MATH_DEFINES ensures M_PI etc. are available
    compile_args = ['/std:c++20', '/D_USE_MATH_DEFINES', '/D_CRT_SECURE_NO_WARNINGS', '/wd4244']
    libraries = ['ws2_32'] 
else:
    # GCC/Clang Flags
    compile_args = ['-std=c++17']
    libraries = []
    
sources = ['zeropilot_wrapper.cpp']
sources += glob.glob(
    os.path.join(zeropilot_root, 'src', '**', '*.cpp'),
    recursive=True
)

zeropilot = Extension(
    'zeropilot',
    sources=sources,
    include_dirs=[
        '.',
        f'{zeropilot_root}/include/attitude_manager',
        f'{zeropilot_root}/include/system_manager',
        f'{zeropilot_root}/include/telemetry_manager',
        f'{zeropilot_root}/include/thread_msgs',
        f'{zeropilot_root}/include/driver_ifaces',
        '../external/c_library_v2',
        '../external/c_library_v2/common',
    ],
    libraries=libraries,
    extra_compile_args=compile_args,
)

setup(
    name='zeropilot',
    version='1.0',
    ext_modules=[zeropilot],
)
