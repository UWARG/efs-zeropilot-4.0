from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext
import os
import glob
import platform


# Strip C++-only flags when compiling C sources (e.g. CMSIS-DSP)
class BuildExt(build_ext):
    def build_extensions(self):
        original_compile = self.compiler._compile

        def _compile(obj, src, ext, cc_args, extra_postargs, pp_opts):
            if src.endswith('.c'):
                extra_postargs = [a for a in extra_postargs
                                  if not (a.startswith('-std=c++') or a.startswith('/std:c++'))]
            return original_compile(obj, src, ext, cc_args, extra_postargs, pp_opts)

        self.compiler._compile = _compile
        super().build_extensions()

zeropilot_root = '../zeropilot4.0'

# Vehicle type: set V=QUAD or V=PLANE (default: PLANE)
VEHICLE = os.environ.get('V', 'PLANE').upper()
if VEHICLE not in ('QUADCOPTER', 'PLANE'):
    raise ValueError(f"ZP_VEHICLE must be QUADCOPTER or PLANE, got: {VEHICLE}")

# Handle OS-specific compiler and linker settings
if platform.system() == "Windows":
    compile_args = ['/std:c++20', '/D_USE_MATH_DEFINES', '/D_CRT_SECURE_NO_WARNINGS', '/wd4244', '/D__GNUC_PYTHON__', f'/D{VEHICLE}']
    libraries = ['ws2_32']
else:
    # GCC/Clang Flags
    compile_args = ['-std=c++17', '-D__GNUC_PYTHON__', f'-D{VEHICLE}']
    libraries = []

# Collect ZeroPilot source files
sources = ['zeropilot_wrapper.cpp']
sources += glob.glob(
    os.path.join(zeropilot_root, 'src', '**', '*.cpp'),
    recursive=True
)

# Add on CMSIS-DSP source files
sources += [
    '../external/CMSIS-DSP/Source/TransformFunctions/arm_rfft_fast_f32.c',
    '../external/CMSIS-DSP/Source/TransformFunctions/arm_rfft_fast_init_f32.c',
    '../external/CMSIS-DSP/Source/TransformFunctions/arm_cfft_f32.c',
    '../external/CMSIS-DSP/Source/TransformFunctions/arm_cfft_init_f32.c',
    '../external/CMSIS-DSP/Source/TransformFunctions/arm_cfft_radix8_f32.c',
    '../external/CMSIS-DSP/Source/TransformFunctions/arm_bitreversal2.c',
    '../external/CMSIS-DSP/Source/ComplexMathFunctions/arm_cmplx_mag_f32.c',
    '../external/CMSIS-DSP/Source/FastMathFunctions/arm_sin_f32.c',
    '../external/CMSIS-DSP/Source/FastMathFunctions/arm_cos_f32.c',
    '../external/CMSIS-DSP/Source/CommonTables/arm_common_tables.c',
    '../external/CMSIS-DSP/Source/CommonTables/arm_const_structs.c',
]

zeropilot = Extension(
    'zeropilot',
    sources=sources,
    include_dirs=[
        '.',
        f'{zeropilot_root}/include/attitude_manager',
        f'{zeropilot_root}/include/system_manager',
        f'{zeropilot_root}/include/telemetry_manager',
        f'{zeropilot_root}/include/exmem_manager',
        f'{zeropilot_root}/include/thread_msgs',
        f'{zeropilot_root}/include/driver_ifaces',
        f'{zeropilot_root}/include/zp_param',
        '../external/c_library_v2',
        '../external/c_library_v2/common',
        '../external/CMSIS-DSP/Include',
        '../external/CMSIS-DSP/PrivateInclude',
    ],
    libraries=libraries,
    extra_compile_args=compile_args,
)

setup(
    name='zeropilot',
    version='1.0',
    ext_modules=[zeropilot],
    cmdclass={'build_ext': BuildExt},
)
