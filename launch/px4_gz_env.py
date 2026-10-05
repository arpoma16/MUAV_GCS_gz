"""Helpers compartidos por los launch de muav_gcs_gz: descubrimiento de PX4 y entorno de Gazebo.

Replica lo que PX4 genera en <build>/gz_env.sh (plantilla gz_bridge/gz_env.sh.in): los modelos
de PX4 (x500_base, x500_gimbal, gimbal, ...) que este paquete referencia con model://, los
plugins de gz compilados por PX4 (MotorFailurePlugin, OpticalFlowSystem, ...) y el server.config
propio de PX4. Los mundos de este paquete no declaran plugins, asi que dependen de ese
server.config para tener sensores, viento, etc. El servidor de Gazebo tiene que verlos.
"""

import ctypes.util
import os
import shutil

from launch.actions import AppendEnvironmentVariable, SetEnvironmentVariable

PX4_BUILD_TARGET = 'px4_sitl_default'


def nvidia_gpu_available():
    """True si hay una NVIDIA utilizable: modulo del kernel cargado y libreria GLX instalada."""
    return (os.path.exists('/proc/driver/nvidia/version')
            and ctypes.util.find_library('GLX_nvidia') is not None)


def gz_gpu_env_actions():
    """Acciones de launch que mandan el render de la GUI de gz a la NVIDIA (PRIME render offload).

    En laptops hibridas (Intel + NVIDIA, prime-select 'on-demand') el X corre sobre la iGPU y
    las apps GLX usan Mesa salvo que se pida offload; sin esto la GUI de gz no usa la NVIDIA.
    Solo se activa si se detecta una NVIDIA utilizable, asi que no afecta a maquinas sin ella.

    - Si el usuario ya fijo __GLX_VENDOR_LIBRARY_NAME o __NV_PRIME_RENDER_OFFLOAD, no se toca.
    - MUAV_NVIDIA_OFFLOAD=0 (o false/off/no) lo desactiva.
    """
    if os.environ.get('MUAV_NVIDIA_OFFLOAD', 'auto').lower() in ('0', 'false', 'off', 'no'):
        print("[DEBUG] NVIDIA PRIME offload disabled by MUAV_NVIDIA_OFFLOAD")
        return []
    if '__GLX_VENDOR_LIBRARY_NAME' in os.environ or '__NV_PRIME_RENDER_OFFLOAD' in os.environ:
        print("[DEBUG] NVIDIA PRIME offload variables already set, leaving them as they are")
        return []
    if not nvidia_gpu_available():
        print("[DEBUG] No usable NVIDIA GPU detected, using the default renderer")
        return []

    print("[DEBUG] NVIDIA GPU detected: enabling PRIME render offload for Gazebo")
    return [
        SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1'),
        SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'),
    ]


def find_px4():
    # Get PX4 directory path - try multiple methods
    px4_dir = None

    # Method 1: Check environment variable PX4_DIR
    if 'PX4_DIR' in os.environ:
        px4_dir = os.environ['PX4_DIR']

    # Method 2: Try to find px4 executable in PATH and get its directory
    if px4_dir is None:
        px4_bin = shutil.which('px4')
        if px4_bin:
            # px4 binary is typically at PX4-Autopilot/build/px4_sitl_default/bin/px4
            # So we go up 4 levels to get the root directory
            px4_dir = os.path.abspath(os.path.join(os.path.dirname(px4_bin), '../../../..'))

    # Method 3: Check common PX4 installation locations
    if px4_dir is None:
        common_paths = [
            os.path.expanduser('~/PX4-Autopilot'),
            os.path.expanduser('~/px4'),
            '/opt/PX4-Autopilot',
            '/usr/local/PX4-Autopilot',
            '/PX4-Autopilot',
        ]
        for path in common_paths:
            if os.path.exists(os.path.join(path, 'build/px4_sitl_default/bin/px4')):
                px4_dir = path
                break

    # Method 4: Fallback - use default
    if px4_dir is None:
        px4_dir = os.path.expanduser('~/PX4-Autopilot')
        print(f"Warning: PX4 directory not found automatically. Using default: {px4_dir}")
        print("Set PX4_DIR environment variable or ensure 'px4' is in PATH")

    return px4_dir


def px4_gz_env_actions():
    """Acciones de launch con el entorno de Gazebo: GPU (gz_gpu_env_actions) y recursos de PX4.

    Hay que ponerlas ANTES de la accion que arranca gz. Los recursos de PX4 se derivan de
    find_px4():

    - GZ_SIM_RESOURCE_PATH     += <PX4>/Tools/simulation/gz/models
    - GZ_SIM_SYSTEM_PLUGIN_PATH += <PX4>/build/px4_sitl_default/src/modules/simulation/gz_plugins
    - GZ_SIM_SERVER_CONFIG_PATH = <PX4>/src/modules/simulation/gz_bridge/server.config

    Las dos primeras se AGREGAN (append): ros_gz ya pone ahi /opt/ros/<distro>/share. La tercera
    solo se define si el usuario no la habia fijado. Lo que no exista se avisa y se omite.
    """
    px4_dir = find_px4()
    models_dir = os.path.join(px4_dir, 'Tools', 'simulation', 'gz', 'models')
    plugins_dir = os.path.join(
        px4_dir, 'build', PX4_BUILD_TARGET, 'src', 'modules', 'simulation', 'gz_plugins')
    server_config = os.path.join(px4_dir, 'src', 'modules', 'simulation', 'gz_bridge', 'server.config')

    actions = gz_gpu_env_actions()

    def append_once(var, path):
        if path not in os.environ.get(var, '').split(os.pathsep):
            actions.append(AppendEnvironmentVariable(var, path))

    if os.path.isdir(models_dir):
        append_once('GZ_SIM_RESOURCE_PATH', models_dir)
    else:
        print(f"[WARNING] PX4 Gazebo models not found at {models_dir}. Set PX4_DIR (PX4 root).")

    if os.path.isdir(plugins_dir):
        append_once('GZ_SIM_SYSTEM_PLUGIN_PATH', plugins_dir)
    else:
        print(f"[WARNING] PX4 Gazebo plugins not found at {plugins_dir}. "
              f"Build PX4 first (make {PX4_BUILD_TARGET}).")

    if 'GZ_SIM_SERVER_CONFIG_PATH' not in os.environ:
        if os.path.isfile(server_config):
            actions.append(SetEnvironmentVariable('GZ_SIM_SERVER_CONFIG_PATH', server_config))
        else:
            print(f"[WARNING] PX4 server.config not found at {server_config}.")

    print(f"[DEBUG] PX4 Gazebo environment from: {px4_dir}")
    return actions
