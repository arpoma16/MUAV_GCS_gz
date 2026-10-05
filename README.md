# MUAV_GCS_gz

Mundos, modelos y launch de Gazebo (Harmonic, gz-sim 8) para la simulación multi-UAV con PX4 SITL.

```bash
ros2 launch muav_gcs_gz scene.launch.py          # escena completa (config/scene.yaml)
ros2 launch muav_gcs_gz simulation.launch.py     # solo gz + bridges
ros2 launch muav_gcs_gz twoDrones.launch.py      # dos drones fijos
```

> Esta guía describe la ejecución **local** (Ubuntu 24.04 + ROS 2 Jazzy). El entorno Docker
> (Humble) sigue otro camino: ahí las variables de gz se exportaban en `~/.bashrc` apuntando a
> `/root/PX4-gazebo-models`. Esas rutas **no aplican** en local. Datos verificados el 2026-10-05
> con PX4 `v1.18.0-beta1-956-g9fda8a51a4`.

## Qué configura el launch por sí solo

`launch/px4_gz_env.py` arma el entorno del servidor de gz **antes** de arrancarlo. Lo usan
`scene`, `simulation` y `twoDrones` (`px4_gz_env_actions()`), y `spawn_px4` usa su `find_px4()`.
No hace falta exportar nada a mano.

| Variable | Valor | Por qué |
|---|---|---|
| `GZ_SIM_RESOURCE_PATH` | **se agrega** `<PX4>/Tools/simulation/gz/models` | Los modelos de este paquete hacen `model://x500_gimbal`, que vive en PX4 |
| `GZ_SIM_SYSTEM_PLUGIN_PATH` | **se agrega** `<PX4>/build/px4_sitl_default/src/modules/simulation/gz_plugins` | Plugins compilados por PX4 (`MotorFailurePlugin`, ...) |
| `GZ_SIM_SERVER_CONFIG_PATH` | `<PX4>/src/modules/simulation/gz_bridge/server.config` (solo si no estaba fijada) | Los mundos no declaran plugins; sin esto no hay sensores ni viento |
| `__NV_PRIME_RENDER_OFFLOAD`, `__GLX_VENDOR_LIBRARY_NAME` | `1`, `nvidia` (solo si se detecta NVIDIA) | Laptops híbridas: la GUI usaba la iGPU |
| `PX4_GZ_NO_FOLLOW` | `1` por defecto (a cada PX4) | Evita que PX4 ate la cámara de la GUI al dron |
| `GZ_PARTITION` | `docker_sim_harmonic` si no estaba fijada (lo hacen `scene`, `twoDrones` y `spawn_px4`, **no** `simulation`) | Aísla el tráfico de gz-transport |

**Se AGREGA, nunca se sobrescribe, `GZ_SIM_RESOURCE_PATH`**: el hook de `ros_gz_sim_demos`
(`share/ros_gz_sim_demos/environment/ros_gz_sim_demos.dsv`, dependencia de `ros-jazzy-ros-gz`)
ya pone ahí `/opt/ros/jazzy/share`. Pisarla (como hacía el Dockerfile) rompe los modelos de ROS.

**Orden importa:** `ros_gz_sim/launch/gz_sim.launch.py` arma el entorno del proceso gz a partir de
`os.environ` en el momento de incluirse (`GZ_SIM_RESOURCE_PATH` y `GZ_SIM_SYSTEM_PLUGIN_PATH`, a las
que además les suma `LD_LIBRARY_PATH` y los paths de los paquetes con export `gazebo_ros`). Por eso las
acciones del helper van **antes** del `IncludeLaunchDescription`. Verificado de punta a punta: lanzando
el `gz_sim.launch.py` real tras las acciones del helper, el proceso `gz sim` recibe las cinco variables
de la tabla (se lee en `/proc/<pid>/environ`).

Los launch instalados **no tienen su carpeta en `sys.path`**, por eso importan el helper con
`sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))`. Al ser un archivo nuevo, hace
falta `colcon build` (aun con `--symlink-install`) para que se instale.

## Cómo se encuentra PX4 (`find_px4()`)

1. Variable `PX4_DIR`.
2. `px4` en el `PATH` (se sube 4 niveles desde `build/px4_sitl_default/bin/px4`).
3. Rutas comunes (`~/PX4-Autopilot`, `~/px4`, `/opt/PX4-Autopilot`, ...) **solo si existe**
   `build/px4_sitl_default/bin/px4`.
4. Fallback `~/PX4-Autopilot` con un warning.

El target de build está fijo en `px4_sitl_default` (`PX4_BUILD_TARGET` en el helper).

## Modelos

- Los modelos `x500`, `x500_base`, `x500_gimbal`, `gimbal`, `mono_cam`... **no** están en este
  paquete: salen de `PX4-Autopilot/Tools/simulation/gz/models`.
- Los de este paquete (`models/`) los referencian por `model://`. Ejemplo:
  `x500_gimbal_collision` → incluye `x500_gimbal` → incluye `x500`, que declara
  `MotorFailurePlugin`. Si el servidor no ve los de PX4 falla con
  `Unable to find uri[model://x500_gimbal]` y luego `parent frame with name[base_link] ... not found`.
- **Hay dos copias de los modelos y NO son idénticas:**
  - `PX4-Autopilot/Tools/simulation/gz` (submódulo, fijado en `a15af96`) ← **la que usa el helper**,
    porque corresponde al binario de PX4 que corre.
  - `work/PX4-gazebo-models` (clon en `main`, `bb0b9cf`): es la que usa el Dockerfile.
  - `~/.simulation-gazebo/` también existe (lo descarga el script `simulation-gazebo` de PX4).
  Elegir una sola fuente; mezclar versiones puede desalinear topics y plugins del modelo.
- Los **mundos** de este paquete (`world/*.sdf`) no declaran `<plugin>`; `psdk_gz/worlds/hitl_default.sdf`
  sí (por eso el HITL no depende del `server.config`).

## `server.config`: hay dos y difieren

PX4 usa `src/modules/simulation/gz_bridge/server.config`, **no** `Tools/simulation/gz/server.config`.
El de PX4 agrega `gz-sim-wind-effects-system` (WindEffects) y configura el magnetómetro en tesla
(`use_units_gauss=false`, `use_earth_frame_ned=false`). Con el equivocado se pierde el viento
(`windgeneratorbasic`, turbinas) y el magnetómetro queda mal escalado.

## Plugins de gz de PX4

Compilados por el target `px4_gz_plugins`; en `build/px4_sitl_default/src/modules/simulation/gz_plugins/`:
`libMotorFailurePlugin.so`, `libOpticalFlowSystem.so`, `libGstCameraSystem.so`,
`libGenericMotorModelPlugin.so`, `libAirSpeedPlugin.so`, `libBuoyancySystemPlugin.so`,
`libMovingPlatformController.so`, `libSpacecraftThrusterModelPlugin.so`, `libTemplatePlugin.so`,
`libOpticalFlow.so`.

- Síntoma si falta el path: `Failed to load system plugin [MotorFailurePlugin] : Could not find shared library`.
- Si faltan los `.so`: compilar PX4 con Gazebo instalado (`make px4_sitl_default`).
- **Cuidado al buscarlos:** `build/` está en el `.gitignore` de PX4 y `rg --files` lo salta
  (ya nos hizo concluir mal que no existían). Usar `rg --no-ignore --files -g '*.so' build/`.

## De dónde sale `gz_env.sh`

Lo **genera CMake** al compilar PX4 (no está versionado): plantilla
`src/modules/simulation/gz_bridge/gz_env.sh.in`, `configure_file(...)` en `gz_bridge/CMakeLists.txt`
(línea ~143), resultado en `build/px4_sitl_default/gz_env.sh` con rutas absolutas. Solo se genera
si CMake encuentra Gazebo. Lo carga `px4-rc.gzsim` (`. ./gz_env.sh`, `../`, `../../`), o sea
**solo lo ve el proceso `px4`**, no un servidor de gz lanzado aparte. Por eso el helper replica
sus valores a mano: **si PX4 cambia la plantilla, hay que actualizar el helper**. Diferencia
conocida: `gz_env.sh` también agrega `Tools/simulation/gz/worlds` al resource path; el helper no
(los mundos son los de este paquete).

## Motor failure

`PX4-Autopilot/Tools/simulation/gz/tools/motor_failure.sh` publica en
`/model/<modelo>/motor_failure/motor_number` (`gz.msgs.Int32`; motor 1..N, `0` limpia el fallo).
El script **hardcodea** el nombre `x500_<instancia-1>`, así que no sirve con modelos propios.
El plugin arma el topic con el **nombre real del modelo en gz** (`MotorFailureSystem.cpp:73`):

```bash
export GZ_PARTITION=docker_sim_harmonic
gz topic -t /model/<nombre_en_gz>/motor_failure/motor_number -m gz.msgs.Int32 -p "data: 1"
```

## Cámara de la GUI atada al dron

Cuando **PX4 spawnea** el modelo (`PX4_SIM_MODEL`, sin `PX4_GZ_MODEL_NAME`), `px4-rc.gzsim`
manda `/gui/track` en modo FOLLOW salvo que exista `PX4_GZ_NO_FOLLOW`. Con varios drones se
disputan la cámara y no se puede navegar. `spawn_px4.launch.py` lo desactiva por defecto
(`PX4_GZ_NO_FOLLOW=1`). Para reactivarlo: `export PX4_GZ_NO_FOLLOW=` (**vacío**, no `0`: el script
usa `-z`).

## GPU en laptops híbridas

Con `prime-select` en `on-demand` el X corre en la iGPU (Intel) y la GUI de gz usa Mesa; el
servidor sí usa la NVIDIA (EGL). El helper detecta una NVIDIA utilizable (módulo del kernel
`/proc/driver/nvidia/version` **y** `libGLX_nvidia`) y activa PRIME offload. Respeta
`__GLX_VENDOR_LIBRARY_NAME`/`__NV_PRIME_RENDER_OFFLOAD` si ya están fijadas; `MUAV_NVIDIA_OFFLOAD=0`
lo desactiva. Verificar: con la escena abierta, `nvidia-smi pmon -c 1` debe mostrar `gz sim gui`
con tipo `G` (la GUI mapea `libGLX_nvidia` y no `libGLX_mesa`).

## Dos instalaciones de Gazebo

Conviven `/opt/ros/jazzy/opt/gz_*_vendor` (de `ros-jazzy-ros-gz`) y `gz-harmonic` (OSRF,
`/usr`), ambas gz-sim 8.15.0. `which -a gz` devuelve primero el vendorizado de ROS, y los plugins de
PX4 enlazan contra `libgz-sim8` del vendor (`ldd`). Mientras las versiones coincidan no hay
problema; si divergen, riesgo de ABI al cargar plugins.

## PX4 ↔ ROS 2 (`Tools/ros2` de PX4)

`PX4-Autopilot/Tools/ros2/ros2.repos` fija `px4_msgs` (`8fb8277`) y `px4-ros2-interface-lib`
(`22943b5`) para este checkout; `prepare_workspace.py` arma un workspace con esos pines y regenera
los mensajes desde el PX4. Estado verificado: los 271 `.msg` de `px4_msgs` coinciden con
`PX4/msg` + `msg/versioned` (hay que **aplanar** `versioned/` para comparar); el pin de
`px4_msgs` es ancestro de nuestro `main`, y `px4-ros2-interface-lib` quedó 4 commits por delante
del suyo. Ojo: el `CLAUDE.md`/`.repos` dicen `px4_msgs` en `release/1.15` pero el clon real está
en `main`.

## Cómo probar sin lanzar la escena

Servidor aislado (no pisa tu simulación) para comprobar que un modelo se resuelve:

```bash
export GZ_PARTITION=prueba_$$
# (exportar las variables de la tabla de arriba)
gz sim -s -r -v 3 empty.sdf &
gz service -s /world/empty/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean \
  --timeout 10000 --req 'sdf_filename: "'$PWD'/models/x500_gimbal_collision/model.sdf" name: "t1"'
```

- `gz sdf -p <modelo>` **no sirve** para esto: no usa `GZ_SIM_RESOURCE_PATH` y falla igual.
- Al limpiar procesos, `pkill -f`/`pgrep -f` también matchean tu propio shell si el patrón
  aparece en su línea de comando; identificar por nombre exacto (`ps -eo comm`, `ruby` para gz).

## Pendientes conocidos

- `MUAV_psdk_gz/launch/hitl_sim.launch.py` tiene el mismo problema de modelos
  (`model://gimbal`, `x500_base` de PX4) y no usa este helper.
- El helper asume el build en `build/px4_sitl_default`.
