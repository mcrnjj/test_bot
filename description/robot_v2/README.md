# Modelo realista `robot_v2` (export Fusion360, versión ros2_control)

Modelo más realista del robot para simular en Gazebo (Foxy / Ubuntu 20.04),
integrado al paquete `test_bot`. Esta versión proviene de `claude_description`
y usa **ros2_control + diff_drive_controller** (en vez del plugin simple
`libgazebo_ros_diff_drive`).

## Archivos
- `body.xacro` — links/joints del export Fusion (cuerpo, ruedas, ToF, cámara,
  caster + bola). Root renombrado a `body_link`; ruedas = `left_wheel_joint` /
  `right_wheel_joint`; meshes → `package://test_bot/description/meshes/`.
  Conserva el bloque `<ros2_control>` (interfaces de hardware Gazebo).
- `robot_v2.gazebo` — plugins Gazebo: `gazebo_ros2_control` (controller_manager),
  `camera` (/camera/image_raw + camera_info), `imu` (/imu/data) + fricción.
- `../robot_v2.urdf.xacro` — **wrapper** que se carga. Aporta `base_footprint`,
  `base_link`, reorientación (frente → +X), `camera_link_optical`, `imu_link`.
- `../../config/controllers.yaml` — config de `diff_drive_controller` y
  `joint_state_broadcaster`.

## Dependencias extra en el PC de pruebas (Foxy)
ros2_control NO viene con la sim original. Instala:
```bash
sudo apt install ros-foxy-gazebo-ros2-control \
                 ros-foxy-ros2-control \
                 ros-foxy-ros2-controllers
```

## Cómo probar
```bash
colcon build --packages-select test_bot && source install/setup.bash

# Sim con el modelo nuevo (es el default ahora):
ros2 launch test_bot launch_sim.launch.py
# (arranca Gazebo, spawnea el robot y carga joint_state_broadcaster +
#  diff_drive_controller automáticamente)

# Volver al modelo simple sin editar:
ROBOT_MODEL=robot.urdf.xacro ros2 launch test_bot launch_sim.launch.py

# Conducir (OJO: el topic del controlador NO es /cmd_vel por defecto):
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel_unstamped
```

### Verifica que los controladores cargaron
```bash
ros2 control list_controllers
# joint_state_broadcaster  [active]
# diff_drive_controller    [active]
```

## Topics del diff_drive_controller
| | Topic |
|---|---|
| cmd_vel IN | `/diff_drive_controller/cmd_vel_unstamped` (Twist) |
| odom OUT | `/diff_drive_controller/odom` |
| TF odom→base_link | según `enable_odom_tf` en `controllers.yaml` |

Para el stack completo (`map.launch`: ekf + nav2): pon `enable_odom_tf: false`
en `controllers.yaml` y apunta el EKF a `/diff_drive_controller/odom` (o remapea
cmd_vel/odom). Para probar SOLO el modelo con `launch_sim`, déjalo en `true`.

## Geometría derivada del export — VERIFICAR/AFINAR en sim
| Qué | Dónde | Valor actual |
|---|---|---|
| Radio de rueda | `config/controllers.yaml` | 0.0325 m |
| Separación de ruedas | `config/controllers.yaml` | 0.169 m |
| Altura del eje (base_link) | `robot_v2.urdf.xacro` base_footprint_joint | 0.0325 m |
| Offset/rotación cuerpo | `robot_v2.urdf.xacro` body_joint | xyz `0.047 0.08 0.084`, yaw `-90°` |

### Si algo sale mal
- **Gira al revés / sobre sí mismo**: intercambia `left_wheel_names`/`right_wheel_names`
  en `config/controllers.yaml`.
- **Avanza de lado / cámara mira mal**: ajusta el `rpy` de `body_joint` en el wrapper.
- **Flota o se hunde**: ajusta la z de `base_footprint_joint`.
- **No carga el controlador**: revisa que instalaste `gazebo-ros2-control` y que
  `ros2 control list_controllers` responde (el controller_manager lo crea el plugin
  al spawnear; los spawner.py corren tras `spawn_entity`).
- **Robot sin TF odom→base_link con solo `launch_sim`**: `enable_odom_tf: true`.
- **Ruedas patinan**: sube `mu1/mu2` de las ruedas en `robot_v2.gazebo`.

> El modelo es un export Fusion360; los frames de los links no están centrados,
> así que la separación y el centro de rotación son aproximados. Para nav2 preciso
> conviene afinar `wheel_separation` midiendo la odometría real en sim.
