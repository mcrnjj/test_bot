# Modelo realista `robot_v2` (export Fusion360)

Modelo más realista del robot para simular en Gazebo (Foxy / Ubuntu 20.04),
integrado al paquete `test_bot`.

## Archivos
- `body.xacro` — links/joints del export Fusion (cuerpo, ruedas, ToF, cámara, caster).
  Root renombrado a `body_link`; ruedas = `left_wheel_joint` / `right_wheel_joint`;
  meshes → `package://test_bot/description/meshes/`.
- `robot_v2.gazebo` — plugins Gazebo: `diff_drive` (/cmd_vel, /odom), `camera`
  (/camera/image_raw + camera_info), `imu` (/imu/data) + fricción.
- `../robot_v2.urdf.xacro` — **wrapper** que se carga. Aporta `base_footprint`,
  `base_link`, reorientación (frente → +X), `camera_link_optical`, `imu_link`.

## Cómo probar
```bash
colcon build --packages-select test_bot && source install/setup.bash

# Sim con el modelo nuevo (es el default ahora):
ros2 launch test_bot launch_sim.launch.py

# Volver al modelo simple sin editar:
ROBOT_MODEL=robot.urdf.xacro ros2 launch test_bot launch_sim.launch.py

# Conducir:
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Stack completo (mapa + aruco + ekf + nav2) en otra terminal:
ros2 launch test_bot map.launch.py
```

## Geometría derivada del export — VERIFICAR/AFINAR en sim
Estos valores se calcularon desde los frames del export y pueden necesitar ajuste:

| Qué | Dónde | Valor actual |
|---|---|---|
| Diámetro de rueda | `robot_v2.gazebo` | 0.065 m |
| Separación de ruedas | `robot_v2.gazebo` | 0.169 m |
| Altura del eje (base_link) | `robot_v2.urdf.xacro` base_footprint_joint | 0.0325 m |
| Offset/rotación cuerpo | `robot_v2.urdf.xacro` body_joint | xyz `0.047 0.08 0.084`, yaw `-90°` |

### Si algo sale mal
- **Gira al revés / sobre sí mismo**: intercambia `left_joint`/`right_joint` en `robot_v2.gazebo`.
- **Avanza de lado / cámara mira mal**: ajusta el `rpy` de `body_joint` (orientación del frente).
- **Flota o se hunde**: ajusta la z de `base_footprint_joint` (altura del eje).
- **Robot no aparece en RViz con solo `launch_sim`** (sin EKF): pon
  `publish_odom_tf` a `true` en `robot_v2.gazebo` (con `map.launch`/EKF déjalo en `false`).
- **Ruedas patinan**: sube `mu1/mu2` de las ruedas en `robot_v2.gazebo`.

> El modelo es un export Fusion360; los frames de los links no están centrados,
> así que la separación y el centro de rotación son aproximados. Para nav2 preciso
> conviene afinar `wheel_separation` midiendo la odometría real en sim.
