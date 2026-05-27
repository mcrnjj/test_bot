#!/usr/bin/env python3
"""
generate_aruco_models.py
------------------------
Genera marcadores ArUco listos para usar en Gazebo Classic + ROS2:
  - PNG por cada marcador (carpeta out/markers_png/)
  - Modelo Gazebo por cada marcador (carpeta out/models/aruco_marker_<id>/)
  - Base de datos YAML con la pose de cada marcador en el frame 'map'
    (out/config/markers_db.yaml)
  - Archivo .world derivado del original con los <include> de cada marcador
    (out/worlds/test_world_with_markers.world)

Uso:
    python3 generate_aruco_models.py \
        --base-world /ruta/a/test_world.world \
        --out /ruta/de/salida \
        [--dict DICT_5X5_250] [--size 0.15]

Las poses de los marcadores están definidas en MARKER_POSES más abajo.
Edita esa lista para adaptarla a tu mapa.
"""

import argparse
import os
import shutil
import sys
import xml.etree.ElementTree as ET

try:
    import cv2
except ImportError:
    sys.exit("ERROR: necesitas opencv-contrib-python ('pip install opencv-contrib-python')")

import yaml


# ----------------------------------------------------------------------------
# CONFIGURACIÓN: poses de los marcadores en el frame 'map' (mundo)
# Cada entrada: (id, x, y, yaw_rad)
# El marcador queda plano en el piso (roll=pitch=0) a z = 0.001 m.
# ----------------------------------------------------------------------------
# ----------------------------------------------------------------------------
# MARCADORES EN PAREDES (no piso).
# Cada entrada: (id, x, y, z, facing) donde facing es la direccion en la que
# apunta la NORMAL del marcador (hacia donde mira la cara del marcador):
#   'N' = +Y (norte)    'S' = -Y (sur)
#   'E' = +X (este)     'W' = -X (oeste)
# El script calcula RPY automaticamente para hacer el marcador vertical.
# Las posiciones (x,y) deben estar JUSTO sobre la cara interior de la pared,
# con un pequeno offset (~5mm) para no enterrarse dentro de la pared.
# ----------------------------------------------------------------------------
import math as _math

WALL_MARKERS = [
    # id 0: esq. sup. izq., pared OESTE, mira al ESTE
    (0, -0.37,  1.35, 0.15, 'E'),
    # id 1: esq. inf. der., pared ESTE, mira al OESTE
    (1,  0.37, -1.35, 0.15, 'W'),
    # id 2: pared OESTE, 2.00 m bajo el borde superior, mira al ESTE
    (2, -0.37, -0.50, 0.15, 'E'),
]

# RPY para que el marcador (originalmente plano X-Y con normal +Z) quede
# vertical con su normal apuntando en cada direccion cardinal.
# Correccion empirica de la orientacion de la textura. Si despues de
# regenerar todo, los robots aparecen rotados 90 grados (tumbados sobre un
# costado) al ver un marcador, prueba con +/- pi/2 o pi. Es una rotacion
# alrededor del eje +Z local del marcador (la normal de la cara).
TEXTURE_YAW_OFFSET = 0.0# rad. Valores tipicos: 0, pi/2, pi, -pi/2

_FACING_TO_RPY = {
    'N': ( _math.pi / 2, 0.0,  _math.pi),
    'S': ( _math.pi / 2, 0.0,  0.0),
    'E': ( _math.pi / 2, 0.0,  _math.pi / 2),
    'W': ( _math.pi / 2, 0.0, -_math.pi / 2),
}


def _build_wall_markers():
    """Convierte WALL_MARKERS a tuplas internas (id, x, y, z, roll, pitch, yaw).
    Aplica TEXTURE_YAW_OFFSET como rotacion alrededor del eje local +Z del
    marcador (la cara visible) para compensar el UV mapping de Gazebo si hace
    falta."""
    from scipy.spatial.transform import Rotation as _Rot
    out = []
    for (mid, x, y, z, facing) in WALL_MARKERS:
        if facing not in _FACING_TO_RPY:
            raise ValueError(f"facing invalido: {facing}")
        R_facing = _Rot.from_euler('xyz', _FACING_TO_RPY[facing])
        R_tex = _Rot.from_euler('z', TEXTURE_YAW_OFFSET)   # rotacion en marker local Z
        R_total = R_facing * R_tex                          # primero textura, despues facing
        roll, pitch, yaw = R_total.as_euler('xyz')
        out.append((mid, float(x), float(y), float(z),
                    float(roll), float(pitch), float(yaw)))
    return out


MARKER_POSES = _build_wall_markers()


# ----------------------------------------------------------------------------
MODEL_CONFIG_TEMPLATE = """<?xml version="1.0"?>
<model>
  <name>aruco_marker_{id}</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <description>ArUco marker id={id}, dict={dict_name}, size={size} m</description>
</model>
"""

MODEL_SDF_TEMPLATE = """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="aruco_marker_{id}">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <cast_shadows>false</cast_shadows>
        <geometry>
          <box><size>{size} {size} 0.001</size></box>
        </geometry>
        <material>
          <script>
            <uri>model://aruco_marker_{id}/materials/scripts</uri>
            <uri>model://aruco_marker_{id}/materials/textures</uri>
            <name>ArucoMarker/Id{id}</name>
          </script>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <box><size>{size} {size} 0.001</size></box>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
"""

OGRE_MATERIAL_TEMPLATE = """material ArucoMarker/Id{id}
{{
  technique
  {{
    pass
    {{
      ambient  1 1 1 1
      diffuse  1 1 1 1
      specular 0 0 0 1 0
      texture_unit
      {{
        texture marker_{id}.png
        filtering none
      }}
    }}
  }}
}}
"""


def get_aruco_dict(name: str):
    if not hasattr(cv2.aruco, name):
        raise ValueError(f"Diccionario ArUco desconocido: {name}")
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))


def generate_marker_png(dictionary, marker_id: int, side_pixels: int = 800):
    if hasattr(cv2.aruco, "generateImageMarker"):
        return cv2.aruco.generateImageMarker(dictionary, marker_id, side_pixels)
    # OpenCV antiguo
    img = cv2.aruco.drawMarker(dictionary, marker_id, side_pixels)
    return img


def write_model(out_models_dir: str, marker_id: int, dict_name: str, size: float, png_img):
    model_dir = os.path.join(out_models_dir, f"aruco_marker_{marker_id}")
    scripts_dir = os.path.join(model_dir, "materials", "scripts")
    textures_dir = os.path.join(model_dir, "materials", "textures")
    os.makedirs(scripts_dir, exist_ok=True)
    os.makedirs(textures_dir, exist_ok=True)

    with open(os.path.join(model_dir, "model.config"), "w") as f:
        f.write(MODEL_CONFIG_TEMPLATE.format(id=marker_id, dict_name=dict_name, size=size, tn="na"+"me"))
    with open(os.path.join(model_dir, "model.sdf"), "w") as f:
        f.write(MODEL_SDF_TEMPLATE.format(id=marker_id, size=size, tn="na"+"me"))
    with open(os.path.join(scripts_dir, f"aruco_marker_{marker_id}.material"), "w") as f:
        f.write(OGRE_MATERIAL_TEMPLATE.format(id=marker_id))
    cv2.imwrite(os.path.join(textures_dir, f"marker_{marker_id}.png"), png_img)


def write_markers_db(out_config_dir: str, dict_name: str, size: float):
    os.makedirs(out_config_dir, exist_ok=True)
    db = {
        "aruco_dict": dict_name,
        "marker_size": size,
        "markers": [
            {
                "id": int(mid),
                "frame_id": "map",
                "x": float(x), "y": float(y), "z": float(z),
                "roll": float(roll), "pitch": float(pitch), "yaw": float(yaw),
            }
            for (mid, x, y, z, roll, pitch, yaw) in MARKER_POSES
        ],
    }
    path = os.path.join(out_config_dir, "markers_db.yaml")
    with open(path, "w") as f:
        yaml.safe_dump(db, f, sort_keys=False)
    return path


def build_world_with_markers(base_world_path: str, out_world_path: str):
    """Lee el .world original e inyecta un <include> por cada marcador antes
    de cerrar el tag </world>."""
    with open(base_world_path, "r") as f:
        content = f.read()

    includes = []
    for (mid, x, y, z, roll, pitch, yaw) in MARKER_POSES:
        includes.append(
            f"    <include>\n"
            f"      <name>aruco_marker_{mid}</name>\n"
            f"      <uri>model://aruco_marker_{mid}</uri>\n"
            f"      <pose>{x} {y} {z} {roll} {pitch} {yaw}</pose>\n"
            f"    </include>"
        )
    block = "\n    <!-- ArUco fiducial markers (auto-generated) -->\n" + "\n".join(includes) + "\n"

    if "</world>" not in content:
        raise RuntimeError("El .world base no contiene </world>")
    new_content = content.replace("</world>", block + "  </world>")

    os.makedirs(os.path.dirname(out_world_path), exist_ok=True)
    with open(out_world_path, "w") as f:
        f.write(new_content)


def main():
    parser = argparse.ArgumentParser(description="Genera modelos ArUco para Gazebo + ROS2")
    parser.add_argument("--base-world", required=True, help="Ruta al .world original (test_world.world)")
    parser.add_argument("--out", required=True, help="Directorio de salida")
    parser.add_argument("--dict", default="DICT_5X5_250",
                        help="Diccionario ArUco (DICT_4X4_50, DICT_5X5_250, etc.)")
    parser.add_argument("--size", type=float, default=0.10,
                        help="Tamaño del marcador en metros (lado del cuadrado negro)")
    parser.add_argument("--png-resolution", type=int, default=800,
                        help="Resolución del PNG (lado en píxeles)")
    args = parser.parse_args()

    out = os.path.abspath(args.out)
    out_png = os.path.join(out, "markers_png")
    out_models = os.path.join(out, "models")
    out_config = os.path.join(out, "config")
    out_worlds = os.path.join(out, "worlds")
    for d in (out_png, out_models, out_config, out_worlds):
        os.makedirs(d, exist_ok=True)

    dictionary = get_aruco_dict(args.dict)

    print(f"[+] Generando {len(MARKER_POSES)} marcadores ({args.dict}, {args.size} m)")
    for (mid, x, y, z, roll, pitch, yaw) in MARKER_POSES:
        img = generate_marker_png(dictionary, mid, args.png_resolution)
        cv2.imwrite(os.path.join(out_png, f"marker_{mid}.png"), img)
        write_model(out_models, mid, args.dict, args.size, img)
        print(f"    - id={mid:>2}  pose=({x:+.2f}, {y:+.2f}, {z:.2f}) "
              f"rpy=({roll:+.2f}, {pitch:+.2f}, {yaw:+.2f})")

    db_path = write_markers_db(out_config, args.dict, args.size)
    print(f"[+] Base de datos: {db_path}")

    out_world = os.path.join(out_worlds, "test_world_with_markers.world")
    build_world_with_markers(args.base_world, out_world)
    print(f"[+] World con marcadores: {out_world}")

    print("\n[OK] Listo. Pasos a seguir:")
    print(f"  1) export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:{out_models}")
    print(f"  2) Lanza Gazebo con: {out_world}")


if __name__ == "__main__":
    main()