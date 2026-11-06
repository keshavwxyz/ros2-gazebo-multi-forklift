#!/usr/bin/env python3
"""
Make the Gazebo models of the QR-labelled loads (pallet + box) from the mission config.

For each box in warehouse_mission.yaml (and each test decoy), this script writes
models/<name>/{model.config, model.sdf, materials/textures/<name>.png}. The QR label is
a real QR code (OpenCV encoder) of the payload of the box. Its size is the configured
symbol size, with a quiet zone of 4 modules, on the front (+x) face of the box. The
perception node must read it from the camera image. Nothing else identifies the box.

The output is deterministic and is in the repository. test/test_load_models.py fails if
the output is not current. After you change the mission, run the script again:

  python3 src/forklift_simulation/scripts/generate_load_models.py
"""

import argparse
import pathlib
import xml.etree.ElementTree as ET

import cv2
from forklift_perception.qr_codec import label_edge, render_qr_label
from forklift_task_manager.mission_config import load_mission
import yaml

PKG = pathlib.Path(__file__).resolve().parents[1]
DEFAULT_MISSION = PKG.parent / 'forklift_task_manager' / 'config' / 'warehouse_mission.yaml'
DEFAULT_DECOYS = PKG / 'config' / 'test_decoys.yaml'
PX_PER_MODULE = 16
FRICTION = 0.8          # wood/cardboard on steel forks and concrete


def box_inertia(m, x, y, z):
    return m * (y * y + z * z) / 12, m * (x * x + z * z) / 12, m * (x * x + y * y) / 12


def model_name(load_id):
    return f'qr_load_{load_id.lower()}'


def _collision_and_visual(link, name, pose, size, rgba):
    surface = (f'<surface><friction><ode><mu>{FRICTION}</mu><mu2>{FRICTION}</mu2></ode>'
               f'</friction></surface>')
    geo = f'<geometry><box><size>{size[0]:.4f} {size[1]:.4f} {size[2]:.4f}</size></box></geometry>'
    p = ' '.join(f'{v:.4f}' for v in pose)
    link.append(ET.fromstring(f'<collision name="{name}"><pose>{p} 0 0 0</pose>{geo}{surface}'
                              f'</collision>'))
    link.append(ET.fromstring(f'<visual name="{name}_v"><pose>{p} 0 0 0</pose>{geo}<material>'
                              f'<ambient>{rgba}</ambient><diffuse>{rgba}</diffuse></material>'
                              f'</visual>'))


def build_model_sdf(name, payload, load, qr_edge, n_modules):
    pallet_h, deck = load.pallet_height, load.deck_thickness
    model = ET.Element('model', name=name)
    ET.SubElement(model, 'static').text = 'false'
    link = ET.SubElement(model, 'link', name='load')

    # Inertia: a solid block around the pallet + box, with the
    # COM at the geometric center (approximation).
    total_h = pallet_h + load.box_height
    ixx, iyy, izz = box_inertia(load.mass, load.pallet_depth, load.pallet_width, total_h)
    link.append(ET.fromstring(
        f'<inertial><pose>0 0 {total_h / 2:.4f} 0 0 0</pose><mass>{load.mass}</mass>'
        f'<inertia><ixx>{ixx:.5f}</ixx><iyy>{iyy:.5f}</iyy><izz>{izz:.5f}</izz>'
        f'<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>'))

    wood, cardboard = '0.62 0.45 0.25 1', '0.72 0.56 0.36 1'
    _collision_and_visual(link, 'deck', (0, 0, pallet_h - deck / 2),
                          (load.pallet_depth, load.pallet_width, deck), wood)
    for i, y in enumerate(load.stringer_y):
        _collision_and_visual(link, f'stringer_{i}', (0, y, (pallet_h - deck) / 2),
                              (load.pallet_depth, load.stringer_width, pallet_h - deck), wood)
    _collision_and_visual(link, 'cargo', (0, 0, pallet_h + load.box_height / 2),
                          (load.box_depth, load.box_width, load.box_height), cardboard)

    # QR label: a plane 1 mm in front of the +x face of the box, at the center of the face.
    qx, _, qz = load.qr_center
    size = label_edge(qr_edge, n_modules)
    link.append(ET.fromstring(
        f'<visual name="qr_label"><pose>{qx + 0.001:.4f} 0 {qz:.4f} 0 0 0</pose>'
        f'<geometry><plane><normal>1 0 0</normal><size>{size:.5f} {size:.5f}</size></plane>'
        f'</geometry><material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>'
        f'<specular>0 0 0 1</specular><pbr><metal>'
        f'<albedo_map>model://{name}/materials/textures/{name}.png</albedo_map>'
        f'<metalness>0.0</metalness><roughness>1.0</roughness></metal></pbr></material>'
        f'</visual>'))

    # Ground truth for evaluation only: the world pose of the load on /model/<name>/pose. The
    # bridge sends it to /ground_truth/loads/<name> for the
    # tests (the autonomy nodes never use it).
    model.append(ET.fromstring(
        '<plugin filename="gz-sim-pose-publisher-system" '
        'name="gz::sim::systems::PosePublisher"><publish_link_pose>false</publish_link_pose>'
        '<publish_model_pose>true</publish_model_pose><publish_nested_model_pose>true'
        '</publish_nested_model_pose><use_pose_vector_msg>false</use_pose_vector_msg>'
        '<update_frequency>30</update_frequency></plugin>'))

    sdf = ET.Element('sdf', version='1.10')
    sdf.append(model)
    ET.indent(sdf)
    header = (f'<?xml version="1.0"?>\n<!-- scripts/generate_load_models.py made this file '
              f'from the mission config. Do not edit it. QR payload: {payload} -->\n')
    return header + ET.tostring(sdf, encoding='unicode') + '\n'


def build_model_config(name, payload):
    return (f'<?xml version="1.0"?>\n<model>\n  <name>{name}</name>\n  <version>1.0</version>\n'
            f'  <sdf version="1.10">model.sdf</sdf>\n'
            f'  <description>QR-labelled pallet load. payload={payload}</description>\n'
            f'</model>\n')


def generate(mission_path=DEFAULT_MISSION, decoys_path=DEFAULT_DECOYS, out_dir=PKG / 'models'):
    """Write all load models. Return {model name: payload}."""
    mission = load_mission(mission_path)
    loads = [(model_name(b.box_id), b.payload) for b in mission.boxes]
    decoys = yaml.safe_load(pathlib.Path(decoys_path).read_text()).get('decoys', [])
    loads += [(model_name(d['name']), d['payload']) for d in decoys]
    written = {}
    for name, payload in loads:
        label, n = render_qr_label(payload, px_per_module=PX_PER_MODULE)
        tex_dir = pathlib.Path(out_dir) / name / 'materials' / 'textures'
        tex_dir.mkdir(parents=True, exist_ok=True)
        # Each model has a unique file name. The Gazebo renderer keeps textures in a cache by file
        # name. With the same names in different model folders, all models showed the first label.
        cv2.imwrite(str(tex_dir / f'{name}.png'), label)
        (tex_dir.parents[1] / 'model.sdf').write_text(
            build_model_sdf(name, payload, mission.load, mission.qr_edge_length, n))
        (tex_dir.parents[1] / 'model.config').write_text(build_model_config(name, payload))
        written[name] = payload
    return written


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[1])
    parser.add_argument('--mission', default=str(DEFAULT_MISSION))
    parser.add_argument('--decoys', default=str(DEFAULT_DECOYS))
    parser.add_argument('--out', default=str(PKG / 'models'))
    args = parser.parse_args()
    for name, payload in generate(args.mission, args.decoys, args.out).items():
        print(f'{name}: {payload}')


if __name__ == '__main__':
    main()
