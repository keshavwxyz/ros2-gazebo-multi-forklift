"""
Static guard: the autonomy code never uses simulator shortcuts.

Only the forks can move the loads (no teleport through entity-pose services, no attachment
joints in the simulator). No autonomy node can read the ground truth or depend on Gazebo
entity names. Only forklift_simulation (the adapter) and the tests can do these things.
"""

import pathlib
import re

import pytest

SRC = pathlib.Path(__file__).resolve().parents[2]
AUTONOMY_PACKAGES = ['forklift_control', 'forklift_perception', 'forklift_navigation',
                     'forklift_localization', 'forklift_task_manager', 'forklift_description',
                     'forklift_bringup', 'forklift_interfaces']
FORBIDDEN = {
    'entity pose service (teleport)': re.compile(r'set_entity_pose|SetEntityPose|set_pose'),
    'simulator attachment joint': re.compile(r'DetachableJoint|detachable_joint'),
    'ground-truth topic': re.compile(r'ground_truth'),
    'Gazebo entity name of a load': re.compile(r'qr_load_|load_box_'),
}


def source_files(package):
    root = SRC / package
    for path in root.rglob('*'):
        if path.suffix in {'.py', '.cpp', '.hpp', '.yaml', '.xacro', '.urdf'} and \
                '/test/' not in str(path) and path.is_file():
            yield path


@pytest.mark.parametrize('package', AUTONOMY_PACKAGES)
def test_no_shortcuts_in_autonomy_code(package):
    if not (SRC / package).exists():
        pytest.skip(f'{package} not in this workspace')
    hits = []
    for path in source_files(package):
        text = path.read_text(errors='ignore')
        for what, pattern in FORBIDDEN.items():
            for m in pattern.finditer(text):
                line = text.count('\n', 0, m.start()) + 1
                hits.append(f'{path.relative_to(SRC)}:{line}: {what} ({m.group(0)})')
    assert hits == [], '\n'.join(hits)
