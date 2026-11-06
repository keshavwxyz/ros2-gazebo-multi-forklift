# Gazebo load models

This directory has the Gazebo models of the QR-labelled loads. The world and the launch files
find them through `model://` URIs.

| Model | Purpose |
|---|---|
| `qr_load_box_a`, `qr_load_box_b`, `qr_load_box_c` | Mission loads: one pallet and one box with a QR label |
| `qr_load_decoy_malformed` | Test decoy: the QR payload is not valid |
| `qr_load_decoy_unknown_box` | Test decoy: the QR payload has a box ID that is not in the mission |

Each model has `model.config`, `model.sdf` and a QR texture in `materials/textures/`. The QR
label is on the front (+x) face of the box. Only the camera image identifies the box.

Do not edit these files manually. The script `scripts/generate_load_models.py` makes them from
`forklift_task_manager/config/warehouse_mission.yaml` and `config/test_decoys.yaml`. After you
change the mission, run the script again:

```bash
python3 src/forklift_simulation/scripts/generate_load_models.py
```

The test `test/test_load_models.py` fails if the models are not current.
