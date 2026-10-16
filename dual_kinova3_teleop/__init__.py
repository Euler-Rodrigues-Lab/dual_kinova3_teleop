"""Dual Kinova Gen3 simulation and shared-device teleoperation."""
from pathlib import Path
ASSETS = Path(__file__).parent / 'assets'
SAMPLE_MOTIONS = {name: ASSETS/'sample_motion'/f'{name}.npz' for name in ('ipman_roll','picking_up_mustard')}
def model_path(): return ASSETS/'source/robot/robot.xml'
def scene_path(variant='robotiq'):
    if variant not in ('robotiq','psyonic'): raise ValueError('variant must be robotiq or psyonic')
    return ASSETS/f'scene_{variant}.xml'
