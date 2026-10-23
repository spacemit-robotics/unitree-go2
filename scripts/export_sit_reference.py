# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0
"""Export the sit training NPZ to the shared GO2_MOTION_V1 format."""
import argparse
from pathlib import Path

import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument('motion', type=Path)
parser.add_argument('output', type=Path)
args = parser.parse_args()
with np.load(args.motion) as motion:
    assert float(motion['fps']) == 50.0
    joints = motion['joint_pos'][:471]
    velocities = motion['joint_vel'][:471]
    quaternions = motion['body_quat_w'][:471, 0]
    assert joints.shape == velocities.shape == (471, 12)
    assert quaternions.shape == (471, 4)
    rows = np.concatenate((joints, velocities, quaternions), axis=1)
    assert np.isfinite(rows).all()
with args.output.open('w') as output:
    output.write('GO2_MOTION_V1 471 50\n')
    np.savetxt(output, rows, fmt='%.9e')
