from scipy.spatial.transform import Rotation as R
from numpy import pi

# orientation = R.from_quat([ 0.5, 0.5, 0.5, 0.5 ])

# rot_local = R.from_quat([ 0, -0.3826834, 0, 0.9238795 ])

# rot_global = R.from_quat([ 0, -0.3826834, 0, 0.9238795 ])

orientation = R.from_rotvec([0, 0, pi / 2])

rot_local = R.from_rotvec([0, -pi / 2, 0])

rot_global = R.from_rotvec([pi / 2, 0, 0])

rot_global_calc = orientation * rot_local * orientation.inv()

final_position = orientation * rot_local

print(final_position.as_euler('ZYX', degrees=True))