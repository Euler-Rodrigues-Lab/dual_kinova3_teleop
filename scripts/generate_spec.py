"""Geometry-only extraction from the unchanged original MJCF; no solver code."""
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import hashlib,io,zipfile
ROOT=Path(__file__).resolve().parents[1]/'dual_kinova3_teleop/assets'
xml_path_v1=str(ROOT/'source/robot/robot.xml')
DEFAULT_JOINT_ANGLES = {
    "right": np.array(
        [
            np.pi / 2,
            np.pi / 2,
            -np.pi / 2,
            np.pi / 2,
            0.0,
            np.pi / 6,
            np.pi / 2 - np.radians(12.0),
        ],
        dtype=float,
    ),
    "left": np.array(
        [
            np.pi / 2,
            -np.pi / 2,
            np.pi / 2,
            -np.pi / 2,
            0.0,
            -np.pi / 6,
            -np.pi / 2 + np.radians(12.0),
        ],
        dtype=float,
    ),
}

_ARM_XML_CONFIG = {
    "right": {
        "arm_base": "right_arm_fixed_base_link",
        "link_names": [
            "right_shoulder_link",
            "right_HalfArm1_Link",
            "right_HalfArm2_Link",
            "right_forearm_link",
            "right_SphericalWrist1_Link",
            "right_SphericalWrist2_Link",
            "right_Bracelet_Link",
        ],
        "joint_names": [
            "right_Actuator1",
            "right_Actuator2",
            "right_Actuator3",
            "right_Actuator4",
            "right_Actuator5",
            "right_Actuator6",
            "right_Actuator7",
        ],
        "hand_body": "right_hand",
        "eef_body": "right_end_effector",
    },
    "left": {
        "arm_base": "left_arm_fixed_base_link",
        "link_names": [
            "left_shoulder_link",
            "left_HalfArm1_Link",
            "left_HalfArm2_Link",
            "left_forearm_link",
            "left_SphericalWrist1_Link",
            "left_SphericalWrist2_Link",
            "left_Bracelet_Link",
        ],
        "joint_names": [
            "left_Actuator1",
            "left_Actuator2",
            "left_Actuator3",
            "left_Actuator4",
            "left_Actuator5",
            "left_Actuator6",
            "left_Actuator7",
        ],
        "hand_body": "left_hand",
        "eef_body": "left_end_effector",
    },
}

def get_frame_transforms_from_xml(
    xml_path=xml_path_v1,
    arm_side="right",
    verbose=False,
    default_joint_angles=None,
):
    """
    Extract per-arm transforms from Dual Kinova3 MuJoCo XML.

    Returns a transform dictionary compatible with IK_2R_2R_3R_SEW_Lowpass_Singularity.
    """
    if arm_side not in _ARM_XML_CONFIG:
        raise ValueError(f"arm_side must be one of {list(_ARM_XML_CONFIG.keys())}")

    tree = ET.parse(xml_path)
    root = tree.getroot()
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError("Could not find worldbody in XML")

    base_body = _find_body_in_xml(worldbody, "base")
    if base_body is None:
        raise ValueError("Could not find body 'base' in XML")

    upper_body = _find_body_in_xml(base_body, "vention_upper_body")
    if upper_body is None:
        raise ValueError("Could not find body 'vention_upper_body' in XML")

    cfg = _ARM_XML_CONFIG[arm_side]
    arm_base = _find_body_in_xml(upper_body, cfg["arm_base"])
    if arm_base is None:
        raise ValueError(f"Could not find body '{cfg['arm_base']}' in XML")

    R_base_0, p_base_0 = _parse_transform_from_body(arm_base)

    R_transforms = []
    p_transforms = []
    h_transforms = []
    joint_lower = []
    joint_upper = []
    joint_names = []

    # First entry is transformed into torso-centric base frame.
    link1 = _find_body_in_xml(arm_base, cfg["link_names"][0])
    if link1 is None:
        raise ValueError(f"Could not find body '{cfg['link_names'][0]}'")

    R_a_1, p_a_1 = _parse_transform_from_body(link1)
    R_01 = R_base_0 @ R_a_1
    p_01 = p_base_0 + R_base_0 @ p_a_1

    joint1_name = cfg["joint_names"][0]
    h1 = _parse_joint_axis(link1, joint1_name)
    q1_low, q1_high = _parse_joint_limits(link1, joint1_name)

    R_transforms.append(R_01)
    p_transforms.append(p_01)
    h_transforms.append(h1)
    joint_lower.append(q1_low)
    joint_upper.append(q1_high)
    joint_names.append(joint1_name)

    current_body = link1

    # Remaining 6 arm joints are standard local-link transforms.
    for link_name, joint_name in zip(cfg["link_names"][1:], cfg["joint_names"][1:]):
        body = _find_body_in_xml(current_body, link_name)
        if body is None:
            raise ValueError(f"Could not find body '{link_name}'")

        R_i_iplus1, p_i_iplus1 = _parse_transform_from_body(body)
        h_i = _parse_joint_axis(body, joint_name)

        R_transforms.append(R_i_iplus1)
        p_transforms.append(p_i_iplus1)
        h_transforms.append(h_i)
        q_low, q_high = _parse_joint_limits(body, joint_name)
        joint_lower.append(q_low)
        joint_upper.append(q_high)
        joint_names.append(joint_name)

        current_body = body

    # Joint-7 to end-effector frame includes the explicit hand correction frame.
    hand_body = _find_body_in_xml(current_body, cfg["hand_body"])
    if hand_body is None:
        raise ValueError(f"Could not find body '{cfg['hand_body']}'")

    eef_body = _find_body_in_xml(hand_body, cfg["eef_body"])
    if eef_body is None:
        raise ValueError(f"Could not find body '{cfg['eef_body']}'")

    R_7_hand, p_7_hand = _parse_transform_from_body(hand_body)
    R_hand_T, p_hand_T = _parse_transform_from_body(eef_body)

    R_7_T = R_7_hand @ R_hand_T
    p_7_T = p_7_hand + R_7_hand @ p_hand_T

    R_transforms.append(R_7_T)
    p_transforms.append(p_7_T)
    h_transforms.append(np.array([0.0, 0.0, 1.0], dtype=float))
    joint_names.append(cfg["eef_body"])

    if default_joint_angles is None:
        default_joint_angles = {
            "right": DEFAULT_JOINT_ANGLES["right"].copy(),
            "left": DEFAULT_JOINT_ANGLES["left"].copy(),
        }

    if verbose:
        print(f"--- Transforms for {arm_side} arm ({xml_path}) ---")
        for i, (R_i, p_i, h_i, name_i) in enumerate(
            zip(R_transforms, p_transforms, h_transforms, joint_names)
        ):
            print(f"Segment {i + 1} ({name_i}):")
            print(f"  p: {p_i}")
            print(f"  h: {h_i}")
            print(f"  R:\n{R_i}")
        print(f"R_base_0:\n{R_base_0}")
        print(f"p_base_0: {p_base_0}")
        print("-------------------------------------------")

    return {
        "R": R_transforms,
        "p": p_transforms,
        "h": h_transforms,
        "joint_lower": np.array(joint_lower, dtype=float),
        "joint_upper": np.array(joint_upper, dtype=float),
        "joint_names": joint_names,
        "side": arm_side,
        "R_base_0": R_base_0,
        "p_base_0": p_base_0,
        "default_joint_angles": default_joint_angles,
    }

def _find_body_in_xml(parent, body_name):
    """Find a body by name, trying direct children first then descendants."""
    if parent is None:
        return None

    body = parent.find(f"./body[@name='{body_name}']")
    if body is not None:
        return body

    return parent.find(f".//body[@name='{body_name}']")

def _parse_joint_axis(body_element, joint_name):
    """Parse a joint axis from a body; defaults to +z when unspecified."""
    if body_element is None:
        return np.array([0.0, 0.0, 1.0], dtype=float)

    joint = body_element.find(f"./joint[@name='{joint_name}']")
    if joint is None:
        joint = body_element.find(f".//joint[@name='{joint_name}']")

    if joint is None:
        return np.array([0.0, 0.0, 1.0], dtype=float)

    axis = np.array([float(x) for x in joint.get("axis", "0 0 1").split()], dtype=float)
    norm = np.linalg.norm(axis)
    if norm < 1e-12:
        return np.array([0.0, 0.0, 1.0], dtype=float)
    return axis / norm

def _parse_joint_limits(body_element, joint_name):
    """Parse joint limits from MuJoCo joint range="low high"."""
    if body_element is None:
        return -np.inf, np.inf

    joint = body_element.find(f"./joint[@name='{joint_name}']")
    if joint is None:
        joint = body_element.find(f".//joint[@name='{joint_name}']")

    if joint is None:
        return -np.inf, np.inf

    range_str = joint.get("range", None)
    if range_str is None:
        return -np.inf, np.inf

    vals = [float(x) for x in range_str.split()]
    if len(vals) != 2:
        return -np.inf, np.inf

    return float(vals[0]), float(vals[1])

def _parse_transform_from_body(body_element):
    """Parse body pose (R, p) from MuJoCo body fields pos + quat."""
    if body_element is None:
        return np.eye(3), np.zeros(3)

    pos_str = body_element.get("pos", "0 0 0")
    p = np.array([float(x) for x in pos_str.split()], dtype=float)

    quat_str = body_element.get("quat", "1 0 0 0")
    quat = np.array([float(x) for x in quat_str.split()], dtype=float)

    if quat.shape[0] == 4:
        # MuJoCo stores quaternions as (w, x, y, z).
        quat_scipy = np.array([quat[1], quat[2], quat[3], quat[0]], dtype=float)
        R = Rotation.from_quat(quat_scipy).as_matrix()
    else:
        R = np.eye(3)

    return R, p

def main():
    path=Path(xml_path_v1)
    d={'meta/schema':'geo_kin_core.spec/1','meta/kind':'robot','meta/generator_version':'dual_kinova3-reference/1','meta/source_filename':'robot.xml','meta/source_sha256':hashlib.sha256(path.read_bytes()).hexdigest(),'meta/parts':['right_arm','left_arm']}
    for side in ['right','left']:
        tr=get_frame_transforms_from_xml(arm_side=side)
        for key in ['R','p','h','joint_names','joint_lower','joint_upper','R_base_0','p_base_0']:d[f'{side}_arm/{key}']=np.asarray(tr[key]).copy()
    dest=ROOT/'specs/dual_kinova3_v1.npz';dest.parent.mkdir(exist_ok=True)
    with zipfile.ZipFile(dest,'w') as z:
        for key,value in sorted(d.items()):
            b=io.BytesIO();np.lib.format.write_array(b,np.asarray(value),allow_pickle=False)
            z.writestr(zipfile.ZipInfo(key+'.npy',(1980,1,1,0,0,0)),b.getvalue())
    print(dest)
if __name__=='__main__':main()
