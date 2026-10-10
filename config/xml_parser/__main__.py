"""`python3 -m config.xml_parser` — the single file's old demo: prints a few PAR6 values."""

from config.xml_parser import (
    get_active_gripper, get_all_values, get_float_value, get_joint_count,
    get_robot_name, get_single_value, parse_robot_config, set_single_value,
)


# Example usage:  python3 -m config.xml_parser   (from the repo root)
#
# Run as a MODULE, not as a path. `python3 config/xml_parser.py` puts config/ on sys.path
# instead of the repo root, so `project_paths` is not importable and this block dies before
# it does anything — which is how it sat broken, along with an import of another robot's XML
# constant, one that has never existed in project_paths.py (only PAR6_XML does).
if __name__ == "__main__":
    from project_paths import PAR6_XML, PROJECT_ROOT
    xml_path = str(PAR6_XML)
    output_path = str(PROJECT_ROOT / "PAR6_modified.xml")

    # Load and parse
    root, joints = parse_robot_config(xml_path)

    # Robot metadata
    print("Robot Name:", get_robot_name(root))
    print("Active Gripper:", get_active_gripper(root))

    # Single value
    print("KD of joint 2:", get_single_value(joints, 2, "KD"))

    joint_num = get_joint_count(joints)
    print("Joint count:", joint_num)

    print("joint num", get_single_value(joints, 2, "KD"))
    # All gear ratios
    var = get_all_values(joints, "gear_ratio")
    print(type(var))
    print("Gear Ratios:", get_all_values(joints, "gear_ratio"))
    print("joint type:", get_all_values(joints, "joint_type"))
    print("delta_t:",get_float_value(root,"RTI_loop_time"))
    print("delta_t_RTI_joint:",get_float_value(root,"RTI_Joint_mode_loop_time"))
    joint_num = get_joint_count(joints) 
    print(type(joint_num))
    # Modify a single value
    set_single_value(joints, 2, "KP", 3.1415)
