#!/usr/bin/env python3

import argparse
import onnxruntime as ort

# Metadata keys read by the controller (mjlab exporter names), see the "ONNX metadata" docs
JOINT_KEYS = [
    ("joint_stiffness", "kp"),
    ("joint_damping", "kd"),
    ("default_joint_pos", "q0"),
    ("action_scale", "scale"),
]
USED_KEYS = {"joint_names", "observation_names"} | {key for key, _ in JOINT_KEYS}


def shape_to_string(shape):
    return "[" + ", ".join(str(dim) for dim in shape) + "]"


def print_tensors(title, tensors):
    print(f"{title}:")
    for tensor in tensors:
        print(f"  Name : {tensor.name}")
        print(f"  Type : {tensor.type}")
        print(f"  Shape: {shape_to_string(tensor.shape)}")

        # Compute flattened size if all dimensions are known
        if all(isinstance(dim, int) for dim in tensor.shape):
            size = 1
            for dim in tensor.shape:
                size *= dim
            print(f"  Total elements: {size}")
        else:
            print("  Total elements: dynamic")
        print()


def split_list(value):
    """Same parsing as the controller: comma-separated, spaces trimmed"""
    return [item.strip() for item in value.split(",") if item.strip()]


def print_metadata(metadata, action_size=None, raw=False):
    """Print the custom metadata as the controller will use it"""
    print("Metadata:")
    if not metadata:
        print("  None. policy.yaml must give kp/kd, and the convention an 'all' joint group.")
        print()
        return

    joints = split_list(metadata.get("joint_names", ""))
    columns = []
    print("  Used by the controller:")
    print(f"    joint_names       : {f'{len(joints)} joints' if joints else 'absent'}")
    for key, label in JOINT_KEYS:
        if key not in metadata:
            status = "absent"
        elif not joints:
            status = "ignored by the controller: no joint_names"
        else:
            items = split_list(metadata[key])
            try:
                values = [float(item) for item in items]
            except ValueError:
                values = None
            if values is None:
                status = "ignored by the controller: not a list of numbers"
            elif len(values) != len(joints):
                status = f"ignored by the controller: {len(values)} values for {len(joints)} joints"
            else:
                status = f"used as {label}"
                columns.append((label, values))
        print(f"    {key:<18}: {status}")
    observations = split_list(metadata.get("observation_names", ""))
    print(f"    observation_names : {f'{len(observations)} terms' if observations else 'absent'}")

    if action_size is not None and joints and action_size != len(joints):
        print(f"  Note: the action size ({action_size}) differs from the number of joint_names ({len(joints)}),"
              " so action.joints must name a smaller group.")
    print()

    if joints:
        print("  Joint order (the 'all' group):")
        header = f"    {'#':>3}  {'joint':<{max(len(j) for j in joints)}}"
        header += "".join(f"  {label:>8}" for label, _ in columns)
        print(header)
        for i, joint in enumerate(joints):
            row = f"    {i:>3}  {joint:<{max(len(j) for j in joints)}}"
            row += "".join(f"  {values[i]:>8.6g}" for _, values in columns)
            print(row)
        print()

    if observations:
        print("  Observation terms, in input order (compare with observations.yaml):")
        for i, name in enumerate(observations):
            print(f"    {i:>3}  {name}")
        print()

    others = {key: value for key, value in metadata.items() if key not in USED_KEYS}
    if others:
        print("  Other keys (not used by the controller):")
        for key, value in sorted(others.items()):
            if not raw and len(value) > 80:
                value = value[:77] + "..."
            print(f"    {key}: {value}")
        print()


def main():
    parser = argparse.ArgumentParser(description="Print the inputs, outputs and training metadata of an ONNX policy")
    parser.add_argument("model", help="path to the .onnx file")
    parser.add_argument("--raw", action="store_true", help="print long metadata values in full")
    args = parser.parse_args()

    session = ort.InferenceSession(args.model)

    print(f"Model: {args.model}")
    print()

    print_tensors("Inputs", session.get_inputs())
    print_tensors("Outputs", session.get_outputs())

    outputs = session.get_outputs()
    action_size = outputs[0].shape[-1] if outputs and isinstance(outputs[0].shape[-1], int) else None
    print_metadata(session.get_modelmeta().custom_metadata_map, action_size, args.raw)


if __name__ == "__main__":
    main()
