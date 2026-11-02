"""按 YAML 中的 FRD 仿真杆臂为外部同钟航迹生成 APC。"""

import argparse
from pathlib import Path

from uavsim.config.loader import load_config
from uavsim.motion.lever_arm import export_apc_from_aligned


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=Path("main/motion_config.yaml"))
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    config = load_config(args.config)
    report = export_apc_from_aligned(args.source, args.output, config.antenna.phase_center_frd_m)
    print(args.output, report["lever_frd_m"], report["samples"])


if __name__ == "__main__":
    main()
