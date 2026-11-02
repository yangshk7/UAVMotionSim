"""可选的上位机进度事件；默认 CLI 和数值测试不输出事件。"""

import json
import os
import sys


def report_progress(**payload) -> None:
    if os.environ.get("UAVSIM_GUI_PROGRESS") == "1":
        sys.stdout.write("UAVSIM_GUI_EVENT " + json.dumps({"type": "progress", **payload}, ensure_ascii=False) + "\n")
        sys.stdout.flush()
