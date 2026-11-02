"""可选的上位机进度事件；默认 CLI 和数值测试不输出事件。"""

import json
import os
import sys
from contextlib import contextmanager


@contextmanager
def convergence_progress(rate: int):
    old = os.environ.get("UAVSIM_CONVERGENCE_RATE")
    os.environ["UAVSIM_CONVERGENCE_RATE"] = str(rate)
    try:
        yield
    finally:
        if old is None:
            os.environ.pop("UAVSIM_CONVERGENCE_RATE", None)
        else:
            os.environ["UAVSIM_CONVERGENCE_RATE"] = old


def report_progress(**payload) -> None:
    if os.environ.get("UAVSIM_GUI_PROGRESS") == "1":
        if "fraction" in payload and os.environ.get("UAVSIM_CONVERGENCE_RATE"):
            payload.update(phase="convergence", rate=int(os.environ["UAVSIM_CONVERGENCE_RATE"]))
        sys.stdout.write("UAVSIM_GUI_EVENT " + json.dumps({"type": "progress", **payload}, ensure_ascii=False) + "\n")
        sys.stdout.flush()
