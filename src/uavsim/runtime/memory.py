"""进程峰值内存，仅记录操作系统实际提供的数值。"""

import ctypes
import os


def peak_working_set_bytes() -> int | None:
    if os.name == "nt":

        class Counters(ctypes.Structure):
            _fields_ = [("cb", ctypes.c_ulong), ("page_faults", ctypes.c_ulong)] + [
                (name, ctypes.c_size_t)
                for name in [
                    "peak_working_set",
                    "working_set",
                    "peak_page_pool",
                    "page_pool",
                    "peak_nonpage_pool",
                    "nonpage_pool",
                    "pagefile",
                    "peak_pagefile",
                ]
            ]

        counters = Counters()
        counters.cb = ctypes.sizeof(counters)
        # Windows pseudo-handle -1 表示当前进程，不打开其他进程。
        okay = ctypes.windll.psapi.GetProcessMemoryInfo(
            ctypes.c_void_p(-1), ctypes.byref(counters), counters.cb
        )
        return int(counters.peak_working_set) if okay else None
    try:
        import resource
        import sys

        value = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
        return int(value if sys.platform == "darwin" else value * 1024)
    except (ImportError, OSError):
        return None
