from common import run_example

if __name__ == "__main__":
    run_example(
        "attitude",
        {"disturbances": {"attitude": {axis: {"enabled": True} for axis in ["roll", "pitch", "yaw"]}}},
    )
