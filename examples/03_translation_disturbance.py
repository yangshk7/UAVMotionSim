from common import run_example

if __name__ == "__main__":
    run_example(
        "translation",
        {"disturbances": {"translation": {axis: {"enabled": True} for axis in ["x", "y", "z"]}}},
    )
