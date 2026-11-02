from common import run_example

if __name__ == "__main__":
    run_example(
        "navigation",
        {
            "disturbances": {
                key: {"enabled": True}
                for key in [
                    "navigation_position_error",
                    "navigation_velocity_error",
                    "navigation_attitude_error",
                ]
            }
        },
    )
