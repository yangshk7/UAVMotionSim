from common import run_example

if __name__ == "__main__":
    run_example("two_pass", {"disturbances": {"all_enabled": True}}, passes=2)
