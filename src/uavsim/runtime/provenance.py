"""记录源代码指纹，不依赖是否已经创建 Git 提交。"""

import hashlib
import subprocess
from pathlib import Path


def source_provenance() -> dict:
    root = Path(__file__).resolve().parents[3]
    paths = []
    for directory, pattern in [("src", "*.py"), ("configs", "*.yaml"), ("matlab", "*.m")]:
        paths.extend((root / directory).rglob(pattern))
    paths.extend(root / name for name in ["pyproject.toml", "uv.lock"] if (root / name).exists())
    digest = hashlib.sha256()
    for path in sorted(paths):
        digest.update(path.relative_to(root).as_posix().encode("utf-8"))
        digest.update(b"\0")
        digest.update(path.read_bytes())
    revision = None
    try:
        result = subprocess.run(
            ["git", "rev-parse", "HEAD"], cwd=root, capture_output=True, text=True, check=False
        )
        if result.returncode == 0:
            revision = result.stdout.strip()
    except OSError:
        pass
    return {
        "source_sha256": digest.hexdigest(),
        "git_revision_at_run": revision,
        "fingerprint_scope": "src/**/*.py, configs/**/*.yaml, matlab/**/*.m, pyproject.toml, uv.lock",
    }
