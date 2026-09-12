#!/usr/bin/env python3
"""Extract the arm-supplied release into a container's persistent runtime volume."""

import io
import os
from pathlib import Path
import re
import sys
import tarfile
import tempfile


def deploy(root, release, data):
    if not re.fullmatch(r"[a-f0-9]{16}", release):
        raise ValueError("invalid release ID")
    target = Path(root) / "releases" / release
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.exists():
        if not (target / ".complete").is_file() or (target / ".complete").read_text() != release:
            raise RuntimeError("incomplete existing release")
        return
    temporary = Path(tempfile.mkdtemp(prefix=".staging-", dir=target.parent))
    with tarfile.open(fileobj=io.BytesIO(data), mode="r:gz") as tar:
        for member in tar.getmembers():
            if not member.isfile() or member.name.startswith("/") or ".." in Path(member.name).parts:
                raise RuntimeError("invalid deployment archive member")
            path = temporary / member.name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(tar.extractfile(member).read())
    (temporary / ".complete").write_text(release)
    os.rename(temporary, target)


if __name__ == "__main__":
    deploy(sys.argv[1], sys.argv[2], sys.stdin.buffer.read())
