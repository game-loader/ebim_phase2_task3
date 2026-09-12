"""Fetch exact driver revisions at image build time; retain sources and licenses."""

import json
from pathlib import Path
import re
import subprocess
import sys
import os


def git_environment():
    env = os.environ.copy()
    proxy = env.get("EBIM_GIT_PROXY", "").rstrip("/")
    if proxy:
        # Environment config is inherited by recursive submodule Git processes.
        # Keep canonical origins and leave host/global Git config untouched.
        index = int(env.get("GIT_CONFIG_COUNT", "0"))
        env[f"GIT_CONFIG_KEY_{index}"] = f"url.{proxy}/https://github.com/.insteadOf"
        env[f"GIT_CONFIG_VALUE_{index}"] = "https://github.com/"
        env["GIT_CONFIG_COUNT"] = str(index + 1)
    return env


def fetch(lock, destination):
    for name, source in json.loads(Path(lock).read_text()).items():
        if not re.fullmatch(r"[a-zA-Z0-9_]+", name) or not re.fullmatch(r"[a-f0-9]{40}", source["commit"]):
            raise ValueError("driver lock requires safe names and full commit IDs")
        target = Path(destination) / name
        target.mkdir(parents=True)
        def git(*args):
            return subprocess.check_output(["git", "-C", str(target), *args], text=True, env=git_environment())
        git("init")
        git("remote", "add", "origin", source["url"])
        git("fetch", "--depth", "1", "origin", source["commit"])
        git("checkout", "--detach", "FETCH_HEAD")
        if git("rev-parse", "HEAD").strip() != source["commit"]:
            raise RuntimeError("driver revision mismatch")
        if source.get("tag"):
            tag = source["tag"]
            if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._-]*", tag):
                raise ValueError("invalid release tag")
            git("fetch", "--depth", "1", "origin", f"refs/tags/{tag}:refs/tags/{tag}")
            if git("rev-parse", f"{tag}^{{commit}}").strip() != source["commit"]:
                raise RuntimeError("release tag differs from locked commit")
        git("submodule", "update", "--init", "--recursive", "--depth", "1")


if __name__ == "__main__":
    fetch(sys.argv[1], sys.argv[2])
