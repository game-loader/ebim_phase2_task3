"""Fetch exact driver revisions at image build time; retain sources and licenses."""

import json
from pathlib import Path
import re
import subprocess
import sys
import os


def clone_url(url):
    proxy = os.environ.get("EBIM_GIT_PROXY", "").rstrip("/")
    return proxy + "/" + url if proxy and url.startswith("https://github.com/") else url


def fetch(lock, destination):
    for name, source in json.loads(Path(lock).read_text()).items():
        if not re.fullmatch(r"[a-zA-Z0-9_]+", name) or not re.fullmatch(r"[a-f0-9]{40}", source["commit"]):
            raise ValueError("driver lock requires safe names and full commit IDs")
        target = Path(destination) / name
        target.mkdir(parents=True)
        def git(*args):
            return subprocess.check_output(["git", "-C", str(target), *args], text=True)
        git("init")
        git("remote", "add", "origin", clone_url(source["url"]))
        git("fetch", "--depth", "1", "origin", source["commit"])
        git("checkout", "--detach", "FETCH_HEAD")
        if git("rev-parse", "HEAD").strip() != source["commit"]:
            raise RuntimeError("driver revision mismatch")
        git("submodule", "update", "--init", "--recursive", "--depth", "1")


if __name__ == "__main__":
    fetch(sys.argv[1], sys.argv[2])
