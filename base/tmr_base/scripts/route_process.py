"""Keep nested route children within the container watchdog's process group."""

import os
import subprocess


def supervised():
    return os.environ.get("EBIM_BASE_ROUTE_SUPERVISED") == "1"


def spawn_route_child(command, **kwargs):
    return subprocess.Popen(command, start_new_session=not supervised(), **kwargs)


def signal_route_child(child, signum):
    if supervised():
        # The outer watchdog owns group shutdown. A local stage timeout should
        # signal only its child, without interrupting its own parent group.
        child.send_signal(signum)
    else:
        os.killpg(child.pid, signum)
