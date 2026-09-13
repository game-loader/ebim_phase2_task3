#!/usr/bin/env bash
# Run on the Linux arm machine. Only Bash and Docker are required on the host.
set -euo pipefail
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
operation=${1:-plan}
[[ $# -gt 0 ]] && shift
profile="$root/hardware.yaml"
image=${EBIM_IMAGE:-franka-duo-table-mission:phase2}
name=${EBIM_CONTAINER:-ebim-cup-bowl-runtime}
ssh_dir=${EBIM_SSH_DIR:-$HOME/.ssh}
activate=()
execute=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --hardware) profile=${2:?missing hardware file}; shift 2 ;;
    --activate) activate=(--activate); shift ;;
    --execute) execute=(--execute); shift ;;
    *) echo "Unknown option: $1" >&2; exit 2 ;;
  esac
done
case "$operation" in plan|check|up|status|logs|down|mission|run) ;; *) echo "Unknown operation: $operation" >&2; exit 2 ;; esac
if [[ ${#activate[@]} -gt 0 && "$operation" != up ]]; then
  echo '--activate is only valid with up' >&2; exit 2
fi
if [[ ${#execute[@]} -gt 0 && "$operation" != mission && "$operation" != run ]]; then
  echo '--execute is only valid with mission/run' >&2; exit 2
fi
if [[ "$operation" == run ]]; then
  [[ ${#execute[@]} -gt 0 ]] || { echo 'run requires --execute (physical mission)' >&2; exit 2; }
  bash "$0" up --hardware "$profile" --activate
  exec bash "$0" mission --execute
fi
owned() {
  local label
  label=$(docker inspect --format '{{ index .Config.Labels "io.ebim.runtime" }}' "$name")
  [[ "$label" == arm ]] || { echo "Refusing unrelated container: $name" >&2; exit 1; }
}
inside() { docker exec "$name" /app/entrypoint.sh hardware "$@"; }
if [[ "$operation" == check ]] && docker container inspect "$name" >/dev/null 2>&1; then
  owned
  inside check
  exit 0
fi
case "$operation" in
  status|logs|mission|down)
    owned
    case "$operation" in
      status)
        docker exec "$name" cat /app/runtime/service.json
        inside status ;;
      logs) exec docker logs --tail 150 "$name" ;;
      mission)
        if [[ ${#execute[@]} -gt 0 ]]; then
          docker exec "$name" /usr/bin/python3 -c 'import json; assert json.load(open("/app/runtime/service.json"))["state"] == "ready", "runtime is not ready; inspect status/logs"'
        fi
        inside mission ${execute[@]+"${execute[@]}"} ;;
      down)
        # If deactivation or ownership checks fail, leave the container alive.
        inside down
        docker stop --time 180 "$name"
        docker rm "$name" ;;
    esac
    exit 0 ;;
esac
if [[ "$operation" == up ]] && docker container inspect "$name" >/dev/null 2>&1; then
  echo "Container $name already exists; use status/down before starting again." >&2
  exit 1
fi
profile=$(cd "$(dirname "$profile")" && pwd)/$(basename "$profile")
[[ -f "$profile" ]] || { echo "Missing profile: $profile" >&2; exit 1; }
[[ "$profile" != *,* ]] || { echo 'Profile path cannot contain commas' >&2; exit 1; }
profile_dir=$(dirname "$profile")
temporary=$(mktemp)
trap 'rm -f "$temporary"' EXIT
# Parse YAML in a network-isolated helper from the already-loaded image.
# NUL-separated arguments are read literally; no eval and no Docker socket.
docker run --rm --pull never --network none --entrypoint /usr/bin/python3 \
  --mount "type=bind,src=$profile_dir,dst=$profile_dir,readonly" \
  "$image" /app/docker/launch_args.py "$profile" "$image" "$ssh_dir" "$name" "$operation" ${activate[@]+"${activate[@]}"} > "$temporary"
args=()
while IFS= read -r -d '' argument; do args+=("$argument"); done < "$temporary"
docker "${args[@]}"
if [[ "$operation" == up ]]; then
  # Wait for completed bring-up, but retain the container on failure/timeout.
  for ((attempt=0; attempt<240; attempt++)); do
    state=$(docker exec "$name" /usr/bin/python3 -c 'import json,pathlib; p=pathlib.Path("/app/runtime/service.json"); s=json.loads(p.read_text()) if p.exists() else {}; print(s["state"] if s.get("instance") == str(pathlib.Path("/proc/1/ns/pid").stat().st_ino) else "starting")')
    case "$state" in
      ready) echo "Runtime ready: $name"; exit 0 ;;
      starting) sleep 5 ;;
      *) echo "Runtime $state; inspect: bash scripts/docker_hardware.sh status" >&2; exit 1 ;;
    esac
  done
  echo 'Startup wait timed out; runtime retained for inspection.' >&2
  exit 1
fi
