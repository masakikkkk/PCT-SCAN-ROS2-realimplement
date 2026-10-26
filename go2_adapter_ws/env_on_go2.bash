#!/usr/bin/env bash
# Source after loading the robot's EXISTING Foxy / Unitree / DDS environment.
# Resolves the full repository and both overlays without assuming a username.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo '[go2-env] Use: source go2_adapter_ws/env_on_go2.bash' >&2
  exit 2
fi

_go2_pct_load_environment() {
  local go2_env_flags="$-" go2_env_root go2_env_config go2_env_status=0
  if [[ "${ROS_DISTRO:-}" != foxy ]]; then
    echo '[go2-env] First load the existing Go2 Foxy / Unitree / DDS environment.' >&2
    return 2
  fi
  go2_env_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)" || return 2
  go2_env_config="${GO2_NAV_CONFIG:-$go2_env_root/go2_adapter_ws/src/go2_pct_adapter/config/go2.yaml}"
  if [[ ! -f "$go2_env_config" ||
        ! -f "$go2_env_root/install/setup.bash" ||
        ! -f "$go2_env_root/go2_adapter_ws/install/setup.bash" ]]; then
    echo '[go2-env] Missing configuration or build output; follow DEPLOY_GO2.md first.' >&2
    return 2
  fi
  export PCT_ROS_DISTRO=foxy
  source "$go2_env_root/scripts/_pct_scan_env.sh" || go2_env_status=$?
  if [[ "$go2_env_status" == 0 ]]; then
    set +u
    source "$go2_env_root/go2_adapter_ws/install/setup.bash" || go2_env_status=$?
  fi
  # The upstream environment helper enables nounset; preserve the caller's setting.
  if [[ "$go2_env_flags" == *u* ]]; then set -u; else set +u; fi
  if [[ "$go2_env_status" != 0 ]]; then
    echo '[go2-env] Failed to load the planner or adapter environment.' >&2
    return "$go2_env_status"
  fi
  export GO2_NAV_ROOT="$go2_env_root"
  export GO2_NAV_CONFIG="$go2_env_config"
  echo "[go2-env] Repository: $GO2_NAV_ROOT"
  echo "[go2-env] Configuration: $GO2_NAV_CONFIG"
}

if _go2_pct_load_environment; then
  unset -f _go2_pct_load_environment
else
  unset -f _go2_pct_load_environment
  return 2
fi
