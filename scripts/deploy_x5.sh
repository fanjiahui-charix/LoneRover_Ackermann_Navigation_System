#!/usr/bin/env bash
set -euo pipefail

# Deploy only the generated install tree. Keep credentials out of the repository;
# rsync will use the user's SSH agent/key or ask for the SSH password interactively.

workspace_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
install_dir="${workspace_root}/install"
deploy_host="${X5_DEPLOY_HOST:-}"
deploy_dest="${X5_DEPLOY_DEST:-/root/ros_workspace/install/}"

if [ -z "${deploy_host}" ]; then
  echo "Usage: X5_DEPLOY_HOST=<user>@<vehicle-ip> $0" >&2
  exit 2
fi

if [ ! -d "${install_dir}" ]; then
  echo "Missing ${install_dir}; build the X5 workspace first." >&2
  exit 2
fi

rsync_options=(-avL --exclude '/x5/')
if [ "${X5_DEPLOY_DELETE:-0}" = "1" ]; then
  rsync_options+=(--delete --delete-excluded)
fi

echo "Deploying ${install_dir}/ to ${deploy_host}:${deploy_dest}"
rsync "${rsync_options[@]}" "${install_dir}/" "${deploy_host}:${deploy_dest}"
