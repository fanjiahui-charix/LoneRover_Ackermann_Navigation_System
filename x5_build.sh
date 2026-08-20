#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SYSROOT_LINK="${WORKSPACE_ROOT}/../sysroot_docker/usr"
SYSROOT_TARGET="${WORKSPACE_ROOT}/../sysroot_docker/usr_x5"
X5_TOOLCHAIN_ROOT="${X5_TOOLCHAIN_ROOT:-${WORKSPACE_ROOT}/../robot_dev_config}"
X5_TOOLCHAIN_FILE="${X5_TOOLCHAIN_FILE:-${X5_TOOLCHAIN_ROOT}/aarch64_toolchainfile.cmake}"
X5_CLEAR_COLCON_IGNORE="${X5_CLEAR_COLCON_IGNORE:-${X5_TOOLCHAIN_ROOT}/clear_COLCON_IGNORE.sh}"
BUILD_BASE="${WORKSPACE_ROOT}/build"
INSTALL_BASE="${WORKSPACE_ROOT}/install"
LOG_BASE="${WORKSPACE_ROOT}/log"
CACHE_VERSION="2026-06-27-x5-cache-v3"
COLCON_WORKERS="${X5_COLCON_WORKERS:-2}"
BUILD_JOBS="${X5_CMAKE_BUILD_JOBS:-2}"
MAKE_LOAD_LIMIT="${X5_MAKE_LOAD_LIMIT:-4}"
PACKAGE_MODE="${X5_PACKAGE_MODE:-up-to}"
FORCE_CONFIGURE="${X5_FORCE_CONFIGURE:-0}"
BUILD_TESTING="${X5_BUILD_TESTING:-0}"
DEPLOY_AFTER_BUILD="${X5_DEPLOY_AFTER_BUILD:-0}"
DEPLOY_HOST="${X5_DEPLOY_HOST:-}"
DEPLOY_DEST="${X5_DEPLOY_DEST:-/root/ros_workspace/install/}"
PACKAGES=()
STALE_PATH_PATTERNS=(
  "/mnt/2025smartcar"
  "/mnt/test/cc_ws"
  "/root/new/ws_horizon"
  "/root/tros_ws"
)

usage() {
  cat <<'EOF'
用法:
  ./x5_build.sh [--select | --up-to] [--force-configure | --no-force-configure] [--build-testing | --no-build-testing] [--deploy] [package...]

说明:
  不带 package 时默认编译整个工作区。
  --up-to               编译目标包及其依赖，适合首次打通整条链
  --select              只编译指定包，适合已经有依赖产物时做轻量验证
  --force-configure     强制所有包重新执行 CMake configure
  --no-force-configure  使用现有缓存，减少重复重配
  --build-testing       交叉编译测试目标（不在主机运行 aarch64 测试）
  --no-build-testing    不构建测试目标，默认行为
  --deploy              编译成功后直接 rsync 到目标车机
  --deploy-host         目标车机地址，例如 root@<vehicle-ip>
  --deploy-dest         目标车机安装目录，默认 /root/ros_workspace/install/

环境变量:
  X5_PACKAGE_MODE       默认 up-to，可改成 select
  X5_FORCE_CONFIGURE    默认 0，可改成 1
  X5_BUILD_TESTING      默认 0，可改成 1
  X5_COLCON_WORKERS     默认 2，colcon 同时编译的包数量
  X5_CMAKE_BUILD_JOBS   默认 2，单个 CMake 包内部编译并发
  X5_MAKE_LOAD_LIMIT    默认 4，make/ninja 负载上限
  X5_DEPLOY_AFTER_BUILD 默认 0，可改成 1
  X5_DEPLOY_HOST        使用 --deploy 时必填，例如 root@<vehicle-ip>
  X5_DEPLOY_DEST        默认 /root/ros_workspace/install/
  X5_TOOLCHAIN_ROOT     外部 robot_dev_config checkout，默认工作区同级目录
  X5_TOOLCHAIN_FILE     外部 aarch64_toolchainfile.cmake 的完整路径
  X5_CLEAR_COLCON_IGNORE 外部 clear_COLCON_IGNORE.sh 的完整路径
EOF
}

deploy_install_tree() {
  if ! command -v rsync >/dev/null 2>&1; then
    echo "错误: 未找到 rsync，无法自动推送到车机。" >&2
    exit 1
  fi
  if [ -z "${DEPLOY_HOST}" ]; then
    echo "错误: 部署前请设置 X5_DEPLOY_HOST，或传入 --deploy-host。" >&2
    exit 1
  fi

  echo "开始推送安装产物到 ${DEPLOY_HOST}:${DEPLOY_DEST}"
  rsync -avL --delete --delete-excluded --exclude '/x5/' "${INSTALL_BASE}/" "${DEPLOY_HOST}:${DEPLOY_DEST}"
}

tree_contains_stale_paths() {
  local path="$1"
  [ -d "${path}" ] || return 1

  local pattern
  for pattern in "${STALE_PATH_PATTERNS[@]}"; do
    if grep -RIl --exclude-dir=.git -- "${pattern}" "${path}" >/dev/null 2>&1; then
      return 0
    fi
  done
  return 1
}

clean_stale_x5_cache() {
  local cleaned=0
  local cache_version_file="${BUILD_BASE}/.x5_build_cache_version"

  if [ -d "${BUILD_BASE}" ] && {
      [ ! -f "${cache_version_file}" ] ||
      [ "$(cat "${cache_version_file}" 2>/dev/null || true)" != "${CACHE_VERSION}" ];
    }; then
    echo "检测到 X5 构建缓存版本变化，清理 ${BUILD_BASE} 和 ${INSTALL_BASE}。"
    rm -rf "${BUILD_BASE}" "${INSTALL_BASE}"
    cleaned=1
  fi

  if tree_contains_stale_paths "${BUILD_BASE}"; then
    echo "检测到 ${BUILD_BASE} 中存在旧工作区绝对路径，清理 X5 build 缓存。"
    rm -rf "${BUILD_BASE}"
    cleaned=1
  fi

  if tree_contains_stale_paths "${INSTALL_BASE}"; then
    echo "检测到 ${INSTALL_BASE} 中存在旧工作区绝对路径，清理 X5 install 缓存。"
    rm -rf "${INSTALL_BASE}"
    cleaned=1
  fi

  if [ "${cleaned}" = "1" ] && [ "${FORCE_CONFIGURE}" = "0" ]; then
    echo "已清理旧缓存，本次会重新配置受影响的包。"
  fi
}

clean_legacy_x5_subdirs() {
  local legacy_paths=(
    "${WORKSPACE_ROOT}/build/x5"
    "${WORKSPACE_ROOT}/install/x5"
    "${WORKSPACE_ROOT}/log/x5"
  )
  local legacy_path
  local removed=0

  for legacy_path in "${legacy_paths[@]}"; do
    if [ -e "${legacy_path}" ]; then
      echo "清理旧的 x5 子目录: ${legacy_path}"
      rm -rf "${legacy_path}"
      removed=1
    fi
  done

  if [ "${removed}" = "1" ]; then
    echo "已清理旧的 x5 子目录，后续产物会直接进入 build/install/log。"
  fi
}

while [ $# -gt 0 ]; do
  case "$1" in
    --select)
      PACKAGE_MODE="select"
      shift
      ;;
    --up-to)
      PACKAGE_MODE="up-to"
      shift
      ;;
    --force-configure)
      FORCE_CONFIGURE=1
      shift
      ;;
    --no-force-configure)
      FORCE_CONFIGURE=0
      shift
      ;;
    --build-testing)
      BUILD_TESTING=1
      shift
      ;;
    --no-build-testing)
      BUILD_TESTING=0
      shift
      ;;
    --deploy)
      DEPLOY_AFTER_BUILD=1
      shift
      ;;
    --no-deploy)
      DEPLOY_AFTER_BUILD=0
      shift
      ;;
    --deploy-host)
      if [ $# -lt 2 ] || [ -z "${2}" ]; then
        echo "错误: --deploy-host 需要跟一个目标地址，例如 root@<vehicle-ip>" >&2
        exit 1
      fi
      DEPLOY_HOST="${2:-}"
      shift 2
      ;;
    --deploy-dest)
      if [ $# -lt 2 ] || [ -z "${2}" ]; then
        echo "错误: --deploy-dest 需要跟一个目标目录，例如 /root/ros_workspace/install/" >&2
        exit 1
      fi
      DEPLOY_DEST="${2:-}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      PACKAGES+=("$1")
      shift
      ;;
  esac
done

if [ -e "${SYSROOT_LINK}" ] && [ ! -L "${SYSROOT_LINK}" ]; then
  echo "错误: ${SYSROOT_LINK} 现在是目录，不是软链接。请先整理 sysroot 后再编译。" >&2
  exit 1
fi

if [ -L "${SYSROOT_LINK}" ]; then
  rm "${SYSROOT_LINK}"
fi

ln -s "${SYSROOT_TARGET}" "${SYSROOT_LINK}"

export ROS_VERSION=2

cd "${WORKSPACE_ROOT}"

if [ -f "${X5_CLEAR_COLCON_IGNORE}" ]; then
  "${X5_CLEAR_COLCON_IGNORE}"
else
  echo "错误: 外部清理脚本不存在: ${X5_CLEAR_COLCON_IGNORE}" >&2
  echo "请准备 D-Robotics robot_dev_config，或设置 X5_CLEAR_COLCON_IGNORE。" >&2
  exit 1
fi

if [ ! -f "${X5_TOOLCHAIN_FILE}" ]; then
  echo "错误: X5 工具链文件不存在: ${X5_TOOLCHAIN_FILE}" >&2
  echo "请准备外部 robot_dev_config，或设置 X5_TOOLCHAIN_FILE。" >&2
  exit 1
fi

echo "编译平台: X5"
echo "输出目录: ${BUILD_BASE} | ${INSTALL_BASE} | ${LOG_BASE}"
echo "并发限制: colcon workers=${COLCON_WORKERS}, build jobs=${BUILD_JOBS}, load limit=${MAKE_LOAD_LIMIT}"
echo "包模式: ${PACKAGE_MODE}"
if [ ${#PACKAGES[@]} -eq 0 ]; then
  echo "目标包: 全工作区"
else
  echo "目标包: ${PACKAGES[*]}"
fi
echo "强制重配: ${FORCE_CONFIGURE}"
echo "构建测试目标: ${BUILD_TESTING}"
echo "自动部署: ${DEPLOY_AFTER_BUILD}"
if [ "${DEPLOY_AFTER_BUILD}" = "1" ]; then
  echo "部署目标: ${DEPLOY_HOST}:${DEPLOY_DEST}"
fi

set +u
source /opt/ros/humble/setup.bash
set -u

export TARGET_ARCH=aarch64
export TARGET_TRIPLE=aarch64-linux-gnu
export CROSS_COMPILE="/usr/bin/${TARGET_TRIPLE}-"

unset COLCON_PREFIX_PATH
unset COLCON_CURRENT_PREFIX
unset CMAKE_PREFIX_PATH

export CMAKE_PREFIX_PATH="${INSTALL_BASE}:${CMAKE_PREFIX_PATH:-}"
export AMENT_PREFIX_PATH="${INSTALL_BASE}:${AMENT_PREFIX_PATH:-}"
export COLCON_PREFIX_PATH="${INSTALL_BASE}:${COLCON_PREFIX_PATH:-}"
export PKG_CONFIG_SYSROOT_DIR="${WORKSPACE_ROOT}/../sysroot_docker"
export PKG_CONFIG_LIBDIR="${SYSROOT_TARGET}/lib/aarch64-linux-gnu/pkgconfig:${SYSROOT_TARGET}/lib/pkgconfig:${SYSROOT_TARGET}/share/pkgconfig"
export PKG_CONFIG_PATH="${PKG_CONFIG_LIBDIR}"
export CMAKE_BUILD_PARALLEL_LEVEL="${BUILD_JOBS}"
export MAKEFLAGS="-j${BUILD_JOBS} -l${MAKE_LOAD_LIMIT}"

# Link-time search paths for transitive Horizon X5 SDK dependencies.
# This is only for cross-linking on the host; ros2 commands still run on the car.
HOBOT_RPATH_LINK_FLAGS="-Wl,-rpath-link,${SYSROOT_TARGET}/hobot/lib -Wl,-rpath-link,${SYSROOT_TARGET}/lib -Wl,-rpath-link,${SYSROOT_TARGET}/lib/aarch64-linux-gnu"

clean_stale_x5_cache
clean_legacy_x5_subdirs

mkdir -p "${BUILD_BASE}" "${INSTALL_BASE}" "${LOG_BASE}"
printf '%s\n' "${CACHE_VERSION}" > "${BUILD_BASE}/.x5_build_cache_version"
if [ -f "${X5_TOOLCHAIN_ROOT}/create_soft_link.py" ]; then
  cp "${X5_TOOLCHAIN_ROOT}/create_soft_link.py" "${INSTALL_BASE}/"
fi

COLCON_PACKAGE_ARGS=()
if [ ${#PACKAGES[@]} -gt 0 ]; then
  if [ "${PACKAGE_MODE}" = "select" ]; then
    COLCON_PACKAGE_ARGS=(--packages-select "${PACKAGES[@]}")
  else
    COLCON_PACKAGE_ARGS=(--packages-up-to "${PACKAGES[@]}")
  fi
fi

COLCON_CONFIGURE_ARGS=()
if [ "${FORCE_CONFIGURE}" = "1" ]; then
  COLCON_CONFIGURE_ARGS=(--cmake-force-configure)
fi

colcon --log-base "${LOG_BASE}" build \
  "${COLCON_PACKAGE_ARGS[@]}" \
  --parallel-workers "${COLCON_WORKERS}" \
  --build-base "${BUILD_BASE}" \
  --install-base "${INSTALL_BASE}" \
  --merge-install \
  "${COLCON_CONFIGURE_ARGS[@]}" \
  --cmake-args \
    --no-warn-unused-cli \
    -DCMAKE_TOOLCHAIN_FILE="${X5_TOOLCHAIN_FILE}" \
    -DPLATFORM_X5=ON \
    -DTHIRDPARTY=ON \
    -DBUILD_TESTING="${BUILD_TESTING}" \
    -DCMAKE_BUILD_WITH_INSTALL_RPATH=ON \
    -DCMAKE_EXE_LINKER_FLAGS="${HOBOT_RPATH_LINK_FLAGS}" \
    -DCMAKE_SHARED_LINKER_FLAGS="${HOBOT_RPATH_LINK_FLAGS}" \
    -DCMAKE_MODULE_LINKER_FLAGS="${HOBOT_RPATH_LINK_FLAGS}" \
    -DCMAKE_BUILD_RPATH="${BUILD_BASE}/poco_vendor/poco_external_project_install/lib/;${BUILD_BASE}/libyaml_vendor/libyaml_install/lib/" \
    -DCMAKE_SUPPRESS_DEVELOPER_WARNINGS=ON

if [ "${DEPLOY_AFTER_BUILD}" = "1" ]; then
  deploy_install_tree
fi
