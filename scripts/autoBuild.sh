#!/usr/bin/env bash
# ============================================================================
# pure-nav 自动构建脚本
#
# 设计原则：第三方库与本项目模块**分开构建、分开存放**，互不污染。
#
#   build/thirdparty/              ★ 第三方库专属区域（本脚本管理）
#   ├── build/<库名>/                每个库各自独立的 CMake 构建树
#   ├── install/                     统一安装前缀 include/ lib/ bin/ lib/cmake/
#   ├── deps/                        免 root 补装的构建期依赖（如 libepoxy-dev 头文件）
#   ├── logs/<库名>.log              各库完整构建日志
#   └── env.sh                       自动生成：CMAKE_PREFIX_PATH / LD_LIBRARY_PATH
#
#   build/                         本项目构建树（CMakeFiles/ lib/ compile_commands.json ...）
#   bin/                           本项目可执行文件
#
# 三个第三方库（git submodule，源码在 src/thirdparty/）：
#   Livox-SDK2      -> liblivox_lidar_sdk_{static,shared}.a/.so
#   Pangolin        -> libpango_*.so（可视化）
#   matplotlib-cpp  -> header-only，安装 matplotlibcpp.h + CMake 包配置
#
# 用法：scripts/autoBuild.sh [命令] [选项]      （-h 查看完整帮助）
# ============================================================================
set -Eeuo pipefail

# ----------------------------------------------------------------------------
# 路径（脚本可在任意目录被调用）
# ----------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

BUILD_DIR="${ROOT_DIR}/build"             # 本项目构建树
BIN_DIR="${ROOT_DIR}/bin"                 # 本项目可执行文件

TP_ROOT="${BUILD_DIR}/thirdparty"         # ★ 第三方库专属区域
TP_BUILD_DIR="${TP_ROOT}/build"           #   各库独立构建树
TP_PREFIX="${TP_ROOT}/install"            #   统一安装前缀
TP_DEPS_DIR="${TP_ROOT}/deps"             #   构建期依赖（免 root 解包）
TP_LOG_DIR="${TP_ROOT}/logs"              #   构建日志
TP_ENV_FILE="${TP_ROOT}/env.sh"           #   生成的环境脚本

SRC_TP_DIR="${ROOT_DIR}/src/thirdparty"   # 第三方库源码（submodule）
TP_LIBS=(Livox-SDK2 Pangolin matplotlib-cpp)

# ----------------------------------------------------------------------------
# 默认参数
# ----------------------------------------------------------------------------
JOBS="$(nproc 2>/dev/null || echo 4)"
BUILD_TYPE="Release"
BUILD_TESTS="OFF"
WERROR="OFF"
REBUILD="OFF"
COMMAND="all"
CMAKE_EXTRA=()

CXX_COMPILER=""
CC_COMPILER=""
MULTIARCH=""

# ----------------------------------------------------------------------------
# 日志（全部走 stderr，保证 stdout 只出现命令结果）
# ----------------------------------------------------------------------------
log()  { printf '\033[1;34m[autoBuild]\033[0m %s\n' "$*" >&2; }
warn() { printf '\033[1;33m[autoBuild][警告]\033[0m %s\n' "$*" >&2; }
die()  { printf '\033[1;31m[autoBuild][错误]\033[0m %s\n' "$*" >&2; exit 1; }
have() { command -v "$1" >/dev/null 2>&1; }

usage() {
    cat <<'EOF'
pure-nav 自动构建脚本

用法:
  scripts/autoBuild.sh [命令] [选项]

命令:
  all          第三方库 + 本项目（默认；第三方库已就绪时自动跳过）
  thirdparty   只构建三个第三方库（总是重新配置，增量编译）
  project      只构建本项目（配置 -> 编译；产物在 build/ 与 bin/）
  clean        删除第三方库构建中间产物 build/thirdparty/build（保留 install）
  distclean    删除 build/ 与 bin/ 下全部产物（保留 .gitkeep）
  status       查看第三方库 / 本项目的构建产物状态
  help         显示本帮助

选项:
  -j N             并行任务数（默认 nproc）
  -t TYPE          构建类型 Release|Debug|RelWithDebInfo|MinSizeRel（默认 Release）
  --tests          同时构建 src/test 单元测试（-DPURE_NAV_BUILD_TESTS=ON）
  --werror         警告视为错误（-DPURE_NAV_WARNINGS_AS_ERRORS=ON）
  --rebuild        先清空对应构建目录再构建
  -D VAR=VALUE     追加/覆盖本项目 CMake 变量，可重复，例如 -D PURE_NAV_BUILD_TESTS=ON
  -h, --help       显示本帮助

产物布局:
  build/thirdparty/install/   第三方库（与项目模块分开，见脚本头部注释）
  build/lib/                  本项目静态库
  bin/                        本项目可执行文件

环境变量:
  CXX                         显式指定 C++ 编译器（默认自动探测；缺失时免 root 本地解包 g++）
  CC                          显式指定 C 编译器
  PURE_NAV_TOOLCHAIN_DIR      本地解包 g++ 的缓存目录（默认 ~/.cache/pure-nav/toolchain）
  PURE_NAV_NO_LOCAL_TOOLCHAIN 设为 1 时禁用免 root 本地解包，直接报错提示装 g++
EOF
}

# ----------------------------------------------------------------------------
# 参数解析
# ----------------------------------------------------------------------------
parse_args() {
    while [ $# -gt 0 ]; do
        case "$1" in
            all|thirdparty|project|clean|distclean|status|help) COMMAND="$1"; shift ;;
            -j)   [ $# -ge 2 ] || die "-j 需要一个参数"; JOBS="$2"; shift 2 ;;
            -j*)  JOBS="${1#-j}"; shift ;;
            -t)   [ $# -ge 2 ] || die "-t 需要一个参数"; BUILD_TYPE="$2"; shift 2 ;;
            -t*)  BUILD_TYPE="${1#-t}"; shift ;;
            --tests)   BUILD_TESTS="ON"; shift ;;
            --werror)  WERROR="ON"; shift ;;
            --rebuild) REBUILD="ON"; shift ;;
            -D)   [ $# -ge 2 ] || die "-D 需要一个参数"; CMAKE_EXTRA+=("$2"); shift 2 ;;
            -D*)  CMAKE_EXTRA+=("${1#-D}"); shift ;;
            -h|--help) COMMAND="help"; shift ;;
            *) die "未知参数：$1（-h 查看用法）" ;;
        esac
    done

    case "$BUILD_TYPE" in
        Debug|Release|RelWithDebInfo|MinSizeRel) ;;
        *) die "不支持的构建类型：$BUILD_TYPE" ;;
    esac
    case "$JOBS" in
        ''|*[!0-9]*) die "-j 必须是正整数：$JOBS" ;;
    esac
    [ "$JOBS" -ge 1 ] 2>/dev/null || die "-j 必须 >= 1：$JOBS"
}

# ----------------------------------------------------------------------------
# C++ 编译器：自动探测；系统缺失时免 root 本地解包 g++（见 ARCHITECTURE.md 7.2）
# ----------------------------------------------------------------------------
probe_cxx() {
    local c="$1" tmp
    [ -n "$c" ] || return 1
    have "$c" || return 1
    tmp="$(mktemp -d 2>/dev/null)" || return 1
    printf 'int main(){return 0;}\n' > "${tmp}/probe.cpp"
    if "$c" -x c++ -std=c++11 "${tmp}/probe.cpp" -o "${tmp}/probe.out" >/dev/null 2>&1; then
        rm -rf "$tmp"
        return 0
    fi
    rm -rf "$tmp"
    return 1
}

# 依次尝试：$CXX -> g++ -> c++ -> clang++ -> x86_64-linux-gnu-g++
find_cxx() {
    local c
    for c in ${CXX:-} g++ c++ clang++ x86_64-linux-gnu-g++; do
        if probe_cxx "$c"; then printf '%s' "$c"; return 0; fi
    done
    return 1
}

# 免 root 方案：apt 只下载 .deb，dpkg-deb 解包到缓存目录，再生成包装脚本。
# 系统已有 gcc（C 前端）与 libstdc++ 头文件，因此只需要补 cc1plus（在
# g++-<ver>-<gnu-type> 包里）。输出：包装脚本的绝对路径。
provision_toolchain() {
    [ "${PURE_NAV_NO_LOCAL_TOOLCHAIN:-0}" = "1" ] && return 1
    have apt-get || return 1
    have dpkg-deb || return 1
    have gcc || return 1

    local tc="${PURE_NAV_TOOLCHAIN_DIR:-${XDG_CACHE_HOME:-$HOME/.cache}/pure-nav/toolchain}"
    local wrapper="${tc}/bin/g++-wrapped"
    local ver=""
    ver="$(basename "$(ls -1d /usr/include/c++/* 2>/dev/null | sort -V | tail -1)" 2>/dev/null || true)"
    [ -n "$ver" ] || ver="$(gcc -dumpversion 2>/dev/null | cut -d. -f1)"
    [ -n "$ver" ] || ver="15"

    if [ -x "$wrapper" ] && probe_cxx "$wrapper"; then
        printf '%s' "$wrapper"
        return 0
    fi

    local debs="${tc}/deb"
    mkdir -p "$debs" "${tc}/usr/include" "${tc}/usr/lib/gcc/${MULTIARCH}" \
             "${tc}/usr/libexec/gcc/${MULTIARCH}/${ver}"

    log "系统缺少 C++ 前端，尝试免 root 本地解包 g++-${ver}（首次约 15MB）..."

    # 包名在不同架构上不同：amd64 上是 g++-15-x86-64-linux-gnu
    local pkg="" cand
    for cand in "g++-${ver}-x86-64-linux-gnu" "g++-${ver}-${MULTIARCH}" "g++-${ver}"; do
        if ( cd "$debs" && apt-get download "$cand" >/dev/null 2>&1 ); then
            pkg="$cand"
            break
        fi
    done
    [ -n "$pkg" ] || return 1

    local deb
    for deb in "$debs"/g++-*.deb; do
        [ -e "$deb" ] || continue
        dpkg-deb -x "$deb" "$tc" >/dev/null 2>&1 || true
    done

    # 把驱动脚本需要的系统目录挂进来（-B 会把前缀重定位到 ${tc}/usr）
    ln -sfn /usr/include/c++ "${tc}/usr/include/c++"
    if [ -d "/usr/include/${MULTIARCH}" ]; then
        ln -sfn "/usr/include/${MULTIARCH}" "${tc}/usr/include/${MULTIARCH}"
    fi
    if [ -d "/usr/lib/gcc/${MULTIARCH}/${ver}" ]; then
        ln -sfn "/usr/lib/gcc/${MULTIARCH}/${ver}" "${tc}/usr/lib/gcc/${MULTIARCH}/${ver}"
    fi
    # cc1plus 之外的前端/插件（collect2 / liblto_plugin.so 等）仍用系统的
    local f b
    for f in /usr/libexec/gcc/"${MULTIARCH}"/"${ver}"/*; do
        [ -e "$f" ] || continue
        b="$(basename "$f")"
        [ -e "${tc}/usr/libexec/gcc/${MULTIARCH}/${ver}/${b}" ] || \
            ln -s "$f" "${tc}/usr/libexec/gcc/${MULTIARCH}/${ver}/${b}"
    done

    local driver
    driver="$(find "${tc}/usr/bin" -maxdepth 1 -type f -name '*g++*' 2>/dev/null | head -1)"
    [ -n "$driver" ] || return 1

    mkdir -p "${tc}/bin"
    cat > "$wrapper" <<EOF
#!/bin/sh
# pure-nav 自动生成：调用免 root 解包的 g++（无需 sudo）
exec "${driver}" -B"${tc}/usr/libexec/gcc/${MULTIARCH}/${ver}/" "\$@"
EOF
    chmod +x "$wrapper"

    if probe_cxx "$wrapper"; then
        printf '%s' "$wrapper"
        return 0
    fi
    return 1
}

setup_compilers() {
    MULTIARCH="$(gcc -dumpmachine 2>/dev/null || echo x86_64-linux-gnu)"

    local c
    if c="$(find_cxx)"; then
        CXX_COMPILER="$(command -v "$c")"
    else
        log "未找到可用的 C++ 编译器（g++/clang++），启用免 root 本地解包方案"
        if ! c="$(provision_toolchain)"; then
            die "无法获得 C++ 编译器。请执行：sudo apt install -y g++（或用 PURE_NAV_TOOLCHAIN_DIR 指定缓存目录）"
        fi
        CXX_COMPILER="$c"
    fi

    if [ -n "${CC:-}" ]; then
        CC_COMPILER="$(command -v "${CC}" 2>/dev/null || true)"
    else
        CC_COMPILER="$(command -v gcc 2>/dev/null || command -v cc 2>/dev/null || true)"
    fi
    [ -n "$CC_COMPILER" ] || die "找不到 C 编译器（gcc）"

    log "C   编译器：${CC_COMPILER}"
    log "C++ 编译器：${CXX_COMPILER}"
    if ! "${CXX_COMPILER}" --version >/dev/null 2>&1; then
        die "C++ 编译器不可用：${CXX_COMPILER}"
    fi
    log "C++ 版本  ：$("${CXX_COMPILER}" --version 2>/dev/null | head -1 | tr -d '\r')"
}

# 编译器变化 / 缓存里的编译器失效时，自动清掉 CMake 缓存，避免陈旧的绝对路径报错
ensure_build_dir() {
    local bdir="$1" stamp="${1}/.pure_nav_cxx_compiler"
    if [ -f "${bdir}/CMakeCache.txt" ]; then
        local cached cached_real chosen_real
        cached="$(grep -m1 -E '^CMAKE_CXX_COMPILER:' "${bdir}/CMakeCache.txt" | cut -d= -f2- || true)"
        if [ -n "$cached" ]; then
            cached_real="$(readlink -f "$cached" 2>/dev/null || printf '%s' "$cached")"
            chosen_real="$(readlink -f "$CXX_COMPILER" 2>/dev/null || printf '%s' "$CXX_COMPILER")"
            case "$cached" in
                */*)
                    if [ ! -x "$cached" ]; then
                        warn "缓存中的 C++ 编译器已失效（$cached），清理 $bdir"
                        rm -rf "$bdir/CMakeCache.txt" "$bdir/CMakeFiles"
                    elif [ "$cached_real" != "$chosen_real" ]; then
                        warn "C++ 编译器已变化（$cached -> $CXX_COMPILER），清理 $bdir"
                        rm -rf "$bdir/CMakeCache.txt" "$bdir/CMakeFiles"
                    fi ;;
                *)
                    if [ "$cached" != "$CXX_COMPILER" ]; then
                        warn "C++ 编译器已变化（$cached -> $CXX_COMPILER），清理 $bdir"
                        rm -rf "$bdir/CMakeCache.txt" "$bdir/CMakeFiles"
                    fi ;;
            esac
        fi
    fi
    mkdir -p "$bdir"
    printf '%s\n' "$CXX_COMPILER" > "$stamp"
}

# 同时输出到控制台和日志文件；pipefail 保证 cmake 失败能被感知
run_tee() {
    local log_file="$1"; shift
    mkdir -p "$(dirname "$log_file")"
    "$@" 2>&1 | tee "$log_file"
}

cmake_compiler_args() {
    # 参数为 with-c 时同时传入 C 编译器；纯 CXX 工程不要传，
    # 否则 CMake 会提示 "Manually-specified variables were not used"。
    if [ "${1:-}" = "with-c" ]; then
        printf '%s\n' "-DCMAKE_C_COMPILER=${CC_COMPILER}" "-DCMAKE_CXX_COMPILER=${CXX_COMPILER}"
    else
        printf '%s\n' "-DCMAKE_CXX_COMPILER=${CXX_COMPILER}"
    fi
}

# ----------------------------------------------------------------------------
# 第三方库构建
# ----------------------------------------------------------------------------
# 系统只装了 libepoxy0（运行库），没有 libepoxy-dev；Pangolin 在 Linux 上强制
# find_package(epoxy)，且把 HAVE_EPOXY 作为 PUBLIC 编译定义导出，消费方包含
# <pangolin/pangolin.h> 时会间接 #include <epoxy/gl.h>。因此这里做两件事：
#   1) 免 root 解包 libepoxy-dev 到 build/thirdparty/deps/；
#   2) 把头文件与链接库补进 build/thirdparty/install，让预编译前缀自包含。
ensure_epoxy_dev() {
    local dst="${TP_DEPS_DIR}/root"
    local libdir="${dst}/usr/lib/${MULTIARCH}"

    if [ ! -f "${dst}/usr/include/epoxy/gl.h" ] || [ ! -e "${libdir}/libepoxy.so" ]; then
        have apt-get || die "缺少 apt-get，无法补装 libepoxy-dev"
        have dpkg-deb || die "缺少 dpkg-deb，无法解包 libepoxy-dev"

        log "补装构建期依赖：libepoxy-dev（免 root 解包到 ${TP_DEPS_DIR}）"
        mkdir -p "${TP_DEPS_DIR}/deb" "$libdir"
        if ! ( cd "${TP_DEPS_DIR}/deb" && apt-get download libepoxy-dev >/dev/null 2>&1 ); then
            die "下载 libepoxy-dev 失败；请执行 sudo apt install -y libepoxy-dev"
        fi
        local deb
        for deb in "${TP_DEPS_DIR}/deb"/libepoxy-dev_*.deb; do
            [ -e "$deb" ] || continue
            dpkg-deb -x "$deb" "$dst"
        done

        # libepoxy.so -> libepoxy.so.0 的符号链接需要指向真实运行库
        if [ -e "/usr/lib/${MULTIARCH}/libepoxy.so.0" ]; then
            ln -sfn "/usr/lib/${MULTIARCH}/libepoxy.so.0" "${libdir}/libepoxy.so.0"
        else
            ( cd "${TP_DEPS_DIR}/deb" && apt-get download libepoxy0 >/dev/null 2>&1 ) || true
            local rt
            for rt in "${TP_DEPS_DIR}/deb"/libepoxy0_*.deb; do
                [ -e "$rt" ] || continue
                dpkg-deb -x "$rt" "$dst"
            done
        fi
        [ -e "${libdir}/libepoxy.so" ] || die "libepoxy-dev 解包失败"
    fi

    # 让统一安装前缀自包含：消费方链接任意 pango_* 目标即可获得 $PREFIX/include
    mkdir -p "${TP_PREFIX}/include" "${TP_PREFIX}/lib"
    if [ ! -f "${TP_PREFIX}/include/epoxy/gl.h" ]; then
        rm -rf "${TP_PREFIX}/include/epoxy"
        cp -a "${dst}/usr/include/epoxy" "${TP_PREFIX}/include/epoxy"
    fi
    if [ ! -e "${TP_PREFIX}/lib/libepoxy.so" ]; then
        if [ -e "/usr/lib/${MULTIARCH}/libepoxy.so.0" ]; then
            ln -sfn "/usr/lib/${MULTIARCH}/libepoxy.so.0" "${TP_PREFIX}/lib/libepoxy.so.0"
        elif [ -e "${libdir}/libepoxy.so.0" ]; then
            cp -a "${libdir}/libepoxy.so.0" "${TP_PREFIX}/lib/libepoxy.so.0"
        fi
        ln -sfn "libepoxy.so.0" "${TP_PREFIX}/lib/libepoxy.so"
    fi
}

tp_installed_one() {
    case "$1" in
        Livox-SDK2)
            [ -f "${TP_PREFIX}/lib/liblivox_lidar_sdk_static.a" ] \
                && [ -f "${TP_PREFIX}/include/livox_lidar_api.h" ] ;;
        Pangolin)
            [ -f "${TP_PREFIX}/lib/cmake/Pangolin/PangolinConfig.cmake" ] ;;
        matplotlib-cpp)
            [ -f "${TP_PREFIX}/include/matplotlibcpp.h" ] ;;
        *) return 1 ;;
    esac
}

tp_all_installed() {
    local name
    for name in "${TP_LIBS[@]}"; do
        tp_installed_one "$name" || return 1
    done
    return 0
}

tp_build_one() {
    local name="$1"
    local src="${SRC_TP_DIR}/${name}"
    local bdir="${TP_BUILD_DIR}/${name}"
    local log_file="${TP_LOG_DIR}/${name}.log"

    [ -d "$src" ] || die "第三方库源码缺失：${src}
 请先执行：git submodule update --init --recursive"
    [ -f "${src}/CMakeLists.txt" ] || die "第三方库没有顶层 CMakeLists.txt：${src}"

    local -a cfg=("-DCMAKE_BUILD_TYPE=${BUILD_TYPE}" "-DCMAKE_INSTALL_PREFIX=${TP_PREFIX}")
    local -a bt=()
    local compiler_scope="with-c"

    case "$name" in
        Livox-SDK2)
            # Livox 顶层写 cmake_minimum_required(VERSION 3.0)，CMake 4.x 已移除 <3.5 兼容
            cfg+=("-DCMAKE_POLICY_VERSION_MINIMUM=3.5")
            # 只编译 SDK 本体，跳过 examples/samples
            bt=(--target livox_lidar_sdk_static livox_lidar_sdk_shared)
            ;;
        Pangolin)
            ensure_epoxy_dev
            cfg+=("-DBUILD_TOOLS=OFF" "-DBUILD_EXAMPLES=OFF" "-DBUILD_TESTS=OFF")
            # Python 绑定会引入 pybind11 + wheel 依赖，哨兵可视化用不到
            cfg+=("-DBUILD_PANGOLIN_PYTHON=OFF")
            # 用自包含的安装前缀里的 epoxy，导出的 pango_* 目标即可带上 epoxy/gl.h
            cfg+=("-Depoxy_INCLUDE_DIRS=${TP_PREFIX}/include")
            cfg+=("-Depoxy_LIBRARIES=${TP_PREFIX}/lib/libepoxy.so")
            ;;
        matplotlib-cpp)
            # header-only：编译 examples 以验证头文件可用，安装头文件 + CMake 包配置
            compiler_scope="cxx-only"
            ;;
    esac

    if [ "$REBUILD" = "ON" ]; then
        log "--rebuild：清空 ${bdir}"
        rm -rf "$bdir"
    fi
    ensure_build_dir "$bdir"

    log "──────── ${name} ────────"
    log "配置 ${name}（${src} -> ${bdir}）"
    local -a cxxargs
    mapfile -t cxxargs < <(cmake_compiler_args "$compiler_scope")
    if ! run_tee "$log_file" cmake -S "$src" -B "$bdir" "${cfg[@]}" "${cxxargs[@]}"; then
        tail -n 30 "$log_file" >&2 || true
        die "${name} 配置失败，完整日志：${log_file}"
    fi

    log "编译 ${name}（-j ${JOBS}）"
    if ! run_tee "$log_file" cmake --build "$bdir" -j "$JOBS" ${bt[@]+"${bt[@]}"}; then
        tail -n 30 "$log_file" >&2 || true
        die "${name} 编译失败，完整日志：${log_file}"
    fi

    log "安装 ${name} -> ${TP_PREFIX}"
    if ! run_tee "$log_file" cmake --install "$bdir"; then
        tail -n 30 "$log_file" >&2 || true
        die "${name} 安装失败，完整日志：${log_file}"
    fi
    log "${name} 完成"
}

write_tp_env() {
    mkdir -p "$TP_ROOT"
    cat > "$TP_ENV_FILE" <<EOF
# 由 scripts/autoBuild.sh 自动生成 —— 让其他工具找到预编译第三方库
# 用法： source build/thirdparty/env.sh
export PURE_NAV_THIRDPARTY_PREFIX="${TP_PREFIX}"
export CMAKE_PREFIX_PATH="${TP_PREFIX}\${CMAKE_PREFIX_PATH:+:\${CMAKE_PREFIX_PATH}}"
export LD_LIBRARY_PATH="${TP_PREFIX}/lib\${LD_LIBRARY_PATH:+:\${LD_LIBRARY_PATH}}"
export CPATH="${TP_PREFIX}/include\${CPATH:+:\${CPATH}}"
EOF
}

cmd_thirdparty() {
    setup_compilers
    mkdir -p "$TP_BUILD_DIR" "$TP_PREFIX" "$TP_LOG_DIR"

    local name
    for name in "${TP_LIBS[@]}"; do
        tp_build_one "$name"
    done

    write_tp_env
    log "第三方库全部就绪：${TP_PREFIX}"
    log "环境脚本：${TP_ENV_FILE}（source 后即可 find_package）"
}

# ----------------------------------------------------------------------------
# 本项目构建
# ----------------------------------------------------------------------------
cmd_project() {
    setup_compilers
    ensure_build_dir "$BUILD_DIR"

    local -a args=(
        -S "$ROOT_DIR"
        -B "$BUILD_DIR"
        "-DCMAKE_BUILD_TYPE=${BUILD_TYPE}"
        "-DPURE_NAV_BUILD_TESTS=${BUILD_TESTS}"
        "-DPURE_NAV_WARNINGS_AS_ERRORS=${WERROR}"
        "-DPURE_NAV_THIRDPARTY_PREFIX=${TP_PREFIX}"
        "-DCMAKE_PREFIX_PATH=${TP_PREFIX}"
        "-DCMAKE_CXX_COMPILER=${CXX_COMPILER}"
    )

    if ! tp_all_installed; then
        warn "第三方库尚未全部构建，本项目仍可配置；如需接入请先执行：scripts/autoBuild.sh thirdparty"
    fi

    log "配置本项目（${ROOT_DIR} -> ${BUILD_DIR}）"
    if ! cmake "${args[@]}" ${CMAKE_EXTRA[@]+"${CMAKE_EXTRA[@]}"}; then
        die "本项目配置失败"
    fi

    log "编译本项目（-j ${JOBS}）"
    if ! cmake --build "$BUILD_DIR" -j "$JOBS"; then
        die "本项目编译失败"
    fi

    log "本项目构建完成：可执行文件在 ${BIN_DIR}/，静态库在 ${BUILD_DIR}/lib/"
    if [ "$BUILD_TESTS" = "ON" ]; then
        log "运行单元测试：ctest --test-dir ${BUILD_DIR} --output-on-failure"
    fi
}

cmd_all() {
    if [ "$REBUILD" = "ON" ] || ! tp_all_installed; then
        cmd_thirdparty
    else
        log "第三方库已就绪，跳过（需要重建请用：scripts/autoBuild.sh thirdparty --rebuild）"
    fi
    cmd_project
}

# ----------------------------------------------------------------------------
# 清理 / 状态
# ----------------------------------------------------------------------------
cmd_clean() {
    log "删除第三方库构建中间产物：${TP_BUILD_DIR}"
    rm -rf "$TP_BUILD_DIR"
    mkdir -p "$TP_BUILD_DIR"
    log "已保留安装产物：${TP_PREFIX}"
}

cmd_distclean() {
    log "删除全部构建产物（build/ 与 bin/，保留 .gitkeep）"
    find "$BUILD_DIR" -mindepth 1 -maxdepth 1 ! -name '.gitkeep' -exec rm -rf {} + 2>/dev/null || true
    find "$BIN_DIR" -mindepth 1 -maxdepth 1 ! -name '.gitkeep' -exec rm -rf {} + 2>/dev/null || true
    log "完成"
}

cmd_status() {
    printf 'pure-nav 构建状态\n'
    printf '  构建类型      : %s\n' "$BUILD_TYPE"
    printf '  第三方库区域  : %s\n' "$TP_ROOT"
    local name state
    for name in "${TP_LIBS[@]}"; do
        if tp_installed_one "$name"; then state="已安装"; else state="未构建"; fi
        printf '    %-14s %s\n' "$name" "$state"
    done
    printf '  项目构建树    : %s\n' "$BUILD_DIR"
    if [ -f "${BUILD_DIR}/CMakeCache.txt" ]; then
        printf '    CMakeCache    : 存在\n'
    else
        printf '    CMakeCache    : 不存在（尚未配置）\n'
    fi
    if [ -d "${BUILD_DIR}/lib" ]; then
        printf '    静态库        :\n'
        find "${BUILD_DIR}/lib" -maxdepth 1 -name '*.a' -printf '      %f\n' 2>/dev/null || true
    fi
    if [ -d "$BIN_DIR" ]; then
        printf '    可执行文件    :\n'
        find "$BIN_DIR" -maxdepth 1 -type f -executable -printf '      %f\n' 2>/dev/null || true
    fi
}

# ----------------------------------------------------------------------------
# 入口
# ----------------------------------------------------------------------------
parse_args "$@"

case "$COMMAND" in
    all)        cmd_all ;;
    thirdparty) cmd_thirdparty ;;
    project)    cmd_project ;;
    clean)      cmd_clean ;;
    distclean)  cmd_distclean ;;
    status)     cmd_status ;;
    help)       usage ;;
    *)          die "未知命令：$COMMAND" ;;
esac
