#! /usr/bin/env bash
set -e

TOP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
# source "source devel/setup.bash"

ARCH="$(uname -m)"
SUPPORTED_ARCHS=" x86_64 aarch64 "
APOLLO_VERSION="@non-git"
APOLLO_ENV=""

USE_ESD_CAN=false
: ${STAGE:=dev}

AVAILABLE_COMMANDS="build build_cpu build_gpu usage -h --help"

BOLD='\033[1m'
RED='\033[0;31m'
GREEN='\033[32m'
WHITE='\033[34m'
YELLOW='\033[33m'
NO_COLOR='\033[0m'

function check_architecture_support() {
    if [[ "${SUPPORTED_ARCHS}" != *" ${ARCH} "* ]]; then
        error "Unsupported CPU arch: ${ARCH}. Currently, Apollo only" \
            "supports running on the following CPU archs:"
        error "${TAB}${SUPPORTED_ARCHS}"
        exit 1
    fi
}

function check_platform_support() {
    local platform="$(uname -s)"
    if [[ "${platform}" != "Linux" ]]; then
        error "Unsupported platform: ${platform}."
        error "${TAB}Apollo is expected to run on Linux systems (E.g., Debian/Ubuntu)."
        exit 1
    fi
}

function check_minimal_memory_requirement() {
    local minimal_mem_gb="2.0"
    local actual_mem_gb="$(free -m | awk '/Mem:/ {printf("%0.2f", $2 / 1024.0)}')"
    if (($(echo "$actual_mem_gb < $minimal_mem_gb" | bc -l))); then
        warning "System memory [${actual_mem_gb}G] is lower than the minimum required" \
            "[${minimal_mem_gb}G]. Apollo build could fail."
    fi
}

function determine_esdcan_use() {
    local esdcan_dir="${APOLLO_ROOT_DIR}/third_party/can_card_library/esd_can"
    local use_esd=false
    if [[ "${ARCH}" == "x86_64" ]] &&
        [[ -f "${esdcan_dir}/include/ntcan.h" ]] &&
        [[ -f "${esdcan_dir}/lib/libntcan.so.4" ]]; then
        use_esd=true
    fi
    USE_ESD_CAN="${use_esd}"
}

function check_apollo_version() {
    local branch="$(git_branch)"
    if [ "${branch}" == "${APOLLO_VERSION}" ]; then
        return
    fi
    local sha1="$(git_sha1)"
    local stamp="$(git_date)"
    APOLLO_VERSION="${branch}-${stamp}-${sha1}"
}

function apollo_env_setup() {
    check_apollo_version

    check_architecture_support
    check_platform_support
    check_minimal_memory_requirement
    determine_gpu_use_target
    determine_esdcan_use

    APOLLO_ENV="${APOLLO_ENV} STAGE=${STAGE}"
    APOLLO_ENV="${APOLLO_ENV} USE_ESD_CAN=${USE_ESD_CAN}"
    # Add more here ...

    info "Apollo Environment Settings:"
    info "${TAB}APOLLO_ROOT_DIR: ${APOLLO_ROOT_DIR}"
    info "${TAB}APOLLO_CACHE_DIR: ${APOLLO_CACHE_DIR}"
    info "${TAB}APOLLO_IN_DOCKER: ${APOLLO_IN_DOCKER}"
    info "${TAB}APOLLO_VERSION: ${APOLLO_VERSION}"
    if "${APOLLO_IN_DOCKER}"; then
        info "${TAB}DOCKER_IMG: ${DOCKER_IMG##*:}"
    fi
    info "${TAB}APOLLO_ENV: ${APOLLO_ENV}"
    info "${TAB}USE_GPU: USE_GPU_HOST=${USE_GPU_HOST} USE_GPU_TARGET=${USE_GPU_TARGET}"

    if [[ -z "${APOLLO_BAZEL_DIST_DIR}" ]]; then
        source "${TOP_DIR}/cyber/setup.bash"
    fi
    if [[ ! -d "${APOLLO_BAZEL_DIST_DIR}" ]]; then
        mkdir -p "${APOLLO_BAZEL_DIST_DIR}"
    fi

    if [ ! -f "${APOLLO_ROOT_DIR}/.apollo.bazelrc" ]; then
        env ${APOLLO_ENV} bash "${APOLLO_ROOT_DIR}/scripts/apollo_config.sh" --noninteractive
    fi
}

#TODO(all): Update node modules
function build_dreamview_frontend() {
    pushd "${APOLLO_ROOT_DIR}/modules/dreamview/frontend" >/dev/null
    yarn build
    popd >/dev/null
}

function build_test_and_lint() {
    env ${APOLLO_ENV} bash "${build_sh}"
    env ${APOLLO_ENV} bash "${test_sh}" --config=unit_test
    env ${APOLLO_ENV} bash "${APOLLO_ROOT_DIR}/scripts/apollo_lint.sh" --cpp
    success "Build and Test and Lint finished."
}

function _usage() {
    echo -e "\n${RED}Usage${NO_COLOR}:
    .${BOLD}/apollo.sh${NO_COLOR} [OPTION]"
    echo -e "\n${RED}Options${NO_COLOR}:
    ${BLUE}build [module]${NO_COLOR}: run build for cyber (<module> = cyber) or modules/<module>.  If <module> unspecified, build all.
    ${BLUE}build_cpu [module]${NO_COLOR}: build in CPU mode. Equivalent to 'bazel build --config=cpu'
    ${BLUE}build_gpu [module]${NO_COLOR}: run build in GPU mode. Equivalent to 'bazel build --config=gpu'
    ${BLUE}usage${NO_COLOR}: show this message and exit
    "
}

function _check_command() {
    local name="${BASH_SOURCE[0]}"
    local commands="$(echo ${AVAILABLE_COMMANDS} | xargs)"
    local help_msg="Run './apollo.sh --help' for usage."
    local cmd="$@"

    python scripts/command_checker.py --name "${name}" --command "${cmd}" --available "${commands}" --helpmsg "${help_msg}"
}

function build_cpu() {
    catkin_make -DUSE_GPU=${USE_GPU} -DNO_GPU=true
}

function build_gpu() {
    catkin_make -DUSE_GPU=${USE_GPU}
}

function build() {
    if [ "$USE_GPU" == "1" ]; then
        echo -e "${YELLOW}Running build under GPU mode. GPU is required to run the build.${NO_COLOR}"
        build_gpu
    else
        echo -e "${YELLOW}Running build under CPU mode. No GPU is required to run the build.${NO_COLOR}"
        build_cpu
    fi
}

function main() {
    if [ "$#" -eq 0 ]; then
        _usage
        exit 0
    fi

    local cmd="$1"
    shift
    case "${cmd}" in
        build)
            build
            ;;
        build_cpu)
            build_cpu
            ;;
        build_gpu)
            build_gpu
            ;;
        usage)
            _usage
            ;;
        -h|--help)
            _usage
            ;;
        *)
            _check_command "${cmd}"
            ;;
    esac
}

main "$@"
