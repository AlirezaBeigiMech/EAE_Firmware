#!/usr/bin/env bash
# run_kunit_uml.sh — Run NodeB KUnit tests under UML (no host kernel rebuild)
# Default source path set to your repo:
#   ~/Documents/Interview_ECE/CAN_PID/CAN_Simulation/controller
#
# Usage (defaults are usually fine now):
#   ./run_kunit_uml.sh
# Or customize:
#   ./run_kunit_uml.sh -k ./linux-kunit -b v6.11 -t 180 -j 8 --no-clone

set -euo pipefail

# -------- Defaults (tweaked for your setup) --------
SRC_REPO="$(pwd)/controller"
KERNEL_DIR="./linux-kunit"
KERNEL_BRANCH="master"
TIMEOUT="120"
JOBS="$(nproc)"
DO_CLONE=1

# -------- Optional cleanup --------
if [[ "${1:-}" == "--clean" ]]; then
  echo "[i] Removing temporary kernel folder: ${KERNEL_DIR}"
  rm -rf "${KERNEL_DIR}"
  echo "[i] Cleanup complete."
  exit 0
fi

# -------- Parse args --------
while [[ $# -gt 0 ]]; do
  case "$1" in
    -s|--source) SRC_REPO="$2"; shift 2;;
    -k|--kernel) KERNEL_DIR="$2"; shift 2;;
    -b|--branch) KERNEL_BRANCH="$2"; shift 2;;
    -t|--timeout) TIMEOUT="$2"; shift 2;;
    -j|--jobs) JOBS="$2"; shift 2;;
    --no-clone) DO_CLONE=0; shift 1;;
    -h|--help)
      cat <<EOF
Usage: $0 [options]

Options:
  -s, --source   Path to your OOT repo (default: ${SRC_REPO})
  -k, --kernel   Path to Linux checkout for UML runs (default: ${KERNEL_DIR})
  -b, --branch   Kernel branch/tag (default: ${KERNEL_BRANCH})
  -t, --timeout  KUnit run timeout seconds (default: ${TIMEOUT})
  -j, --jobs     Parallel build jobs (default: $(nproc))
  --no-clone     Reuse existing kernel checkout; don't fetch/clone
  -h, --help     Show this help
EOF
      exit 0;;
    *) echo "Unknown arg: $1" >&2; exit 1;;
  esac
done



# -------- Verify required files exist in your repo --------
req=(
  "controller_kernel.c"
  "tests/nodeb_test_hooks.h"
  "tests/nodeb_kunit_test.c"
)
for f in "${req[@]}"; do
  if [[ ! -f "${SRC_REPO}/${f}" ]]; then
    echo "ERROR: Missing ${f} in ${SRC_REPO}" >&2
    echo "       Expected layout:" >&2
    printf "         %s\n" "${req[@]}" >&2
    exit 1
  fi
done

# -------- Light dependency checks --------
need() { command -v "$1" >/dev/null 2>&1 || { echo "Missing '$1' in PATH"; exit 1; }; }
need git
need python3
need make

# -------- Clone or reuse kernel --------
if [[ ${DO_CLONE} -eq 1 ]]; then
  if [[ -d "${KERNEL_DIR}/.git" ]]; then
    echo "[i] Kernel dir exists: ${KERNEL_DIR} — pulling ${KERNEL_BRANCH}"
    git -C "${KERNEL_DIR}" fetch --depth=1 origin "${KERNEL_BRANCH}"
    git -C "${KERNEL_DIR}" checkout -f FETCH_HEAD
  else
    echo "[i] Cloning Linux into ${KERNEL_DIR} (branch: ${KERNEL_BRANCH})"
    git clone --depth=1 --branch "${KERNEL_BRANCH}" https://git.kernel.org/pub/scm/linux/kernel/git/torvalds/linux.git "${KERNEL_DIR}"
  fi
else
  [[ -d "${KERNEL_DIR}/.git" ]] || { echo "ERROR: --no-clone but ${KERNEL_DIR} is not a git repo" >&2; exit 1; }
  echo "[i] Reusing existing kernel at ${KERNEL_DIR}"
fi

# -------- Stage your sources in-tree --------
DST_DIR="${KERNEL_DIR}/drivers/misc/nodeb"
DST_TESTS="${DST_DIR}/tests"
mkdir -p "${DST_TESTS}"

echo "[i] Copying sources from: ${SRC_REPO}"
cp -v "${SRC_REPO}/controller_kernel.c" "${DST_DIR}/"
cp -v "${SRC_REPO}/tests/nodeb_test_hooks.h"   "${DST_DIR}/"
cp -v "${SRC_REPO}/tests/nodeb_kunit_test.c" "${DST_TESTS}/"

# -------- Write Makefile & Kconfig for the driver/tests --------
cat > "${DST_DIR}/Makefile" <<'EOF'
# drivers/misc/nodeb/Makefile
obj-$(CONFIG_NODEB) += controller_kernel.o
obj-$(CONFIG_NODEB_KUNIT_TEST) += tests/nodeb_kunit_test.o
ccflags-y += -Wno-unused-function
EOF

cat > "${DST_DIR}/Kconfig" <<'EOF'
# drivers/misc/nodeb/Kconfig
config NODEB
    tristate "NodeB demo driver (controller)"
    default y

config NODEB_KUNIT_TEST
    tristate "NodeB KUnit tests"
    depends on KUNIT
    default y
EOF

# Ensure Kconfig is included by drivers/misc/Kconfig
MISC_KCONFIG="${KERNEL_DIR}/drivers/misc/Kconfig"
if ! grep -q 'source "drivers/misc/nodeb/Kconfig"' "${MISC_KCONFIG}"; then
  echo '[i] Patching drivers/misc/Kconfig to include nodeb/Kconfig'
  echo 'source "drivers/misc/nodeb/Kconfig"' >> "${MISC_KCONFIG}"
fi

# -------- Create .kunitconfig --------
cat > "${KERNEL_DIR}/.kunitconfig" <<EOF
CONFIG_KUNIT=y
CONFIG_KUNIT_DEBUGFS=y
CONFIG_UML=y

# Our driver + tests
CONFIG_NODEB=y
CONFIG_NODEB_KUNIT_TEST=y
EOF

# -------- Run KUnit under UML --------
echo "[i] Running KUnit under UML (timeout=${TIMEOUT}s, jobs=${JOBS})"
cd "${KERNEL_DIR}"
python3 ./tools/testing/kunit/kunit.py run --timeout="${TIMEOUT}" --jobs="${JOBS}"

