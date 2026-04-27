#!/bin/sh
set -eu

WEBOTS_HOME=${WEBOTS_HOME:?}
WEBOTS_CONTROLLER_LIB_PATH=${WEBOTS_CONTROLLER_LIB_PATH:-"$WEBOTS_HOME/lib/controller"}
WEBOTS_LIB_PATH=${WEBOTS_LIB_PATH:-"$WEBOTS_HOME/lib/webots"}
WEBOTS_CONTROLLER_C_LIB=${WEBOTS_CONTROLLER_C_LIB:-}
WEBOTS_CONTROLLER_CPP_LIB=${WEBOTS_CONTROLLER_CPP_LIB:-}
WEBOTS_QT_VERSION=${WEBOTS_QT_VERSION:-}
WEBOTS_QT_INCLUDE_DIR=${WEBOTS_QT_INCLUDE_DIR:-}
WEBOTS_QT_LIB_DIR=${WEBOTS_QT_LIB_DIR:-}
WEBOTS_QT_LIBRARIES=${WEBOTS_QT_LIBRARIES:-}
WEBOTS_QT_CORE_LIBRARIES=${WEBOTS_QT_CORE_LIBRARIES:-}
WEBOTS_QT_MOC=${WEBOTS_QT_MOC:-}

if [ -n "$WEBOTS_CONTROLLER_C_LIB" ] || [ -n "$WEBOTS_CONTROLLER_CPP_LIB" ]; then
  mkdir -p "$WEBOTS_CONTROLLER_LIB_PATH"
fi
if [ -n "$WEBOTS_CONTROLLER_C_LIB" ] && [ -f "$WEBOTS_CONTROLLER_C_LIB" ]; then
  cp -f "$WEBOTS_CONTROLLER_C_LIB" "$WEBOTS_CONTROLLER_LIB_PATH/"
fi
if [ -n "$WEBOTS_CONTROLLER_CPP_LIB" ] && [ -f "$WEBOTS_CONTROLLER_CPP_LIB" ]; then
  cp -f "$WEBOTS_CONTROLLER_CPP_LIB" "$WEBOTS_CONTROLLER_LIB_PATH/"
fi

if [ -z "$WEBOTS_QT_VERSION" ]; then
  if command -v moc-qt6 >/dev/null 2>&1; then
    WEBOTS_QT_VERSION=6
  elif command -v moc-qt5 >/dev/null 2>&1; then
    WEBOTS_QT_VERSION=5
  elif command -v moc >/dev/null 2>&1; then
    moc_major=$(moc -v 2>&1 | awk '{print $2}' | cut -d. -f1 || true)
    if [ "$moc_major" = "5" ]; then
      WEBOTS_QT_VERSION=5
    elif [ "$moc_major" = "6" ]; then
      WEBOTS_QT_VERSION=6
    fi
  fi
fi
if [ -z "$WEBOTS_QT_VERSION" ] && command -v pkg-config >/dev/null 2>&1; then
  if pkg-config --exists Qt6Core 2>/dev/null; then
    WEBOTS_QT_VERSION=6
  elif pkg-config --exists Qt5Core 2>/dev/null; then
    WEBOTS_QT_VERSION=5
  fi
fi
if [ -z "$WEBOTS_QT_INCLUDE_DIR" ] && command -v pkg-config >/dev/null 2>&1; then
  if [ "$WEBOTS_QT_VERSION" = "6" ]; then
    WEBOTS_QT_INCLUDE_DIR=$(pkg-config --variable=includedir Qt6Core 2>/dev/null || true)
    WEBOTS_QT_LIB_DIR=$(pkg-config --variable=libdir Qt6Core 2>/dev/null || true)
  elif [ "$WEBOTS_QT_VERSION" = "5" ]; then
    WEBOTS_QT_INCLUDE_DIR=$(pkg-config --variable=includedir Qt5Core 2>/dev/null || true)
    WEBOTS_QT_LIB_DIR=$(pkg-config --variable=libdir Qt5Core 2>/dev/null || true)
  fi
fi
if [ -z "$WEBOTS_QT_INCLUDE_DIR" ]; then
  if [ "$WEBOTS_QT_VERSION" = "5" ] && [ -d /usr/include/qt5 ]; then
    WEBOTS_QT_INCLUDE_DIR=/usr/include/qt5
  elif [ -d /usr/include/qt6 ]; then
    WEBOTS_QT_INCLUDE_DIR=/usr/include/qt6
  elif [ -d /usr/include/qt ]; then
    WEBOTS_QT_INCLUDE_DIR=/usr/include/qt
  fi
fi
if [ -z "$WEBOTS_QT_LIB_DIR" ]; then
  if [ "$WEBOTS_QT_VERSION" = "5" ] && [ -d /usr/lib/qt5 ]; then
    WEBOTS_QT_LIB_DIR=/usr/lib/qt5
  elif [ -d /usr/lib/qt6 ]; then
    WEBOTS_QT_LIB_DIR=/usr/lib/qt6
  elif [ -d /usr/lib64/qt6 ]; then
    WEBOTS_QT_LIB_DIR=/usr/lib64/qt6
  fi
fi
if [ -z "$WEBOTS_QT_LIBRARIES" ]; then
  if [ "$WEBOTS_QT_VERSION" = "5" ]; then
    WEBOTS_QT_LIBRARIES='-lQt5Core -lQt5Gui -lQt5Widgets'
    WEBOTS_QT_CORE_LIBRARIES=${WEBOTS_QT_CORE_LIBRARIES:-'-lQt5Core'}
  else
    WEBOTS_QT_LIBRARIES='-lQt6Core -lQt6Gui -lQt6Widgets'
    WEBOTS_QT_CORE_LIBRARIES=${WEBOTS_QT_CORE_LIBRARIES:-'-lQt6Core'}
  fi
fi
if [ -z "$WEBOTS_QT_MOC" ]; then
  if [ "$WEBOTS_QT_VERSION" = "5" ] && command -v moc-qt5 >/dev/null 2>&1; then
    WEBOTS_QT_MOC=$(command -v moc-qt5)
  elif [ "$WEBOTS_QT_VERSION" = "6" ] && command -v moc-qt6 >/dev/null 2>&1; then
    WEBOTS_QT_MOC=$(command -v moc-qt6)
  elif command -v moc >/dev/null 2>&1; then
    WEBOTS_QT_MOC=$(command -v moc)
  fi
fi

build_makefile() {
  make -C "$1" WEBOTS_HOME="$WEBOTS_HOME" \
    WEBOTS_CONTROLLER_LIB_PATH="$WEBOTS_CONTROLLER_LIB_PATH" \
    WEBOTS_LIB_PATH="$WEBOTS_LIB_PATH" \
    ${WEBOTS_QT_INCLUDE_DIR:+QT_INCLUDE_DIR="$WEBOTS_QT_INCLUDE_DIR"} \
    ${WEBOTS_QT_LIB_DIR:+QT_LIB_DIR="$WEBOTS_QT_LIB_DIR"} \
    ${WEBOTS_QT_LIBRARIES:+QT_LIBRARIES="$WEBOTS_QT_LIBRARIES"} \
    ${WEBOTS_QT_CORE_LIBRARIES:+QT_CORE_LIBRARIES="$WEBOTS_QT_CORE_LIBRARIES"} \
    ${WEBOTS_QT_MOC:+MOC="$WEBOTS_QT_MOC"}
}

build_makefile_controller() {
  make -C "$1" WEBOTS_HOME="$WEBOTS_HOME" \
    WEBOTS_CONTROLLER_LIB_PATH="$WEBOTS_CONTROLLER_LIB_PATH" \
    WEBOTS_LIB_PATH="$WEBOTS_LIB_PATH" \
    MAIN_TARGET_COPY='$(MAIN_TARGET)' \
    ${WEBOTS_QT_INCLUDE_DIR:+QT_INCLUDE_DIR="$WEBOTS_QT_INCLUDE_DIR"} \
    ${WEBOTS_QT_LIB_DIR:+QT_LIB_DIR="$WEBOTS_QT_LIB_DIR"} \
    ${WEBOTS_QT_LIBRARIES:+QT_LIBRARIES="$WEBOTS_QT_LIBRARIES"} \
    ${WEBOTS_QT_CORE_LIBRARIES:+QT_CORE_LIBRARIES="$WEBOTS_QT_CORE_LIBRARIES"} \
    ${WEBOTS_QT_MOC:+MOC="$WEBOTS_QT_MOC"}
}

fail=0
tmp_list=$(mktemp)

find "$WEBOTS_HOME/resources/projects" "$WEBOTS_HOME/projects" -type f -name Makefile -path "*/libraries/*/Makefile" > "$tmp_list"
while IFS= read -r makefile; do
  dir=$(dirname "$makefile")
  echo "# building library in $dir"
  if ! build_makefile "$dir"; then
    echo "# library build failed in $dir" >&2
    fail=1
  fi
done < "$tmp_list"

find "$WEBOTS_HOME/resources/projects" "$WEBOTS_HOME/projects" -type f -name Makefile -path "*/controllers/*/Makefile" > "$tmp_list"
while IFS= read -r makefile; do
  dir=$(dirname "$makefile")
  echo "# building controller in $dir"
  if ! build_makefile_controller "$dir"; then
    echo "# controller build failed in $dir" >&2
    fail=1
  fi
done < "$tmp_list"
rm -f "$tmp_list"

if [ "${WEBOTS_CONTROLLERS_STRICT:-0}" = "1" ] && [ "$fail" -ne 0 ]; then
  exit 1
fi
