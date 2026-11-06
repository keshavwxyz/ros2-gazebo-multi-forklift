#!/usr/bin/env bash
# Source this file from the workspace root:  source scripts/env.sh
# This file sets up ROS 2 Jazzy, the workspace overlay (if you built it) and the dev tools.
_ws_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
[[ -f "$_ws_root/install/setup.bash" ]] && source "$_ws_root/install/setup.bash"
if _cat="$("$_ws_root/scripts/setup_xml_catalog.sh" 2>/dev/null)"; then
  export XML_CATALOG_FILES="$_cat${XML_CATALOG_FILES:+ $XML_CATALOG_FILES}"
else
  echo "[env.sh] warning: could not cache package.xml schemas; xmllint tests may be slow" >&2
fi
unset _ws_root _cat
