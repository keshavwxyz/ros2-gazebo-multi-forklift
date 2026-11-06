#!/usr/bin/env bash
# Keep a local copy of the ROS package.xml schemas and register them in an XML catalog.
# Thus, `ament_xmllint` (libxml2) can validate package.xml without network access.
#
# Reason: on networks with a defective IPv6 connection, libxml2 tries each AAAA record of
# download.ros.org before IPv4. Each try takes approximately 60 s. Thus, the lint test takes
# more time than the 60 s timeout of colcon. An alternative for the full system is to set
# IPv4 as the preferred protocol in /etc/gai.conf.
#
# Usage: source scripts/env.sh   (this command runs this script the first time)
set -euo pipefail
CACHE_DIR="${XDG_CACHE_HOME:-$HOME/.cache}/forklift_autonomy/xml_catalog"
mkdir -p "$CACHE_DIR"
for fmt in 2 3; do
  xsd="$CACHE_DIR/package_format${fmt}.xsd"
  if [[ ! -s "$xsd" ]]; then
    curl -4 -fsS --max-time 30 -o "$xsd" "http://download.ros.org/schema/package_format${fmt}.xsd"
  fi
done
cat > "$CACHE_DIR/catalog.xml" <<CAT
<?xml version="1.0"?>
<catalog xmlns="urn:oasis:names:tc:entity:xmlns:xml:catalog">
  <rewriteURI uriStartString="http://download.ros.org/schema/" rewritePrefix="file://$CACHE_DIR/"/>
  <rewriteSystem systemIdStartString="http://download.ros.org/schema/" rewritePrefix="file://$CACHE_DIR/"/>
</catalog>
CAT
echo "$CACHE_DIR/catalog.xml"
