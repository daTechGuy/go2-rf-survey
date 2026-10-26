#!/usr/bin/env bash
# Stands in for tools/preflight.sh: exit 0 motion-ready, 1 sensing-only, 2 not ready.
rc="${FAKE_PREFLIGHT_RC:-0}"
echo "== robot (fake)"
case "$rc" in
  0) echo "sensing: READY   motion: READY (after you confirm breathing)" ;;
  1) echo "sensing: READY   motion: NOT READY" ;;
  *) echo "sensing: NOT READY   motion: NOT READY" ;;
esac
exit "$rc"
