#!/usr/bin/env bash
# Stands in for docker/run.sh in the sandbox copy of the repo.
cmd="$*"
SURV="$HOME/surveys"
mkdir -p "$SURV" "$FAKE/containers"
ev() { echo "$(date +%s.%N) $*" >> "$FAKE/events.log"; }

if [ -n "${DETACH:-}" ]; then
  if [ -f "$FAKE/fail_$DETACH" ]; then ev "start-failed $DETACH"; exit 1; fi
  : > "$FAKE/containers/$DETACH"
  ev "start $DETACH"
  case "$DETACH" in
    go2loop)
      dry=true; [[ "$cmd" == *"dry_run:=false"* ]] && dry=false
      python "$FAKE_LIB_W/fakeloop.py" reset "$dry"
      echo "go2_loop up (fake), dry_run=$dry" > "$SURV/go2loop.log" ;;
    go2remote)
      args=$(echo "$cmd" | sed -n 's#.*remote_trigger.py\(.*\) > /surveys/go2remote.log.*#\1#p')
      # shellcheck disable=SC2086
      PYTHONPATH="$FAKE_LIB_W" python -u "$REPO_SB/tools/remote_trigger.py" $args \
        > "$SURV/go2remote.log" 2>&1 &
      echo "$!" > "$FAKE/containers/$DETACH" ;;
    go2rec)
      bag=$(echo "$cmd" | sed -n 's#.*-o /surveys/\([^ ]*\) .*#\1#p')
      mkdir -p "$SURV/$bag"; echo fake > "$SURV/$bag/${bag}_0.db3" ;;
  esac
  exit 0
fi

case "$cmd" in
  *bag_to_map.py*)
    bag=$(basename "$(echo "$cmd" | awk '{print $NF}')")
    ev "floorplan $bag"
    echo "fake" > "$SURV/${bag}_map-topdown.png"
    echo "wrote /surveys/${bag}_map-topdown.png" ;;
  *) echo "run.sh $cmd" >> "$FAKE/unhandled" ;;
esac
