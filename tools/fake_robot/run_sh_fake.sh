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
      planner=loop; [[ "$cmd" == *"planner:=cmu"* ]] && planner=cmu
      python "$FAKE_LIB_W/fakeloop.py" reset "$dry"
      echo "go2_loop up (fake), dry_run=$dry, planner=$planner" > "$SURV/go2loop.log" ;;
    go2remote)
      args=$(echo "$cmd" | sed -n 's#.*remote_trigger.py\(.*\) > /surveys/go2remote.log.*#\1#p')
      # shellcheck disable=SC2086
      PYTHONPATH="$FAKE_LIB_W" python -u "$REPO_SB/tools/remote_trigger.py" $args \
        > "$SURV/go2remote.log" 2>&1 &
      echo "$!" > "$FAKE/containers/$DETACH" ;;
    go2rec)
      bag=$(echo "$cmd" | sed -n 's#.*-o /surveys/\([^ ]*\) .*#\1#p')
      mkdir -p "$SURV/$bag"; echo fake > "$SURV/$bag/${bag}_0.db3" ;;
    go2wifi)
      wiface=$(echo "$cmd" | sed -n 's#.*interface:=\([^ ]*\).*#\1#p')
      wcsv=$(echo "$cmd" | sed -n 's#.*output_csv:=/surveys/\([^ >]*\).*#\1#p')
      printf "timestamp,x,y,z,frame_id,bssid,ssid,freq_mhz,band,rssi_dbm\n1.0,0.0,0.0,0.0,odom,aa:bb:cc:dd:ee:01,CampusWiFi,5180,5,-52.0\n" > "$SURV/$wcsv"
      echo "scanning on $wiface every 1.0 s (min move 0.5 m) -> /surveys/$wcsv" > "$SURV/go2wifi.log" ;;
  esac
  exit 0
fi

case "$cmd" in
  *bag_to_map.py*)
    bag=$(basename "$(echo "$cmd" | awk '{print $NF}')")
    ev "floorplan $bag"
    echo "fake" > "$SURV/${bag}_map-topdown.png"
    echo "image: ${bag}_map.pgm" > "$SURV/${bag}_map.yaml"
    echo "wrote /surveys/${bag}_map-topdown.png" ;;
  *heatmap.py*)
    csv=$(echo "$cmd" | sed -n 's#.*heatmap.py[[:space:]]*\([^ ]*\).*#\1#p')
    stem=$(basename "${csv%.csv}")
    ev "heatmap $stem"
    echo "fake png" > "$SURV/${stem}-heatmap.png"
    echo "  12 readings, 2 SSID(s), 4 position(s) -> /surveys/${stem}-heatmap.png" ;;
  *go2_survey_wifi.scanner*)
    ev "wifi-check"
    echo "2 BSS(es) on wlan0" ;;
  *front_camera.py*--once*)
    # go2loop.sh camera-snap / camera-check: one still (demo.sh's report pictures)
    shot=$(echo "$cmd" | sed -n 's#.*--once /surveys/\([^ "]*\).*#\1#p')
    ev "camera $shot"
    [ -f "$FAKE/no_camera" ] || echo "fake jpeg" > "$SURV/$shot" ;;
  *cmu_ctl.py*)
    sub=$(echo "$cmd" | sed -n 's#.*cmu_ctl.py[[:space:]]*\(.*\)#\1#p')
    ev "cmu $sub"
    if [[ "$sub" == check* ]] && [ -f "$FAKE/fail_cmu" ]; then
      echo "CMU check: NOT READY -- /way_point has 0 subscribers"; exit 1
    fi
    echo "CMU $sub OK (fake)" ;;
  *) echo "run.sh $cmd" >> "$FAKE/unhandled" ;;
esac
