#!/bin/bash

# Lista dei topic
TOPICS=(
  "/odom"
  "/map"
  "/tf"
  "/tf_static"
  "/base_scan"
  "/cmd_vel"
  "/cmd_vel_nav"
)
monitor_topic() {
  local topic=$1
  if ros2 topic info "$topic" | grep -q "Publisher count: 0"; then
    printf "%-28s | %s\n" "$topic" "🔴 NO PUBLISHER"
  else
    hz_line=$(timeout 5s ros2 topic hz "$topic" 2>/dev/null | grep "average rate:" | tail -n 1)
    hz=$(echo "$hz_line" | awk '{print int($3)}' 2>/dev/null) 
    if [ -z "$hz" ]; then
      printf "%-28s | %s\n" "$topic" "🟠 N/A (No data)"
    else
      if [ "$hz" -ge 15 ]; then
        color="🟢"
      elif [ "$hz" -ge 5 ]; then
        color="🟡"
      else
        color="🔴"
      fi
      printf "%-28s | %s %d Hz\n" "$topic" "$color" "$hz"
    fi
  fi
}



echo ""
echo "TOPIC                        | HZ        "
echo "-----------------------------+-----------"

while true; do
  for topic in "${TOPICS[@]}"; do
    monitor_topic "$topic"
  done
  echo "------------------------------------------"
  sleep 10
done