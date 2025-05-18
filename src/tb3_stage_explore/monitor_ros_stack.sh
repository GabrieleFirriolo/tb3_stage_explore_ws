#!/bin/bash

echo "[INFO] Scanning active ROS 2 node processes..."

# Cerca i processi principali ROS 2
PIDS=$(ps -e -o pid,cmd | grep -E 'stage_ros2|gmap|component_container_isolated|explore' | grep -v grep | awk '{print $1}' | paste -sd,)

if [ -z "$PIDS" ]; then
  echo "[ERROR] Nessun processo ROS2 trovato da monitorare."
  exit 1
fi

echo "[INFO] Processi trovati con PID: $PIDS"

# Crea file di log con timestamp
TIMESTAMP=$(date +"%Y%m%d_%H%M")
LOGFILE="ros2_perf_$TIMESTAMP.log"

echo "[INFO] Logging performance in $LOGFILE..."
echo "Premi CTRL+C per interrompere"

# Esegui il monitoraggio
pidstat -h -r -u -p "$PIDS" 1 > "$LOGFILE"
