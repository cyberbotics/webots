#!/bin/bash

if [[ $# -lt 1 ]] || [[ $1 != "start" && $1 != "restart" && $1 != "stop" ]] ; then
  echo "Start, restart or stop simulation and session servers."
  echo "Usage: server.sh COMMAND [CONFIG]"
  echo "COMMAND: either 'start', 'restart' or 'stop'"
  echo "CONFIG:  refer to config/*/<CONFIG>.json files"
  echo "         not needed for 'stop' command"
  echo "         default to 'local' if not specified"
  echo "Examples:"
  echo "./server.sh start               -- start local session and simulation servers"
  echo "./server.sh start cyberbotics1  -- start specific session and simulation servers"
  echo "./server.sh start cyberbotics2  -- start only specific simulation server"
  echo "./server.sh restart local       -- restart local session and simulation servers"
  echo "./server.sh stop                -- stop all session and simulation servers"
  exit
fi

SESSION_PID="log/.session.pid"
SIMULATION_PID="log/.simulation.pid"

if [[ $1 == "restart" || $1 == "stop" ]] ; then
  if [ -e $SESSION_PID ] ; then
    if kill $(<$SESSION_PID) ; then
      echo "Stopped session server ($(<$SESSION_PID))"
    fi
    rm $SESSION_PID
  fi
  if [ -e $SIMULATION_PID ] ; then
    if kill $(<$SIMULATION_PID) ; then
      echo "Stopped simulation server ($(<$SIMULATION_PID))"
    fi
    rm $SIMULATION_PID
  fi
fi

if [[ $1 == "start" || $1 == "restart" ]] ; then
  if [[ $# -lt 2 ]] ; then
    CONFIG="local"
  else
    CONFIG=$2
  fi
  SESSION_CONFIG="config/session/$CONFIG.json"
  SIMULATION_CONFIG="config/simulation/$CONFIG.json"
  if [ -e $SESSION_CONFIG ] ; then
    ./session_server.py $SESSION_CONFIG >/dev/null &
    echo $! > $SESSION_PID
    echo "Started $CONFIG session server ($!)"
  fi
  if [ -e $SIMULATION_CONFIG ] ; then
    ./simulation_server.py $SIMULATION_CONFIG >/dev/null &
    echo $! > $SIMULATION_PID
    echo "Started $CONFIG simulation server ($!)"
  fi
fi

exit
