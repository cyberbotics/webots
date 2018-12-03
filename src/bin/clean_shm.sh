#!/bin/bash
#system cleanup for every shared memory segment or semaphore the user has permission to


if [ "$(uname)" == "Darwin" ]; then
  for n in `ipcs -b -m | egrep ^m | awk '{ print $2; }'`; do ipcrm -m $n; done
  for n in `ipcs -b -s | egrep ^s | awk '{ print $2; }'`; do ipcrm -s $n; done
elif [ "$(expr substr $(uname -s) 1 5)" == "Linux" ]; then
  for i in `ipcs -m | tail -n +4 | awk '{print $2}'`
  do
      ipcrm shm $i
  done
  for i in `ipcs -s | tail -n +4 | awk '{print $2}'`
  do
      ipcrm sem $i
  done
else
  echo "Unsupported OS"
  exit 1
fi
