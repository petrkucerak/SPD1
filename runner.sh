#!/bin/bash

python3 gateway/lora_server.py &
PID1=$!


cd server && yarn && yarn build && yarn server &
PID2=$!

# Wait for both programs to complete
wait $PID1
wait $PID2

echo "Both programs have completed."