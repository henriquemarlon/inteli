#!/bin/bash

echo "Opening PATH"
cd ./src
echo "Starting server and Broker..."
python -m flask run --host='0.0.0.0' & 
P1=$!
node ./broker/index.js &
P2=$!
wait $P1 $P2
