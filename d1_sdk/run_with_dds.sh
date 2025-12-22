#!/bin/bash
# Helper script to run D1 SDK programs with correct DDS configuration

export CYCLONEDDS_URI='<CycloneDDS><Domain><General><NetworkInterfaceAddress>enx00e04c680099</NetworkInterfaceAddress></General></Domain></CycloneDDS>'

cd ~/d1_sdk/build
"$@"
