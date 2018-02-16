#!/usr/bin/env bash
protoc -I=proto --cpp_out=aruco_test/gen proto/*.proto