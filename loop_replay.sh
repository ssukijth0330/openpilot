#!/bin/bash
export NO_NAV=1
export SEND_RAW_PRED=1
while ./selfdrive/test/process_replay/model_replay.py; do :; done
