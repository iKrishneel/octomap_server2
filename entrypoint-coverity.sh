#!/bin/bash -e

cd /main_ws/src/
/cov/bin/cov-manage-im --mode auth-key --mode auth-key --create --output-file auth_key.txt --url https://coverity.ssrc.fi:443 --user cov_scan --set description:"ci test key" --set expiration:"after_1_days"
/cov/bin/coverity scan
/cov/bin/coverity list
