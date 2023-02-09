#!/bin/bash

echo "running cov manage im"
/cov/bin/cov-manage-im --mode auth-key --create --output-file auth_key.txt --url https://coverity.ssrc.fi:443 --user cov_scan --set description:"ci test key" --set expiration:"after_1_days"
echo "running coverity scan exclude java"
/cov/bin/coverity scan --exclude-language java
echo "running coverity list"
/cov/bin/coverity list