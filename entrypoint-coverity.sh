#!/bin/bash -eu

echo "pull coverity token"
/cov/bin/cov-manage-im --mode auth-key --create --output-file auth_key.txt --url https://coverity.ssrc.fi:443 --user cov_scan --set description:"ci test key" --set expiration:"after_1_days"
authentication_id=$(grep -ow "id\":[[:digit:]]\+" auth_key.txt |grep -ow '[[:digit:]]\+')

echo "running coverity scan"
/cov/bin/coverity scan
/cov/bin/coverity list

echo "revoking authentication token"
/cov/bin/cov-manage-im --mode auth-key --revoke $authentication_id --url https://coverity.ssrc.fi:443 --user cov_scan