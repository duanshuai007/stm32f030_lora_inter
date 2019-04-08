#!/bin/sh

id=$1
cmd=$2

echo $id
echo $cmd

cat /dev/null > endpoint_cmd.txt
echo "id=${id}" >> endpoint_cmd.txt
echo "cmd=${cmd}" >> endpoint_cmd.txt
