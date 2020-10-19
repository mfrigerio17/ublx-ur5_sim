#!/bin/sh

BASE_PATH=`pwd`

python -m rmt.viewer \
-t $BASE_PATH/model/ur5-meshes-transforms.motdsl \
$BASE_PATH/model/ur5.kindsl $BASE_PATH/model/ur5-meshes.yaml

