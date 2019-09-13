source /usr/share/gazebo-7/setup.sh
export GAZEBO_PREFIX=/home/nbore/Workspace/bts_ws/install
export GAZEBO_MODEL_DATABASE_URI=http://gazebosim.org/models
export GAZEBO_RESOURCE_PATH=${GAZEBO_PREFIX}/share/uuv_descriptions:${GAZEBO_RESOURCE_PATH}
export GAZEBO_MODEL_PATH=${GAZEBO_PREFIX}/share/uuv_descriptions/worlds:${GAZEBO_PREFIX}/share/uuv_descriptions/world_models:${GAZEBO_MODEL_PATH}:${GAZEBO_PREFIX}/share/bts_tutorial/world_models:${GAZEBO_PREFIX}/share/smarc_private_worlds/world_models
export GAZEBO_PLUGIN_PATH=${GAZEBO_PREFIX}/lib:${GAZEBO_PREFIX}/lib/x86_64-linux-gnu:${GAZEBO_PLUGIN_PATH}

#source ${GAZEBO_PREFIX}/setup.bash
