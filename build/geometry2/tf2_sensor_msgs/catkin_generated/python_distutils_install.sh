#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/xtark/robot_follow_yolo/src/geometry2/tf2_sensor_msgs"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/xtark/robot_follow_yolo/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/xtark/robot_follow_yolo/install/lib/python3/dist-packages:/home/xtark/robot_follow_yolo/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/xtark/robot_follow_yolo/build" \
    "/usr/bin/python3" \
    "/home/xtark/robot_follow_yolo/src/geometry2/tf2_sensor_msgs/setup.py" \
     \
    build --build-base "/home/xtark/robot_follow_yolo/build/geometry2/tf2_sensor_msgs" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/xtark/robot_follow_yolo/install" --install-scripts="/home/xtark/robot_follow_yolo/install/bin"
