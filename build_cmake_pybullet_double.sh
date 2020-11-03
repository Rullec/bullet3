#!/bin/sh

if [ -e CMakeCache.txt ]; then
  rm CMakeCache.txt
fi

# MODE="RELEASE"
# MODE="DEBUG"
# if [ "${MODE}" = "RELEASE" ]; then
##################### We must build two times ###################
# echo "Build bullet in release mode"
mkdir -p build_cmake
cd build_cmake
cmake -DBUILD_PYBULLET=ON -DBUILD_PYBULLET_NUMPY=OFF -DUSE_DOUBLE_PRECISION=ON -DBT_USE_EGL=ON -DCMAKE_BUILD_TYPE=Release .. || exit 1
# cmake -DBUILD_PYBULLET=ON -DBUILD_PYBULLET_NUMPY=OFF -DUSE_DOUBLE_PRECISION=ON -DBT_USE_EGL=ON -DCMAKE_BUILD_TYPE=debug .. || exit 1

# float
# cmake -DBUILD_PYBULLET=ON -DBUILD_PYBULLET_NUMPY=ON -DUSE_DOUBLE_PRECISION=OFF -DBT_USE_EGL=ON -DCMAKE_BUILD_TYPE=Release .. || exit 1

make -j $(command nproc 2>/dev/null || echo 12) V=1 || exit 1
cd examples
cd pybullet
if [ -e pybullet.dylib ]; then
  ln -f -s pybullet.dylib pybullet.so
fi
if [ -e pybullet_envs ]; then
  rm pybullet_envs
fi
if [ -e pybullet_data ]; then
  rm pybullet_data
fi
if [ -e pybullet_utils ]; then
  rm pybullet_utils
fi
ln -s ../../../examples/pybullet/gym/pybullet_envs .
ln -s ../../../examples/pybullet/gym/pybullet_data .
ln -s ../../../examples/pybullet/gym/pybullet_utils .
echo "Completed build of Bullet."
