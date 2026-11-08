rm -rf build/
export CI_PROJECT_ROOT=$(realpath .)
mkdir -p build && cd build
export CI_BUILD_ROOT=$(pwd)
export CI_USE_ENV=1
source ${CI_PROJECT_ROOT}/ci/micropython.sh
ci_prepare_all
ci_build_board pico RPI_PICO
