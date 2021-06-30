#!/bin/bash

set -e

#pushd ../../
#cargo +nightly test
#popd

cargo +nightly build --release
arm-none-eabi-objcopy target/thumbv7em-none-eabihf/release/audio -O ihex target/thumbv7em-none-eabihf/release/audio.hex
arm-none-eabi-size target/thumbv7em-none-eabihf/release/audio
