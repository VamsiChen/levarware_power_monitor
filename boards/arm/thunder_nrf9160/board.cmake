# SPDX-License-Identifier: Apache-2.0

board_runner_args(jlink "--device=cortext-m33" "--speed=4000")
include(${ZEPHYR_BASE}/boards/common/nrfjprog.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
