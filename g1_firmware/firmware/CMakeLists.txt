cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(g1v0)
# The VM core (protocol 5) is compiled in place from the ScouterKit tree, never copied.
file(REAL_PATH ${CMAKE_CURRENT_SOURCE_DIR} G1_APP_DIR)
get_filename_component(G1_SK_DIR ${G1_APP_DIR}/../.. ABSOLUTE)
set(G1_VM_DIR ${G1_SK_DIR}/src/vm)
target_sources(app PRIVATE
	src/early.S src/retained.c src/health.c src/board.c src/netcore.c
	src/pmic.c src/power.c src/imu.c src/diag.c src/caselink.c src/panel.c src/exec.c src/proto.c src/netfallback.c src/netimg.S src/main.c
	src/mic.c src/touch.c src/touchcfg.S src/light.c src/peer.c src/idle.c src/guard.c src/assets.c src/bonds.c
	src/fc.c src/fc_inflate.c src/ancs.c
	${G1_VM_DIR}/vm.c ${G1_VM_DIR}/vm_gfx.c)
target_include_directories(app PRIVATE ${G1_VM_DIR})
# DIAG BONDS reads the host's struct bt_keys (aging counter, key flags): its internal header, for this file only.
set_source_files_properties(src/bonds.c PROPERTIES INCLUDE_DIRECTORIES ${ZEPHYR_BASE}/subsys/bluetooth/host)
# The interpreter runs every host packet and program instruction: -O2 instead of the image's -Os.
set_source_files_properties(${G1_VM_DIR}/vm.c ${G1_VM_DIR}/vm_gfx.c PROPERTIES COMPILE_OPTIONS "-O2")
set_source_files_properties(src/fc.c src/fc_inflate.c PROPERTIES COMPILE_OPTIONS "-O2")
# liblc3 (Apache-2.0, vendored with its licence): the LC3 encoder of the microphone, 10 ms frames only.
set(G1_LC3_DIR ${G1_SK_DIR}/third_party/liblc3)
file(GLOB G1_LC3_SRC ${G1_LC3_DIR}/src/*.c)
target_sources(app PRIVATE ${G1_LC3_SRC})
target_include_directories(app PRIVATE ${G1_LC3_DIR}/include)
set_source_files_properties(${G1_LC3_SRC} PROPERTIES
	COMPILE_OPTIONS "-std=c11;-O3;-ffast-math;-Wno-unused-parameter;-Wno-double-promotion"
	COMPILE_DEFINITIONS "LC3_PLUS=0;LC3_PLUS_HR=0")
# The first instructions after reset go through early.S (WDT0 handling), then into Zephyr.
zephyr_ld_options(-Wl,--wrap=z_arm_reset)
# Even's net-core image embedded for the self-fallback (src/netimg.S, src/netfallback.c), fetched by
# scripts/fetch_stock.sh 1.6.6 into the work directory; never part of this tree.
set(G1_NET_IMAGE $ENV{G1_WORK}/firmware/1.6.6/net_core_app_update.bin CACHE FILEPATH "net image for the self-fallback")
if(NOT EXISTS ${G1_NET_IMAGE})
	message(FATAL_ERROR "${G1_NET_IMAGE} not found: run scripts/fetch_stock.sh 1.6.6")
endif()
set(G1_NET_IMAGE_NAME 1.6.6 CACHE STRING "version label of G1_NET_IMAGE")
set_source_files_properties(src/netimg.S PROPERTIES
	COMPILE_DEFINITIONS "G1_NET_IMAGE_PATH=\"${G1_NET_IMAGE}\"" OBJECT_DEPENDS ${G1_NET_IMAGE})
set_source_files_properties(src/netfallback.c PROPERTIES
	COMPILE_DEFINITIONS "G1_NET_IMAGE_NAME=\"${G1_NET_IMAGE_NAME}\"")
# The TouchBar controller's configuration table, as stock loads it (src/touchcfg.S, src/touch.c), taken from
# Even's 1.6.6 image by scripts/fetch_stock.sh 1.6.6 into the work directory; never part of this tree.
set(G1_TOUCH_TABLE $ENV{G1_WORK}/fw/1.6.6_aw9320x_table.bin CACHE FILEPATH "TouchBar controller table")
if(NOT EXISTS ${G1_TOUCH_TABLE})
	message(FATAL_ERROR "${G1_TOUCH_TABLE} not found: run scripts/fetch_stock.sh 1.6.6")
endif()
set_source_files_properties(src/touchcfg.S PROPERTIES
	COMPILE_DEFINITIONS "G1_TOUCH_TABLE_PATH=\"${G1_TOUCH_TABLE}\"" OBJECT_DEPENDS ${G1_TOUCH_TABLE})
