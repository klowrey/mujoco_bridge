
if(DEFINED ENV{MJ_ROOT_DIR} )
	set(MJ_ROOT_DIR "$ENV{MJ_ROOT_DIR}" )
endif()

find_path(MJ_INCLUDE_DIR
	NAMES
		mujoco.h
	PATHS
		${MJ_ROOT_DIR}/include/mujoco
		/usr/include/mujoco
		/usr/local/include/mujoco
)

if(MJ_INCLUDE_DIR)

	px4_add_git_submodule(TARGET git_mujoco_bridge PATH "${PX4_SOURCE_DIR}/Tools/simulation/mujoco/mujoco_bridge")

	include(ExternalProject)
	ExternalProject_Add(mujoco_bridge
		SOURCE_DIR ${PX4_SOURCE_DIR}/Tools/simulation/mujoco/mujoco_bridge
		CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
		BINARY_DIR ${PX4_BINARY_DIR}/build_mujoco_bridge
		INSTALL_COMMAND ""
		DEPENDS git_mujoco_bridge
		USES_TERMINAL_CONFIGURE true
		USES_TERMINAL_BUILD true
		EXCLUDE_FROM_ALL true
		BUILD_ALWAYS 1
	)

	# mujoco: create targets for mujoco
	set(models
    #rascal
		#quadrotor_x
		#hexarotor_x
		#malolo
    skydio_x2
    #genesis
	)

	set(worlds
		none
    #LSZH
	)


	# find corresponding airframes
	file(GLOB mujoco_airframes
		RELATIVE ${PX4_SOURCE_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes
		${PX4_SOURCE_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/*_mujoco_*
	)

	# remove any .post files
	foreach(mujoco_airframe IN LISTS mujoco_airframes)
		if(mujoco_airframe MATCHES ".post")
			list(REMOVE_ITEM mujoco_airframes ${mujoco_airframe})
		endif()
	endforeach()
	list(REMOVE_DUPLICATES mujoco_airframes)

	# default mujoco target
	add_custom_target(mujoco
		COMMAND ${PX4_SOURCE_DIR}/Tools/simulation/mujoco/sitl_run.sh $<TARGET_FILE:px4> "skydio_x2" ${PX4_SOURCE_DIR} ${PX4_BINARY_DIR}
		WORKING_DIRECTORY ${SITL_WORKING_DIR}
		USES_TERMINAL
		DEPENDS px4 mujoco_bridge
	)

	foreach(model ${models})

		# match model to airframe
		set(airframe_model_only)
		set(airframe_sys_autostart)
		set(mujoco_airframe_found)
		foreach(mujoco_airframe IN LISTS mujoco_airframes)

			string(REGEX REPLACE ".*_mujoco_" "" airframe_model_only ${mujoco_airframe})
			string(REGEX REPLACE "_mujoco_.*" "" airframe_sys_autostart ${mujoco_airframe})

			if(model STREQUAL ${airframe_model_only})
				set(mujoco_airframe_found ${mujoco_airframe})
				break()
			endif()
		endforeach()

		if(mujoco_airframe_found)
			message(STATUS "mujoco model: ${model} (${airframe_model_only}), airframe: ${mujoco_airframe_found}, SYS_AUTOSTART: ${airframe_sys_autostart}")
		else()
			message(WARNING "mujoco missing model: ${model} (${airframe_model_only}), airframe: ${mujoco_airframe_found}, SYS_AUTOSTART: ${airframe_sys_autostart}")
		endif()


        #foreach(world ${worlds})
		#	if(world STREQUAL "none")
				add_custom_target(mujoco_${model}
					COMMAND ${PX4_SOURCE_DIR}/Tools/simulation/mujoco/sitl_run.sh $<TARGET_FILE:px4> ${model} ${PX4_SOURCE_DIR} ${PX4_BINARY_DIR}
					WORKING_DIRECTORY ${SITL_WORKING_DIR}
					USES_TERMINAL
					DEPENDS px4 mujoco_bridge
				)
        #		else()
	    #			add_custom_target(mujoco_${model}__${world}
	    #				COMMAND ${PX4_SOURCE_DIR}/Tools/simulation/mujoco/sitl_run.sh $<TARGET_FILE:px4> ${model} ${PX4_SOURCE_DIR} ${PX4_BINARY_DIR}
	    #				WORKING_DIRECTORY ${SITL_WORKING_DIR}
	    #				USES_TERMINAL
	    #				DEPENDS px4 mujoco_bridge
	    #			)
	    #		endif()
	    #	endforeach()
	endforeach()

endif()
