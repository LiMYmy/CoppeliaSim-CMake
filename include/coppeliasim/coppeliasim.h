#ifndef _COPPELIASIM_H
#define _COPPELIASIM_H

#include "coppeliasim_link.h"
#include "coppeliasim_link_manager.h"
#include "coppeliasim_joint.h"
#include "coppeliasim_joint_manager.h"
#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"

#include <Windows.h>
#include <iostream>
#include <string>
#include <vector>
#include <memory>

using loggerSharedPtr = std::shared_ptr<spdlog::logger>;

using ConfigFunction = void(*)(CoppeliaSimLinkManagerSharedPtr, CoppeliaSimJointManagerSharedPtr);
using CycleFunction = void(*)(uint64_t, CoppeliaSimLinkManagerSharedPtr, CoppeliaSimJointManagerSharedPtr, int, loggerSharedPtr, loggerSharedPtr);

class CoppeliaSim : public CoppeliaSimBase
{
public:
	CoppeliaSim();
	CoppeliaSim(std::string ip, int32_t port);

	void Init();

	float GetSimulationTimeStep();
	float GetDynamicStepSize();

	/**
	* @brief Can only be written when using a custom dt value (which can be set in the simulation settings dialog), and when simulation is stopped.
	*/
	void SetSimulationTimeStep(float ms);

	/**
	* @brief Can only be written when using a customized configuration for the physics engine, and when simulation is stopped.
	*/
	void SetDynamicStepSize(float ms);

	/**
	 * @brief 设置仿真所需的连杆
	 * 
	 * @param link_name 连杆名称
	 */
	void SetLinks(std::vector<std::string> link_name);

	/**
	 * @brief 设置仿真所需的关节
	 * 
	 * @param joint_name 关节名称
	 */
	void SetJoints(std::vector<std::string> joint_name);

	/**
	 * @brief 仿真启动函数。仿真器会将设置的连杆与关节传入，用户对其进行操作
	 * 
	 * @param func 用户自定义运行函数
	 */
	void Run(ConfigFunction config_func, CycleFunction cycle_func);

	/**
	 * @brief 获取循环次数
	 * 
	 * @return [uint64_t] 循环次数
	 */
	uint64_t GetCycleTimes();

	std::shared_ptr<spdlog::logger> logger;
	std::shared_ptr<spdlog::logger> file_recorder;

private:
	void Pause();
	void Stop();
	void Trigger();

	std::string ip_ = "127.0.0.1";
	int32_t port_ = 19997;
	simxInt client_id_ = 0;
	
	uint64_t cycle_times_ = 0;

	CoppeliaSimLinkManagerSharedPtr links_;
	CoppeliaSimJointManagerSharedPtr joints_;

	int keyboard_ = 0;

};


#endif // !_COPPELIASIM_H

