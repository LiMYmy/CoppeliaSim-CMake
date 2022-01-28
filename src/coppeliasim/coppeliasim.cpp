#include "coppeliasim.h"

#include <conio.h>

CoppeliaSim::CoppeliaSim()
{
}

CoppeliaSim::CoppeliaSim(std::string ip, int32_t port)
	: ip_(ip)
	, port_(port)
{
}

void CoppeliaSim::Init()
{
	using namespace std;

	auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_st>();
	console_sink->set_level(spdlog::level::debug);
	//console_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v");
	console_sink->set_pattern("[%H:%M:%S.%e][%^%l%$] %v");

	auto only_output_sink = std::make_shared<spdlog::sinks::basic_file_sink_st>("../logs/data.txt", true);
	only_output_sink->set_pattern("%v");
	only_output_sink->set_level(spdlog::level::info);

	logger = std::make_shared<spdlog::logger>(spdlog::logger("main_logger", { console_sink }));
	logger->set_level(spdlog::level::debug);
	file_recorder = std::make_shared<spdlog::logger>(spdlog::logger("file_recorder", { only_output_sink }));

	spdlog::register_logger(logger);
	spdlog::register_logger(file_recorder);

	//simxFinish(-1);
	client_id_ = simxStart(ip_.c_str(), port_, true, false, 5000, 3);
	logger->info("Client ID: {}", client_id_);
	links_ = std::make_shared<CoppeliaSimLinkManager>(client_id_);
	joints_ = std::make_shared<CoppeliaSimJointManager>(client_id_);
}

float CoppeliaSim::GetSimulationTimeStep()
{
	float step;
	simxInt ret;
	ret = simxGetFloatingParameter(client_id_, sim_floatparam_simulation_time_step, &step, simx_opmode_oneshot_wait);
	if (ret != simx_return_ok)
	{
		ShowRemoteApiReturnCodesError(ret);
	}
	return step;
}

float CoppeliaSim::GetDynamicStepSize()
{
	float step;
	simxInt ret;
	ret = simxGetFloatingParameter(client_id_, sim_floatparam_dynamic_step_size, &step, simx_opmode_oneshot_wait);
	if (ret != simx_return_ok)
	{
		ShowRemoteApiReturnCodesError(ret);
	}
	return step;
}


void CoppeliaSim::SetSimulationTimeStep(float s)
{
	simxInt ret;
	ret = simxSetFloatingParameter(client_id_, sim_floatparam_simulation_time_step, s, simx_opmode_oneshot);
	if ((ret != simx_return_ok) && (ret != simx_return_novalue_flag))
	{
		ShowRemoteApiReturnCodesError(ret);
	}
}

void CoppeliaSim::SetDynamicStepSize(float s)
{
	simxInt ret;
	ret = simxSetFloatingParameter(client_id_, sim_floatparam_dynamic_step_size, s, simx_opmode_oneshot);
	if ((ret != simx_return_ok) && (ret != simx_return_novalue_flag))
	{
		ShowRemoteApiReturnCodesError(ret);
	}
}

void CoppeliaSim::SetLinks(std::vector<std::string> link_name)
{
	links_->RegisterLinks(link_name);
}

void CoppeliaSim::SetJoints(std::vector<std::string> joint_name)
{
	joints_->RegisterJoints(joint_name);
}

size_t CoppeliaSim::GetCycleTimes()
{
	return cycle_times_;
}

void CoppeliaSim::Run(ConfigFunction config_func, CycleFunction cycle_func)
{
	int ping_time;
	//bool first = true;

	if (client_id_ == -1)
	{
		logger->error("can't connect to server");
		return;
	}

	config_func(links_, joints_);

	simxSynchronous(client_id_, true);
	simxStartSimulation(client_id_, simx_opmode_oneshot);

	links_->RefreshData(simx_opmode_oneshot_wait);
	joints_->RefreshData(simx_opmode_oneshot_wait);

	while (1)
	{
		keyboard_ = 0;
		if (_kbhit())
		{
			keyboard_ = _getch();
			if (keyboard_ == ' ')
				break;
		}
		cycle_func(cycle_times_, links_, joints_, keyboard_, logger, file_recorder);
		Trigger();
		cycle_times_++;

		links_->RefreshData();
		joints_->RefreshData();

	}

	Stop();
}

void CoppeliaSim::Pause()
{
	using namespace std;
	static int ret;
	ret = simxPauseSimulation(client_id_, simx_opmode_oneshot_wait);
	if (ret != simx_return_ok)
	{
		logger->error("error while invoking simxPauseSimulation");
		ShowRemoteApiReturnCodesError(ret);
	}
}

void CoppeliaSim::Stop()
{
	using namespace std;
	static int ret;

	//Pause();
	ret = simxStopSimulation(client_id_, simx_opmode_oneshot_wait);
	if (ret != simx_return_ok)
	{
		logger->error("error while invoking simxStopSimulation");
		ShowRemoteApiReturnCodesError(ret);
	}

	simxFinish(client_id_);
}

void CoppeliaSim::Trigger()
{
	simxSynchronousTrigger(client_id_);
}
