#ifndef _COPPELIASIM_BASE_H
#define _COPPELIASIM_BASE_H

#include "extApi.h"
#include "spdlog/spdlog.h"

#include <string>
#include <iostream>

enum class CoppeliaSimPort : uint32_t
{
	LinkPort = 30001,
	JointPort = 30101
};

class CoppeliaSimBase
{
protected:
	CoppeliaSimBase()
	{
		logger = spdlog::get("main_logger");
	}

	simxInt GetHandler(std::string name)
	{
		using namespace std;
		simxInt handle, ret;
		ret = simxGetObjectHandle(client_id_, name.c_str(), &handle, simx_opmode_oneshot_wait);
		if (ret != simx_return_ok)
		{
			ShowRemoteApiReturnCodesError(ret);
		}

		logger->info("Object: {} Handle: {}", name, handle);
		return handle;
	}

	/**
	* https://www.coppeliarobotics.com/helpFiles/en/remoteApiConstants.htm#functionErrorCodes
	*/
	void ShowRemoteApiReturnCodesError(simxInt val)
	{
		using namespace std;
		switch (val)
		{
		case 2:
			logger->error("The function timed out (probably the network is down or too slow)");
			break;
		case 4:
			logger->error("The specified operation mode is not supported for the given function");
			break;
		case 8:
			logger->error("The function caused an error on the server side(e.g.an invalid handle was specified)");
			break;
		case 16:
			logger->error("The communication thread is still processing previous split command of the same type");
			break;
		case 32:
			logger->error("The function caused an error on the client side");
			break;
		case 64:
			logger->error("simxStart was not yet called");
			break;
		default:
			break;
		}
	}

	simxInt client_id_ = 0;
	simxInt handler_ = 0;

	std::shared_ptr<spdlog::logger> logger;
};

#endif // !_COPPELIASIM_BASE_H

