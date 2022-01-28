#include "coppeliasim_link.h"

CoppeliaSimLink::CoppeliaSimLink(simxInt client_id, std::string name)
  : CoppeliaSimBase()
  , name_(name)
{
  client_id_ = client_id;
  handler_ = GetHandler(name_);
}

/**
 * @brief 获取相对于世界坐标系的位置
 *
 * @return [float[3]] 坐标(x, y, z)
 */
float* CoppeliaSimLink::GetPosition()
{
  return position_;
}

/**
 * @brief 获取相对于世界坐标系的姿态
 *
 * @return [float[3]] 姿态(alpha, beta, gamma)
 */
float* CoppeliaSimLink::GetOrientation()
{
  return orientation_;
}

void CoppeliaSimLink::RefreshData(int mode)
{
  simxGetObjectPosition(client_id_, handler_, -1, position_, mode);
  simxGetObjectOrientation(client_id_, handler_, -1, orientation_, mode);
}


