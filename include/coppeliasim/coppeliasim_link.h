#ifndef _COPPELIASIM_BODY_H
#define _COPPELIASIM_BODY_H

#include "coppeliasim_base.h"

#include <vector>
#include <map>
#include <memory>

class CoppeliaSimLink : protected CoppeliaSimBase
{
  friend class CoppeliaSimLinkManager;
public:
  CoppeliaSimLink(simxInt client_id, std::string name);

  /**
   * @brief 获取相对于世界坐标系的位置
   * 
   * @return [float[3]] 坐标(x, y, z)
   */
  float* GetPosition();

  /**
   * @brief 获取相对于世界坐标系的姿态
   *
   * @return [float[3]] 姿态(alpha, beta, gamma)
   */
  float* GetOrientation();

private:
  void RefreshData(int mode);

  std::string name_;

  float position_[3] = { 0 };
  float orientation_[3] = { 0 };
};

using CoppeliaSimLinkSharedPtr = std::shared_ptr<CoppeliaSimLink>;

#endif // !_COPPELIASIM_BODY_H

