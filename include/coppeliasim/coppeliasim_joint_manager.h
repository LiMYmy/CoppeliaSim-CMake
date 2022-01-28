#ifndef _COPPELIASIM_JOINT_MANAGER_H
#define _COPPELIASIM_JOINT_MANAGER_H

#include "coppeliasim_joint.h"

class CoppeliaSimJointManager : public CoppeliaSimBase
{
  friend class CoppeliaSim;
public:
  CoppeliaSimJointManager(simxInt client_id);

  /**
   * @brief 获取关节Joint
   *
   * @param id 所要获取的关节id
   * @return [CoppeliaSimJoint&] 若id超出索引范围，返回第0号关节
   */
  CoppeliaSimJointSharedPtr Joint(int id);

  /**
   * @brief 获取关节Joint。运行速度比 Joint(int id) 方法慢。
   *
   * @param id 所要获取的关节名字
   * @return [CoppeliaSimJoint&] 若名字所对应的关节不存在，返回第0号关节
   */
  CoppeliaSimJointSharedPtr Joint(std::string name);

private:

  void RegisterJoints(std::vector<std::string> names);

  void RefreshData(int mode = simx_opmode_oneshot);

  //simxInt client_id_;

  std::vector<CoppeliaSimJointSharedPtr> joints_;
  std::map<std::string, int> name_id_map_;

};

using CoppeliaSimJointManagerSharedPtr = std::shared_ptr<CoppeliaSimJointManager>;

#endif // !_COPPELIASIM_JOINT_MANAGER_H
