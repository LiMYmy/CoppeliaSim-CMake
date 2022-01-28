#ifndef _COPPELIASIM_LINK_MANAGER_H
#define _COPPELIASIM_LINK_MANAGER_H

#include "coppeliasim_link.h"

class CoppeliaSimLinkManager : public CoppeliaSimBase
{
  friend class CoppeliaSim;
public:
  CoppeliaSimLinkManager(simxInt client_id);

  /**
   * @brief 获取连杆Link
   *
   * @param id 所要获取的连杆id
   * @return [CoppeliaSimLink&] 若id超出索引范围，返回第0号连杆。
   */
  CoppeliaSimLinkSharedPtr Link(int id);

  /**
   * @brief 获取连杆Link。运行速度比 Link(int id) 方法慢。
   *
   * @param id 所要获取的连杆名字
   * @return [CoppeliaSimLink&] 若名字所对应的连杆不存在，返回第0号连杆。
   */
  CoppeliaSimLinkSharedPtr Link(std::string name);

private:
  void RegisterLinks(std::vector<std::string> names);

  void RefreshData(int mode = simx_opmode_oneshot);

  std::vector<CoppeliaSimLinkSharedPtr> links_;
  std::map<std::string, int> name_id_map_;
};

using CoppeliaSimLinkManagerSharedPtr = std::shared_ptr<CoppeliaSimLinkManager>;

#endif // !_COPPELIASIM_LINK_MANAGER_H

