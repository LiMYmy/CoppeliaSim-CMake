#include "coppeliasim_link_manager.h"

CoppeliaSimLinkManager::CoppeliaSimLinkManager(simxInt client_id)
{
  client_id_ = client_id;
}

void CoppeliaSimLinkManager::RegisterLinks(std::vector<std::string> names)
{
  for (auto i : names)
  {
    CoppeliaSimLinkSharedPtr link = std::make_shared<CoppeliaSimLink>(client_id_, i);
    links_.push_back(link);
    name_id_map_.insert(std::pair<std::string, int>(i, links_.size() + 1));
  }
}

void CoppeliaSimLinkManager::RefreshData(int mode)
{
  for (int i = 0; i < links_.size(); i++)
  {
    links_[i]->RefreshData(mode);
  }
}

/**
 * @brief 获取连杆Link
 *
 * @param id 所要获取的连杆id
 * @return [CoppeliaSimLink&] 若id超出索引范围，返回第0号连杆。
 */
CoppeliaSimLinkSharedPtr CoppeliaSimLinkManager::Link(int id)
{
  using namespace std;
  if (id < links_.size())
  {
    return links_[id];
  }
  else
  {
    logger->error("Link id {} is out of range", id);
    return links_[0];
  }
}

/**
 * @brief 获取连杆Link。运行速度比 Link(int id) 方法慢。
 *
 * @param id 所要获取的连杆名字
 * @return [CoppeliaSimLink&] 若名字所对应的连杆不存在，返回第0号连杆。
 */
CoppeliaSimLinkSharedPtr CoppeliaSimLinkManager::Link(std::string name)
{
  using namespace std;

  auto val = name_id_map_.find(name);
  if (val != name_id_map_.end())
  {
    return Link(val->second);
  }
  else
  {
    logger->error("Can't find Link name: {}", name);
    return links_[0];
  }
}
