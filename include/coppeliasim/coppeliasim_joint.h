#ifndef _COPPELIASIM_JOINT_H
#define _COPPELIASIM_JOINT_H

#include "coppeliasim_base.h"

#include <vector>
#include <map>
#include <math.h>

enum class JointMode
{
  Idle,
  Position,
  Velocity,
  Torque
};

class CoppeliaSimJoint : public CoppeliaSimBase
{
  friend class CoppeliaSimJointManager;

public:
  CoppeliaSimJoint(simxInt client_id, std::string name);

  /**
  * @brief 设置关节目标位置，单位：rad。实际执行效果受PID参数影响。
  */
  CoppeliaSimJoint& SetPosition(float pos);

  /**
  * @brief 设置关节目标力矩，单位：N*m
  */
  CoppeliaSimJoint& SetTorque(float torque);

  /**
  * @brief 设置关节目标速度，单位：rpm
  */
  CoppeliaSimJoint& SetVelocity(float vel);
  

  /**
   * @brief 设置PID参数
   * 
   * @param p
   * @param i
   * @param d
   */
  CoppeliaSimJoint& SetPID(float p, float i, float d);

  /**
   * @brief 设置最大输出力矩，单位：N*m。默认值：300
   * 
   * @param torque 力矩
   */
  CoppeliaSimJoint& SetMaxTorque(float torque);

  /**
   * @brief 设置最大输出转速，单位：rpm。默认值：5000
   *
   * @param velocity 力矩
   */
  CoppeliaSimJoint& SetMaxVelocity(float velocity);

  /**
   * @brief 获取实际位置，单位：rad
   * 
   * @return [float]
   */
  float GetPosition();
  /**
   * @brief 获取实际力矩，单位：N*m
   *
   * @return [float]
   */
  float GetTorque();
  /**
   * @brief 获取实际速度，单位：rpm
   *
   * @return [float]
   */
  float GetVelocity();
  

private:
  void RefreshData(int mode);
  void SetMode(JointMode mode);

  /**
   * @brief 转速单位换算。rad/s -> rpm
   * 
   * @param vel 转速，单位rad/s
   * @return [float] 转速，单位rpm
   */
  inline float RadS2RPM(float vel);

  /**
   * @brief 转速单位换算。rpm -> rad/s
   * 
   * @param vel 转速，单位rpm
   * @return [float] 转速，单位rad/s
   */
  inline float RPM2RadS(float vel);

  std::string name_;

  JointMode mode_ = JointMode::Idle;

  float position_ = 0;
  float torque_ = 0;
  float velocity_ = 0;

  float max_torque_ = 300;
  float max_velocity_ = 5000;
};

using CoppeliaSimJointSharedPtr = std::shared_ptr<CoppeliaSimJoint>;

#endif // !_COPPELIASIM_JOINT_H
