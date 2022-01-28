#include "coppeliasim.h"

#include <math.h>
#define Deg2Rad M_PI/180.0
#define Rad2Deg 180.0/M_PI

using namespace std;

vector<string> link_name = {
  "Torso"
};
vector<string> joint_name = {
  "LeftShoulder", "LeftArm", "LeftElbow", "LeftWrist",
  "LeftHip", "LeftThigh", "LeftKnee", "LeftWheel"
};

CoppeliaSimLinkSharedPtr link0;
CoppeliaSimJointSharedPtr joint2;

/**
 * 初始化，在这里对机器人进行参数设定等。只运行一次。
 */
void ConfigFunc(CoppeliaSimLinkManagerSharedPtr links, CoppeliaSimJointManagerSharedPtr joints)
{
  link0 = links->Link(0);
  joint2 = joints->Joint(2);
}

/**
 * 循环函数。在这里对机器人进行操作，循环执行。
 */
void CycleFunc(uint64_t cycle_times, CoppeliaSimLinkManagerSharedPtr links, CoppeliaSimJointManagerSharedPtr joints, int keyboard, loggerSharedPtr logger, loggerSharedPtr file_recorder)
{
  logger->info("cycle: {}", cycle_times);
  if (keyboard == 'a')
  {
    logger->info("link position: {} {} {}", links->Link(0)->GetPosition()[0], links->Link(0)->GetPosition()[1], links->Link(0)->GetPosition()[2]);
    //cout << "link position: " << link0->GetPosition()[0] << " " << link0->GetPosition()[1] << " " << link0->GetPosition()[2] << endl;
    cout << "link orientation: " << links->Link(0)->GetOrientation()[0] << " " << links->Link(0)->GetOrientation()[1] << " " << links->Link(0)->GetOrientation()[2] << endl;
    //cout << "joint position: " << joints->Joint(2).GetPosition() * Rad2Deg << " velocity: " << joints->Joint(2).GetVelocity() << endl;
    cout << "joint position: " << joint2->GetPosition() * Rad2Deg << " velocity: " << joint2->GetVelocity() << endl;
  }
  
  //joints->Joint(0)->SetTorque(50);
  //joints->Joint(1)->SetVelocity(10);
  //joints->Joint(2).SetMaxTorque(200);
  joints->Joint(2)->SetPosition(-80*Deg2Rad);

  file_recorder->info("{}", joint2->GetPosition() * Rad2Deg);
}

int main()
{
  CoppeliaSim sim;

  sim.Init();

  sim.SetLinks(link_name);
  sim.SetJoints(joint_name);

  sim.SetSimulationTimeStep(0.001);
  sim.logger->info("Simulation Step: {}s", sim.GetSimulationTimeStep());
  sim.SetDynamicStepSize(0.001);
  sim.logger->info("Dynamic Step: {}s", sim.GetDynamicStepSize());

  sim.Run(ConfigFunc, CycleFunc);
}
