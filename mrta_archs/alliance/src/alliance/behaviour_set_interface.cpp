#include "alliance/behaviour_set_interface.h"
#include "alliance/robot_interface.h"

namespace alliance
{
BehaviourSetInterface::BehaviourSetInterface(RobotInterface* robot, Task* task)
    : Subject::Subject(robot->getId() + "/" + task->getId()), task_(task)
{
}

BehaviourSetInterface::~BehaviourSetInterface() { task_ = NULL; }

Task* BehaviourSetInterface::getTask() const { return task_; }
}
