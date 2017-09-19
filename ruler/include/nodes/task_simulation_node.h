#ifndef _NODES_TASK_SIMULATION_NODE_H_
#define _NODES_TASK_SIMULATION_NODE_H_

#include "ruler/ruler.h"
#include "utilities/ros_node.h"

namespace nodes
{
class TaskSimulationNode : public utilities::ROSNode
{
public:
  TaskSimulationNode(const ros::NodeHandlePtr& nh,
                           const ros::Rate& loop_rate = ros::Rate(30.0));
  virtual ~TaskSimulationNode();

protected:
  typedef std::list<ruler::TaskSimulationPtr>::iterator iterator;
  typedef std::list<ruler::TaskSimulationPtr>::const_iterator const_iterator;
  std::list<ruler::TaskSimulationPtr> scheduled_simulations_;

private:
  ros::Time start_timestmap_;
  virtual void readParameters();
  virtual void controlLoop();
};

typedef boost::shared_ptr<TaskSimulationNode> TaskSimulationNodePtr;
}
#endif // _NODES_TASK_SIMULATION_NODE_H_
