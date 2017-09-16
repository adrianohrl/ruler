#ifndef _RULER_PREEMPTIVE_TASK_SIMULATION_H_
#define _RULER_PREEMPTIVE_TASK_SIMULATION_H_

#include "ruler/task_simulation.h"
#include "ruler/preemptive_task.h"

namespace ruler
{
class PreemptiveTaskSimulation : public TaskSimulation
{
public:
  PreemptiveTaskSimulation(
      const PreemptiveTaskPtr& task,
      const utilities::ContinuousNoisySignalPtr& expected_sample_time);
  virtual ~PreemptiveTaskSimulation();
};

typedef boost::shared_ptr<PreemptiveTaskSimulation> PreemptiveTaskSimulationPtr;
typedef boost::shared_ptr<PreemptiveTaskSimulation const>
    PreemptiveTaskSimulationConstPtr;
}

#endif // _RULER_PREEMPTIVE_TASK_SIMULATION_H_
