#include "kimm_spot_kinova/formulation/inverse_dynamics_formulation_base.hpp"

namespace spotkinova{
    TaskLevel::TaskLevel(tasks::TaskBase & t, unsigned int priority):
    task(t),
    priority(priority)
    {}

  InverseDynamicsFormulationBase::InverseDynamicsFormulationBase(const std::string & name, RobotWrapper & robot, bool verbose)
  : m_name(name)
  , m_robot(robot)
  , m_verbose(verbose)
  {}

}
