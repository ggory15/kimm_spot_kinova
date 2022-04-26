#include "kimm_spot_kinova/solver/solver_HQP_factory.hpp"
#include "kimm_spot_kinova/solver/solver_HQP_eiquadprog.hpp"
#include "kimm_spot_kinova/solver/solver_HQP_eiquadprog_fast.hpp"

namespace spotkinova
{
  namespace solver
  {
    SolverHQPBase* SolverHQPFactory::createNewSolver(const SolverHQP solverType, const std::string & name)
    {
      if(solverType==SOLVER_HQP_EIQUADPROG)
        return new SolverHQuadProg(name);
      
      if(solverType==SOLVER_HQP_EIQUADPROG_FAST)
        return new SolverHQuadProgFast(name);
      
      assert(false && "Specified solver type not recognized");
      return NULL;
    }
    
  }
}