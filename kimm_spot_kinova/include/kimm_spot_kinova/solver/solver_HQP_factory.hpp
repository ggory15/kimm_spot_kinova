#ifndef __solver_hqp_factory_hpp__
#define __solver_hqp_factory_hpp__

#include "kimm_spot_kinova/solver/solver_HQP_base.hpp"
#include "kimm_spot_kinova/solver/solver_HQP_eiquadprog_rt.hpp"

namespace spotkinova{
    namespace solver{
        struct SolverHQPFactory
        {
        
        /**
         * @brief Create a new HQP solver of the specified type.
         *
         * @param solverType Type of HQP solver.
         * @param name Name of the solver.
         *
         * @return A pointer to the new solver.
         */
        static SolverHQPBase * createNewSolver(const SolverHQP solverType,
                                                const std::string & name);
        
        /**
         * @brief Create a new HQP solver of the specified type.
         *
         * @param solverType Type of HQP solver.
         * @param name Name of the solver.
         *
         * @return A pointer to the new solver.
         */
        template<int nVars, int nEqCon, int nIneqCon>
        static SolverHQPBase* createNewSolver(const SolverHQP solverType,
                                                const std::string & name);
        };
    }
}

#endif