#ifndef __solver_hqp_output_hpp__
#define __solver_hqp_output_hpp__

#include "kimm_spot_kinova/solver/fwd.hpp"
#include "kimm_spot_kinova/math/fwd.hpp"

#include <vector>

namespace spotkinova{
    namespace solver{
        class HQPOutput
        {
            public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            
            typedef math::Vector Vector;
            typedef math::VectorXi VectorXi;
            
            HQPStatus status;    /// solver status
            Vector x;            /// solution
            Vector lambda;       /// Lagrange multipliers
            VectorXi activeSet;  /// indexes of active inequalities
            int iterations;      /// number of iterations performed by the solver
            
            HQPOutput(){}
            
            HQPOutput(int nVars, int nEqCon, int nInCon)
            {
                resize(nVars, nEqCon, nInCon);
            }
            
            void resize(int nVars, int nEqCon, int nInCon)
            {
                x.resize(nVars);
                lambda.resize(nEqCon+nInCon);
                activeSet.resize(nInCon);
            }
        };

    }
}

#endif