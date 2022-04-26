#ifndef __solver_fwd_hpp__
#define __solver_fwd_hpp__

#include <memory>

#include "kimm_spot_kinova/config.hpp"
#include "kimm_spot_kinova/math/fwd.hpp"
#include <pinocchio/container/aligned-vector.hpp>

#define DEFAULT_HESSIAN_REGULARIZATION 1e-8

namespace spotkinova{
    namespace solver{
        enum TSID_DLLAPI SolverHQP
        {
            SOLVER_HQP_EIQUADPROG = 0,
            SOLVER_HQP_EIQUADPROG_FAST = 1,
            SOLVER_HQP_EIQUADPROG_RT = 2
            
        };
        enum TSID_DLLAPI HQPStatus
        {
            HQP_STATUS_UNKNOWN=-1,
            HQP_STATUS_OPTIMAL=0,
            HQP_STATUS_INFEASIBLE=1,
            HQP_STATUS_UNBOUNDED=2,
            HQP_STATUS_MAX_ITER_REACHED=3,
            HQP_STATUS_ERROR=4
        };
        
        class HQPOutput;
        
        class TSID_DLLAPI SolverHQPBase;
        
        template<int nVars, int nEqCon, int nIneqCon>
        class TSID_DLLAPI SolverHQuadProgRT;
        
        template<typename T1, typename T2>
        class aligned_pair
        {
            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW
                
                aligned_pair(const T1 & t1, const T2 & t2) : first(t1), second(t2) {}
                
                T1 first;
                T2 second;
        
        };
        
        template<typename T1, typename T2>
        inline aligned_pair<T1,T2> make_pair(const T1 & t1, const T2 & t2) {
            return aligned_pair<T1,T2>(t1,t2); 
        }
        
        
        typedef pinocchio::container::aligned_vector< aligned_pair<double, std::shared_ptr<math::ConstraintBase> > > ConstraintLevel;
        typedef pinocchio::container::aligned_vector< aligned_pair<double, std::shared_ptr<const math::ConstraintBase> > > ConstConstraintLevel;
        typedef pinocchio::container::aligned_vector<ConstraintLevel> HQPData;
        typedef pinocchio::container::aligned_vector<ConstConstraintLevel> ConstHQPData;
    }
}

#endif