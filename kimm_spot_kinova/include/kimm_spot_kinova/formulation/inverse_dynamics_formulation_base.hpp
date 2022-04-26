#ifndef __inv_dyn_base_hpp__
#define __inv_dyn_base_hpp__

#include "kimm_spot_kinova/math/fwd.hpp"
#include "kimm_spot_kinova/robots/robot-wrapper.hpp"
#include "kimm_spot_kinova/tasks/task_motion.hpp"
#include "kimm_spot_kinova/solver/solver_HQP_base.hpp"

#include <string>

namespace spotkinova{
    struct TaskLevel
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
        tasks::TaskBase & task;
        std::shared_ptr<math::ConstraintBase> constraint;
        unsigned int priority;

        TaskLevel(tasks::TaskBase & t, unsigned int priority);
    };

    class InverseDynamicsFormulationBase
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        typedef pinocchio::Data Data;
        typedef math::Vector Vector;
        typedef math::RefVector RefVector;
        typedef math::ConstRefVector ConstRefVector;
        typedef tasks::TaskMotion TaskMotion;
        typedef tasks::TaskBase TaskBase;
        typedef solver::HQPData HQPData;
        typedef solver::HQPOutput HQPOutput;
        typedef robots::RobotWrapper RobotWrapper;

        InverseDynamicsFormulationBase(const std::string & name, RobotWrapper & robot, bool verbose=false);

        virtual Data & data() = 0;

        virtual unsigned int nVar() const = 0;
        virtual unsigned int nEq() const = 0;
        virtual unsigned int nIn() const = 0;

        virtual bool addMotionTask(TaskMotion & task, double weight, unsigned int priorityLevel, double transition_duration=0.0) = 0;

        virtual bool updateTaskWeight(const std::string & task_name, double weight) = 0;

        virtual bool removeTask(const std::string & taskName, double transition_duration=0.0) = 0;


        virtual const HQPData & computeProblemData(double time, ConstRefVector q, ConstRefVector v) = 0;

        virtual const Vector & getAccelerations(const HQPOutput & sol) = 0;

    protected:
        std::string m_name;
        RobotWrapper m_robot;
        bool m_verbose;
    };
    
}

#endif