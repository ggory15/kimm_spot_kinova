#ifndef __trajectory_euclidian_hpp__
#define __trajectory_euclidian_hpp__

#include "kimm_spot_kinova/trajectories/trajectory_base.hpp"
#include <kimm_trajectory_smoother/Path.h>
#include <kimm_trajectory_smoother/Trajectory.h>

using namespace kimmtraj;

namespace spotkinova{
    namespace trajectory{
        class TrajectoryEuclidianConstant : public TrajectoryBase
        {
            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW
                
                typedef math::Vector         Vector;
                typedef math::ConstRefVector ConstRefVector;

                TrajectoryEuclidianConstant(const std::string & name);

                TrajectoryEuclidianConstant(const std::string & name, ConstRefVector ref);

                unsigned int size() const;

                void setReference(ConstRefVector ref);

                const TrajectorySample & operator()(double time);

                const TrajectorySample & computeNext();

                void getLastSample(TrajectorySample & sample) const;

                bool has_trajectory_ended() const;

            protected:
                Vector m_ref;
        };

        class TrajectoryEuclidianCubic : public TrajectoryBase
		{
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            typedef math::Vector         Vector;
            typedef math::ConstRefVector ConstRefVector;

			TrajectoryEuclidianCubic(const std::string & name);
			TrajectoryEuclidianCubic(const std::string & name, ConstRefVector init_M, ConstRefVector goal_M, const double & duration, const double & stime);

			unsigned int size() const;
			const TrajectorySample & operator()(double time);
			const TrajectorySample & computeNext();
			void getLastSample(TrajectorySample & sample) const;
			bool has_trajectory_ended() const;
			void setReference(ConstRefVector ref);
			void setInitSample(ConstRefVector init_M);
			void setGoalSample(ConstRefVector goal_M);
			void setDuration(const double & duration);
			void setCurrentTime(const double & time);
			void setStartTime(const double & time);

		protected:
			Vector m_ref;
			Vector m_init, m_goal, m_cubic;
			double m_duration, m_stime, m_time;
		};

        class TrajectoryEuclidianTimeopt : public TrajectoryBase
		{
            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW

                typedef math::Vector         Vector;
                typedef math::ConstRefVector ConstRefVector;

                TrajectoryEuclidianTimeopt(const std::string & name);
                TrajectoryEuclidianTimeopt(const std::string & name, ConstRefVector MaxVel, ConstRefVector MaxAcc);

                unsigned int size() const;
                const TrajectorySample & operator()(double time);
                const TrajectorySample & computeNext();

                void getLastSample(TrajectorySample & sample) const;
                bool has_trajectory_ended() const;
                void setReference(ConstRefVector ref);
                void addWaypoint(ConstRefVector waypoints);
                void clearWaypoints();
                void setCurrentTime(const double & time);
                void setStartTime(const double & time);
                void setMaxVelocity(ConstRefVector MaxVel);
                void setMaxAcceleration(ConstRefVector MaxAcc);

            protected:
                Vector m_ref;
                int m_size;
                Vector m_maxvel, m_maxacc;
                double m_duration, m_stime, m_time;
                stdlist_Eigenvec m_waypoints;
                bool m_calc;
                Trajectory* m_traj;

		};
    }
}

#endif 