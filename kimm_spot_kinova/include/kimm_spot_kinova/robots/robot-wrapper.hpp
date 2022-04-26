#ifndef __spot_kinova_robot_wrapper_hpp__
#define __spot_kinova_robot_wrapper_hpp__

#include "kimm_spot_kinova/robots/fwd.hpp"
#include "kimm_spot_kinova/math/fwd.hpp"

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/centroidal.hpp>

#include <string>
#include <vector>

namespace spotkinova
{
  namespace robots
  {
    ///
    /// \brief Wrapper for a robot based on pinocchio
    ///
    class RobotWrapper
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      
      typedef math::Scalar Scalar;
      typedef pinocchio::Model Model;
      typedef pinocchio::Data Data;
      typedef pinocchio::Motion Motion;
      typedef pinocchio::Frame Frame;
      typedef pinocchio::SE3 SE3;
      typedef math::Vector  Vector;
      typedef math::Vector3 Vector3;
      typedef math::Vector6 Vector6;
      typedef math::Matrix Matrix;
      typedef math::Matrix3x Matrix3x;
      typedef math::RefVector RefVector;
      typedef math::ConstRefVector ConstRefVector;
      
      
      RobotWrapper(const std::string & filename,
                   const std::vector<std::string> & package_dirs,
                   bool verbose=false);

      RobotWrapper(const std::string & filename,
                   const std::vector<std::string> & package_dirs,
                   const pinocchio::JointModelVariant & rootJoint,
                   bool verbose=false);
      
      virtual int nq() const;
      virtual int nv() const;
      virtual int na() const;
      
      const Model & model() const;
      Model & model();
      
      void computeAllTerms(Data & data, const Vector & q, const Vector & v);
      
      void com(const Data & data,
               RefVector com_pos,
               RefVector com_vel,
               RefVector com_acc) const;
      
      const Vector3 & com(const Data & data) const;
    
      const SE3 & position(const Data & data,
                           const Model::JointIndex index) const;
      
      const Motion & velocity(const Data & data,
                              const Model::JointIndex index) const;
            
      void jacobianWorld(const Data & data,
                         const Model::JointIndex index,
                         Data::Matrix6x & J);
      
      void jacobianLocal(const Data & data,
                         const Model::JointIndex index,
                         Data::Matrix6x & J);
      
      SE3 framePosition(const Data & data,
                        const Model::FrameIndex index) const;
      
      void framePosition(const Data & data,
                         const Model::FrameIndex index,
                         SE3 & framePosition) const;
      
      Motion frameVelocity(const Data & data,
                           const Model::FrameIndex index) const;

      void frameVelocity(const Data & data,
                         const Model::FrameIndex index,
                         Motion & frameVelocity) const;
      
      void frameJacobianLocal(Data & data,
                              const Model::FrameIndex index,
                              Data::Matrix6x & J);


    protected:
      
      
      /// \brief Robot model.
      Model m_model;
      std::string m_model_filename;
      bool m_verbose;
      
      int m_na;     /// number of actuators (nv for fixed-based, nv-6 for floating-base robots)

    };
    
  } // namespace robots

} // namespace spotkinova

#endif // ifndef __spot_kinova_robot_wrapper_hpp__