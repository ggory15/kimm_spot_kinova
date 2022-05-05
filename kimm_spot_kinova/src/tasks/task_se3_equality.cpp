#include "kimm_spot_kinova/tasks/task_se3_equality.hpp"
#include "kimm_spot_kinova/robots/robot-wrapper.hpp"
#include "kimm_spot_kinova/math/util.hpp"
#include <Eigen/SVD>

using namespace std;
namespace spotkinova
{
  namespace tasks
  {
    using namespace math;
    using namespace trajectory;
    using namespace pinocchio;

    TaskSE3Equality::TaskSE3Equality(const std::string & name,
                                     RobotWrapper & robot,
                                     const std::string & frameName,
                                     const Eigen::Vector3d & offset):
      TaskMotion(name, robot),
      m_frame_name(frameName),
      m_constraint(name, 6, 7),
      m_ref(12, 6),
      m_spot(false),
      m_offset(offset)
    {
      if (robot.na() > 7)
        m_spot=true;
      assert(m_robot.model().existFrame(frameName));
      m_frame_id = m_robot.model().getFrameId(frameName);

      m_v_ref.setZero();
      m_a_ref.setZero();
      m_M_ref.setIdentity();
      m_wMl.setIdentity();
      m_p_error_vec.setZero(6);
      m_v_error_vec.setZero(6);
      m_p.resize(12);
      m_v.resize(6);
      m_p_ref.resize(12);
      m_v_ref_vec.resize(6);
      m_Kp.setZero(6);
      m_Kd.setZero(6);
      m_a_des.setZero(6);

      if (!m_spot){
        m_J.setZero(6, 7);
        m_J_rotated.setZero(6, 7);
      }
      else{
        m_J.setZero(6, 25);
        m_J_rotated.setZero(6, 25);
        m_J_reduced.setZero(6, 7);
      }
      m_mask.resize(6);
      m_mask.fill(1.);
      setMask(m_mask);
     
    }

    void TaskSE3Equality::setMask(math::ConstRefVector mask)
    {
      TaskMotion::setMask(mask);
      int n = dim();

      m_constraint.resize(n, (unsigned int)m_J.cols());
      m_p_error_masked_vec.resize(n);
      m_v_error_masked_vec.resize(n);
      m_drift_masked.resize(n);
      m_a_des_masked.resize(n);
    }

    int TaskSE3Equality::dim() const
    {
      return (int)m_mask.sum();
    }

    const Vector & TaskSE3Equality::Kp() const { return m_Kp; }

    const Vector & TaskSE3Equality::Kd() const { return m_Kd; }

    void TaskSE3Equality::Kp(ConstRefVector Kp)
    {
      assert(Kp.size()==6);
      m_Kp = Kp;
    }

    void TaskSE3Equality::Kd(ConstRefVector Kd)
    {
      assert(Kd.size()==6);
      m_Kd = Kd;
    }

    void TaskSE3Equality::setReference(TrajectorySample & ref)
    {
      m_ref = ref;
      vectorToSE3(ref.pos, m_M_ref);
      m_v_ref = Motion(ref.vel);
      m_a_ref = Motion(ref.acc);
    }

    const TrajectorySample & TaskSE3Equality::getReference() const
    {
      return m_ref;
    }

    const Vector & TaskSE3Equality::position_error() const
    {
      return m_p_error_masked_vec;
    }

    const Vector & TaskSE3Equality::velocity_error() const
    {
      return m_v_error_masked_vec;
    }

    const Vector & TaskSE3Equality::position() const
    {
      return m_p;
    }

    const Vector & TaskSE3Equality::velocity() const
    {
      return m_v;
    }

    const Vector & TaskSE3Equality::position_ref() const
    {
      return m_p_ref;
    }

    const Vector & TaskSE3Equality::velocity_ref() const
    {
      return m_v_ref_vec;
    }

    const Vector & TaskSE3Equality::getDesiredAcceleration() const
    {
      return m_a_des_masked;
    }

    Vector TaskSE3Equality::getAcceleration(ConstRefVector dv) const
    {
      return m_constraint.matrix()*dv + m_drift_masked;
    }

    Index TaskSE3Equality::frame_id() const
    {
      return m_frame_id;
    }

    const ConstraintBase & TaskSE3Equality::getConstraint() const
    {
      return m_constraint;
    }

    const ConstraintBase & TaskSE3Equality::compute(const double ,
                                                    ConstRefVector ,
                                                    ConstRefVector ,
                                                    Data & data)
    {
      SE3 oMi, oMi_prev;
      Motion v_frame;
      m_robot.framePosition(data, m_frame_id, oMi_prev);
      m_robot.frameVelocity(data, m_frame_id, v_frame);
      m_robot.frameJacobianLocal(data, m_frame_id, m_J);

      SE3 T_offset;
      T_offset.setIdentity();
      T_offset.translation(m_offset); 
      oMi = oMi_prev * T_offset;
      Eigen::MatrixXd Adj_mat(6,6);
      Adj_mat.setIdentity();
      Eigen::Vector3d offset_local;
      offset_local = m_offset; 

      Adj_mat(0, 4) = -offset_local(2);
      Adj_mat(0, 5) = offset_local(1);
      Adj_mat(1, 3) = offset_local(2);
      Adj_mat(1, 5) = -offset_local(0);
      Adj_mat(2, 3) = -offset_local(1);
      Adj_mat(2, 4) = offset_local(0);
      Adj_mat.topRightCorner(3,3) = -Adj_mat.topRightCorner(3,3);

      if (m_spot){
        m_J.topLeftCorner(6, 6).setZero();
        m_J.topRightCorner(6, 12).setZero();
      }

      errorInSE3(oMi, m_M_ref, m_p_error);          // pos err in local frame
      SE3ToVector(m_M_ref, m_p_ref);
      SE3ToVector(oMi, m_p);

      m_wMl.rotation(oMi.rotation());
      m_p_error_vec = m_p_error.toVector();
      m_v_error =  m_wMl.actInv(m_v_ref) - v_frame; 
      m_a_des = m_Kp.cwiseProduct(m_p_error_vec);

      v_frame.linear() = v_frame.linear() + Adj_mat.topRightCorner(3,3) * v_frame.angular();
      m_J = Adj_mat * m_J;

      
      int idx = 0;
      for (int i = 0; i < 6; i++) {
        if (m_mask(i) != 1.) continue;

        m_constraint.matrix().row(idx) = m_J.row(i);
        m_constraint.vector().row(idx) = (m_a_des).row(i);
        m_a_des_masked(idx)            = m_a_des(i);
        m_drift_masked(idx)            = m_drift.toVector()(i);
        m_p_error_masked_vec(idx)      = m_p_error_vec(i);
        m_v_error_masked_vec(idx)      = m_v_error_vec(i);
  
        idx += 1;
        
      }
  
      return m_constraint;
    }    
  }
}
