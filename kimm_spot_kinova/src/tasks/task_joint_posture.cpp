#include "kimm_spot_kinova/tasks/task_joint_posture.hpp"
#include "kimm_spot_kinova/robots/robot-wrapper.hpp"

namespace spotkinova
{
  namespace tasks
  {
    using namespace math;
    using namespace trajectory;
    using namespace pinocchio;

    TaskJointPosture::TaskJointPosture(const std::string & name,
                                     RobotWrapper & robot):
      TaskMotion(name, robot), m_constraint(name, robot.na(), robot.nv()), m_ref(robot.na()), m_spot(false)
    {
      if (robot.na() > 7)
        m_spot =true;

      if (!m_spot){
        m_constraint.resize(7, 7);
        m_ref.resize(7);
        m_Kp.setZero(7);
        m_Kd.setZero(7);
        Vector m = Vector::Ones(7);
        setMask(m);
      }
      else{
        m_constraint.resize(7, 26);
        m_ref.resize(7);
        m_Kp.setZero(7);
        m_Kd.setZero(7);
        Vector m = Vector::Ones(7);
        setMask(m);
      }
    }

    const Vector & TaskJointPosture::mask() const
    {
      return m_mask;
    }

    void TaskJointPosture::mask(const Vector & m)
    {
      // std::cerr<<"The method TaskJointPosture::mask is deprecated. Use TaskJointPosture::setMask instead.\n";
      return setMask(m);
    }

    void TaskJointPosture::setMask(ConstRefVector m)
    {
      assert(m.size()==7);
      if (!m_spot){
        m_mask = m;
        const Vector::Index dim = static_cast<Vector::Index>(m.sum());
        Matrix S = Matrix::Zero(dim, m_robot.nv());
        m_activeAxes.resize(dim);
        unsigned int j=0;
        for(unsigned int i=0; i<m.size(); i++)
          if(m(i)!=0.0)
          {
            assert(m(i)==1.0);
            S(j, i) = 1.0;
            m_activeAxes(j) = i;
            j++;
          }
        m_constraint.resize((unsigned int)dim, m_robot.nv());
        m_constraint.setMatrix(S);
      }
      else{
        m_mask = m;
        const Vector::Index dim = static_cast<Vector::Index>(m.sum());
        Matrix S = Matrix::Zero(dim, m_robot.nv());
        m_activeAxes.resize(dim);
        unsigned int j=0;
        for(unsigned int i=0; i<m.size(); i++)
          if(m(i)!=0.0)
          {
            assert(m(i)==1.0);
            S(j, i+6) = 1.0;
            m_activeAxes(j) = i;
            j++;
          }
        m_constraint.resize((unsigned int)dim, m_robot.nv());
        m_constraint.setMatrix(S);
      }
    }

    int TaskJointPosture::dim() const
    {
      return (int)m_mask.sum();
    }

    const Vector & TaskJointPosture::Kp(){ return m_Kp; }

    const Vector & TaskJointPosture::Kd(){ return m_Kd; }

    void TaskJointPosture::Kp(ConstRefVector Kp)
    {
      assert(Kp.size()== 7);
      m_Kp = Kp;
    }

    void TaskJointPosture::Kd(ConstRefVector Kd)
    {
      assert(Kd.size()==7);
      m_Kd = Kd;
    }

    void TaskJointPosture::setReference(const TrajectorySample & ref)
    {
      assert(ref.pos.size()==7);
      assert(ref.vel.size()==7);
      assert(ref.acc.size()==7);
      m_ref = ref;
    }

    const TrajectorySample & TaskJointPosture::getReference() const
    {
      return m_ref;
    }

    const Vector & TaskJointPosture::getDesiredAcceleration() const
    {
      return m_a_des;
    }

    Vector TaskJointPosture::getAcceleration(ConstRefVector dv) const
    {
      return m_constraint.matrix()*dv;
    }

    const Vector & TaskJointPosture::position_error() const
    {
      return m_p_error;
    }

    const Vector & TaskJointPosture::velocity_error() const
    {
      return m_v_error;
    }

    const Vector & TaskJointPosture::position() const
    {
      return m_p;
    }

    const Vector & TaskJointPosture::velocity() const
    {
      return m_v;
    }

    const Vector & TaskJointPosture::position_ref() const
    {
      return m_ref.pos;
    }

    const Vector & TaskJointPosture::velocity_ref() const
    {
      return m_ref.vel;
    }

    const ConstraintBase & TaskJointPosture::getConstraint() const
    {
      return m_constraint;
    }

    const ConstraintBase & TaskJointPosture::compute(const double ,
                                                    ConstRefVector q,
                                                    ConstRefVector v,
                                                    Data & )
    {
      using namespace std;
      if (!m_spot){
        m_p = q;
        m_p_error = m_p - m_ref.pos;
        for (int i=0; i<7; i++){
          if (i == 0 || i == 2 || i == 4 || i == 6){
            if (fabs(m_p_error(i)) > M_PI){ 
              if (m_p(i) < 0.0)
                m_p_error(i) = -m_p_error(i)-2.0*M_PI; // -170 170 : m_p_error -340: -20
              else
                m_p_error(i) = -m_p_error(i)+2.0*M_PI; // 170 -170 : m_p_errro 340: 20
            }
          }
        }

        m_a_des = - m_Kp.cwiseProduct(m_p_error);
      }
      else{
        m_p = q.segment(7,7);
        m_p_error = m_p - m_ref.pos;
        for (int i=0; i<7; i++){
          if (i == 0 || i == 2 || i == 4 || i == 6){
            if (fabs(m_p_error(i)) > M_PI){ 
              if (m_p(i) <= 0.0)
                m_p_error(i) = -m_p_error(i)-2.0*M_PI; // -170 170 : m_p_error -340: -20
              else
                m_p_error(i) = -m_p_error(i)+2.0*M_PI; // 170 -170 : m_p_errro 340: 20
                
            }
          }
        }

        m_a_des = - m_Kp.cwiseProduct(m_p_error);
      }
      for(unsigned int i=0; i<m_activeAxes.size(); i++)
        m_constraint.vector()(i) = m_a_des(m_activeAxes(i));

      // using namespace std;
      // cout << m_constraint.matrix() << endl;
      return m_constraint;
    }    
  }
}
