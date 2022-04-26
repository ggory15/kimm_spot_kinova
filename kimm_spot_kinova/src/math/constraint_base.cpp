#include <kimm_spot_kinova/math/constraint_base.hpp>

using namespace spotkinova::math;

ConstraintBase::ConstraintBase(const std::string & name):
  m_name(name){}

ConstraintBase::ConstraintBase(const std::string & name,
                               const unsigned int rows,
                               const unsigned int cols):
  m_name(name)
{
  m_A = Matrix::Zero(rows, cols);
}

ConstraintBase::ConstraintBase(const std::string & name,
                               ConstRefMatrix A):
  m_name(name),
  m_A(A)
{}

const std::string & ConstraintBase::name() const
{
  return m_name;
}

const Matrix & ConstraintBase::matrix() const
{
  return m_A;
}

Matrix & ConstraintBase::matrix()
{
  return m_A;
}

bool ConstraintBase::setMatrix(ConstRefMatrix A)
{
  assert(m_A.cols()==A.cols());
  assert(m_A.rows()==A.rows());
  m_A = A;
  return true;
}
