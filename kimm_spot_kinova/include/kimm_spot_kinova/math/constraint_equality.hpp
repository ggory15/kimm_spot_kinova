#ifndef __math_constraint_equality_hpp__
#define __math_constraint_equality_hpp__

#include "kimm_spot_kinova/math/constraint_base.hpp"

namespace spotkinova{
    namespace math{
        class ConstraintEquality : public ConstraintBase
        {
            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW

                ConstraintEquality(const std::string & name);

                ConstraintEquality(const std::string & name,
                                    const unsigned int rows,
                                    const unsigned int cols);

                ConstraintEquality(const std::string & name,
                                    ConstRefMatrix A,
                                    ConstRefVector b);
                virtual ~ConstraintEquality() {}

                unsigned int rows() const;
                unsigned int cols() const;
                void resize(const unsigned int r, const unsigned int c);

                bool isEquality() const;
                bool isInequality() const;
                bool isBound() const;

                const Vector & vector()     const;
                const Vector & lowerBound() const;
                const Vector & upperBound() const;

                Vector & vector();
                Vector & lowerBound();
                Vector & upperBound();

                bool setVector(ConstRefVector b);
                bool setLowerBound(ConstRefVector lb);
                bool setUpperBound(ConstRefVector ub);

                bool checkConstraint(ConstRefVector x, double tol=1e-6) const;

            protected:
                Vector m_b;
        };
    }
}
#endif