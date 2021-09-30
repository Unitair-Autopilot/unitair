/** @file
Probability tools.
*/
#pragma once
#include <random>
#include "tools/general.hpp"
#include "tools/arithmetic.hpp"

namespace unitair {
namespace tools {

//////////////////////////////////////////////////

/// Samples a scalar uniform distribution from lo to hi
inline Real sample_uniform(Real lo=0.0, Real hi=1.0) {
    static std::mt19937 rng(time(nullptr));
    static std::uniform_real_distribution<Real> uniform;
    return lo + (hi-lo)*uniform(rng);
}

/// Samples a scalar normal distribution with the given mean and standard-deviation
inline Real sample_normal(Real mean=0.0, Real stdev=1.0) {
    static std::mt19937 rng(time(nullptr));
    static std::normal_distribution<Real> normal;
    return mean + stdev*normal(rng);
}

/// Samples a binary Bernoulli distribution with the given bias parameter (chance of true)
inline bool sample_bernoulli(Real p=0.5) {
    return sample_uniform(0.0, 1.0) < p;
}

/// Creates a reasonably random unit-quaternion with positive real-part (no guarantee on the distribution)
inline Quat arbitrary_quat() {
    return Quat(fabs(sample_normal()), sample_normal(), sample_normal(), sample_normal()).normalized();
}

//////////////////////////////////////////////////

/*
These declarations (adapted from https://github.com/beniz/eigenmvn/blob/master/eigenmvn.h) prepare
Eigen for use with multivariate random sampling. They must occur in the Eigen::internal namespace.
*/
} // namespace tools
} // namespace unitair
/// @cond HIDDEN_SYMBOLS
namespace Eigen {
namespace internal {
    template <typename Scalar> struct RandnOp {
        static std::mt19937 rng;
        mutable std::normal_distribution<Scalar> normal;
        EIGEN_EMPTY_STRUCT_CTOR(RandnOp)
        template <typename Index> Scalar const operator()(Index, Index=0) const {return normal(rng);}
        void seed(uint64_t const& s) {rng.seed(s);}
    };
    template <typename Scalar> std::mt19937 RandnOp<Scalar>::rng;
    template <typename Scalar> struct functor_traits<RandnOp<Scalar>> {
        enum {Cost=50*NumTraits<Scalar>::MulCost, PacketAccess=false, IsRepeatable=false};
    };
} // namespace internal
} // namespace Eigen
/// @endcond
namespace unitair {
namespace tools {

//////////////////////////////////////////////////

/// Provides samples from a real multivariate normal distribution by linearly transforming independent scalar samples
template <Int dim>
class Gaussian {
    /// @name Given
    //@{
    RealVec<dim> mean; ///< Mean vector of the distribution
    RealMat<dim,dim> covar; ///< Covariance matrix of the distribution
    bool posdef; ///< Whether or not the covariance matrix is known to be positive-*definite* (when in doubt, assume false)
    //@}

    /// @name Derived
    //@{
    RealMat<dim,dim> transformation; ///< Correlating transformation matrix
    Eigen::internal::RandnOp<Real> randnop; ///< Generator of zero-mean, unit-variance, independent, scalar normal samples
    //@}

public:
    /// @name Core
    //@{
    /// Constructs with the given parameters and then computes the necessary transformation matrix
    Gaussian(RealVec<dim> const& mean, RealMat<dim,dim> const& covar, bool posdef=false) :
        mean(mean),
        covar(covar),
        posdef(posdef) {
        randnop.seed(time(nullptr));
        if (posdef) {
            // Use fast Cholesky decomposition if covar is positive-definite
            Eigen::LLT<RealMat<dim,dim>> cholesky(covar);
            if (cholesky.info() == Eigen::Success) {transformation = cholesky.matrixL();}
            else {throw std::runtime_error("Cholesky failed in instance of class Gaussian. Try posdef=false.");}
        } else {
            // Use robust eigen decomposition if covar is positive-semidefinite
            Eigen::SelfAdjointEigenSolver<RealMat<dim,dim>> decomp(covar);
            transformation.noalias() = decomp.eigenvectors() * decomp.eigenvalues().cwiseMax(0.0).cwiseSqrt().asDiagonal();
        }
    }
    //@}

    /// @name Utilities
    //@{
    /// Draw n samples from the gaussian by transforming independent samples, and return them as columns in a dim-by-n matrix
    RealMat<dim, Eigen::Dynamic> sample(Int n) const {
        return (transformation * RealMat<dim, Eigen::Dynamic>::NullaryExpr(dim, n, randnop)).colwise() + mean;
    }
    //@}

    /// @name Access
    //@{
    inline RealVec<dim> const& get_mean() const {return mean;}
    inline RealMat<dim,dim> const& get_covar() const {return covar;}
    inline bool is_posdef() const {return posdef;}
    //@}
};

//////////////////////////////////////////////////

} // namespace tools
} // namespace unitair
