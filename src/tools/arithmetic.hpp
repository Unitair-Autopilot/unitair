/** @file
Arithmetic / linear algebra tools.
*/
#pragma once
#include <cmath>
#include <limits>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "tools/general.hpp"

/// @name Compiler Configuration
//@{
#ifndef UNITAIR_NATURAL_TYPE
    #define UNITAIR_NATURAL_TYPE size_t
#endif
#ifndef UNITAIR_INTEGER_TYPE
    #define UNITAIR_INTEGER_TYPE int
#endif
#ifndef UNITAIR_REAL_TYPE
    #define UNITAIR_REAL_TYPE double
#endif
//@}

namespace unitair {
namespace tools {

//////////////////////////////////////////////////

/// @name Numeric Types
//@{
using Nat  = UNITAIR_NATURAL_TYPE; ///< Natural number (including zero)
using Int  = UNITAIR_INTEGER_TYPE; ///< Integer
using Real = UNITAIR_REAL_TYPE; ///< Real number
using Quat = Eigen::Quaternion<Real>; ///< Quaternion
//@}

/// @name Linear Algebra Types
//@{
// (dimensions are specified as Int rather than Nat to support Eigen::Dynamic)
template <Int dim>
using RealVec  = Eigen::Matrix<Real, dim, 1>; ///< Real vector (constant dimension)
using RealVecN = RealVec<Eigen::Dynamic>; ///< Real vector (variable dimension)
template <Int rows, Int cols>
using RealMat  = Eigen::Matrix<Real, rows, cols>; ///< Real matrix (constant dimensions)
using RealMatN = RealMat<Eigen::Dynamic, Eigen::Dynamic>; ///< Real matrix (variable dimensions)
//@}

/// @name Numeric Constants
//@{
static Real constexpr YINF    = 1e38; ///< Largest positive YAML float
static Real constexpr INF     = std::numeric_limits<Real>::infinity(); ///< Largest positive number
static Real constexpr EPS     = std::numeric_limits<Real>::epsilon(); ///< Smallest positive number
static Real constexpr PI      = M_PI; ///< Ratio of a circle's circumference to its diameter
static Real constexpr RAD2DEG = 180.0/PI; ///< Factor for scaling radians to degrees
static Real constexpr DEG2RAD = PI/180.0; ///< Factor for scaling degrees to radians
//@}

//////////////////////////////////////////////////

/// Computes the cardinal sine function of a real number x
inline Real sinc(Real x) {
    if (fabs(x) < EPS) {return 1.0;}
    return sin(x)/x;
}

/// Wraps a signed integer i onto the valid range of indexes for an array of length n
inline Nat imod(Int i, Nat n) {
    if (n == 0) {throw std::runtime_error("Calling imod with n=0 implies indexing an empty array.");}
    return ((i % Int(n)) + n) % n;
}

/// Clips a number x between lo and hi
template <class S>
inline S clip(S x, S lo, S hi) {
    if (x < lo) {return lo;}
    else if (x > hi) {return hi;}
    return x;
}

/// Determines if the real scalars x and y are within tol of each other
inline bool is_close(Real x, Real y, Real tol=EPS) {
    return fabs(x - y) < tol;
}

/// Determines if the real vectors x and y are within tol of each other, coefficient-wise
inline bool all_close(RealVecN const& x, RealVecN const& y, Real tol=EPS) {
    if (x.rows() != y.rows()) {return false;}
    for (Int i=0; i<x.rows(); ++i) {
        if (not is_close(x(i), y(i), tol)) {return false;}
    }
    return true;
}

//////////////////////////////////////////////////

/// @name Linear Algebra Types
//@{
/// Wrapper for Eigen::SparseMatrix<Real, format> that enables O(1) access and assigning sub-blocks (at O(n*m) memory cost)
template <int format=Eigen::ColMajor> // Eigen::ColMajor (default) or Eigen::RowMajor
class RealMatS {
    /// @name Derived
    //@{
    Eigen::SparseMatrix<Real, format> matrix; ///< The operable sparse matrix object
    Seq<Seq<Real*>> data; ///< Pointers to the values within matrix for circumventing the binary-search in SparseMatrix::coeffRef
    bool initialized; ///< Whether or not initialize was called (false: insert mode | true: modify mode)
    //@}

public:
    /// @name Core
    //@{
    /// Constructs empty (must resize before any insertion)
    RealMatS() : initialized(false) {};
    /// Constructs with the given shape
    RealMatS(Int rows, Int cols) :
        matrix(rows, cols),
        data(rows, Seq<Real*>(cols, nullptr)),
        initialized(false) {
    }
    /// Constructs by pruning the given dense matrix
    RealMatS(RealMatN const& dense, Real threshold=0.0) :
        matrix(dense.sparseView(threshold, 1.0)),
        data(dense.rows(), Seq<Real*>(dense.cols(), nullptr)),
        initialized(false) {
    }

    /// Changes the shape of the matrix if it hasn't been initialized yet
    void resize(Int rows, Int cols) {
        if (initialized) {throw std::runtime_error("Cannot resize initialized RealMatS.");}
        matrix.resize(rows, cols);
        data.clear(); data.resize(rows, Seq<Real*>(cols, nullptr));
    }

    /// Prunes values with magnitude less than threshold, compresses the matrix, and assigns the data pointers so future access will modify not insert
    void initialize(Real threshold=-1.0) {
        // Can't re-initialize
        if (initialized) {throw std::runtime_error("RealMatS can only be initialized once.");}
        // Prune and compress the matrix (leaves it address-stable)
        if (threshold >= 0.0) {matrix.prune(threshold, 1.0);}
        matrix.makeCompressed();
        // Iterate over nonzeros and point data to matrix
        for (Int i=0; i<matrix.outerSize(); ++i) {
            for (typename Eigen::SparseMatrix<Real, format>::InnerIterator j(matrix, i); j; ++j) {
                data.at(j.row()).at(j.col()) = &matrix.coeffRef(j.row(), j.col());
            }
        }
        // Disallow new insertions (which would mess with addressing)
        initialized = true;
    }

    // Prevent erroneous copying
    RealMatS(RealMatS const&) =delete;
    RealMatS& operator=(RealMatS const&) =delete;
    //@}

    /// @name Access
    //@{
    /// Checks whether the access mode is insert (false) or modify (true)
    inline bool is_initialized() const {return initialized;}

    /// Read-only reference to the whole underlying matrix for use in expressions
    inline Eigen::SparseMatrix<Real, format> const& operator()() const {return matrix;}
    /// Provides the value of the specified coefficient in the matrix (fast dereference if initialized, else slow search)
    inline Real operator()(Int row, Int col) const {
        if (data.at(row).at(col)) {return *data[row][col];}
        else if (initialized) {return 0.0;}
        else {return matrix.coeff(row, col);}
    }

    /// Sets the specified nonzero coefficient (fast modification if initialized, else slow insertion)
    inline RealMatS<format>& set_value(Real value, Int row, Int col) {
        if (data.at(row).at(col)) {*data[row][col] = value;}
        else if (initialized) {throw std::runtime_error("Tried to modify zero ("+std::to_string(row)+","+std::to_string(col)+") in initialized RealMatS.");}
        else {matrix.coeffRef(row, col) = value;}
        return *this;
    }
    /// Sets a nonzero placeholder for the specified coefficient (if uninitialized)
    inline RealMatS<format>& set_value_holder(Int row, Int col) {
        if (initialized) {throw std::runtime_error("Tried to set placeholder in initialized RealMatS.");}
        set_value(INF, row, col);
        return *this;
    }

    /// Sets the given block of nonzeros anchored at the specified top-left indexes (fast modification if initialized, else slow insertion)
    template <Int rows, Int cols>
    inline RealMatS<format>& set_block(RealMat<rows,cols> const& block, Int row0, Int col0) {
        for (Int i=0; i<block.rows(); ++i) {for (Int j=0; j<block.cols(); ++j) {set_value(block(i,j), row0+i, col0+j);}}
        return *this;
    }
    /// Sets a nonzero placeholder for the specified block (if uninitialized)
    template <Int rows, Int cols>
    inline RealMatS<format>& set_block_holder(Int row0, Int col0) {
        if (initialized) {throw std::runtime_error("Tried to set placeholder in initialized RealMatS.");}
        set_block<rows,cols>(RealMat<rows,cols>::Constant(INF), row0, col0);
        return *this;
    }

    /// Sets the given diagonal block (as a vector) anchored at the specified top-left indexes (fast modification if initialized, else slow insertion)
    template <Int dim>
    inline RealMatS<format>& set_diagonal(RealVec<dim> const& diagonal, Int row0, Int col0) {
        for (Int d=0; d<dim; ++d) {set_value(diagonal(d), row0+d, col0+d);}
        return *this;
    }
    /// Sets the given uniform-diagonal block anchored at the specified top-left indexes (fast modification if initialized, else slow insertion)
    template <Int dim>
    inline RealMatS<format>& set_diagonal(Real value, Int row0, Int col0) {
        for (Int d=0; d<dim; ++d) {set_value(value, row0+d, col0+d);}
        return *this;
    }
    /// Sets a nonzero placeholder for the specified diagonal (if uninitialized)
    template <Int dim>
    inline RealMatS<format>& set_diagonal_holder(Int row0, Int col0) {
        if (initialized) {throw std::runtime_error("Tried to set placeholder in initialized RealMatS.");}
        set_diagonal<dim>(INF, row0, col0);
        return *this;
    }

    /// Sets the given skew-symmetric block (as a vector) anchored at the specified top-left indexes (fast modification if initialized, else slow insertion)
    inline RealMatS<format>& set_skewsym(RealVec<3> const& crossvec, Int row0, Int col0) {
        set_value(-crossvec(2), row0+0, col0+1);
        set_value( crossvec(1), row0+0, col0+2);
        set_value( crossvec(2), row0+1, col0+0);
        set_value(-crossvec(0), row0+1, col0+2);
        set_value(-crossvec(1), row0+2, col0+0);
        set_value( crossvec(0), row0+2, col0+1);
        return *this;
    }
    /// Sets a nonzero placeholder for the specified skew-symmetric block (if uninitialized)
    inline RealMatS<format>& set_skewsym_holder(Int row0, Int col0) {
        if (initialized) {throw std::runtime_error("Tried to set placeholder in initialized RealMatS.");}
        set_skewsym(RealVec<3>::Constant(INF), row0, col0);
        return *this;
    }
    //@}
};
//@}

//////////////////////////////////////////////////

} // namespace tools
} // namespace unitair
