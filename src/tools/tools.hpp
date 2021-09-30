/** @file
Convenience header to include all Unitair tools at once
and namespace-expose extremely common types.
*/
#pragma once
#include "tools/general.hpp"
#include "tools/arithmetic.hpp"
#include "tools/geometry.hpp"
#include "tools/geodesy.hpp"
#include "tools/probability.hpp"
#include "tools/conversions.hpp"

namespace unitair {

//////////////////////////////////////////////////

/// @name Essential Types
//@{
using tools::Ptr; ///< Smart pointer
using tools::Seq; ///< Sequential container

using tools::Str; ///< Character string
using tools::Dict; ///< Dictionary

using tools::Nat; ///< Natural number (including zero)
using tools::Int; ///< Integer
using tools::Real; ///< Real number
using tools::Quat; ///< Quaternion

using tools::RealVec; ///< Real vector (constant dimension)
using tools::RealVecN; ///< Real vector (variable dimension)

using tools::RealMat; ///< Real matrix (constant dimensions)
using tools::RealMatN; ///< Real matrix (variable dimensions)
using tools::RealMatS; ///< Real sparse matrix (variable dimensions)
//@}

//////////////////////////////////////////////////

} // namespace unitair
