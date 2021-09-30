/** @file
General tools.
*/
#pragma once
#include <iostream>
#include <memory>
#include <vector>
#include <map>

namespace unitair {
namespace tools {

//////////////////////////////////////////////////

/// @name Standard Types
//@{
template <class T> using Ptr = std::shared_ptr<T>; ///< Smart pointer
template <class T> using Seq = std::vector<T>; ///< Sequential container

using Str = std::string; ///< Character string
template <class T> using Dict = std::map<Str, T>; ///< Dictionary
//@}

//////////////////////////////////////////////////

} // namespace tools
} // namespace unitair
