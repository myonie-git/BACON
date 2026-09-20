
#ifndef AABB_H
#define AABB_H

#include <iostream>
#include <Eigen/Geometry>
#include <limits>
#include "types.h"
#include <fstream>
#include <sstream>
#include <string>

template <typename S_>
class AABB
{
public:
    using S = S_;

    Vector3<S> min_;
    Vector3<S> max_;

    AABB()
        : min_(Vector3<S>::Constant(std::numeric_limits<S>::max())),
          max_(Vector3<S>::Constant(-std::numeric_limits<S>::max()))
    {
    }

    // AABB(const Vector3<S>& v);
    AABB(const Vector3<S>& a, const Vector3<S>& b)
        : min_(a.cwiseMin(b)), max_(a.cwiseMax(b))
    {
    }

    /// @brief Check whether two AABB are overlap
    bool overlap(const AABB<S>& other) const;

    /// @brief Check whether two AABB are overlap
    bool overlap(const AABB<S>& other, int &timer) const;

    /// @brief Check whether two AABB are overlap
    bool overlap_obbcount(const AABB<S>& other, int &aabb_count) const{
        aabb_count++;
        return overlap(other);
    };

    /// @brief Merge the AABB and another AABB
    AABB<S>& operator += (const AABB<S>& other);

    /// @brief Return the merged AABB of current AABB and the other one
    AABB<S> operator + (const AABB<S>& other) const;

    void printAABB() const;

    void printEnv(std::ofstream& outfile) const;

    /// @brief Width of the AABB
    S width() const;

    /// @brief Height of the AABB
    S height() const;

    /// @brief Depth of the AABB
    S depth() const;

    S size() const;

    /// @brief Center of the AABB
    Vector3<S> center() const;
        
};

#endif