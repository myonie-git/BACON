#ifndef OBB_H
#define OBB_H

#include <iostream>
#include <Eigen/Geometry>
#include <limits>
 
#include "fcl/common/types.h"
#include "fcl/math/geometry.h"

#include "types.h"

template <typename S_>
class OBB
{
public:
    using S = S_;

    //旋转的角度 3x3矩阵   
    Matrix3<S> axis;

    Vector3<S> To;

    Vector3<S> extent;

    OBB(){

    }

    OBB(const Matrix3<S>& axis_,
        const Vector3<S>& center_,
        const Vector3<S>& extent_)
        : axis(axis_), To(center_), extent(extent_){

        }

    bool overlap(const OBB<S>& other) const;
    bool overlap(const OBB<S>& other, int &timer) const;
    bool overlap_gpu(const OBB<S>& other) const;

    /// @brief Check whether two AABB are overlap
    bool overlap_obbcount(const OBB<S>& other, int &obb_count) const{
        obb_count++;
        return overlap(other);
    };

    OBB<S>& operator +=(const Vector3<S>& p);

    OBB<S>& operator += (const OBB<S>& other);

    OBB<S> operator + (const OBB<S>& other) const;
        
    /// @brief Check whether the OBB contains a point.
    bool contain(const Vector3<S>& p) const;

    void printOBB() const;

    void computeVertices() const;

    S width() const;

    S height() const;

    S depth() const;

    S volume() const;

    S size() const;

    const Vector3<S> center() const;
};

/// @brief Compute the 8 vertices of a OBB
template <typename S>
void computeVertices(const OBB<S>& b, Vector3<S> vertices[8]);

/// @brief OBB merge method when the centers of two smaller OBB are far away
template <typename S>
OBB<S> merge_largedist(const OBB<S>& b1, const OBB<S>& b2);

/// @brief OBB merge method when the centers of two smaller OBB are close
template <typename S>
OBB<S> merge_smalldist(const OBB<S>& b1, const OBB<S>& b2);

/// @brief Translate the OBB bv
template <typename S, typename Derived>
OBB<S> translate(
    const OBB<S>& bv, const Eigen::MatrixBase<Derived>& t);

/// @brief Check collision between two obbs, b1 is in configuration (R0, T0) and
/// b2 is in identity.
template <typename S, typename DerivedA, typename DerivedB>
bool overlap(const Eigen::MatrixBase<DerivedA>& R0,
             const Eigen::MatrixBase<DerivedB>& T0,
             const OBB<S>& b1, const OBB<S>& b2);

template <typename S>
bool obbDisjoint(
    const Matrix3<S>& B,
    const Vector3<S>& T,
    const Vector3<S>& a,
    const Vector3<S>& b,
    int &timer);


/// @brief Check collision between two boxes: the first box is in configuration
/// (R, T) and its half dimension is set by a; the second box is in identity
/// configuration and its half dimension is set by b.
template <typename S>
bool obbDisjoint(
    const Matrix3<S>& B,
    const Vector3<S>& T,
    const Vector3<S>& a,
    const Vector3<S>& b);
    

/// @brief Check collision between two boxes: the first box is in configuration
/// (R, T) and its half dimension is set by a; the second box is in identity
/// configuration and its half dimension is set by b.
template <typename S>
bool obbDisjoint(
    const Transform3<S>& tf,
    const Vector3<S>& a,
    const Vector3<S>& b);

template <typename S>
bool obbDisjoint_gpu(const S* B, const S* T,
                     const S* a, const S* b);


#endif