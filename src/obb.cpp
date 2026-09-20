#include "obb.h"
#include "aabb.h"
#include "types.h"

extern template
class OBB<double>;

//==============================================================================
extern template
void computeVertices(const OBB<double>& b, Vector3<double> vertices[8]);

//==============================================================================
extern template
OBB<double> merge_largedist(const OBB<double>& b1, const OBB<double>& b2);

//==============================================================================
extern template
OBB<double> merge_smalldist(const OBB<double>& b1, const OBB<double>& b2);

//==============================================================================
extern template
bool obbDisjoint(
    const Matrix3<double>& B,
    const Vector3<double>& T,
    const Vector3<double>& a,
    const Vector3<double>& b);

//==============================================================================
extern template
bool obbDisjoint(
    const Transform3<double>& tf,
    const Vector3<double>& a,
    const Vector3<double>& b);

//==============================================================================
template <typename S>
bool OBB<S>::overlap(const OBB<S>& other, int &timer) const
{
  /// compute the relative transform that takes us from this->frame to
  /// other.frame

  // std::cout << "OBB" << std::endl;
  Vector3<S> t = other.To - To;
  Vector3<S> T(
        axis.col(0).dot(t), axis.col(1).dot(t), axis.col(2).dot(t));
  Matrix3<S> R = axis.transpose() * other.axis;
  timer += 47;
  return !obbDisjoint(R, T, extent, other.extent, timer);

  // const Vector3<S>& sphere_center = other.To;
  // const S sphere_radius2 = other.extent.squaredNorm();

  // // 2) 检测该球体是否与 self 的 OBB 相交：计算球心到 OBB 的最近点距离
  // //    将球心投影到 self 的局部坐标系，然后在 [-extent, extent] 上截断。
  // Vector3<S> d = sphere_center - To;
  // Vector3<S> local_center(
  //       axis.col(0).dot(d), axis.col(1).dot(d), axis.col(2).dot(d));

  // Vector3<S> closest = local_center;
  // for(int i = 0; i < 3; ++i)
  // {
  //   if(closest[i] < -extent[i]) closest[i] = -extent[i];
  //   else if(closest[i] > extent[i]) closest[i] = extent[i];
  // }

  // const Vector3<S> diff = local_center - closest;
  // const S dist2 = diff.squaredNorm();

  // // 粗粒度代价计数（用于对比不同碰撞检测策略，非精确 FLOPs）
  // timer += 30;
  // return dist2 <= sphere_radius2;

}

//==============================================================================
template <typename S>
bool OBB<S>::overlap(const OBB<S>& other) const
{
  /// compute the relative transform that takes us from this->frame to
  /// other.frame
  // std::cout << "OBB" << std::endl;
  Vector3<S> t = other.To - To;
  Vector3<S> T(
        axis.col(0).dot(t), axis.col(1).dot(t), axis.col(2).dot(t));
  Matrix3<S> R = axis.transpose() * other.axis;

  return !obbDisjoint(R, T, extent, other.extent);

  
  // const Vector3<S>& sphere_center = other.To;
  // const S sphere_radius2 = other.extent.squaredNorm();

  // // 2) 检测该球体是否与 self 的 OBB 相交：计算球心到 OBB 的最近点距离
  // //    将球心投影到 self 的局部坐标系，然后在 [-extent, extent] 上截断。
  // Vector3<S> d = sphere_center - To;
  // Vector3<S> local_center(
  //       axis.col(0).dot(d), axis.col(1).dot(d), axis.col(2).dot(d));

  // Vector3<S> closest = local_center;
  // for(int i = 0; i < 3; ++i)
  // {
  //   if(closest[i] < -extent[i]) closest[i] = -extent[i];
  //   else if(closest[i] > extent[i]) closest[i] = extent[i];
  // }

  // const Vector3<S> diff = local_center - closest;
  // const S dist2 = diff.squaredNorm();
  // return dist2 <= sphere_radius2;
}

//==============================================================================
template <typename S>
bool OBB<S>::overlap_gpu(const OBB<S>& other) const
{
  /// compute the relative transform that takes us from this->frame to
  /// other.frame
  const S* axis_ptr = axis.data();
  const S* other_axis_ptr = other.axis.data();

  const S* to_ptr = To.data();
  const S* other_to_ptr = other.To.data();

  const S* extent_ptr = extent.data();
  const S* other_extent_ptr =  other.extent.data();

  S t[3];
  for(int i = 0; i < 3; i++){
    t[i] = other_to_ptr[i] - to_ptr[i];
  }

  S T[3];
  for(int i = 0; i < 3; i++){
    T[i] = axis_ptr[i * 3 + 0] * t[0] +
           axis_ptr[i * 3 + 1] * t[1] + 
           axis_ptr[i * 3 + 2] * t[2];
  }

  S R0_trans[9];
  for(int i = 0; i < 3; i++){
    for(int j = 0; j < 3; j++){
      R0_trans[i * 3 + j] = axis_ptr[j * 3 + i];
    }
  }


  //卷积 
  S R[9];
  for(int i = 0; i < 3; i++){
    for(int j = 0; j < 3; j++){
      R[i * 3 + j] = 0;
      for(int k = 0; k < 3; k++){
        R[i * 3 + j] += R0_trans[k * 3 + j] * other_axis_ptr[i * 3 + k];
      }
    }
  }

  return !obbDisjoint_gpu(R, T, extent_ptr,  other_extent_ptr);
}

//==============================================================================
template <typename S>
bool OBB<S>::contain(const Vector3<S>& p) const
{
  Vector3<S> local_p = p - To;
  S proj = local_p.dot(axis.col(0));
  if((proj > extent[0]) || (proj < -extent[0]))
    return false;

  proj = local_p.dot(axis.col(1));
  if((proj > extent[1]) || (proj < -extent[1]))
    return false;

  proj = local_p.dot(axis.col(2));
  if((proj > extent[2]) || (proj < -extent[2]))
    return false;

  return true;
}

template <typename S>
void OBB<S>::printOBB() const {
    // 打印中心点
    std::cout << "Center: (" << center()[0] << ", " << center()[1] << ", " << center()[2] << ")" << std::endl;

    // 打印轴向矩阵
    std::cout << "Axes:" << std::endl;
    for (int i = 0; i < 3; ++i) {
        std::cout << "  Axis " << i + 1 << ": ("
                  << axis(0, i) << ", "
                  << axis(1, i) << ", "
                  << axis(2, i) << ")" << std::endl;
    }

    // 打印半长度
    std::cout << "Extents: (" << extent[0] << ", " << extent[1] << ", " << extent[2] << ")" << std::endl;
}

//==============================================================================
template <typename S>
void OBB<S>::computeVertices() const
{
  Vector3<S> vertices[8];

  Vector3<S> extAxis0 = axis.col(0) * extent[0];
  Vector3<S> extAxis1 = axis.col(1) * extent[1];
  Vector3<S> extAxis2 = axis.col(2) * extent[2];

  vertices[0] = To - extAxis0 - extAxis1 - extAxis2;
  vertices[1] = To + extAxis0 - extAxis1 - extAxis2;
  vertices[2] = To + extAxis0 + extAxis1 - extAxis2;
  vertices[3] = To - extAxis0 + extAxis1 - extAxis2;
  vertices[4] = To - extAxis0 - extAxis1 + extAxis2;
  vertices[5] = To + extAxis0 - extAxis1 + extAxis2;
  vertices[6] = To + extAxis0 + extAxis1 + extAxis2;
  vertices[7] = To - extAxis0 + extAxis1 + extAxis2;

  // 打印每个顶点的坐标
  for(int i = 0; i < 8; ++i)
  {
    std::cout << "Vertex " << i << ": (" 
              << vertices[i][0] << ", "
              << vertices[i][1] << ", "
              << vertices[i][2] << ")" << std::endl;
  }
}

//==============================================================================
template <typename S>
OBB<S>& OBB<S>::operator +=(const Vector3<S>& p)
{
  OBB<S> bvp(axis, p, Vector3<S>::Zero());
  *this += bvp;

  return *this;
}

//==============================================================================
template <typename S>
OBB<S>& OBB<S>::operator +=(const OBB<S>& other)
{
  *this = *this + other;

  return *this;
}

//==============================================================================
template <typename S>
OBB<S> OBB<S>::operator +(const OBB<S>& other) const
{
  Vector3<S> center_diff = To - other.To;
  S max_extent = std::max(std::max(extent[0], extent[1]), extent[2]);
  S max_extent2 = std::max(std::max(other.extent[0], other.extent[1]), other.extent[2]);
  if(center_diff.norm() > 2 * (max_extent + max_extent2))
  {
    return merge_largedist(*this, other);
  }
  else
  {
    return merge_smalldist(*this, other);
  }
}

//==============================================================================
template <typename S>
S OBB<S>::width() const
{
  return 2 * extent[0];
}

//==============================================================================
template <typename S>
S OBB<S>::height() const
{
  return 2 * extent[1];
}

//==============================================================================
template <typename S>
S OBB<S>::depth() const
{
  return 2 * extent[2];
}

//==============================================================================
template <typename S>
S OBB<S>::volume() const
{
  return width() * height() * depth();
}

//==============================================================================
template <typename S>
S OBB<S>::size() const
{
  return extent.squaredNorm();
}

//==============================================================================
template <typename S>
const Vector3<S> OBB<S>::center() const
{
  return To;
}

//==============================================================================
template <typename S>
OBB<S> merge_largedist(const OBB<S>& b1, const OBB<S>& b2)
{
  Vector3<S> vertex[16];
  computeVertices(b1, vertex);
  computeVertices(b2, vertex + 8);
  Matrix3<S> M;
  Matrix3<S> E;
  Vector3<S> s(0, 0, 0);

  OBB<S> b;
  b.axis.col(0) = b1.To - b2.To;
  b.axis.col(0).normalize();

  Vector3<S> vertex_proj[16];
  for(int i = 0; i < 16; ++i)
  {
    vertex_proj[i] = vertex[i];
    vertex_proj[i].noalias() -= b.axis.col(0) * vertex[i].dot(b.axis.col(0));
  }

  fcl::getCovariance<S>(vertex_proj, nullptr, nullptr, nullptr, 16, M);
  fcl::eigen_old(M, s, E);

  int min, mid, max;
  if (s[0] > s[1])
  {
    max = 0;
    min = 1;
  }
  else
  {
    min = 0;
    max = 1;
  }

  if (s[2] < s[min])
  {
    mid = min;
    min = 2;
  }
  else if (s[2] > s[max])
  {
    mid = max;
    max = 2;
  }
  else
  {
    mid = 2;
  }

  b.axis.col(1) << E.col(0)[max], E.col(1)[max], E.col(2)[max];
  b.axis.col(2) << E.col(0)[mid], E.col(1)[mid], E.col(2)[mid];

  // set obb centers and extensions
  fcl::getExtentAndCenter<S>(
        vertex, nullptr, nullptr, nullptr, 16, b.axis, b.To, b.extent);

  return b;
}

//==============================================================================
template <typename S>
void computeVertices(const OBB<S>& b, Vector3<S> vertices[8])
{
  const Vector3<S>& extent = b.extent;
  const Vector3<S>& To = b.To;

  Vector3<S> extAxis0 = b.axis.col(0) * extent[0];
  Vector3<S> extAxis1 = b.axis.col(1) * extent[1];
  Vector3<S> extAxis2 = b.axis.col(2) * extent[2];

  vertices[0] = To - extAxis0 - extAxis1 - extAxis2;
  vertices[1] = To + extAxis0 - extAxis1 - extAxis2;
  vertices[2] = To + extAxis0 + extAxis1 - extAxis2;
  vertices[3] = To - extAxis0 + extAxis1 - extAxis2;
  vertices[4] = To - extAxis0 - extAxis1 + extAxis2;
  vertices[5] = To + extAxis0 - extAxis1 + extAxis2;
  vertices[6] = To + extAxis0 + extAxis1 + extAxis2;
  vertices[7] = To - extAxis0 + extAxis1 + extAxis2;
}

//==============================================================================
template <typename S>
OBB<S> merge_smalldist(const OBB<S>& b1, const OBB<S>& b2)
{
  OBB<S> b;
  b.To = (b1.To + b2.To) * 0.5;
  Quaternion<S> q0(b1.axis);
  Quaternion<S> q1(b2.axis);
  if(q0.dot(q1) < 0)
    q1.coeffs() = -q1.coeffs();

  Quaternion<S> q(q0.coeffs() + q1.coeffs());
  q.normalize();
  b.axis = q.toRotationMatrix();


  Vector3<S> vertex[8], diff;
  S real_max = std::numeric_limits<S>::max();
  Vector3<S> pmin(real_max, real_max, real_max);
  Vector3<S> pmax(-real_max, -real_max, -real_max);

  computeVertices(b1, vertex);
  for(int i = 0; i < 8; ++i)
  {
    diff = vertex[i] - b.To;
    for(int j = 0; j < 3; ++j)
    {
      S dot = diff.dot(b.axis.col(j));
      if(dot > pmax[j])
        pmax[j] = dot;
      else if(dot < pmin[j])
        pmin[j] = dot;
    }
  }

  computeVertices(b2, vertex);
  for(int i = 0; i < 8; ++i)
  {
    diff = vertex[i] - b.To;
    for(int j = 0; j < 3; ++j)
    {
      S dot = diff.dot(b.axis.col(j));
      if(dot > pmax[j])
        pmax[j] = dot;
      else if(dot < pmin[j])
        pmin[j] = dot;
    }
  }

  for(int j = 0; j < 3; ++j)
  {
    b.To += (b.axis.col(j) * (0.5 * (pmax[j] + pmin[j])));
    b.extent[j] = 0.5 * (pmax[j] - pmin[j]);
  }

  return b;
}

//==============================================================================
template <typename S, typename Derived>
OBB<S> translate(
    const OBB<S>& bv, const Eigen::MatrixBase<Derived>& t)
{
  OBB<S> res(bv);
  res.To += t;
  return res;
}

//==============================================================================
template <typename S, typename DerivedA, typename DerivedB>
bool overlap(const Eigen::MatrixBase<DerivedA>& R0,
             const Eigen::MatrixBase<DerivedB>& T0,
             const OBB<S>& b1, const OBB<S>& b2)
{
  typename DerivedA::PlainObject R0b2 = R0 * b2.axis;
  typename DerivedA::PlainObject R = b1.axis.transpose() * R0b2;

  typename DerivedB::PlainObject Ttemp = R0 * b2.To + T0 - b1.To;
  typename DerivedB::PlainObject T = Ttemp.transpose() * b1.axis;
  return !obbDisjoint(R, T, b1.extent, b2.extent);
}

//==============================================================================
template <typename S>
bool obbDisjoint(const Matrix3<S>& B, const Vector3<S>& T,
                 const Vector3<S>& a, const Vector3<S>& b, int &timer)
{
    S t, s;
  const S reps = 1e-6;

  Matrix3<S> Bf = B.cwiseAbs();
  Bf.array() += reps;

  // A1 x A2 = A0
  timer ++; 
  t = ((T[0] < 0.0) ? -T[0] : T[0]);
  S k;
  k = Bf.row(0).dot(b);
  if(t > (a[0] + Bf.row(0).dot(b)))
    return true;
  
  // B1 x B2 = B0
  timer ++;
  s =  B.col(0).dot(T);
  t = ((s < 0.0) ? -s : s);

  if(t > (b[0] + Bf.col(0).dot(a)))
    return true;

  // A2 x A0 = A1
  timer ++;
  t = ((T[1] < 0.0) ? -T[1] : T[1]);

  if(t > (a[1] + Bf.row(1).dot(b)))
    return true;

  // A0 x A1 = A2
  timer ++;
  t =((T[2] < 0.0) ? -T[2] : T[2]);

  if(t > (a[2] + Bf.row(2).dot(b)))
    return true;

  // B2 x B0 = B1
  timer ++;
  s = B.col(1).dot(T);
  t = ((s < 0.0) ? -s : s);

  if(t > (b[1] + Bf.col(1).dot(a)))
    return true;

  // B0 x B1 = B2
  timer ++;
  s = B.col(2).dot(T);
  t = ((s < 0.0) ? -s : s);

  if(t > (b[2] + Bf.col(2).dot(a)))
    return true;

  // A0 x B0
  timer ++;
  s = T[2] * B(1, 0) - T[1] * B(2, 0);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[1] * Bf(2, 0) + a[2] * Bf(1, 0) +
          b[1] * Bf(0, 2) + b[2] * Bf(0, 1)))
    return true;

  // A0 x B1
  timer ++;
  s = T[2] * B(1, 1) - T[1] * B(2, 1);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[1] * Bf(2, 1) + a[2] * Bf(1, 1) +
          b[0] * Bf(0, 2) + b[2] * Bf(0, 0)))
    return true;

  // A0 x B2
  timer ++;
  s = T[2] * B(1, 2) - T[1] * B(2, 2);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[1] * Bf(2, 2) + a[2] * Bf(1, 2) +
          b[0] * Bf(0, 1) + b[1] * Bf(0, 0)))
    return true;
  double kk0, kk1, kk2;
  
  // A1 x B0
  timer ++;
  kk0 = B(0,1);
  kk1 = B(1,2);
  kk2 = B(2,0);

  s = T[0] * B(2, 0) - T[2] * B(0, 0);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[0] * Bf(2, 0) + a[2] * Bf(0, 0) +
          b[1] * Bf(1, 2) + b[2] * Bf(1, 1)))
    return true;

  // A1 x B1
  timer ++;
  s = T[0] * B(2, 1) - T[2] * B(0, 1);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[0] * Bf(2, 1) + a[2] * Bf(0, 1) +
          b[0] * Bf(1, 2) + b[2] * Bf(1, 0)))
    return true;

  // A1 x B2
  timer ++;
  s = T[0] * B(2, 2) - T[2] * B(0, 2);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[0] * Bf(2, 2) + a[2] * Bf(0, 2) +
          b[0] * Bf(1, 1) + b[1] * Bf(1, 0)))
    return true;

  // A2 x B0
  timer ++;
  s = T[1] * B(0, 0) - T[0] * B(1, 0);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[0] * Bf(1, 0) + a[1] * Bf(0, 0) +
          b[1] * Bf(2, 2) + b[2] * Bf(2, 1)))
    return true;

  // A2 x B1
  timer ++;
  s = T[1] * B(0, 1) - T[0] * B(1, 1);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[0] * Bf(1, 1) + a[1] * Bf(0, 1) +
          b[0] * Bf(2, 2) + b[2] * Bf(2, 0)))
    return true;

  // A2 x B2
  timer ++;
  s = T[1] * B(0, 2) - T[0] * B(1, 2);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[0] * Bf(1, 2) + a[1] * Bf(0, 2) +
          b[0] * Bf(2, 1) + b[1] * Bf(2, 0)))
    return true;

  return false;
}

//==============================================================================
template <typename S>
bool obbDisjoint(const Matrix3<S>& B, const Vector3<S>& T,
                 const Vector3<S>& a, const Vector3<S>& b)
{
  S t, s;
  const S reps = 1e-6;

  Matrix3<S> Bf = B.cwiseAbs();
  Bf.array() += reps;

  // if any of these tests are one-sided, then the polyhedra are disjoint

  // A1 x A2 = A0
  t = ((T[0] < 0.0) ? -T[0] : T[0]);
  S k;
  k = Bf.row(0).dot(b);
  if(t > (a[0] + Bf.row(0).dot(b)))
    return true;

  // B1 x B2 = B0
  s =  B.col(0).dot(T);
  t = ((s < 0.0) ? -s : s);

  if(t > (b[0] + Bf.col(0).dot(a)))
    return true;

  // A2 x A0 = A1
  t = ((T[1] < 0.0) ? -T[1] : T[1]);

  if(t > (a[1] + Bf.row(1).dot(b)))
    return true;

  // A0 x A1 = A2
  t =((T[2] < 0.0) ? -T[2] : T[2]);

  if(t > (a[2] + Bf.row(2).dot(b)))
    return true;

  // B2 x B0 = B1
  s = B.col(1).dot(T);
  t = ((s < 0.0) ? -s : s);

  if(t > (b[1] + Bf.col(1).dot(a)))
    return true;

  // B0 x B1 = B2
  s = B.col(2).dot(T);
  t = ((s < 0.0) ? -s : s);

  if(t > (b[2] + Bf.col(2).dot(a)))
    return true;

  // A0 x B0
  s = T[2] * B(1, 0) - T[1] * B(2, 0);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[1] * Bf(2, 0) + a[2] * Bf(1, 0) +
          b[1] * Bf(0, 2) + b[2] * Bf(0, 1)))
    return true;

  // A0 x B1
  s = T[2] * B(1, 1) - T[1] * B(2, 1);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[1] * Bf(2, 1) + a[2] * Bf(1, 1) +
          b[0] * Bf(0, 2) + b[2] * Bf(0, 0)))
    return true;

  // A0 x B2
  s = T[2] * B(1, 2) - T[1] * B(2, 2);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[1] * Bf(2, 2) + a[2] * Bf(1, 2) +
          b[0] * Bf(0, 1) + b[1] * Bf(0, 0)))
    return true;
  double kk0, kk1, kk2;
  // A1 x B0
  kk0 = B(0,1);
  kk1 = B(1,2);
  kk2 = B(2,0);

  s = T[0] * B(2, 0) - T[2] * B(0, 0);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[0] * Bf(2, 0) + a[2] * Bf(0, 0) +
          b[1] * Bf(1, 2) + b[2] * Bf(1, 1)))
    return true;

  // A1 x B1
  s = T[0] * B(2, 1) - T[2] * B(0, 1);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[0] * Bf(2, 1) + a[2] * Bf(0, 1) +
          b[0] * Bf(1, 2) + b[2] * Bf(1, 0)))
    return true;

  // A1 x B2
  s = T[0] * B(2, 2) - T[2] * B(0, 2);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[0] * Bf(2, 2) + a[2] * Bf(0, 2) +
          b[0] * Bf(1, 1) + b[1] * Bf(1, 0)))
    return true;

  // A2 x B0
  s = T[1] * B(0, 0) - T[0] * B(1, 0);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[0] * Bf(1, 0) + a[1] * Bf(0, 0) +
          b[1] * Bf(2, 2) + b[2] * Bf(2, 1)))
    return true;

  // A2 x B1
  s = T[1] * B(0, 1) - T[0] * B(1, 1);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[0] * Bf(1, 1) + a[1] * Bf(0, 1) +
          b[0] * Bf(2, 2) + b[2] * Bf(2, 0)))
    return true;

  // A2 x B2
  s = T[1] * B(0, 2) - T[0] * B(1, 2);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[0] * Bf(1, 2) + a[1] * Bf(0, 2) +
          b[0] * Bf(2, 1) + b[1] * Bf(2, 0)))
    return true;

  return false;
}


//==============================================================================
template <typename S>
bool obbDisjoint_gpu(const S* B, const S* T, const S* a, const S* b){
  
  S t_val, s_val;
  const S reps = 1e-6;

  S Bf[9];
  for(int i = 0; i < 9; i++){
    Bf[i] = std::abs(B[i]) + reps;
  }

  for(int i = 0; i < 3; i++){
    t_val = std::abs(T[i]);
    S sum = 0.0;
    for(int j = 0; j < 3;j++){
      sum += Bf[j * 3 + i] * b[j];
    }
    if(t_val > (a[i] + sum))
        return true;
  }

  for(int i = 0; i < 3; i++) {
      S s = 0.0;
      for(int j = 0; j < 3; j++)
          s += B[i * 3 + j] * T[j];
      t_val = std::abs(s);
      S sum = 0.0;
      for(int j = 0; j < 3; j++)
          sum += Bf[i * 3 + j] * a[j];
      if(t_val > (b[i] + sum))
          return true; 
  }

  for(int i = 0; i < 3; i++) { // A 的轴
    for(int j = 0; j < 3; j++) { // B 的轴
      S s = T[(i+2)%3] * B[j * 3 + (i+1)%3] - T[(i+1)%3] * B[j * 3 + (i+2)%3];
      t_val = std::abs(s);
      S threshold = a[(i+1)%3] * Bf[j * 3 + (i+2)%3] + a[(i+2)%3] * Bf[j * 3 + (i+1)%3]
                  + b[(j+1)%3] * Bf[(j+2)%3 * 3 + i] + b[(j+2)%3] * Bf[(j+1)%3 * 3 + i];
      if(t_val > threshold)
          return true; 
    }
  }

  return false;
}

//==============================================================================
template <typename S>
bool obbDisjoint(
    const Transform3<S>& tf,
    const Vector3<S>& a,
    const Vector3<S>& b)
{
  S t, s;
  const S reps = 1e-6;

  Matrix3<S> Bf = tf.linear().cwiseAbs();
  Bf.array() += reps;

  // if any of these tests are one-sided, then the polyhedra are disjoint

  // A1 x A2 = A0
  t = ((tf.translation()[0] < 0.0) ? -tf.translation()[0] : tf.translation()[0]);

  if(t > (a[0] + Bf.row(0).dot(b)))
    return true;

  // B1 x B2 = B0
  s =  tf.linear().col(0).dot(tf.translation());
  t = ((s < 0.0) ? -s : s);

  if(t > (b[0] + Bf.col(0).dot(a)))
    return true;

  // A2 x A0 = A1
  t = ((tf.translation()[1] < 0.0) ? -tf.translation()[1] : tf.translation()[1]);

  if(t > (a[1] + Bf.row(1).dot(b)))
    return true;

  // A0 x A1 = A2
  t =((tf.translation()[2] < 0.0) ? -tf.translation()[2] : tf.translation()[2]);

  if(t > (a[2] + Bf.row(2).dot(b)))
    return true;

  // B2 x B0 = B1
  s = tf.linear().col(1).dot(tf.translation());
  t = ((s < 0.0) ? -s : s);

  if(t > (b[1] + Bf.col(1).dot(a)))
    return true;

  // B0 x B1 = B2
  s = tf.linear().col(2).dot(tf.translation());
  t = ((s < 0.0) ? -s : s);

  if(t > (b[2] + Bf.col(2).dot(a)))
    return true;

  // A0 x B0
  s = tf.translation()[2] * tf.linear()(1, 0) - tf.translation()[1] * tf.linear()(2, 0);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[1] * Bf(2, 0) + a[2] * Bf(1, 0) +
          b[1] * Bf(0, 2) + b[2] * Bf(0, 1)))
    return true;

  // A0 x B1
  s = tf.translation()[2] * tf.linear()(1, 1) - tf.translation()[1] * tf.linear()(2, 1);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[1] * Bf(2, 1) + a[2] * Bf(1, 1) +
          b[0] * Bf(0, 2) + b[2] * Bf(0, 0)))
    return true;

  // A0 x B2
  s = tf.translation()[2] * tf.linear()(1, 2) - tf.translation()[1] * tf.linear()(2, 2);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[1] * Bf(2, 2) + a[2] * Bf(1, 2) +
          b[0] * Bf(0, 1) + b[1] * Bf(0, 0)))
    return true;

  // A1 x B0
  s = tf.translation()[0] * tf.linear()(2, 0) - tf.translation()[2] * tf.linear()(0, 0);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[0] * Bf(2, 0) + a[2] * Bf(0, 0) +
          b[1] * Bf(1, 2) + b[2] * Bf(1, 1)))
    return true;

  // A1 x B1
  s = tf.translation()[0] * tf.linear()(2, 1) - tf.translation()[2] * tf.linear()(0, 1);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[0] * Bf(2, 1) + a[2] * Bf(0, 1) +
          b[0] * Bf(1, 2) + b[2] * Bf(1, 0)))
    return true;

  // A1 x B2
  s = tf.translation()[0] * tf.linear()(2, 2) - tf.translation()[2] * tf.linear()(0, 2);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[0] * Bf(2, 2) + a[2] * Bf(0, 2) +
          b[0] * Bf(1, 1) + b[1] * Bf(1, 0)))
    return true;

  // A2 x B0
  s = tf.translation()[1] * tf.linear()(0, 0) - tf.translation()[0] * tf.linear()(1, 0);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[0] * Bf(1, 0) + a[1] * Bf(0, 0) +
          b[1] * Bf(2, 2) + b[2] * Bf(2, 1)))
    return true;

  // A2 x B1
  s = tf.translation()[1] * tf.linear()(0, 1) - tf.translation()[0] * tf.linear()(1, 1);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[0] * Bf(1, 1) + a[1] * Bf(0, 1) +
          b[0] * Bf(2, 2) + b[2] * Bf(2, 0)))
    return true;

  // A2 x B2
  s = tf.translation()[1] * tf.linear()(0, 2) - tf.translation()[0] * tf.linear()(1, 2);
  t = ((s < 0.0) ? -s : s);

  if(t > (a[0] * Bf(1, 2) + a[1] * Bf(0, 2) +
          b[0] * Bf(2, 1) + b[1] * Bf(2, 0)))
    return true;

  return false;
}


template class OBB<double>;
template class OBB<float>;
template bool obbDisjoint<double>(const Eigen::Matrix<double, 3, 3>&, const Eigen::Matrix<double, 3, 1>&, const Eigen::Matrix<double, 3, 1>&, const Eigen::Matrix<double, 3, 1>&, int &timer);
template bool obbDisjoint<float>(const Eigen::Matrix<float, 3, 3>&, const Eigen::Matrix<float, 3, 1>&, const Eigen::Matrix<float, 3, 1>&, const Eigen::Matrix<float, 3, 1>&, int &timer);
template bool obbDisjoint<double>(const Eigen::Matrix<double, 3, 3>&, const Eigen::Matrix<double, 3, 1>&, const Eigen::Matrix<double, 3, 1>&, const Eigen::Matrix<double, 3, 1>&);
template bool obbDisjoint<float>(const Eigen::Matrix<float, 3, 3>&, const Eigen::Matrix<float, 3, 1>&, const Eigen::Matrix<float, 3, 1>&, const Eigen::Matrix<float, 3, 1>&);
template OBB<double> merge_largedist<double>(const OBB<double>&, const OBB<double>&);
template OBB<float> merge_largedist<float>(const OBB<float>&, const OBB<float>&);
template OBB<double> merge_smalldist<double>(const OBB<double>&, const OBB<double>&);
template OBB<float> merge_smalldist<float>(const OBB<float>&, const OBB<float>&);
template void computeVertices<double>(const OBB<double>&, Eigen::Matrix<double, 3, 1> vertices[8]);
template void computeVertices<float>(const OBB<float>&, Eigen::Matrix<float, 3, 1> vertices[8]);