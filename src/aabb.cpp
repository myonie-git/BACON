#include "aabb.h"

// template <typename S>
// AABB<S>::AABB() 
//     : min_(Vector3<S>::Constant(std::numeric_limits<S>::max())),
//     max_(Vector3<S>::Constant(-std::numeric_limits<S>::max()))
// {

// }

// template <typename S>
// AABB<S>::AABB(const Vector3<S>& a, const Vector3<S>& b)
//   : min_(a.cwiseMin(b)),
//     max_(a.cwiseMax(b))
// {
//   // Do nothing
// }

template <typename S>
bool AABB<S>::overlap(const AABB<S>& other) const
{
  // std::cout << "AABB" << std::endl;
  if ((min_.array() > other.max_.array()).any())
    return false;

  if ((max_.array() < other.min_.array()).any())
    return false;

  return true;

  // const Vector3<S> sphere_center = (other.min_ + other.max_) * S(0.5);
  // const Vector3<S> half_diag = (other.max_ - other.min_) * S(0.5);
  // const S sphere_radius2 = half_diag.squaredNorm();

  // // 2) 球体 vs self AABB：球心到 AABB 的最近点距离
  // Vector3<S> closest = sphere_center;
  // for(int i = 0; i < 3; ++i)
  // {
  //   if(closest[i] < min_[i]) closest[i] = min_[i];
  //   else if(closest[i] > max_[i]) closest[i] = max_[i];
  // }

  // const Vector3<S> diff = sphere_center - closest;
  // const S dist2 = diff.squaredNorm();
  // return dist2 <= sphere_radius2;

}

template <typename S>
bool AABB<S>::overlap(const AABB<S>& other, int &timer) const
{
  // std::cout << "AABB" << std::endl;
  timer+=2;
  if ((min_.array() > other.max_.array()).any())
    return false;

  if ((max_.array() < other.min_.array()).any())
    return false;

  return true;
  const Vector3<S> sphere_center = (other.min_ + other.max_) * S(0.5);
  // const Vector3<S> half_diag = (other.max_ - other.min_) * S(0.5);
  // const S sphere_radius2 = half_diag.squaredNorm();

  // // 2) 球体 vs self AABB：球心到 AABB 的最近点距离
  // Vector3<S> closest = sphere_center;
  // for(int i = 0; i < 3; ++i)
  // {
  //   if(closest[i] < min_[i]) closest[i] = min_[i];
  //   else if(closest[i] > max_[i]) closest[i] = max_[i];
  // }

  // const Vector3<S> diff = sphere_center - closest;
  // const S dist2 = diff.squaredNorm();
  // // 粗粒度代价计数（用于对比不同碰撞检测策略，非精确 FLOPs）
  // timer += 12;
  // return dist2 <= sphere_radius2;
}

template <typename S>
AABB<S>& AABB<S>::operator +=(const AABB<S>& other)
{
  min_ = min_.cwiseMin(other.min_);
  max_ = max_.cwiseMax(other.max_);
  return *this;
}

template <typename S>
AABB<S> AABB<S>::operator +(const AABB<S>& other) const
{
    AABB res(*this);
    return res += other;
}

template <typename S>
void AABB<S>::printEnv(std::ofstream& outfile) const{
    // Calculate the center (Position) of the AABB
    Vector3<S> center = this->center();
    
    // Calculate the Size of the AABB
    S width = this->width();
    S height = this->height();
    S depth = this->depth();
    
    // Output the Position and Size in the required format
    outfile << "Position: (" 
            << center.x() << ", " 
            << center.y() << ", " 
            << center.z() << ")\n"
            << "Size: (" 
            << width << ", " 
            << height << ", " 
            << depth << ")\n\n";
}



template <typename S>
void AABB<S>::printAABB() const{
  // std::cout << "AABB Information:" << std::endl;

  // 打印最小点
  std::cout << "Min: (" << min_[0] << ", " << min_[1] << ", " << min_[2] << ")" << std::endl;

  // 打印最大点
  std::cout << "Max: (" << max_[0] << ", " << max_[1] << ", " << max_[2] << ")" << std::endl;

  // 打印中心点
  // Vector3<S> center = this->center();
  std::cout << "Center: (" << center()[0] << ", " << center()[1] << ", " << center()[2] << ")" << std::endl;
  std::cout << "Extent: (" << this->width() / 2 << ", " << this->height() / 2 << ", " << this->depth() / 2 << ")" << std::endl;

  // // 打印宽度、高度和深度
  // std::cout << "Width: " << this->width() << std::endl;
  // std::cout << "Height: " << this->height() << std::endl;
  // std::cout << "Depth: " << this->depth() << std::endl;

  // 打印总体积
  // std::cout << "Size: " << this->size() << std::endl;

}


//==============================================================================
template <typename S>
S AABB<S>::width() const
{
  return max_[0] - min_[0];
}

//==============================================================================
template <typename S>
S AABB<S>::height() const
{
  return max_[1] - min_[1];
}

//==============================================================================
template <typename S>
S AABB<S>::depth() const
{
  return max_[2] - min_[2];
}

//==============================================================================
template <typename S>
S AABB<S>::size() const
{
  return (max_ - min_).squaredNorm();
}


//==============================================================================
template <typename S>
Vector3<S> AABB<S>::center() const
{
  return (min_ + max_) * 0.5;
}


// 显式实例化模板类 以防万一可以加一个实例化类
template class AABB<double>;
// template class AABB<float>;