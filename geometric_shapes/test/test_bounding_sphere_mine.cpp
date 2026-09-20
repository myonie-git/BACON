/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/** \Author Acorn Pooley */

#include <geometric_shapes/bodies.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/body_operations.h>
#include <boost/filesystem.hpp>
// #include <gtest/gtest.h>

void testSphereBoundingSphere1() {
  shapes::Sphere shape(1.0);
  Eigen::Vector3d center;
  double radius;
  computeShapeBoundingSphere(&shape, center, radius);

  // EXPECT_EQ(1.0, radius);
  // EXPECT_EQ(0.0, center.x());
  // EXPECT_EQ(0.0, center.y());
  // EXPECT_EQ(0.0, center.z());
  if (radius != 1.0 || center.x() != 0.0 || center.y() != 0.0 || center.z() != 0.0) {
    std::cerr << "testSphereBoundingSphere1 failed!" << std::endl;
  }
}

void testSphereBoundingSphere2() {
  shapes::Shape* shape = new shapes::Sphere(2.0);
  Eigen::Vector3d center;
  double radius;
  computeShapeBoundingSphere(shape, center, radius);

  // EXPECT_EQ(2.0, radius);
  // EXPECT_EQ(0.0, center.x());
  // EXPECT_EQ(0.0, center.y());
  // EXPECT_EQ(0.0, center.z());
  if (radius != 2.0 || center.x() != 0.0 || center.y() != 0.0 || center.z() != 0.0) {
    std::cerr << "testSphereBoundingSphere2 failed!" << std::endl;
  }
  delete shape;
}

void testBoxBoundingSphere1() {
  shapes::Shape* shape = new shapes::Box(2.0, 4.0, 6.0);
  Eigen::Vector3d center;
  double radius;
  computeShapeBoundingSphere(shape, center, radius);

  // EXPECT_EQ(sqrt(14.0), radius);
  // EXPECT_EQ(0.0, center.x());
  // EXPECT_EQ(0.0, center.y());
  // EXPECT_EQ(0.0, center.z());
  if (radius != sqrt(14.0) || center.x() != 0.0 || center.y() != 0.0 || center.z() != 0.0) {
    std::cerr << "testBoxBoundingSphere1 failed!" << std::endl;
  }
}

void testBoxBoundingSphere2() {
  shapes::Shape* shape = new shapes::Box(2.0, 2.0, 2.0);
  Eigen::Vector3d center;
  double radius;
  computeShapeBoundingSphere(shape, center, radius);

  // EXPECT_EQ(sqrt(3.0), radius);
  // EXPECT_EQ(0.0, center.x());
  // EXPECT_EQ(0.0, center.y());
  // EXPECT_EQ(0.0, center.z());
  if (radius != sqrt(3.0) || center.x() != 0.0 || center.y() != 0.0 || center.z() != 0.0) {
    std::cerr << "testBoxBoundingSphere2 failed!" << std::endl;
  }
}

void testCylBoundingSphere1() {
  shapes::Shape* shape = new shapes::Cylinder(1.0, 4.0);
  Eigen::Vector3d center;
  double radius;
  computeShapeBoundingSphere(shape, center, radius);

  // EXPECT_EQ(sqrt(1 + 4), radius);
  // EXPECT_EQ(0.0, center.x());
  // EXPECT_EQ(0.0, center.y());
  // EXPECT_EQ(0.0, center.z());
  if (radius != sqrt(1 + 4) || center.x() != 0.0 || center.y() != 0.0 || center.z() != 0.0) {
    std::cerr << "testCylBoundingSphere1 failed!" << std::endl;
  }
}

void testCylBoundingSphere2() {
  shapes::Shape* shape = new shapes::Cylinder(2.0, 20.0);
  Eigen::Vector3d center;
  double radius;
  computeShapeBoundingSphere(shape, center, radius);

  // EXPECT_EQ(sqrt(4 + 100), radius);
  // EXPECT_EQ(0.0, center.x());
  // EXPECT_EQ(0.0, center.y());
  // EXPECT_EQ(0.0, center.z());
  if (radius != sqrt(4 + 100) || center.x() != 0.0 || center.y() != 0.0 || center.z() != 0.0) {
    std::cerr << "testCylBoundingSphere2 failed!" << std::endl;
  }
}

void testConeBoundingSphere1() {
  shapes::Shape* shape = new shapes::Cone(20.0, 2.0);
  Eigen::Vector3d center;
  double radius;
  computeShapeBoundingSphere(shape, center, radius);

  // EXPECT_EQ(20.0, radius);
  // EXPECT_EQ(0.0, center.x());
  // EXPECT_EQ(0.0, center.y());
  // EXPECT_EQ(-1.0, center.z());
  if (radius != 20.0 || center.x() != 0.0 || center.y() != 0.0 || center.z() != -1.0) {
    std::cerr << "testConeBoundingSphere1 failed!" << std::endl;
  }
}

void testConeBoundingSphere2() {
  shapes::Shape* shape = new shapes::Cone(5.0, 5.0);
  Eigen::Vector3d center;
  double radius;
  computeShapeBoundingSphere(shape, center, radius);

  // EXPECT_EQ(5.0, radius);
  // EXPECT_EQ(0.0, center.x());
  // EXPECT_EQ(0.0, center.y());
  // EXPECT_EQ(-2.5, center.z());
  if (radius != 5.0 || center.x() != 0.0 || center.y() != 0.0 || center.z() != -2.5) {
    std::cerr << "testConeBoundingSphere2 failed!" << std::endl;
  }
}

void testConeBoundingSphere3() {
  double height = 1.0 + 1.0 / sqrt(2);
  shapes::Shape* shape = new shapes::Cone(1 / sqrt(2), height);
  Eigen::Vector3d center;
  double radius;
  computeShapeBoundingSphere(shape, center, radius);

  // EXPECT_EQ(1.0, radius);
  // EXPECT_EQ(0.0, center.x());
  // EXPECT_EQ(0.0, center.y());
  // EXPECT_EQ(height / 2 - 1, center.z());
  if (radius != 1.0 || center.x() != 0.0 || center.y() != 0.0 || center.z() != height / 2 - 1) {
    std::cerr << "testConeBoundingSphere3 failed!" << std::endl;
  }
}

void testConeBoundingSphere4() {
  shapes::Shape* shape = new shapes::Cone(3, 10);
  Eigen::Vector3d center;
  double radius;
  computeShapeBoundingSphere(shape, center, radius);

  // EXPECT_EQ(109.0 / 20, radius);
  // EXPECT_EQ(0.0, center.x());
  // EXPECT_EQ(0.0, center.y());
  // EXPECT_EQ(5 - (109.0 / 20), center.z());
  if (radius != 109.0 / 20 || center.x() != 0.0 || center.y() != 0.0 || center.z() != 5 - (109.0 / 20)) {
    std::cerr << "testConeBoundingSphere4 failed!" << std::endl;
  }
}

void testMeshBoundingSphere1() {
  shapes::Shape* shape = new shapes::Mesh(8, 12);
  shapes::Mesh* m = static_cast<shapes::Mesh*>(shape);

  // box mesh

  m->vertices[3 * 0 + 0] = 0;
  m->vertices[3 * 0 + 1] = 0;
  m->vertices[3 * 0 + 2] = 0;

  m->vertices[3 * 1 + 0] = 1;
  m->vertices[3 * 1 + 1] = 0;
  m->vertices[3 * 1 + 2] = 0;

  m->vertices[3 * 2 + 0] = 0;
  m->vertices[3 * 2 + 1] = 1;
  m->vertices[3 * 2 + 2] = 0;

  m->vertices[3 * 3 + 0] = 1;
  m->vertices[3 * 3 + 1] = 1;
  m->vertices[3 * 3 + 2] = 0;

  m->vertices[3 * 4 + 0] = 0;
  m->vertices[3 * 4 + 1] = 0;
  m->vertices[3 * 4 + 2] = 1;

  m->vertices[3 * 5 + 0] = 1;
  m->vertices[3 * 5 + 1] = 0;
  m->vertices[3 * 5 + 2] = 1;

  m->vertices[3 * 6 + 0] = 0;
  m->vertices[3 * 6 + 1] = 1;
  m->vertices[3 * 6 + 2] = 1;

  m->vertices[3 * 7 + 0] = 1;
  m->vertices[3 * 7 + 1] = 1;
  m->vertices[3 * 7 + 2] = 1;

  m->triangles[3 * 0 + 0] = 0;
  m->triangles[3 * 0 + 1] = 1;
  m->triangles[3 * 0 + 2] = 2;

  m->triangles[3 * 1 + 0] = 1;
  m->triangles[3 * 1 + 1] = 3;
  m->triangles[3 * 1 + 2] = 2;

  m->triangles[3 * 2 + 0] = 5;
  m->triangles[3 * 2 + 1] = 4;
  m->triangles[3 * 2 + 2] = 6;

  m->triangles[3 * 3 + 0] = 5;
  m->triangles[3 * 3 + 1] = 6;
  m->triangles[3 * 3 + 2] = 7;

  m->triangles[3 * 4 + 0] = 1;
  m->triangles[3 * 4 + 1] = 5;
  m->triangles[3 * 4 + 2] = 3;

  m->triangles[3 * 5 + 0] = 5;
  m->triangles[3 * 5 + 1] = 7;
  m->triangles[3 * 5 + 2] = 3;

  m->triangles[3 * 6 + 0] = 4;
  m->triangles[3 * 6 + 1] = 0;
  m->triangles[3 * 6 + 2] = 2;

  m->triangles[3 * 7 + 0] = 4;
  m->triangles[3 * 7 + 1] = 2;
  m->triangles[3 * 7 + 2] = 6;

  m->triangles[3 * 8 + 0] = 2;
  m->triangles[3 * 8 + 1] = 3;
  m->triangles[3 * 8 + 2] = 6;

  m->triangles[3 * 9 + 0] = 3;
  m->triangles[3 * 9 + 1] = 7;
  m->triangles[3 * 9 + 2] = 6;

  m->triangles[3 * 10 + 0] = 1;
  m->triangles[3 * 10 + 1] = 0;
  m->triangles[3 * 10 + 2] = 4;

  m->triangles[3 * 11 + 0] = 1;
  m->triangles[3 * 11 + 1] = 4;
  m->triangles[3 * 11 + 2] = 5;

  Eigen::Vector3d center;
  double radius;
  computeShapeBoundingSphere(shape, center, radius);

  // EXPECT_EQ(sqrt(0.75), radius);
  // EXPECT_EQ(0.5, center.x());
  // EXPECT_EQ(0.5, center.y());
  // EXPECT_EQ(0.5, center.z());
  if (radius != sqrt(0.75) || center.x() != 0.5 || center.y() != 0.5 || center.z() != 0.5) {
    std::cerr << "testMeshBoundingSphere1 failed!" << std::endl;
  }
}

int main(int argc, char** argv)
{
//   testSphereBoundingSphere1();
//   testSphereBoundingSphere2();
//   testBoxBoundingSphere1();
//   testBoxBoundingSphere2();
//   testCylBoundingSphere1();
//   testCylBoundingSphere2();
//   testConeBoundingSphere1();
//   testConeBoundingSphere2();
//   testConeBoundingSphere3();
//   testConeBoundingSphere4();
  testMeshBoundingSphere1();
  return 0;
}
