/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Jorge Santos
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

/** \Author Jorge Santos */

#include <geometric_shapes/bodies.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/body_operations.h>
#include <boost/filesystem.hpp>
#include "resources/config.h"
#include <random_numbers/random_numbers.h>
#include <iostream>

class CompareMeshVsPrimitive
{
public:
  CompareMeshVsPrimitive()
  {
    // BOX
    shapes::Box box(1.0, 1.0, 1.0);
    shape_meshes.push_back(shapes::createMeshFromShape(&box));
    loaded_meshes.push_back(shapes::createMeshFromResource(
        "file://" + (boost::filesystem::path(TEST_RESOURCES_DIR) / "/cube.stl").string()));

    shape_convex_meshes.push_back(new bodies::ConvexMesh(shape_meshes.back()));
    loaded_convex_meshes.push_back(new bodies::ConvexMesh(loaded_meshes.back()));
  }

  ~CompareMeshVsPrimitive()
  {
    for (size_t i = 0; i < shape_meshes.size(); ++i)
    {
      delete shape_meshes[i];
      delete loaded_meshes[i];

      delete shape_convex_meshes[i];
      delete loaded_convex_meshes[i];
    }
  }

  void testContainsPoint()
  {
    for (size_t i = 0; i < shape_meshes.size(); ++i)
    {
      bodies::Body* shape_cms = shape_convex_meshes[i];
      bodies::Body* loaded_cms = loaded_convex_meshes[i];

      Eigen::Vector3d p;
      bool found = false;
      for (int j = 0; j < 100; ++j)
      {
        if ((shape_cms->samplePointInside(rng, 10000, p)) || (loaded_cms->samplePointInside(rng, 10000, p)))
        {
          found = true;
          if (shape_cms->containsPoint(p) != loaded_cms->containsPoint(p))
          {
            std::cerr << "testContainsPoint failed!" << std::endl;
            return;
          }
        }
      }
      if (!found)
      {
        std::cerr << "No point inside the meshes was found (very unlikely)" << std::endl;
        return;
      }
    }
    std::cout << "testContainsPoint passed!" << std::endl;
  }

  void testIntersectsRay()
  {
    for (size_t i = 0; i < shape_meshes.size(); ++i)
    {
      bodies::Body* shape_cms = shape_convex_meshes[i];
      bodies::Body* loaded_cms = loaded_convex_meshes[i];

      bool intersects = false;
      for (int j = 0; j < 100; ++j)
      {
        Eigen::Vector3d ray_o(rng.uniformReal(-1.0, +1.0), rng.uniformReal(-1.0, +1.0), rng.uniformReal(-1.0, +1.0));
        Eigen::Vector3d ray_d(rng.uniformReal(-1.0, +1.0), rng.uniformReal(-1.0, +1.0), rng.uniformReal(-1.0, +1.0));
        EigenSTL::vector_Vector3d vi1, vi2;
        shape_cms->intersectsRay(ray_o, ray_d, &vi1);
        loaded_cms->intersectsRay(ray_o, ray_d, &vi2);

        if (vi1.size() == vi2.size() && !vi1.empty() && !vi2.empty())
        {
          if (std::abs(vi1[0].x() - vi2[0].x()) > 0.01 ||
              std::abs(vi1[0].y() - vi2[0].y()) > 0.01 ||
              std::abs(vi1[0].z() - vi2[0].z()) > 0.01)
          {
            std::cerr << "testIntersectsRay failed!" << std::endl;
            return;
          }
          intersects = true;
        }
      }
      if (!intersects)
      {
        std::cerr << "No ray intersects the meshes (very unlikely)" << std::endl;
        return;
      }
    }
    std::cout << "testIntersectsRay passed!" << std::endl;
  }

  void testBoundingSphere()
  {
    for (size_t i = 0; i < shape_meshes.size(); ++i)
    {
      shapes::Mesh* shape_ms = shape_meshes[i];
      shapes::Mesh* loaded_ms = loaded_meshes[i];

      shapes::Sphere shape(1.0);
      Eigen::Vector3d center1, center2;
      double radius1, radius2;
      computeShapeBoundingSphere(shape_ms, center1, radius1);
      computeShapeBoundingSphere(loaded_ms, center2, radius2);

      if (std::abs(radius1 - radius2) > 0.001 ||
          std::abs(center1.x() - center2.x()) > 0.001 ||
          std::abs(center1.y() - center2.y()) > 0.001 ||
          std::abs(center1.z() - center2.z()) > 0.001)
      {
        std::cerr << "testBoundingSphere failed!" << std::endl;
        return;
      }
    }
    std::cout << "testBoundingSphere passed!" << std::endl;
  }

  void testBoxVertexCount()
  {
    if (shape_meshes.back()->vertex_count != loaded_meshes.back()->vertex_count)
    {
      std::cerr << "testBoxVertexCount failed!" << std::endl;
    }
    else
    {
      std::cout << "testBoxVertexCount passed!" << std::endl;
    }
  }

  void testBoxTriangleCount()
  {
    if (shape_meshes.back()->triangle_count != loaded_meshes.back()->triangle_count)
    {
      std::cerr << "testBoxTriangleCount failed!" << std::endl;
    }
    else
    {
      std::cout << "testBoxTriangleCount passed!" << std::endl;
    }
  }

private:
  random_numbers::RandomNumberGenerator rng;

  std::vector<shapes::Mesh*> shape_meshes;
  std::vector<shapes::Mesh*> loaded_meshes;

  std::vector<bodies::Body*> shape_convex_meshes;
  std::vector<bodies::Body*> loaded_convex_meshes;
};

int main()
{
  CompareMeshVsPrimitive tester;
  tester.testContainsPoint();
  tester.testIntersectsRay();
  tester.testBoundingSphere();
  tester.testBoxVertexCount();
  tester.testBoxTriangleCount();

  return 0;
}
