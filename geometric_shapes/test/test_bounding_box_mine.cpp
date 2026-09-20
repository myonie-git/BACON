#include <geometric_shapes/bodies.h>
#include <geometric_shapes/body_operations.h>
#include <random_numbers/random_numbers.h>
#include <iostream>
#include <cmath>

void testSphereBoundingBox_Sphere1()
{
    shapes::Sphere shape(1.0);
    bodies::Sphere body(&shape);
    bodies::AABB bbox;
    body.computeBoundingBox(bbox);
    bodies::OBB obbox;
    body.computeBoundingBox(obbox);

    if (std::abs(-1.0 - bbox.min().x()) > 1e-4 ||
        std::abs(-1.0 - bbox.min().y()) > 1e-4 ||
        std::abs(-1.0 - bbox.min().z()) > 1e-4 ||
        std::abs(1.0 - bbox.max().x()) > 1e-4 ||
        std::abs(1.0 - bbox.max().y()) > 1e-4 ||
        std::abs(1.0 - bbox.max().z()) > 1e-4 ||
        std::abs(2.0 - obbox.getExtents().x()) > 1e-4 ||
        std::abs(2.0 - obbox.getExtents().y()) > 1e-4 ||
        std::abs(2.0 - obbox.getExtents().z()) > 1e-4 ||
        std::abs(0.0 - obbox.getPose().translation().x()) > 1e-4 ||
        std::abs(0.0 - obbox.getPose().translation().y()) > 1e-4 ||
        std::abs(0.0 - obbox.getPose().translation().z()) > 1e-4 ||
        !obbox.getPose().linear().isApprox(Eigen::Matrix3d::Identity(), 1e-4)) 
    {
        std::cerr << "testSphereBoundingBox_Sphere1 failed!" << std::endl;
    } 
    else 
    {
        std::cout << "testSphereBoundingBox_Sphere1 passed!" << std::endl;
    }
}

void testSphereBoundingBox_Sphere2()
{
    shapes::Sphere shape(2.0);
    bodies::Sphere body(&shape);
    Eigen::Isometry3d pose;
    pose.setIdentity();
    pose.translation() = Eigen::Vector3d(1, 2, 3);
    body.setPose(pose);
    bodies::AABB bbox;
    body.computeBoundingBox(bbox);
    bodies::OBB obbox;
    body.computeBoundingBox(obbox);

    if (std::abs(-1.0 - bbox.min().x()) > 1e-4 ||
        std::abs(0.0 - bbox.min().y()) > 1e-4 ||
        std::abs(1.0 - bbox.min().z()) > 1e-4 ||
        std::abs(3.0 - bbox.max().x()) > 1e-4 ||
        std::abs(4.0 - bbox.max().y()) > 1e-4 ||
        std::abs(5.0 - bbox.max().z()) > 1e-4 ||
        std::abs(4.0 - obbox.getExtents().x()) > 1e-4 ||
        std::abs(4.0 - obbox.getExtents().y()) > 1e-4 ||
        std::abs(4.0 - obbox.getExtents().z()) > 1e-4 ||
        !obbox.getPose().isApprox(pose, 1e-4)) 
    {
        std::cerr << "testSphereBoundingBox_Sphere2 failed!" << std::endl;
    } 
    else 
    {
        std::cout << "testSphereBoundingBox_Sphere2 passed!" << std::endl;
    }

    pose *= Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d(1, 1, 1).normalized());
    body.setPose(pose);
    body.computeBoundingBox(bbox);
    body.computeBoundingBox(obbox);

    if (std::abs(-1.0 - bbox.min().x()) > 1e-4 ||
        std::abs(0.0 - bbox.min().y()) > 1e-4 ||
        std::abs(1.0 - bbox.min().z()) > 1e-4 ||
        std::abs(3.0 - bbox.max().x()) > 1e-4 ||
        std::abs(4.0 - bbox.max().y()) > 1e-4 ||
        std::abs(5.0 - bbox.max().z()) > 1e-4 ||
        std::abs(4.0 - obbox.getExtents().x()) > 1e-4 ||
        std::abs(4.0 - obbox.getExtents().y()) > 1e-4 ||
        std::abs(4.0 - obbox.getExtents().z()) > 1e-4 ||
        !obbox.getPose().translation().isApprox(pose.translation(), 1e-4) ||
        !obbox.getPose().linear().isApprox(Eigen::Matrix3d::Identity(), 1e-4)) 
    {
        std::cerr << "testSphereBoundingBox_Sphere2 failed!" << std::endl;
    } 
    else 
    {
        std::cout << "testSphereBoundingBox_Sphere2 passed!" << std::endl;
    }

    random_numbers::RandomNumberGenerator gen;
    double quatData[4];
    Eigen::Quaterniond quat;

    for (size_t i = 0; i < 10; ++i)
    {
        gen.quaternion(quatData);
        quat.x() = quatData[0];
        quat.y() = quatData[1];
        quat.z() = quatData[2];
        quat.w() = quatData[3];
        pose.linear() = quat.toRotationMatrix();
        body.setPose(pose);
        bodies::AABB bbox2;
        bodies::OBB obbox2;
        body.computeBoundingBox(bbox2);
        body.computeBoundingBox(obbox2);

        if (std::abs(bbox2.min().x() - bbox.min().x()) > 1e-4 ||
            std::abs(bbox2.min().y() - bbox.min().y()) > 1e-4 ||
            std::abs(bbox2.min().z() - bbox.min().z()) > 1e-4 ||
            std::abs(bbox2.max().x() - bbox.max().x()) > 1e-4 ||
            std::abs(bbox2.max().y() - bbox.max().y()) > 1e-4 ||
            std::abs(bbox2.max().z() - bbox.max().z()) > 1e-4 ||
            std::abs(obbox.getExtents().x() - obbox2.getExtents().x()) > 1e-4 ||
            std::abs(obbox.getExtents().y() - obbox2.getExtents().y()) > 1e-4 ||
            std::abs(obbox.getExtents().z() - obbox2.getExtents().z()) > 1e-4 ||
            !obbox2.getPose().isApprox(obbox.getPose(), 1e-4)) 
        {
            std::cerr << "testSphereBoundingBox_Sphere2 failed!" << std::endl;
            break;
        }
    }

    std::cout << "testSphereBoundingBox_Sphere2 passed!" << std::endl;
}

void testBoxBoundingBox_Box1()
{
    shapes::Box shape(1.0, 2.0, 3.0);
    bodies::Box body(&shape);
    bodies::AABB bbox;
    body.computeBoundingBox(bbox);
    bodies::OBB obbox;
    body.computeBoundingBox(obbox);

    if (std::abs(-0.5 - bbox.min().x()) > 1e-4 ||
        std::abs(-1.0 - bbox.min().y()) > 1e-4 ||
        std::abs(-1.5 - bbox.min().z()) > 1e-4 ||
        std::abs(0.5 - bbox.max().x()) > 1e-4 ||
        std::abs(1.0 - bbox.max().y()) > 1e-4 ||
        std::abs(1.5 - bbox.max().z()) > 1e-4 ||
        std::abs(1.0 - obbox.getExtents().x()) > 1e-4 ||
        std::abs(2.0 - obbox.getExtents().y()) > 1e-4 ||
        std::abs(3.0 - obbox.getExtents().z()) > 1e-4 ||
        std::abs(0.0 - obbox.getPose().translation().x()) > 1e-4 ||
        std::abs(0.0 - obbox.getPose().translation().y()) > 1e-4 ||
        std::abs(0.0 - obbox.getPose().translation().z()) > 1e-4 ||
        !obbox.getPose().linear().isApprox(Eigen::Matrix3d::Identity(), 1e-4)) 
    {
        std::cerr << "testBoxBoundingBox_Box1 failed!" << std::endl;
    } 
    else 
    {
        std::cout << "testBoxBoundingBox_Box1 passed!" << std::endl;
    }
}

void testBoxBoundingBox_Box2()
{
    shapes::Box shape(1.0, 2.0, 3.0);
    bodies::Box body(&shape);
    Eigen::Isometry3d pose;
    pose.setIdentity();
    pose.translation() = Eigen::Vector3d(1, 2, 3);
    body.setPose(pose);
    bodies::AABB bbox;
    body.computeBoundingBox(bbox);
    bodies::OBB obbox;
    body.computeBoundingBox(obbox);

    if (std::abs(0.5 - bbox.min().x()) > 1e-4 ||
        std::abs(1.0 - bbox.min().y()) > 1e-4 ||
        std::abs(1.5 - bbox.min().z()) > 1e-4 ||
        std::abs(1.5 - bbox.max().x()) > 1e-4 ||
        std::abs(3.0 - bbox.max().y()) > 1e-4 ||
        std::abs(4.5 - bbox.max().z()) > 1e-4 ||
        std::abs(1.0 - obbox.getExtents().x()) > 1e-4 ||
        std::abs(2.0 - obbox.getExtents().y()) > 1e-4 ||
        std::abs(3.0 - obbox.getExtents().z()) > 1e-4 ||
        std::abs(1.0 - obbox.getPose().translation().x()) > 1e-4 ||
        std::abs(2.0 - obbox.getPose().translation().y()) > 1e-4 ||
        std::abs(3.0 - obbox.getPose().translation().z()) > 1e-4 ||
        !obbox.getPose().linear().isApprox(Eigen::Matrix3d::Identity(), 1e-4)) 
    {
        std::cerr << "testBoxBoundingBox_Box2 failed!" << std::endl;
    } 
    else 
    {
        std::cout << "testBoxBoundingBox_Box2 passed!" << std::endl;
    }

    pose *= Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d(1, 1, 1).normalized());
    body.setPose(pose);
    body.computeBoundingBox(bbox);
    body.computeBoundingBox(obbox);

    if (std::abs(-0.7767 - bbox.min().x()) > 1e-4 ||
        std::abs(0.8452 - bbox.min().y()) > 1e-4 ||
        std::abs(1.4673 - bbox.min().z()) > 1e-4 ||
        std::abs(2.7767 - bbox.max().x()) > 1e-4 ||
        std::abs(3.1547 - bbox.max().y()) > 1e-4 ||
        std::abs(4.5326 - bbox.max().z()) > 1e-4 ||
        std::abs(1.0 - obbox.getExtents().x()) > 1e-4 ||
        std::abs(2.0 - obbox.getExtents().y()) > 1e-4 ||
        std::abs(3.0 - obbox.getExtents().z()) > 1e-4 ||
        std::abs(1.0 - obbox.getPose().translation().x()) > 1e-4 ||
        std::abs(2.0 - obbox.getPose().translation().y()) > 1e-4 ||
        std::abs(3.0 - obbox.getPose().translation().z()) > 1e-4 ||
        !obbox.getPose().linear().isApprox(pose.linear(), 1e-4)) 
    {
        std::cerr << "testBoxBoundingBox_Box2 failed!" << std::endl;
    } 
    else 
    {
        std::cout << "testBoxBoundingBox_Box2 passed!" << std::endl;
    }
}

void testCylinderBoundingBox_Cylinder1()
{
    shapes::Cylinder shape(1.0, 2.0);
    bodies::Cylinder body(&shape);
    bodies::AABB bbox;
    body.computeBoundingBox(bbox);
    bodies::OBB obbox;
    body.computeBoundingBox(obbox);

    if (std::abs(-1.0 - bbox.min().x()) > 1e-4 ||
        std::abs(-1.0 - bbox.min().y()) > 1e-4 ||
        std::abs(-1.0 - bbox.min().z()) > 1e-4 ||
        std::abs(1.0 - bbox.max().x()) > 1e-4 ||
        std::abs(1.0 - bbox.max().y()) > 1e-4 ||
        std::abs(1.0 - bbox.max().z()) > 1e-4 ||
        std::abs(2.0 - obbox.getExtents().x()) > 1e-4 ||
        std::abs(2.0 - obbox.getExtents().y()) > 1e-4 ||
        std::abs(2.0 - obbox.getExtents().z()) > 1e-4 ||
        std::abs(0.0 - obbox.getPose().translation().x()) > 1e-4 ||
        std::abs(0.0 - obbox.getPose().translation().y()) > 1e-4 ||
        std::abs(0.0 - obbox.getPose().translation().z()) > 1e-4 ||
        !obbox.getPose().linear().isApprox(Eigen::Matrix3d::Identity(), 1e-4)) 
    {
        std::cerr << "testCylinderBoundingBox_Cylinder1 failed!" << std::endl;
    } 
    else 
    {
        std::cout << "testCylinderBoundingBox_Cylinder1 passed!" << std::endl;
    }
}

void testCylinderBoundingBox_Cylinder2()
{
    shapes::Cylinder shape(1.0, 2.0);
    bodies::Cylinder body(&shape);
    Eigen::Isometry3d pose;
    pose.setIdentity();
    pose.translation() = Eigen::Vector3d(1, 2, 3);
    body.setPose(pose);
    bodies::AABB bbox;
    body.computeBoundingBox(bbox);
    bodies::OBB obbox;
    body.computeBoundingBox(obbox);

    if (std::abs(0.0 - bbox.min().x()) > 1e-4 ||
        std::abs(1.0 - bbox.min().y()) > 1e-4 ||
        std::abs(2.0 - bbox.min().z()) > 1e-4 ||
        std::abs(2.0 - bbox.max().x()) > 1e-4 ||
        std::abs(3.0 - bbox.max().y()) > 1e-4 ||
        std::abs(4.0 - bbox.max().z()) > 1e-4 ||
        std::abs(2.0 - obbox.getExtents().x()) > 1e-4 ||
        std::abs(2.0 - obbox.getExtents().y()) > 1e-4 ||
        std::abs(2.0 - obbox.getExtents().z()) > 1e-4 ||
        std::abs(1.0 - obbox.getPose().translation().x()) > 1e-4 ||
        std::abs(2.0 - obbox.getPose().translation().y()) > 1e-4 ||
        std::abs(3.0 - obbox.getPose().translation().z()) > 1e-4 ||
        !obbox.getPose().linear().isApprox(Eigen::Matrix3d::Identity(), 1e-4)) 
    {
        std::cerr << "testCylinderBoundingBox_Cylinder2 failed!" << std::endl;
    } 
    else 
    {
        std::cout << "testCylinderBoundingBox_Cylinder2 passed!" << std::endl;
    }

    pose *= Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d(1, 1, 1).normalized());
    body.setPose(pose);
    body.computeBoundingBox(bbox);
    body.computeBoundingBox(obbox);

    if (std::abs(-0.3238 - bbox.min().x()) > 1e-4 ||
        std::abs(0.7862 - bbox.min().y()) > 1e-4 ||
        std::abs(1.7239 - bbox.min().z()) > 1e-4 ||
        std::abs(2.3238 - bbox.max().x()) > 1e-4 ||
        std::abs(3.2138 - bbox.max().y()) > 1e-4 ||
        std::abs(4.2761 - bbox.max().z()) > 1e-4 ||
        std::abs(2.0 - obbox.getExtents().x()) > 1e-4 ||
        std::abs(2.0 - obbox.getExtents().y()) > 1e-4 ||
        std::abs(2.0 - obbox.getExtents().z()) > 1e-4 ||
        std::abs(1.0 - obbox.getPose().translation().x()) > 1e-4 ||
        std::abs(2.0 - obbox.getPose().translation().y()) > 1e-4 ||
        std::abs(3.0 - obbox.getPose().translation().z()) > 1e-4 ||
        !obbox.getPose().linear().isApprox(pose.linear(), 1e-4)) 
    {
        std::cerr << "testCylinderBoundingBox_Cylinder2 failed!" << std::endl;
    } 
    else 
    {
        std::cout << "testCylinderBoundingBox_Cylinder2 passed!" << std::endl;
    }

    random_numbers::RandomNumberGenerator gen(0);
    const auto rollPitch =
        Eigen::AngleAxisd(1.0, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(1.0, Eigen::Vector3d::UnitY());

    pose.linear() = rollPitch.toRotationMatrix();
    body.setPose(pose);
    body.computeBoundingBox(bbox);

    bodies::AABB bbox2;
    bodies::OBB obbox2;
    for (size_t i = 0; i < 10; ++i)
    {
        const auto angle = gen.uniformReal(-M_PI, M_PI);
        const auto yaw = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ());
        pose.linear() = (rollPitch * yaw).toRotationMatrix();
        body.setPose(pose);
        body.computeBoundingBox(bbox2);
        body.computeBoundingBox(obbox2);

        if (std::abs(bbox2.min().x() - bbox.min().x()) > 1e-4 ||
            std::abs(bbox2.min().y() - bbox.min().y()) > 1e-4 ||
            std::abs(bbox2.min().z() - bbox.min().z()) > 1e-4 ||
            std::abs(bbox2.max().x() - bbox.max().x()) > 1e-4 ||
            std::abs(bbox2.max().y() - bbox.max().y()) > 1e-4 ||
            std::abs(bbox2.max().z() - bbox.max().z()) > 1e-4 ||
            std::abs(2.0 - obbox2.getExtents().x()) > 1e-4 ||
            std::abs(2.0 - obbox2.getExtents().y()) > 1e-4 ||
            std::abs(2.0 - obbox2.getExtents().z()) > 1e-4 ||
            std::abs(1.0 - obbox2.getPose().translation().x()) > 1e-4 ||
            std::abs(2.0 - obbox2.getPose().translation().y()) > 1e-4 ||
            std::abs(3.0 - obbox2.getPose().translation().z()) > 1e-4 ||
            !obbox2.getPose().linear().isApprox(pose.linear(), 1e-4)) 
        {
            std::cerr << "testCylinderBoundingBox_Cylinder2 failed!" << std::endl;
            break;
        }
    }

    std::cout << "testCylinderBoundingBox_Cylinder2 passed!" << std::endl;
}

shapes::Mesh* createBoxMesh(const Eigen::Vector3d& min, const Eigen::Vector3d& max)
{
    shapes::Mesh* m = new shapes::Mesh(8, 12);

    m->vertices[3 * 0 + 0] = min.x();
    m->vertices[3 * 0 + 1] = min.y();
    m->vertices[3 * 0 + 2] = min.z();

    m->vertices[3 * 1 + 0] = max.x();
    m->vertices[3 * 1 + 1] = min.y();
    m->vertices[3 * 1 + 2] = min.z();

    m->vertices[3 * 2 + 0] = min.x();
    m->vertices[3 * 2 + 1] = max.y();
    m->vertices[3 * 2 + 2] = min.z();

    m->vertices[3 * 3 + 0] = max.x();
    m->vertices[3 * 3 + 1] = max.y();
    m->vertices[3 * 3 + 2] = min.z();

    m->vertices[3 * 4 + 0] = min.x();
    m->vertices[3 * 4 + 1] = min.y();
    m->vertices[3 * 4 + 2] = max.z();

    m->vertices[3 * 5 + 0] = max.x();
    m->vertices[3 * 5 + 1] = min.y();
    m->vertices[3 * 5 + 2] = max.z();

    m->vertices[3 * 6 + 0] = min.x();
    m->vertices[3 * 6 + 1] = max.y();
    m->vertices[3 * 6 + 2] = max.z();

    m->vertices[3 * 7 + 0] = max.x();
    m->vertices[3 * 7 + 1] = max.y();
    m->vertices[3 * 7 + 2] = max.z();

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

    return m;
}

void testMeshBoundingBox_Mesh1()
{
    shapes::Mesh* m = createBoxMesh({ -1, -1, -1 }, { 1, 1, 1 });

    bodies::ConvexMesh body(m);
    bodies::AABB bbox;
    body.computeBoundingBox(bbox);
    bodies::OBB obbox;
    body.computeBoundingBox(obbox);

    if (std::abs(-1.0 - bbox.min().x()) > 1e-4 ||
        std::abs(-1.0 - bbox.min().y()) > 1e-4 ||
        std::abs(-1.0 - bbox.min().z()) > 1e-4 ||
        std::abs(1.0 - bbox.max().x()) > 1e-4 ||
        std::abs(1.0 - bbox.max().y()) > 1e-4 ||
        std::abs(1.0 - bbox.max().z()) > 1e-4 ||
        std::abs(2.0 - obbox.getExtents().x()) > 1e-4 ||
        std::abs(2.0 - obbox.getExtents().y()) > 1e-4 ||
        std::abs(2.0 - obbox.getExtents().z()) > 1e-4 ||
        std::abs(0.0 - obbox.getPose().translation().x()) > 1e-4 ||
        std::abs(0.0 - obbox.getPose().translation().y()) > 1e-4 ||
        std::abs(0.0 - obbox.getPose().translation().z()) > 1e-4 ||
        !obbox.getPose().linear().isApprox(Eigen::Matrix3d::Identity(), 1e-4)) 
    {
        std::cerr << "testMeshBoundingBox_Mesh1 failed!" << std::endl;
    } 
    else 
    {
        std::cout << "testMeshBoundingBox_Mesh1 passed!" << std::endl;
    }

    delete m;
}

void testMeshBoundingBox_Mesh2()
{
    shapes::Mesh* m = createBoxMesh({ -0.5, -1.0, -1.5 }, { 0.5, 1.0, 1.5 });

    bodies::ConvexMesh body(m);
    Eigen::Isometry3d pose;
    pose.setIdentity();
    pose.translation() = Eigen::Vector3d(1, 2, 3);
    body.setPose(pose);
    bodies::AABB bbox;
    body.computeBoundingBox(bbox);
    bodies::OBB obbox;
    body.computeBoundingBox(obbox);

    if (std::abs(0.5 - bbox.min().x()) > 1e-4 ||
        std::abs(1.0 - bbox.min().y()) > 1e-4 ||
        std::abs(1.5 - bbox.min().z()) > 1e-4 ||
        std::abs(1.5 - bbox.max().x()) > 1e-4 ||
        std::abs(3.0 - bbox.max().y()) > 1e-4 ||
        std::abs(4.5 - bbox.max().z()) > 1e-4 ||
        std::abs(1.0 - obbox.getExtents().x()) > 1e-4 ||
        std::abs(2.0 - obbox.getExtents().y()) > 1e-4 ||
        std::abs(3.0 - obbox.getExtents().z()) > 1e-4 ||
        std::abs(1.0 - obbox.getPose().translation().x()) > 1e-4 ||
        std::abs(2.0 - obbox.getPose().translation().y()) > 1e-4 ||
        std::abs(3.0 - obbox.getPose().translation().z()) > 1e-4 ||
        !obbox.getPose().linear().isApprox(Eigen::Matrix3d::Identity(), 1e-4)) 
    {
        std::cerr << "testMeshBoundingBox_Mesh2 failed!" << std::endl;
    } 
    else 
    {
        std::cout << "testMeshBoundingBox_Mesh2 passed!" << std::endl;
    }

    pose *= Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d(1, 1, 1).normalized());
    body.setPose(pose);
    body.computeBoundingBox(bbox);
    body.computeBoundingBox(obbox);

    if (std::abs(-0.7767 - bbox.min().x()) > 1e-4 ||
        std::abs(0.8452 - bbox.min().y()) > 1e-4 ||
        std::abs(1.4673 - bbox.min().z()) > 1e-4 ||
        std::abs(2.7767 - bbox.max().x()) > 1e-4 ||
        std::abs(3.1547 - bbox.max().y()) > 1e-4 ||
        std::abs(4.5326 - bbox.max().z()) > 1e-4 ||
        std::abs(1.0 - obbox.getExtents().x()) > 1e-4 ||
        std::abs(2.0 - obbox.getExtents().y()) > 1e-4 ||
        std::abs(3.0 - obbox.getExtents().z()) > 1e-4 ||
        std::abs(1.0 - obbox.getPose().translation().x()) > 1e-4 ||
        std::abs(2.0 - obbox.getPose().translation().y()) > 1e-4 ||
        std::abs(3.0 - obbox.getPose().translation().z()) > 1e-4 ||
        !obbox.getPose().linear().isApprox(pose.linear(), 1e-4)) 
    {
        std::cerr << "testMeshBoundingBox_Mesh2 failed!" << std::endl;
    } 
    else 
    {
        std::cout << "testMeshBoundingBox_Mesh2 passed!" << std::endl;
    }

    delete m;
}

int main()
{
    testSphereBoundingBox_Sphere1();
    testSphereBoundingBox_Sphere2();
    testBoxBoundingBox_Box1();
    testBoxBoundingBox_Box2();
    testCylinderBoundingBox_Cylinder1();
    testCylinderBoundingBox_Cylinder2();
    testMeshBoundingBox_Mesh1();
    testMeshBoundingBox_Mesh2();

    return 0;
}
