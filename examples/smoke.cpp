#include "collision_object.h"
#include "collision_env.h"
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

struct Robot { const char* name; const char* file; const char* root; };
}

int main() {
    try {
        const Robot robots[] = {
            {"fanuc", "fanuc_description.urdf", "base_link"},
            {"prbt", "prbt_description.urdf", "prbt_base_link"},
            {"panda", "panda_description.urdf", "panda_link0"},
            {"jaco2", "j2n6s200_standalone.urdf", "root"},
            {"jaco3", "j2n6s300_standalone.urdf", "root"},
            {"go1", "go1.urdf", "base"},
        };
        using S = double;
        const AABB<S> a(Vector3<S>(0, 0, 0), Vector3<S>(1, 1, 1));
        const AABB<S> b(Vector3<S>(0.5, 0.5, 0.5), Vector3<S>(2, 2, 2));
        const AABB<S> c(Vector3<S>(3, 3, 3), Vector3<S>(4, 4, 4));
        require(a.overlap(b) && !a.overlap(c), "AABB overlap regression");
        CollisionObject<S> oa(a), ob(b), oc(c);
        require(oa.obb.overlap(ob.obb) && !oa.obb.overlap(oc.obb), "OBB overlap regression");

        for (const auto& robot : robots) {
            URDFModel model;
            require(model.loadURDF(std::string(BACON_SOURCE_DIR) + "/" + robot.file),
                    std::string("Cannot load ") + robot.name);
            require(model.links.count(robot.root) == 1, std::string("Missing root for ") + robot.name);
            require(!model.collisionGeometries.empty(), std::string("No geometry for ") + robot.name);
            model.setJointAngles({});
            model.calculateWorldCoordinates(robot.root);
            std::vector<Vector3<S>> original_centers;
            for (const auto& geometry : model.collisionGeometries) {
                CollisionObject<S> object(*geometry);
                require(object.aabb.min_.allFinite() && object.aabb.max_.allFinite(), "Non-finite bounds");
                original_centers.push_back(object.obb.To);
            }

            // Match the OpenMP baseline's copied-model pose path.
            URDFModel local = model;
            std::map<std::string, double> angles;
            for (const auto& joint : local.joints)
                if (joint.type == "revolute" || joint.type == "continuous") angles[joint.name] = 0.37;
            if (std::string(robot.name) == "prbt")
                for (int i = 1; i <= 6; ++i)
                    require(angles.count("prbt_joint_" + std::to_string(i)) == 1, "PRBT joint name mismatch");
            local.setJointAngles(angles);
            local.calculateWorldCoordinates(robot.root);
            size_t moved = 0;
            for (size_t i = 0; i < local.collisionGeometries.size(); ++i) {
                const auto& geometry = local.collisionGeometries[i];
                const auto& pose = local.links.at(geometry->parentLink->name);
                CollisionObject<S> object(*geometry, pose);
                const Vector3<S> expected = pose.position + pose.rotation * geometry->obb.center();
                require((object.obb.To - expected).norm() < 1e-8, "Copied-model pose mismatch");
                if ((object.obb.To - original_centers[i]).norm() > 1e-6) ++moved;
                CollisionObject<S> original(*model.collisionGeometries[i]);
                require((original.obb.To - original_centers[i]).norm() < 1e-8, "Original model mutated");
            }
            require(moved > 0, std::string("Joint sampling did not move ") + robot.name);
            std::cout << robot.name << ": " << model.collisionGeometries.size()
                      << " collision objects; copied poses OK\n";
        }
        std::cout << "BACON smoke check passed\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "BACON smoke check failed: " << e.what() << '\n';
        return 1;
    }
}
