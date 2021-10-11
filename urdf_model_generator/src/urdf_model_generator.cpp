/***************************************************************************************
* Copyright 2021 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*   http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
***************************************************************************************/


#include <fstream>
#include "urdf/model.h"


static const std::string type_str[] = {"unknown", "revolute", "continuous",
    "prismatic", "floating", "planar", "fixed"};

namespace urdf {    // urdf namespace

class Vector3Xtext : public Vector3 {
public:
    std::string dumpXtext(std::string keyword = "xyz") const {
        if (!isSet()) {
            return "";
        }
        return keyword + " \"" + std::to_string(this->x) + " " + std::to_string(this->y) + " " + std::to_string(this->z) + "\"";
    }

    bool isSet() const {
        return !(this->x == 0.0 && this->y == 0.0 && this->z == 0.0);
    }
};


class RotationXtext : public Rotation {
public:
    std::string dumpXtext() const {
        if(!isSet()) {
            return "";
        }
        double roll, pitch, yaw;
        this->getRPY(roll, pitch, yaw);
        return "rpy \"" + std::to_string(roll) + " " + std::to_string(pitch) + " " + std::to_string(yaw) + "\"";
    }

    bool isSet() const {
        return !(this->x == this->y == this->z == 0.0 && this->w == 1.0);
    }
};


class PoseXtext : public Pose {
public:
    std::string dumpXtext() const {
        const Vector3Xtext& pos = static_cast<const Vector3Xtext&>(this->position);
        const RotationXtext& rot = static_cast<const RotationXtext&>(this->rotation);
        return "Pose { " + rot.dumpXtext() + " " + pos.dumpXtext() + " }";
    }

    bool isSet() const {
        const Vector3Xtext& pos = static_cast<const Vector3Xtext&>(this->position);
        const RotationXtext& rot = static_cast<const RotationXtext&>(this->rotation);
        return pos.isSet() && rot.isSet();
    }
};


class JointXtext : public Joint {
public:
    std::string dumpXtext() const {
        std::string xtext_str = "\t\tJoint {\n \
\t\t\tname " + this->name + "\n \
\t\t\ttype " + type_str[this->type];

        const PoseXtext& origin = static_cast<const PoseXtext&>(this->parent_to_joint_origin_transform);
        if(origin.isSet()) {
            xtext_str += "\n\t\t\torigin " + origin.dumpXtext();
        }

	    xtext_str += "\n\t\t\tparent Parent { link " + this->parent_link_name + " }\n \
\t\t\tchild Child { link " + this->child_link_name + " }";

        const Vector3Xtext& axis_ = static_cast<const Vector3Xtext&>(this->axis);
        if(axis_.isSet()) {
            xtext_str += "\n\t\t\taxis Axis { " + axis_.dumpXtext() + " }";
        }

        xtext_str += " },\n";
        return xtext_str;
    }
};
typedef std::shared_ptr<const JointXtext> JointXtextConstSharedPtr;


class GeometryXtext : public Geometry{
public:
    std::string dumpXtext() const {
        return "\t\t\t\tgeometry Geometry {";
    }
};
typedef std::shared_ptr<GeometryXtext> GeometryXtextSharedPtr;


class MeshXtext : public Mesh {
public:
    std::string dumpXtext() const {
        return "\n\t\t\t\t\tmesh Mesh { filename \"" + this->filename + "\" }";
        // TODO: scale
    }
};
typedef std::shared_ptr<MeshXtext> MeshXtextSharedPtr;


class BoxXtext : public Box {
public:
    std::string dumpXtext() const {
        std::string xtext_str = "\n\t\t\t\t\tbox Box { ";
        const Vector3Xtext& dim = static_cast<const Vector3Xtext&>(this->dim);
        xtext_str += dim.dumpXtext("size") + " }";;
        return xtext_str;
    }
};
typedef std::shared_ptr<BoxXtext> BoxXtextSharedPtr;


class GeometryHandler {
protected:
    std::string dumpGeometryXtext(GeometrySharedPtr geom) const {
        std::string xtext_str = "";
        GeometryXtextSharedPtr geometry = std::static_pointer_cast<GeometryXtext>(geom);
        if(geometry) {
            xtext_str += geometry->dumpXtext();
            if(geom->type == Geometry::MESH) {
                MeshXtextSharedPtr mesh = std::static_pointer_cast<MeshXtext>(geom);
                xtext_str += mesh->dumpXtext();
            } else if(geom->type == Geometry::BOX) {
                BoxXtextSharedPtr box = std::static_pointer_cast<BoxXtext>(geom);
                std::cout << box->dim.x << std::endl;
                xtext_str += box->dumpXtext();
            }
            xtext_str += " }";
        }
        return xtext_str;
    }
};


// class MassXtext : public Mass {
// public:
//     std::string dumpXtext(double mass) const {
//         return "\n\t\t\t\tmass Mass { " + std::to_string(this->mass);
//     }
// };


class InertialXtext : public Inertial {
//   Pose origin;
//   double mass;
//   double ixx,ixy,ixz,iyy,iyz,izz;

public:
    std::string dumpXtext() const {
        std::string xtext_str = "\n\t\t\tinertial Inertial {";
        const PoseXtext& origin = static_cast<const PoseXtext&>(this->origin);
        xtext_str += "\n\t\t\t\torigin " + origin.dumpXtext();
        xtext_str +=  "\n\t\t\t\tmass Mass { value " + std::to_string(this->mass) + " }";
        xtext_str += "\n\t\t\t\tinertia Inertia { ixx " +  std::to_string(this->ixx) + " " +
            "ixy " +  std::to_string(this->ixy) + " " +
            "ixz " +  std::to_string(this->ixz) + " " +
            "iyy " +  std::to_string(this->iyy) + " " +
            "iyz " +  std::to_string(this->iyz) + " " +
            "izz " +  std::to_string(this->izz) + " }";

        xtext_str += " }";

        return xtext_str;
    }
};
typedef std::shared_ptr<InertialXtext> InertialXtextSharedPtr;


class MaterialXtext : public Material {

};


class VisualXtext : public Visual, GeometryHandler {
public:
    std::string dumpXtext() const {
        std::string xtext_str = "\n\t\t\tvisual Visual {" + this->name + "\n";

        const PoseXtext& origin = static_cast<const PoseXtext&>(this->origin);
        if(origin.isSet()) {
            xtext_str += "\n\t\torigin " + origin.dumpXtext();
        }

        xtext_str += dumpGeometryXtext(this->geometry) + " }";;
        return xtext_str;
    }

    bool isSet() {
        const PoseXtext& origin = static_cast<const PoseXtext&>(this->origin);
        // this is not enough
        return origin.isSet();
    }
};
typedef std::shared_ptr<VisualXtext> VisualXtextSharedPtr;


class CollisionXtext : public Collision, GeometryHandler {
public:
    std::string dumpXtext() const {
        std::string xtext_str = "\n\t\t\tcollision Collision {" + this->name + "\n";

        const PoseXtext& origin = static_cast<const PoseXtext&>(this->origin);
        if(origin.isSet()) {
            xtext_str += "\n\t\torigin " + origin.dumpXtext();
        }

        xtext_str += dumpGeometryXtext(this->geometry) + " }";;
        return xtext_str;
    }
};
typedef std::shared_ptr<CollisionXtext> CollisionXtextSharedPtr;


class LinkXtext : public Link {
public:
    std::string dumpXtext() const {
        std::string xtext_str = "\t\tLink { name " + this->name;
        InertialXtextSharedPtr inertial = std::static_pointer_cast<InertialXtext>(this->inertial);
        if(inertial) {
            xtext_str += inertial->dumpXtext();
        }
        VisualXtextSharedPtr visual = std::static_pointer_cast<VisualXtext>(this->visual);
        if(visual) {
            xtext_str += visual->dumpXtext();
        }
        CollisionXtextSharedPtr collision = std::static_pointer_cast<CollisionXtext>(this->collision);
        if(collision) {
            xtext_str += collision->dumpXtext();
        }
        xtext_str += " },\n";
        return xtext_str;
    }
};
typedef std::shared_ptr<const LinkXtext> LinkXtextConstSharedPtr;


class ModelXtext : public Model {
public:
    ModelXtext(std::string file) : Model() {
        this->initFile(file);
    }

    bool save(std::string filename) {
        std::ofstream model_file;
        model_file.open(filename);
        model_file << dumpXtext();
        model_file.close();
        return true;
    }

    std::string dumpXtext() {
        LinkXtextConstSharedPtr root_link = getRootLink();
        if (!root_link) {
            std::cerr << "no root link " << this->getName().c_str() << std::endl;
            return "";
        }

        xtext_str = "RobotType { name " + this->getName() + "\n";
        link_str = "\tlink {\n" + root_link->dumpXtext();
        traverse_tree(root_link);
        return compileXtext();
    }

    LinkXtextConstSharedPtr getRootLink() const {
        return std::static_pointer_cast<const urdf::LinkXtext>(this->getRoot());
    }

private:
    std::string xtext_str, link_str, joint_str;

    std::string compileXtext() {
        if(!joint_str.empty()) {
            joint_str.pop_back();
            joint_str.pop_back();
            xtext_str += "\tjoint {\n" + joint_str + " }\n";
        }
        if(!link_str.empty()) {
            link_str.pop_back();
            link_str.pop_back();
            xtext_str += link_str + " }\n";
        }
        xtext_str += "}\n";
        return xtext_str;
    }

    // Note: The original code is from
    // https://github.com/ros/urdf/blob/e84c0ce88a37cf78ac456c7fae4b4e9bd7430069/urdf/test/test_robot_model_parser.cpp
    // <transmission> is PR2 specific extension, ignoring
    bool traverse_tree(LinkXtextConstSharedPtr link, int level = 0) {
        level += 2;
        bool retval = true;
        for (const LinkConstSharedPtr & child : link->child_links) {
            LinkXtextConstSharedPtr child_ = std::static_pointer_cast<const LinkXtext>(child);
            link_str += child_->dumpXtext();

            if (child_ && child_->parent_joint) {
                JointXtextConstSharedPtr joint = std::static_pointer_cast<const JointXtext>(child->parent_joint);
                joint_str += joint->dumpXtext();

                // check rpy
                double roll, pitch, yaw;
                child_->parent_joint->parent_to_joint_origin_transform.rotation.getRPY(roll, pitch, yaw);

                if (std::isnan(roll) || std::isnan(pitch) || std::isnan(yaw)) {
                    std::cerr << "getRPY() returned nan!" <<std::endl;;
                    return false;
                }
                // recurse down the tree
                retval &= this->traverse_tree(child_, level);
            } else {
                std::cout << "root link: " << link->name.c_str() << " has a null child!" << std::endl;
                return false;
            }
        }

        // no more children
        return retval;
    }
};

}   // urdf namespace


int main(int argc, char** argv)
{
    urdf::ModelXtext model(argv[1]);
    // std::cout << model.dumpXtext() << std::endl;
    model.save(argv[2]);
    return 0;
}
