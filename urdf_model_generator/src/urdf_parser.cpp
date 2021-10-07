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

URDF_TYPEDEF_CLASS_POINTER(LinkXtext);

namespace urdf {    // urdf namespace

class Vector3Xtext : public Vector3 {
public:
    std::string dump_xtext() const {
        if (!isSet()) {
            return "";
        }
        return "xyz \"" + std::to_string(this->x) + " " + std::to_string(this->y) + " " + std::to_string(this->z) + "\"";
    }

    bool isSet() const {
        return !(this->x == this->y == this->z == 0.0);
    }
};


class RotationXtext : public Rotation {
public:
    std::string dump_xtext() const {
        if(!isSet()) {
            return "";
        }
        return "rpy \"" + std::to_string(this->x) + " " + std::to_string(this->y) + " " + std::to_string(this->z) + " " + std::to_string(this->w) + "\"";
    }

    bool isSet() const {
        return !(this->x == this->y == this->z == 0.0 && this->w == 1.0);
    }
};


class PoseXtext : public Pose {
public:
    std::string dump_xtext() const {
        const Vector3Xtext& pos = static_cast<const Vector3Xtext&>(this->position);
        const RotationXtext& rot = static_cast<const RotationXtext&>(this->rotation);
        return "Pose { " + rot.dump_xtext() + " " + pos.dump_xtext() + " }";
    }

    bool isSet() const {
        const Vector3Xtext& pos = static_cast<const Vector3Xtext&>(this->position);
        const RotationXtext& rot = static_cast<const RotationXtext&>(this->rotation);
        return pos.isSet() && rot.isSet();
    }
};

class JointXtext : public Joint {
public:
    std::string dump_xtext() const {
        std::string xtext_str = "\t\tJoint {\n \
	\tname " + this->name + "\n \
	\ttype " + type_str[this->type] + "\n";

        const PoseXtext& origin = static_cast<const PoseXtext&>(this->parent_to_joint_origin_transform);
        if(origin.isSet()) {
            xtext_str += "\t\torigin " + origin.dump_xtext() + " \n";
        }

	    xtext_str += "\t\tparent Parent { link " + this->parent_link_name + " }\n \
	\tchild Child { link " + this->child_link_name + " }\n";

        const Vector3Xtext& axis_ = static_cast<const Vector3Xtext&>(this->axis);
        if(axis_.isSet()) {
            xtext_str += "\t\taxis Axis { " + axis_.dump_xtext() + " }\n";
        }

        xtext_str += "},\n";
        return xtext_str;
    }
};
typedef std::shared_ptr<const JointXtext> JointXtextConstSharedPtr;


class LinkXtext : public Link {
public:
    std::string dump_xtext() const {
        return "\t\t Link { name " + this->name + " },\n";
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
        model_file << dump_xtext();
        model_file.close();
        return true;
    }

    std::string dump_xtext() {
        LinkXtextConstSharedPtr root_link = getRootLink();
        if (!root_link) {
            std::cerr << "no root link " << this->getName().c_str() << std::endl;
            return "";
        }

        xtext_str = "RobotType { name " + this->getName() + "\n";
        link_str = "\tlink {\n" + root_link->dump_xtext();
        traverse_tree(root_link);
        return compile_xtext();
    }

    LinkXtextConstSharedPtr getRootLink() const {
        return std::static_pointer_cast<const urdf::LinkXtext>(this->getRoot());
    }

private:
    std::string xtext_str, link_str, joint_str;

    std::string compile_xtext() {
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
        xtext_str += "}";
        return xtext_str;
    }

    // Note: The original code is from
    // https://github.com/ros/urdf/blob/e84c0ce88a37cf78ac456c7fae4b4e9bd7430069/urdf/test/test_robot_model_parser.cpp
    bool traverse_tree(LinkXtextConstSharedPtr link, int level = 0) {
        level += 2;
        bool retval = true;
        for (const LinkConstSharedPtr & child : link->child_links) {
            LinkXtextConstSharedPtr child_ = std::static_pointer_cast<const LinkXtext>(child);
            link_str += child_->dump_xtext();

            if (child_ && child_->parent_joint) {
                JointXtextConstSharedPtr joint = std::static_pointer_cast<const JointXtext>(child->parent_joint);
                joint_str += joint->dump_xtext();

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
    // std::cout << model.dump_xtext() << std::endl;
    model.save(argv[2]);
    return 0;
}
