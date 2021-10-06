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

// Note: The original code is from 
// https://github.com/ros/urdf/blob/e84c0ce88a37cf78ac456c7fae4b4e9bd7430069/urdf/test/test_robot_model_parser.cpp

#include "urdf/model.h"

class URDFParser {
public:
    URDFParser(std::string file) {
        this->robot.initFile(file);
    }

    bool checkModel() {
        // get root link
        urdf::LinkConstSharedPtr root_link = robot.getRoot();
        if (!root_link) {
            std::cerr << "no root link " << robot.getName().c_str() << std::endl;
            return false;
        }

        model_str = "RobotType { name " + robot.getName() + "\n";

        std::cout << "link name: " << root_link->name << std::endl;
        print_link(root_link);

        // go through entire tree
        return traverse_tree(root_link);
    }

    std::string generateModelString() {
        if(!joint_str.empty()) {
            joint_str.pop_back();
            joint_str.pop_back();
            model_str += joint_str + "}\n";
        }
        if(!link_str.empty()) {
            link_str.pop_back();
            link_str.pop_back();
            model_str += link_str + "}\n";
        }
        model_str += "}";
        return model_str;
    }

    std::string getModelString() {
        return model_str;
    }

private:
    size_t num_joints;
    size_t num_links;

    std::string model_str;
    std::string link_str;
    std::string joint_str;

    urdf::Model robot;


    // the idea is the link and joint classes are auto-generated from the model
    // so the link and object classes used below (which currently are defined in urdfdom repo),
    // are from the same model
    // so once the URDF has been parsed, the tree can be traversed as below to generate the model string already
    // for each model element, there has to be a function to convert it into DSL string
    bool traverse_tree(urdf::LinkConstSharedPtr link, int level = 0) {
        std::cout << "Traversing tree at level " << level << " link size " << link->child_links.size() << std::endl;
        level += 2;
        bool retval = true;
        for (const urdf::LinkSharedPtr & child : link->child_links) {
            std::cout << "link name: " << child->name << std::endl;
            print_link(child);
            ++num_links;
            if (child && child->parent_joint) {
                std::cout << "joint name: " << child->parent_joint->name << std::endl;
                print_joint(child->parent_joint);
                ++num_joints;
                // check rpy
                double roll, pitch, yaw;
                child->parent_joint->parent_to_joint_origin_transform.rotation.getRPY(roll, pitch, yaw);

                if (std::isnan(roll) || std::isnan(pitch) || std::isnan(yaw)) {
                    std::cerr << "getRPY() returned nan!" <<std::endl;;
                    return false;
                }
                // recurse down the tree
                retval &= this->traverse_tree(child, level);
            } else {
                std::cout << "root link: " << link->name.c_str() << " has a null child!" << std::endl;
                return false;
            }
        }
        // no more children
        return retval;
    }

    void print_link(urdf::LinkConstSharedPtr link) {
        if(link_str.empty()) {
            link_str = "\tlink {\n";
        }
        link_str += "\t Link { name " + link->name + "},\n";
    }

    void print_joint(urdf::JointConstSharedPtr joint) {
        if(joint_str.empty()) {
            joint_str = "\tjoint {\n";
        }
        joint_str += "\tJoint {\n \
	\tname " + joint->name + "\n \
	\ttype " + "revolute" + "\n \
	\tparent Parent { link " + joint->parent_link_name + "}\n \
	\tchild Child { link " + joint->child_link_name + "}\n \
	},\n";
    }
};

int main(int argc, char** argv)
{
    std::cout << "Hello World " << std::endl;
    URDFParser parser(argv[1]);
    bool isValid = parser.checkModel();
    std::cout << "Is tree valid? " << isValid << std::endl;
    std::cout << parser.generateModelString() << std::endl;
    return 0;
}