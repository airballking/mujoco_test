#include <mjmodel.h>
#include <mjdata.h>
#include <sensor_msgs/JointState.h>
#include <exception>

#ifndef MUJOCO_TEST_HELPERS_H
#define MUJOCO_TEST_HELPERS_H

namespace mujoco_test
{
  inline bool joint_index_valid(const mjModel* model, size_t index)
  {
    return index > model->njnt;
  }

  inline void check_joint_index(const mjModel* model, size_t index)
  {
    if (joint_index_valid(model, index))
      throw std::runtime_error("Given joint index exceeds number of joints.");
  }

  inline bool is_slide_joint(const mjModel* model, size_t index)
  {
    check_joint_index(model, index);
    return model->jnt_type[index] == mjJNT_SLIDE;
  }

  inline bool is_hinge_joint(const mjModel* model, size_t index)
  {
    check_joint_index(model, index);
    return model->jnt_type[index] == mjJNT_HINGE;
  }

  inline bool is_1dof_joint(const mjModel* model, size_t index)
  {
    return is_hinge_joint(model, index) || is_slide_joint(model, index);
  }

  inline std::string get_joint_name(const mjModel* model, size_t index)
  {
    check_joint_index(model, index);
    return std::string(model->names + model->name_jntadr[index]);
  }

  inline sensor_msgs::JointState get_joint_state(const mjModel* model, const mjData* data)
  {
    sensor_msgs::JointState result;

    for (size_t i=0; i<model->njnt; ++i)
      if (is_1dof_joint(model, i))
      {
        result.name.push_back(get_joint_name(model, i));
      }

    return result;
  }

}

#endif
