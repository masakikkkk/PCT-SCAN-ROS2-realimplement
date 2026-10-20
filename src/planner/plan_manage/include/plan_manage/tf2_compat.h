#ifndef PLAN_MANAGE_TF2_COMPAT_H
#define PLAN_MANAGE_TF2_COMPAT_H

// Foxy uses .h; newer geometry2 versions also provide .hpp headers.
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

#if __has_include(<tf2/utils.hpp>)
#include <tf2/utils.hpp>
#else
#include <tf2/utils.h>
#endif

#endif
