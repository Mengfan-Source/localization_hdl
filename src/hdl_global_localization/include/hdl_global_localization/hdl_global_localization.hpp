#include <iostream>
#include <boost/filesystem.hpp>

#include <ros/ros.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl_ros/point_cloud.h>
#include <hdl_global_localization/SetGlobalMap.h>
#include <hdl_global_localization/QueryGlobalLocalization.h>
#include <hdl_global_localization/SetGlobalLocalizationEngine.h>

#include <hdl_global_localization/engines/global_localization_bbs.hpp>
#include <hdl_global_localization/engines/global_localization_fpfh_ransac.hpp>
#include <hdl_global_localization/engines/global_localization_fpfh_teaser.hpp>