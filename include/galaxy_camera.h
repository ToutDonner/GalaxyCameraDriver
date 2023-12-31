//
// Created by qiayuan on 6/27/20.
//

#ifndef SRC_GALAXY_CAMERA_INCLUDE_GALAXY_CAMERA_H_
#define SRC_GALAXY_CAMERA_INCLUDE_GALAXY_CAMERA_H_

#include <image_transport/image_transport.h>

#include "libgxiapi/DxImageProc.h"
#include "libgxiapi/GxIAPI.h"

namespace galaxy_camera {
class GalaxyCamera {
 public:
  GalaxyCamera(const ros::NodeHandle &nh, image_transport::CameraPublisher &pub,
               sensor_msgs::CameraInfo info, uint32_t height, uint32_t width,
               uint32_t step, uint32_t offset_x, uint32_t offset_y,
               const std::string &encoding, std::string device_sn);

  ~GalaxyCamera();

  static sensor_msgs::Image image_;

 private:
  void writeConfig();

  static GX_DEV_HANDLE dev_handle_;

  int last_channel_ = 0;
  ros::NodeHandle nh_;

  static double gamma_param_;
  static int64_t colorcorrect_;
  static int64_t contrast_param_;
  static char *img;
  static char *img2;
  static image_transport::CameraPublisher pub_;
  static sensor_msgs::CameraInfo info_;
  static double timeCom;
  static double exposureCom;
  static double x;
  static void GX_STDC onFrameCB(GX_FRAME_CALLBACK_PARAM *pFrame);
};

}  // namespace galaxy_camera

#endif  // SRC_GALAXY_CAMERA_INCLUDE_GALAXY_CAMERA_H_
