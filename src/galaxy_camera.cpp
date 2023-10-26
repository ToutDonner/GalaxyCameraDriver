//
// Created by qiayuan on 6/27/20.
//

#include <galaxy_camera.h>
#include <stdio.h>

#include <iostream>
#include <string>
#include <utility>

using namespace std;
namespace galaxy_camera {

GalaxyCamera::GalaxyCamera(const ros::NodeHandle &nh,
                           image_transport::CameraPublisher &pub,
                           sensor_msgs::CameraInfo info, uint32_t height,
                           uint32_t width, uint32_t step, uint32_t offset_x,
                           uint32_t offset_y, const std::string &encoding,
                           std::string device_sn) {
  nh_ = nh;
  pub_ = pub;
  info_ = std::move(info);
  image_.height = height;
  image_.width = width;
  image_.step = step;
  image_.data.resize(height * step);
  image_.encoding = encoding;
  img = new char[height * step];
  img2 = new char[height * step];
  assert(GXInitLib() == GX_STATUS_SUCCESS);  // Initializes the library.
  uint32_t device_num = 0;
  GXUpdateDeviceList(&device_num, 1000);
  // assert(device_num == 1); // TODO add multi camera support.
  //  Opens the device.
  GX_OPEN_PARAM open_param;
  open_param.accessMode = GX_ACCESS_CONTROL;
  open_param.openMode = GX_OPEN_SN;

  char c[20];
  strcpy(c, device_sn.c_str());
  open_param.pszContent = c;
  // open_param.openMode = GX_OPEN_INDEX;
  // open_param.pszContent = (char *) "1";
  GX_STATUS status = GXOpenDevice(&open_param, &dev_handle_);
  ROS_INFO("!!!!!!!!device_sn %s", device_sn.c_str());
  // cout<<"status:"<<status<<endl;
  //  Get handle
  assert(GXOpenDevice(&open_param, &dev_handle_) != GX_STATUS_SUCCESS);
  ROS_INFO("Camera Opened");
  int64_t nLinkThroughputval;
  GXGetInt(dev_handle_, GX_INT_DEVICE_LINK_CURRENT_THROUGHPUT,
           &nLinkThroughputval);
  // double sizeofImage = 1280 * 1024 / 10000 * 0.0286;
  double sizeofImage = height * width / 10000 * 0.0286;
  double translateSpeed = nLinkThroughputval / 1000000 / 8;
  timeCom = sizeofImage / translateSpeed;
  nh_.param("exposure_value", exposureCom, 0.0);
  exposureCom /= 1000000;
  cout << "------------>BandWidth:" << nLinkThroughputval << "BPs" << endl;
  cout << "------------>Tize of image:" << sizeofImage << "MB" << endl;
  cout << "------------>Translate Speed:" << translateSpeed << "MB/s" << endl;
  cout << "------------>Translate Time:" << timeCom << "s" << endl;
  cout << "------------>Exposure time:" << exposureCom << "s" << endl;
  cout << "------------>Totally Compasation:" << timeCom + exposureCom << "s"
       << endl;

  int64_t format = 0;
  if (encoding == "mono8") format = GX_PIXEL_FORMAT_MONO8;
  if (encoding == "mono16") format = GX_PIXEL_FORMAT_MONO16;
  if (encoding == "bgr8") format = GX_PIXEL_FORMAT_BAYER_GB8;
  if (encoding == "rgb8") format = GX_PIXEL_FORMAT_BAYER_RG8;
  if (encoding == "bgra8") format = GX_PIXEL_FORMAT_BAYER_BG8;
  if (format == 0) static_assert(true, "Illegal format");

  // assert(GXSetEnum(dev_handle_, GX_ENUM_PIXEL_FORMAT, format) ==
  // GX_STATUS_SUCCESS); assert(GXSetInt(dev_handle_, GX_INT_WIDTH, width) ==
  // GX_STATUS_SUCCESS); assert(GXSetInt(dev_handle_, GX_INT_HEIGHT, height) ==
  // GX_STATUS_SUCCESS); assert(GXSetInt(dev_handle_, GX_INT_OFFSET_X, offset_x)
  // == GX_STATUS_SUCCESS); assert(GXSetInt(dev_handle_, GX_INT_OFFSET_Y,
  // offset_y) == GX_STATUS_SUCCESS);
  // write config to camera
  ROS_INFO("Writing parameters to camera...");
  writeConfig();

  GXRegisterCaptureCallback(dev_handle_, nullptr, GalaxyCamera::onFrameCB);
  GXStreamOn(dev_handle_);
  ROS_INFO("Stream On.");

  ROS_INFO("Done.");
}

void GalaxyCamera::onFrameCB(GX_FRAME_CALLBACK_PARAM *pFrame) {
  if (pFrame->status == GX_FRAME_STATUS_SUCCESS) {
    double rosTime = ros::Time().now().toSec();
    DxRaw8toRGB24((void *)pFrame->pImgBuf, img, pFrame->nWidth, pFrame->nHeight,
                  RAW2RGB_NEIGHBOUR, BAYERBG, false);

    // color correct
    GXGetInt(dev_handle_, GX_INT_COLOR_CORRECTION_PARAM, &colorcorrect_);

    // contrast
    int nLutLength;
    assert(GXGetInt(dev_handle_, GX_INT_CONTRAST_PARAM, &contrast_param_) ==
           GX_STATUS_SUCCESS);
    assert(DxGetContrastLut(contrast_param_, NULL, &nLutLength) == DX_OK);
    float *pContrastLut = new float[nLutLength];
    assert(DxGetContrastLut(contrast_param_, pContrastLut, &nLutLength) ==
           DX_OK);

    //计算gamma
    GXGetFloat(dev_handle_, GX_FLOAT_GAMMA_PARAM, &gamma_param_);
    assert(DxGetGammatLut(gamma_param_, NULL, &nLutLength) == DX_OK);
    float *pGammaLut = new float[nLutLength];
    assert(DxGetGammatLut(gamma_param_, pGammaLut, &nLutLength) == DX_OK);

    // 画质增强   colorcorrect_  pContrastLut  pGammaLut
    assert(DxImageImprovment(img, img, pFrame->nWidth, pFrame->nHeight,
                             colorcorrect_, pContrastLut, pGammaLut) == DX_OK);

    memcpy((char *)(&image_.data[0]), img, image_.step * image_.height);

    ros::Time now = ros::Time().fromSec(rosTime - timeCom - exposureCom);
    image_.header.stamp = now;
    info_.header.stamp = now;
    pub_.publish(image_, info_);
  }
}

void GalaxyCamera::writeConfig() {
  double frame_rate, exposure_max, exposure_min, exposure_value, gain_min,
      gain_max, gain_value, black_value, white_value, gamma_value;
  bool exposure_auto, gain_auto, black_auto, white_auto;
  int white_selector;

  // get parameters
  nh_.param("frame_rate", frame_rate, 30.0);

  nh_.param("exposure_auto", exposure_auto, true);
  nh_.param("exposure_min", exposure_min, 30000.0);
  nh_.param("exposure_max", exposure_max, 32000.0);
  nh_.param("exposure_value", exposure_value, 0.0);

  nh_.param("gain_auto", gain_auto, true);
  nh_.param("gain_min", gain_min, 0.0);
  nh_.param("gain_max", gain_max, 32.0);
  nh_.param("gain_value", gain_value, 16.0);

  nh_.param("black_auto", black_auto, true);
  nh_.param("black_value", black_value, 2.0);

  nh_.param("white_auto", white_auto, true);
  nh_.param("white_selector", white_selector, 0);
  nh_.param("white_value", white_value, 2.3867);

  nh_.param("gamma_value", gamma_value, 1.60);

  // write to camera
  // Frame Rate
  GXSetEnum(dev_handle_, GX_ENUM_ACQUISITION_FRAME_RATE_MODE,
            GX_ACQUISITION_FRAME_RATE_MODE_ON);
  GXSetFloat(dev_handle_, GX_FLOAT_ACQUISITION_FRAME_RATE, frame_rate);

  // Exposure
  if (exposure_auto) {
    GXSetFloat(dev_handle_, GX_FLOAT_AUTO_EXPOSURE_TIME_MAX, exposure_max);
    GXSetFloat(dev_handle_, GX_FLOAT_AUTO_EXPOSURE_TIME_MIN, exposure_min);
    GXSetEnum(dev_handle_, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_CONTINUOUS);
  } else {
    GXSetEnum(dev_handle_, GX_ENUM_EXPOSURE_AUTO, GX_EXPOSURE_AUTO_OFF);
    GXSetFloat(dev_handle_, GX_FLOAT_EXPOSURE_TIME, exposure_value);
  }

  // Gain
  if (gain_auto) {
    GXSetFloat(dev_handle_, GX_FLOAT_AUTO_GAIN_MIN, gain_min);
    GXSetFloat(dev_handle_, GX_FLOAT_AUTO_GAIN_MAX, gain_max);
    GXSetEnum(dev_handle_, GX_ENUM_GAIN_AUTO, GX_GAIN_AUTO_CONTINUOUS);
  } else {
    GXSetEnum(dev_handle_, GX_ENUM_GAIN_AUTO, GX_GAIN_AUTO_OFF);
    GXSetFloat(dev_handle_, GX_FLOAT_GAIN, gain_value);
  }

  // Black level
  if (black_auto) {
    GXSetEnum(dev_handle_, GX_ENUM_BLACKLEVEL_AUTO,
              GX_BLACKLEVEL_AUTO_CONTINUOUS);
  } else {
    GXSetEnum(dev_handle_, GX_ENUM_BLACKLEVEL_AUTO, GX_BLACKLEVEL_AUTO_OFF);
    GXSetFloat(dev_handle_, GX_FLOAT_BLACKLEVEL, black_value);
  }
  // Gamma
  if (gamma_value) {
    GXSetBool(dev_handle_, GX_BOOL_GAMMA_ENABLE, true);
    GX_GAMMA_MODE_ENTRY nValue;
    // nValue = GX_GAMMA_SELECTOR_USER;
    nValue = GX_GAMMA_SELECTOR_SRGB;
    GXSetEnum(dev_handle_, GX_ENUM_GAMMA_MODE, nValue);

    // GXSetFloat(dev_handle_, GX_FLOAT_GAMMA_PARAM, gamma_value);
  }

  // Balance White
  switch (white_selector) {
    case 0:
      GXSetEnum(dev_handle_, GX_ENUM_BALANCE_RATIO_SELECTOR,
                GX_BALANCE_RATIO_SELECTOR_RED);
      break;
    case 1:
      GXSetEnum(dev_handle_, GX_ENUM_BALANCE_RATIO_SELECTOR,
                GX_BALANCE_RATIO_SELECTOR_GREEN);
      break;
    case 2:
      GXSetEnum(dev_handle_, GX_ENUM_BALANCE_RATIO_SELECTOR,
                GX_BALANCE_RATIO_SELECTOR_BLUE);
      break;
  }
  if (white_auto) {
    GXSetEnum(dev_handle_, GX_ENUM_BALANCE_WHITE_AUTO,
              GX_BALANCE_WHITE_AUTO_CONTINUOUS);
  } else {
    GXSetEnum(dev_handle_, GX_ENUM_BALANCE_WHITE_AUTO,
              GX_BALANCE_WHITE_AUTO_OFF);
    GXSetFloat(dev_handle_, GX_FLOAT_BALANCE_RATIO, white_value);
  }
}

GalaxyCamera::~GalaxyCamera() {
  GXStreamOff(dev_handle_);
  GXUnregisterCaptureCallback(dev_handle_);
  GXCloseDevice(dev_handle_);
  GXCloseLib();
}

GX_DEV_HANDLE GalaxyCamera::dev_handle_;
char *GalaxyCamera::img;
char *GalaxyCamera::img2;
sensor_msgs::Image GalaxyCamera::image_;
image_transport::CameraPublisher GalaxyCamera::pub_;
sensor_msgs::CameraInfo GalaxyCamera::info_;
double GalaxyCamera::gamma_param_ = 0;
int64_t GalaxyCamera::colorcorrect_{};
int64_t GalaxyCamera::contrast_param_{};
double GalaxyCamera::timeCom = 0;
double GalaxyCamera::exposureCom = 0;
double GalaxyCamera::x = 1.6;
}  // namespace galaxy_camera
