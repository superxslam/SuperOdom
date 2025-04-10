//
// Created by shibo zhao on 2020/6/29.
//

#ifndef POINT_OS_H
#define POINT_OS_H

#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>

namespace point_os {
struct EIGEN_ALIGN16 PointOS {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PCL_ADD_POINT4D;
  float intensity;
  float t;

  static inline PointOS make(float x, float y, float z, float intensity,
                             float t) {
    return {x, y, z, 0.0f, intensity, t};
  }
};

struct EIGEN_ALIGN16 PointcloudXYZITR {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PCL_ADD_POINT4D;
  float intensity;
  float time;
  uint16_t ring;
  static inline PointcloudXYZITR make(float x, float y, float z,
                                      float intensity, float t, uint16_t ring) {
    return {x, y, z, 0.0f, intensity, t, ring};
  }
};


struct OusterPointXYZIRT {
        PCL_ADD_POINT4D;
        float intensity;
        uint32_t t;
        uint16_t reflectivity;
        // uint8_t ring;
        // uint16_t noise;
        uint32_t range;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    } EIGEN_ALIGN16;

} // namespace point_os

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT( point_os::PointOS,
    (float, x,x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, t, t)
    )


POINT_CLOUD_REGISTER_POINT_STRUCT( point_os::PointcloudXYZITR,
   (float, x,x)
   (float, y, y)
   (float, z, z)
   (float, intensity, intensity)
   (float, time, time)
   (uint16_t, ring, ring)
)

// POINT_CLOUD_REGISTER_POINT_STRUCT(point_os::OusterPointXYZIRT,
//                                   (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
//                                           (uint32_t, t, t) (uint16_t, reflectivity, reflectivity)
//                                           (uint8_t, ring, ring) (uint16_t, noise, noise) (uint32_t, range, range)
// )

POINT_CLOUD_REGISTER_POINT_STRUCT(point_os::OusterPointXYZIRT,
                                  (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
                                          (uint32_t, t, t) (uint16_t, reflectivity, reflectivity)
                                          (uint32_t, range, range)
)


// clang-format on
#endif // POINT_OS_H
