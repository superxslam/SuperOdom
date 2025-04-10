//
// Created by ubuntu on 2020/9/14.
//

#ifndef LIDARPOINT_H
#define LIDARPOINT_H
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct EIGEN_ALIGN16 _PointXYZTIId {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PCL_ADD_POINT4D
  float time;
  uint8_t intensity;
  uint8_t laserId;
} EIGEN_ALIGN16;

PCL_EXPORTS std::ostream &operator<<(std::ostream &os, const _PointXYZTIId &p);

/** \brief A point structure representing Euclidean xyz coordinates, time,
 * intensity, and laserId. \ingroup common
 */
struct PointXYZTIId : public _PointXYZTIId {
  inline PointXYZTIId(const _PointXYZTIId &p) {
    x = p.x;
    y = p.y;
    z = p.z;
    data[3] = 1.0f;
    time = p.time;
    intensity = p.intensity;
    laserId = p.laserId;
  }

  inline PointXYZTIId() {
    x = 0.0f;
    y = 0.0f;
    z = 0.0f;
    data[3] = 1.0f;
    time = 0.0;
    intensity = 0;
    laserId = 0;
  }

  friend std::ostream &operator<<(std::ostream &os, const PointXYZTIId &p);
};

struct EIGEN_ALIGN16 PointXYZITR
{
  PCL_ADD_POINT4D;
  float intensity;
  float time;
  uint16_t ring;

  static inline PointXYZITR make(float x, float y, float z, float intensity, float t, uint16_t ring)
  {
    return {x, y, z, 0.f, intensity, t, ring};
  }
};
// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZTIId,
                                 (float, x, x)
                                 (float, y, y)
                                 (float, z, z)
                                 (float, time, time)
                                 (uint8_t, intensity, intensity)
                                 (uint8_t, laserId, laserId))

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZITR,
                                  (float, x, x)
                                    (float, y, y)
                                    (float, z, z)
                                    (float, intensity, intensity)
                                    (float, time, time)
                                    (uint16_t, ring, ring))
// clang-format on

#endif // LIDARPOINT_H
