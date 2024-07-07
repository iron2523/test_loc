#ifdef HAS_CPUID
#include <cpuid.h>
#endif

#include <sys/time.h>

#include <ios>
#include <iostream>
#include <sstream>
#include <string>

template <typename T>
std::string to_string_with_precision(const T a_value, const int n = 6)
{
    std::ostringstream out;
    out.precision(n);
    out << std::fixed << a_value;
    return out.str();
}

//BOOST
#include <boost/format.hpp>

//PCL
#define PCL_NO_PRECOMPILE

//DLIO
#include <nano_gicp/nano_gicp.h>

namespace dlio {
  enum class SensorType { OUSTER, VELODYNE, HESAI, LIVOX, UNKNOWN };

  class OdomNode;
  class MapNode;

  struct Point {
    Point(): data{0.f, 0.f, 0.f, 1.f} {}

    PCL_ADD_POINT4D;
    float intensity; // intensity
    union {
    std::uint32_t t;   // (Ouster) time since beginning of scan in nanoseconds
    float time;        // (Velodyne) time since beginning of scan in seconds
    double timestamp;  // (Hesai) absolute timestamp in seconds
                       // (Livox) absolute timestamp in (seconds * 10e9)
    };
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  } EIGEN_ALIGN16;
}

POINT_CLOUD_REGISTER_POINT_STRUCT(dlio::Point,
                                 (float, x, x)
                                 (float, y, y)
                                 (float, z, z)
                                 (float, intensity, intensity)
                                 (std::uint32_t, t, t)
                                 (float, time, time)
                                 (double, timestamp, timestamp))

typedef dlio::Point PointType;

// 增加
// struct LiovxPointCustomMsg
// {
//     PCL_ADD_POINT4D
//     PCL_ADD_INTENSITY;
//     float time;
//     uint16_t ring;
//     uint16_t tag;
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// } EIGEN_ALIGN16;
// POINT_CLOUD_REGISTER_POINT_STRUCT (LiovxPointCustomMsg,
//     (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity) (float, time, time)
//     (uint16_t, ring, ring) (uint16_t, tag, tag)
// )
// using PointXYZIRT = LiovxPointCustomMsg;