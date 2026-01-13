#pragma once
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LaneletOrArea.h>

#include <boost/geometry.hpp>
#include <functional>
#include <type_traits>

#include "lanelet2_ml_converter/Forward.h"

namespace lanelet {
namespace ml_converter {

enum class LaneletRepresentationType { Centerline, Boundaries };
enum class ParametrizationType { Bezier, BezierEndpointFixed, LineString };
enum class LineStringType {
  RoadBorder,
  Dashed,
  Solid,
  CurbstoneHigh,
  CurbstoneLow,
  Fence,
  SolidSolid,
  SolidDashed,
  DashedSolid,
  Virtual,
  Centerline,
  Unknown,
  DrivableArea,
};
enum class TEType { TrafficLight, TrafficSign, Unknown };

/// @brief Type grouping for compound instances: list of lists of LineStringTypes
/// Types in the same sublist will be grouped together for compound instance generation
using LineStringTypeGrouping = std::vector<std::vector<LineStringType>>;

/// @brief Get the default LineStringTypeGrouping where each type has its own group
inline LineStringTypeGrouping getDefaultLineStringTypeGrouping() {
  return {
      {LineStringType::RoadBorder},    {LineStringType::Dashed},       {LineStringType::Solid},
      {LineStringType::CurbstoneHigh}, {LineStringType::CurbstoneLow}, {LineStringType::Fence},
      {LineStringType::SolidSolid},    {LineStringType::SolidDashed},  {LineStringType::DashedSolid},
      {LineStringType::Virtual},       {LineStringType::Centerline},   {LineStringType::Unknown},
      {LineStringType::DrivableArea},
  };
}

/// @brief Get a RoadBorderMerged grouping: merges road border with fence, curbstone high and curbstone low
inline LineStringTypeGrouping getRoadBorderMergedGrouping() {
  return {
      {LineStringType::RoadBorder, LineStringType::Fence, LineStringType::CurbstoneHigh, LineStringType::CurbstoneLow},
      {LineStringType::Dashed},
      {LineStringType::Solid},
      {LineStringType::Unknown},
      {LineStringType::SolidSolid},
      {LineStringType::SolidDashed},
      {LineStringType::DashedSolid},
      {LineStringType::Virtual},
      {LineStringType::Centerline},
      {LineStringType::DrivableArea},
  };
}

/// @brief Get MapTR default simple grouping: RoadBorderMerged grouping with all lane dividers merged as well (including
/// dashed and solid, excluding Virtual)
inline LineStringTypeGrouping getMapTRDefaultSimpleGrouping() {
  return {
      {LineStringType::RoadBorder, LineStringType::Fence, LineStringType::CurbstoneHigh, LineStringType::CurbstoneLow},
      {LineStringType::Dashed, LineStringType::Solid, LineStringType::SolidSolid, LineStringType::SolidDashed,
       LineStringType::DashedSolid},
      {LineStringType::CurbstoneHigh},
      {LineStringType::CurbstoneLow},
      {LineStringType::Fence},
      {LineStringType::Virtual},
      {LineStringType::Centerline},
      {LineStringType::Unknown},
      {LineStringType::DrivableArea},
  };
}

/// @brief Get M3TR default grouping: RoadBorderMerged grouping, merges Solid, SolidSolid, SolidDashed, and DashedSolid
/// LineStringTypes
inline LineStringTypeGrouping getM3TRDefaultGrouping() {
  return {
      {LineStringType::RoadBorder, LineStringType::Fence, LineStringType::CurbstoneHigh, LineStringType::CurbstoneLow},
      {LineStringType::Dashed},
      {LineStringType::Solid, LineStringType::SolidSolid, LineStringType::SolidDashed, LineStringType::DashedSolid},
      {LineStringType::Virtual},
      {LineStringType::Centerline},
      {LineStringType::Unknown},
      {LineStringType::DrivableArea},
  };
}

/// @brief Get the group index for a LineStringType
/// @return The group index if found, -1 if not found
inline int getLineStringTypeGroupIndex(LineStringType type, const LineStringTypeGrouping& grouping) {
  for (size_t i = 0; i < grouping.size(); ++i) {
    for (const auto& t : grouping[i]) {
      if (t == type) {
        return static_cast<int>(i);
      }
    }
  }
  return -1;
}

/// @brief Check if two types are in the same group
inline bool areLineStringTypesSameGroup(LineStringType type1, LineStringType type2,
                                        const LineStringTypeGrouping& grouping) {
  return getLineStringTypeGroupIndex(type1, grouping) == getLineStringTypeGroupIndex(type2, grouping) &&
         getLineStringTypeGroupIndex(type1, grouping) != -1;
}

/// @brief Check if a type is conceptually a road border (default mapping)
inline bool isConceptualRoadBorder(LineStringType type) {
  return type == LineStringType::RoadBorder || type == LineStringType::Fence || type == LineStringType::CurbstoneHigh ||
         type == LineStringType::CurbstoneLow;
}

/// @brief Check if a type is conceptually a lane divider (default mapping)
inline bool isConceptualLaneDivider(LineStringType type) {
  return type == LineStringType::Dashed || type == LineStringType::Solid || type == LineStringType::SolidSolid ||
         type == LineStringType::SolidDashed || type == LineStringType::DashedSolid || type == LineStringType::Virtual;
}

/// @brief Check if a type is conceptually a centerline (default mapping)
inline bool isConceptualCenterline(LineStringType type) { return type == LineStringType::Centerline; }

/// @brief Check if a type is conceptually a drivable area border (default mapping)
inline bool isConceptualDrivableAreaBorder(LineStringType type) { return type == LineStringType::DrivableArea; }

struct OrientedRect {
  BasicPoint3d center;
  double yaw;
  bool from2d{false};
  boost::geometry::model::polygon<BasicPoint2d> bg_poly;
  boost::geometry::model::polygon<BasicPoint2d>::ring_type& bounds() { return bg_poly.outer(); }
  const boost::geometry::model::polygon<BasicPoint2d>::ring_type& bounds_const() { return bg_poly.outer(); }

  friend OrientedRect getRotatedRect(const BasicPoint3d& center, double extentLongitudinal, double extentLateral,
                                     double yaw, bool from2dPos);

 private:
  OrientedRect() noexcept {}
};

}  // namespace ml_converter
}  // namespace lanelet
