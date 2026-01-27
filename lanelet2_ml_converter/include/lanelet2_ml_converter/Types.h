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
  BikeCenterline,
  Unknown,
  DrivableArea,
  Divider,
  BikeMarkingDashed,
  BikeMarkingSolid,
  GuardRail,
  PedestrianCrossing,
  ZebraCrossing,
};

// TL = Traffic Light, TS = Traffic Sign
enum class TEType {
  TLCar,
  TLBike,
  TLPedestrian,
  TLMisc,
  TSMisc,
  TSNoEntry,
  TSTurnRight,
  TSTurnLeft,
  TSTurnLeftOrRight,
  TSGoStraight,
  TSGoStraightOrRight,
  TSGoStraightOrLeft,
  TSPassRight,
  TSPassLeft,
  TSOneWayStreet,
  TSYield,
  TSRightOfWay,
  TSPriorityRoad,
  TSStop,
  TSCrossbuck,
  TSRoundabout,
  TSSpeedLimit,
  TSPedestrianCrossing,
  ArrowTurnRight,
  ArrowTurnLeft,
  ArrowTurnLeftOrRight,
  ArrowGoStraight,
  ArrowGoStraightOrRight,
  ArrowGoStraightOrLeft,
  BikeSymbol,
  Symbol30,
  Symbol50,
  Symbol70,
  StopLine,
  Unknown
};

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

/// @brief Type grouping for compound instances: maps lists of LineStringTypes to a representative LineStringType
/// Each pair contains a vector of LineStringTypes that map to a representative LineStringType
using LineStringTypeGrouping = std::vector<std::pair<std::vector<LineStringType>, LineStringType>>;

/// @brief Get the default LineStringTypeGrouping where each type has its own group
inline LineStringTypeGrouping getDefaultLineStringTypeGrouping() {
  return {
      {{LineStringType::RoadBorder}, LineStringType::RoadBorder},
      {{LineStringType::Dashed}, LineStringType::Dashed},
      {{LineStringType::Solid}, LineStringType::Solid},
      {{LineStringType::CurbstoneHigh}, LineStringType::CurbstoneHigh},
      {{LineStringType::CurbstoneLow}, LineStringType::CurbstoneLow},
      {{LineStringType::Fence}, LineStringType::Fence},
      {{LineStringType::SolidSolid}, LineStringType::SolidSolid},
      {{LineStringType::SolidDashed}, LineStringType::SolidDashed},
      {{LineStringType::DashedSolid}, LineStringType::DashedSolid},
      {{LineStringType::Virtual}, LineStringType::Virtual},
      {{LineStringType::Centerline}, LineStringType::Centerline},
      {{LineStringType::Unknown}, LineStringType::Unknown},
      {{LineStringType::DrivableArea}, LineStringType::DrivableArea},
      {{LineStringType::BikeMarkingDashed}, LineStringType::BikeMarkingDashed},
      {{LineStringType::BikeMarkingSolid}, LineStringType::BikeMarkingSolid},
      {{LineStringType::GuardRail}, LineStringType::GuardRail},
      {{LineStringType::PedestrianCrossing}, LineStringType::PedestrianCrossing},
      {{LineStringType::ZebraCrossing}, LineStringType::ZebraCrossing},
  };
}

/// @brief Get a RoadBorderMerged grouping: merges road border with fence, curbstone high and curbstone low
inline LineStringTypeGrouping getRoadBorderMergedGrouping() {
  return {
      {{LineStringType::RoadBorder, LineStringType::Fence, LineStringType::CurbstoneHigh, LineStringType::CurbstoneLow,
        LineStringType::GuardRail},
       LineStringType::RoadBorder},
      {{LineStringType::Dashed}, LineStringType::Dashed},
      {{LineStringType::Solid}, LineStringType::Solid},
      {{LineStringType::Unknown}, LineStringType::Unknown},
      {{LineStringType::SolidSolid}, LineStringType::SolidSolid},
      {{LineStringType::SolidDashed}, LineStringType::SolidDashed},
      {{LineStringType::DashedSolid}, LineStringType::DashedSolid},
      {{LineStringType::Virtual}, LineStringType::Virtual},
      {{LineStringType::Centerline}, LineStringType::Centerline},
      {{LineStringType::BikeCenterline}, LineStringType::BikeCenterline},
      {{LineStringType::DrivableArea}, LineStringType::DrivableArea},
      {{LineStringType::BikeMarkingDashed}, LineStringType::BikeMarkingDashed},
      {{LineStringType::BikeMarkingSolid}, LineStringType::BikeMarkingSolid},
      {{LineStringType::PedestrianCrossing}, LineStringType::PedestrianCrossing},
      {{LineStringType::ZebraCrossing}, LineStringType::ZebraCrossing},
  };
}

/// @brief Get MapTR default simple grouping: RoadBorderMerged grouping with all lane dividers merged as well (including
/// dashed and solid, excluding Virtual)
inline LineStringTypeGrouping getMapTRDefaultSimpleGrouping() {
  return {
      {{LineStringType::RoadBorder, LineStringType::Fence, LineStringType::CurbstoneHigh, LineStringType::CurbstoneLow,
        LineStringType::GuardRail},
       LineStringType::RoadBorder},
      {{LineStringType::Dashed, LineStringType::Solid, LineStringType::SolidSolid, LineStringType::SolidDashed,
        LineStringType::DashedSolid, LineStringType::BikeMarkingDashed, LineStringType::BikeMarkingSolid},
       LineStringType::Divider},
      {{LineStringType::CurbstoneHigh}, LineStringType::CurbstoneHigh},
      {{LineStringType::CurbstoneLow}, LineStringType::CurbstoneLow},
      {{LineStringType::Fence}, LineStringType::Fence},
      {{LineStringType::Virtual}, LineStringType::Virtual},
      {{LineStringType::Centerline}, LineStringType::Centerline},
      {{LineStringType::BikeCenterline}, LineStringType::BikeCenterline},
      {{LineStringType::Unknown}, LineStringType::Unknown},
      {{LineStringType::DrivableArea}, LineStringType::DrivableArea},
      {{LineStringType::PedestrianCrossing, LineStringType::ZebraCrossing}, LineStringType::PedestrianCrossing},
  };
}

/// @brief Get M3TR default grouping: RoadBorderMerged grouping, merges Solid, SolidSolid, SolidDashed, and DashedSolid
/// LineStringTypes
inline LineStringTypeGrouping getM3TRDefaultGrouping() {
  return {
      {{LineStringType::RoadBorder, LineStringType::Fence, LineStringType::CurbstoneHigh, LineStringType::CurbstoneLow,
        LineStringType::GuardRail},
       LineStringType::RoadBorder},
      {{LineStringType::Dashed, LineStringType::BikeMarkingDashed}, LineStringType::Dashed},
      {{LineStringType::Solid, LineStringType::SolidSolid, LineStringType::SolidDashed, LineStringType::DashedSolid,
        LineStringType::BikeMarkingSolid},
       LineStringType::Solid},
      {{LineStringType::Virtual}, LineStringType::Virtual},
      {{LineStringType::Centerline}, LineStringType::Centerline},
      {{LineStringType::BikeCenterline}, LineStringType::BikeCenterline},
      {{LineStringType::Unknown}, LineStringType::Unknown},
      {{LineStringType::DrivableArea}, LineStringType::DrivableArea},
      {{LineStringType::BikeMarkingDashed}, LineStringType::BikeMarkingDashed},
      {{LineStringType::BikeMarkingSolid}, LineStringType::BikeMarkingSolid},
      {{LineStringType::PedestrianCrossing}, LineStringType::PedestrianCrossing},
      {{LineStringType::PedestrianCrossing, LineStringType::ZebraCrossing}, LineStringType::PedestrianCrossing},
  };
}

/// @brief Get the representative type for a LineStringType from the grouping
/// @return The representative LineStringType if found, the input type if not found
inline LineStringType getLineStringTypeRepresentative(LineStringType type, const LineStringTypeGrouping& grouping) {
  for (const auto& pair : grouping) {
    for (const auto& t : pair.first) {
      if (t == type) {
        return pair.second;
      }
    }
  }
  return type;
}

/// @brief Get the group index for a LineStringType
/// @return The group index if found, -1 if not found
inline int getLineStringTypeGroupIndex(LineStringType type, const LineStringTypeGrouping& grouping) {
  for (size_t i = 0; i < grouping.size(); ++i) {
    for (const auto& t : grouping[i].first) {
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

}  // namespace ml_converter
}  // namespace lanelet
