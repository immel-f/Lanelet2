#pragma once
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LaneletOrArea.h>

#include <boost/geometry.hpp>
#include <functional>
#include <type_traits>

#include "lanelet2_ml_converter/Forward.h"

namespace lanelet {
namespace ml_converter {

/// @brief How a LaneletInstance is laid out in its instance vector
enum class LaneletRepresentationType {
  Centerline,  //!< centerline points, followed by the type of the left and the right boundary
  Boundaries   //!< left boundary points, right boundary points, followed by both types
};

/// @brief How the geometry of an instance is parametrized
/// @note Only LineString is implemented, the others throw when passed to process()
enum class ParametrizationType {
  Bezier,               //!< not implemented
  BezierEndpointFixed,  //!< not implemented
  LineString            //!< polyline, i.e. the points as they are
};

/**
 * @brief Type of a lane line string instance
 *
 * Most values are derived from the tagging of a lanelet boundary, see bdTypeToEnum(). The following ones are
 * not:
 * - Centerline and BikeCenterline: computed centerlines of lanelets and compound lanelet paths
 * - DrivableArea: the extent of the physically drivable space. Comes from line strings tagged
 *   `drivable_space_border`, which are not lanelet boundaries and are stated independently of how the lane
 *   boundaries are tagged, see MapData::computeDrivableAreaBorders()
 * - Divider: only ever appears as the representative type of a grouping that merges lane markings
 */
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
  Building,
  Wall,
  PedestrianCrossing,
  ZebraCrossing,
};

/**
 * @brief Type of a traffic element instance
 *
 * Prefixes: TL = traffic light, TS = traffic sign. The remaining values are road surface markings (arrows,
 * symbols) and stop lines. Traffic signs are mapped from their German StVO codes, see teTypeToEnum().
 */
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
  BusSymbol,
  Symbol30,
  Symbol50,
  Symbol70,
  StopLine,
  Unknown
};

/**
 * @brief The local reference frame: an axis aligned rectangle rotated by a yaw angle
 *
 * Everything outside of this rectangle is cut away when instances are processed, and the surviving geometry is
 * expressed relative to its center and yaw angle.
 *
 * @note Cannot be constructed directly, use getRotatedRect() to obtain one.
 */
struct OrientedRect {
  BasicPoint3d center;  //!< center of the rectangle in the map frame
  double yaw;           //!< heading of the rectangle in the map frame [rad]

  /// @brief Whether this rectangle was built from a 2d pose
  /// If true, the z coordinate of center is meaningless and all instance output is forced to be 2d, no matter
  /// what was passed as pointsIn2d.
  bool from2d{false};

  boost::geometry::model::polygon<BasicPoint2d> bg_poly;  //!< the rectangle in the map frame, used for clipping

  //! The corner points of the rectangle in the map frame
  boost::geometry::model::polygon<BasicPoint2d>::ring_type& bounds() { return bg_poly.outer(); }

  //! The corner points of the rectangle in the map frame
  const boost::geometry::model::polygon<BasicPoint2d>::ring_type& bounds_const() { return bg_poly.outer(); }

  friend OrientedRect getRotatedRect(const BasicPoint3d& center, double extentLongitudinal, double extentLateral,
                                     double yaw, bool from2dPos);

 private:
  OrientedRect() noexcept {}
};

/**
 * @brief Type grouping for compound instances: maps lists of LineStringTypes to a representative LineStringType
 *
 * Each pair contains a vector of LineStringTypes that map to a representative LineStringType. A grouping does
 * two things at once:
 * 1. it decides which boundaries are chained into one compound instance - consecutive boundaries are chained as
 *    long as they stay within the same group
 * 2. it decides the label of the result - a compound instance carries the representative type of its group
 *
 * This is how the label set is adapted to what a model is supposed to predict, e.g. getM3TRDefaultGrouping()
 * reproduces the label set of M3TR. Use getDefaultLineStringTypeGrouping() to keep every type separate.
 *
 * @note A grouping must assign a group to every type a lanelet boundary can have, otherwise unrelated types
 * would end up chained together, see checkLineStringTypeGroupingCoverage().
 */
using LineStringTypeGrouping = std::vector<std::pair<std::vector<LineStringType>, LineStringType>>;

/**
 * @brief Type grouping for traffic elements: maps lists of TETypes to a representative TEType
 *
 * Each pair contains a vector of TETypes that map to a representative TEType. Unlike a LineStringTypeGrouping
 * this only relabels: traffic elements are never chained, so a grouping merely decides which types collapse
 * into one label. Types that are in no group keep their own type.
 */
using TETypeGrouping = std::vector<std::pair<std::vector<TEType>, TEType>>;

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
      {{LineStringType::BikeCenterline}, LineStringType::BikeCenterline},
      {{LineStringType::Unknown}, LineStringType::Unknown},
      {{LineStringType::DrivableArea}, LineStringType::DrivableArea},
      {{LineStringType::Divider}, LineStringType::Divider},
      {{LineStringType::BikeMarkingDashed}, LineStringType::BikeMarkingDashed},
      {{LineStringType::BikeMarkingSolid}, LineStringType::BikeMarkingSolid},
      {{LineStringType::GuardRail}, LineStringType::GuardRail},
      {{LineStringType::PedestrianCrossing}, LineStringType::PedestrianCrossing},
      {{LineStringType::ZebraCrossing}, LineStringType::ZebraCrossing},
      {{LineStringType::Building}, LineStringType::Building},
      {{LineStringType::Wall}, LineStringType::Wall},
  };
}

/// @brief Get a RoadBorderMerged grouping: merges road border with fence, curbstone high and curbstone low
inline LineStringTypeGrouping getRoadBorderMergedGrouping() {
  return {
      {{LineStringType::RoadBorder, LineStringType::Fence, LineStringType::CurbstoneHigh, LineStringType::CurbstoneLow,
        LineStringType::GuardRail, LineStringType::Building, LineStringType::Wall},
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
        LineStringType::GuardRail, LineStringType::Building, LineStringType::Wall},
       LineStringType::RoadBorder},
      {{LineStringType::Dashed, LineStringType::Solid, LineStringType::SolidSolid, LineStringType::SolidDashed,
        LineStringType::DashedSolid, LineStringType::BikeMarkingDashed, LineStringType::BikeMarkingSolid},
       LineStringType::Divider},
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
        LineStringType::GuardRail, LineStringType::Building, LineStringType::Wall},
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

/// @brief Get the default TETypeGrouping where each type has its own group
inline TETypeGrouping getDefaultTETypeGrouping() {
  return {
      {{TEType::TLCar}, TEType::TLCar},
      {{TEType::TLBike}, TEType::TLBike},
      {{TEType::TLPedestrian}, TEType::TLPedestrian},
      {{TEType::TLMisc}, TEType::TLMisc},
      {{TEType::TSMisc}, TEType::TSMisc},
      {{TEType::TSNoEntry}, TEType::TSNoEntry},
      {{TEType::TSTurnRight}, TEType::TSTurnRight},
      {{TEType::TSTurnLeft}, TEType::TSTurnLeft},
      {{TEType::TSTurnLeftOrRight}, TEType::TSTurnLeftOrRight},
      {{TEType::TSGoStraight}, TEType::TSGoStraight},
      {{TEType::TSGoStraightOrRight}, TEType::TSGoStraightOrRight},
      {{TEType::TSGoStraightOrLeft}, TEType::TSGoStraightOrLeft},
      {{TEType::TSPassRight}, TEType::TSPassRight},
      {{TEType::TSPassLeft}, TEType::TSPassLeft},
      {{TEType::TSOneWayStreet}, TEType::TSOneWayStreet},
      {{TEType::TSYield}, TEType::TSYield},
      {{TEType::TSRightOfWay}, TEType::TSRightOfWay},
      {{TEType::TSPriorityRoad}, TEType::TSPriorityRoad},
      {{TEType::TSStop}, TEType::TSStop},
      {{TEType::TSCrossbuck}, TEType::TSCrossbuck},
      {{TEType::TSRoundabout}, TEType::TSRoundabout},
      {{TEType::TSSpeedLimit}, TEType::TSSpeedLimit},
      {{TEType::TSPedestrianCrossing}, TEType::TSPedestrianCrossing},
      {{TEType::ArrowTurnRight}, TEType::ArrowTurnRight},
      {{TEType::ArrowTurnLeft}, TEType::ArrowTurnLeft},
      {{TEType::ArrowTurnLeftOrRight}, TEType::ArrowTurnLeftOrRight},
      {{TEType::ArrowGoStraight}, TEType::ArrowGoStraight},
      {{TEType::ArrowGoStraightOrRight}, TEType::ArrowGoStraightOrRight},
      {{TEType::ArrowGoStraightOrLeft}, TEType::ArrowGoStraightOrLeft},
      {{TEType::BikeSymbol}, TEType::BikeSymbol},
      {{TEType::BusSymbol}, TEType::BusSymbol},
      {{TEType::Symbol30}, TEType::Symbol30},
      {{TEType::Symbol50}, TEType::Symbol50},
      {{TEType::Symbol70}, TEType::Symbol70},
      {{TEType::StopLine}, TEType::StopLine},
      {{TEType::Unknown}, TEType::Unknown},
  };
}

/// @brief Get the representative type for a TEType from the grouping
/// @return The representative TEType if found, the input type if not found
inline TEType getTETypeRepresentative(TEType type, const TETypeGrouping& grouping) {
  for (const auto& pair : grouping) {
    for (const auto& t : pair.first) {
      if (t == type) {
        return pair.second;
      }
    }
  }
  return type;
}

/// @brief Get the group index for a TEType
/// @return The group index if found, -1 if not found
inline int getTETypeGroupIndex(TEType type, const TETypeGrouping& grouping) {
  for (size_t i = 0; i < grouping.size(); ++i) {
    for (const auto& t : grouping[i].first) {
      if (t == type) {
        return static_cast<int>(i);
      }
    }
  }
  return -1;
}

/// @brief Check if two TE types are in the same group
inline bool areTETypesSameGroup(TEType type1, TEType type2, const TETypeGrouping& grouping) {
  return getTETypeGroupIndex(type1, grouping) == getTETypeGroupIndex(type2, grouping) &&
         getTETypeGroupIndex(type1, grouping) != -1;
}

}  // namespace ml_converter
}  // namespace lanelet
