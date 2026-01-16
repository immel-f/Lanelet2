#pragma once
#include <lanelet2_core/Exceptions.h>
#include <lanelet2_core/Forward.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <boost/geometry.hpp>

#include "Forward.h"
#include "Serialize.h"
#include "Types.h"
namespace lanelet {
namespace ml_converter {

OrientedRect getRotatedRect(const BasicPoint3d& center, double extentLongitudinal, double extentLateral, double yaw,
                            bool from2dPos);

LaneletSubmapConstPtr extractSubmap(LaneletMapConstPtr laneletMap, const BasicPoint2d& center,
                                    double extentLongitudinal, double extentLateral);

inline std::string lineStringTypeToString(LineStringType type) {
  if (type == LineStringType::RoadBorder)
    return "RoadBorder";
  else if (type == LineStringType::DrivableArea)
    return "DrivableArea";
  else if (type == LineStringType::Dashed)
    return "Dashed";
  else if (type == LineStringType::Solid)
    return "Solid";
  else if (type == LineStringType::CurbstoneHigh)
    return "CurbstoneHigh";
  else if (type == LineStringType::CurbstoneLow)
    return "CurbstoneLow";
  else if (type == LineStringType::Fence)
    return "Fence";
  else if (type == LineStringType::SolidSolid)
    return "SolidSolid";
  else if (type == LineStringType::SolidDashed)
    return "SolidDashed";
  else if (type == LineStringType::DashedSolid)
    return "DashedSolid";
  else if (type == LineStringType::Virtual)
    return "Virtual";
  else if (type == LineStringType::Centerline)
    return "Centerline";
  else if (type == LineStringType::Unknown)
    return "Unknown";
  else {
    throw std::runtime_error("Unexpected Line String type!");
    return "Unknown";
  }
}

inline LineStringType bdTypeToEnum(ConstLineString3d lString) {
  Attribute type = lString.attributeOr(AttributeName::Type, "");
  Attribute subtype = lString.attributeOr(AttributeName::Subtype, "");

  // Check for specific types first
  if (type == AttributeValueString::RoadBorder) {
    return LineStringType::RoadBorder;
  } else if (type == AttributeValueString::Curbstone) {
    // For Curbstone, check the subtype
    if (subtype == AttributeValueString::Low) {
      return LineStringType::CurbstoneLow;
    }
    // Default to CurbstoneHigh if no subtype or subtype is High
    return LineStringType::CurbstoneHigh;
  } else if (type == AttributeValueString::Fence) {
    return LineStringType::Fence;
  } else if (type == AttributeValueString::Virtual) {
    return LineStringType::Virtual;
  }

  // Check subtype for lane marking types
  if (subtype == AttributeValueString::Dashed) {
    return LineStringType::Dashed;
  } else if (subtype == AttributeValueString::Solid) {
    return LineStringType::Solid;
  } else if (subtype == AttributeValueString::SolidSolid) {
    return LineStringType::SolidSolid;
  } else if (subtype == AttributeValueString::SolidDashed) {
    return LineStringType::SolidDashed;
  } else if (subtype == AttributeValueString::DashedSolid) {
    return LineStringType::DashedSolid;
  }

  // Default to Unknown if no match found
  return LineStringType::Unknown;
}

inline TEType teTypeToEnum(const ConstLineString3d& te) {
  Attribute type = te.attributeOr(AttributeName::Type, "");
  Attribute subtype = te.attributeOr(AttributeName::Subtype, "");

  // Handle stop_line type
  if (type == "stop_line") {
    return TEType::StopLine;
  }

  // Handle arrows
  if (type == "arrow") {
    if (subtype == "right") {
      return TEType::ArrowTurnRight;
    } else if (subtype == "left") {
      return TEType::ArrowTurnLeft;
    } else if (subtype == "straight") {
      return TEType::ArrowGoStraight;
    } else if (subtype == "straight_right") {
      return TEType::ArrowGoStraightOrRight;
    } else if (subtype == "straight_left") {
      return TEType::ArrowGoStraightOrLeft;
    } else if (subtype == "left_right") {
      return TEType::ArrowTurnLeftOrRight;
    }
    return TEType::Unknown;  // Unknown arrow subtype
  }

  // Handle symbols
  if (type == "symbol") {
    if (subtype == "bicycle") {
      return TEType::BikeSymbol;
    }
    return TEType::Unknown;  // Unknown symbol subtype
  }

  // Handle traffic lights
  if (type == AttributeValueString::TrafficLight) {
    if (subtype == "red_yellow_green") {
      return TEType::TLCar;
    } else if (subtype == "bike") {
      return TEType::TLBike;
    } else if (subtype == "pedestrian" || type == "traffic_light_pedestrians") {
      return TEType::TLPedestrian;
    }
    return TEType::TLMisc;  // Default for unknown traffic light subtypes
  }

  // Handle traffic light pedestrians as separate type
  if (type == "traffic_light_pedestrians") {
    return TEType::TLPedestrian;
  }

  // Handle traffic signs
  if (type == AttributeValueString::TrafficSign) {
    // German traffic sign codes based on official StVO regulation
    // Regulatory signs (200-299)
    if (subtype == "de205") {
      return TEType::TSYield;  // Yield sign
    } else if (subtype == "de206") {
      return TEType::TSStop;  // Stop sign
    } else if (subtype == "de209") {
      return TEType::TSTurnRight;  // Turn right ahead
    } else if (subtype == "de209-10") {
      return TEType::TSTurnLeft;  // Turn left ahead
    } else if (subtype == "de209-30") {
      return TEType::TSGoStraight;  // Go straight ahead
    } else if (subtype == "de211") {
      return TEType::TSTurnRight;  // Turn right (here)
    } else if (subtype == "de211-10") {
      return TEType::TSTurnLeft;  // Turn left (here)
    } else if (subtype == "de214") {
      return TEType::TSGoStraightOrRight;  // Go straight or turn right ahead
    } else if (subtype == "de214-10") {
      return TEType::TSGoStraightOrLeft;  // Go straight or turn left ahead
    } else if (subtype == "de214-30") {
      return TEType::TSTurnLeftOrRight;  // Turn left or right ahead
    } else if (subtype == "de215") {
      return TEType::TSRoundabout;  // Roundabout
    } else if (subtype == "de220-10" || subtype == "de220-20") {
      return TEType::TSOneWayStreet;  // One-way street
    } else if (subtype == "de222") {
      return TEType::TSPassRight;  // Pass on the right
    } else if (subtype == "de222-10") {
      return TEType::TSPassLeft;  // Pass on the left
    } else if (subtype == "de267") {
      return TEType::TSNoEntry;                       // No entry
    } else if (subtype.value().find("de274") == 0) {  // de274-* (speed limit signs)
      return TEType::TSSpeedLimit;
      // Directional signs (300-399)
    } else if (subtype == "de301") {
      return TEType::TSRightOfWay;  // Intersection ahead - right-of-way only for this intersection
    } else if (subtype == "de306") {
      return TEType::TSPriorityRoad;                  // Priority road - right-of-way on all following intersections
    } else if (subtype.value().find("de310") == 0) {  // de310 (town/city limit sign)
      return TEType::TSMisc;
    } else if (subtype == "de350-10" || subtype == "de350-20") {
      return TEType::TSPedestrianCrossing;  // Pedestrian crossing directional signs
    }
    return TEType::TSMisc;  // Default for unknown traffic sign subtypes
  }

  // Default to Unknown if no match found
  return TEType::Unknown;
}

BasicLineString3d resampleLineString(const BasicLineString3d& polyline, int32_t nPoints);

std::vector<BasicLineString3d> cutLineString(const OrientedRect& bbox, const BasicLineString3d& polyline);

BasicLineString3d transformLineString(const OrientedRect& bbox, const BasicLineString3d& polyline, double pitch,
                                      double roll);

void saveMapData(const std::string& filename, const std::vector<MapDataPtr>& mDataVec,
                 bool binary);  // saves all in one file

std::vector<MapDataPtr> loadMapData(const std::string& filename, bool binary);  // loads entire vector from one file

void saveMapDataMultiFile(const std::string& path, const std::vector<std::string>& filenames,
                          const std::vector<MapDataPtr>& mDataVec,
                          bool binary);  // saves in one file per LaneData

std::vector<MapDataPtr> loadMapDataMultiFile(const std::string& path, const std::vector<std::string>& filenames,
                                             bool binary);  // loads from one file per LaneData

}  // namespace ml_converter
}  // namespace lanelet