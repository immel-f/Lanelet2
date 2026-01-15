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
  if (type == AttributeValueString::TrafficLight) {
    return TEType::TLMisc;
  } else if (type == AttributeValueString::TrafficSign) {
    return TEType::TSMisc;
  } else {
    // throw std::runtime_error("Unexpected Traffic Element Type!");
    return TEType::Unknown;
  }
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