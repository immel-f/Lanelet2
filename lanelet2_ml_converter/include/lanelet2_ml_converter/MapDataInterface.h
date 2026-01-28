#pragma once

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/utility/Optional.h>

#include "lanelet2_ml_converter/Forward.h"
#include "lanelet2_ml_converter/MapData.h"
#include "lanelet2_ml_converter/Types.h"
#include "lanelet2_routing/RoutingGraph.h"

namespace lanelet {
class LaneletLayer;

namespace ml_converter {

class MapDataInterface {
 public:
  struct Configuration {
    Configuration() noexcept : lineStringTypeGrouping{getDefaultLineStringTypeGrouping()},
                               teTypeGrouping{getDefaultTETypeGrouping()} {}
    Configuration(LaneletRepresentationType reprType, ParametrizationType paramType, double submapExtentLongitudinal,
                  double submapExtentLateral, int nPointsLanes, int nPointsTE) noexcept
        : reprType{reprType},
          paramType{paramType},
          submapExtentLongitudinal{submapExtentLongitudinal},
          submapExtentLateral{submapExtentLateral},
          nPointsLanes{nPointsLanes},
          nPointsTE{nPointsTE},
          lineStringTypeGrouping{getDefaultLineStringTypeGrouping()},
          teTypeGrouping{getDefaultTETypeGrouping()} {}
    LaneletRepresentationType reprType{LaneletRepresentationType::Boundaries};
    ParametrizationType paramType{ParametrizationType::LineString};
    double submapExtentLongitudinal{30};  // in driving direction
    double submapExtentLateral{15};       // in lateral direction
    bool ignoreMapElevation{false};       // if true, elevation (z coordinate) in map elements is ignored and set to 0
    bool resampleLanes{true};             // if true, lane instances are resampled; if false, no fixed resampling
    int nPointsLanes{
        20};  // number of points for lane resampling when resampleLanes=true, if set < 2 then no resampling as well
    bool resampleTE{true};  // if true, TE instances are resampled; if false, no fixed resampling
    int nPointsTE{
        20};  // number of points for TE resampling when resampleTE=true, if set < 2 then no resampling as well
    LineStringTypeGrouping lineStringTypeGrouping;  // grouping of types for compound instance generation
    TETypeGrouping teTypeGrouping;                  // grouping of types for traffic element instance generation
  };
  MapDataInterface(LaneletMapConstPtr laneletMap);
  MapDataInterface(LaneletMapConstPtr laneletMap, Configuration config);

  const Configuration& config() { return config_; }

  void setCurrPosAndExtractSubmap2d(const BasicPoint2d& pt, double yaw);
  void setCurrPosAndExtractSubmap(const BasicPoint3d& pt, double yaw);
  void setCurrPosAndExtractSubmap(const BasicPoint3d& pt, double yaw, double pitch, double roll);
  MapDataPtr mapData(bool processAll);

  std::vector<MapDataPtr> mapDataBatch2d(std::vector<BasicPoint2d> pts, std::vector<double> yaws);
  std::vector<MapDataPtr> mapDataBatch(std::vector<BasicPoint3d> pts, std::vector<double> yaws);
  std::vector<MapDataPtr> mapDataBatch(std::vector<BasicPoint3d> pts, std::vector<double> yaws,
                                       std::vector<double> pitches, std::vector<double> rolls);

 private:
  MapDataPtr getMapData(LaneletSubmapConstPtr localSubmap, const OrientedRect& bbox,
                        lanelet::routing::RoutingGraphConstPtr localSubmapGraph,
                        lanelet::routing::RoutingGraphConstPtr bikeSubmapGraph, double pitch, double roll,
                        bool processAll);

  LaneletMapConstPtr laneletMap_;
  LaneletSubmapConstPtr localSubmap_;
  std::unordered_map<Id, int> teId2Index_;
  routing::RoutingGraphConstPtr localSubmapGraph_;
  routing::RoutingGraphConstPtr bikeSubmapGraph_;
  Optional<BasicPoint3d> currPos_;  // in the map frame
  Optional<double> currRoll_;       // in the map frame, [rad]
  Optional<double> currPitch_;      // in the map frame, [rad]
  Optional<double> currYaw_;        // in the map frame, [rad]
  Optional<OrientedRect> currBbox_;
  Configuration config_;
  traffic_rules::TrafficRulesPtr trafficRules_;
  traffic_rules::TrafficRulesPtr bikeTrafficRules_;
};

}  // namespace ml_converter
}  // namespace lanelet