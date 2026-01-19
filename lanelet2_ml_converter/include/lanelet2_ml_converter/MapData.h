#pragma once
#include <lanelet2_core/Exceptions.h>
#include <lanelet2_core/Forward.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_routing/Forward.h>

#include <boost/geometry.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <type_traits>

#include "lanelet2_ml_converter/Forward.h"
#include "lanelet2_ml_converter/MapInstances.h"
#include "lanelet2_ml_converter/Types.h"
#include "lanelet2_routing/RoutingGraph.h"
#include "lanelet2_routing/internal/Graph.h"

namespace lanelet {
namespace ml_converter {

using MapDataPtr = std::shared_ptr<MapData>;

struct Edge {
  Edge() = default;
  Edge(Id el1, Id el2, bool isLaneChange) : el1_{el1}, el2_{el2}, isLaneChange_{isLaneChange} {}
  Id el1_;
  Id el2_;
  bool isLaneChange_;
};

namespace internal {
struct CompoundElsList {
  CompoundElsList(const Id& start, bool startInverted, LineStringType type)
      : ids{start}, inverted{startInverted}, type{type} {}
  CompoundElsList(const std::vector<Id>& ids, const std::vector<bool>& inverted, LineStringType type)
      : ids{ids}, inverted{inverted}, type{type} {}
  std::vector<Id> ids;
  std::vector<bool> inverted;
  LineStringType type;
};
}  // namespace internal

using Edges = std::map<Id, std::vector<Edge>>;  // key = id from
using TEToCenterlineEdges = std::vector<std::pair<TEInstancePtr, CompoundLaneLineStringInstancePtr>>;
using TEToTEEdges = std::vector<std::pair<TEInstancePtr, TEInstancePtr>>;

// Index-based edge types for TensorInstanceData
// TEToCenterline: (source_te_type, source_idx, target_centerline_idx)
// TEToTE: (source_type, source_idx, target_type, target_idx)
using TEToCenterlineIndexEdges = std::vector<std::tuple<TEType, size_t, size_t>>;
using TEToTEIndexEdges = std::vector<std::tuple<TEType, size_t, TEType, size_t>>;

template <typename T>
std::vector<std::shared_ptr<T>> getValidElements(const std::vector<std::shared_ptr<T>>& vec) {
  std::vector<std::shared_ptr<T>> res;
  std::copy_if(vec.begin(), vec.end(), std::back_inserter(res), [](std::shared_ptr<T> el) { return el->valid(); });
  return res;
}

template <typename T>
std::map<Id, std::shared_ptr<T>> getValidElements(const std::map<Id, std::shared_ptr<T>>& map) {
  std::map<Id, std::shared_ptr<T>> res;
  std::copy_if(map.begin(), map.end(), std::inserter(res, res.end()),
               [](const auto& pair) { return pair.second->valid(); });
  return res;
}

class MapData {
 public:
  struct TensorInstanceData {
   public:
    // Filter linestrings/compoundlinestrings by type
    std::vector<MatrixXd> lineStringsOfType(LineStringType type) const;
    std::vector<MatrixXd> compoundLineStringsOfType(LineStringType type) const;
    std::vector<MatrixXd> teInstancesOfType(TEType type) const;

    // Get associated compound linestring for a given type and index within that type
    CompoundLaneLineStringInstancePtr pointMatrixCpdLineStrings(LineStringType type, size_t index);

    // Get edge connections with type-local indices
    const TEToCenterlineIndexEdges& teToCenterlineIndexEdges() const { return teToCenterlineIndexEdges_; }
    const TEToTEIndexEdges& teToTEIndexEdges() const { return teToTEIndexEdges_; }

    const std::string& uuid() { return uuid_; }
    friend class MapData;

   private:
    // All linestrings organized by type for efficient O(1) lookup
    std::map<LineStringType, std::vector<MatrixXd>> lineStringsByType_;
    // All compound linestrings organized by type for efficient O(1) lookup
    std::map<LineStringType, std::vector<MatrixXd>> compoundLineStringsByType_;
    // Mapping from type to vector of compound feature instances (index is implicit in vector position)
    std::map<LineStringType, std::vector<CompoundLaneLineStringInstancePtr>> compoundLineStringInstancesByType_;
    // All traffic elements organized by type for efficient O(1) lookup
    std::map<TEType, std::vector<MatrixXd>> teInstancesByType_;
    
    // Edge connections using type-local indices
    TEToCenterlineIndexEdges teToCenterlineIndexEdges_;
    TEToTEIndexEdges teToTEIndexEdges_;

    std::string uuid_;
  };

  MapData() noexcept : uuid_{boost::lexical_cast<std::string>(boost::uuids::random_generator()())} {}
  static MapDataPtr build(LaneletSubmapConstPtr& localSubmap, lanelet::routing::RoutingGraphConstPtr localSubmapGraph,
                          traffic_rules::TrafficRulesPtr trafficRules, bool ignoreMapElevation = false,
                          const LineStringTypeGrouping& lineStringTypeGrouping = getDefaultLineStringTypeGrouping());
  bool processAll(const OrientedRect& bbox, const ParametrizationType& paramType, int32_t nPoints, double pitch = 0,
                  double roll = 0);

  LaneLineStringInstances lineStringsOfType(LineStringType type) const;

  LaneLineStringInstances validLineStringsOfType(LineStringType type) const;

  CompoundLaneLineStringInstanceList compoundLineStringsOfType(LineStringType type) const;

  CompoundLaneLineStringInstanceList validCompoundLineStringsOfType(LineStringType type) const;

  CompoundLaneLineStringInstanceList associatedCpdLineStringsOfType(Id mapId, LineStringType type) const;

  TEInstances teInstancesOfType(TEType type) const;

  TEInstances validTEInstancesOfType(TEType type) const;

  const LaneletInstances& laneletInstances() { return laneletInstances_; }
  const Edges& llEdges() { return llEdges_; }
  const Edges& teEdges() { return teEdges_; }
  const TEToCenterlineEdges& teToCenterlineEdges() const { return teToCenterlineEdges_; }
  const TEToTEEdges& teToTEEdges() const { return teToTEEdges_; }
  const std::string& uuid() { return uuid_; }

  /// The computed data will be buffered, if the underlying features change you need to set ignoreBuffer appropriately
  /// The data will also only be useful if you called process on the features beforehand (e.g. with processAll())
  TensorInstanceData getTensorInstanceData(bool pointsIn2d, bool ignoreBuffer);

  template <class Archive>
  friend void boost::serialization::serialize(Archive& ar, lanelet::ml_converter::MapData& feat,
                                              const unsigned int /*version*/);

 private:
  void initLeftBoundaries(LaneletSubmapConstPtr& localSubmap, lanelet::routing::RoutingGraphConstPtr localSubmapGraph,
                          traffic_rules::TrafficRulesPtr trafficRules, bool ignoreMapElevation = false);
  void initRightBoundaries(LaneletSubmapConstPtr& localSubmap, lanelet::routing::RoutingGraphConstPtr localSubmapGraph,
                           traffic_rules::TrafficRulesPtr trafficRules, bool ignoreMapElevation = false);
  void initLaneletInstances(LaneletSubmapConstPtr& localSubmap, lanelet::routing::RoutingGraphConstPtr localSubmapGraph,
                            traffic_rules::TrafficRulesPtr trafficRules, bool ignoreMapElevation = false);
  void initCompoundInstances(LaneletSubmapConstPtr& localSubmap,
                             lanelet::routing::RoutingGraphConstPtr localSubmapGraph,
                             traffic_rules::TrafficRulesPtr trafficRules, bool ignoreMapElevation = false);
  void updateAssociatedCpdInstanceIndices();
  void getPaths(lanelet::routing::RoutingGraphConstPtr localSubmapGraph, std::vector<ConstLanelets>& paths,
                ConstLanelet start, ConstLanelets initPath = ConstLanelets());

  LineStringType getLineStringTypeFromId(Id id);
  LaneLineStringInstancePtr getLineStringFeatFromId(Id id, bool inverted);
  std::vector<internal::CompoundElsList> computeCompoundLeftBorders(const ConstLanelets& path);
  std::vector<internal::CompoundElsList> computeCompoundRightBorders(const ConstLanelets& path);
  CompoundLaneLineStringInstancePtr computeCompoundCenterline(const ConstLanelets& path,
                                                              bool ignoreMapElevation = false);
  void computeDrivableAreaBorders(LaneletSubmapConstPtr& localSubmap);

  // Collect non-lane traffic elements
  void collectStopLines(LaneletSubmapConstPtr& localSubmap, bool ignoreMapElevation = false);
  void collectArrows(LaneletSubmapConstPtr& localSubmap, bool ignoreMapElevation = false);
  void collectTrafficLights(LaneletSubmapConstPtr& localSubmap, bool ignoreMapElevation = false);
  void collectTrafficSigns(LaneletSubmapConstPtr& localSubmap, bool ignoreMapElevation = false);
  void collectSymbols(LaneletSubmapConstPtr& localSubmap, bool ignoreMapElevation = false);
  
  // Convert teEdges_ to instance pointer associations
  void convertTEEdges();

  LaneLineStringInstances laneLineStrings_;  // all lane line strings (road borders, lane dividers, ...)

  CompoundLaneLineStringInstanceList
      compoundLaneLineStrings_;  // all compound line strings (road borders, lane dividers, centerlines, ...)

  TEInstances
      teInstances_;  // all traffic element instances (stop lines, road markings, traffic lights, traffic signs, ...)

  LaneletInstances laneletInstances_;

  // Maps LineStringType to a map of (mapId -> vector of compound feature indices)
  std::map<LineStringType, std::map<Id, std::vector<size_t>>> associatedCpdLineStringsIndices_;

  Edges llEdges_;     // edge list for lanelet/centerline connectivity
  Edges teEdges_;     // edge list for traffic element connectivity (e.g., traffic light to stop line)
  
  // Converted TE edge associations
  TEToCenterlineEdges teToCenterlineEdges_;
  TEToTEEdges teToTEEdges_;
  
  std::string uuid_;  // sample id

  Optional<TensorInstanceData> tfData_;
  LineStringTypeGrouping lineStringTypeGrouping_{getDefaultLineStringTypeGrouping()};
};

}  // namespace ml_converter
}  // namespace lanelet