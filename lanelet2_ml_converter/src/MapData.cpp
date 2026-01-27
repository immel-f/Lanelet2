#include "lanelet2_ml_converter/MapData.h"

#include <iostream>

#include "lanelet2_core/Attribute.h"
#include "lanelet2_core/geometry/LineString.h"
#include "lanelet2_core/geometry/Polygon.h"
#include "lanelet2_ml_converter/Utils.h"

namespace lanelet {
namespace ml_converter {

using namespace internal;

MapDataPtr MapData::build(LaneletSubmapConstPtr& localSubmap, lanelet::routing::RoutingGraphConstPtr localSubmapGraph,
                          traffic_rules::TrafficRulesPtr trafficRules,
                          lanelet::routing::RoutingGraphConstPtr bikeSubmapGraph, bool ignoreMapElevation,
                          const LineStringTypeGrouping& lineStringTypeGrouping) {
  MapDataPtr data = std::make_shared<MapData>();
  data->lineStringTypeGrouping_ = lineStringTypeGrouping;
  data->initLeftBoundaries(localSubmap, localSubmapGraph, trafficRules, ignoreMapElevation);
  data->initRightBoundaries(localSubmap, localSubmapGraph, trafficRules, ignoreMapElevation);
  data->initLaneletInstances(localSubmap, localSubmapGraph, trafficRules, ignoreMapElevation);
  data->initCompoundInstances(localSubmap, localSubmapGraph, trafficRules, bikeSubmapGraph, ignoreMapElevation);
  data->updateAssociatedCpdInstanceIndices();

  // Collect non-lane traffic elements
  data->collectStopLines(localSubmap, ignoreMapElevation);
  data->collectArrows(localSubmap, ignoreMapElevation);
  data->collectTrafficLights(localSubmap, ignoreMapElevation);
  data->collectTrafficSigns(localSubmap, ignoreMapElevation);
  data->collectSymbols(localSubmap, ignoreMapElevation);

  // Convert TE edges to instance associations
  data->convertTEEdges();

  return data;
}

void MapData::initLeftBoundaries(LaneletSubmapConstPtr& localSubmap,
                                 lanelet::routing::RoutingGraphConstPtr localSubmapGraph,
                                 traffic_rules::TrafficRulesPtr trafficRules, bool ignoreMapElevation) {
  for (const auto& ll : localSubmap->laneletLayer) {
    Id boundID = ll.leftBound3d().id();
    BasicLineString3d bound =
        ll.leftBound3d().inverted() ? ll.leftBound3d().invert().basicLineString() : ll.leftBound3d().basicLineString();
    if (ignoreMapElevation) {
      for (auto& pt : bound) {
        pt[2] = 0;
      }
    }
    LaneLineStringInstances::iterator itLineString = laneLineStrings_.find(boundID);
    if (itLineString != laneLineStrings_.end()) {
      itLineString->second->addLaneletID(ll.id());
    } else {
      laneLineStrings_.insert({boundID, std::make_shared<LaneLineStringInstance>(
                                            bound, boundID, bdTypeToEnum(ll.leftBound3d()), Ids{ll.id()}, false)});
    }

    Optional<ConstLanelet> leftLL = localSubmapGraph->left(ll);
    Optional<ConstLanelet> adjLeftLL = localSubmapGraph->adjacentLeft(ll);

    if (leftLL) {
      llEdges_[ll.id()].push_back(Edge(ll.id(), leftLL->id(), true));
    }
  }
}

void MapData::initRightBoundaries(LaneletSubmapConstPtr& localSubmap,
                                  lanelet::routing::RoutingGraphConstPtr localSubmapGraph,
                                  traffic_rules::TrafficRulesPtr trafficRules, bool ignoreMapElevation) {
  for (const auto& ll : localSubmap->laneletLayer) {
    Id boundID = ll.rightBound3d().id();
    BasicLineString3d bound = ll.rightBound3d().inverted() ? ll.rightBound3d().invert().basicLineString()
                                                           : ll.rightBound3d().basicLineString();
    if (ignoreMapElevation) {
      for (auto& pt : bound) {
        pt[2] = 0;
      }
    }
    LaneLineStringInstances::iterator itLineString = laneLineStrings_.find(boundID);
    if (itLineString != laneLineStrings_.end()) {
      itLineString->second->addLaneletID(ll.id());
    } else {
      laneLineStrings_.insert({boundID, std::make_shared<LaneLineStringInstance>(
                                            bound, boundID, bdTypeToEnum(ll.rightBound3d()), Ids{ll.id()}, false)});
    }
    Optional<ConstLanelet> rightLL = localSubmapGraph->right(ll);
    Optional<ConstLanelet> adjRightLL = localSubmapGraph->adjacentRight(ll);
    if (rightLL) {
      llEdges_[ll.id()].push_back(Edge(ll.id(), rightLL->id(), true));
    }
  }
}

void MapData::initLaneletInstances(LaneletSubmapConstPtr& localSubmap,
                                   lanelet::routing::RoutingGraphConstPtr localSubmapGraph,
                                   traffic_rules::TrafficRulesPtr trafficRules, bool ignoreMapElevation) {
  for (const auto& ll : localSubmap->laneletLayer) {
    LaneLineStringInstancePtr leftBoundary = getLineStringFeatFromId(ll.leftBound().id(), ll.leftBound().inverted());
    LaneLineStringInstancePtr rightBoundary = getLineStringFeatFromId(ll.rightBound().id(), ll.leftBound().inverted());
    BasicLineString3d centerlineLString = ll.centerline3d().basicLineString();
    if (ignoreMapElevation) {
      for (auto& pt : centerlineLString) {
        pt[2] = 0;
      }
    }

    // Determine if this is a bike lane based on subtype attribute
    Attribute subtype = ll.attributeOr(AttributeName::Subtype, "");
    LineStringType centerlineType =
        (subtype == AttributeValueString::BicycleLane) ? LineStringType::BikeCenterline : LineStringType::Centerline;

    LaneLineStringInstancePtr centerline = std::make_shared<LaneLineStringInstance>(
        centerlineLString, ll.centerline3d().id(), centerlineType, Ids{ll.id()}, ll.centerline3d().inverted());
    laneletInstances_.insert(
        {ll.id(), std::make_shared<LaneletInstance>(leftBoundary, rightBoundary, centerline, ll.id())});
  }
}

bool isLaneletInPath(const ConstLanelets& path, const ConstLanelet& ll) {
  for (const auto& el : path) {
    if (ll.id() == el.id()) {
      return true;
    }
  }
  return false;
}

// Idea: algorithm for paths that starts with LLs with no previous, splits on junctions and terminates on LLs with
// no successors
void MapData::getPaths(lanelet::routing::RoutingGraphConstPtr localSubmapGraph, std::vector<ConstLanelets>& paths,
                       ConstLanelet start, ConstLanelets initPath) {
  initPath.push_back(start);
  ConstLanelet current = start;

  // Get the subtype of the starting lanelet to maintain consistency along the path
  Attribute startSubtype = start.attributeOr(AttributeName::Subtype, "");

  ConstLanelets successorLLs = localSubmapGraph->following(current, false);

  // Filter successors to only include those with matching subtype
  ConstLanelets filteredSuccessors;
  for (const auto& successor : successorLLs) {
    Attribute successorSubtype = successor.attributeOr(AttributeName::Subtype, "");
    if (successorSubtype == startSubtype) {
      filteredSuccessors.push_back(successor);
    }
  }
  successorLLs = filteredSuccessors;

  while (!successorLLs.empty()) {
    for (size_t i = 1; i != successorLLs.size(); i++) {
      if (isLaneletInPath(initPath, successorLLs[i])) {
        continue;
      }
      llEdges_[current.id()].push_back(Edge(current.id(), successorLLs[i].id(), false));
      getPaths(localSubmapGraph, paths, successorLLs[i], initPath);
    }
    if (isLaneletInPath(initPath, successorLLs.front())) {
      break;
    }
    initPath.push_back(successorLLs.front());
    llEdges_[current.id()].push_back(Edge(current.id(), successorLLs.front().id(), false));
    current = successorLLs.front();
    successorLLs = localSubmapGraph->following(current, false);

    // Filter successors again for the new current lanelet
    filteredSuccessors.clear();
    for (const auto& successor : successorLLs) {
      Attribute successorSubtype = successor.attributeOr(AttributeName::Subtype, "");
      if (successorSubtype == startSubtype) {
        filteredSuccessors.push_back(successor);
      }
    }
    successorLLs = filteredSuccessors;
  }
  paths.push_back(initPath);
}

LineStringType MapData::getLineStringTypeFromId(Id id) {
  LaneLineStringInstances::iterator it = laneLineStrings_.find(id);
  if (it != laneLineStrings_.end()) {
    return it->second->type();
  } else {
    throw std::runtime_error("Lanelet boundary " + std::to_string(id) + " is not in the instance list!");
  }
}

LaneLineStringInstancePtr makeInverted(const LaneLineStringInstancePtr& feat) {
  return std::make_shared<LaneLineStringInstance>(
      BasicLineString3d(feat->rawInstance().rbegin(), feat->rawInstance().rend()), feat->mapID(), feat->type(),
      feat->laneletIDs(), !feat->inverted());
}

LaneLineStringInstancePtr MapData::getLineStringFeatFromId(Id id, bool inverted) {
  LaneLineStringInstances::iterator it = laneLineStrings_.find(id);
  if (it != laneLineStrings_.end()) {
    return (inverted == it->second->inverted()) ? it->second : makeInverted(it->second);
  } else {
    throw std::runtime_error("Lanelet boundary " + std::to_string(id) + " is not in the instance list!");
  }
}

std::vector<CompoundElsList> MapData::computeCompoundLeftBorders(const ConstLanelets& path) {
  std::vector<CompoundElsList> compoundBorders;
  ConstLanelet start = path.front();
  LineStringType currType = getLineStringTypeFromId(start.leftBound3d().id());
  LineStringType currRepType = getLineStringTypeRepresentative(currType, lineStringTypeGrouping_);
  int currGroupIdx = getLineStringTypeGroupIndex(currType, lineStringTypeGrouping_);

  compoundBorders.push_back(CompoundElsList{start.leftBound3d().id(), start.leftBound3d().inverted(), currRepType});

  for (size_t i = 1; i != path.size(); i++) {
    LineStringType newType = getLineStringTypeFromId(path[i].leftBound3d().id());
    LineStringType newRepType = getLineStringTypeRepresentative(newType, lineStringTypeGrouping_);
    int newGroupIdx = getLineStringTypeGroupIndex(newType, lineStringTypeGrouping_);
    if (currGroupIdx == newGroupIdx) {
      compoundBorders.back().ids.push_back(path[i].leftBound3d().id());
      compoundBorders.back().inverted.push_back(path[i].leftBound3d().inverted());
    } else {
      compoundBorders.push_back(
          CompoundElsList{path[i].leftBound3d().id(), path[i].leftBound3d().inverted(), newRepType});
      currType = newType;
      currRepType = newRepType;
      currGroupIdx = newGroupIdx;
    }
  }
  return compoundBorders;
}

std::vector<CompoundElsList> MapData::computeCompoundRightBorders(const ConstLanelets& path) {
  std::vector<CompoundElsList> compoundBorders;
  ConstLanelet start = path.front();
  LineStringType currType = getLineStringTypeFromId(start.rightBound3d().id());
  LineStringType currRepType = getLineStringTypeRepresentative(currType, lineStringTypeGrouping_);
  int currGroupIdx = getLineStringTypeGroupIndex(currType, lineStringTypeGrouping_);

  compoundBorders.push_back(CompoundElsList{start.rightBound3d().id(), start.rightBound3d().inverted(), currRepType});

  for (size_t i = 1; i != path.size(); i++) {
    LineStringType newType = getLineStringTypeFromId(path[i].rightBound3d().id());
    LineStringType newRepType = getLineStringTypeRepresentative(newType, lineStringTypeGrouping_);
    int newGroupIdx = getLineStringTypeGroupIndex(newType, lineStringTypeGrouping_);
    if (currGroupIdx == newGroupIdx) {
      compoundBorders.back().ids.push_back(path[i].rightBound3d().id());
      compoundBorders.back().inverted.push_back(path[i].rightBound3d().inverted());
    } else {
      compoundBorders.push_back(
          CompoundElsList{path[i].rightBound3d().id(), path[i].rightBound3d().inverted(), newRepType});
      currType = newType;
      currRepType = newRepType;
      currGroupIdx = newGroupIdx;
    }
  }
  return compoundBorders;
}

CompoundLaneLineStringInstancePtr MapData::computeCompoundCenterline(const ConstLanelets& path,
                                                                     bool ignoreMapElevation) {
  LaneLineStringInstanceList compoundCenterlines;

  // Determine centerline type from first lanelet in path
  Attribute subtype = path.front().attributeOr(AttributeName::Subtype, "");
  LineStringType centerlineType =
      (subtype == AttributeValueString::BicycleLane) ? LineStringType::BikeCenterline : LineStringType::Centerline;

  for (const auto& ll : path) {
    BasicLineString3d centerlineLString = ll.centerline3d().basicLineString();
    if (ignoreMapElevation) {
      for (auto& pt : centerlineLString) {
        pt[2] = 0;
      }
    }
    compoundCenterlines.push_back(std::make_shared<LaneLineStringInstance>(centerlineLString, ll.id(), centerlineType,
                                                                           Ids{ll.id()}, ll.centerline3d().inverted()));
  }
  return std::make_shared<CompoundLaneLineStringInstance>(compoundCenterlines, centerlineType);
}

void MapData::computeDrivableAreaBorders(LaneletSubmapConstPtr& localSubmap) {
  // Collect all LineStrings from the map with drivable_space_border attribute
  std::vector<Id> drivableAreaIds;
  std::map<Id, std::pair<Id, Id>> endpointIdMap;  // Maps linestring ID to (first_point_id, last_point_id)

  for (const auto& lineString : localSubmap->lineStringLayer) {
    Attribute drivableAreaBorder = lineString.attributeOr("drivable_space_border", "");
    if (drivableAreaBorder == "true") {
      Id lsId = lineString.id();
      BasicLineString3d lsBasic = lineString.basicLineString();

      // Add to laneLineStrings_ if not already present
      LaneLineStringInstances::iterator it = laneLineStrings_.find(lsId);
      if (it == laneLineStrings_.end()) {
        laneLineStrings_.insert({lsId, std::make_shared<LaneLineStringInstance>(
                                           lsBasic, lsId, LineStringType::DrivableArea, Ids{}, false)});
      }

      // Store the point IDs of the endpoints for connectivity checking
      if (lineString.size() >= 2) {
        drivableAreaIds.push_back(lsId);
        endpointIdMap[lsId] = {lineString.front().id(), lineString.back().id()};
      }
    }
  }

  // Build adjacency list for connected components based on point IDs
  std::map<Id, std::vector<Id>> adjacency;

  for (size_t i = 0; i < drivableAreaIds.size(); ++i) {
    for (size_t j = i + 1; j < drivableAreaIds.size(); ++j) {
      Id id1 = drivableAreaIds[i];
      Id id2 = drivableAreaIds[j];

      const auto& endpoints1 = endpointIdMap[id1];
      const auto& endpoints2 = endpointIdMap[id2];

      // Check if they share an endpoint (based on point IDs)
      bool connected = endpoints1.first == endpoints2.first || endpoints1.first == endpoints2.second ||
                       endpoints1.second == endpoints2.first || endpoints1.second == endpoints2.second;

      if (connected) {
        adjacency[id1].push_back(id2);
        adjacency[id2].push_back(id1);
      }
    }
  }

  // Find connected components using DFS
  std::set<Id> visited;
  for (const auto& id : drivableAreaIds) {
    if (visited.find(id) != visited.end()) {
      continue;
    }

    // DFS to find component
    std::vector<Id> component;
    std::vector<Id> stack;
    stack.push_back(id);

    while (!stack.empty()) {
      Id current = stack.back();
      stack.pop_back();

      if (visited.find(current) != visited.end()) {
        continue;
      }

      visited.insert(current);
      component.push_back(current);

      for (const auto& neighbor : adjacency[current]) {
        if (visited.find(neighbor) == visited.end()) {
          stack.push_back(neighbor);
        }
      }
    }

    // Handle linestrings in component - build chains starting from nodes with dangling endpoints
    std::set<size_t> usedIndices;

    while (usedIndices.size() < component.size()) {
      // Find the first unused linestring, preferring nodes with dangling endpoints
      size_t startIdx = component.size();
      bool startFirstEndpointDangling = false;
      bool startSecondEndpointDangling = false;

      // First, try to find an unused node where either endpoint is not connected to neighbors
      for (size_t i = 0; i < component.size(); ++i) {
        if (usedIndices.count(i)) continue;

        Id nodeId = component[i];
        const auto& nodeEndpoints = endpointIdMap[nodeId];

        // Check if either endpoint is dangling (not matching any neighbor's endpoints)
        bool firstEndpointDangling = true;
        bool secondEndpointDangling = true;

        for (const auto& neighborId : adjacency[nodeId]) {
          const auto& neighborEndpoints = endpointIdMap[neighborId];

          if (nodeEndpoints.first == neighborEndpoints.first || nodeEndpoints.first == neighborEndpoints.second) {
            firstEndpointDangling = false;
          }
          if (nodeEndpoints.second == neighborEndpoints.first || nodeEndpoints.second == neighborEndpoints.second) {
            secondEndpointDangling = false;
          }
        }

        if (firstEndpointDangling || secondEndpointDangling) {
          startIdx = i;
          startFirstEndpointDangling = firstEndpointDangling;
          startSecondEndpointDangling = secondEndpointDangling;
          break;
        }
      }

      // If no dangling endpoint found, use the first unused node
      if (startIdx >= component.size()) {
        for (size_t i = 0; i < component.size(); ++i) {
          if (!usedIndices.count(i)) {
            startIdx = i;
            break;
          }
        }
      }

      if (startIdx >= component.size()) {
        break;  // All linestrings have been used
      }

      // Start a new chain with this linestring
      LaneLineStringInstanceList compoundFeatures;
      usedIndices.insert(startIdx);

      // Determine which endpoint to start from based on dangling status
      Id startNodeId = component[startIdx];
      const auto& startEndpoints = endpointIdMap[startNodeId];
      Id currentEndpoint = startEndpoints.second;  // Default to second endpoint

      // If first endpoint is dangling, start from the second one; otherwise start from the first
      if (startFirstEndpointDangling && !startSecondEndpointDangling) {
        currentEndpoint = startEndpoints.second;
        compoundFeatures.push_back(getLineStringFeatFromId(component[startIdx], false));
      } else {
        currentEndpoint = startEndpoints.first;
        compoundFeatures.push_back(makeInverted(getLineStringFeatFromId(component[startIdx], false)));
      }

      // Build the continuous chain
      bool foundConnection = true;
      while (foundConnection && usedIndices.size() < component.size()) {
        foundConnection = false;

        for (size_t i = 0; i < component.size(); ++i) {
          if (usedIndices.count(i)) continue;

          const auto& endpoints = endpointIdMap[component[i]];

          // Check if this linestring connects to current endpoint
          if (endpoints.first == currentEndpoint) {
            // Connects normally, no inversion needed
            compoundFeatures.push_back(getLineStringFeatFromId(component[i], false));
            currentEndpoint = endpoints.second;
            usedIndices.insert(i);
            foundConnection = true;
            break;
          } else if (endpoints.second == currentEndpoint) {
            // Connects with inversion - manually invert the linestring
            compoundFeatures.push_back(makeInverted(getLineStringFeatFromId(component[i], false)));
            currentEndpoint = endpoints.first;
            usedIndices.insert(i);
            foundConnection = true;
            break;
          }
        }
      }

      // Create a compound instance for this chain
      if (!compoundFeatures.empty()) {
        compoundLaneLineStrings_.push_back(
            std::make_shared<CompoundLaneLineStringInstance>(compoundFeatures, LineStringType::DrivableArea));
      }
    }
  }
}

Optional<Id> findNearestIntersectingLanelet(const ConstLineString3d& lineString, LaneletSubmapConstPtr& localSubmap,
                                            bool requireIntersection = true, double maxDistance = 1.0) {
  // Convert to 2D hybrid linestring
  ConstHybridLineString2d ls2d = utils::to2D(utils::toHybrid(lineString));

  // Calculate centroid for nearest lanelet query
  BasicPoint2d centroid(0.0, 0.0);
  for (const auto& pt : lineString) {
    centroid.x() += pt.x();
    centroid.y() += pt.y();
  }
  centroid.x() /= lineString.size();
  centroid.y() /= lineString.size();

  // Get only the nearest lanelet to the centroid
  ConstLanelets nearestLanelets = localSubmap->laneletLayer.nearest(centroid, 1);

  if (!nearestLanelets.empty()) {
    const auto& ll = nearestLanelets.front();

    // Get lanelet's 2D polygon using built-in function
    CompoundHybridPolygon2d laneletPolygon = utils::to2D(utils::toHybrid(ll.polygon3d()));

    if (requireIntersection) {
      // Check if linestring intersects with lanelet polygon
      if (geometry::intersects(ls2d, laneletPolygon)) {
        return ll.id();
      }
    } else {
      // Check if distance to lanelet centerline is below threshold
      ConstHybridLineString2d centerline2d = utils::to2D(utils::toHybrid(ll.centerline()));
      double distance = geometry::distance(ls2d, centerline2d);
      if (distance <= maxDistance) {
        return ll.id();
      }
    }
  }
  return {};
}

void MapData::collectStopLines(LaneletSubmapConstPtr& localSubmap, bool ignoreMapElevation) {
  for (const auto& lineString : localSubmap->lineStringLayer) {
    Attribute type = lineString.attributeOr(AttributeName::Type, "");
    if (type == "stop_line") {
      Id lsId = lineString.id();
      BasicLineString3d lsBasic = lineString.basicLineString();

      if (ignoreMapElevation) {
        for (auto& pt : lsBasic) {
          pt[2] = 0;
        }
      }

      TEType teType = teTypeToEnum(lineString);
      teInstances_.insert({lsId, std::make_shared<TEInstance>(lsBasic, lsId, teType)});

      // Find associated lanelets via regulatory elements
      auto regElemsOwningLs = localSubmap->regulatoryElementLayer.findUsages(lineString);
      for (const auto& regElem : regElemsOwningLs) {
        // Find lanelets that reference this regulatory element
        auto laneletsOwningRegelem = localSubmap->laneletLayer.findUsages(regElem);
        for (const auto& lanelet : laneletsOwningRegelem) {
          teEdges_[lsId].push_back(Edge(lsId, lanelet.id(), false));
        }
      }
    }
  }
}

void MapData::collectArrows(LaneletSubmapConstPtr& localSubmap, bool ignoreMapElevation) {
  for (const auto& lineString : localSubmap->lineStringLayer) {
    Attribute type = lineString.attributeOr(AttributeName::Type, "");
    if (type == "arrow") {
      Id lsId = lineString.id();
      BasicLineString3d lsBasic = lineString.basicLineString();

      if (ignoreMapElevation) {
        for (auto& pt : lsBasic) {
          pt[2] = 0;
        }
      }

      TEType teType = teTypeToEnum(lineString);
      teInstances_.insert({lsId, std::make_shared<TEInstance>(lsBasic, lsId, teType)});

      // Find the nearest intersecting lanelet
      Optional<Id> laneletId = findNearestIntersectingLanelet(lineString, localSubmap);
      if (laneletId) {
        teEdges_[lsId].push_back(Edge(lsId, *laneletId, false));
      }
    }
  }
}

void MapData::collectTrafficLights(LaneletSubmapConstPtr& localSubmap, bool ignoreMapElevation) {
  for (const auto& lineString : localSubmap->lineStringLayer) {
    Attribute type = lineString.attributeOr(AttributeName::Type, "");
    if (type == AttributeValueString::TrafficLight || type == "traffic_light_pedestrians") {
      Id lsId = lineString.id();
      BasicLineString3d lsBasic = lineString.basicLineString();

      if (ignoreMapElevation) {
        for (auto& pt : lsBasic) {
          pt[2] = 0;
        }
      }

      TEType teType = teTypeToEnum(lineString);
      teInstances_.insert({lsId, std::make_shared<TEInstance>(lsBasic, lsId, teType)});

      // Find associated stop line via regulatory elements
      auto regElemsOwningLs = localSubmap->regulatoryElementLayer.findUsages(lineString);
      for (const auto& regElem : regElemsOwningLs) {
        // Check if this is a TrafficLight regulatory element
        if (regElem->attribute(AttributeName::Subtype).value() == "traffic_light") {
          // Get the stop line from the regulatory element using template syntax
          auto stopLines = regElem->getParameters<ConstLineString3d>(RoleName::RefLine);
          if (!stopLines.empty()) {
            Id stopLineId = stopLines.front().id();
            // Check if this stop line is in our teInstances_ (already collected)
            if (teInstances_.find(stopLineId) != teInstances_.end()) {
              teEdges_[lsId].push_back(Edge(lsId, stopLineId, false));
            }
          }
        }
      }
    }
  }
}

void MapData::collectTrafficSigns(LaneletSubmapConstPtr& localSubmap, bool ignoreMapElevation) {
  for (const auto& lineString : localSubmap->lineStringLayer) {
    Attribute type = lineString.attributeOr(AttributeName::Type, "");
    if (type == AttributeValueString::TrafficSign) {
      Id lsId = lineString.id();
      BasicLineString3d lsBasic = lineString.basicLineString();

      if (ignoreMapElevation) {
        for (auto& pt : lsBasic) {
          pt[2] = 0;
        }
      }

      TEType teType = teTypeToEnum(lineString);
      teInstances_.insert({lsId, std::make_shared<TEInstance>(lsBasic, lsId, teType)});
    }
  }
}

void MapData::collectSymbols(LaneletSubmapConstPtr& localSubmap, bool ignoreMapElevation) {
  for (const auto& lineString : localSubmap->lineStringLayer) {
    Attribute type = lineString.attributeOr(AttributeName::Type, "");
    if (type == "symbol") {
      Id lsId = lineString.id();
      BasicLineString3d lsBasic = lineString.basicLineString();

      if (ignoreMapElevation) {
        for (auto& pt : lsBasic) {
          pt[2] = 0;
        }
      }

      TEType teType = teTypeToEnum(lineString);
      teInstances_.insert({lsId, std::make_shared<TEInstance>(lsBasic, lsId, teType)});

      // Find the nearest intersecting lanelet
      Optional<Id> laneletId = findNearestIntersectingLanelet(lineString, localSubmap);
      if (laneletId) {
        teEdges_[lsId].push_back(Edge(lsId, *laneletId, false));
      }
    }
  }
}

void MapData::convertTEEdges() {
  teToCenterlineEdges_.clear();
  teToTEEdges_.clear();

  for (const auto& edgeEntry : teEdges_) {
    Id sourceId = edgeEntry.first;

    // Find source TE instance
    auto sourceIt = teInstances_.find(sourceId);
    if (sourceIt == teInstances_.end()) {
      continue;  // Skip if source TE not found
    }
    TEInstancePtr sourceTEPtr = sourceIt->second;

    for (const auto& edge : edgeEntry.second) {
      Id targetId = edge.el2_;

      // Check if target is a TE instance (TE to TE edge)
      auto targetTEIt = teInstances_.find(targetId);
      if (targetTEIt != teInstances_.end()) {
        teToTEEdges_.push_back({sourceTEPtr, targetTEIt->second});
        continue;
      }

      // Otherwise, target should be a lanelet - find all compound centerlines containing it
      CompoundLaneLineStringInstanceList centerlines = compoundLineStringsOfType(LineStringType::Centerline);
      for (const auto& cpdLineString : centerlines) {
        // Check if this compound centerline contains the target lanelet
        bool containsTargetLanelet = false;
        for (const auto& feature : cpdLineString->features()) {
          for (const auto& laneletId : feature->laneletIDs()) {
            if (laneletId == targetId) {
              containsTargetLanelet = true;
              break;
            }
          }
          if (containsTargetLanelet) break;
        }

        if (containsTargetLanelet) {
          teToCenterlineEdges_.push_back({sourceTEPtr, cpdLineString});
        }
      }
    }
  }
}

std::map<Id, size_t>::const_iterator findFirstOccElement(const CompoundElsList& elsList,
                                                         const std::map<Id, size_t>& searchMap) {
  for (const auto& el : elsList.ids) {
    std::map<Id, size_t>::const_iterator it = searchMap.find(el);
    if (it != searchMap.end()) {
      return it;
    }
  }
  return searchMap.end();
}

bool hasElementNotInOther(const CompoundElsList& elsList1, const CompoundElsList& elsList2) {
  for (const auto& el : elsList1.ids) {
    if (std::find(elsList2.ids.begin(), elsList2.ids.end(), el) == elsList2.ids.end()) {
      return true;
    }
  }
  return false;
}

void insertAndCheckNewCompoundInstances(std::vector<CompoundElsList>& compFeats,
                                        const std::vector<CompoundElsList>& newCompFeats,
                                        std::map<Id, size_t>& elInsertIdx) {
  for (const auto& compEl : newCompFeats) {
    std::map<Id, size_t>::const_iterator firstOccIt = findFirstOccElement(compEl, elInsertIdx);
    if (firstOccIt == elInsertIdx.end()) {
      compFeats.push_back(compEl);
      for (const Id& el : compEl.ids) {
        elInsertIdx[el] = compFeats.size() - 1;
      }
    } else if ((compFeats[firstOccIt->second].ids.size() < compEl.ids.size()) &&
               compFeats[firstOccIt->second].type == compEl.type &&
               !hasElementNotInOther(compFeats[firstOccIt->second], compEl)) {
      compFeats[firstOccIt->second] = compEl;
      for (const Id& el : compEl.ids) {
        elInsertIdx[el] = firstOccIt->second;
      }
    } else if (compFeats[firstOccIt->second].type == compEl.type) {
      std::vector<Id> leftoverIds;
      std::vector<bool> leftoverInverted;
      bool lastLeftover{false};
      for (size_t i = 0; i < compEl.ids.size(); i++) {
        if (!elInsertIdx.count(compEl.ids[i])) {
          leftoverIds.push_back(compEl.ids[i]);
          leftoverInverted.push_back(compEl.inverted[i]);
          lastLeftover = true;
        } else if (lastLeftover) {
          CompoundElsList leftover(leftoverIds, leftoverInverted, compEl.type);
          compFeats.push_back(leftover);
          for (const Id& el : leftover.ids) {
            elInsertIdx[el] = compFeats.size() - 1;
          }
          leftoverIds.clear();
          leftoverInverted.clear();
          lastLeftover = false;
        }
      }
      if (lastLeftover) {
        CompoundElsList leftover(leftoverIds, leftoverInverted, compEl.type);
        compFeats.push_back(leftover);
        for (const Id& el : leftover.ids) {
          elInsertIdx[el] = compFeats.size() - 1;
        }
        leftoverIds.clear();
        leftoverInverted.clear();
        lastLeftover = false;
      }
    }
  }
}

void MapData::initCompoundInstances(LaneletSubmapConstPtr& localSubmap,
                                    lanelet::routing::RoutingGraphConstPtr localSubmapGraph,
                                    traffic_rules::TrafficRulesPtr trafficRules,
                                    lanelet::routing::RoutingGraphConstPtr bikeSubmapGraph, bool ignoreMapElevation) {
  std::vector<CompoundElsList> compoundedBordersAndDividers;
  std::map<Id, size_t> elInsertIdx;

  // Process vehicle routing graph
  std::vector<ConstLanelets> vehiclePaths;
  for (const auto& ll : localSubmap->laneletLayer) {
    if (!trafficRules->canPass(ll)) {
      continue;
    }

    ConstLanelets previousLLs = localSubmapGraph->previous(ll, false);
    if (previousLLs.empty()) {
      getPaths(localSubmapGraph, vehiclePaths, ll);
    }
  }

  // Process bicycle routing graph if provided
  std::vector<ConstLanelets> bikePaths;
  if (bikeSubmapGraph) {
    for (const auto& ll : localSubmap->laneletLayer) {
      // Check if this is a bike lane
      Attribute subtype = ll.attributeOr(AttributeName::Subtype, "");
      if (subtype != AttributeValueString::BicycleLane) {
        continue;
      }

      ConstLanelets previousLLs = bikeSubmapGraph->previous(ll, false);
      if (previousLLs.empty()) {
        getPaths(bikeSubmapGraph, bikePaths, ll);
      }
    }
  }

  // Combine all paths for border processing to avoid duplicating shared dividers
  std::vector<ConstLanelets> allPaths;
  allPaths.insert(allPaths.end(), vehiclePaths.begin(), vehiclePaths.end());
  allPaths.insert(allPaths.end(), bikePaths.begin(), bikePaths.end());

  // Process borders from all paths together
  for (const auto& path : allPaths) {
    std::vector<CompoundElsList> compoundedLeft = computeCompoundLeftBorders(path);
    insertAndCheckNewCompoundInstances(compoundedBordersAndDividers, compoundedLeft, elInsertIdx);
    std::vector<CompoundElsList> compoundedRight = computeCompoundRightBorders(path);
    insertAndCheckNewCompoundInstances(compoundedBordersAndDividers, compoundedRight, elInsertIdx);
    compoundLaneLineStrings_.push_back(computeCompoundCenterline(path, ignoreMapElevation));
  }
  for (const auto& compFeat : compoundedBordersAndDividers) {
    LaneLineStringInstanceList toBeCompounded;
    if (compFeat.ids.size() != compFeat.inverted.size()) {
      throw std::runtime_error("Unequal sizes of ids and inverted!");
    }
    for (size_t i = 0; i < compFeat.ids.size(); i++) {
      LaneLineStringInstancePtr cmpdFeat = getLineStringFeatFromId(compFeat.ids[i], compFeat.inverted[i]);
      toBeCompounded.push_back(cmpdFeat);
    }
    LineStringType cmpdType = toBeCompounded.front()->type();
    compoundLaneLineStrings_.push_back(std::make_shared<CompoundLaneLineStringInstance>(toBeCompounded, cmpdType));
  }

  // Process drivable area borders
  computeDrivableAreaBorders(localSubmap);
}

void MapData::updateAssociatedCpdInstanceIndices() {
  for (size_t i = 0; i < compoundLaneLineStrings_.size(); i++) {
    const auto& cpdFeat = compoundLaneLineStrings_[i];
    LineStringType type = cpdFeat->type();
    for (const auto& indFeat : cpdFeat->features()) {
      for (const auto& id : indFeat->laneletIDs()) {
        if (type != LineStringType::Centerline) {
          associatedCpdLineStringsIndices_[type][id].push_back(i);
        }
      }
      associatedCpdLineStringsIndices_[type][indFeat->mapID()].push_back(i);
    }
  }
}

bool MapData::processAll(const OrientedRect& bbox, const ParametrizationType& paramType, int32_t nPoints, double pitch,
                         double roll) {
  bool validLineStrings = processInstances(laneLineStrings_, bbox, paramType, nPoints, pitch, roll);
  bool validLaneletInstances = processInstances(laneletInstances_, bbox, paramType, nPoints, pitch, roll);
  bool validCompoundLineStrings = processInstances(compoundLaneLineStrings_, bbox, paramType, nPoints, pitch, roll);
  bool validTEInstances = processInstances(teInstances_, bbox, paramType, nPoints, pitch, roll);

  if (validLineStrings && validLaneletInstances && validCompoundLineStrings && validTEInstances) {
    return true;
  } else {
    return false;
  }
}

MapData::TensorInstanceData MapData::getTensorInstanceData(bool pointsIn2d, bool ignoreBuffer) {
  if (!tfData_.has_value() || ignoreBuffer) {
    tfData_ = MapData::TensorInstanceData();
    tfData_->uuid_ = uuid_;

    // Collect all valid linestrings organized by type
    size_t globalLineStringIdx = 0;
    for (const auto& type_pair : laneLineStrings_) {
      if (type_pair.second->valid()) {
        std::vector<MatrixXd> matrices = type_pair.second->pointMatrices(pointsIn2d);
        LineStringType type = type_pair.second->type();
        for (const auto& mat : matrices) {
          tfData_->lineStringsByType_[type].push_back(mat);
          globalLineStringIdx++;
        }
      }
    }

    // Collect all valid compound linestrings organized by type
    // Build mapping from instance pointer to (type, index)
    std::map<CompoundLaneLineStringInstancePtr, std::pair<LineStringType, size_t>> cpdInstanceToIndex;
    for (size_t i = 0; i < compoundLaneLineStrings_.size(); ++i) {
      const auto& cpdFeat = compoundLaneLineStrings_[i];
      if (cpdFeat->valid()) {
        std::vector<MatrixXd> matrices = cpdFeat->pointMatrices(pointsIn2d);
        LineStringType type = cpdFeat->type();
        size_t typeIndex = tfData_->compoundLineStringsByType_[type].size();
        for (const auto& mat : matrices) {
          tfData_->compoundLineStringsByType_[type].push_back(mat);
          tfData_->compoundLineStringInstancesByType_[type].push_back(cpdFeat);
        }
        cpdInstanceToIndex[cpdFeat] = {type, typeIndex};
      }
    }

    // Collect all valid traffic elements organized by type
    // Build mapping from instance pointer to (type, index)
    std::map<TEInstancePtr, std::pair<TEType, size_t>> teInstanceToIndex;
    for (const auto& te_pair : teInstances_) {
      if (te_pair.second->valid()) {
        std::vector<MatrixXd> matrices = te_pair.second->pointMatrices(pointsIn2d);
        TEType type = te_pair.second->teType();
        size_t typeIndex = tfData_->teInstancesByType_[type].size();
        for (const auto& mat : matrices) {
          tfData_->teInstancesByType_[type].push_back(mat);
        }
        teInstanceToIndex[te_pair.second] = {type, typeIndex};
      }
    }

    // Convert teToCenterlineEdges_ to index-based edges
    for (const auto& edge : teToCenterlineEdges_) {
      auto teIt = teInstanceToIndex.find(edge.first);
      auto cpdIt = cpdInstanceToIndex.find(edge.second);
      if (teIt != teInstanceToIndex.end() && cpdIt != cpdInstanceToIndex.end()) {
        if (cpdIt->second.first != LineStringType::Centerline) {
          std::cerr << "Warning: TE to centerline edge contains non-centerline compound linestring (type: "
                    << static_cast<int>(cpdIt->second.first) << ")" << std::endl;
        } else {
          tfData_->teToCenterlineIndexEdges_.push_back(
              std::make_tuple(teIt->second.first, teIt->second.second, cpdIt->second.second));
        }
      }
    }

    // Convert teToTEEdges_ to index-based edges
    for (const auto& edge : teToTEEdges_) {
      auto teSourceIt = teInstanceToIndex.find(edge.first);
      auto teTargetIt = teInstanceToIndex.find(edge.second);
      if (teSourceIt != teInstanceToIndex.end() && teTargetIt != teInstanceToIndex.end()) {
        tfData_->teToTEIndexEdges_.push_back(std::make_tuple(teSourceIt->second.first, teSourceIt->second.second,
                                                             teTargetIt->second.first, teTargetIt->second.second));
      }
    }
  }
  return tfData_.value();
}

std::vector<MatrixXd> MapData::TensorInstanceData::lineStringsOfType(LineStringType type) const {
  auto it = lineStringsByType_.find(type);
  if (it != lineStringsByType_.end()) {
    return it->second;
  }
  return std::vector<MatrixXd>();
}

std::vector<MatrixXd> MapData::TensorInstanceData::compoundLineStringsOfType(LineStringType type) const {
  auto it = compoundLineStringsByType_.find(type);
  if (it != compoundLineStringsByType_.end()) {
    return it->second;
  }
  return std::vector<MatrixXd>();
}

std::vector<MatrixXd> MapData::TensorInstanceData::teInstancesOfType(TEType type) const {
  auto it = teInstancesByType_.find(type);
  if (it != teInstancesByType_.end()) {
    return it->second;
  }
  return std::vector<MatrixXd>();
}

CompoundLaneLineStringInstancePtr MapData::TensorInstanceData::pointMatrixCpdLineStrings(LineStringType type,
                                                                                         size_t index) {
  auto typeIt = compoundLineStringInstancesByType_.find(type);
  if (typeIt == compoundLineStringInstancesByType_.end()) {
    throw std::out_of_range("No compound line strings exist for the given type!");
  }
  if (index >= typeIt->second.size()) {
    throw std::out_of_range("A point matrix with type-local index " + std::to_string(index) + " does not exist!");
  }
  return typeIt->second[index];
}

CompoundLaneLineStringInstanceList MapData::associatedCpdLineStringsOfType(Id mapId, LineStringType type) const {
  CompoundLaneLineStringInstanceList assoFeats;
  try {
    const auto& typeIndices = associatedCpdLineStringsIndices_.at(type);
    for (const auto& idx : typeIndices.at(mapId)) {
      assoFeats.push_back(compoundLaneLineStrings_[idx]);
    }
  } catch (const std::out_of_range&) {
    return assoFeats;
  }
  return assoFeats;
}

LaneLineStringInstances MapData::lineStringsOfType(LineStringType type) const {
  LaneLineStringInstances result;
  for (const auto& pair : laneLineStrings_) {
    if (pair.second->type() == type) {
      result.insert(pair);
    }
  }
  return result;
}

CompoundLaneLineStringInstanceList MapData::compoundLineStringsOfType(LineStringType type) const {
  CompoundLaneLineStringInstanceList result;
  for (const auto& feat : compoundLaneLineStrings_) {
    if (feat->type() == type) {
      result.push_back(feat);
    }
  }
  return result;
}

LaneLineStringInstances MapData::validLineStringsOfType(LineStringType type) const {
  return getValidElements(lineStringsOfType(type));
}

CompoundLaneLineStringInstanceList MapData::validCompoundLineStringsOfType(LineStringType type) const {
  return getValidElements(compoundLineStringsOfType(type));
}

TEInstances MapData::teInstancesOfType(TEType type) const {
  TEInstances result;
  for (const auto& pair : teInstances_) {
    if (pair.second->teType() == type) {
      result.insert(pair);
    }
  }
  return result;
}

TEInstances MapData::validTEInstancesOfType(TEType type) const { return getValidElements(teInstancesOfType(type)); }

}  // namespace ml_converter
}  // namespace lanelet