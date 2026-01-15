#include "lanelet2_ml_converter/MapData.h"

#include "lanelet2_ml_converter/Utils.h"

namespace lanelet {
namespace ml_converter {

using namespace internal;

MapDataPtr MapData::build(LaneletSubmapConstPtr& localSubmap, lanelet::routing::RoutingGraphConstPtr localSubmapGraph,
                            traffic_rules::TrafficRulesPtr trafficRules, bool ignoreMapElevation,
                            const LineStringTypeGrouping& lineStringTypeGrouping) {
  MapDataPtr data = std::make_shared<MapData>();
  data->lineStringTypeGrouping_ = lineStringTypeGrouping;
  data->initLeftBoundaries(localSubmap, localSubmapGraph, trafficRules, ignoreMapElevation);
  data->initRightBoundaries(localSubmap, localSubmapGraph, trafficRules, ignoreMapElevation);
  data->initLaneletInstances(localSubmap, localSubmapGraph, trafficRules, ignoreMapElevation);
  data->initCompoundInstances(localSubmap, localSubmapGraph, trafficRules, ignoreMapElevation);
  data->updateAssociatedCpdInstanceIndices();
  return data;
}

void MapData::initLeftBoundaries(LaneletSubmapConstPtr& localSubmap,
                                  lanelet::routing::RoutingGraphConstPtr localSubmapGraph,
                                  traffic_rules::TrafficRulesPtr trafficRules, bool ignoreMapElevation) {
  for (const auto& ll : localSubmap->laneletLayer) {
    if (!trafficRules->canPass(ll)) {
      continue;
    }

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
      edges_[ll.id()].push_back(Edge(ll.id(), leftLL->id(), true));
    }
  }
}

void MapData::initRightBoundaries(LaneletSubmapConstPtr& localSubmap,
                                   lanelet::routing::RoutingGraphConstPtr localSubmapGraph,
                                   traffic_rules::TrafficRulesPtr trafficRules, bool ignoreMapElevation) {
  for (const auto& ll : localSubmap->laneletLayer) {
    if (!trafficRules->canPass(ll)) {
      continue;
    }

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
      edges_[ll.id()].push_back(Edge(ll.id(), rightLL->id(), true));
    }
  }
}

void MapData::initLaneletInstances(LaneletSubmapConstPtr& localSubmap,
                                    lanelet::routing::RoutingGraphConstPtr localSubmapGraph,
                                    traffic_rules::TrafficRulesPtr trafficRules, bool ignoreMapElevation) {
  for (const auto& ll : localSubmap->laneletLayer) {
    if (!trafficRules->canPass(ll)) {
      continue;
    }

    LaneLineStringInstancePtr leftBoundary = getLineStringFeatFromId(ll.leftBound().id(), ll.leftBound().inverted());
    LaneLineStringInstancePtr rightBoundary = getLineStringFeatFromId(ll.rightBound().id(), ll.leftBound().inverted());
    BasicLineString3d centerlineLString = ll.centerline3d().basicLineString();
    if (ignoreMapElevation) {
      for (auto& pt : centerlineLString) {
        pt[2] = 0;
      }
    }
    LaneLineStringInstancePtr centerline =
        std::make_shared<LaneLineStringInstance>(centerlineLString, ll.centerline3d().id(), LineStringType::Centerline,
                                                 Ids{ll.id()}, ll.centerline3d().inverted());
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
  ConstLanelets successorLLs = localSubmapGraph->following(current, false);
  while (!successorLLs.empty()) {
    for (size_t i = 1; i != successorLLs.size(); i++) {
      if (isLaneletInPath(initPath, successorLLs[i])) {
        continue;
      }
      edges_[current.id()].push_back(Edge(current.id(), successorLLs[i].id(), false));
      getPaths(localSubmapGraph, paths, successorLLs[i], initPath);
    }
    if (isLaneletInPath(initPath, successorLLs.front())) {
      break;
    }
    initPath.push_back(successorLLs.front());
    edges_[current.id()].push_back(Edge(current.id(), successorLLs.front().id(), false));
    current = successorLLs.front();
    successorLLs = localSubmapGraph->following(current, false);
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
  for (const auto& ll : path) {
    BasicLineString3d centerlineLString = ll.centerline3d().basicLineString();
    if (ignoreMapElevation) {
      for (auto& pt : centerlineLString) {
        pt[2] = 0;
      }
    }
    compoundCenterlines.push_back(std::make_shared<LaneLineStringInstance>(
        centerlineLString, ll.id(), LineStringType::Centerline, Ids{ll.id()}, ll.centerline3d().inverted()));
  }
  return std::make_shared<CompoundLaneLineStringInstance>(compoundCenterlines, LineStringType::Centerline);
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

    // Order the component to form continuous chains
    if (!component.empty()) {
      std::set<size_t> usedIndices;

      while (usedIndices.size() < component.size()) {
        // Find the first unused linestring
        size_t startIdx = 0;
        while (startIdx < component.size() && usedIndices.count(startIdx)) {
          startIdx++;
        }

        if (startIdx >= component.size()) {
          break;  // All linestrings have been used
        }

        // Start a new chain with this linestring
        LaneLineStringInstanceList compoundFeatures;
        usedIndices.insert(startIdx);
        Id currentEndpoint = endpointIdMap[component[startIdx]].second;
        compoundFeatures.push_back(getLineStringFeatFromId(component[startIdx], false));

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
                                     traffic_rules::TrafficRulesPtr trafficRules, bool ignoreMapElevation) {
  std::vector<CompoundElsList> compoundedBordersAndDividers;
  std::map<Id, size_t> elInsertIdx;

  std::vector<ConstLanelets> paths;
  for (const auto& ll : localSubmap->laneletLayer) {
    if (!trafficRules->canPass(ll)) {
      continue;
    }

    ConstLanelets previousLLs = localSubmapGraph->previous(ll, false);
    ConstLanelets successorLLs = localSubmapGraph->following(ll, false);
    if (previousLLs.empty()) {
      getPaths(localSubmapGraph, paths, ll);
    }
  }

  for (const auto& path : paths) {
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

  if (validLineStrings && validLaneletInstances && validCompoundLineStrings) {
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
    for (size_t i = 0; i < compoundLaneLineStrings_.size(); ++i) {
      const auto& cpdFeat = compoundLaneLineStrings_[i];
      if (cpdFeat->valid()) {
        std::vector<MatrixXd> matrices = cpdFeat->pointMatrices(pointsIn2d);
        LineStringType type = cpdFeat->type();
        for (const auto& mat : matrices) {
          tfData_->compoundLineStringsByType_[type].push_back(mat);
          tfData_->compoundLineStringInstancesByType_[type].push_back(cpdFeat);
        }
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

}  // namespace ml_converter
}  // namespace lanelet