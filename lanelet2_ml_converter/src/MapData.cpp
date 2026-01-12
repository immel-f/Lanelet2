#include "lanelet2_ml_converter/MapData.h"

#include "lanelet2_ml_converter/Utils.h"

namespace lanelet {
namespace ml_converter {

using namespace internal;

LaneDataPtr LaneData::build(LaneletSubmapConstPtr& localSubmap, lanelet::routing::RoutingGraphConstPtr localSubmapGraph,
                            traffic_rules::TrafficRulesPtr trafficRules, bool ignoreMapElevation,
                            bool untaggedDrivableAreaMode) {
  LaneDataPtr data = std::make_shared<LaneData>();
  data->initLeftBoundaries(localSubmap, localSubmapGraph, trafficRules, ignoreMapElevation);
  data->initRightBoundaries(localSubmap, localSubmapGraph, trafficRules, ignoreMapElevation);
  data->initLaneletInstances(localSubmap, localSubmapGraph, trafficRules, ignoreMapElevation);
  data->initCompoundInstances(localSubmap, localSubmapGraph, trafficRules, ignoreMapElevation);
  data->updateAssociatedCpdInstanceIndices();
  return data;
}

inline bool isRoadBorder(const ConstLineString3d& lstring) {
  Attribute type = lstring.attributeOr(AttributeName::Type, "");
  return type == AttributeValueString::RoadBorder || type == AttributeValueString::Curbstone ||
         type == AttributeValueString::Fence;
}

void LaneData::initLeftBoundaries(LaneletSubmapConstPtr& localSubmap,
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

void LaneData::initRightBoundaries(LaneletSubmapConstPtr& localSubmap,
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

void LaneData::initLaneletInstances(LaneletSubmapConstPtr& localSubmap,
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
void LaneData::getPaths(lanelet::routing::RoutingGraphConstPtr localSubmapGraph, std::vector<ConstLanelets>& paths,
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

LineStringType LaneData::getLineStringTypeFromId(Id id) {
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

LaneLineStringInstancePtr LaneData::getLineStringFeatFromId(Id id, bool inverted) {
  LaneLineStringInstances::iterator it = laneLineStrings_.find(id);
  if (it != laneLineStrings_.end()) {
    return (inverted == it->second->inverted()) ? it->second : makeInverted(it->second);
  } else {
    throw std::runtime_error("Lanelet boundary " + std::to_string(id) + " is not in the instance list!");
  }
}

std::vector<CompoundElsList> LaneData::computeCompoundLeftBorders(const ConstLanelets& path) {
  std::vector<CompoundElsList> compoundBorders;
  ConstLanelet start = path.front();
  LineStringType currType = getLineStringTypeFromId(start.leftBound3d().id());

  compoundBorders.push_back(CompoundElsList{start.leftBound3d().id(), start.leftBound3d().inverted(), currType});

  for (size_t i = 1; i != path.size(); i++) {
    LineStringType newType = getLineStringTypeFromId(path[i].leftBound3d().id());
    if (currType == newType) {
      compoundBorders.back().ids.push_back(path[i].leftBound3d().id());
      compoundBorders.back().inverted.push_back(path[i].leftBound3d().inverted());
    } else {
      compoundBorders.push_back(CompoundElsList{path[i].leftBound3d().id(), path[i].leftBound3d().inverted(), newType});
      currType = newType;
    }
  }
  return compoundBorders;
}

std::vector<CompoundElsList> LaneData::computeCompoundRightBorders(const ConstLanelets& path) {
  std::vector<CompoundElsList> compoundBorders;
  ConstLanelet start = path.front();
  LineStringType currType = getLineStringTypeFromId(start.rightBound3d().id());

  compoundBorders.push_back(CompoundElsList{start.rightBound3d().id(), start.rightBound3d().inverted(), currType});

  for (size_t i = 1; i != path.size(); i++) {
    LineStringType newType = getLineStringTypeFromId(path[i].rightBound3d().id());
    if (currType == newType) {
      compoundBorders.back().ids.push_back(path[i].rightBound3d().id());
      compoundBorders.back().inverted.push_back(path[i].rightBound3d().inverted());
    } else {
      compoundBorders.push_back(
          CompoundElsList{path[i].rightBound3d().id(), path[i].rightBound3d().inverted(), newType});
      currType = newType;
    }
  }
  return compoundBorders;
}

CompoundLaneLineStringInstancePtr LaneData::computeCompoundCenterline(const ConstLanelets& path,
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

void LaneData::computeDrivableAreaBorders(LaneletSubmapConstPtr& localSubmap) {
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
          compoundLineStrings_.push_back(
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

void LaneData::initCompoundInstances(LaneletSubmapConstPtr& localSubmap,
                                     lanelet::routing::RoutingGraphConstPtr localSubmapGraph,
                                     traffic_rules::TrafficRulesPtr trafficRules, bool ignoreMapElevation,
                                     bool untaggedDrivableAreaMode) {
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
    compoundLineStrings_.push_back(computeCompoundCenterline(path, ignoreMapElevation));
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
    compoundLineStrings_.push_back(std::make_shared<CompoundLaneLineStringInstance>(toBeCompounded, cmpdType));
  }

  // Process drivable area borders if not in untagged mode
  if (!untaggedDrivableAreaMode) {
    computeDrivableAreaBorders(localSubmap);
  }
}

void LaneData::updateAssociatedCpdInstanceIndices() {
  for (size_t i = 0; i < compoundLineStrings_.size(); i++) {
    const auto& cpdFeat = compoundLineStrings_[i];
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

bool LaneData::processAll(const OrientedRect& bbox, const ParametrizationType& paramType, int32_t nPoints, double pitch,
                          double roll) {
  bool validLineStrings = processInstances(laneLineStrings_, bbox, paramType, nPoints, pitch, roll);
  bool validLaneletInstances = processInstances(laneletInstances_, bbox, paramType, nPoints, pitch, roll);
  bool validCompoundLineStrings = processInstances(compoundLineStrings_, bbox, paramType, nPoints, pitch, roll);

  if (validLineStrings && validLaneletInstances && validCompoundLineStrings) {
    return true;
  } else {
    return false;
  }
}

LaneData::TensorInstanceData LaneData::getTensorInstanceData(bool pointsIn2d, bool ignoreBuffer) {
  if (!tfData_.has_value() || ignoreBuffer) {
    tfData_ = LaneData::TensorInstanceData();
    tfData_->uuid_ = uuid_;
    tfData_->roadBorders_ = getPointMatrices(validRoadBorders(), pointsIn2d);
    tfData_->laneDividers_ = getPointMatrices(validLaneDividers(), pointsIn2d);
    tfData_->compoundRoadBorders_ = getPointMatrices(validCompoundRoadBorders(), pointsIn2d);
    tfData_->compoundLaneDividers_ = getPointMatrices(validCompoundLaneDividers(), pointsIn2d);
    tfData_->compoundCenterlines_ = getPointMatrices(validCompoundCenterlines(), pointsIn2d);
    for (const auto& ft : validLaneDividers()) {
      tfData_->laneDividerTypes_.push_back(ft.second->typeInt());
    }
    for (const auto& ft : validCompoundLaneDividers()) {
      tfData_->compoundLaneDividerTypes_.push_back(ft->typeInt());
    }
    size_t pointMatrixIdxRb = 0;
    for (const auto& cpdFeat : compoundLineStrings_) {
      if (cpdFeat->type() == LineStringType::RoadBorder) {
        for (const auto& lString : cpdFeat->cutResampledAndTransformedInstance()) {
          tfData_->pointMatrixCpdRoadBorder_[pointMatrixIdxRb] = cpdFeat;
          pointMatrixIdxRb++;
        }
      }
    }
    size_t pointMatrixIdxLd = 0;
    for (const auto& cpdFeat : compoundLineStrings_) {
      if (cpdFeat->type() != LineStringType::RoadBorder && cpdFeat->type() != LineStringType::Centerline) {
        for (const auto& lString : cpdFeat->cutResampledAndTransformedInstance()) {
          tfData_->pointMatrixCpdLaneDivider_[pointMatrixIdxLd] = cpdFeat;
          pointMatrixIdxLd++;
        }
      }
    }
    size_t pointMatrixIdxCl = 0;
    for (const auto& cpdFeat : compoundLineStrings_) {
      if (cpdFeat->type() == LineStringType::Centerline) {
        for (const auto& lString : cpdFeat->cutResampledAndTransformedInstance()) {
          tfData_->pointMatrixCpdCenterline_[pointMatrixIdxCl] = cpdFeat;
          pointMatrixIdxCl++;
        }
      }
    }
  }
  return tfData_.value();
}

CompoundLaneLineStringInstanceList LaneData::associatedCpdLineStringsOfType(Id mapId, LineStringType type) const {
  CompoundLaneLineStringInstanceList assoFeats;
  try {
    const auto& typeIndices = associatedCpdLineStringsIndices_.at(type);
    for (const auto& idx : typeIndices.at(mapId)) {
      assoFeats.push_back(compoundLineStrings_[idx]);
    }
  } catch (const std::out_of_range&) {
    return assoFeats;
  }
  return assoFeats;
}

CompoundLaneLineStringInstanceList LaneData::associatedCpdRoadBorders(Id mapId) {
  return associatedCpdLineStringsOfType(mapId, LineStringType::RoadBorder);
}

CompoundLaneLineStringInstanceList LaneData::associatedCpdLaneDividers(Id mapId) {
  // For lane dividers, we need to check all divider types (Dashed, Solid, Mixed, Virtual)
  CompoundLaneLineStringInstanceList result;
  for (const auto& pair : associatedCpdLineStringsIndices_) {
    LineStringType type = pair.first;
    if (type == LineStringType::Dashed || type == LineStringType::Solid || type == LineStringType::Mixed ||
        type == LineStringType::Virtual) {
      CompoundLaneLineStringInstanceList typeResult = associatedCpdLineStringsOfType(mapId, type);
      result.insert(result.end(), typeResult.begin(), typeResult.end());
    }
  }
  return result;
}

CompoundLaneLineStringInstanceList LaneData::associatedCpdCenterlines(Id mapId) {
  return associatedCpdLineStringsOfType(mapId, LineStringType::Centerline);
}

CompoundLaneLineStringInstanceList LaneData::associatedCpdDrivableAreaBorders(Id mapId) {
  return associatedCpdLineStringsOfType(mapId, LineStringType::DrivableArea);
}

CompoundLaneLineStringInstancePtr pointMatrixCpdFeat(
    size_t index, const std::map<size_t, CompoundLaneLineStringInstancePtr>& assoFeats) {
  CompoundLaneLineStringInstancePtr feat;
  try {
    feat = assoFeats.at(index);
  } catch (const std::out_of_range& e) {
    throw std::out_of_range("A point matrix with index " + std::to_string(index) + " does not exist!");
  }
  return feat;
}

CompoundLaneLineStringInstancePtr LaneData::TensorInstanceData::pointMatrixCpdRoadBorder(size_t index) {
  return pointMatrixCpdFeat(index, pointMatrixCpdRoadBorder_);
}

CompoundLaneLineStringInstancePtr LaneData::TensorInstanceData::pointMatrixCpdLaneDivider(size_t index) {
  return pointMatrixCpdFeat(index, pointMatrixCpdLaneDivider_);
}

CompoundLaneLineStringInstancePtr LaneData::TensorInstanceData::pointMatrixCpdCenterline(size_t index) {
  return pointMatrixCpdFeat(index, pointMatrixCpdCenterline_);
}

LaneLineStringInstances LaneData::lineStringsOfType(LineStringType type) const {
  LaneLineStringInstances result;
  for (const auto& pair : laneLineStrings_) {
    if (pair.second->type() == type) {
      result.insert(pair);
    }
  }
  return result;
}

LaneLineStringInstances LaneData::roadBorders() const { return lineStringsOfType(LineStringType::RoadBorder); }

LaneLineStringInstances LaneData::laneDividers() const {
  LaneLineStringInstances result;
  for (const auto& pair : laneLineStrings_) {
    if (pair.second->type() == LineStringType::Dashed || pair.second->type() == LineStringType::Solid ||
        pair.second->type() == LineStringType::Mixed || pair.second->type() == LineStringType::Virtual) {
      result.insert(pair);
    }
  }
  return result;
}

LaneLineStringInstances LaneData::drivableAreaBorders() const {
  return lineStringsOfType(LineStringType::DrivableArea);
}

CompoundLaneLineStringInstanceList LaneData::compoundLineStringsOfType(LineStringType type) const {
  CompoundLaneLineStringInstanceList result;
  for (const auto& feat : compoundLineStrings_) {
    if (feat->type() == type) {
      result.push_back(feat);
    }
  }
  return result;
}

CompoundLaneLineStringInstanceList LaneData::compoundRoadBorders() const {
  return compoundLineStringsOfType(LineStringType::RoadBorder);
}

CompoundLaneLineStringInstanceList LaneData::compoundLaneDividers() const {
  CompoundLaneLineStringInstanceList result;
  for (const auto& feat : compoundLineStrings_) {
    if (feat->type() == LineStringType::Dashed || feat->type() == LineStringType::Solid ||
        feat->type() == LineStringType::Mixed || feat->type() == LineStringType::Virtual) {
      result.push_back(feat);
    }
  }
  return result;
}

CompoundLaneLineStringInstanceList LaneData::compoundCenterlines() const {
  return compoundLineStringsOfType(LineStringType::Centerline);
}

CompoundLaneLineStringInstanceList LaneData::compoundDrivableAreaBorders() const {
  return compoundLineStringsOfType(LineStringType::DrivableArea);
}

LaneLineStringInstances LaneData::validRoadBorders() const { return getValidElements(roadBorders()); }

LaneLineStringInstances LaneData::validLaneDividers() const { return getValidElements(laneDividers()); }

CompoundLaneLineStringInstanceList LaneData::validCompoundRoadBorders() const {
  return getValidElements(compoundRoadBorders());
}

CompoundLaneLineStringInstanceList LaneData::validCompoundLaneDividers() const {
  return getValidElements(compoundLaneDividers());
}

CompoundLaneLineStringInstanceList LaneData::validCompoundCenterlines() const {
  return getValidElements(compoundCenterlines());
}

CompoundLaneLineStringInstanceList LaneData::validCompoundDrivableAreaBorders() const {
  return getValidElements(compoundDrivableAreaBorders());
}

}  // namespace ml_converter
}  // namespace lanelet