/// Tests for the LineStringType / TEType grouping mechanism: which types share a group, which
/// representative type they resolve to, and how that governs the merging and labelling of compound
/// instances.

#include <gtest/gtest.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <algorithm>
#include <set>
#include <vector>

#include "lanelet2_ml_converter/MapData.h"
#include "lanelet2_ml_converter/MapInstances.h"
#include "lanelet2_ml_converter/Types.h"
#include "lanelet2_routing/RoutingGraph.h"
#include "test_map.h"

using namespace lanelet;
using namespace lanelet::ml_converter;
using namespace lanelet::ml_converter::tests;

namespace lanelet {
namespace ml_converter {
// defined in MapData.cpp, deliberately not part of the public header
std::vector<internal::CompoundElsList> splitAtRemovedElements(const internal::CompoundElsList& elsList,
                                                              const std::vector<Id>& removedElements);
std::vector<internal::CompoundElsList> resolveCompoundCandidates(std::vector<internal::CompoundElsList> candidates);
}  // namespace ml_converter
}  // namespace lanelet

namespace {

LaneletSubmapConstPtr wholeTestMapSubmap(const LaneletMapConstPtr& laneletMap) {
  auto nonConstMap = std::const_pointer_cast<LaneletMap>(laneletMap);
  Lanelets allLanelets(nonConstMap->laneletLayer.begin(), nonConstMap->laneletLayer.end());
  auto submapUPtr = utils::createSubmap(allLanelets, {});
  LineStrings3d allLineStrings(nonConstMap->lineStringLayer.begin(), nonConstMap->lineStringLayer.end());
  for (const auto& lineString : allLineStrings) {
    submapUPtr->add(lineString);
  }
  return std::shared_ptr<const LaneletSubmap>(std::move(submapUPtr));
}

using CompoundElsList = lanelet::ml_converter::internal::CompoundElsList;

CompoundElsList chain(std::vector<Id> ids) {
  return CompoundElsList(ids, std::vector<bool>(ids.size(), false), LineStringType::Divider);
}

/// resolved chains as plain id lists, sorted so that the comparison does not depend on the emission order
std::vector<std::vector<Id>> resolvedChainIds(std::vector<CompoundElsList> candidates) {
  std::vector<std::vector<Id>> ids;
  for (const auto& resolved : resolveCompoundCandidates(candidates)) {
    ids.push_back(resolved.ids);
  }
  std::sort(ids.begin(), ids.end());
  return ids;
}

std::set<Id> featureMapIds(const CompoundLaneLineStringInstancePtr& compound) {
  std::set<Id> ids;
  for (const auto& feat : compound->features()) {
    ids.insert(feat->mapID());
  }
  return ids;
}

/// Compounds of the given types are built by chaining boundaries along lanelet paths. Overlapping paths must
/// not make a linestring show up in two of them.
void expectNoSharedLineStrings(const MapDataPtr& data, const std::vector<LineStringType>& types) {
  std::map<Id, size_t> occurrences;
  for (LineStringType type : types) {
    for (const auto& compound : data->compoundLineStringsOfType(type)) {
      for (const Id& id : featureMapIds(compound)) {
        occurrences[id]++;
      }
    }
  }
  for (const auto& occurrence : occurrences) {
    EXPECT_EQ(occurrence.second, 1u) << "linestring " << occurrence.first << " belongs to " << occurrence.second
                                     << " compounds";
  }
}

}  // namespace

TEST(MLConverterGrouping, EveryGroupIsReachable) {  // NOLINT
  // A type resolves to the first group that lists it, so listing a type in more than one group makes the
  // later entry unreachable. Every group of a well-formed grouping has to be reachable by some type.
  std::vector<std::pair<std::string, LineStringTypeGrouping>> groupings{
      {"default", getDefaultLineStringTypeGrouping()},
      {"roadBorderMerged", getRoadBorderMergedGrouping()},
      {"mapTRDefaultSimple", getMapTRDefaultSimpleGrouping()},
      {"m3TRDefault", getM3TRDefaultGrouping()}};

  for (const auto& named : groupings) {
    std::set<int> reachable;
    for (int type = 0; type <= static_cast<int>(LineStringType::ZebraCrossing); type++) {
      int idx = getLineStringTypeGroupIndex(static_cast<LineStringType>(type), named.second);
      if (idx >= 0) {
        reachable.insert(idx);
      }
    }
    EXPECT_EQ(reachable.size(), named.second.size()) << "grouping " << named.first << " has unreachable groups";
  }

  TETypeGrouping teGrouping = getDefaultTETypeGrouping();
  std::set<int> reachableTe;
  for (int type = 0; type <= static_cast<int>(TEType::Unknown); type++) {
    int idx = getTETypeGroupIndex(static_cast<TEType>(type), teGrouping);
    if (idx >= 0) {
      reachableTe.insert(idx);
    }
  }
  EXPECT_EQ(reachableTe.size(), teGrouping.size());
}

TEST(MLConverterGrouping, GroupingCoverageIsChecked) {  // NOLINT
  // every built-in grouping covers each type a boundary can have
  EXPECT_NO_THROW(checkLineStringTypeGroupingCoverage(getDefaultLineStringTypeGrouping()));
  EXPECT_NO_THROW(checkLineStringTypeGroupingCoverage(getRoadBorderMergedGrouping()));
  EXPECT_NO_THROW(checkLineStringTypeGroupingCoverage(getMapTRDefaultSimpleGrouping()));
  EXPECT_NO_THROW(checkLineStringTypeGroupingCoverage(getM3TRDefaultGrouping()));

  // types that can never appear on a boundary need no group - the merged groupings do not list Divider
  EXPECT_EQ(getLineStringTypeGroupIndex(LineStringType::Divider, getMapTRDefaultSimpleGrouping()), -1);

  // dropping a type a boundary can have is rejected, and the message names it
  LineStringTypeGrouping withoutFence;
  for (const auto& group : getDefaultLineStringTypeGrouping()) {
    if (group.second != LineStringType::Fence) {
      withoutFence.push_back(group);
    }
  }
  EXPECT_THROW(checkLineStringTypeGroupingCoverage(withoutFence), std::runtime_error);
  try {
    checkLineStringTypeGroupingCoverage(withoutFence);
    FAIL() << "an incomplete grouping has to be rejected";
  } catch (const std::runtime_error& error) {
    EXPECT_NE(std::string(error.what()).find("Fence"), std::string::npos) << error.what();
  }
}

TEST_F(MLConverterTest, MapDataBuildRejectsIncompleteGrouping) {  // NOLINT
  traffic_rules::TrafficRulesPtr trafficRules{
      traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Vehicle)};
  routing::RoutingGraphConstPtr laneletMapGraph = routing::RoutingGraph::build(*laneletMap, *trafficRules);
  LaneletSubmapConstPtr laneletSubmap = wholeTestMapSubmap(laneletMap);

  LineStringTypeGrouping withoutDashed;
  for (const auto& group : getDefaultLineStringTypeGrouping()) {
    if (group.second != LineStringType::Dashed) {
      withoutDashed.push_back(group);
    }
  }
  EXPECT_THROW(MapData::build(laneletSubmap, laneletMapGraph, trafficRules, nullptr, false, withoutDashed,
                              getDefaultTETypeGrouping()),
               std::runtime_error);
  EXPECT_NO_THROW(MapData::build(laneletSubmap, laneletMapGraph, trafficRules, nullptr, false,
                                 getDefaultLineStringTypeGrouping(), getDefaultTETypeGrouping()));
}

TEST(MLConverterGrouping, SplitAtRemovedElements) {  // NOLINT
  using CompoundElsList = lanelet::ml_converter::internal::CompoundElsList;
  const std::vector<Id> ids{1, 2, 3, 4};
  const std::vector<bool> inv{false, true, false, true};
  const CompoundElsList chain(ids, inv, LineStringType::Divider);

  {  // a run at the front
    std::vector<CompoundElsList> runs = splitAtRemovedElements(chain, {1, 2});
    ASSERT_EQ(runs.size(), 1u);
    EXPECT_EQ(runs[0].ids, (std::vector<Id>{3, 4}));
    EXPECT_EQ(runs[0].inverted, (std::vector<bool>{false, true}));
  }
  {  // a run at the back
    std::vector<CompoundElsList> runs = splitAtRemovedElements(chain, {3, 4});
    ASSERT_EQ(runs.size(), 1u);
    EXPECT_EQ(runs[0].ids, (std::vector<Id>{1, 2}));
    EXPECT_EQ(runs[0].inverted, (std::vector<bool>{false, true}));
  }
  {  // a run in the middle leaves two contiguous chains rather than one with a gap
    std::vector<CompoundElsList> runs = splitAtRemovedElements(chain, {2, 3});
    ASSERT_EQ(runs.size(), 2u);
    EXPECT_EQ(runs[0].ids, (std::vector<Id>{1}));
    EXPECT_EQ(runs[0].inverted, (std::vector<bool>{false}));
    EXPECT_EQ(runs[1].ids, (std::vector<Id>{4}));
    EXPECT_EQ(runs[1].inverted, (std::vector<bool>{true}));
  }
  {  // removing at both ends keeps the middle together
    std::vector<CompoundElsList> runs = splitAtRemovedElements(chain, {1, 4});
    ASSERT_EQ(runs.size(), 1u);
    EXPECT_EQ(runs[0].ids, (std::vector<Id>{2, 3}));
    EXPECT_EQ(runs[0].inverted, (std::vector<bool>{true, false}));
  }
  {  // taking everything over leaves nothing, and the caller drops the emptied entry
    EXPECT_TRUE(splitAtRemovedElements(chain, {1, 2, 3, 4}).empty());
  }
  {  // nothing in common
    std::vector<CompoundElsList> runs = splitAtRemovedElements(chain, {7, 8});
    ASSERT_EQ(runs.size(), 1u);
    EXPECT_EQ(runs[0].ids, ids);
    EXPECT_EQ(runs[0].inverted, inv);
  }
}

TEST(MLConverterGrouping, ResolveCompoundCandidates) {  // NOLINT
  // the longest candidate stays intact, shorter overlapping ones contribute only what is left of them
  EXPECT_EQ(resolvedChainIds({chain({1}), chain({2}), chain({1, 2, 3})}), (std::vector<std::vector<Id>>{{1, 2, 3}}));
  EXPECT_EQ(resolvedChainIds({chain({1, 26}), chain({2}), chain({1, 2, 3})}),
            (std::vector<std::vector<Id>>{{1, 2, 3}, {26}}));

  // a short candidate may never shatter a longer one
  EXPECT_EQ(resolvedChainIds({chain({1, 2, 3}), chain({2})}), (std::vector<std::vector<Id>>{{1, 2, 3}}));

  // a candidate bridging two others is the longest and wins, leaving their unclaimed ends behind
  EXPECT_EQ(resolvedChainIds({chain({1, 2, 3}), chain({4, 5, 6}), chain({2, 3, 7, 4, 5})}),
            (std::vector<std::vector<Id>>{{1}, {2, 3, 7, 4, 5}, {6}}));

  // greedy alone would leave {1,2,3},{4},{5} here because {2,3,4} claims 4 before {4,5} is looked at;
  // the improvement pass adopts {4,5} because that removes two chains and creates one
  EXPECT_EQ(resolvedChainIds({chain({1, 2, 3}), chain({2, 3, 4}), chain({4, 5})}),
            (std::vector<std::vector<Id>>{{1, 2, 3}, {4, 5}}));

  // every element of every candidate ends up in exactly one chain
  std::vector<CompoundElsList> candidates{chain({1, 2, 3}), chain({2, 3, 4}), chain({4, 5}), chain({6, 7})};
  std::map<Id, size_t> occurrences;
  for (const auto& resolved : resolveCompoundCandidates(candidates)) {
    for (const Id& el : resolved.ids) {
      occurrences[el]++;
    }
  }
  EXPECT_EQ(occurrences.size(), 7u);
  for (const auto& occurrence : occurrences) {
    EXPECT_EQ(occurrence.second, 1u) << "element " << occurrence.first;
  }
}

TEST(MLConverterGrouping, ResolveCompoundCandidatesIsOrderIndependent) {  // NOLINT
  // the candidates are collected from the lanelet paths in whatever order those were enumerated, so the result
  // may not depend on that order
  std::vector<CompoundElsList> candidates{chain({1, 2, 3}), chain({2, 3, 4}), chain({4, 5}), chain({6, 7})};
  const std::vector<std::vector<Id>> expected = resolvedChainIds(candidates);

  std::vector<size_t> order{0, 1, 2, 3};
  size_t permutations = 0;
  do {
    std::vector<CompoundElsList> permuted;
    for (const size_t& i : order) {
      permuted.push_back(candidates[i]);
    }
    EXPECT_EQ(resolvedChainIds(permuted), expected) << "permutation " << permutations;
    permutations++;
  } while (std::next_permutation(order.begin(), order.end()));
  EXPECT_EQ(permutations, 24u);
}

TEST_F(MLConverterTest, CompoundMergingUnderMapTRGrouping) {  // NOLINT
  traffic_rules::TrafficRulesPtr trafficRules{
      traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Vehicle)};
  traffic_rules::TrafficRulesPtr bikeTrafficRules{
      traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Bicycle)};
  routing::RoutingGraphConstPtr laneletMapGraph = routing::RoutingGraph::build(*laneletMap, *trafficRules);
  routing::RoutingGraphConstPtr bikeMapGraph = routing::RoutingGraph::build(*laneletMap, *bikeTrafficRules);

  LaneletSubmapConstPtr laneletSubmap = wholeTestMapSubmap(laneletMap);

  MapDataPtr defaultData = MapData::build(laneletSubmap, laneletMapGraph, trafficRules, bikeMapGraph, false,
                                          getDefaultLineStringTypeGrouping(), getDefaultTETypeGrouping());
  MapDataPtr mapTRData = MapData::build(laneletSubmap, laneletMapGraph, trafficRules, bikeMapGraph, false,
                                        getMapTRDefaultSimpleGrouping(), getDefaultTETypeGrouping());

  // The lanelet path 2000 -> 2001 -> 2002 has the right boundaries l1005 (dashed), l1004 (solid) and
  // l1003 (dashed). Under the default grouping those are three different groups and therefore stay
  // three separate compound instances.
  CompoundLaneLineStringInstanceList defaultDashed =
      defaultData->associatedCpdLineStringsOfType(1005, LineStringType::Dashed);
  ASSERT_FALSE(defaultDashed.empty());
  for (const auto& compound : defaultDashed) {
    EXPECT_EQ(compound->features().size(), 1u);
    EXPECT_EQ(featureMapIds(compound), (std::set<Id>{1005}));
  }

  CompoundLaneLineStringInstanceList defaultSolid =
      defaultData->associatedCpdLineStringsOfType(1004, LineStringType::Solid);
  ASSERT_FALSE(defaultSolid.empty());
  for (const auto& compound : defaultSolid) {
    EXPECT_EQ(compound->features().size(), 1u);
  }

  EXPECT_TRUE(defaultData->compoundLineStringsOfType(LineStringType::Divider).empty());

  // Under the MapTR grouping dashed and solid share the Divider group, so the same three boundaries
  // become one compound instance and no instance carries the Dashed or Solid type any more.
  EXPECT_TRUE(mapTRData->compoundLineStringsOfType(LineStringType::Dashed).empty());
  EXPECT_TRUE(mapTRData->compoundLineStringsOfType(LineStringType::Solid).empty());

  // A boundary is chained with its neighbours once per lanelet path it belongs to, and different paths can
  // yield different chains: l1003 is chained with l1004 and l1005 along the path through ll2000..ll2002, but
  // forms a chain of its own on the path where a virtual boundary precedes it. The shorter chain is fully
  // covered by the longer one and must not survive as a separate compound.
  const std::set<Id> mergedIds{1003, 1004, 1005};
  for (Id id : mergedIds) {
    CompoundLaneLineStringInstanceList assoc = mapTRData->associatedCpdLineStringsOfType(id, LineStringType::Divider);
    ASSERT_EQ(assoc.size(), 1u) << "linestring " << id << " has to belong to exactly one Divider compound";
    EXPECT_EQ(featureMapIds(assoc.front()), mergedIds) << "for " << id;
  }

  // under the default grouping the three boundaries never end up in a common compound
  for (const auto& compound : defaultData->compoundLineStringsOfType(LineStringType::Dashed)) {
    EXPECT_NE(featureMapIds(compound), mergedIds);
  }

  // no linestring may be handed out as part of two different path-derived compounds, with or without grouping
  expectNoSharedLineStrings(defaultData, {LineStringType::RoadBorder, LineStringType::Dashed, LineStringType::Solid,
                                          LineStringType::Virtual});
  expectNoSharedLineStrings(mapTRData, {LineStringType::RoadBorder, LineStringType::Divider, LineStringType::Virtual});

  // merging must not lose geometry: the compound spans all three boundaries
  // (the aggregate return value is not checked here - ll2019 has a degenerate centerline, see MapDataTrafficElements)
  mapTRData->processAll(bbox, ParametrizationType::LineString, true, 20, false, 10, 0.0, 0.0);
  std::vector<MatrixXd> dividerMats =
      mapTRData->getTensorInstanceData(true, false).compoundLineStringsOfType(LineStringType::Divider);
  EXPECT_FALSE(dividerMats.empty());
  for (const auto& mat : dividerMats) {
    EXPECT_EQ(mat.rows(), 20);
    EXPECT_EQ(mat.cols(), 2);
  }
}
