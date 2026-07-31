/// Tests for traffic sign handling: the German StVO subtype table in teTypeToEnum and the collection
/// of sign linestrings into TEInstances.

#include <gtest/gtest.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <string>
#include <vector>

#include "lanelet2_ml_converter/MapData.h"
#include "lanelet2_ml_converter/MapInstances.h"
#include "lanelet2_ml_converter/Utils.h"
#include "lanelet2_routing/RoutingGraph.h"
#include "test_map.h"

using namespace lanelet;
using namespace lanelet::ml_converter;
using namespace lanelet::ml_converter::tests;

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

/// vertical linestring carrying the given type/subtype, like a real sign
ConstLineString3d makeSign(Id id, const std::string& type, const std::string& subtype) {
  LineString3d lString(id, Points3d{Point3d(id * 10, 0, 0, 2.0), Point3d(id * 10 + 1, 0, 0, 2.6)});
  lString.setAttribute(AttributeName::Type, type);
  lString.setAttribute(AttributeName::Subtype, subtype);
  return ConstLineString3d(lString);
}

}  // namespace

TEST(MLConverterTrafficSigns, TeTypeToEnumCoversGermanSubtypes) {  // NOLINT
  struct SubtypeCase {
    const char* subtype;
    TEType expected;
  };
  const std::vector<SubtypeCase> cases{{"de201", TEType::TSCrossbuck},
                                       {"de201-50", TEType::TSCrossbuck},
                                       {"de205", TEType::TSYield},
                                       {"de206", TEType::TSStop},
                                       {"de209", TEType::TSTurnRight},
                                       {"de209-10", TEType::TSTurnLeft},
                                       {"de209-30", TEType::TSGoStraight},
                                       {"de211", TEType::TSTurnRight},
                                       {"de211-10", TEType::TSTurnLeft},
                                       {"de214", TEType::TSGoStraightOrRight},
                                       {"de214-10", TEType::TSGoStraightOrLeft},
                                       {"de214-30", TEType::TSTurnLeftOrRight},
                                       {"de215", TEType::TSRoundabout},
                                       {"de220-10", TEType::TSOneWayStreet},
                                       {"de220-20", TEType::TSOneWayStreet},
                                       {"de222", TEType::TSPassRight},
                                       {"de222-10", TEType::TSPassLeft},
                                       {"de267", TEType::TSNoEntry},
                                       {"de274-50", TEType::TSSpeedLimit},
                                       {"de274.1", TEType::TSSpeedLimit},
                                       {"de301", TEType::TSRightOfWay},
                                       {"de306", TEType::TSPriorityRoad},
                                       {"de310", TEType::TSMisc},
                                       {"de350-10", TEType::TSPedestrianCrossing},
                                       {"de350-20", TEType::TSPedestrianCrossing},
                                       {"de999", TEType::TSMisc}};

  Id id = 5000;
  for (const auto& testCase : cases) {
    ConstLineString3d sign = makeSign(id++, AttributeValueString::TrafficSign, testCase.subtype);
    EXPECT_EQ(teTypeToEnum(sign), testCase.expected) << "subtype " << testCase.subtype;
  }
}

TEST(MLConverterTrafficSigns, TeTypeToEnumNonSignTypes) {  // NOLINT
  EXPECT_EQ(teTypeToEnum(makeSign(6000, "stop_line", "")), TEType::StopLine);
  EXPECT_EQ(teTypeToEnum(makeSign(6001, "arrow", "left")), TEType::ArrowTurnLeft);
  EXPECT_EQ(teTypeToEnum(makeSign(6002, "arrow", "straight_left")), TEType::ArrowGoStraightOrLeft);
  EXPECT_EQ(teTypeToEnum(makeSign(6003, "arrow", "nonsense")), TEType::Unknown);
  EXPECT_EQ(teTypeToEnum(makeSign(6004, "symbol", "bicycle")), TEType::BikeSymbol);
  EXPECT_EQ(teTypeToEnum(makeSign(6005, "symbol", "50")), TEType::Symbol50);
  EXPECT_EQ(teTypeToEnum(makeSign(6006, "symbol", "nonsense")), TEType::Unknown);
  // traffic lights are distinguished by type, not by subtype
  EXPECT_EQ(teTypeToEnum(makeSign(6007, AttributeValueString::TrafficLight, "red_yellow_green")), TEType::TLCar);
  EXPECT_EQ(teTypeToEnum(makeSign(6008, AttributeValueString::TrafficLight, "nonsense")), TEType::TLCar);
  EXPECT_EQ(teTypeToEnum(makeSign(6009, "traffic_light_bikes", "")), TEType::TLBike);
  EXPECT_EQ(teTypeToEnum(makeSign(6010, "traffic_light_pedestrians", "")), TEType::TLPedestrian);
  EXPECT_EQ(teTypeToEnum(makeSign(6011, "traffic_light_misc", "")), TEType::TLMisc);
  EXPECT_EQ(teTypeToEnum(makeSign(6012, "nonsense", "")), TEType::Unknown);
}

TEST_F(MLConverterTest, TrafficSignsAreCollectedFromTheMap) {  // NOLINT
  traffic_rules::TrafficRulesPtr trafficRules{
      traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Vehicle)};
  traffic_rules::TrafficRulesPtr bikeTrafficRules{
      traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Bicycle)};
  routing::RoutingGraphConstPtr laneletMapGraph = routing::RoutingGraph::build(*laneletMap, *trafficRules);
  routing::RoutingGraphConstPtr bikeMapGraph = routing::RoutingGraph::build(*laneletMap, *bikeTrafficRules);

  LaneletSubmapConstPtr laneletSubmap = wholeTestMapSubmap(laneletMap);
  MapDataPtr mapData = MapData::build(laneletSubmap, laneletMapGraph, trafficRules, bikeMapGraph);

  TEInstances stopSigns = mapData->teInstancesOfType(TEType::TSStop);
  EXPECT_EQ(stopSigns.size(), 1u);
  EXPECT_TRUE(stopSigns.find(1038) != stopSigns.end());

  TEInstances speedLimits = mapData->teInstancesOfType(TEType::TSSpeedLimit);
  EXPECT_EQ(speedLimits.size(), 1u);
  EXPECT_TRUE(speedLimits.find(1039) != speedLimits.end());

  TEInstances crossbucks = mapData->teInstancesOfType(TEType::TSCrossbuck);
  EXPECT_EQ(crossbucks.size(), 1u);
  EXPECT_TRUE(crossbucks.find(1040) != crossbucks.end());

  TEInstances miscSigns = mapData->teInstancesOfType(TEType::TSMisc);
  EXPECT_EQ(miscSigns.size(), 1u);
  EXPECT_TRUE(miscSigns.find(1041) != miscSigns.end());

  // signs carry no edges of their own in the test map
  EXPECT_TRUE(mapData->teEdges().find(1038) == mapData->teEdges().end());

  // and they process into resampled point matrices like any other vertical traffic element
  // (the aggregate return value is not checked here - ll2019 has a degenerate centerline, see MapDataTrafficElements)
  mapData->processAll(bbox, ParametrizationType::LineString, true, 20, true, 10, 0.0, 0.0);
  MapData::TensorInstanceData tfData = mapData->getTensorInstanceData(true, false);
  for (TEType type : {TEType::TSStop, TEType::TSSpeedLimit, TEType::TSCrossbuck, TEType::TSMisc}) {
    std::vector<MatrixXd> mats = tfData.teInstancesOfType(type);
    ASSERT_EQ(mats.size(), 1u) << "type " << static_cast<int>(type);
    EXPECT_EQ(mats.front().rows(), 10);
    EXPECT_EQ(mats.front().cols(), 2);
  }
}
