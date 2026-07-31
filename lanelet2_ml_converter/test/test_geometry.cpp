/// Geometry invariants of the generated instances: the orientation of closed perimeters and lanelet
/// boundaries, and the elevation of instances that were clipped against the local bounding box.

#include <gtest/gtest.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <boost/geometry.hpp>
#include <vector>

#include "lanelet2_ml_converter/MapData.h"
#include "lanelet2_ml_converter/MapDataInterface.h"
#include "lanelet2_ml_converter/MapInstances.h"
#include "lanelet2_ml_converter/Utils.h"
#include "lanelet2_routing/RoutingGraph.h"
#include "test_map.h"

using namespace lanelet;
using namespace lanelet::ml_converter;
using namespace lanelet::ml_converter::tests;

namespace {

double signedArea(const MatrixXd& ring) {
  double area = 0;
  for (int i = 0; i + 1 < ring.rows(); i++) {
    area = area + ring(i, 0) * ring(i + 1, 1) - ring(i + 1, 0) * ring(i, 1);
  }
  return 0.5 * area;
}

BasicLineString2d toLineString2d(const MatrixXd& mat) {
  BasicLineString2d lString;
  for (int i = 0; i < mat.rows(); i++) {
    lString.push_back(BasicPoint2d(mat(i, 0), mat(i, 1)));
  }
  return lString;
}

/// A single crosswalk lanelet, 4 m wide and 6 m long. With rightBoundStoredReversed the right boundary is
/// stored in the opposite direction and inverted by the lanelet, which is what map editors produce when the
/// two ways happen to be digitized in opposite directions.
LaneletMapPtr makeCrosswalkMap(bool rightBoundStoredReversed) {
  Points3d leftPts{Point3d(1, -2, 0, 0), Point3d(2, -2, 3, 0), Point3d(3, -2, 6, 0)};
  Points3d rightPts{Point3d(4, 2, 0, 0), Point3d(5, 2, 3, 0), Point3d(6, 2, 6, 0)};
  if (rightBoundStoredReversed) {
    std::reverse(rightPts.begin(), rightPts.end());
  }
  LineString3d left(10, leftPts);
  left.setAttribute(AttributeName::Type, AttributeValueString::Zebra);
  LineString3d right(11, rightPts);
  right.setAttribute(AttributeName::Type, AttributeValueString::Zebra);

  Lanelet crosswalk(20, left, rightBoundStoredReversed ? right.invert() : right);
  crosswalk.setAttribute(AttributeName::Subtype, AttributeValueString::Crosswalk);
  return utils::createMap(Lanelets{crosswalk});
}

/// A map holding a single straight lanelet of the given subtype
LaneletMapPtr makeSingleLaneletMap(const std::string& subtype) {
  Points3d leftPts{Point3d(1, 0, 0, 0), Point3d(2, 0, 5, 0), Point3d(3, 0, 10, 0)};
  Points3d rightPts{Point3d(4, 3, 0, 0), Point3d(5, 3, 5, 0), Point3d(6, 3, 10, 0)};
  LineString3d left(10, leftPts);
  left.setAttribute(AttributeName::Type, AttributeValueString::RoadBorder);
  LineString3d right(11, rightPts);
  right.setAttribute(AttributeName::Type, AttributeValueString::RoadBorder);

  Lanelet lanelet(20, left, right);
  lanelet.setAttribute(AttributeName::Subtype, subtype);
  return utils::createMap(Lanelets{lanelet});
}

/// Submap containing every lanelet and linestring of the test map
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

}  // namespace

TEST(MLConverterGeometry, PedestrianCrossingPerimeterWinding) {  // NOLINT
  // The perimeter is built from the crosswalk's left and right boundary. Its winding must follow the
  // lanelet, so that it stays a simple clockwise ring no matter how the two ways are stored in the map.
  std::vector<MatrixXd> rings;
  for (bool reversed : {false, true}) {
    LaneletMapPtr map = makeCrosswalkMap(reversed);

    MapDataInterface::Configuration config{};
    config.paramType = ParametrizationType::LineString;
    config.submapExtentLongitudinal = 30;
    config.submapExtentLateral = 30;
    config.nPointsLanes = 9;
    config.nPointsTE = 9;

    MapDataInterface parser(map, config);
    parser.setCurrPosAndExtractSubmap2d(BasicPoint2d(0, 3), 0.0);
    MapDataPtr mapData = parser.mapData(true);

    std::vector<MatrixXd> crossings =
        mapData->getTensorInstanceData(true, false).compoundLineStringsOfType(LineStringType::ZebraCrossing);
    ASSERT_EQ(crossings.size(), 1u) << "reversed=" << reversed;
    rings.push_back(crossings.front());
  }

  for (size_t i = 0; i < rings.size(); i++) {
    const MatrixXd& ring = rings[i];
    ASSERT_EQ(ring.rows(), 9);
    // the perimeter must be closed
    EXPECT_NEAR((ring.row(0) - ring.row(ring.rows() - 1)).norm(), 0.0, 1e-9) << "ring " << i;
    // clockwise, matching lanelet2's polygon convention. A perimeter crossing itself has zero signed area.
    EXPECT_NEAR(signedArea(ring), -22.5, 1e-6) << "ring " << i;
    // and it must not cross itself
    EXPECT_TRUE(boost::geometry::is_simple(toLineString2d(ring))) << "ring " << i;
  }

  // both digitization directions have to yield exactly the same geometry
  EXPECT_TRUE(rings[0].isApprox(rings[1], 1e-9));
}

TEST(MLConverterGeometry, OnlyDrivingLaneletsBecomeLaneletInstances) {  // NOLINT
  const std::vector<std::pair<std::string, bool>> cases{
      {AttributeValueString::Road, true},           {AttributeValueString::BicycleLane, true},
      {AttributeValueString::Crosswalk, false},     {AttributeValueString::Walkway, false},
      {AttributeValueString::SharedWalkway, false}, {AttributeValueString::Stairs, false}};

  for (const auto& testCase : cases) {
    LaneletMapPtr map = makeSingleLaneletMap(testCase.first);

    MapDataInterface::Configuration config{};
    config.paramType = ParametrizationType::LineString;
    config.submapExtentLongitudinal = 50;
    config.submapExtentLateral = 50;
    config.nPointsLanes = 10;
    config.nPointsTE = 10;

    MapDataInterface parser(map, config);
    parser.setCurrPosAndExtractSubmap2d(BasicPoint2d(1.5, 5), 0.0);
    MapDataPtr mapData = parser.mapData(true);

    EXPECT_EQ(!mapData->laneletInstances().empty(), testCase.second) << "subtype " << testCase.first;
  }
}

/// Which subtypes become lanelet instances is covered exhaustively by OnlyDrivingLaneletsBecomeLaneletInstances.
/// What only the shared test map can show is that excluding them does not lose the crossings themselves, and that
/// it leaves every remaining instance processable.
TEST_F(MLConverterTest, CrosswalksAreRepresentedAsPerimetersOnly) {  // NOLINT
  traffic_rules::TrafficRulesPtr trafficRules{
      traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Vehicle)};
  traffic_rules::TrafficRulesPtr bikeTrafficRules{
      traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Bicycle)};
  routing::RoutingGraphConstPtr laneletMapGraph = routing::RoutingGraph::build(*laneletMap, *trafficRules);
  routing::RoutingGraphConstPtr bikeMapGraph = routing::RoutingGraph::build(*laneletMap, *bikeTrafficRules);

  LaneletSubmapConstPtr laneletSubmap = wholeTestMapSubmap(laneletMap);
  MapDataPtr mapData = MapData::build(laneletSubmap, laneletMapGraph, trafficRules, bikeMapGraph);

  // ll2018 and ll2019 are crosswalks: they are covered by their perimeter instances instead
  EXPECT_TRUE(mapData->laneletInstances().find(2018) == mapData->laneletInstances().end());
  EXPECT_TRUE(mapData->laneletInstances().find(2019) == mapData->laneletInstances().end());
  EXPECT_EQ(mapData->compoundLineStringsOfType(LineStringType::ZebraCrossing).size(), 2u);

  // ll2019 stores its two bounds in opposite directions, so its centerline collapses to a doubled point. As a
  // lanelet instance that is invalid and drags the aggregate result down, which is what makes this more than a
  // restatement of the subtype filter.
  EXPECT_TRUE(mapData->processAll(bbox, ParametrizationType::LineString, true, 20, false, 10, 0.0, 0.0));
}

TEST_F(MLConverterTest, LaneletInstanceBoundariesKeepLaneletOrientation) {  // NOLINT
  traffic_rules::TrafficRulesPtr trafficRules{
      traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Vehicle)};
  traffic_rules::TrafficRulesPtr bikeTrafficRules{
      traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Bicycle)};
  routing::RoutingGraphConstPtr laneletMapGraph = routing::RoutingGraph::build(*laneletMap, *trafficRules);
  routing::RoutingGraphConstPtr bikeMapGraph = routing::RoutingGraph::build(*laneletMap, *bikeTrafficRules);

  LaneletSubmapConstPtr laneletSubmap = wholeTestMapSubmap(laneletMap);
  MapDataPtr mapData = MapData::build(laneletSubmap, laneletMapGraph, trafficRules, bikeMapGraph);

  // ll2007 is built from l1012.invert() and l1013, i.e. the two bounds have different inverted() flags
  LaneletInstances::const_iterator it = mapData->laneletInstances().find(2007);
  ASSERT_TRUE(it != mapData->laneletInstances().end());

  const BasicLineString3d& left = it->second->leftBoundary()->rawInstance();
  const BasicLineString3d& right = it->second->rightBoundary()->rawInstance();
  ASSERT_GE(left.size(), 2u);
  ASSERT_GE(right.size(), 2u);

  // both bounds must run in the lanelet's own direction (from y = -7 towards y = -5)
  EXPECT_NEAR(left.front().x(), 7.0, 1e-9);
  EXPECT_NEAR(left.front().y(), -7.0, 1e-9);
  EXPECT_NEAR(left.back().y(), -5.0, 1e-9);
  EXPECT_NEAR(right.front().x(), 9.0, 1e-9);
  EXPECT_NEAR(right.front().y(), -7.0, 1e-9);
  EXPECT_NEAR(right.back().y(), -5.0, 1e-9);

  // the invariant behind those numbers: left and right may not point in opposite directions
  BasicPoint3d leftDir = left.back() - left.front();
  BasicPoint3d rightDir = right.back() - right.front();
  EXPECT_GT(leftDir.dot(rightDir), 0.0);
}

TEST(MLConverterGeometry, ClippedLineStringsKeepTheirElevation) {  // NOLINT
  // from2dPos = false, so the output keeps its z instead of being forced to 2d
  OrientedRect bbox = getRotatedRect(BasicPoint3d{0, 0, 0}, 15, 10, 0, false);

  {  // constant, non-zero elevation
    BasicLineString3d line{BasicPoint3d{0, 0, 5}, BasicPoint3d{10, 0, 5}, BasicPoint3d{20, 0, 5}};
    LaneLineStringInstance feat(line, Id(1), LineStringType::Solid, Ids{1}, false);
    ASSERT_TRUE(feat.process(bbox, ParametrizationType::LineString, -1));
    EXPECT_TRUE(feat.wasCut());
    std::vector<MatrixXd> mats = feat.pointMatrices(false);
    ASSERT_EQ(mats.size(), 1u);
    ASSERT_EQ(mats[0].cols(), 3);
    for (int i = 0; i < mats[0].rows(); i++) {
      EXPECT_NEAR(mats[0](i, 2), 5.0, 1e-9) << "row " << i;
    }
  }

  {  // an elevation of zero is a valid elevation and has to be preserved as such
    BasicLineString3d line{BasicPoint3d{0, 0, 0}, BasicPoint3d{10, 0, 0}, BasicPoint3d{20, 0, 0}};
    LaneLineStringInstance feat(line, Id(2), LineStringType::Solid, Ids{1}, false);
    ASSERT_TRUE(feat.process(bbox, ParametrizationType::LineString, -1));
    std::vector<MatrixXd> mats = feat.pointMatrices(false);
    ASSERT_EQ(mats.size(), 1u);
    ASSERT_EQ(mats[0].cols(), 3);
    for (int i = 0; i < mats[0].rows(); i++) {
      EXPECT_NEAR(mats[0](i, 2), 0.0, 1e-9) << "row " << i;
    }
  }

  {  // the point introduced by clipping takes the elevation of the nearest original vertex
    BasicLineString3d line{BasicPoint3d{0, 0, 0}, BasicPoint3d{20, 0, 20}};
    LaneLineStringInstance feat(line, Id(3), LineStringType::Solid, Ids{1}, false);
    ASSERT_TRUE(feat.process(bbox, ParametrizationType::LineString, -1));
    const BasicLineStrings3d& cut = feat.cutInstance();
    ASSERT_EQ(cut.size(), 1u);
    EXPECT_NEAR(cut.front().back().x(), 15.0, 1e-6);
    EXPECT_NEAR(cut.front().back().z(), 20.0, 1e-6);
  }
}

TEST_F(MLConverterTest, IgnoreMapElevationYieldsExactlyZeroZ) {  // NOLINT
  MapDataInterface::Configuration config{};
  config.paramType = ParametrizationType::LineString;
  config.submapExtentLongitudinal = 5;
  config.submapExtentLateral = 3;
  config.ignoreMapElevation = true;
  config.nPointsLanes = 10;
  config.nPointsTE = 10;

  MapDataInterface parser(laneletMap, config);
  // the 3d overload keeps from2d unset, so the output really is 3-dimensional
  parser.setCurrPosAndExtractSubmap(BasicPoint3d(3, -3, 0), 0.0);
  MapDataPtr mapData = parser.mapData(true);

  MapData::TensorInstanceData tfData = mapData->getTensorInstanceData(false, false);
  size_t checkedInstances = 0;
  for (int type = 0; type <= static_cast<int>(LineStringType::ZebraCrossing); type++) {
    for (const auto& mat : tfData.compoundLineStringsOfType(static_cast<LineStringType>(type))) {
      ASSERT_EQ(mat.cols(), 3);
      for (int i = 0; i < mat.rows(); i++) {
        EXPECT_NEAR(mat(i, 2), 0.0, 1e-9) << "type " << type << " row " << i;
      }
      checkedInstances++;
    }
  }
  EXPECT_GT(checkedInstances, 0u);
}

TEST(MLConverterGeometry, VerticalTrafficElementClipping) {  // NOLINT
  // traffic lights and signs are vertical linestrings: their 2d projection is a single point, so they can
  // only be kept or dropped as a whole
  BasicLineString3d trafficLight{BasicPoint3d{5, -3.5, 3}, BasicPoint3d{5, -3.5, 4}};

  OrientedRect bbox = getRotatedRect(BasicPoint3d{5, -3.5, 0}, 10, 10, 0, false);
  TEInstance inside(trafficLight, Id(1), TEType::TLCar);
  ASSERT_TRUE(inside.process(bbox, ParametrizationType::LineString, -1));
  EXPECT_TRUE(inside.valid());
  std::vector<MatrixXd> mats = inside.pointMatrices(false);
  ASSERT_EQ(mats.size(), 1u);
  ASSERT_EQ(mats[0].rows(), 2);
  ASSERT_EQ(mats[0].cols(), 3);
  // the vertical extent has to survive, otherwise the element collapses and is dropped as too short
  EXPECT_NEAR(std::abs(mats[0](1, 2) - mats[0](0, 2)), 1.0, 1e-9);

  OrientedRect farBbox = getRotatedRect(BasicPoint3d{100, 100, 0}, 10, 10, 0, false);
  TEInstance outside(trafficLight, Id(2), TEType::TLCar);
  EXPECT_FALSE(outside.process(farBbox, ParametrizationType::LineString, -1));
  EXPECT_FALSE(outside.valid());
}
