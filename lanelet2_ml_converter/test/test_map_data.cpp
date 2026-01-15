#include <gtest/gtest.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <matplot/matplot.h>

#include "lanelet2_ml_converter/MapData.h"
#include "lanelet2_ml_converter/MapInstances.h"
#include "lanelet2_routing/RoutingGraph.h"
#include "test_map.h"

using namespace lanelet;
using namespace lanelet::ml_converter;
using namespace lanelet::ml_converter::tests;

TEST_F(MLConverterTest, MapData) {  // NOLINT
  traffic_rules::TrafficRulesPtr trafficRules{
      traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Vehicle)};
  routing::RoutingGraphConstPtr laneletMapGraph = routing::RoutingGraph::build(*laneletMap, *trafficRules);

  // Cast to non-const to use non-const search
  auto nonConstMap = std::const_pointer_cast<LaneletMap>(laneletMap);
  Lanelets allLanelets(nonConstMap->laneletLayer.begin(), nonConstMap->laneletLayer.end());

  // Create submap from lanelets
  auto submapUPtr = utils::createSubmap(allLanelets, {});

  // Add all linestrings from the map to the submap
  LineStrings3d allLineStrings(nonConstMap->lineStringLayer.begin(), nonConstMap->lineStringLayer.end());
  for (const auto& lineString : allLineStrings) {
    submapUPtr->add(lineString);
  }

  // Convert unique_ptr to const shared_ptr
  LaneletSubmapConstPtr laneletSubmap = std::shared_ptr<const LaneletSubmap>(std::move(submapUPtr));

  MapDataPtr mapData = MapData::build(laneletSubmap, laneletMapGraph, trafficRules);

  bool valid = mapData->processAll(bbox, ParametrizationType::LineString, 20);
  std::vector<Eigen::MatrixXd> compoundRoadBorders =
      mapData->getTensorInstanceData(true, false).compoundLineStringsOfType(LineStringType::RoadBorder);
  std::vector<Eigen::MatrixXd> compoundLaneDividers =
      mapData->getTensorInstanceData(true, false).compoundLineStringsOfType(LineStringType::Dashed);
  for (const auto& mat : mapData->getTensorInstanceData(true, false).compoundLineStringsOfType(LineStringType::Solid)) {
    compoundLaneDividers.push_back(mat);
  }
  for (const auto& mat :
       mapData->getTensorInstanceData(true, false).compoundLineStringsOfType(LineStringType::Virtual)) {
    compoundLaneDividers.push_back(mat);
  }
  std::vector<Eigen::MatrixXd> compoundCenterlines =
      mapData->getTensorInstanceData(true, false).compoundLineStringsOfType(LineStringType::Centerline);
  std::vector<Eigen::MatrixXd> drivableArea =
      mapData->getTensorInstanceData(true, false).lineStringsOfType(LineStringType::DrivableArea);
  std::vector<Eigen::MatrixXd> compoundDrivableArea =
      mapData->getTensorInstanceData(true, false).compoundLineStringsOfType(LineStringType::DrivableArea);

  EXPECT_TRUE(mapData->laneletInstances().find(2007) != mapData->laneletInstances().end());
  EXPECT_EQ(mapData->laneletInstances().find(2007)->second->leftBoundary()->mapID(), 1012);
  EXPECT_TRUE(mapData->lineStringsOfType(LineStringType::RoadBorder).find(1001) !=
              mapData->lineStringsOfType(LineStringType::RoadBorder).end());

  EXPECT_EQ(compoundRoadBorders.size(), 3);
  EXPECT_EQ(compoundLaneDividers.size(), 8);
  EXPECT_EQ(compoundCenterlines.size(), 4);
  EXPECT_EQ(drivableArea.size(), 5);
  EXPECT_EQ(compoundDrivableArea.size(), 2);

  EXPECT_EQ(mapData->associatedCpdLineStringsOfType(1021, LineStringType::DrivableArea).size(), 1);
  auto assoDrivableAreaList = mapData->associatedCpdLineStringsOfType(1021, LineStringType::DrivableArea);
  CompoundLaneLineStringInstancePtr assoDrivableArea = assoDrivableAreaList.front();
  // std::cerr << assoDrivableArea->features().back()->mapID() << std::endl;
  EXPECT_TRUE(assoDrivableArea->features().back()->mapID() == 1021 ||
              assoDrivableArea->features().back()->mapID() == 1023 ||
              assoDrivableArea->features().back()->mapID() == 1022);

  EXPECT_EQ(mapData->associatedCpdLineStringsOfType(2001, LineStringType::RoadBorder).size(), 1);
  CompoundLaneLineStringInstancePtr assoBorder =
      mapData->associatedCpdLineStringsOfType(2001, LineStringType::RoadBorder).front();
  EXPECT_EQ(assoBorder->features().back()->mapID(), 1000);
  EXPECT_EQ(assoBorder->features().back()->laneletIDs().front(), 2002);

  std::vector<CompoundLaneLineStringInstancePtr> ld2004 =
      mapData->associatedCpdLineStringsOfType(2004, LineStringType::Dashed);
  for (const auto& mat : mapData->associatedCpdLineStringsOfType(2004, LineStringType::Solid)) {
    ld2004.push_back(mat);
  }
  for (const auto& mat : mapData->associatedCpdLineStringsOfType(2004, LineStringType::Virtual)) {
    ld2004.push_back(mat);
  }
  EXPECT_EQ(ld2004.size(), 2);

  std::vector<CompoundLaneLineStringInstancePtr> ld2009 =
      mapData->associatedCpdLineStringsOfType(2009, LineStringType::Dashed);
  for (const auto& mat : mapData->associatedCpdLineStringsOfType(2009, LineStringType::Solid)) {
    ld2009.push_back(mat);
  }
  for (const auto& mat : mapData->associatedCpdLineStringsOfType(2009, LineStringType::Virtual)) {
    ld2009.push_back(mat);
  }
  EXPECT_EQ(ld2009.size(), 1);
  CompoundLaneLineStringInstancePtr assoDivider = ld2009.front();
  EXPECT_EQ(assoDivider->features().front()->mapID(), 1015);
  EXPECT_EQ(assoDivider->features().front()->laneletIDs().size(), 2);

  EXPECT_EQ(mapData->associatedCpdLineStringsOfType(2003, LineStringType::Centerline).size(), 2);
  EXPECT_EQ(mapData->associatedCpdLineStringsOfType(2000, LineStringType::Centerline).size(), 1);

  CompoundLaneLineStringInstancePtr assoCenterline =
      mapData->associatedCpdLineStringsOfType(2000, LineStringType::Centerline).front();
  EXPECT_EQ(assoCenterline->features().front()->mapID(), 2000);
  EXPECT_EQ(assoCenterline->features().front()->laneletIDs().front(), 2000);

  // matplot::hold(matplot::on);
  // matplot::xlim({-16, -4});
  // matplot::ylim({-10, 6});
  // matplot::gcf()->size(1000, 1000);

  // for (const auto& mat : compoundRoadBorders) {
  //   std::vector<double> x;
  //   std::vector<double> y;
  //   x.resize(mat.rows());
  //   y.resize(mat.rows());
  //   Eigen::VectorXd::Map(&x[0], mat.rows()) = mat.col(0);
  //   Eigen::VectorXd::Map(&y[0], mat.rows()) = mat.col(1);
  //   matplot::plot(x, y, "r")->line_width(3);
  // }
  // for (const auto& mat : getPointMatrices(mapData->compoundLineStringsOfType(LineStringType::Dashed), true)) {
  //   std::vector<double> x;
  //   std::vector<double> y;
  //   x.resize(mat.rows());
  //   y.resize(mat.rows());
  //   Eigen::VectorXd::Map(&x[0], mat.rows()) = mat.col(0);
  //   Eigen::VectorXd::Map(&y[0], mat.rows()) = mat.col(1);
  //   matplot::plot(x, y, "--bo")->line_width(3);
  // }
  // for (const auto& mat : getPointMatrices(mapData->compoundLineStringsOfType(LineStringType::Solid), true)) {
  //   std::vector<double> x;
  //   std::vector<double> y;
  //   x.resize(mat.rows());
  //   y.resize(mat.rows());
  //   Eigen::VectorXd::Map(&x[0], mat.rows()) = mat.col(0);
  //   Eigen::VectorXd::Map(&y[0], mat.rows()) = mat.col(1);
  //   matplot::plot(x, y, "b")->line_width(3);
  // }
  // for (const auto& mat : getPointMatrices(mapData->compoundLineStringsOfType(LineStringType::Virtual), true)) {
  //   std::vector<double> x;
  //   std::vector<double> y;
  //   x.resize(mat.rows());
  //   y.resize(mat.rows());
  //   Eigen::VectorXd::Map(&x[0], mat.rows()) = mat.col(0);
  //   Eigen::VectorXd::Map(&y[0], mat.rows()) = mat.col(1);
  //   matplot::plot(x, y, "m")->line_width(3);
  // }
  // for (const auto& mat : compoundCenterlines) {
  //   std::vector<double> x;
  //   std::vector<double> y;
  //   x.resize(mat.rows());
  //   y.resize(mat.rows());
  //   Eigen::VectorXd::Map(&x[0], mat.rows()) = mat.col(0);
  //   Eigen::VectorXd::Map(&y[0], mat.rows()) = mat.col(1);

  //   // for (auto i : x) std::cerr << i << ' ';
  //   // std::cerr << std::endl;
  //   // for (auto i : y) std::cerr << i << ' ';
  //   // std::cerr << std::endl;

  //   matplot::plot(x, y, "--gs")->line_width(3);
  // }
  // for (const auto& mat : getPointMatrices(mapData->compoundLineStringsOfType(LineStringType::DrivableArea), true)) {
  //   // Draw arrows between consecutive points
  //   for (int i = 0; i < mat.rows() - 1; ++i) {
  //     matplot::arrow(mat(i, 0), mat(i, 1), mat(i + 1, 0), mat(i + 1, 1))->color("r").line_width(2);
  //     // std::cerr << mat(i, 0) << mat(i, 1) << std::endl;
  //   }
  //   // std::cerr << "---" << std::endl;
  // }
  // matplot::save("map_data_processed.png");
}