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

TEST_F(MLConverterTest, LaneData) {  // NOLINT
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

  LaneDataPtr laneData = LaneData::build(laneletSubmap, laneletMapGraph, trafficRules);

  bool valid = laneData->processAll(bbox, ParametrizationType::LineString, 20);
  std::vector<Eigen::MatrixXd> compoundRoadBorders = getPointMatrices(laneData->compoundRoadBorders(), true);
  std::vector<Eigen::MatrixXd> compoundLaneDividers = getPointMatrices(laneData->compoundLaneDividers(), true);
  std::vector<Eigen::MatrixXd> compoundCenterlines = getPointMatrices(laneData->compoundCenterlines(), true);
  std::vector<Eigen::MatrixXd> drivableArea =
      getPointMatrices(laneData->lineStringsOfType(LineStringType::DrivableArea), true);
  std::vector<Eigen::MatrixXd> compoundDrivableArea =
      getPointMatrices(laneData->compoundLineStringsOfType(LineStringType::DrivableArea), true);

  EXPECT_TRUE(laneData->laneletInstances().find(2007) != laneData->laneletInstances().end());
  EXPECT_EQ(laneData->laneletInstances().find(2007)->second->leftBoundary()->mapID(), 1012);
  EXPECT_TRUE(laneData->roadBorders().find(1001) != laneData->roadBorders().end());

  EXPECT_EQ(compoundRoadBorders.size(), 3);
  EXPECT_EQ(compoundLaneDividers.size(), 8);
  EXPECT_EQ(compoundCenterlines.size(), 4);
  EXPECT_EQ(drivableArea.size(), 5);
  EXPECT_EQ(compoundDrivableArea.size(), 2);

  EXPECT_EQ(laneData->associatedCpdLineStringsOfType(1021, LineStringType::DrivableArea).size(), 1);
  auto assoDrivableAreaList = laneData->associatedCpdLineStringsOfType(1021, LineStringType::DrivableArea);
  CompoundLaneLineStringInstancePtr assoDrivableArea = assoDrivableAreaList.front();
  std::cerr << assoDrivableArea->features().back()->mapID() << std::endl;
  EXPECT_TRUE(assoDrivableArea->features().back()->mapID() == 1021 ||
              assoDrivableArea->features().back()->mapID() == 1023 ||
              assoDrivableArea->features().back()->mapID() == 1022);

  EXPECT_EQ(laneData->associatedCpdRoadBorders(2001).size(), 1);
  CompoundLaneLineStringInstancePtr assoBorder = laneData->associatedCpdRoadBorders(2001).front();
  EXPECT_EQ(assoBorder->features().back()->mapID(), 1000);
  EXPECT_EQ(assoBorder->features().back()->laneletIDs().front(), 2002);

  EXPECT_EQ(laneData->associatedCpdLaneDividers(2004).size(), 2);
  EXPECT_EQ(laneData->associatedCpdLaneDividers(2009).size(), 1);
  CompoundLaneLineStringInstancePtr assoDivider = laneData->associatedCpdLaneDividers(2009).front();
  EXPECT_EQ(assoDivider->features().front()->mapID(), 1015);
  EXPECT_EQ(assoDivider->features().front()->laneletIDs().size(), 2);

  EXPECT_EQ(laneData->associatedCpdCenterlines(2003).size(), 2);
  EXPECT_EQ(laneData->associatedCpdCenterlines(2000).size(), 1);

  // for (auto i : getPointMatrices(laneData->associatedCpdCenterlines(2003), true))
  // std::cerr << i << std::endl << "---" << std::endl;
  // std::cerr << "------------------------" << std::endl;
  // for (auto i : getPointMatrices(laneData->associatedCpdCenterlines(2000), true))
  // std::cerr << i << std::endl << "---" << std::endl;
  // std::cerr << "------------------------" << std::endl;

  CompoundLaneLineStringInstancePtr assoCenterline = laneData->associatedCpdCenterlines(2000).front();
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
  // for (const auto& mat : getPointMatrices(laneData->compoundLineStringsOfType(LineStringType::Dashed), true)) {
  //   std::vector<double> x;
  //   std::vector<double> y;
  //   x.resize(mat.rows());
  //   y.resize(mat.rows());
  //   Eigen::VectorXd::Map(&x[0], mat.rows()) = mat.col(0);
  //   Eigen::VectorXd::Map(&y[0], mat.rows()) = mat.col(1);
  //   matplot::plot(x, y, "--bo")->line_width(3);
  // }
  // for (const auto& mat : getPointMatrices(laneData->compoundLineStringsOfType(LineStringType::Solid), true)) {
  //   std::vector<double> x;
  //   std::vector<double> y;
  //   x.resize(mat.rows());
  //   y.resize(mat.rows());
  //   Eigen::VectorXd::Map(&x[0], mat.rows()) = mat.col(0);
  //   Eigen::VectorXd::Map(&y[0], mat.rows()) = mat.col(1);
  //   matplot::plot(x, y, "b")->line_width(3);
  // }
  // for (const auto& mat : getPointMatrices(laneData->compoundLineStringsOfType(LineStringType::Virtual), true)) {
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
  // for (const auto& mat : getPointMatrices(laneData->compoundLineStringsOfType(LineStringType::DrivableArea), true)) {
  //   // Draw arrows between consecutive points
  //   for (int i = 0; i < mat.rows() - 1; ++i) {
  //     matplot::arrow(mat(i, 0), mat(i, 1), mat(i + 1, 0), mat(i + 1, 1))->color("r").line_width(2);
  //     // std::cerr << mat(i, 0) << mat(i, 1) << std::endl;
  //   }
  //   // std::cerr << "---" << std::endl;
  // }
  // matplot::save("map_data_processed.png");
}