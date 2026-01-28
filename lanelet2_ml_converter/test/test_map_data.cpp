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

  // std::cerr << std::endl << "--- IN MAP DATA TEST ---" << std::endl;

  traffic_rules::TrafficRulesPtr trafficRules{
      traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Vehicle)};
  traffic_rules::TrafficRulesPtr bikeTrafficRules{
      traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Bicycle)};
  routing::RoutingGraphConstPtr laneletMapGraph = routing::RoutingGraph::build(*laneletMap, *trafficRules);
  routing::RoutingGraphConstPtr bikeMapGraph = routing::RoutingGraph::build(*laneletMap, *bikeTrafficRules);

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

  MapDataPtr mapData = MapData::build(laneletSubmap, laneletMapGraph, trafficRules, bikeMapGraph);

  bool valid = mapData->processAll(bbox, ParametrizationType::LineString, true, 20, false, 10, 0.0, 0.0);
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

  EXPECT_EQ(compoundRoadBorders.size(), 4);    // 3 vehicle + 1 bike (3 bike borders combined)
  EXPECT_EQ(compoundLaneDividers.size(), 11);  // 8 vehicle + 3 bike (1 solid + 2 dashed)
  EXPECT_EQ(compoundCenterlines.size(), 4);    // vehicle centerlines only
  EXPECT_EQ(drivableArea.size(), 5);
  EXPECT_EQ(compoundDrivableArea.size(), 2);

  EXPECT_EQ(mapData->associatedCpdLineStringsOfType(1021, LineStringType::DrivableArea).size(), 1);
  auto assoDrivableAreaList = mapData->associatedCpdLineStringsOfType(1021, LineStringType::DrivableArea);
  CompoundLaneLineStringInstancePtr assoDrivableArea = assoDrivableAreaList.front();

  // for (const auto& el : mapData->validCompoundLineStringsOfType(LineStringType::DrivableArea)) {
  //   for (const auto& feat : el->features()) {
  //     std::cerr << feat->mapID() << " ";
  //   }
  //   std::cerr << std::endl << "---" << std::endl;
  // }
  // std::cerr << assoDrivableArea->features().back()->mapID() << std::endl;

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

  // Test bike centerlines
  std::vector<Eigen::MatrixXd> compoundBikeCenterlines =
      mapData->getTensorInstanceData(true, false).compoundLineStringsOfType(LineStringType::BikeCenterline);
  EXPECT_EQ(compoundBikeCenterlines.size(), 2);  // two bike centerline paths

  EXPECT_GE(mapData->associatedCpdLineStringsOfType(2012, LineStringType::BikeCenterline).size(), 1);
  CompoundLaneLineStringInstancePtr assoBikeCenterline =
      mapData->associatedCpdLineStringsOfType(2012, LineStringType::BikeCenterline).front();
  EXPECT_EQ(assoBikeCenterline->features().front()->mapID(), 2012);
  EXPECT_EQ(assoBikeCenterline->features().front()->laneletIDs().front(), 2012);

  // Verify bike centerlines are separate from vehicle centerlines
  EXPECT_EQ(mapData->associatedCpdLineStringsOfType(2012, LineStringType::Centerline).size(), 0);
  EXPECT_EQ(mapData->associatedCpdLineStringsOfType(2000, LineStringType::BikeCenterline).size(), 0);

  // Plotting is now done in MapDataTrafficElements test
}

TEST_F(MLConverterTest, MapDataTrafficElements) {  // NOLINT
  traffic_rules::TrafficRulesPtr trafficRules{
      traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Vehicle)};
  traffic_rules::TrafficRulesPtr bikeTrafficRules{
      traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Bicycle)};
  routing::RoutingGraphConstPtr laneletMapGraph = routing::RoutingGraph::build(*laneletMap, *trafficRules);
  routing::RoutingGraphConstPtr bikeMapGraph = routing::RoutingGraph::build(*laneletMap, *bikeTrafficRules);

  // Cast to non-const to use non-const search
  auto nonConstMap = std::const_pointer_cast<LaneletMap>(laneletMap);
  Lanelets allLanelets(nonConstMap->laneletLayer.begin(), nonConstMap->laneletLayer.end());

  // Create submap from lanelets
  auto submapUPtr = utils::createSubmap(allLanelets, {});

  // Add all linestrings from the map to the submap (including traffic elements)
  LineStrings3d allLineStrings(nonConstMap->lineStringLayer.begin(), nonConstMap->lineStringLayer.end());
  for (const auto& lineString : allLineStrings) {
    submapUPtr->add(lineString);
  }

  // Add all regulatory elements to the submap
  for (const auto& regElem : nonConstMap->regulatoryElementLayer) {
    submapUPtr->add(regElem);
  }

  // Convert unique_ptr to const shared_ptr
  LaneletSubmapConstPtr laneletSubmap = std::shared_ptr<const LaneletSubmap>(std::move(submapUPtr));

  MapDataPtr mapData = MapData::build(laneletSubmap, laneletMapGraph, trafficRules, bikeMapGraph);

  bool valid = mapData->processAll(bbox, ParametrizationType::LineString, true, 20, false, 10, 0.0, 0.0);
  EXPECT_TRUE(valid);

  // Test that traffic elements were collected
  auto stopLines = mapData->teInstancesOfType(TEType::StopLine);
  EXPECT_EQ(stopLines.size(), 1);
  EXPECT_TRUE(stopLines.find(1030) != stopLines.end());

  auto trafficLights = mapData->teInstancesOfType(TEType::TLCar);
  EXPECT_EQ(trafficLights.size(), 1);
  EXPECT_TRUE(trafficLights.find(1031) != trafficLights.end());

  auto straightArrows = mapData->teInstancesOfType(TEType::ArrowGoStraight);
  EXPECT_EQ(straightArrows.size(), 1);
  EXPECT_TRUE(straightArrows.find(1032) != straightArrows.end());

  auto straightOrRightArrows = mapData->teInstancesOfType(TEType::ArrowGoStraightOrRight);
  EXPECT_EQ(straightOrRightArrows.size(), 1);
  EXPECT_TRUE(straightOrRightArrows.find(1034) != straightOrRightArrows.end());

  auto symbols30 = mapData->teInstancesOfType(TEType::Symbol30);
  EXPECT_EQ(symbols30.size(), 1);
  EXPECT_TRUE(symbols30.find(1033) != symbols30.end());

  // Test TensorInstanceData
  auto tfData = mapData->getTensorInstanceData(true, false);

  std::vector<Eigen::MatrixXd> stopLineMatrices = tfData.teInstancesOfType(TEType::StopLine);
  EXPECT_EQ(stopLineMatrices.size(), 1);
  EXPECT_GT(stopLineMatrices[0].rows(), 0);
  EXPECT_EQ(stopLineMatrices[0].cols(), 2);  // 2D points

  std::vector<Eigen::MatrixXd> trafficLightMatrices = tfData.teInstancesOfType(TEType::TLCar);
  EXPECT_EQ(trafficLightMatrices.size(), 1);

  std::vector<Eigen::MatrixXd> arrowMatrices = tfData.teInstancesOfType(TEType::ArrowGoStraight);
  EXPECT_EQ(arrowMatrices.size(), 1);

  // Test edges: Traffic light to stop line (TE to TE)
  auto teToTEEdges = tfData.teToTEIndexEdges();
  EXPECT_GT(teToTEEdges.size(), 0);

  bool foundTLToStopLine = false;
  for (const auto& edge : teToTEEdges) {
    if (std::get<0>(edge) == TEType::TLCar && std::get<2>(edge) == TEType::StopLine) {
      foundTLToStopLine = true;
      break;
    }
  }
  EXPECT_TRUE(foundTLToStopLine);

  // Test edges: Stop line to centerline (TE to centerline)
  auto teToCenterlineEdges = tfData.teToCenterlineIndexEdges();
  EXPECT_GT(teToCenterlineEdges.size(), 0);

  bool foundStopLineToCenterline = false;
  for (const auto& edge : teToCenterlineEdges) {
    if (std::get<0>(edge) == TEType::StopLine) {
      foundStopLineToCenterline = true;
      break;
    }
  }
  EXPECT_TRUE(foundStopLineToCenterline);

  // Test edges: Arrows to centerline
  bool foundArrowToCenterline = false;
  for (const auto& edge : teToCenterlineEdges) {
    if (std::get<0>(edge) == TEType::ArrowGoStraight || std::get<0>(edge) == TEType::ArrowTurnLeft) {
      foundArrowToCenterline = true;
      break;
    }
  }
  EXPECT_TRUE(foundArrowToCenterline);

  // // Plot all map elements and traffic elements in one comprehensive image
  // matplot::figure(true);
  // matplot::hold(matplot::on);
  // matplot::xlim({-16, 0});
  // matplot::ylim({-10, 6});
  // matplot::gcf()->size(1200, 1200);
  // matplot::title("Complete Map with Traffic Elements and Bike Lanes");

  // // Plot road borders in red
  // std::vector<Eigen::MatrixXd> compoundRoadBorders = tfData.compoundLineStringsOfType(LineStringType::RoadBorder);
  // for (const auto& mat : compoundRoadBorders) {
  //   std::vector<double> x;
  //   std::vector<double> y;
  //   x.resize(mat.rows());
  //   y.resize(mat.rows());
  //   Eigen::VectorXd::Map(&x[0], mat.rows()) = mat.col(0);
  //   Eigen::VectorXd::Map(&y[0], mat.rows()) = mat.col(1);
  //   matplot::plot(x, y, "r")->line_width(3);
  // }

  // // Plot dashed lane dividers
  // std::vector<Eigen::MatrixXd> compoundDashed = tfData.compoundLineStringsOfType(LineStringType::Dashed);
  // for (const auto& mat : compoundDashed) {
  //   std::vector<double> x;
  //   std::vector<double> y;
  //   x.resize(mat.rows());
  //   y.resize(mat.rows());
  //   Eigen::VectorXd::Map(&x[0], mat.rows()) = mat.col(0);
  //   Eigen::VectorXd::Map(&y[0], mat.rows()) = mat.col(1);
  //   matplot::plot(x, y, "--bo")->line_width(3);
  // }

  // // Plot solid lane dividers
  // std::vector<Eigen::MatrixXd> compoundSolid = tfData.compoundLineStringsOfType(LineStringType::Solid);
  // for (const auto& mat : compoundSolid) {
  //   std::vector<double> x;
  //   std::vector<double> y;
  //   x.resize(mat.rows());
  //   y.resize(mat.rows());
  //   Eigen::VectorXd::Map(&x[0], mat.rows()) = mat.col(0);
  //   Eigen::VectorXd::Map(&y[0], mat.rows()) = mat.col(1);
  //   matplot::plot(x, y, "b")->line_width(3);
  // }

  // // Plot virtual lane dividers
  // std::vector<Eigen::MatrixXd> compoundVirtual = tfData.compoundLineStringsOfType(LineStringType::Virtual);
  // for (const auto& mat : compoundVirtual) {
  //   std::vector<double> x;
  //   std::vector<double> y;
  //   x.resize(mat.rows());
  //   y.resize(mat.rows());
  //   Eigen::VectorXd::Map(&x[0], mat.rows()) = mat.col(0);
  //   Eigen::VectorXd::Map(&y[0], mat.rows()) = mat.col(1);
  //   matplot::plot(x, y, "m")->line_width(3);
  // }

  // // Plot centerlines
  // std::vector<Eigen::MatrixXd> compoundCenterlines = tfData.compoundLineStringsOfType(LineStringType::Centerline);
  // for (const auto& mat : compoundCenterlines) {
  //   std::vector<double> x;
  //   std::vector<double> y;
  //   x.resize(mat.rows());
  //   y.resize(mat.rows());
  //   Eigen::VectorXd::Map(&x[0], mat.rows()) = mat.col(0);
  //   Eigen::VectorXd::Map(&y[0], mat.rows()) = mat.col(1);
  //   matplot::plot(x, y, "--gs")->line_width(3);
  // }

  // // Plot bike centerlines in orange
  // std::vector<Eigen::MatrixXd> compoundBikeCenterlines =
  //     tfData.compoundLineStringsOfType(LineStringType::BikeCenterline);
  // for (const auto& mat : compoundBikeCenterlines) {
  //   std::vector<double> x;
  //   std::vector<double> y;
  //   x.resize(mat.rows());
  //   y.resize(mat.rows());
  //   Eigen::VectorXd::Map(&x[0], mat.rows()) = mat.col(0);
  //   Eigen::VectorXd::Map(&y[0], mat.rows()) = mat.col(1);
  //   matplot::plot(x, y, "--o")->line_width(3).color("orange");
  // }

  // // Plot stop lines in dark red with thick line and circles
  // // std::cerr << "Stop Lines:" << std::endl;

  // for (const auto& mat : stopLineMatrices) {
  //   std::vector<double> x;
  //   std::vector<double> y;
  //   x.resize(mat.rows());
  //   y.resize(mat.rows());
  //   Eigen::VectorXd::Map(&x[0], mat.rows()) = mat.col(0);
  //   Eigen::VectorXd::Map(&y[0], mat.rows()) = mat.col(1);

  //   //   for (const auto& el : x) {
  //   //     std::cerr << el << " ";
  //   //   }
  //   //   std::cerr << std::endl;
  //   //   for (const auto& el : y) {
  //   //     std::cerr << el << " ";
  //   //   }
  //   //   std::cerr << std::endl << "---" << std::endl;

  //   matplot::plot(x, y, "-o")->line_width(5).color({0.6, 0, 0}).marker_size(10);
  // }

  // // // Plot traffic lights in yellow with square markers
  // // std::cerr << "Traffic Lights:" << std::endl;

  // for (const auto& mat : trafficLightMatrices) {
  //   std::vector<double> x;
  //   std::vector<double> y;
  //   x.resize(mat.rows());
  //   y.resize(mat.rows());
  //   Eigen::VectorXd::Map(&x[0], mat.rows()) = mat.col(0);
  //   Eigen::VectorXd::Map(&y[0], mat.rows()) = mat.col(1);

  //   //   for (const auto& el : x) {
  //   //     std::cerr << el << " ";
  //   //   }
  //   //   std::cerr << std::endl;
  //   //   for (const auto& el : y) {
  //   //     std::cerr << el << " ";
  //   //   }
  //   //   std::cerr << std::endl << "---" << std::endl;

  //   matplot::plot(x, y, "-s")->line_width(5).color("yellow").marker_size(12);
  // }

  // // Plot straight arrows in green with triangle markers
  // for (const auto& mat : arrowMatrices) {
  //   std::vector<double> x;
  //   std::vector<double> y;
  //   x.resize(mat.rows());
  //   y.resize(mat.rows());
  //   Eigen::VectorXd::Map(&x[0], mat.rows()) = mat.col(0);
  //   Eigen::VectorXd::Map(&y[0], mat.rows()) = mat.col(1);
  //   matplot::plot(x, y, "-^")->line_width(5).color("green").marker_size(12);
  // }

  // // Plot straight-or-right arrows in cyan
  // std::vector<Eigen::MatrixXd> straightOrRightArrowMatrices =
  // tfData.teInstancesOfType(TEType::ArrowGoStraightOrRight); for (const auto& mat : straightOrRightArrowMatrices) {
  //   std::vector<double> x;
  //   std::vector<double> y;
  //   x.resize(mat.rows());
  //   y.resize(mat.rows());
  //   Eigen::VectorXd::Map(&x[0], mat.rows()) = mat.col(0);
  //   Eigen::VectorXd::Map(&y[0], mat.rows()) = mat.col(1);
  //   matplot::plot(x, y, "-v")->line_width(5).color("cyan").marker_size(12);
  // }

  // // Plot speed limit symbols in magenta with diamond markers
  // std::vector<Eigen::MatrixXd> symbolMatrices = tfData.teInstancesOfType(TEType::Symbol30);
  // for (const auto& mat : symbolMatrices) {
  //   std::vector<double> x;
  //   std::vector<double> y;
  //   x.resize(mat.rows());
  //   y.resize(mat.rows());
  //   Eigen::VectorXd::Map(&x[0], mat.rows()) = mat.col(0);
  //   Eigen::VectorXd::Map(&y[0], mat.rows()) = mat.col(1);
  //   matplot::plot(x, y, "-d")->line_width(5).color("magenta").marker_size(12);
  // }

  // // Plot TE to TE edges (e.g., traffic light to stop line)
  // for (const auto& edge : teToTEEdges) {
  //   TEType sourceType = std::get<0>(edge);
  //   size_t sourceIdx = std::get<1>(edge);
  //   TEType targetType = std::get<2>(edge);
  //   size_t targetIdx = std::get<3>(edge);

  //   auto sourceMatrices = tfData.teInstancesOfType(sourceType);
  //   auto targetMatrices = tfData.teInstancesOfType(targetType);

  //   if (sourceIdx < sourceMatrices.size() && targetIdx < targetMatrices.size()) {
  //     const auto& sourceMat = sourceMatrices[sourceIdx];
  //     const auto& targetMat = targetMatrices[targetIdx];

  //     // Use center point of each TE for the arrow
  //     double sourceX = sourceMat.col(0).mean();
  //     double sourceY = sourceMat.col(1).mean();
  //     double targetX = targetMat.col(0).mean();
  //     double targetY = targetMat.col(1).mean();

  //     matplot::arrow(sourceX, sourceY, targetX, targetY)->color({1.0, 0.5, 0.0}).line_width(3);
  //   }
  // }

  // // Plot TE to centerline edges (e.g., stop line to lanelet, arrows to lanelet)
  // for (const auto& edge : teToCenterlineEdges) {
  //   TEType sourceType = std::get<0>(edge);
  //   size_t sourceIdx = std::get<1>(edge);
  //   size_t centerlineIdx = std::get<2>(edge);

  //   auto sourceMatrices = tfData.teInstancesOfType(sourceType);

  //   if (sourceIdx < sourceMatrices.size() && centerlineIdx < compoundCenterlines.size()) {
  //     const auto& sourceMat = sourceMatrices[sourceIdx];
  //     const auto& centerlineMat = compoundCenterlines[centerlineIdx];

  //     // Use center point of TE
  //     double sourceX = sourceMat.col(0).mean();
  //     double sourceY = sourceMat.col(1).mean();

  //     double targetX = centerlineMat(10, 0);
  //     double targetY = centerlineMat(10, 1);

  //     matplot::arrow(sourceX, sourceY, targetX, targetY)->color({0.0, 0.8, 0.8}).line_width(3);
  //   }
  // }

  // // Plot drivable area borders with arrows
  // std::vector<Eigen::MatrixXd> compoundDrivableArea = tfData.compoundLineStringsOfType(LineStringType::DrivableArea);
  // for (const auto& mat : compoundDrivableArea) {
  //   for (int i = 0; i < mat.rows() - 1; ++i) {
  //     matplot::arrow(mat(i, 0), mat(i, 1), mat(i + 1, 0), mat(i + 1, 1))->color("orange").line_width(4);
  //   }
  // }

  // matplot::save("map_data_complete.png");
}