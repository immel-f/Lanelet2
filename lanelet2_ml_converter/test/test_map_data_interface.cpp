#include <gtest/gtest.h>
// #include <matplot/matplot.h>

#include <chrono>

#include "lanelet2_ml_converter/MapData.h"
#include "lanelet2_ml_converter/MapDataInterface.h"
#include "lanelet2_ml_converter/MapInstances.h"
#include "lanelet2_routing/RoutingGraph.h"
#include "lanelet2_traffic_rules/TrafficRulesFactory.h"
#include "test_map.h"

using namespace lanelet;
using namespace lanelet::ml_converter;
using namespace lanelet::ml_converter::tests;

// template <class result_t = std::chrono::microseconds, class clock_t = std::chrono::steady_clock,
//           class duration_t = std::chrono::microseconds>
// auto since(std::chrono::time_point<clock_t, duration_t> const& start) {
//   return std::chrono::duration_cast<result_t>(clock_t::now() - start);
//  }

TEST_F(MLConverterTest, MapDataInterface) {  // NOLINT
  traffic_rules::TrafficRulesPtr trafficRules{
      traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Vehicle)};
  MapDataInterface::Configuration config{};
  config.reprType = LaneletRepresentationType::Centerline;
  config.paramType = ParametrizationType::LineString;
  config.submapExtentLongitudinal = 5;
  config.submapExtentLateral = 3;
  config.nPoints = 20;

  MapDataInterface parser(laneletMap, config);
  std::vector<BasicPoint2d> pts{BasicPoint2d(0, -3), BasicPoint2d(3, -3), BasicPoint2d(5, -3),
                                BasicPoint2d(6, -5), BasicPoint2d(6, -8), BasicPoint2d(6, -11)};
  std::vector<double> yaws{0, 0, M_PI / 4, M_PI / 3, M_PI / 2, M_PI / 2};

  // auto start = std::chrono::steady_clock::now();
  std::vector<MapDataPtr> mDataVec = parser.mapDataBatch2d(pts, yaws);
  // std::cerr << "Elapsed(micros)=" << since(start).count() << std::endl;

  for (size_t i = 0; i < mDataVec.size(); i++) {
    std::vector<Eigen::MatrixXd> compoundRoadBorders =
        mDataVec[i]->getTensorInstanceData(true, false).compoundLineStringsOfType(LineStringType::RoadBorder);
    std::vector<Eigen::MatrixXd> compoundLaneDividers =
        mDataVec[i]->getTensorInstanceData(true, false).compoundLineStringsOfType(LineStringType::Dashed);
    for (const auto& mat :
         mDataVec[i]->getTensorInstanceData(true, false).compoundLineStringsOfType(LineStringType::Solid)) {
      compoundLaneDividers.push_back(mat);
    }
    for (const auto& mat :
         mDataVec[i]->getTensorInstanceData(true, false).compoundLineStringsOfType(LineStringType::Virtual)) {
      compoundLaneDividers.push_back(mat);
    }
    std::vector<Eigen::MatrixXd> compoundCenterlines =
        mDataVec[i]->getTensorInstanceData(true, false).compoundLineStringsOfType(LineStringType::Centerline);

    switch (i) {
      case 0: {
        LaneletInstances::const_iterator itLL0 = mDataVec[i]->laneletInstances().find(2000);
        EXPECT_TRUE(itLL0 != mDataVec[i]->laneletInstances().end());
        LaneletInstances::const_iterator itLL1 = mDataVec[i]->laneletInstances().find(2002);
        EXPECT_TRUE(itLL1 == mDataVec[i]->laneletInstances().end());
        break;
      }
      case 1: {
        LaneletInstances::const_iterator itLL = mDataVec[i]->laneletInstances().find(2011);
        EXPECT_TRUE(itLL != mDataVec[i]->laneletInstances().end());
        break;
      }
      case 5: {
        LaneletInstances::const_iterator itLL = mDataVec[i]->laneletInstances().find(2004);
        EXPECT_TRUE(itLL == mDataVec[i]->laneletInstances().end());
        break;
      }
    }

    // matplot::hold(matplot::on);
    // matplot::cla();
    // matplot::xlim({-15, 15});
    // matplot::ylim({-15, 15});
    // matplot::gcf()->size(1000, 1000);
    // for (const auto& mat : compoundRoadBorders) {
    //   std::vector<double> x;
    //   std::vector<double> y;
    //   x.resize(mat.rows());
    //   y.resize(mat.rows());
    //   Eigen::VectorXd::Map(&x[0], mat.rows()) = mat.col(0);
    //   Eigen::VectorXd::Map(&y[0], mat.rows()) = mat.col(1);
    //   matplot::pltrafficRulesuto& mat : compoundLaneDividers) {
    //   std::vector<double> x;
    //   std::vector<double> y;
    //   x.resize(mat.rows());
    //   y.resize(mat.rows());
    //   Eigen::VectorXd::Map(&x[0], mat.rows()) = mat.col(0);
    //   Eigen::VectorXd::Map(&y[0], mat.rows()) = mat.col(1);
    //   matplot::plot(x, y, "b")->line_width(3);
    // }
    // for (const auto& mat : compoundCenterlines) {
    //   std::vector<double> x;
    //   std::vector<double> y;
    //   x.resize(mat.rows());
    //   y.resize(mat.rows());
    //   Eigen::VectorXd::Map(&x[0], mat.rows()) = mat.col(0);
    //   Eigen::VectorXd::Map(&y[0], mat.rows()) = mat.col(1);
    //   matplot::plot(x, y, "--gs")->line_width(3).marker_color("g");
    // }
    // matplot::save("map_data_processed_" + std::to_string(i) + ".png");
  }
}

TEST_F(MLConverterTest, MapDataSaveLoad) {
  traffic_rules::TrafficRulesPtr trafficRules{
      traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Vehicle)};
  MapDataInterface::Configuration config{};
  config.reprType = LaneletRepresentationType::Centerline;
  config.paramType = ParametrizationType::LineString;
  config.submapExtentLongitudinal = 5;
  config.submapExtentLateral = 3;
  config.nPoints = 20;
  MapDataInterface parser(laneletMap, config);

  std::vector<BasicPoint2d> pts{BasicPoint2d(0, -3), BasicPoint2d(3, -3), BasicPoint2d(5, -3),
                                BasicPoint2d(6, -5), BasicPoint2d(6, -8), BasicPoint2d(6, -11)};
  std::vector<double> yaws{0, 0, M_PI / 4, M_PI / 3, M_PI / 2, M_PI / 2};

  std::vector<MapDataPtr> mDataVec = parser.mapDataBatch2d(pts, yaws);

  saveMapData("/tmp/lane_data_save_test.xml", mDataVec, false);
  std::vector<MapDataPtr> lDataLoaded = loadMapData("/tmp/lane_data_save_test.xml", false);
  EXPECT_EQ(mDataVec.size(), lDataLoaded.size());
  EXPECT_EQ(
      mDataVec.front()->getTensorInstanceData(true, false).compoundLineStringsOfType(LineStringType::Centerline).size(),
      lDataLoaded.front()
          ->getTensorInstanceData(true, false)
          .compoundLineStringsOfType(LineStringType::Centerline)
          .size());
  EXPECT_EQ(
      mDataVec.front()->getTensorInstanceData(true, false).compoundLineStringsOfType(LineStringType::RoadBorder).size(),
      lDataLoaded.front()
          ->getTensorInstanceData(true, false)
          .compoundLineStringsOfType(LineStringType::RoadBorder)
          .size());
  std::vector<Eigen::MatrixXd> backLaneDividersOrig =
      mDataVec.back()->getTensorInstanceData(true, false).compoundLineStringsOfType(LineStringType::Dashed);
  for (const auto& mat :
       mDataVec.back()->getTensorInstanceData(true, false).compoundLineStringsOfType(LineStringType::Solid)) {
    backLaneDividersOrig.push_back(mat);
  }
  for (const auto& mat :
       mDataVec.back()->getTensorInstanceData(true, false).compoundLineStringsOfType(LineStringType::Virtual)) {
    backLaneDividersOrig.push_back(mat);
  }
  std::vector<Eigen::MatrixXd> backLaneDividersLoaded =
      lDataLoaded.back()->getTensorInstanceData(true, false).compoundLineStringsOfType(LineStringType::Dashed);
  for (const auto& mat :
       lDataLoaded.back()->getTensorInstanceData(true, false).compoundLineStringsOfType(LineStringType::Solid)) {
    backLaneDividersLoaded.push_back(mat);
  }
  for (const auto& mat :
       lDataLoaded.back()->getTensorInstanceData(true, false).compoundLineStringsOfType(LineStringType::Virtual)) {
    backLaneDividersLoaded.push_back(mat);
  }
  EXPECT_EQ(backLaneDividersOrig.size(), backLaneDividersLoaded.size());

  saveMapData("/tmp/lane_data_save_test.bin", mDataVec, true);
  lDataLoaded = loadMapData("/tmp/lane_data_save_test.bin", true);
  EXPECT_EQ(mDataVec.size(), lDataLoaded.size());
  EXPECT_EQ(
      mDataVec.front()->getTensorInstanceData(true, false).compoundLineStringsOfType(LineStringType::Centerline).size(),
      lDataLoaded.front()
          ->getTensorInstanceData(true, false)
          .compoundLineStringsOfType(LineStringType::Centerline)
          .size());
  EXPECT_EQ(
      mDataVec.front()->getTensorInstanceData(true, false).compoundLineStringsOfType(LineStringType::RoadBorder).size(),
      lDataLoaded.front()
          ->getTensorInstanceData(true, false)
          .compoundLineStringsOfType(LineStringType::RoadBorder)
          .size());
  backLaneDividersOrig =
      mDataVec.back()->getTensorInstanceData(true, false).compoundLineStringsOfType(LineStringType::Dashed);
  for (const auto& mat :
       mDataVec.back()->getTensorInstanceData(true, false).compoundLineStringsOfType(LineStringType::Solid)) {
    backLaneDividersOrig.push_back(mat);
  }
  for (const auto& mat :
       mDataVec.back()->getTensorInstanceData(true, false).compoundLineStringsOfType(LineStringType::Virtual)) {
    backLaneDividersOrig.push_back(mat);
  }
  backLaneDividersLoaded =
      lDataLoaded.back()->getTensorInstanceData(true, false).compoundLineStringsOfType(LineStringType::Dashed);
  for (const auto& mat :
       lDataLoaded.back()->getTensorInstanceData(true, false).compoundLineStringsOfType(LineStringType::Solid)) {
    backLaneDividersLoaded.push_back(mat);
  }
  for (const auto& mat :
       lDataLoaded.back()->getTensorInstanceData(true, false).compoundLineStringsOfType(LineStringType::Virtual)) {
    backLaneDividersLoaded.push_back(mat);
  }
  EXPECT_EQ(backLaneDividersOrig.size(), backLaneDividersLoaded.size());
}

TEST_F(MLConverterTest, MapDataInterfaceTrafficElements) {  // NOLINT
  traffic_rules::TrafficRulesPtr trafficRules{
      traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Vehicle)};
  MapDataInterface::Configuration config{};
  config.reprType = LaneletRepresentationType::Centerline;
  config.paramType = ParametrizationType::LineString;
  config.submapExtentLongitudinal = 10;
  config.submapExtentLateral = 5;
  config.nPoints = 20;

  MapDataInterface parser(laneletMap, config);

  // Position near traffic light and stop line (lanelet 2001 at x=6, y=-1)
  BasicPoint2d pos(6, -1);
  double yaw = 0;

  parser.setCurrPosAndExtractSubmap2d(pos, yaw);
  MapDataPtr mapData = parser.mapData(true);

  EXPECT_TRUE(mapData != nullptr);

  // Verify traffic elements are extracted in the submap
  auto stopLines = mapData->teInstancesOfType(TEType::StopLine);
  EXPECT_GT(stopLines.size(), 0);

  auto trafficLights = mapData->teInstancesOfType(TEType::TLCar);
  EXPECT_GT(trafficLights.size(), 0);

  auto arrows = mapData->teInstancesOfType(TEType::ArrowGoStraight);
  EXPECT_GT(arrows.size(), 0);

  // Get TensorInstanceData
  auto tfData = mapData->getTensorInstanceData(true, false);

  // Verify TE matrices
  std::vector<Eigen::MatrixXd> stopLineMatrices = tfData.teInstancesOfType(TEType::StopLine);
  EXPECT_GT(stopLineMatrices.size(), 0);

  std::vector<Eigen::MatrixXd> trafficLightMatrices = tfData.teInstancesOfType(TEType::TLCar);
  EXPECT_GT(trafficLightMatrices.size(), 0);

  // Verify edges are created
  auto teToTEEdges = tfData.teToTEIndexEdges();
  EXPECT_GT(teToTEEdges.size(), 0);

  auto teToCenterlineEdges = tfData.teToCenterlineIndexEdges();
  EXPECT_GT(teToCenterlineEdges.size(), 0);
}

TEST_F(MLConverterTest, MapDataInterfaceBatchTrafficElements) {  // NOLINT
  traffic_rules::TrafficRulesPtr trafficRules{
      traffic_rules::TrafficRulesFactory::create(Locations::Germany, Participants::Vehicle)};
  MapDataInterface::Configuration config{};
  config.reprType = LaneletRepresentationType::Centerline;
  config.paramType = ParametrizationType::LineString;
  config.submapExtentLongitudinal = 8;
  config.submapExtentLateral = 4;
  config.nPoints = 20;

  MapDataInterface parser(laneletMap, config);

  // Test multiple positions: one near traffic light, one near symbol
  std::vector<BasicPoint2d> pts{BasicPoint2d(6, -1), BasicPoint2d(7, -8)};
  std::vector<double> yaws{0, M_PI / 2};

  std::vector<MapDataPtr> mDataVec = parser.mapDataBatch2d(pts, yaws);

  EXPECT_EQ(mDataVec.size(), 2);

  // First position should have traffic light and stop line
  auto tfData0 = mDataVec[0]->getTensorInstanceData(true, false);
  auto stopLines0 = tfData0.teInstancesOfType(TEType::StopLine);
  auto trafficLights0 = tfData0.teInstancesOfType(TEType::TLCar);

  // At least one of these should be present depending on submap extraction
  EXPECT_TRUE(stopLines0.size() > 0 || trafficLights0.size() > 0);

  // Second position should have symbol
  auto tfData1 = mDataVec[1]->getTensorInstanceData(true, false);
  auto symbols30 = tfData1.teInstancesOfType(TEType::Symbol30);
  EXPECT_GT(symbols30.size(), 0);

  // Verify both have edge information
  for (const auto& mData : mDataVec) {
    auto tfData = mData->getTensorInstanceData(true, false);
    // At least one type of edge should exist if TEs are present
    auto teToCenterline = tfData.teToCenterlineIndexEdges();
    auto teToTE = tfData.teToTEIndexEdges();

    // If there are any traffic elements, there should be edges
    bool hasAnyTE = false;
    for (int teType = 0; teType < static_cast<int>(TEType::Unknown); ++teType) {
      if (tfData.teInstancesOfType(static_cast<TEType>(teType)).size() > 0) {
        hasAnyTE = true;
        break;
      }
    }

    if (hasAnyTE) {
      EXPECT_TRUE(teToCenterline.size() > 0 || teToTE.size() > 0);
    }
  }
}