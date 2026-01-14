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
  std::vector<LaneDataPtr> lDataVec = parser.laneDataBatch2d(pts, yaws);
  // std::cerr << "Elapsed(micros)=" << since(start).count() << std::endl;

  for (size_t i = 0; i < lDataVec.size(); i++) {
    std::vector<Eigen::MatrixXd> compoundRoadBorders =
        lDataVec[i]->getTensorInstanceData(true, false).compoundLineStringsOfType(LineStringType::RoadBorder);
    std::vector<Eigen::MatrixXd> compoundLaneDividers =
        lDataVec[i]->getTensorInstanceData(true, false).compoundLineStringsOfType(LineStringType::Dashed);
    for (const auto& mat :
         lDataVec[i]->getTensorInstanceData(true, false).compoundLineStringsOfType(LineStringType::Solid)) {
      compoundLaneDividers.push_back(mat);
    }
    for (const auto& mat :
         lDataVec[i]->getTensorInstanceData(true, false).compoundLineStringsOfType(LineStringType::Virtual)) {
      compoundLaneDividers.push_back(mat);
    }
    std::vector<Eigen::MatrixXd> compoundCenterlines =
        lDataVec[i]->getTensorInstanceData(true, false).compoundLineStringsOfType(LineStringType::Centerline);

    switch (i) {
      case 0: {
        LaneletInstances::const_iterator itLL0 = lDataVec[i]->laneletInstances().find(2000);
        EXPECT_TRUE(itLL0 != lDataVec[i]->laneletInstances().end());
        LaneletInstances::const_iterator itLL1 = lDataVec[i]->laneletInstances().find(2002);
        EXPECT_TRUE(itLL1 == lDataVec[i]->laneletInstances().end());
        break;
      }
      case 1: {
        LaneletInstances::const_iterator itLL = lDataVec[i]->laneletInstances().find(2011);
        EXPECT_TRUE(itLL != lDataVec[i]->laneletInstances().end());
        break;
      }
      case 5: {
        LaneletInstances::const_iterator itLL = lDataVec[i]->laneletInstances().find(2004);
        EXPECT_TRUE(itLL == lDataVec[i]->laneletInstances().end());
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

  std::vector<LaneDataPtr> lDataVec = parser.laneDataBatch2d(pts, yaws);

  saveLaneData("/tmp/lane_data_save_test.xml", lDataVec, false);
  std::vector<LaneDataPtr> lDataLoaded = loadLaneData("/tmp/lane_data_save_test.xml", false);
  EXPECT_EQ(lDataVec.size(), lDataLoaded.size());
  EXPECT_EQ(
      lDataVec.front()->getTensorInstanceData(true, false).compoundLineStringsOfType(LineStringType::Centerline).size(),
      lDataLoaded.front()
          ->getTensorInstanceData(true, false)
          .compoundLineStringsOfType(LineStringType::Centerline)
          .size());
  EXPECT_EQ(
      lDataVec.front()->getTensorInstanceData(true, false).compoundLineStringsOfType(LineStringType::RoadBorder).size(),
      lDataLoaded.front()
          ->getTensorInstanceData(true, false)
          .compoundLineStringsOfType(LineStringType::RoadBorder)
          .size());
  std::vector<Eigen::MatrixXd> backLaneDividersOrig =
      lDataVec.back()->getTensorInstanceData(true, false).compoundLineStringsOfType(LineStringType::Dashed);
  for (const auto& mat :
       lDataVec.back()->getTensorInstanceData(true, false).compoundLineStringsOfType(LineStringType::Solid)) {
    backLaneDividersOrig.push_back(mat);
  }
  for (const auto& mat :
       lDataVec.back()->getTensorInstanceData(true, false).compoundLineStringsOfType(LineStringType::Virtual)) {
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

  saveLaneData("/tmp/lane_data_save_test.bin", lDataVec, true);
  lDataLoaded = loadLaneData("/tmp/lane_data_save_test.bin", true);
  EXPECT_EQ(lDataVec.size(), lDataLoaded.size());
  EXPECT_EQ(
      lDataVec.front()->getTensorInstanceData(true, false).compoundLineStringsOfType(LineStringType::Centerline).size(),
      lDataLoaded.front()
          ->getTensorInstanceData(true, false)
          .compoundLineStringsOfType(LineStringType::Centerline)
          .size());
  EXPECT_EQ(
      lDataVec.front()->getTensorInstanceData(true, false).compoundLineStringsOfType(LineStringType::RoadBorder).size(),
      lDataLoaded.front()
          ->getTensorInstanceData(true, false)
          .compoundLineStringsOfType(LineStringType::RoadBorder)
          .size());
  backLaneDividersOrig =
      lDataVec.back()->getTensorInstanceData(true, false).compoundLineStringsOfType(LineStringType::Dashed);
  for (const auto& mat :
       lDataVec.back()->getTensorInstanceData(true, false).compoundLineStringsOfType(LineStringType::Solid)) {
    backLaneDividersOrig.push_back(mat);
  }
  for (const auto& mat :
       lDataVec.back()->getTensorInstanceData(true, false).compoundLineStringsOfType(LineStringType::Virtual)) {
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