#include <gtest/gtest.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_traffic_rules/GermanTrafficRules.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <memory>
#include <utility>

#include "lanelet2_ml_converter/Forward.h"
#include "lanelet2_ml_converter/Utils.h"

/// The coordinates and relations for this test can be found in "LaneletTestMap.xml" which can be viewed in
/// https://www.draw.io
namespace lanelet {
namespace ml_converter {
namespace tests {

class MapTestData {
 public:
  MapTestData() {
    initPoints();
    initLineStrings();
    initLanelets();
    initTrafficElements();
    initRegulatoryElements();
    laneletMap = std::make_shared<LaneletMap>(lanelets, areas, regulatoryElements, std::unordered_map<Id, Polygon3d>(),
                                              lines, points);
  }

  void addPoint(double x, double y, double z) {
    points.insert(std::pair<Id, Point3d>(pointId, Point3d(pointId, x, y, z)));
    pointId++;
  }

  void addLine(const Points3d& points) {
    lines.insert(std::pair<Id, LineString3d>(lineId, LineString3d(lineId, points)));
    lineId++;
  }

  void addLaneletVehicle(const LineString3d& left, const LineString3d& right) {
    Lanelet ll{laneletId, left, right};
    ll.setAttribute(AttributeName::Subtype, AttributeValueString::Road);
    lanelets.insert(std::make_pair(laneletId, ll));
    laneletId++;
  }

  void addLaneletBike(const LineString3d& left, const LineString3d& right) {
    Lanelet ll{laneletId, left, right};
    ll.setAttribute(AttributeName::Subtype, AttributeValueString::BicycleLane);
    lanelets.insert(std::make_pair(laneletId, ll));
    laneletId++;
  }

  Id pointId{0};
  Id lineId{1000};
  Id laneletId{2000};
  Id regElemId{3000};
  std::unordered_map<Id, Lanelet> lanelets;
  std::unordered_map<Id, Point3d> points;
  std::unordered_map<Id, LineString3d> lines;
  std::unordered_map<Id, Area> areas;
  std::unordered_map<Id, RegulatoryElementPtr> regulatoryElements;
  LaneletMapPtr laneletMap;

 private:
  void initPoints() {
    points.clear();
    addPoint(0.0, 0.0, 1.0);    // p0
    addPoint(2.0, 0.0, 1.0);    // p1
    addPoint(4.0, 0.0, 1.0);    // p2
    addPoint(6.0, 0.0, 1.0);    // p3
    addPoint(8.0, 0.0, 1.0);    // p4
    addPoint(10.0, 0.0, 1.0);   // p5
    addPoint(12.0, 0.0, 1.0);   // p6
    addPoint(14.0, 0.0, 1.0);   // p7
    addPoint(6.0, -1.0, 1.0);   // p8
    addPoint(8.0, -1.0, 1.0);   // p9
    addPoint(0.0, -2.0, 1.0);   // p10
    addPoint(2.0, -2.0, 1.0);   // p11
    addPoint(4.0, -2.0, 1.0);   // p12
    addPoint(6.0, -2.0, 1.0);   // p13
    addPoint(8.0, -2.0, 1.0);   // p14
    addPoint(10.0, -2.0, 1.0);  // p15
    addPoint(12.0, -2.0, 1.0);  // p16
    addPoint(14.0, -2.0, 1.0);  // p17
    addPoint(6.0, -3.0, 1.0);   // p18
    addPoint(8.0, -3.0, 1.0);   // p19
    addPoint(0.0, -4.0, 1.0);   // p20
    addPoint(2.0, -4.0, 1.0);
    addPoint(4.0, -4.0, 1.0);
    addPoint(6.0, -4.0, 1.0);
    addPoint(8.0, -4.0, 1.0);
    addPoint(10.0, -4.0, 1.0);
    addPoint(12.0, -4.0, 1.0);
    addPoint(14.0, -4.0, 1.0);  // p27
    addPoint(5.0, -5.0, 1.0);
    addPoint(7.0, -5.0, 1.0);
    addPoint(9.0, -5.0, 1.0);  // p30
    addPoint(5.0, -7.0, 1.0);
    addPoint(7.0, -7.0, 1.0);
    addPoint(9.0, -7.0, 1.0);  // p33
    addPoint(5.0, -9.0, 1.0);
    addPoint(7.0, -9.0, 1.0);
    addPoint(9.0, -9.0, 1.0);  // p36
    addPoint(5.0, -11.0, 1.0);
    addPoint(7.0, -11.0, 1.0);
    addPoint(9.0, -11.0, 1.0);  // p39

    addPoint(5.0, -2.0, 1.0);  // p40
    addPoint(6.0, -1.0, 1.0);  // p41
    addPoint(8.0, -1.0, 1.0);  // p42
    addPoint(9.0, -2.0, 1.0);  // p43
    addPoint(7.0, -3.0, 1.0);  // p44
    addPoint(7.0, -4.0, 1.0);  // p45
    addPoint(9.0, -0.5, 1.0);  // p46

    // Bike lane points above lanelets 2000, 2001, 2002
    addPoint(0.0, 1.0, 1.0);   // p47
    addPoint(2.0, 1.0, 1.0);   // p48
    addPoint(4.0, 1.0, 1.0);   // p49
    addPoint(6.0, 1.0, 1.0);   // p50
    addPoint(8.0, 1.0, 1.0);   // p51
    addPoint(10.0, 1.0, 1.0);  // p52
    addPoint(12.0, 1.0, 1.0);  // p53
    addPoint(14.0, 1.0, 1.0);  // p54
    addPoint(0.0, 2.0, 1.0);   // p55
    addPoint(2.0, 2.0, 1.0);   // p56
    addPoint(4.0, 2.0, 1.0);   // p57
    addPoint(6.0, 2.0, 1.0);   // p58
    addPoint(8.0, 2.0, 1.0);   // p59
    addPoint(10.0, 2.0, 1.0);  // p60
    addPoint(12.0, 2.0, 1.0);  // p61
    addPoint(14.0, 2.0, 1.0);  // p62
  }

  void initLineStrings() {
    lines.clear();
    addLine(Points3d{points.at(5), points.at(6), points.at(7)});  // l1000
    lines.at(1000).setAttribute(AttributeName::Type, AttributeValueString::RoadBorder);
    addLine(Points3d{points.at(2), points.at(3), points.at(4), points.at(5)});  // l1001
    lines.at(1001).setAttribute(AttributeName::Type, AttributeValueString::RoadBorder);
    addLine(Points3d{points.at(0), points.at(1), points.at(2)});  // l1002
    lines.at(1002).setAttribute(AttributeName::Type, AttributeValueString::RoadBorder);

    addLine(Points3d{points.at(15), points.at(16), points.at(17)});  // l1003
    lines.at(1003).setAttribute(AttributeName::Type, AttributeValueString::LineThin);
    lines.at(1003).setAttribute(AttributeName::Subtype, AttributeValueString::Dashed);
    addLine(Points3d{points.at(12), points.at(13), points.at(14), points.at(15)});  // l1004
    lines.at(1004).setAttribute(AttributeName::Type, AttributeValueString::LineThin);
    lines.at(1004).setAttribute(AttributeName::Subtype, AttributeValueString::Solid);
    addLine(Points3d{points.at(10), points.at(11), points.at(12)});  // l1005
    lines.at(1005).setAttribute(AttributeName::Type, AttributeValueString::LineThin);
    lines.at(1005).setAttribute(AttributeName::Subtype, AttributeValueString::Dashed);

    addLine(Points3d{points.at(20), points.at(21), points.at(22)});  // l1006
    lines.at(1006).setAttribute(AttributeName::Type, AttributeValueString::RoadBorder);
    addLine(Points3d{points.at(22), points.at(23), points.at(24), points.at(25)});  // l1007
    lines.at(1007).setAttribute(AttributeName::Type, AttributeValueString::LineThin);
    lines.at(1007).setAttribute(AttributeName::Subtype, AttributeValueString::Dashed);
    addLine(Points3d{points.at(25), points.at(26), points.at(27)});  // l1008
    lines.at(1008).setAttribute(AttributeName::Type, AttributeValueString::RoadBorder);

    addLine(Points3d{points.at(30), points.at(25)});  // l1009
    lines.at(1009).setAttribute(AttributeName::Type, AttributeValueString::RoadBorder);
    addLine(Points3d{points.at(22), points.at(28)});  // l1010
    lines.at(1010).setAttribute(AttributeName::Type, AttributeValueString::RoadBorder);

    addLine(Points3d{points.at(28), points.at(31)});  // l1011
    lines.at(1011).setAttribute(AttributeName::Type, AttributeValueString::RoadBorder);
    addLine(Points3d{points.at(29), points.at(32)});  // l1012
    lines.at(1012).setAttribute(AttributeName::Type, AttributeValueString::LineThin);
    lines.at(1012).setAttribute(AttributeName::Subtype, AttributeValueString::Solid);
    addLine(Points3d{points.at(33), points.at(30)});  // l1013
    lines.at(1013).setAttribute(AttributeName::Type, AttributeValueString::RoadBorder);

    addLine(Points3d{points.at(31), points.at(34), points.at(37)});  // l1014
    lines.at(1014).setAttribute(AttributeName::Type, AttributeValueString::RoadBorder);
    addLine(Points3d{points.at(32), points.at(35), points.at(38)});  // l1015
    lines.at(1015).setAttribute(AttributeName::Type, AttributeValueString::LineThin);
    lines.at(1015).setAttribute(AttributeName::Subtype, AttributeValueString::Dashed);
    addLine(Points3d{points.at(39), points.at(36), points.at(33)});  // l1016
    lines.at(1016).setAttribute(AttributeName::Type, AttributeValueString::RoadBorder);

    addLine(Points3d{points.at(12), points.at(18), points.at(29)});  // l1017
    lines.at(1017).setAttribute(AttributeName::Type, AttributeValueString::Virtual);
    addLine(Points3d{points.at(29), points.at(19), points.at(15)});  // l1018
    lines.at(1018).setAttribute(AttributeName::Type, AttributeValueString::Virtual);

    addLine(Points3d{points.at(40), points.at(41), points.at(42), points.at(43)});  // l1019
    lines.at(1019).setAttribute("drivable_space_border", "true");
    addLine(Points3d{points.at(40), points.at(18), points.at(44)});  // l1020
    lines.at(1020).setAttribute("drivable_space_border", "true");
    addLine(Points3d{points.at(44), points.at(45)});  // l1021
    lines.at(1021).setAttribute("drivable_space_border", "true");
    addLine(Points3d{points.at(44), points.at(19), points.at(43)});  // l1022
    lines.at(1022).setAttribute("drivable_space_border", "true");
    addLine(Points3d{points.at(42), points.at(46)});  // l1023
    lines.at(1023).setAttribute("drivable_space_border", "true");

    // Bike lane linestrings above lanelets 2000, 2001, 2002
    addLine(Points3d{points.at(55), points.at(56), points.at(57)});  // l1024
    lines.at(1024).setAttribute(AttributeName::Type, AttributeValueString::RoadBorder);
    addLine(Points3d{points.at(57), points.at(58), points.at(59), points.at(60)});  // l1025
    lines.at(1025).setAttribute(AttributeName::Type, AttributeValueString::RoadBorder);
    addLine(Points3d{points.at(60), points.at(61), points.at(62)});  // l1026
    lines.at(1026).setAttribute(AttributeName::Type, AttributeValueString::RoadBorder);
    addLine(Points3d{points.at(47), points.at(48), points.at(49)});  // l1027
    lines.at(1027).setAttribute(AttributeName::Type, AttributeValueString::LineThin);
    lines.at(1027).setAttribute(AttributeName::Subtype, AttributeValueString::Dashed);
    addLine(Points3d{points.at(49), points.at(50), points.at(51), points.at(52)});  // l1028
    lines.at(1028).setAttribute(AttributeName::Type, AttributeValueString::LineThin);
    lines.at(1028).setAttribute(AttributeName::Subtype, AttributeValueString::Solid);
    addLine(Points3d{points.at(52), points.at(53), points.at(54)});  // l1029
    lines.at(1029).setAttribute(AttributeName::Type, AttributeValueString::LineThin);
    lines.at(1029).setAttribute(AttributeName::Subtype, AttributeValueString::Dashed);

    // Zebra crossing borders - using points 28-29-30 (left) and 31-32-33 (right)
    addLine(Points3d{points.at(28), points.at(29), points.at(30)});  // l1030
    lines.at(1030).setAttribute(AttributeName::Type, AttributeValueString::Zebra);
    addLine(Points3d{points.at(31), points.at(32), points.at(33)});  // l1031
    lines.at(1031).setAttribute(AttributeName::Type, AttributeValueString::Zebra);
  }
  void initLanelets() {
    lanelets.clear();
    addLaneletVehicle(lines.at(1002), lines.at(1005));           // ll2000
    addLaneletVehicle(lines.at(1001), lines.at(1004));           // ll2001
    addLaneletVehicle(lines.at(1000), lines.at(1003));           // ll2002
    addLaneletVehicle(lines.at(1005), lines.at(1006));           // ll2003
    addLaneletVehicle(lines.at(1004), lines.at(1007));           // ll2004
    addLaneletVehicle(lines.at(1003), lines.at(1008));           // ll2005
    addLaneletVehicle(lines.at(1012), lines.at(1011));           // ll2006
    addLaneletVehicle(lines.at(1012).invert(), lines.at(1013));  // ll2007
    addLaneletVehicle(lines.at(1015), lines.at(1014));           // ll2008
    addLaneletVehicle(lines.at(1015).invert(), lines.at(1016));  // ll2009
    addLaneletVehicle(lines.at(1017), lines.at(1010));           // ll2010
    addLaneletVehicle(lines.at(1018), lines.at(1009));           // ll2011
    addLaneletBike(lines.at(1027), lines.at(1002));              // ll2012
    addLaneletBike(lines.at(1028), lines.at(1001));              // ll2013
    addLaneletBike(lines.at(1029), lines.at(1000));              // ll2014
    addLaneletBike(lines.at(1029), lines.at(1026));              // ll2015
    addLaneletBike(lines.at(1028), lines.at(1025));              // ll2016
    addLaneletBike(lines.at(1027), lines.at(1024));              // ll2017

    // Add zebra crossing lanelet
    Lanelet crosswalk{laneletId, lines.at(1030), lines.at(1031)};
    crosswalk.setAttribute(AttributeName::Subtype, "crosswalk");
    lanelets.insert(std::make_pair(laneletId, crosswalk));  // ll2018
    laneletId++;
  }
  void initTrafficElements() {
    // Stop line across lanelet 2003 at x=4
    addLine(Points3d{points.at(12), points.at(22)});  // l1032
    lines.at(1032).setAttribute(AttributeName::Type, "stop_line");

    // Traffic light positioned above the stop line
    addPoint(5.0, -3.5, 3.0);                         // p63 - bottom of traffic light
    addPoint(5.0, -3.5, 4.0);                         // p64 - top of traffic light
    addLine(Points3d{points.at(63), points.at(64)});  // l1033
    lines.at(1033).setAttribute(AttributeName::Type, AttributeValueString::TrafficLight);
    lines.at(1033).setAttribute(AttributeName::Subtype, "red_yellow_green");

    // Straight arrow on lanelet 2000
    addPoint(1.0, -1.0, 1.0);                         // p65
    addPoint(3.0, -1.0, 1.0);                         // p66
    addLine(Points3d{points.at(65), points.at(66)});  // l1034
    lines.at(1034).setAttribute(AttributeName::Type, "arrow");
    lines.at(1034).setAttribute(AttributeName::Subtype, "straight");

    // Speed limit symbol (30) on lanelet 2008
    addPoint(6.0, -9.0, 1.0);                         // p67
    addPoint(6.0, -10.0, 1.0);                        // p68
    addLine(Points3d{points.at(67), points.at(68)});  // l1035
    lines.at(1035).setAttribute(AttributeName::Type, "symbol");
    lines.at(1035).setAttribute(AttributeName::Subtype, "30");

    // Straight Right arrow on lanelet 2003
    addPoint(1.0, -3.0, 1.0);                         // p69
    addPoint(3.0, -3.0, 1.0);                         // p70
    addLine(Points3d{points.at(69), points.at(70)});  // l1036
    lines.at(1036).setAttribute(AttributeName::Type, "arrow");
    lines.at(1036).setAttribute(AttributeName::Subtype, "straight_right");

    // Zebra crossing lanelet with reversed right border geometry ordering
    addLine(Points3d{points.at(33), points.at(32), points.at(31)});  // l1037
    lines.at(1037).setAttribute(AttributeName::Type, AttributeValueString::Zebra);

    Lanelet reversedCrosswalk{laneletId, lines.at(1030), lines.at(1037)};
    reversedCrosswalk.setAttribute(AttributeName::Subtype, "crosswalk");
    lanelets.insert(std::make_pair(laneletId, reversedCrosswalk));  // ll2019
    laneletId++;
  }
  void initRegulatoryElements() {
    // Create TrafficLight regulatory element connecting traffic light to stop line
    // and associating with lanelet 2003
    AttributeMap trafficLightAttrs;
    trafficLightAttrs[AttributeName::Type] = "regulatory_element";
    trafficLightAttrs[AttributeName::Subtype] = "traffic_light";
    auto trafficLightRegElem = TrafficLight::make(regElemId++, trafficLightAttrs, {lines.at(1033)}, lines.at(1032));
    regulatoryElements.insert({trafficLightRegElem->id(), trafficLightRegElem});

    // Add regulatory element to the lanelet
    lanelets.at(2003).addRegulatoryElement(trafficLightRegElem);
  }
};

namespace {                   // NOLINT
static MapTestData testData;  // NOLINT
}  // namespace

class MLConverterTest : public ::testing::Test {
 public:
  const std::unordered_map<Id, Lanelet>& lanelets{testData.lanelets};
  const std::unordered_map<Id, Point3d>& points{testData.points};
  const std::unordered_map<Id, LineString3d>& lines{testData.lines};
  const LaneletMapConstPtr laneletMap{testData.laneletMap};
  const BasicPoint3d centerBbox{5, 5, 0};
  const double extentLongitudinalBbox{15};
  const double extentLateralBbox{10};
  const double yawBbox{M_PI / 2.0};
  OrientedRect bbox;
  MLConverterTest() : bbox{getRotatedRect(centerBbox, extentLongitudinalBbox, extentLateralBbox, yawBbox, true)} {}
};

}  // namespace tests
}  // namespace ml_converter
}  // namespace lanelet
