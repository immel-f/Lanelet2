#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <map>
#include <vector>

#include "lanelet2_ml_converter/MapData.h"
#include "lanelet2_ml_converter/MapDataInterface.h"
#include "lanelet2_ml_converter/MapInstances.h"
#include "lanelet2_ml_converter/Utils.h"
#include "lanelet2_python/internal/converter.h"
#include "lanelet2_python/internal/eigen_converter.h"

using namespace boost::python;
using namespace lanelet;
using namespace lanelet::ml_converter;

class MapInstanceWrap : public MapInstance, public wrapper<MapInstance> {
 public:
  std::vector<VectorXd> computeInstanceVectors(bool onlyPoints, bool pointsIn2d) const {
    return this->get_override("computeInstanceVectors")(onlyPoints, pointsIn2d);
  }
  bool process(const OrientedRect &bbox, const ParametrizationType &paramType, int32_t nPoints, double pitch,
               double roll) {
    return this->get_override("process")(bbox, paramType, nPoints, pitch, roll);
  }
};

class LineStringInstanceWrap : public LineStringInstance, public wrapper<LineStringInstance> {
 public:
  std::vector<VectorXd> computeInstanceVectors(bool onlyPoints, bool pointsIn2d) const {
    return this->get_override("computeInstanceVectors")(onlyPoints, pointsIn2d);
  }
  bool process(const OrientedRect &bbox, const ParametrizationType &paramType, int32_t nPoints, double pitch,
               double roll) {
    return this->get_override("process")(bbox, paramType, nPoints, pitch, roll);
  }
  std::vector<MatrixXd> pointMatrices(bool pointsIn2d) const { return this->get_override("pointMatrices")(pointsIn2d); }
};

class LaneLineStringInstanceWrap : public LaneLineStringInstance, public wrapper<LaneLineStringInstance> {
 public:
  LaneLineStringInstanceWrap() {}

  LaneLineStringInstanceWrap(const BasicLineString3d &feature, Id mapID, LineStringType type,
                             const std::vector<Id> &laneletID, bool inverted)
      : LaneLineStringInstance(feature, mapID, type, laneletID, inverted) {}

  std::vector<VectorXd> computeInstanceVectors(bool onlyPoints, bool pointsIn2d) const {
    if (override f = this->get_override("computeInstanceVectors")) return f(onlyPoints, pointsIn2d);
    return LaneLineStringInstance::computeInstanceVectors(onlyPoints, pointsIn2d);
  }
  std::vector<VectorXd> default_computeInstanceVectors(bool onlyPoints, bool pointsIn2d) const {
    return this->LaneLineStringInstance::computeInstanceVectors(onlyPoints, pointsIn2d);
  }
  bool process(const OrientedRect &bbox, const ParametrizationType &paramType, int32_t nPoints, double pitch,
               double roll) {
    if (override f = this->get_override("process")) return f(bbox, paramType, nPoints, pitch, roll);
    return LaneLineStringInstance::process(bbox, paramType, nPoints, pitch, roll);
  }
  bool default_process(const OrientedRect &bbox, const ParametrizationType &paramType, int32_t nPoints, double pitch,
                       double roll) {
    return this->LaneLineStringInstance::process(bbox, paramType, nPoints, pitch, roll);
  }
  std::vector<MatrixXd> pointMatrices(bool pointsIn2d) const {
    if (override f = this->get_override("pointMatrices")) return f(pointsIn2d);
    return LaneLineStringInstance::pointMatrices(pointsIn2d);
  }
  std::vector<MatrixXd> default_pointMatrices(bool pointsIn2d) const {
    return this->LaneLineStringInstance::pointMatrices(pointsIn2d);
  }
};

class TEInstanceWrap : public TEInstance, public wrapper<TEInstance> {
 public:
  TEInstanceWrap() {}

  TEInstanceWrap(const BasicLineString3d &feature, Id mapID, TEType type) : TEInstance(feature, mapID, type) {}

  std::vector<VectorXd> computeInstanceVectors(bool onlyPoints, bool pointsIn2d) const {
    if (override f = this->get_override("computeInstanceVectors")) return f(onlyPoints, pointsIn2d);
    return TEInstance::computeInstanceVectors(onlyPoints, pointsIn2d);
  }
  std::vector<VectorXd> default_computeInstanceVectors(bool onlyPoints, bool pointsIn2d) const {
    return this->TEInstance::computeInstanceVectors(onlyPoints, pointsIn2d);
  }
  bool process(const OrientedRect &bbox, const ParametrizationType &paramType, int32_t nPoints, double pitch,
               double roll) {
    if (override f = this->get_override("process")) return f(bbox, paramType, nPoints, pitch, roll);
    return TEInstance::process(bbox, paramType, nPoints, pitch, roll);
  }
  bool default_process(const OrientedRect &bbox, const ParametrizationType &paramType, int32_t nPoints, double pitch,
                       double roll) {
    return this->TEInstance::process(bbox, paramType, nPoints, pitch, roll);
  }
  std::vector<MatrixXd> pointMatrices(bool pointsIn2d) const {
    if (override f = this->get_override("pointMatrices")) return f(pointsIn2d);
    return TEInstance::pointMatrices(pointsIn2d);
  }
  std::vector<MatrixXd> default_pointMatrices(bool pointsIn2d) const {
    return this->TEInstance::pointMatrices(pointsIn2d);
  }
};

class CompoundLaneLineStringInstanceWrap : public CompoundLaneLineStringInstance,
                                           public wrapper<CompoundLaneLineStringInstance> {
 public:
  CompoundLaneLineStringInstanceWrap() {}

  CompoundLaneLineStringInstanceWrap(const LaneLineStringInstanceList &features, LineStringType compoundType)
      : CompoundLaneLineStringInstance(features, compoundType) {}

  std::vector<VectorXd> computeInstanceVectors(bool onlyPoints, bool pointsIn2d) const {
    if (override f = this->get_override("computeInstanceVectors")) return f(onlyPoints, pointsIn2d);
    return CompoundLaneLineStringInstance::computeInstanceVectors(onlyPoints, pointsIn2d);
  }
  std::vector<VectorXd> default_computeInstanceVectors(bool onlyPoints, bool pointsIn2d) const {
    return this->CompoundLaneLineStringInstance::computeInstanceVectors(onlyPoints, pointsIn2d);
  }
  bool process(const OrientedRect &bbox, const ParametrizationType &paramType, int32_t nPoints, double pitch,
               double roll) {
    if (override f = this->get_override("process")) return f(bbox, paramType, nPoints, pitch, roll);
    return CompoundLaneLineStringInstance::process(bbox, paramType, nPoints, pitch, roll);
  }
  bool default_process(const OrientedRect &bbox, const ParametrizationType &paramType, int32_t nPoints, double pitch,
                       double roll) {
    return this->CompoundLaneLineStringInstance::process(bbox, paramType, nPoints, pitch, roll);
  }
  std::vector<MatrixXd> pointMatrices(bool pointsIn2d) const {
    if (override f = this->get_override("pointMatrices")) return f(pointsIn2d);
    return CompoundLaneLineStringInstance::pointMatrices(pointsIn2d);
  }
  std::vector<MatrixXd> default_pointMatrices(bool pointsIn2d) const {
    return this->CompoundLaneLineStringInstance::pointMatrices(pointsIn2d);
  }
};

class LaneletInstanceWrap : public LaneletInstance, public wrapper<LaneletInstance> {
 public:
  LaneletInstanceWrap() {}

  LaneletInstanceWrap(LaneLineStringInstancePtr leftBoundary, LaneLineStringInstancePtr rightBoundary,
                      LaneLineStringInstancePtr centerline, Id mapID)
      : LaneletInstance(leftBoundary, rightBoundary, centerline, mapID) {}
  LaneletInstanceWrap(const ConstLanelet &ll) : LaneletInstance(ll) {}

  std::vector<VectorXd> computeInstanceVectors(bool onlyPoints, bool pointsIn2d) const {
    if (override f = this->get_override("computeInstanceVectors")) return f(onlyPoints, pointsIn2d);
    return LaneletInstance::computeInstanceVectors(onlyPoints, pointsIn2d);
  }
  std::vector<VectorXd> default_computeInstanceVectors(bool onlyPoints, bool pointsIn2d) const {
    return this->LaneletInstance::computeInstanceVectors(onlyPoints, pointsIn2d);
  }
  bool process(const OrientedRect &bbox, const ParametrizationType &paramType, int32_t nPoints, double pitch = 0,
               double roll = 0) {
    if (override f = this->get_override("process")) return f(bbox, paramType, nPoints, pitch, roll);
    return LaneletInstance::process(bbox, paramType, nPoints, pitch, roll);
  }
  bool default_process(const OrientedRect &bbox, const ParametrizationType &paramType, int32_t nPoints,
                       double pitch = 0, double roll = 0) {
    return this->LaneletInstance::process(bbox, paramType, nPoints, pitch, roll);
  }
};

template <typename T>
struct DictToMapConverter {
  DictToMapConverter() { converter::registry::push_back(&convertible, &construct, type_id<T>()); }
  static void *convertible(PyObject *obj) {
    if (!PyDict_CheckExact(obj)) {  // NOLINT
      return nullptr;
    }
    return obj;
  }
  static void construct(PyObject *obj, converter::rvalue_from_python_stage1_data *data) {
    dict d(borrowed(obj));
    list keys = d.keys();
    list values = d.values();
    T map;
    for (auto i = 0u; i < len(keys); ++i) {
      typename T::key_type key = extract<typename T::key_type>(keys[i]);
      typename T::mapped_type value = extract<typename T::mapped_type>(values[i]);
      map.insert(std::make_pair(key, value));
    }
    using StorageType = converter::rvalue_from_python_storage<T>;
    void *storage = reinterpret_cast<StorageType *>(data)->storage.bytes;  // NOLINT
    new (storage) T(map);
    data->convertible = storage;
  }
};

BOOST_PYTHON_MODULE(PYTHON_API_MODULE_NAME) {  // NOLINT

  Py_Initialize();
  np::initialize();

  enum_<LaneletRepresentationType>("LaneletRepresentationType")
      .value("Centerline", LaneletRepresentationType::Centerline)
      .value("Boundaries", LaneletRepresentationType::Boundaries);

  enum_<ParametrizationType>("ParametrizationType")
      .value("Bezier", ParametrizationType::Bezier)
      .value("BezierEndpointFixed", ParametrizationType::BezierEndpointFixed)
      .value("LineString", ParametrizationType::LineString);

  enum_<LineStringType>("LineStringType")
      .value("RoadBorder", LineStringType::RoadBorder)
      .value("Dashed", LineStringType::Dashed)
      .value("Solid", LineStringType::Solid)
      .value("CurbstoneHigh", LineStringType::CurbstoneHigh)
      .value("CurbstoneLow", LineStringType::CurbstoneLow)
      .value("Fence", LineStringType::Fence)
      .value("SolidSolid", LineStringType::SolidSolid)
      .value("SolidDashed", LineStringType::SolidDashed)
      .value("DashedSolid", LineStringType::DashedSolid)
      .value("Virtual", LineStringType::Virtual)
      .value("Centerline", LineStringType::Centerline)
      .value("BikeCenterline", LineStringType::BikeCenterline)
      .value("Unknown", LineStringType::Unknown)
      .value("DrivableArea", LineStringType::DrivableArea)
      .value("Divider", LineStringType::Divider)
      .value("BikeMarkingDashed", LineStringType::BikeMarkingDashed)
      .value("BikeMarkingSolid", LineStringType::BikeMarkingSolid)
      .value("GuardRail", LineStringType::GuardRail)
      .value("PedestrianCrossing", LineStringType::PedestrianCrossing)
      .value("ZebraCrossing", LineStringType::ZebraCrossing);

  enum_<TEType>("TEType")
      .value("TLCar", TEType::TLCar)
      .value("TLBike", TEType::TLBike)
      .value("TLPedestrian", TEType::TLPedestrian)
      .value("TLMisc", TEType::TLMisc)
      .value("TSMisc", TEType::TSMisc)
      .value("TSNoEntry", TEType::TSNoEntry)
      .value("TSTurnRight", TEType::TSTurnRight)
      .value("TSTurnLeft", TEType::TSTurnLeft)
      .value("TSTurnLeftOrRight", TEType::TSTurnLeftOrRight)
      .value("TSGoStraight", TEType::TSGoStraight)
      .value("TSGoStraightOrRight", TEType::TSGoStraightOrRight)
      .value("TSGoStraightOrLeft", TEType::TSGoStraightOrLeft)
      .value("TSPassRight", TEType::TSPassRight)
      .value("TSPassLeft", TEType::TSPassLeft)
      .value("TSOneWayStreet", TEType::TSOneWayStreet)
      .value("TSYield", TEType::TSYield)
      .value("TSRightOfWay", TEType::TSRightOfWay)
      .value("TSPriorityRoad", TEType::TSPriorityRoad)
      .value("TSStop", TEType::TSStop)
      .value("TSCrossbuck", TEType::TSCrossbuck)
      .value("TSRoundabout", TEType::TSRoundabout)
      .value("TSSpeedLimit", TEType::TSSpeedLimit)
      .value("TSPedestrianCrossing", TEType::TSPedestrianCrossing)
      .value("ArrowTurnRight", TEType::ArrowTurnRight)
      .value("ArrowTurnLeft", TEType::ArrowTurnLeft)
      .value("ArrowTurnLeftOrRight", TEType::ArrowTurnLeftOrRight)
      .value("ArrowGoStraight", TEType::ArrowGoStraight)
      .value("ArrowGoStraightOrRight", TEType::ArrowGoStraightOrRight)
      .value("ArrowGoStraightOrLeft", TEType::ArrowGoStraightOrLeft)
      .value("BikeSymbol", TEType::BikeSymbol)
      .value("Symbol30", TEType::Symbol30)
      .value("Symbol50", TEType::Symbol50)
      .value("Symbol70", TEType::Symbol70)
      .value("StopLine", TEType::StopLine)
      .value("Unknown", TEType::Unknown);

  // LineStringTypeGrouping is a vector of pairs mapping LineStringType lists to representative LineStringType
  typedef std::pair<std::vector<LineStringType>, LineStringType> LineStringTypeGroupingPair;
  class_<LineStringTypeGroupingPair>("LineStringTypeGroupingPair",
                                     "Pair of LineStringType list and representative LineStringType")
      .def_readwrite("types", &LineStringTypeGroupingPair::first)
      .def_readwrite("representative", &LineStringTypeGroupingPair::second);

  typedef std::vector<std::pair<std::vector<LineStringType>, LineStringType>> LineStringTypeGrouping;
  class_<LineStringTypeGrouping>(
      "LineStringTypeGrouping",
      "Type grouping for compound instances: maps LineStringType lists to representative types")
      .def(vector_indexing_suite<LineStringTypeGrouping>());

  def("getDefaultLineStringTypeGrouping", &getDefaultLineStringTypeGrouping,
      "Get the default LineStringTypeGrouping where each type has its own group");
  def("getRoadBorderMergedGrouping", &getRoadBorderMergedGrouping,
      "Get a RoadBorderMerged grouping: merges road border with fence, curbstone high and curbstone low");
  def("getMapTRDefaultSimpleGrouping", &getMapTRDefaultSimpleGrouping,
      "Get MapTR default simple grouping: RoadBorderMerged grouping with all lane dividers merged as well (including "
      "dashed and solid, excluding Virtual)");
  def("getM3TRDefaultGrouping", &getM3TRDefaultGrouping,
      "Get M3TR default grouping: RoadBorderMerged grouping, merges Solid, SolidSolid, SolidDashed, and DashedSolid "
      "LineStringTypes");

  // TETypeGrouping is a vector of pairs mapping TEType lists to representative TEType
  typedef std::pair<std::vector<TEType>, TEType> TETypeGroupingPair;
  class_<TETypeGroupingPair>("TETypeGroupingPair", "Pair of TEType list and representative TEType")
      .def_readwrite("types", &TETypeGroupingPair::first)
      .def_readwrite("representative", &TETypeGroupingPair::second);

  typedef std::vector<std::pair<std::vector<TEType>, TEType>> TETypeGrouping;
  class_<TETypeGrouping>("TETypeGrouping",
                         "Type grouping for traffic elements: maps TEType lists to representative types")
      .def(vector_indexing_suite<TETypeGrouping>());

  def("getDefaultTETypeGrouping", &getDefaultTETypeGrouping,
      "Get the default TETypeGrouping where each type has its own group");
  def("getTETypeRepresentative", &getTETypeRepresentative, (arg("type"), arg("grouping")),
      "Get the representative type for a TEType from the grouping");
  def("getTETypeGroupIndex", &getTETypeGroupIndex, (arg("type"), arg("grouping")), "Get the group index for a TEType");
  def("areTETypesSameGroup", &areTETypesSameGroup, (arg("type1"), arg("type2"), arg("grouping")),
      "Check if two TE types are in the same group");
  def("getLineStringTypeRepresentative", &getLineStringTypeRepresentative, (arg("type"), arg("grouping")),
      "Get the representative type for a LineStringType from the grouping");
  def("getLineStringTypeGroupIndex", &getLineStringTypeGroupIndex, (arg("type"), arg("grouping")),
      "Get the group index for a LineStringType");
  def("areLineStringTypesSameGroup", &areLineStringTypesSameGroup, (arg("type1"), arg("type2"), arg("grouping")),
      "Check if two LineString types are in the same group");

  class_<OrientedRect>("OrientedRect", "Oriented rectangle for feature crop area", no_init)
      .add_property("bounds", make_function(&OrientedRect::bounds_const, return_value_policy<copy_const_reference>()));

  def("getRotatedRect", &getRotatedRect,
      (arg("center"), arg("extentLongitudinal"), arg("extentLateral"), arg("yaw"), arg("from2dPos")));
  def("extractSubmap", &extractSubmap,
      (arg("laneletMap"), arg("center"), arg("extentLongitudinal"), arg("extentLateral")));
  def("bdTypeToEnum", &bdTypeToEnum, (arg("lstring")));
  def("teTypeToEnum", &teTypeToEnum, (arg("te")));
  def("resampleLineString", &resampleLineString, (arg("polyline"), arg("nPoints")));
  def("cutLineString", &cutLineString, (arg("bbox"), arg("polyline")));
  def("transformLineString", &transformLineString, (arg("bbox"), arg("polyline"), arg("pitch"), arg("roll")));
  def("saveMapData", &saveMapData, (arg("filename"), arg("mDataVec"), arg("binary")));
  def("loadMapData", &loadMapData, (arg("filename"), arg("binary")));
  def("saveMapDataMultiFile", &saveMapDataMultiFile, (arg("path"), arg("filenames"), arg("mDataVec"), arg("binary")));
  def("loadMapDataMultiFile", &loadMapDataMultiFile, (arg("path"), arg("filenames"), arg("binary")));

  class_<MapInstanceWrap, boost::noncopyable>("MapInstance", "Abstract base map feature class", no_init)
      .add_property("wasCut", &MapInstance::wasCut)
      .add_property("mapID", &MapInstance::mapID)
      .add_property("initialized", &MapInstance::initialized)
      .add_property("valid", &MapInstance::valid)
      .def("computeInstanceVectors", pure_virtual(&MapInstance::computeInstanceVectors),
           (arg("onlyPoints"), arg("pointsIn2d")))
      .def("process", pure_virtual(&MapInstance::process),
           (arg("bbox"), arg("paramType"), arg("nPoints"), arg("pitch") = 0, arg("roll") = 0));

  class_<LineStringInstanceWrap, bases<MapInstance>, boost::noncopyable>("LineStringInstance",
                                                                         "Abstract line string feature class", no_init)
      .add_property("rawInstance",
                    make_function(&LineStringInstance::rawInstance, return_value_policy<copy_const_reference>()))
      .add_property("cutInstance",
                    make_function(&LineStringInstance::cutInstance, return_value_policy<copy_const_reference>()))
      .add_property("cutAndTransformedInstance", make_function(&LineStringInstance::cutAndTransformedInstance,
                                                               return_value_policy<copy_const_reference>()))
      .add_property("cutTransformedAndResampledInstance",
                    make_function(&LineStringInstance::cutTransformedAndResampledInstance,
                                  return_value_policy<copy_const_reference>()))
      .def("computeInstanceVectors", pure_virtual(&LineStringInstance::computeInstanceVectors),
           (arg("onlyPoints"), arg("pointsIn2d")))
      .def("process", pure_virtual(&LineStringInstance::process),
           (arg("bbox"), arg("paramType"), arg("nPoints"), arg("pitch") = 0, arg("roll") = 0))
      .def("pointMatrices", pure_virtual(&LineStringInstance::pointMatrices), (arg("pointsIn2d")));

  class_<LaneLineStringInstanceWrap, bases<LineStringInstance>, LaneLineStringInstancePtr, boost::noncopyable>(
      "LaneLineStringInstance", "Lane line string feature class",
      init<BasicLineString3d, Id, LineStringType, Ids, bool>())
      .def(init<>())
      .add_property("type", &LaneLineStringInstance::type)
      .add_property("inverted", &LaneLineStringInstance::inverted)
      .add_property("typeInt", &LaneLineStringInstance::typeInt)
      .add_property("laneletIDs",
                    make_function(&LaneLineStringInstance::laneletIDs, return_value_policy<copy_const_reference>()))
      .def("addLaneletID", &LaneLineStringInstance::addLaneletID, (arg("id")))
      .def("computeInstanceVectors", &LaneLineStringInstance::computeInstanceVectors,
           &LaneLineStringInstanceWrap::default_computeInstanceVectors, (arg("onlyPoints"), arg("pointsIn2d")))
      .def("process", &LaneLineStringInstance::process, &LaneLineStringInstanceWrap::default_process,
           (arg("bbox"), arg("paramType"), arg("nPoints"), arg("pitch") = 0, arg("roll") = 0))
      .def("pointMatrices", &LaneLineStringInstance::pointMatrices, &LaneLineStringInstanceWrap::default_pointMatrices,
           (arg("pointsIn2d")));

  class_<TEInstanceWrap, bases<LineStringInstance>, TEInstancePtr, boost::noncopyable>(
      "TEInstance", "Traffic element feature class", init<BasicLineString3d, Id, TEType>())
      .def(init<>())
      .add_property("teType", &TEInstance::teType)
      .def("computeInstanceVectors", &TEInstance::computeInstanceVectors,
           &TEInstanceWrap::default_computeInstanceVectors, (arg("onlyPoints"), arg("pointsIn2d")))
      .def("process", &TEInstance::process, &TEInstanceWrap::default_process,
           (arg("bbox"), arg("paramType"), arg("nPoints"), arg("pitch") = 0, arg("roll") = 0))
      .def("pointMatrices", &TEInstance::pointMatrices, &TEInstanceWrap::default_pointMatrices, (arg("pointsIn2d")));

  class_<LaneletInstanceWrap, bases<MapInstance>, LaneletInstancePtr, boost::noncopyable>(
      "LaneletInstance", "Lanelet feature class that contains lower level LaneLineStringInstances",
      init<LaneLineStringInstancePtr, LaneLineStringInstancePtr, LaneLineStringInstancePtr, Id>())
      .def(init<ConstLanelet>())
      .def(init<>())
      .add_property("leftBoundary", make_function(&LaneletInstance::leftBoundary))
      .add_property("rightBoundary", make_function(&LaneletInstance::rightBoundary))
      .add_property("centerline", make_function(&LaneletInstance::centerline))
      .def("setReprType", &LaneletInstance::setReprType, (arg("reprType")))
      .def("computeInstanceVectors", &LaneletInstance::computeInstanceVectors,
           &LaneletInstanceWrap::default_computeInstanceVectors, (arg("onlyPoints"), arg("pointsIn2d")))
      .def("process", &LaneletInstance::process, &LaneletInstanceWrap::default_process,
           (arg("bbox"), arg("paramType"), arg("nPoints"), arg("pitch") = 0, arg("roll") = 0));

  class_<CompoundLaneLineStringInstanceWrap, bases<LaneLineStringInstance>, CompoundLaneLineStringInstancePtr,
         boost::noncopyable>("CompoundLaneLineStringInstance",
                             "Compound lane line string feature class that can trace back the individual features",
                             init<LaneLineStringInstanceList, LineStringType>())
      .def(init<>())
      .add_property("features", make_function(&CompoundLaneLineStringInstance::features))
      .add_property("pathLengthsRaw", make_function(&CompoundLaneLineStringInstance::pathLengthsRaw,
                                                    return_value_policy<copy_const_reference>()))
      .add_property("pathLengthsProcessed", make_function(&CompoundLaneLineStringInstance::pathLengthsProcessed,
                                                          return_value_policy<copy_const_reference>()))
      .add_property("processedInstancesValid", make_function(&CompoundLaneLineStringInstance::processedInstancesValid,
                                                             return_value_policy<copy_const_reference>()))
      .def("computeInstanceVectors", &CompoundLaneLineStringInstance::computeInstanceVectors,
           &CompoundLaneLineStringInstanceWrap::default_computeInstanceVectors, (arg("onlyPoints"), arg("pointsIn2d")))
      .def("process", &CompoundLaneLineStringInstance::process, &CompoundLaneLineStringInstanceWrap::default_process,
           (arg("bbox"), arg("paramType"), arg("nPoints"), arg("pitch") = 0, arg("roll") = 0))
      .def("pointMatrices", &CompoundLaneLineStringInstance::pointMatrices,
           &CompoundLaneLineStringInstanceWrap::default_pointMatrices, (arg("pointsIn2d")));

  class_<Edge>("Edge", "Struct of a lane graph edge", init<Id, Id, bool>())
      .def(init<>())
      .def_readwrite("el1", &Edge::el1_)
      .def_readwrite("el2", &Edge::el2_)
      .def_readwrite("isLaneChange", &Edge::isLaneChange_);

  {
    scope inMapData =
        class_<MapData, MapDataPtr>("MapData", "Class for holding, accessing and processing of map data")
            .def(init<>())
            .def("build", &MapData::build,
                 (arg("localSubmap"), arg("localSubmapGraph"), arg("trafficRules"),
                  arg("bikeSubmapGraph") = routing::RoutingGraphConstPtr(), arg("ignoreMapElevation") = false,
                  arg("lineStringTypeGrouping") = getDefaultLineStringTypeGrouping()))
            .staticmethod("build")
            .def("processAll", &MapData::processAll)
            .def("lineStringsOfType", &MapData::lineStringsOfType, (arg("type")))
            .def("validLineStringsOfType", &MapData::validLineStringsOfType, (arg("type")))
            .def("compoundLineStringsOfType", &MapData::compoundLineStringsOfType, (arg("type")))
            .def("validCompoundLineStringsOfType", &MapData::validCompoundLineStringsOfType, (arg("type")))
            .def("associatedCpdLineStringsOfType", &MapData::associatedCpdLineStringsOfType,
                 (arg("mapId"), arg("type")))
            .def("teInstancesOfType", &MapData::teInstancesOfType, (arg("type")))
            .def("validTEInstancesOfType", &MapData::validTEInstancesOfType, (arg("type")))
            .add_property("laneletInstances",
                          make_function(&MapData::laneletInstances, return_value_policy<copy_const_reference>()))
            .add_property("llEdges", make_function(&MapData::llEdges, return_value_policy<copy_const_reference>()))
            .add_property("teEdges", make_function(&MapData::teEdges, return_value_policy<copy_const_reference>()))
            .add_property("teToCenterlineEdges",
                          make_function(&MapData::teToCenterlineEdges, return_value_policy<copy_const_reference>()))
            .add_property("teToTEEdges",
                          make_function(&MapData::teToTEEdges, return_value_policy<copy_const_reference>()))
            .add_property("uuid", make_function(&MapData::uuid, return_value_policy<copy_const_reference>()))
            .def("getTensorInstanceData", &MapData::getTensorInstanceData, (arg("pointsIn2d"), arg("ignoreBuffer")));

    class_<MapData::TensorInstanceData>("TensorInstanceData", "TensorInstanceData class for MapData", init<>())
        .def("lineStringsOfType", &MapData::TensorInstanceData::lineStringsOfType, (arg("type")))
        .def("compoundLineStringsOfType", &MapData::TensorInstanceData::compoundLineStringsOfType, (arg("type")))
        .def("teInstancesOfType", &MapData::TensorInstanceData::teInstancesOfType, (arg("type")))
        .def("pointMatrixCpdLineStrings", &MapData::TensorInstanceData::pointMatrixCpdLineStrings,
             (arg("type"), arg("index")))
        .add_property("teToCenterlineIndexEdges", make_function(&MapData::TensorInstanceData::teToCenterlineIndexEdges,
                                                                return_value_policy<copy_const_reference>()))
        .add_property("teToTEIndexEdges", make_function(&MapData::TensorInstanceData::teToTEIndexEdges,
                                                        return_value_policy<copy_const_reference>()))
        .add_property("uuid",
                      make_function(&MapData::TensorInstanceData::uuid, return_value_policy<copy_const_reference>()));
  }

  {
    void (MapDataInterface::*setCurrPosAndExtractSubmap2d)(const lanelet::BasicPoint2d &, double) =
        &MapDataInterface::setCurrPosAndExtractSubmap2d;
    void (MapDataInterface::*setCurrPosAndExtractSubmap4d)(const lanelet::BasicPoint3d &, double) =
        &MapDataInterface::setCurrPosAndExtractSubmap;
    void (MapDataInterface::*setCurrPosAndExtractSubmap6d)(const lanelet::BasicPoint3d &, double, double, double) =
        &MapDataInterface::setCurrPosAndExtractSubmap;

    std::vector<MapDataPtr> (MapDataInterface::*mapDataBatch2d)(std::vector<BasicPoint2d>, std::vector<double>) =
        &MapDataInterface::mapDataBatch2d;
    std::vector<MapDataPtr> (MapDataInterface::*mapDataBatch4d)(std::vector<BasicPoint3d>, std::vector<double>) =
        &MapDataInterface::mapDataBatch;
    std::vector<MapDataPtr> (MapDataInterface::*mapDataBatch6d)(std::vector<BasicPoint3d>, std::vector<double>,
                                                                std::vector<double>, std::vector<double>) =
        &MapDataInterface::mapDataBatch;
    scope inMapDataInterface =
        class_<MapDataInterface>("MapDataInterface", "Main Interface Class for processing of Lanelet maps",
                                 init<LaneletMapConstPtr>())
            .def(init<LaneletMapConstPtr, MapDataInterface::Configuration>())
            .add_property("config",
                          make_function(&MapDataInterface::config, return_value_policy<copy_const_reference>()))
            .def("setCurrPosAndExtractSubmap2d", setCurrPosAndExtractSubmap2d, (arg("pt"), arg("yaw")))
            .def("setCurrPosAndExtractSubmap", setCurrPosAndExtractSubmap4d, (arg("pt"), arg("yaw")))
            .def("setCurrPosAndExtractSubmap", setCurrPosAndExtractSubmap6d,
                 (arg("pt"), arg("yaw"), arg("pitch"), arg("roll")))
            .def("mapData", &MapDataInterface::mapData, (arg("processAll")))
            .def("mapDataBatch2d", mapDataBatch2d, (arg("pts"), arg("yaws")))
            .def("mapDataBatch", mapDataBatch4d, (arg("pts"), arg("yaws")))
            .def("mapDataBatch", mapDataBatch6d, (arg("pts"), arg("yaws"), arg("pitches"), arg("rolls")));

    class_<MapDataInterface::Configuration>("Configuration", "Configuration class for MapDataInterface", init<>())
        .def(init<LaneletRepresentationType, ParametrizationType, double, double, int, int>())
        .def_readwrite("reprType", &MapDataInterface::Configuration::reprType)
        .def_readwrite("paramType", &MapDataInterface::Configuration::paramType)
        .def_readwrite("submapExtentLongitudinal", &MapDataInterface::Configuration::submapExtentLongitudinal)
        .def_readwrite("submapExtentLateral", &MapDataInterface::Configuration::submapExtentLateral)
        .def_readwrite("ignoreMapElevation", &MapDataInterface::Configuration::ignoreMapElevation)
        .def_readwrite("resampleLanes", &MapDataInterface::Configuration::resampleLanes)
        .def_readwrite("nPointsLanes", &MapDataInterface::Configuration::nPointsLanes)
        .def_readwrite("resampleTE", &MapDataInterface::Configuration::resampleTE)
        .def_readwrite("nPointsTE", &MapDataInterface::Configuration::nPointsTE)
        .def_readwrite("lineStringTypeGrouping", &MapDataInterface::Configuration::lineStringTypeGrouping)
        .def_readwrite("teTypeGrouping", &MapDataInterface::Configuration::teTypeGrouping);
  }

  // Eigen, stl etc. converters
  converters::convertMatrix<MatrixXd>(true);
  converters::convertVector<VectorXd>(true);

  converters::VectorToListConverter<std::vector<MatrixXd>>();
  converters::VectorToListConverter<BasicLineStrings3d>();
  converters::VectorToListConverter<LaneLineStringInstanceList>();
  converters::VectorToListConverter<CompoundLaneLineStringInstanceList>();
  converters::VectorToListConverter<
      boost::geometry::model::ring<BasicPoint2d, true, true, std::vector, std::allocator>>();
  converters::VectorToListConverter<std::vector<double>>();
  converters::VectorToListConverter<std::vector<int>>();
  converters::VectorToListConverter<std::vector<MapDataPtr>>();
  converters::VectorToListConverter<std::vector<Edge>>();
  converters::VectorToListConverter<TEToCenterlineEdges>();
  converters::VectorToListConverter<TEToTEEdges>();
  converters::VectorToListConverter<TEToCenterlineIndexEdges>();
  converters::VectorToListConverter<TEToTEIndexEdges>();
  converters::IterableConverter()
      .fromPython<std::vector<MatrixXd>>()
      .fromPython<BasicLineString3d>()
      .fromPython<BasicLineStrings3d>()
      .fromPython<std::vector<BasicPoint2d>>()
      .fromPython<std::vector<double>>()
      .fromPython<std::vector<std::string>>()
      .fromPython<std::vector<MapDataPtr>>()
      .fromPython<LaneLineStringInstanceList>()
      .fromPython<CompoundLaneLineStringInstanceList>();
  converters::MapToDictConverter<LaneLineStringInstances>();
  converters::MapToDictConverter<LaneletInstances>();
  converters::MapToDictConverter<Edges>();
  converters::MapToDictConverter<TEInstances>();
  DictToMapConverter<LaneLineStringInstances>();
  DictToMapConverter<LaneletInstances>();
  DictToMapConverter<TEInstances>();
  converters::PairConverter<std::pair<TEInstancePtr, CompoundLaneLineStringInstancePtr>>();
  converters::PairConverter<std::pair<TEInstancePtr, TEInstancePtr>>();
  converters::PyTuple<TEType, size_t, size_t>();
  converters::PyTuple<TEType, size_t, TEType, size_t>();
  class_<std::vector<bool>>("BoolList").def(vector_indexing_suite<std::vector<bool>>());

  def("toPointMatrix", &toPointMatrix, (arg("lString"), arg("pointsIn2d")));

  implicitly_convertible<routing::RoutingGraphPtr, routing::RoutingGraphConstPtr>();
}
