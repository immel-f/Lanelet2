#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <map>
#include <vector>

#include "lanelet2_map_learning/MapData.h"
#include "lanelet2_map_learning/MapDataInterface.h"
#include "lanelet2_map_learning/MapFeatures.h"
#include "lanelet2_map_learning/Utils.h"
#include "lanelet2_python/internal/converter.h"
#include "lanelet2_python/internal/eigen_converter.h"

using namespace boost::python;
using namespace lanelet;
using namespace lanelet::map_learning;

/// TODO: ADD VIRTUAL FUNCTIONS, CONSTRUCTORS, EIGEN TO NUMPY
/// TODO: DEBUG EIGEN SEGFAULTS FROM PYTHON

class MapFeatureWrap : public MapFeature, public wrapper<MapFeature> {
 public:
  VectorXd computeFeatureVector(bool onlyPoints, bool pointsIn2d) const {
    return this->get_override("computeFeatureVector")(onlyPoints, pointsIn2d);
  }
  bool process(const OrientedRect &bbox, const ParametrizationType &paramType, int32_t nPoints) {
    return this->get_override("process")(bbox, paramType, nPoints);
  }
};

class LineStringFeatureWrap : public LineStringFeature, public wrapper<LineStringFeature> {
 public:
  VectorXd computeFeatureVector(bool onlyPoints, bool pointsIn2d) const {
    return this->get_override("computeFeatureVector")(onlyPoints, pointsIn2d);
  }
  bool process(const OrientedRect &bbox, const ParametrizationType &paramType, int32_t nPoints) {
    return this->get_override("process")(bbox, paramType, nPoints);
  }
  MatrixXd pointMatrix(bool pointsIn2d) const { return this->get_override("pointMatrix")(pointsIn2d); }
};

class LaneLineStringFeatureWrap : public LaneLineStringFeature, public wrapper<LaneLineStringFeature> {
 public:
  LaneLineStringFeatureWrap() {}

  LaneLineStringFeatureWrap(const BasicLineString3d &feature, Id mapID, LineStringType type, Id laneletID)
      : LaneLineStringFeature(feature, mapID, type, laneletID) {}

  VectorXd computeFeatureVector(bool onlyPoints, bool pointsIn2d) const {
    if (override f = this->get_override("computeFeatureVector")) return f(onlyPoints, pointsIn2d);
    return LaneLineStringFeature::computeFeatureVector(onlyPoints, pointsIn2d);
  }
  VectorXd default_computeFeatureVector(bool onlyPoints, bool pointsIn2d) const {
    return this->LaneLineStringFeature::computeFeatureVector(onlyPoints, pointsIn2d);
  }
  bool process(const OrientedRect &bbox, const ParametrizationType &paramType, int32_t nPoints) {
    if (override f = this->get_override("process")) return f(bbox, paramType, nPoints);
    return LaneLineStringFeature::process(bbox, paramType, nPoints);
  }
  bool default_process(const OrientedRect &bbox, const ParametrizationType &paramType, int32_t nPoints) {
    return this->LaneLineStringFeature::process(bbox, paramType, nPoints);
  }
  MatrixXd pointMatrix(bool pointsIn2d) const {
    if (override f = this->get_override("pointMatrix")) return f(pointsIn2d);
    return LaneLineStringFeature::pointMatrix(pointsIn2d);
  }
  MatrixXd default_pointMatrix(bool pointsIn2d) const { return this->LaneLineStringFeature::pointMatrix(pointsIn2d); }
};

class CompoundLaneLineStringFeatureWrap : public CompoundLaneLineStringFeature,
                                          public wrapper<CompoundLaneLineStringFeature> {
 public:
  CompoundLaneLineStringFeatureWrap() {}

  CompoundLaneLineStringFeatureWrap(const LaneLineStringFeatureList &features, LineStringType compoundType)
      : CompoundLaneLineStringFeature(features, compoundType) {}

  VectorXd computeFeatureVector(bool onlyPoints, bool pointsIn2d) const {
    if (override f = this->get_override("computeFeatureVector")) return f(onlyPoints, pointsIn2d);
    return CompoundLaneLineStringFeature::computeFeatureVector(onlyPoints, pointsIn2d);
  }
  VectorXd default_computeFeatureVector(bool onlyPoints, bool pointsIn2d) const {
    return this->CompoundLaneLineStringFeature::computeFeatureVector(onlyPoints, pointsIn2d);
  }
  bool process(const OrientedRect &bbox, const ParametrizationType &paramType, int32_t nPoints) {
    if (override f = this->get_override("process")) return f(bbox, paramType, nPoints);
    return CompoundLaneLineStringFeature::process(bbox, paramType, nPoints);
  }
  bool default_process(const OrientedRect &bbox, const ParametrizationType &paramType, int32_t nPoints) {
    return this->CompoundLaneLineStringFeature::process(bbox, paramType, nPoints);
  }
  MatrixXd pointMatrix(bool pointsIn2d) const {
    if (override f = this->get_override("pointMatrix")) return f(pointsIn2d);
    return CompoundLaneLineStringFeature::pointMatrix(pointsIn2d);
  }
  MatrixXd default_pointMatrix(bool pointsIn2d) const {
    return this->CompoundLaneLineStringFeature::pointMatrix(pointsIn2d);
  }
};

class LaneletFeatureWrap : public LaneletFeature, public wrapper<LaneletFeature> {
 public:
  LaneletFeatureWrap() {}

  LaneletFeatureWrap(LaneLineStringFeaturePtr leftBoundary, LaneLineStringFeaturePtr rightBoundary,
                     LaneLineStringFeaturePtr centerline, Id mapID)
      : LaneletFeature(leftBoundary, rightBoundary, centerline, mapID) {}
  LaneletFeatureWrap(const ConstLanelet &ll) : LaneletFeature(ll) {}

  VectorXd computeFeatureVector(bool onlyPoints, bool pointsIn2d) const {
    if (override f = this->get_override("computeFeatureVector")) return f(onlyPoints, pointsIn2d);
    return LaneletFeature::computeFeatureVector(onlyPoints, pointsIn2d);
  }
  VectorXd default_computeFeatureVector(bool onlyPoints, bool pointsIn2d) const {
    return this->LaneletFeature::computeFeatureVector(onlyPoints, pointsIn2d);
  }
  bool process(const OrientedRect &bbox, const ParametrizationType &paramType, int32_t nPoints) {
    if (override f = this->get_override("process")) return f(bbox, paramType, nPoints);
    return LaneletFeature::process(bbox, paramType, nPoints);
  }
  bool default_process(const OrientedRect &bbox, const ParametrizationType &paramType, int32_t nPoints) {
    return this->LaneletFeature::process(bbox, paramType, nPoints);
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
      .value("Mixed", LineStringType::Mixed)
      .value("Virtual", LineStringType::Virtual)
      .value("Centerline", LineStringType::Centerline)
      .value("Unknown", LineStringType::Unknown);

  class_<OrientedRect>("OrientedRect", "Oriented rectangle for feature crop area", init<>())
      .add_property("bounds", make_function(&OrientedRect::bounds_const, return_value_policy<copy_const_reference>()));

  def("getRotatedRect", &getRotatedRect);
  def("extractSubmap", &extractSubmap);
  def("isRoadBorder", &isRoadBorder);
  def("bdTypeToEnum", &bdTypeToEnum);
  def("teTypeToEnum", &teTypeToEnum);
  def("resampleLineString", &resampleLineString);
  def("cutLineString", &cutLineString);
  def("saveLaneData", &saveLaneData);
  def("loadLaneData", &loadLaneData);

  class_<MapFeatureWrap, boost::noncopyable>("MapFeature", "Abstract base map feature class", no_init)
      .add_property("wasCut", &MapFeature::wasCut)
      .add_property("mapID", &MapFeature::mapID)
      .add_property("initialized", &MapFeature::initialized)
      .add_property("valid", &MapFeature::valid)
      .def("computeFeatureVector", pure_virtual(&MapFeature::computeFeatureVector))
      .def("process", pure_virtual(&MapFeature::process));

  class_<LineStringFeatureWrap, bases<MapFeature>, boost::noncopyable>("LineStringFeature",
                                                                       "Abstract line string feature class", no_init)
      .add_property("rawFeature",
                    make_function(&LineStringFeature::rawFeature, return_value_policy<copy_const_reference>()))
      .def("computeFeatureVector", pure_virtual(&LineStringFeature::computeFeatureVector))
      .def("process", pure_virtual(&LineStringFeature::process))
      .def("pointMatrix", pure_virtual(&LineStringFeature::pointMatrix));

  class_<LaneLineStringFeatureWrap, bases<LineStringFeature>, LaneLineStringFeaturePtr, boost::noncopyable>(
      "LaneLineStringFeature", "Lane line string feature class", init<BasicLineString3d, Id, LineStringType, Id>())
      .def(init<>())
      .add_property("cutFeature",
                    make_function(&LaneLineStringFeature::cutFeature, return_value_policy<copy_const_reference>()))
      .add_property("cutAndResampledFeature", make_function(&LaneLineStringFeature::cutAndResampledFeature,
                                                            return_value_policy<copy_const_reference>()))
      .add_property("type", &LaneLineStringFeature::type)
      .add_property("typeInt", &LaneLineStringFeature::typeInt)
      .add_property("laneletIDs",
                    make_function(&LaneLineStringFeature::laneletIDs, return_value_policy<copy_const_reference>()))
      .add_property("addLaneletID", &LaneLineStringFeature::addLaneletID)
      .def("computeFeatureVector", &LaneLineStringFeature::computeFeatureVector,
           &LaneLineStringFeatureWrap::default_computeFeatureVector)
      .def("process", &LaneLineStringFeature::process, &LaneLineStringFeatureWrap::default_process)
      .def("pointMatrix", &LaneLineStringFeature::pointMatrix, &LaneLineStringFeatureWrap::default_pointMatrix);

  class_<LaneletFeatureWrap, bases<MapFeature>, LaneletFeaturePtr, boost::noncopyable>(
      "LaneletFeature", "Lanelet feature class that contains lower level LaneLineStringFeatures",
      init<LaneLineStringFeaturePtr, LaneLineStringFeaturePtr, LaneLineStringFeaturePtr, Id>())
      .def(init<ConstLanelet>())
      .def(init<>())
      .add_property("leftBoundary", make_function(&LaneletFeature::leftBoundary))
      .add_property("rightBoundary", make_function(&LaneletFeature::rightBoundary))
      .add_property("centerline", make_function(&LaneletFeature::centerline))
      .def("setReprType", &LaneletFeature::setReprType)
      .def("computeFeatureVector", &LaneletFeature::computeFeatureVector,
           &LaneletFeatureWrap::default_computeFeatureVector)
      .def("process", &LaneletFeature::process, &LaneletFeatureWrap::default_process);

  class_<CompoundLaneLineStringFeatureWrap, bases<LaneLineStringFeature>, CompoundLaneLineStringFeaturePtr,
         boost::noncopyable>("CompoundLaneLineStringFeature",
                             "Compound lane line string feature class that can trace back the individual features",
                             init<LaneLineStringFeatureList, LineStringType>())
      .def(init<>())
      .add_property("features", make_function(&CompoundLaneLineStringFeature::features))
      .add_property("pathLengthsRaw", make_function(&CompoundLaneLineStringFeature::pathLengthsRaw,
                                                    return_value_policy<copy_const_reference>()))
      .add_property("pathLengthsProcessed", make_function(&CompoundLaneLineStringFeature::pathLengthsProcessed,
                                                          return_value_policy<copy_const_reference>()))
      .add_property("processedFeaturesValid", make_function(&CompoundLaneLineStringFeature::processedFeaturesValid,
                                                            return_value_policy<copy_const_reference>()))
      .def("computeFeatureVector", &CompoundLaneLineStringFeature::computeFeatureVector,
           &CompoundLaneLineStringFeatureWrap::default_computeFeatureVector)
      .def("process", &CompoundLaneLineStringFeature::process, &CompoundLaneLineStringFeatureWrap::default_process)
      .def("pointMatrix", &CompoundLaneLineStringFeature::pointMatrix,
           &CompoundLaneLineStringFeatureWrap::default_pointMatrix);

  class_<Edge>("Edge", "Struct of a lane graph edge", init<Id, Id>())
      .def(init<>())
      .add_property("el1", &Edge::el1_)
      .add_property("el2", &Edge::el2_);

  class_<LaneData>("LaneData", "Class for holding, accessing and processing of lane data")
      .def(init<>())
      .def("build", &LaneData::build)
      .staticmethod("build")
      .def("processAll", &LaneData::processAll)
      .add_property("roadBorders", make_function(&LaneData::roadBorders, return_value_policy<copy_const_reference>()))
      .add_property("laneDividers", make_function(&LaneData::laneDividers, return_value_policy<copy_const_reference>()))
      .add_property("compoundRoadBorders",
                    make_function(&LaneData::compoundRoadBorders, return_value_policy<copy_const_reference>()))
      .add_property("compoundLaneDividers",
                    make_function(&LaneData::compoundLaneDividers, return_value_policy<copy_const_reference>()))
      .add_property("compoundCenterlines",
                    make_function(&LaneData::compoundCenterlines, return_value_policy<copy_const_reference>()))
      .add_property("associatedCpdRoadBorderIndices", make_function(&LaneData::associatedCpdRoadBorderIndices,
                                                                    return_value_policy<copy_const_reference>()))
      .add_property("associatedCpdLaneDividerIndices", make_function(&LaneData::associatedCpdLaneDividerIndices,
                                                                     return_value_policy<copy_const_reference>()))
      .add_property("associatedCpdCenterlineIndices", make_function(&LaneData::associatedCpdCenterlineIndices,
                                                                    return_value_policy<copy_const_reference>()))
      .add_property("laneletFeatures",
                    make_function(&LaneData::laneletFeatures, return_value_policy<copy_const_reference>()))
      .add_property("edges", make_function(&LaneData::edges, return_value_policy<copy_const_reference>()));

  {
    scope inMapDataInterface =
        class_<MapDataInterface>("MapDataInterface", "Main Interface Class for processing of Lanelet maps",
                                 init<LaneletMapConstPtr>())
            .def(init<LaneletMapConstPtr, MapDataInterface::Configuration>())
            .add_property("config",
                          make_function(&MapDataInterface::config, return_value_policy<copy_const_reference>()))
            .def("setCurrPosAndExtractSubmap", &MapDataInterface::setCurrPosAndExtractSubmap)
            .def("laneData", &MapDataInterface::laneData)
            .def("teData", &MapDataInterface::teData)
            .def("laneDataBatch", &MapDataInterface::laneDataBatch)
            .def("laneTEDataBatch", &MapDataInterface::laneTEDataBatch);

    class_<MapDataInterface::Configuration>("Configuration", "Configuration class for MapDataInterface", init<>())
        .def(init<LaneletRepresentationType, ParametrizationType, double, double, int>())
        .def_readwrite("reprType", &MapDataInterface::Configuration::reprType)
        .def_readwrite("paramType", &MapDataInterface::Configuration::paramType)
        .def_readwrite("submapExtentLongitudinal", &MapDataInterface::Configuration::submapExtentLongitudinal)
        .def_readwrite("submapExtentLateral", &MapDataInterface::Configuration::submapExtentLateral)
        .def_readwrite("nPoints", &MapDataInterface::Configuration::nPoints);
  }

  // Eigen, stl etc. converters
  converters::convertMatrix<MatrixXd>(true);
  converters::convertVector<VectorXd>(true);

  converters::VectorToListConverter<std::vector<MatrixXd>>();
  converters::VectorToListConverter<LaneLineStringFeatureList>();
  converters::VectorToListConverter<CompoundLaneLineStringFeatureList>();
  converters::VectorToListConverter<
      boost::geometry::model::ring<BasicPoint2d, true, true, std::vector, std::allocator>>();
  converters::IterableConverter()
      .fromPython<std::vector<MatrixXd>>()
      .fromPython<BasicLineString3d>()
      .fromPython<LaneLineStringFeatureList>()
      .fromPython<CompoundLaneLineStringFeatureList>();
  class_<LaneLineStringFeatures>("LaneLineStringFeatures").def(map_indexing_suite<LaneLineStringFeatures>());
  class_<LaneletFeatures>("LaneletFeatures").def(map_indexing_suite<LaneletFeatures>());
  converters::VectorToListConverter<std::vector<double>>();
  class_<std::vector<bool>>("BoolList").def(vector_indexing_suite<std::vector<bool>>());
  // class_<LaneLineStringFeatureList>("LaneLineStringFeatureList")
  // .def(vector_indexing_suite<LaneLineStringFeatureList>());
  // class_<CompoundLaneLineStringFeatureList>("CompoundLaneLineStringFeatureList")
  // .def(vector_indexing_suite<CompoundLaneLineStringFeatureList>());

  implicitly_convertible<routing::RoutingGraphPtr, routing::RoutingGraphConstPtr>();

  // overloaded template function instantiations
  def<std::vector<MatrixXd> (*)(const std::map<Id, LaneLineStringFeaturePtr> &, bool)>(
      "getPointsMatrices", &getPointsMatrices<LaneLineStringFeature>);
  def<std::vector<MatrixXd> (*)(const std::vector<LaneLineStringFeaturePtr> &, bool)>(
      "getPointsMatrices", &getPointsMatrices<LaneLineStringFeature>);
  def<std::vector<MatrixXd> (*)(const std::map<Id, CompoundLaneLineStringFeaturePtr> &, bool)>(
      "getPointsMatrices", &getPointsMatrices<CompoundLaneLineStringFeature>);
  def<std::vector<MatrixXd> (*)(const std::vector<CompoundLaneLineStringFeaturePtr> &, bool)>(
      "getPointsMatrices", &getPointsMatrices<CompoundLaneLineStringFeature>);

  def<MatrixXd (*)(const std::map<Id, LaneLineStringFeaturePtr> &, bool, bool)>(
      "getFeatureVectorMatrix", &getFeatureVectorMatrix<LaneLineStringFeature>);
  def<MatrixXd (*)(const std::vector<LaneLineStringFeaturePtr> &, bool, bool)>(
      "getFeatureVectorMatrix", &getFeatureVectorMatrix<LaneLineStringFeature>);
  def<MatrixXd (*)(const std::map<Id, CompoundLaneLineStringFeaturePtr> &, bool, bool)>(
      "getFeatureVectorMatrix", &getFeatureVectorMatrix<CompoundLaneLineStringFeature>);
  def<MatrixXd (*)(const std::vector<CompoundLaneLineStringFeaturePtr> &, bool, bool)>(
      "getFeatureVectorMatrix", &getFeatureVectorMatrix<CompoundLaneLineStringFeature>);
  def<MatrixXd (*)(const std::map<Id, LaneletFeaturePtr> &, bool, bool)>("getFeatureVectorMatrix",
                                                                         &getFeatureVectorMatrix<LaneletFeature>);
  def<MatrixXd (*)(const std::vector<LaneletFeaturePtr> &, bool, bool)>("getFeatureVectorMatrix",
                                                                        &getFeatureVectorMatrix<LaneletFeature>);

  def<bool (*)(std::map<Id, LaneLineStringFeaturePtr> &, const OrientedRect &, const ParametrizationType &, int32_t)>(
      "processFeatures", &processFeatures<LaneLineStringFeature>);
  def<bool (*)(std::vector<LaneLineStringFeaturePtr> &, const OrientedRect &, const ParametrizationType &, int32_t)>(
      "processFeatures", &processFeatures<LaneLineStringFeature>);
  def<bool (*)(std::map<Id, CompoundLaneLineStringFeaturePtr> &, const OrientedRect &, const ParametrizationType &,
               int32_t)>("processFeatures", &processFeatures<CompoundLaneLineStringFeature>);
  def<bool (*)(std::vector<CompoundLaneLineStringFeaturePtr> &, const OrientedRect &, const ParametrizationType &,
               int32_t)>("processFeatures", &processFeatures<CompoundLaneLineStringFeature>);
  def<bool (*)(std::map<Id, LaneletFeaturePtr> &, const OrientedRect &, const ParametrizationType &, int32_t)>(
      "processFeatures", &processFeatures<LaneletFeature>);
  def<bool (*)(std::vector<LaneletFeaturePtr> &, const OrientedRect &, const ParametrizationType &, int32_t)>(
      "processFeatures", &processFeatures<LaneletFeature>);
}
