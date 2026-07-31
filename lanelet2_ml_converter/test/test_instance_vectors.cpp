/// Tests for the conversion of processed instances into the flat vector / matrix representations that
/// are handed out as tensors.

#include <gtest/gtest.h>

#include <vector>

#include "lanelet2_ml_converter/MapInstances.h"
#include "lanelet2_ml_converter/Utils.h"

using namespace lanelet;
using namespace lanelet::ml_converter;

namespace lanelet {
namespace ml_converter {
// defined in MapInstances.cpp, deliberately not part of the public header
VectorXd stackVector(const std::vector<VectorXd>& vec);
}  // namespace ml_converter
}  // namespace lanelet

TEST(MLConverterInstanceVectors, StackVectorConcatenatesAllParts) {  // NOLINT
  VectorXd first(2);
  first << 1, 2;
  VectorXd second(3);
  second << 3, 4, 5;
  VectorXd third(2);
  third << 6, 7;

  VectorXd stacked = stackVector(std::vector<VectorXd>{first, second, third});
  ASSERT_EQ(stacked.size(), 7);
  for (int i = 0; i < stacked.size(); i++) {
    EXPECT_NEAR(stacked[i], i + 1, 1e-9) << "index " << i;
  }
}

TEST(MLConverterInstanceVectors, InstanceVectorMatrixRequiresEqualLengths) {  // NOLINT
  OrientedRect bbox = getRotatedRect(BasicPoint3d{0, 0, 0}, 15, 10, 0, true);
  BasicLineString3d line1{BasicPoint3d{-10, 0, 0}, BasicPoint3d{10, 0, 0}};
  BasicLineString3d line2{BasicPoint3d{-10, 1, 0}, BasicPoint3d{10, 1, 0}};
  LaneLineStringInstancePtr feat1 =
      std::make_shared<LaneLineStringInstance>(line1, Id(1), LineStringType::Solid, Ids{1}, false);
  LaneLineStringInstancePtr feat2 =
      std::make_shared<LaneLineStringInstance>(line2, Id(2), LineStringType::Solid, Ids{2}, false);

  ASSERT_TRUE(feat1->process(bbox, ParametrizationType::LineString, 5));
  ASSERT_TRUE(feat2->process(bbox, ParametrizationType::LineString, 5));

  std::vector<LaneLineStringInstancePtr> instances{feat1, feat2};
  MatrixXd mat;
  ASSERT_NO_THROW(mat = getInstanceVectorMatrix(instances, true, true));
  EXPECT_EQ(mat.rows(), 2);
  EXPECT_EQ(mat.cols(), 10);  // 5 points with 2 dimensions each

  // differing point counts cannot be stacked into one matrix and must be rejected
  ASSERT_TRUE(feat2->process(bbox, ParametrizationType::LineString, 7));
  EXPECT_THROW(getInstanceVectorMatrix(instances, true, true), std::runtime_error);
}

TEST(MLConverterInstanceVectors, PointMatrixAndInstanceVectorDimensions) {  // NOLINT
  BasicLineString3d line{BasicPoint3d{1, 2, 3}, BasicPoint3d{4, 5, 6}};

  MatrixXd mat2d = toPointMatrix(line, true);
  ASSERT_EQ(mat2d.rows(), 2);
  ASSERT_EQ(mat2d.cols(), 2);
  EXPECT_NEAR(mat2d(1, 0), 4, 1e-9);

  MatrixXd mat3d = toPointMatrix(line, false);
  ASSERT_EQ(mat3d.cols(), 3);
  EXPECT_NEAR(mat3d(1, 2), 6, 1e-9);

  VectorXd withType = toInstanceVector(line, 7, false, true);
  ASSERT_EQ(withType.size(), 5);  // 2 points with 2 dimensions each + type
  EXPECT_NEAR(withType[4], 7, 1e-9);

  VectorXd onlyPoints = toInstanceVector(line, 7, true, false);
  ASSERT_EQ(onlyPoints.size(), 6);  // 2 points with 3 dimensions each
  EXPECT_NEAR(onlyPoints[5], 6, 1e-9);
}
