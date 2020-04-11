#include "gtest/gtest.h"

#include <cmath> 
#include <random>

#include "pICP/PointCloud.h"

namespace
{

TEST(pointcloud, cloud_2d)
{
    std::random_device rd;
    std::mt19937 e2(rd());
    std::uniform_real_distribution<> circumferenceAngles(0, 2. * M_PI);

    using PointCloud2D = pICP::PointCloud< float, 2 >;

    // Generate coordinates on a random circumference
    PointCloud2D::CoordinatesMatrixType coordinates(2, 20);
    float r = 10.;
    for(size_t pIt = 0; pIt < coordinates.cols(); pIt++)
    {
        float angle = circumferenceAngles(e2);
        coordinates(0, pIt) = r * std::cos( angle );
        coordinates(1, pIt) = r * std::sin( angle );
    }

    PointCloud2D pointcloud2D;
    pointcloud2D.UpdateCoordinatesMatrix( coordinates );

    EXPECT_TRUE( pointcloud2D.SelectCoordinates({10}) == coordinates.col(10) );

    EXPECT_THROW( pointcloud2D.WriteOBJ( "/foo.obj" ), std::runtime_error );
}

TEST(pointcloud, cloud_3d)
{
    std::random_device rd;
    std::mt19937 e2(rd());
    std::uniform_real_distribution<> randomPointCoordinate(0., 1.);

    using PointCloud3D = pICP::PointCloud< float, 3 >;

    // Generate coordinates on a random sphere
    PointCloud3D::CoordinatesMatrixType coordinates(3, 100);
    float r = 10.;
    for(size_t pIt = 0; pIt < coordinates.cols(); pIt++)
    {
        coordinates.col(pIt) = r * Eigen::Vector3f( randomPointCoordinate(e2), randomPointCoordinate(e2), randomPointCoordinate(e2) ).normalized();
    }

    PointCloud3D pointcloud3D;
    pointcloud3D.UpdateCoordinatesMatrix( coordinates );

    EXPECT_TRUE( pointcloud3D.SelectCoordinates({0}) == coordinates.col(0) );
}

}  // namespace
