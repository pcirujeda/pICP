#include "gtest/gtest.h"

#include <math.h>

#include <Eigen/Core>

#include "pICP/utils.h"

namespace
{

TEST(utils, translate_2d)
{
    Eigen::Matrix< float, 2, 5 > coordinates;
    coordinates << 1., 2., 3., 4., 5.,
                   1., 2., 3., 4., 5.;

    Eigen::Matrix< float, 2, 2 > rotation;
    rotation.setIdentity();

    Eigen::Matrix< float, 2, 1 > translation;
    translation << 10., 10;

    Eigen::Matrix< float, 2, 5 > translatedCoordinates = pICP::transformCoordinatesMatrix( coordinates, rotation, translation );

    Eigen::Matrix< float, 2, 5 > expectedCoordinates;
    expectedCoordinates << 11., 12., 13., 14., 15.,
                           11., 12., 13., 14., 15.;

    EXPECT_TRUE( translatedCoordinates == expectedCoordinates );
}

TEST(utils, rotate_2d)
{
    Eigen::Matrix< float, 2, 5 > coordinates;
    coordinates <<  1.,  2.,  3.,  4.,  5.,
                   11., 22., 33., 44., 55.;

    Eigen::Matrix< float, 2, 2 > rotation;
    rotation << std::cos( M_PI/2. ), -std::sin( M_PI/2. ),
                std::sin( M_PI/2.),   std::cos( M_PI/2. );

    Eigen::Matrix< float, 2, 1 > translation;
    translation << 0., 0;

    Eigen::Matrix< float, 2, 5 > translatedCoordinates = pICP::transformCoordinatesMatrix( coordinates, rotation, translation );

    Eigen::Matrix< float, 2, 5 > expectedCoordinates;
    expectedCoordinates << -11., -22., -33., -44., -55.,
                             1.,   2.,   3.,   4.,   5.;

    EXPECT_TRUE( translatedCoordinates == expectedCoordinates );
}

TEST(utils, transform_3d)
{
    Eigen::Matrix< float, 3, 5 > coordinates;
    coordinates << 1., 2., 3., 4., 5.,
                   1., 2., 3., 4., 5.,
                   1., 2., 3., 4., 5.;

    Eigen::Matrix< float, 3, 3 > rotation;
    rotation << 1.,                  0.,                   0.,               
                0., std::cos( M_PI/2. ), -std::sin( M_PI/2. ),
                0.,  std::sin( M_PI/2.),  std::cos( M_PI/2. );

    Eigen::Matrix< float, 3, 1 > translation;
    translation << 1., 2., 3;

    Eigen::Matrix< float, 3, 5 > translatedCoordinates = pICP::transformCoordinatesMatrix( coordinates, rotation, translation );

    Eigen::Matrix< float, 3, 5 > expectedCoordinates;
    expectedCoordinates <<  2., 3.,  4.,  5.,  6.,
                            1., 0., -1., -2., -3.,
                            4., 5.,  6.,  7.,  8.;

    EXPECT_TRUE( translatedCoordinates == expectedCoordinates );
}

TEST(utils, coordinatesDifferenceNorm_equality)
{
    Eigen::Matrix< float, 3, 5 > coordinates;
    coordinates << 1., 2., 3., 4., 5.,
                   1., 2., 3., 4., 5.,
                   1., 2., 3., 4., 5.;

    Eigen::Matrix< float, 3, 5 > sameCoordinates = coordinates;

    EXPECT_EQ(pICP::coordinatesDifferenceNorm( coordinates, sameCoordinates ), 0. );
}

TEST(utils, coordinatesDifferenceNorm_inequality)
{
    Eigen::Matrix< float, 3, 5 > coordinates;
    coordinates << 1., 2., 3., 4., 5.,
                   1., 2., 3., 4., 5.,
                   1., 2., 3., 4., 5.;

    Eigen::Matrix< float, 3, 5 > differentCoordinates = coordinates;
    //differentCoordinates( 1, 3 ) += 2.;
    differentCoordinates( 2, 4 ) += 3.;

    EXPECT_EQ(pICP::coordinatesDifferenceNorm( coordinates, differentCoordinates ), 3. );
}

}  // namespace
