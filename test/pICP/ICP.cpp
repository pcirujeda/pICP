#include "gtest/gtest.h"

#include <cmath> 
#include <random>

#include "pICP/ICP.h"

namespace
{

TEST(icp, data_unavailable)
{
    pICP::IterativeClosestPoint< float, 3 > icp;

    EXPECT_THROW( icp.Align(), std::runtime_error );
}

TEST(icp, transform_not_computed)
{
    using ICP = pICP::IterativeClosestPoint< float, 2 >;

    ICP icp2D;

    ICP::CoordinatesMatrixType coordinates = ICP::CoordinatesMatrixType::Random( 2, 50 );
    icp2D.SetSourceCoordinatesMatrix( coordinates );
    icp2D.SetTargetCoordinatesMatrix( coordinates );

    EXPECT_THROW( icp2D.GetRotationMatrix(), std::runtime_error );
    EXPECT_THROW( icp2D.GetTranslationVector(), std::runtime_error );
}


TEST(icp, registration_2d_identity)
{
    using ICP = pICP::IterativeClosestPoint< float, 2 >;

    ICP icp2D;
    icp2D.SetSamplingRatio( 0.75 );
    icp2D.SetTolerance( 0.01 );

    ICP::CoordinatesMatrixType coordinates = ICP::CoordinatesMatrixType::Random( 2, 50 );

    icp2D.SetSourceCoordinatesMatrix( coordinates );
    icp2D.SetTargetCoordinatesMatrix( coordinates );
    icp2D.Align();

    EXPECT_TRUE( (icp2D.GetRotationMatrix() - ICP::RotationMatrixType::Identity( 2, 2 )).norm() < 1E-3f );
    EXPECT_TRUE( (icp2D.GetTranslationVector() - ICP::TranslationVectorType::Zero()).norm() < 1E-3f );
}

TEST(icp, registration_2d_random_translation)
{
    using ICP = pICP::IterativeClosestPoint< float, 2 >;

    ICP icp2D;
    icp2D.SetSamplingRatio( 0.75 );
    icp2D.SetTolerance( 0.01 );

    ICP::CoordinatesMatrixType coordinates = Eigen::MatrixXf::Random( 2, 100 );

    ICP::RotationMatrixType appliedRotation = ICP::RotationMatrixType::Identity( 2, 2 );
    ICP::TranslationVectorType appliedTranslation = ICP::TranslationVectorType::Random();

    ICP::CoordinatesMatrixType translatedCoordinates = pICP::transformCoordinatesMatrix( coordinates, appliedRotation, appliedTranslation );

    icp2D.SetSourceCoordinatesMatrix( coordinates );
    icp2D.SetTargetCoordinatesMatrix( translatedCoordinates );
    icp2D.Align();

    EXPECT_TRUE( (icp2D.GetRotationMatrix() - appliedRotation).norm() < 1E-3f );
    EXPECT_TRUE( (icp2D.GetTranslationVector() - appliedTranslation).norm() < 1E-3f );
}

TEST(icp, registration_2d_random_rotation)
{
    using ICP = pICP::IterativeClosestPoint< float, 2 >;

    std::random_device rd;
    std::mt19937 e2(rd());
    std::uniform_real_distribution<> circumferenceAngles(0, 2. * M_PI);
    std::uniform_real_distribution<> randomRotationAngle(-0.05, 0.05); // -3º - 3º

    ICP icp2D;
    icp2D.SetSamplingRatio( 0.35 );
    icp2D.SetTolerance( 0.01 );

    // Generate coordinates on a random circumference
    ICP::CoordinatesMatrixType coordinates(2, 20);
    float r = 10.;
    for(size_t pIt = 0; pIt < coordinates.cols(); pIt++)
    {
        float angle = circumferenceAngles(e2);
        coordinates(0, pIt) = r * std::cos( angle );
        coordinates(1, pIt) = r * std::sin( angle );
    }

    ICP::RotationMatrixType appliedRotation = Eigen::Rotation2D<float>( randomRotationAngle(e2) ).toRotationMatrix();
    ICP::TranslationVectorType appliedTranslation = ICP::TranslationVectorType::Zero();

    ICP::CoordinatesMatrixType translatedCoordinates = pICP::transformCoordinatesMatrix( coordinates, appliedRotation, appliedTranslation );

    icp2D.SetSourceCoordinatesMatrix( coordinates );
    icp2D.SetTargetCoordinatesMatrix( translatedCoordinates );
    icp2D.Align();

    EXPECT_TRUE( (icp2D.GetRotationMatrix() - appliedRotation).norm() < 1E-3f );
    EXPECT_TRUE( (icp2D.GetTranslationVector() - appliedTranslation).norm() < 1E-3f );
}

TEST(icp, registration_2d_random_transform)
{
    using ICP = pICP::IterativeClosestPoint< float, 2 >;

    std::random_device rd;
    std::mt19937 e2(rd());
    std::uniform_real_distribution<> circumferenceAngles(0, 2. * M_PI);
    std::uniform_real_distribution<> randomRotationAngle(-0.05, 0.05); // -3º - 3º

    ICP icp2D;
    icp2D.SetSamplingRatio( 0.35 );
    icp2D.SetTolerance( 0.01 );

    // Generate coordinates on a random circumference
    ICP::CoordinatesMatrixType coordinates(2, 20);
    float r = 10.;
    for(size_t pIt = 0; pIt < coordinates.cols(); pIt++)
    {
        float angle = circumferenceAngles(e2);
        coordinates(0, pIt) = r * std::cos( angle );
        coordinates(1, pIt) = r * std::sin( angle );
    }

    ICP::RotationMatrixType appliedRotation = Eigen::Rotation2D<float>( randomRotationAngle(e2) ).toRotationMatrix();
    ICP::TranslationVectorType appliedTranslation = ICP::TranslationVectorType::Random();

    ICP::CoordinatesMatrixType translatedCoordinates = pICP::transformCoordinatesMatrix( coordinates, appliedRotation, appliedTranslation );

    icp2D.SetSourceCoordinatesMatrix( coordinates );
    icp2D.SetTargetCoordinatesMatrix( translatedCoordinates );
    icp2D.Align();

    EXPECT_TRUE( (icp2D.GetRotationMatrix() - appliedRotation).norm() < 1E-3f );
    EXPECT_TRUE( (icp2D.GetTranslationVector() - appliedTranslation).norm() < 1E-3f );
}

TEST(icp, registration_3d_identity)
{
    using ICP = pICP::IterativeClosestPoint< double, 3 >;

    ICP icp3D;
    icp3D.SetSamplingRatio( 0.75 );
    icp3D.SetTolerance( 0.01 );

    ICP::CoordinatesMatrixType coordinates = ICP::CoordinatesMatrixType::Random( 3, 50 );

    icp3D.SetSourceCoordinatesMatrix( coordinates );
    icp3D.SetTargetCoordinatesMatrix( coordinates );
    icp3D.Align();

    EXPECT_TRUE( (icp3D.GetRotationMatrix() - ICP::RotationMatrixType::Identity()).norm() < 1E-3 );
    EXPECT_TRUE( (icp3D.GetTranslationVector() - ICP::TranslationVectorType::Zero()).norm() < 1E-3 );
}

TEST(icp, registration_3d_random_transform)
{
    using ICP = pICP::IterativeClosestPoint< float, 3 >;

    std::random_device rd;
    std::mt19937 e2(rd());
    std::uniform_real_distribution<> randomPointCoordinate(0., 1.);
    std::uniform_real_distribution<> randomRotationAngle(-0.05, 0.05); // -3º - 3º

    ICP icp3D;
    icp3D.SetSamplingRatio( 0.35 );
    icp3D.SetTolerance( 0.01 );

    // Generate coordinates on a random sphere
    ICP::CoordinatesMatrixType coordinates(3, 200);
    float r = 10.;
    for(size_t pIt = 0; pIt < coordinates.cols(); pIt++)
    {
        coordinates.col(pIt) = r * Eigen::Vector3f( randomPointCoordinate(e2), randomPointCoordinate(e2), randomPointCoordinate(e2) ).normalized();
    }

    ICP::RotationMatrixType appliedRotation = Eigen::AngleAxis<float>(
        randomRotationAngle(e2),
        Eigen::Vector3f(randomPointCoordinate(e2), randomPointCoordinate(e2),randomPointCoordinate(e2)).normalized() ).toRotationMatrix();

    ICP::TranslationVectorType appliedTranslation = ICP::TranslationVectorType::Random();

    ICP::CoordinatesMatrixType translatedCoordinates = pICP::transformCoordinatesMatrix( coordinates, appliedRotation, appliedTranslation );

    icp3D.SetSourceCoordinatesMatrix( coordinates );
    icp3D.SetTargetCoordinatesMatrix( translatedCoordinates );
    icp3D.Align();

    EXPECT_TRUE( (icp3D.GetRotationMatrix() - appliedRotation).norm() < 1E-3f );
    EXPECT_TRUE( (icp3D.GetTranslationVector() - appliedTranslation).norm() < 1E-3f );
}

TEST(icp, registration_3d_random_transform_and_noise)
{
    using ICP = pICP::IterativeClosestPoint< float, 3 >;

    std::random_device rd;
    std::mt19937 e2(rd());
    std::uniform_real_distribution<> randomPointCoordinate(0., 1.);
    std::uniform_real_distribution<> randomRotationAngle(-0.05, 0.05); // -3º - 3º

    ICP icp3D;
    icp3D.SetSamplingRatio( 0.35 );
    icp3D.SetTolerance( 0.005 );

    // Generate coordinates on a random sphere
    ICP::CoordinatesMatrixType coordinates(3, 200);
    float r = 10.;
    for(size_t pIt = 0; pIt < coordinates.cols(); pIt++)
    {
        coordinates.col(pIt) = r * Eigen::Vector3f( randomPointCoordinate(e2), randomPointCoordinate(e2), randomPointCoordinate(e2) ).normalized();
    }

    ICP::RotationMatrixType appliedRotation = Eigen::AngleAxis<float>(
        randomRotationAngle(e2),
        Eigen::Vector3f(randomPointCoordinate(e2), randomPointCoordinate(e2),randomPointCoordinate(e2)).normalized() ).toRotationMatrix();

    ICP::TranslationVectorType appliedTranslation = ICP::TranslationVectorType::Random();

    ICP::CoordinatesMatrixType translatedCoordinates = pICP::transformCoordinatesMatrix( coordinates, appliedRotation, appliedTranslation );

    // Add noise to coordinates
    translatedCoordinates += ( ICP::CoordinatesMatrixType::Random(3, 200) * 0.005 );

    icp3D.SetSourceCoordinatesMatrix( coordinates );
    icp3D.SetTargetCoordinatesMatrix( translatedCoordinates );
    icp3D.Align();

    EXPECT_TRUE( (icp3D.GetRotationMatrix() - appliedRotation).norm() < 5E-3f );
    EXPECT_TRUE( (icp3D.GetTranslationVector() - appliedTranslation).norm() < 5E-3f );
}

}  // namespace
