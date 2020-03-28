#pragma once

#include <algorithm>
#include <functional>
#include <random>
#include <stdexcept>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SVD>

#include <external/nanoflann.hpp>

namespace pICP
{

template< typename TCoordinate, unsigned int TDimension = 3 >
class IterativeClosestPoint
{
  public:
    typedef Eigen::Matrix< TCoordinate, TDimension, Eigen::Dynamic > CoordinatesMatrixType;
    typedef Eigen::Matrix< TCoordinate, TDimension, TDimension >     RotationMatrixType;
    typedef Eigen::Matrix< TCoordinate, TDimension, 1 >              TranslationVectorType;

    IterativeClosestPoint();
    ~IterativeClosestPoint() = default;

    void SetIterations( const unsigned int iterations );
    void SetSamplingRatio( const double samplingRatio );
    void SetTolerance( const TCoordinate tolerance );
    void SetVerbose( const bool verbose );

    void SetSourceCoordinatesMatrix( const CoordinatesMatrixType & source );
    void SetTargetCoordinatesMatrix( const CoordinatesMatrixType & target );

    // Main method
    void Align();

    RotationMatrixType GetRotationMatrix();
    TranslationVectorType GetTranslationVector();

  private:
    typedef nanoflann::KDTreeEigenMatrixAdaptor<
        Eigen::Matrix< TCoordinate, Eigen::Dynamic, TDimension >> KDTreeType;
    typedef std::vector< unsigned int >                           IndicesVectorType;

    // Algorithm parameters
    unsigned int _iterations;    // Maximum estimation iterations
    double       _samplingRatio; // Sampling ratio (0-1) for data sampling in transform estimation
    TCoordinate  _tolerance;     // Error tolerance, expressed in average squared error in coordinate units
    bool         _verbose;       // Output current iteration error and transform estimations

    // Data variables
    CoordinatesMatrixType _sourceCoordinatesMatrix;
    CoordinatesMatrixType _mutableSourceCoordinatesMatrix;
    CoordinatesMatrixType _targetCoordinatesMatrix;

    // Internal variables
    std::shared_ptr< KDTreeType > _KDTree;
    std::mt19937                  _randomGenerator;
    IndicesVectorType             _sourceCoordinateIds;
    RotationMatrixType            _rotationMatrix;
    TranslationVectorType         _translationVector;

    void CheckValidDimensions( const CoordinatesMatrixType & coordinatesMatrix ) const;
    void CheckDataAvailable() const;

    // Internal methods
    CoordinatesMatrixType SampleCoordinatesMatrix( const CoordinatesMatrixType & coordinatesMatrix,
                                                   const IndicesVectorType & indices );
    void ComputeSourceToTargetCorrespondences( IndicesVectorType & sourceIds, IndicesVectorType & targetIds );
    void ComputeTransform( const CoordinatesMatrixType & source, const CoordinatesMatrixType & target,
                           RotationMatrixType & rotationMatrix, TranslationVectorType & translationVector );
};

} // namespace pICP

#include "pICP/ICP.hpp"
