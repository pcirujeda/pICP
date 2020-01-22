#pragma once

#include "opencv2/opencv.hpp"

#include <algorithm>
#include <random>
#include <stdexcept>

template< typename TCoordinate, unsigned int Dimension >
class IterativeClosestPoint
{
  public:
    typedef TCoordinate                CoordinateType;
    typedef cv::Mat_< CoordinateType > CoordinatesMatrixType;

    IterativeClosestPoint();
    ~IterativeClosestPoint();

    void SetIterations( const unsigned int iterations );
    void SetSamplingRatio( const CoordinateType samplingRatio );
    void SetTolerance( const CoordinateType tolerance );
    void SetVerbose( const bool verbose );

    void SetSourceCoordinatesMatrix( const CoordinatesMatrixType & source );
    void SetTargetCoordinatesMatrix( const CoordinatesMatrixType & target );

    // Main method
    void Align();

    CoordinatesMatrixType GetRotationMatrix();
    CoordinatesMatrixType GetTranslationVector();

  private:
    typedef cv::Mat_< int > IndicesVectorType;

    // Algorithm parameters
    unsigned int   _iterations;    // Maximum estimation iterations
    CoordinateType _samplingRatio; // Sampling ratio (0-1) for data sampling in transform estimation
    CoordinateType _tolerance;     // Error tolerance, expressed in average squared error in coordinate units
    bool           _verbose;       // Output current iteration error and transform estimations

    // Data variables
    CoordinatesMatrixType _sourceCoordinatesMatrix;
    CoordinatesMatrixType _mutableSourceCoordinatesMatrix;
    CoordinatesMatrixType _targetCoordinatesMatrix;

    // Internal variables
    cv::Ptr< cv::flann::Index > _KDTree;
    std::mt19937                _randomGenerator;
    std::vector< int >          _sourceCoordinateIds;
    CoordinatesMatrixType       _rotationMatrix;
    CoordinatesMatrixType       _translationVector;

    void CheckValidDimensions( const CoordinatesMatrixType & coordinatesMatrix ) const;
    void CheckDataAvailable() const;

    // Internal methods
    CoordinatesMatrixType SampleCoordinatesMatrix( const CoordinatesMatrixType & coordinatesMatrix,
                                                   const IndicesVectorType & indices );
    void ComputeSourceToTargetCorrespondences( IndicesVectorType & sourceIds, IndicesVectorType & targetIds );
    void ComputeTransform( const CoordinatesMatrixType & source, const CoordinatesMatrixType & target,
                           CoordinatesMatrixType & rotationMatrix, CoordinatesMatrixType & translationVector );

};

#include "pICP/ICP.hxx"
