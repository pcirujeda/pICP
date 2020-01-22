#pragma once

#include "pICP/ICP.h"

template< typename TCoordinate, unsigned int Dimension >
IterativeClosestPoint< TCoordinate, Dimension >
::IterativeClosestPoint():
  _iterations( 300 ),
  _samplingRatio( 0.25 ),
  _tolerance( 0.3 ),
  _verbose( false )
{}

template< typename TCoordinate, unsigned int Dimension >
IterativeClosestPoint< TCoordinate, Dimension >
::~IterativeClosestPoint()
{
  this->_KDTree.release();
}

template< typename TCoordinate, unsigned int Dimension >
void
IterativeClosestPoint< TCoordinate, Dimension >
::SetIterations( const unsigned int iterations )
{
  this->_iterations = iterations;
}

template< typename TCoordinate, unsigned int Dimension >
void
IterativeClosestPoint< TCoordinate, Dimension >
::SetSamplingRatio( const CoordinateType samplingRatio )
{
  this->_samplingRatio = samplingRatio;
}

template< typename TCoordinate, unsigned int Dimension >
void
IterativeClosestPoint< TCoordinate, Dimension >
::SetTolerance( const CoordinateType tolerance )
{
  this->_tolerance = tolerance;
}

template< typename TCoordinate, unsigned int Dimension >
void
IterativeClosestPoint< TCoordinate, Dimension >
::SetVerbose( const bool verbose )
{
  this->_verbose = verbose;
}

template< typename TCoordinate, unsigned int Dimension >
void
IterativeClosestPoint< TCoordinate, Dimension >
::SetSourceCoordinatesMatrix( const CoordinatesMatrixType & source )
{
  this->CheckValidDimensions( source );

  this->_sourceCoordinatesMatrix = source.clone();
}

template< typename TCoordinate, unsigned int Dimension >
void
IterativeClosestPoint< TCoordinate, Dimension >
::SetTargetCoordinatesMatrix( const CoordinatesMatrixType & target )
{
  this->CheckValidDimensions( target );

  this->_targetCoordinatesMatrix = target.clone();
}

template< typename TCoordinate, unsigned int Dimension >
typename IterativeClosestPoint< TCoordinate, Dimension >::CoordinatesMatrixType
IterativeClosestPoint< TCoordinate, Dimension >
::GetRotationMatrix()
{
  if( this->_rotationMatrix.empty() )
  {
    throw std::runtime_error( "Transform not computed yet" );
  }

  return this->_rotationMatrix;
}

template< typename TCoordinate, unsigned int Dimension >
typename IterativeClosestPoint< TCoordinate, Dimension >::CoordinatesMatrixType
IterativeClosestPoint< TCoordinate, Dimension >
::GetTranslationVector()
{
  if( this->_translationVector.empty() )
  {
    throw std::runtime_error( "Transform not computed yet" );
  }

  return this->_translationVector;
}

template< typename TCoordinate, unsigned int Dimension >
void
IterativeClosestPoint< TCoordinate, Dimension >
::Align()
{
  this->CheckDataAvailable();

  // Initialize transform
  this->_rotationMatrix = cv::Mat::eye( Dimension, Dimension, CV_32F );
  this->_translationVector = cv::Mat::zeros( Dimension, 1, CV_32F );

  // Initialize a mutable copy of source coordinates matrix
  this->_mutableSourceCoordinatesMatrix = this->_sourceCoordinatesMatrix.clone();

  // Create target coordinates KDTree representation
  CoordinatesMatrixType transposedTargetCoordinates;
  cv::transpose( this->_targetCoordinatesMatrix, transposedTargetCoordinates );
  this->_KDTree.reset( new cv::flann::Index( transposedTargetCoordinates, cv::flann::LinearIndexParams() ) );

  // Prepare random sampling assets
  std::random_device rd;
  this->_randomGenerator = std::mt19937( rd() );
  this->_sourceCoordinateIds = std::vector< int >( this->_sourceCoordinatesMatrix.cols );
  std::iota( this->_sourceCoordinateIds.begin(), this->_sourceCoordinateIds.end(), 0 );

  // Iterate ICP algorithm until error convergence or maximum iterations reached
  unsigned int currentIteration = 0;
  while( currentIteration < this->_iterations )
  {
    // Compute current correspondences from source to target
    IndicesVectorType mutableSourceIds, targetIds;
    this->ComputeSourceToTargetCorrespondences( mutableSourceIds, targetIds );

    // Compute current transform between candidate correspondences
    CoordinatesMatrixType currentRotationMatrix, currentTranslationVector;
    this->ComputeTransform( this->SampleCoordinatesMatrix( this->_mutableSourceCoordinatesMatrix, mutableSourceIds ),
                            this->SampleCoordinatesMatrix( this->_targetCoordinatesMatrix, targetIds ),
                            currentRotationMatrix, currentTranslationVector );

    // Transform current source coordinates
    this->_mutableSourceCoordinatesMatrix = currentRotationMatrix * this->_mutableSourceCoordinatesMatrix +
                                            cv::repeat( currentTranslationVector, 1, this->_mutableSourceCoordinatesMatrix.cols );

    // Evaluate error
    CoordinatesMatrixType currentCoordinateDifferences = this->SampleCoordinatesMatrix( this->_mutableSourceCoordinatesMatrix, mutableSourceIds ) -
                                                         this->SampleCoordinatesMatrix( this->_targetCoordinatesMatrix, targetIds );

    double currentError = std::sqrt( cv::sum( currentCoordinateDifferences.mul( currentCoordinateDifferences ) )[0] / currentCoordinateDifferences.cols );

    // Check error tolerance criteria
    if( currentError < this->_tolerance )
    {
      break;
    }

    if( this->_verbose )
    {
      std::cout << "It: " << currentIteration << ", e: " << currentError << std::endl;
      std::cout << "Current R:" << std::endl << currentRotationMatrix << std::endl;
      std::cout << "Current t: " << std::endl << currentTranslationVector << std::endl;
      std::cout << std::endl;
    }

    currentIteration++;
  }

  // Compute final estimated transformation
  IndicesVectorType sourceIds, targetIds;
  this->ComputeSourceToTargetCorrespondences( sourceIds, targetIds );
  this->ComputeTransform( this->SampleCoordinatesMatrix( this->_sourceCoordinatesMatrix, sourceIds ),
                          this->SampleCoordinatesMatrix( this->_targetCoordinatesMatrix, targetIds ),
                          this->_rotationMatrix, this->_translationVector );

  if( this->_verbose )
  {
    std::cout << "Estimated R: " << std::endl << this->_rotationMatrix << std::endl;
    std::cout << "Estimated t: " << std::endl << this->_translationVector << std::endl;
  }
}

template< typename TCoordinate, unsigned int Dimension >
void
IterativeClosestPoint< TCoordinate, Dimension >
::ComputeSourceToTargetCorrespondences( IndicesVectorType & sourceIds, IndicesVectorType & targetIds )
{
  // Sample random points in current deformed source, avoiding repeated ids
  sourceIds.create( 1, this->_sourceCoordinateIds.size() * this->_samplingRatio );
  std::shuffle( this->_sourceCoordinateIds.begin(), this->_sourceCoordinateIds.end(), this->_randomGenerator );
  for( unsigned int qIt = 0; qIt < this->_sourceCoordinateIds.size() * this->_samplingRatio; qIt++ )
  {
    sourceIds( 0, qIt ) = this->_sourceCoordinateIds[ qIt ];
  }
  CoordinatesMatrixType sampledSourceCoordinatesQueryTransposed;
  cv::transpose( this->SampleCoordinatesMatrix( this->_mutableSourceCoordinatesMatrix, sourceIds ), sampledSourceCoordinatesQueryTransposed );

  // Retrieve closest target points
  IndicesVectorType closestTargetPointIds;
  CoordinatesMatrixType closestTargetPointDistances;
  this->_KDTree->knnSearch( sampledSourceCoordinatesQueryTransposed, closestTargetPointIds, closestTargetPointDistances, 1 );
  cv::transpose( closestTargetPointIds, targetIds );
}

template< typename TCoordinate, unsigned int Dimension >
typename IterativeClosestPoint< TCoordinate, Dimension >::CoordinatesMatrixType
IterativeClosestPoint< TCoordinate, Dimension >
::SampleCoordinatesMatrix( const CoordinatesMatrixType & coordinatesMatrix, const IndicesVectorType & indices )
{
  CoordinatesMatrixType sampledCoordinatesMatrix( Dimension, indices.cols );
  for( size_t iIt = 0; iIt < indices.cols; iIt++ )
  {
    for( unsigned int dim = 0; dim < Dimension; ++dim )
    {
      sampledCoordinatesMatrix( dim, iIt ) = coordinatesMatrix( dim, indices( 0, iIt ) );
    }
  }

  return sampledCoordinatesMatrix;
}

template< typename TCoordinate, unsigned int Dimension >
void
IterativeClosestPoint< TCoordinate, Dimension >
::ComputeTransform( const CoordinatesMatrixType & source, const CoordinatesMatrixType & target,
                    CoordinatesMatrixType & rotationMatrix, CoordinatesMatrixType & translationVector )
{
  CoordinatesMatrixType sourceCentroid, targetCentroid;
  cv::reduce( source, sourceCentroid, 1, CV_REDUCE_AVG );
  cv::reduce( target, targetCentroid, 1, CV_REDUCE_AVG );

  CoordinatesMatrixType centeredSourceCoordinatesMatrix = source - cv::repeat( sourceCentroid, 1, source.cols );
  CoordinatesMatrixType centeredTargetCoordinatesMatrix = target - cv::repeat( targetCentroid, 1, target.cols );

  CoordinatesMatrixType centeredTargetCoordinatesMatrixTransposed;
  cv::transpose( centeredTargetCoordinatesMatrix, centeredTargetCoordinatesMatrixTransposed );

  // W = ( source - source_mean ) * ( target - target_mean )T
  CoordinatesMatrixType W = centeredSourceCoordinatesMatrix * centeredTargetCoordinatesMatrixTransposed;

  // W = U * singular_values * Vt
  CoordinatesMatrixType S, U, Vt;
  cv::SVD::compute( W, S, U, Vt, cv::SVD::FULL_UV );

  // R = V * Ut
  CoordinatesMatrixType Ut, V;
  cv::transpose( U, Ut );
  cv::transpose( Vt, V );
  rotationMatrix = V * Ut;
  if( cv::determinant( rotationMatrix ) < 0 )
  {
    for( size_t r = 0; r < Dimension; r++ )
    {
      rotationMatrix( r, Dimension-1 ) = -( rotationMatrix( r, Dimension-1 ) );
    }
  }

  // t = target_mean - R * source_mean
  translationVector = targetCentroid - rotationMatrix * sourceCentroid;
}

template< typename TCoordinate, unsigned int Dimension >
void
IterativeClosestPoint< TCoordinate, Dimension >
::CheckValidDimensions( const CoordinatesMatrixType & coordinatesMatrix ) const
{
  if( ( coordinatesMatrix.rows != Dimension ) )
  {
    throw std::runtime_error( "Incorrect coordinate dimensions" );
  }
}

template< typename TCoordinate, unsigned int Dimension >
void
IterativeClosestPoint< TCoordinate, Dimension >
::CheckDataAvailable() const
{
  if( this->_sourceCoordinatesMatrix.empty() )
  {
    throw std::runtime_error( "Source coordinates matrix not set" );
  }

  if( this->_targetCoordinatesMatrix.empty() )
  {
    throw std::runtime_error( "Target coordinates matrix not set" );
  }
}
