#pragma once

#include "pICP/ICP.h"

template< typename TCoordinate, unsigned int TDimension >
pICP::IterativeClosestPoint< TCoordinate, TDimension >
::IterativeClosestPoint():
  _iterations( 300 ),
  _samplingRatio( 0.25 ),
  _tolerance( 0.3 ),
  _verbose( false )
{}

template< typename TCoordinate, unsigned int TDimension >
void
pICP::IterativeClosestPoint< TCoordinate, TDimension >
::SetIterations( const unsigned int iterations )
{
  this->_iterations = iterations;
}

template< typename TCoordinate, unsigned int TDimension >
void
pICP::IterativeClosestPoint< TCoordinate, TDimension >
::SetSamplingRatio( const double samplingRatio )
{
  this->_samplingRatio = samplingRatio;
}

template< typename TCoordinate, unsigned int TDimension >
void
pICP::IterativeClosestPoint< TCoordinate, TDimension >
::SetTolerance( const double tolerance )
{
  this->_tolerance = tolerance;
}

template< typename TCoordinate, unsigned int TDimension >
void
pICP::IterativeClosestPoint< TCoordinate, TDimension >
::SetVerbose( const bool verbose )
{
  this->_verbose = verbose;
}

template< typename TCoordinate, unsigned int TDimension >
void
pICP::IterativeClosestPoint< TCoordinate, TDimension >
::SetSourceCoordinatesMatrix( const CoordinatesMatrixType & source )
{
  this->CheckValidDimensions( source );

  this->_sourceCoordinatesMatrix = source;
}

template< typename TCoordinate, unsigned int TDimension >
void
pICP::IterativeClosestPoint< TCoordinate, TDimension >
::SetTargetCoordinatesMatrix( const CoordinatesMatrixType & target )
{
  this->CheckValidDimensions( target );

  this->_targetCoordinatesMatrix = target;
}

template< typename TCoordinate, unsigned int TDimension >
typename pICP::IterativeClosestPoint< TCoordinate, TDimension >::RotationMatrixType
pICP::IterativeClosestPoint< TCoordinate, TDimension >
::GetRotationMatrix()
{
  if( this->_rotationMatrix.size() == 0 )
  {
    throw std::runtime_error( "Transform not computed yet" );
  }

  return this->_rotationMatrix;
}

template< typename TCoordinate, unsigned int TDimension >
typename pICP::IterativeClosestPoint< TCoordinate, TDimension >::TranslationVectorType
pICP::IterativeClosestPoint< TCoordinate, TDimension >
::GetTranslationVector()
{
  if( this->_translationVector.size() == 0 )
  {
    throw std::runtime_error( "Transform not computed yet" );
  }

  return this->_translationVector;
}

template< typename TCoordinate, unsigned int TDimension >
void
pICP::IterativeClosestPoint< TCoordinate, TDimension >
::Align()
{
  this->CheckDataAvailable();

  // Initialize transform components
  this->_rotationMatrix = RotationMatrixType::Identity();
  this->_translationVector = TranslationVectorType::Zero();

  // Initialize a mutable copy of source coordinates matrix
  this->_mutableSourceCoordinatesMatrix = this->_sourceCoordinatesMatrix;

  // Create target coordinates KDTree representation
  const Eigen::Matrix< TCoordinate, Eigen::Dynamic, TDimension > kdtreeCoordinates = this->_targetCoordinatesMatrix.transpose();
  this->_KDTree.reset( new KDTreeType( TDimension, std::cref( kdtreeCoordinates )));
  this->_KDTree->index->buildIndex(); 

  // Prepare random sampling assets
  std::random_device rd;
  this->_randomGenerator = std::mt19937( rd() );
  this->_sourceCoordinateIds = IndicesVectorType( this->_sourceCoordinatesMatrix.cols() );
  std::iota( this->_sourceCoordinateIds.begin(), this->_sourceCoordinateIds.end(), 0 );

  // Iterate ICP algorithm until error convergence or maximum iterations reached
  unsigned int currentIteration = 0;
  while( currentIteration < this->_iterations )
  {
    // Compute current correspondences from source to target
    IndicesVectorType mutableSourceIds, targetIds;
    this->ComputeSourceToTargetCorrespondences( mutableSourceIds, targetIds );

    // Compute current transform between candidate correspondences
    RotationMatrixType currentRotationMatrix;
    TranslationVectorType currentTranslationVector;
    this->ComputeTransform( this->SampleCoordinatesMatrix( this->_mutableSourceCoordinatesMatrix, mutableSourceIds ),
                            this->SampleCoordinatesMatrix( this->_targetCoordinatesMatrix, targetIds ),
                            currentRotationMatrix, currentTranslationVector );

    // Transform current source coordinates
    this->_mutableSourceCoordinatesMatrix = ( currentRotationMatrix * this->_mutableSourceCoordinatesMatrix ).colwise() +
                                            currentTranslationVector;

    // Evaluate error
    CoordinatesMatrixType currentCoordinateDifferences = this->SampleCoordinatesMatrix( this->_mutableSourceCoordinatesMatrix, mutableSourceIds ) -
                                                         this->SampleCoordinatesMatrix( this->_targetCoordinatesMatrix, targetIds );

    double currentError = currentCoordinateDifferences.squaredNorm();

    // Check error tolerance criteria
    if( currentError < this->_tolerance )
    {
      break;
    }

    if( this->_verbose )
    {
      std::cout << "It: " << currentIteration << ", e: " << currentError << std::endl;
      std::cout << "Current R:" << std::endl << currentRotationMatrix << std::endl;
      std::cout << "Current t: " << std::endl << currentTranslationVector.transpose() << std::endl;
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
    std::cout << "Estimated t: " << std::endl << this->_translationVector.transpose() << std::endl;
  }
}

template< typename TCoordinate, unsigned int TDimension >
void
pICP::IterativeClosestPoint< TCoordinate, TDimension >
::ComputeSourceToTargetCorrespondences( IndicesVectorType & sourceIds, IndicesVectorType & targetIds )
{
  // Sample random points in current deformed source, avoiding repeated ids
  size_t samplingSize = this->_sourceCoordinateIds.size() * this->_samplingRatio;

  std::shuffle( this->_sourceCoordinateIds.begin(), this->_sourceCoordinateIds.end(), this->_randomGenerator );
  sourceIds = IndicesVectorType( this->_sourceCoordinateIds.begin(), this->_sourceCoordinateIds.begin() + samplingSize );

  CoordinatesMatrixType sampledSourceCoordinatesQuery = this->SampleCoordinatesMatrix( this->_mutableSourceCoordinatesMatrix, sourceIds );

  // Retrieve closest target points
  size_t knnQueryPts = 1;
  std::vector<typename KDTreeType::IndexType> closestTargetId( knnQueryPts );
  std::vector<TCoordinate > closestTargetDist2( knnQueryPts );

  targetIds.resize( samplingSize );
  for(size_t spIt = 0; spIt < sampledSourceCoordinatesQuery.cols(); spIt++)
  {
    std::vector<TCoordinate> querySourceCoord = { sampledSourceCoordinatesQuery.col(spIt)(0),
                                                  sampledSourceCoordinatesQuery.col(spIt)(1),
                                                  sampledSourceCoordinatesQuery.col(spIt)(2) };
    
    this->_KDTree->index->knnSearch( &querySourceCoord[0], knnQueryPts, &closestTargetId[0], &closestTargetDist2[0] );

    targetIds[spIt] = closestTargetId[0];
  }
}

template< typename TCoordinate, unsigned int TDimension >
typename pICP::IterativeClosestPoint< TCoordinate, TDimension >::CoordinatesMatrixType
pICP::IterativeClosestPoint< TCoordinate, TDimension >
::SampleCoordinatesMatrix( const CoordinatesMatrixType & coordinatesMatrix, const IndicesVectorType & indices )
{
  CoordinatesMatrixType sampledCoordinatesMatrix( TDimension, indices.size() );

  size_t sIt = 0;
  for( const auto iIt: indices )
  {
    sampledCoordinatesMatrix.col( sIt++ ) = coordinatesMatrix.col( iIt );
  }
  
  return sampledCoordinatesMatrix;
}

template< typename TCoordinate, unsigned int TDimension >
void
pICP::IterativeClosestPoint< TCoordinate, TDimension >
::ComputeTransform( const CoordinatesMatrixType & source, const CoordinatesMatrixType & target,
                    RotationMatrixType & rotationMatrix, TranslationVectorType & translationVector )
{
  Eigen::Matrix<TCoordinate, TDimension, 1> sourceCentroid = source.rowwise().mean();
  Eigen::Matrix<TCoordinate, TDimension, 1> targetCentroid = target.rowwise().mean();

  CoordinatesMatrixType centeredSourceCoordinates = source.colwise() - sourceCentroid;
  CoordinatesMatrixType centeredTargetCoordinates = target.colwise() - targetCentroid;

  // w = ( source - source_mean ) * ( target - target_mean )T
  auto w = centeredSourceCoordinates * centeredTargetCoordinates.transpose();

  // w = u * singular_values * vt
  Eigen::JacobiSVD< Eigen::MatrixXf > svd( w, Eigen::ComputeFullU | Eigen::ComputeFullV );

  // R = V * ut
  rotationMatrix = svd.matrixV() * svd.matrixU().transpose();
  if( rotationMatrix.determinant() < 0 )
  {
    rotationMatrix.col( TDimension-1 ) *= -1.;
  }

  // t = target_mean - R * source_mean
  translationVector = targetCentroid - rotationMatrix * sourceCentroid;
}

template< typename TCoordinate, unsigned int TDimension >
void
pICP::IterativeClosestPoint< TCoordinate, TDimension >
::CheckValidDimensions( const CoordinatesMatrixType & coordinatesMatrix ) const
{
  if( ( coordinatesMatrix.rows() != TDimension ) )
  {
    throw std::runtime_error( "Incorrect coordinate TDimensions" );
  }
}

template< typename TCoordinate, unsigned int TDimension >
void
pICP::IterativeClosestPoint< TCoordinate, TDimension >
::CheckDataAvailable() const
{
  if( this->_sourceCoordinatesMatrix.size() == 0 )
  {
    throw std::runtime_error( "Source coordinates matrix not set" );
  }

  if( this->_targetCoordinatesMatrix.size() == 0 )
  {
    throw std::runtime_error( "Target coordinates matrix not set" );
  }
}
