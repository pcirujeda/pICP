#ifndef __PointCloud_hxx
#define __PointCloud_hxx

#include "PointCloud.h"

template< typename TCoordinate, unsigned int Dimension >
PointCloud< TCoordinate, Dimension >
::PointCloud()
{
  this->_coordinatesMatrix.create( Dimension, 0 );
}

template< typename TCoordinate, unsigned int Dimension >
PointCloud< TCoordinate, Dimension >
::~PointCloud()
{
}

template< typename TCoordinate, unsigned int Dimension >
void
PointCloud< TCoordinate, Dimension >
::LoadOBJ( const std::string & objFilename )
{
  // Parse using TinyObj
  std::string error;
  if( !tinyobj::LoadObj( &_objAttributes, &_shapes, &_materials, &error, objFilename.c_str() ) )
  {
    throw std::runtime_error( "Error reading OBJ file " + objFilename + ". " + error );
  }

  if( !error.empty() )
  {
    // Even if the read was successful, a minor error could have been happened
    std::cerr << error << std::endl;
  }
  
  // Populate coordinates matrix
  this->_coordinatesMatrix.create( Dimension, _objAttributes.vertices.size()/3 );
  for( size_t vIt = 0; vIt < _objAttributes.vertices.size()/3; vIt++ )
  {
    for( size_t dimIt = 0; dimIt < Dimension; dimIt++ )
    {
      this->_coordinatesMatrix( dimIt, vIt ) = static_cast< TCoordinate >( _objAttributes.vertices[ 3*vIt + dimIt ] );
    }
  }
}

template< typename TCoordinate, unsigned int Dimension >
void
PointCloud< TCoordinate, Dimension >
::WriteOBJ( const std::string & objFilename )
{
  if( !WriteObj( objFilename.c_str(), _objAttributes, _shapes, _materials ) )
  {
    throw std::runtime_error( "Error writing OBJ file " + objFilename + ". " );
  }
}

template< typename TCoordinate, unsigned int Dimension >
typename PointCloud< TCoordinate, Dimension >::CoordinatesMatrixType
PointCloud< TCoordinate, Dimension >
::GetCoordinatesMatrix()
{
  return this->_coordinatesMatrix;
}

template< typename TCoordinate, unsigned int Dimension >
typename PointCloud< TCoordinate, Dimension >::CoordinatesMatrixType
PointCloud< TCoordinate, Dimension >
::SelectCoordinates( const PointIdentifierContainerType & coordinateIdentifiers )
{
  CoordinatesMatrixType selectedCoordinatesMatrix( Dimension, coordinateIdentifiers.size() );

  size_t scmIt = 0;
  for( typename PointIdentifierContainerType::const_iterator ciIt = coordinateIdentifiers.begin(); ciIt != coordinateIdentifiers.end(); ciIt++ )
  {
    this->_coordinatesMatrix.col( *ciIt ).copyTo( selectedCoordinatesMatrix.col( scmIt ) );
    scmIt++;
  }
  
  return selectedCoordinatesMatrix;
}

template< typename TCoordinate, unsigned int Dimension >
void
PointCloud< TCoordinate, Dimension >
::UpdateCoordinatesMatrix( const CoordinatesMatrixType & coordinates )
{
  this->_coordinatesMatrix = coordinates.clone();

  for( size_t vIt = 0; vIt < this->_coordinatesMatrix.cols; vIt++ )
  {
    for( size_t dimIt = 0; dimIt < Dimension; dimIt++ )
    {
      _objAttributes.vertices[ 3*vIt + dimIt ] = static_cast< TCoordinate >( this->_coordinatesMatrix( dimIt, vIt ) );
    }
  }
}

#endif