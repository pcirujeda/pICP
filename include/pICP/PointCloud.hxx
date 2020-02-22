#pragma once

#include "pICP/PointCloud.h"

template< typename TCoordinate, unsigned int TDimension >
void
PointCloud< TCoordinate, TDimension >
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
    // Even if the read was successful, a minor error could have happened
    std::cerr << error << std::endl;
  }
  
  // Populate coordinates matrix
  this->_coordinatesMatrix.resize( TDimension, _objAttributes.vertices.size()/TDimension );
  for( size_t vIt = 0; vIt < _objAttributes.vertices.size()/TDimension; vIt++ )
  {
    for( size_t dimIt = 0; dimIt < TDimension; dimIt++ )
    {
      this->_coordinatesMatrix( dimIt, vIt ) = static_cast< TCoordinate >( _objAttributes.vertices[ TDimension*vIt + dimIt ] );
    }
  }
}

template< typename TCoordinate, unsigned int TDimension >
void
PointCloud< TCoordinate, TDimension >
::WriteOBJ( const std::string & objFilename )
{
  if( !WriteObj( objFilename.c_str(), _objAttributes, _shapes, _materials ) )
  {
    throw std::runtime_error( "Error writing OBJ file " + objFilename + ". " );
  }
}

template< typename TCoordinate, unsigned int TDimension >
typename PointCloud< TCoordinate, TDimension >::CoordinatesMatrixType
PointCloud< TCoordinate, TDimension >
::GetCoordinatesMatrix()
{
  return this->_coordinatesMatrix;
}

template< typename TCoordinate, unsigned int TDimension >
typename PointCloud< TCoordinate, TDimension >::CoordinatesMatrixType
PointCloud< TCoordinate, TDimension >
::SelectCoordinates( const PointIdentifierContainerType & coordinateIdentifiers )
{
  CoordinatesMatrixType selectedCoordinatesMatrix( TDimension, coordinateIdentifiers.size() );

  size_t scmIt = 0;
  for( const auto ciIt: coordinateIdentifiers )
  {
    selectedCoordinatesMatrix.col( scmIt++ ) = this->_coordinatesMatrix.col( ciIt );
  }
  
  return selectedCoordinatesMatrix;
}

template< typename TCoordinate, unsigned int TDimension >
void
PointCloud< TCoordinate, TDimension >
::UpdateCoordinatesMatrix( const CoordinatesMatrixType & coordinates )
{
  this->_coordinatesMatrix = coordinates;

  for( size_t vIt = 0; vIt < this->_coordinatesMatrix.cols(); vIt++ )
  {
    for( size_t dimIt = 0; dimIt < TDimension; dimIt++ )
    {
      _objAttributes.vertices[ TDimension*vIt + dimIt ] = static_cast< TCoordinate >( this->_coordinatesMatrix( dimIt, vIt ) );
    }
  }
}
