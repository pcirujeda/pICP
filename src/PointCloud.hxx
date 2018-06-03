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
  tinyobj::attrib_t objAttributes;
  std::vector< tinyobj::shape_t > shapes;
  std::vector< tinyobj::material_t > materials;
  std::string error;
  
  if( !tinyobj::LoadObj( &objAttributes, &shapes, &materials, &error, objFilename.c_str() ) || !error.empty() )
  {
    std::cerr << error << std::endl;
  }
  
  // Populate coordinates matrix
  this->_coordinatesMatrix.create( Dimension, objAttributes.vertices.size()/3 );
  for( size_t vIt = 0; vIt < objAttributes.vertices.size()/3; vIt++ )
  {
    for( size_t dimIt = 0; dimIt < Dimension; dimIt++ )
    {
      this->_coordinatesMatrix( dimIt, vIt ) = static_cast< TCoordinate >( objAttributes.vertices[ 3*vIt + dimIt ] );
    }
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

#endif