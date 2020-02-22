#pragma once

#define TINYOBJLOADER_IMPLEMENTATION
#include "external/tiny_obj_loader.h"
#include "external/obj_writer.h"

#include <Eigen/Core>

#include <iostream>
#include <stdexcept>

template< typename TCoordinate, unsigned int TDimension = 3 >
class PointCloud
{
  public:
    typedef Eigen::Matrix< TCoordinate, TDimension, Eigen::Dynamic > CoordinatesMatrixType;
    typedef std::set< unsigned int >                                 PointIdentifierContainerType;

    PointCloud() = default;
    ~PointCloud() = default;

    void LoadOBJ( const std::string & objFilename );
    void WriteOBJ( const std::string & objFilename );
    
    CoordinatesMatrixType GetCoordinatesMatrix();
    CoordinatesMatrixType SelectCoordinates( const PointIdentifierContainerType & coordinateIdentifiers );

    void UpdateCoordinatesMatrix( const CoordinatesMatrixType & coordinates );

  private:
    // Plain OBJ coordinates
    CoordinatesMatrixType _coordinatesMatrix;

    // OBJ file attributes
    tinyobj::attrib_t                  _objAttributes;
    std::vector< tinyobj::shape_t >    _shapes;
    std::vector< tinyobj::material_t > _materials;
};

#include "pICP/PointCloud.hxx"
