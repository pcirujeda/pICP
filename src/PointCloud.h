#ifndef __PointCloud_h
#define __PointCloud_h

#define TINYOBJLOADER_IMPLEMENTATION
#include "external/tiny_obj_loader.h"
#include "external/obj_writer.h"

#include <opencv2/opencv.hpp>

#include <stdexcept>

template< typename TCoordinate, unsigned int Dimension >
class PointCloud
{
  public:
    typedef TCoordinate                CoordinateType;
    typedef cv::Mat_< CoordinateType > CoordinatesMatrixType;
    typedef std::set< unsigned int >   PointIdentifierContainerType;

    PointCloud();
    ~PointCloud();

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

#include "PointCloud.hxx"

#endif