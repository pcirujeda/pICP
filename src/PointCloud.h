#ifndef __PointCloud_h
#define __PointCloud_h

#define TINYOBJLOADER_IMPLEMENTATION
#include "external/tiny_obj_loader.h"

#include <opencv2/opencv.hpp>

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
    
    CoordinatesMatrixType GetCoordinatesMatrix();
    CoordinatesMatrixType SelectCoordinates( const PointIdentifierContainerType & coordinateIdentifiers );

  private:
    CoordinatesMatrixType coordinatesMatrix;
};

#include "PointCloud.hxx"

#endif