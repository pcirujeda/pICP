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

    PointCloud() {};
    ~PointCloud( void) ;

    void LoadOBJ( const std::string & objFilename );
    
    CoordinatesMatrixType GetCoordinatesMatrix();

  private:
    CoordinatesMatrixType coordinatesMatrix;
};

#include "PointCloud.hxx"

#endif