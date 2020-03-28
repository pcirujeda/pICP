#include "boost/program_options.hpp"

#include "pICP/PointCloud.h"
#include "pICP/ICP.h"
#include "pICP/utils.h"

#include <exception>
#include <iostream>

int parseInputArguments( int argc, char* argv[],
                         std::string & sourceObjFileName,
                         std::string & targetObjFileName,
                         unsigned int & iterations,
                         double & samplingRatio,
                         double & tolerance,
                         bool & verbose,
                         std::string & transformedObjFileName )
{
  int res = 0;

  boost::program_options::options_description description("Options");

  description.add_options()
    ("help", "Produce help message")
    ("source-obj-file", boost::program_options::value< std::string >( & sourceObjFileName ), "Input source file (obj)")
    ("target-obj-file", boost::program_options::value< std::string >( & targetObjFileName ), "Input target file (obj)")
    ("iterations", boost::program_options::value< unsigned int >( & iterations )->default_value( 500 ), "(Optional) Maximum number of algorithm iterations")
    ("sampling-ratio", boost::program_options::value< double >( & samplingRatio )->default_value( 0.1 ), "(Optional) Sampling ratio (1 for full sampling)")
    ("tolerance", boost::program_options::value< double >( & tolerance )->default_value( 0.3 ), "(Optional) Error tolerance (average euclidean distance between mesh points, in object units)")
    ("verbose", boost::program_options::value< bool >( & verbose )->default_value( false ), "(Optional) Produce output message at each algorithm iteration with current error and transform estimations")
    ("transformed-obj-file", boost::program_options::value< std::string >( & transformedObjFileName ), "Output transformed source mesh file (obj)")
  ;

  boost::program_options::variables_map variableMap;
  boost::program_options::store( boost::program_options::parse_command_line( argc, argv, description ), variableMap );
  boost::program_options::notify( variableMap );

  if( variableMap.count("help") )
  {
    std::cout << description << std::endl;
    return EXIT_FAILURE;
  }

  if( !variableMap.count("source-obj-file") )
  {
    std::cerr << "Input source mesh file not provided" << std::endl;
    return EXIT_FAILURE;
  }

  if( !variableMap.count("target-obj-file") )
  {
    std::cerr << "Input target mesh file not provided" << std::endl;
    return EXIT_FAILURE;
  }

  if( !variableMap.count("transformed-obj-file") )
  {
    std::cerr << "Output transformed source mesh file not provided" << std::endl;
    return EXIT_FAILURE;
  }

  return res;
}

int main( int argc, char *argv[] )
{
  typedef pICP::PointCloud< float, 3 >            PointCloudType;
  typedef PointCloudType::CoordinatesMatrixType   PointCloudCoordinatesMatrixType;

  typedef pICP::IterativeClosestPoint< float, 3 > ICPType;

  // Parse the input arguments
  std::string sourceObjFileName;
  std::string targetObjFileName;
  unsigned int iterations;
  double samplingRatio;
  double tolerance;
  bool verbose;
  std::string transformedObjFileName;
  if( parseInputArguments( argc, argv,
                           sourceObjFileName,
                           targetObjFileName,
                           iterations,
                           samplingRatio,
                           tolerance,
                           verbose,
                           transformedObjFileName ) )
  {
    return EXIT_FAILURE;
  }

  // Read source and target point clouds
  PointCloudType sourcePointCloud, targetPointCloud;
  try
  {
    sourcePointCloud.LoadOBJ( sourceObjFileName );
    targetPointCloud.LoadOBJ( targetObjFileName );
  }
  catch( std::exception & e )
  {
    std::cout << e.what() << std::endl;
    return EXIT_FAILURE;
  }   

  // Create an instance of the ICP algorithm
  ICPType icpAlgorithm;
  icpAlgorithm.SetIterations( iterations );
  icpAlgorithm.SetSamplingRatio( samplingRatio );
  icpAlgorithm.SetTolerance( tolerance );
  icpAlgorithm.SetVerbose( verbose );

  icpAlgorithm.SetSourceCoordinatesMatrix( sourcePointCloud.GetCoordinatesMatrix() );
  icpAlgorithm.SetTargetCoordinatesMatrix( targetPointCloud.GetCoordinatesMatrix() );  
  try
  {
    icpAlgorithm.Align();
  }
  catch( std::exception & e )
  {
    std::cout << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  // Transform source mesh
  PointCloudCoordinatesMatrixType transformedSourceCoordinates =
    pICP::transformCoordinatesMatrix( sourcePointCloud.GetCoordinatesMatrix(),
                                      icpAlgorithm.GetRotationMatrix(),
                                      icpAlgorithm.GetTranslationVector() );

  sourcePointCloud.UpdateCoordinatesMatrix( transformedSourceCoordinates );                                                               

  // Store transformed OBJ 
  try
  {
    sourcePointCloud.WriteOBJ( transformedObjFileName );
  }
  catch( std::exception & e )
  {
    std::cout << e.what() << std::endl;
    return EXIT_FAILURE;
  }                                                             

  return EXIT_SUCCESS;
}
