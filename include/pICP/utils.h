#pragma once

namespace pICP
{

template <typename CoordinatesMatrixType, typename RotationMatrixType, typename TranslationVectorType>
CoordinatesMatrixType
transformCoordinatesMatrix( const CoordinatesMatrixType& inputCoordinates,    
                            const RotationMatrixType& rotation,
                            const TranslationVectorType& translation )
{
    return ( rotation * inputCoordinates ).colwise() + translation;
}

template <typename CoordinatesMatrixType >
typename CoordinatesMatrixType::Scalar
coordinatesDifferenceNorm( const CoordinatesMatrixType& coordinatesSource, const CoordinatesMatrixType& coordinatesTarget )
{
    return ( coordinatesSource - coordinatesTarget ).squaredNorm();
}

}  // namespace pICP
