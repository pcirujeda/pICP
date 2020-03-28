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

}  // namespace pICP
