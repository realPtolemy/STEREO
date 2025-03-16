#include <cartesian3dgrid/cartesian3dgrid.h>
/*
Struct ShapeDSI is taken from https://github.com/tub-rip/dvs_mcemvs
*/
namespace EMVS
{
struct ShapeDSI
{
public:
    ShapeDSI(){}

    ShapeDSI(size_t dimX, size_t dimY, size_t dimZ, float min_depth, float max_depth, float fov)
        : dimX_(dimX)
        , dimY_(dimY)
        , dimZ_(dimZ)
        , min_depth_(min_depth)
        , max_depth_(max_depth)
        , fov_(fov) {}

    size_t dimX_;
    size_t dimY_;
    size_t dimZ_;

    float min_depth_;
    float max_depth_;

      // Field of View
    float fov_;
};
}
