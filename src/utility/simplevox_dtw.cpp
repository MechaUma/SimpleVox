/*!
 * SimpleVox
 *
 * Copyright (c) 2023 MechaUma
 *
 * This software is released under the MIT.
 * see https://opensource.org/licenses/MIT
 */

#include "simplevox_dtw.h"

#include <math.h>
#include <stdint.h>

namespace
{
    constexpr int kDistanceCoef = 1000;
}

namespace simplevox
{
namespace detail
{

    int InnerProduct(const int16_t *vec1, int n, const int16_t *vec2)
    {
        int val = 0;
        for (int i = 0; i < n; i++)
        {
            val += (int)vec1[i] * vec2[i];
        }
        return val;
    }

    /**
     * @brief 0から2の値をとるコサイン距離を求める
     *
     * @param[in]   inner12 ベクトル１と２の内積
     * @param[in]   inner1  ベクトル１の内積
     * @param[in]   inner2  ベクトル２の内積
     * @return float 2 - 0, 大きいほど類似度が低い
     */
    float CosineDistancef(int inner12, int inner1, int inner2)
    {
        return 1 - ((inner1 == 0 || inner2 == 0) ? 0 : inner12 / sqrtf((float)inner1 * inner2));
    }

    /**
     * @brief 0から2*coefの値をとるコサイン距離を求める
     * @param[in] inner12   ベクトル１と２の内積
     * @param[in] inner1    ベクトル１の内積
     * @param[in] inner2    ベクトル２の内積
     * @param[in] coef      コサイン距離の係数
     * @return 0 - 2000までの値, 大きいほど類似度が低い
     */
    uint32_t CosineDistance(int inner12, int inner1, int inner2)
    {
        return kDistanceCoef * CosineDistancef(inner12, inner1, inner2);
    }

} // namespace detail
} // namespace simplevox
