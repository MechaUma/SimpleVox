/*!
 * SimpleVox
 *
 * Copyright (c) 2023 MechaUma
 *
 * This software is released under the MIT.
 * see https://opensource.org/licenses/MIT
 */

#ifndef DETAIL_SIMPLEVOX_DTW_H_
#define DETAIL_SIMPLEVOX_DTW_H_

#include "../simplevox_dtw.h"

#include <math.h>
#include <memory>

namespace simplevox
{
namespace detail
{
    int InnerProduct(const int16_t* vec1, int n, const int16_t* vec2);
    inline int InnerProduct(const int16_t* vec, int n) { return InnerProduct(vec, n, vec); }
    float CosineDistancef(int inner12, int inner1, int inner2);
    uint32_t CosineDistance(int inner12, int inner1, int inner2);
}
    /**
     * @brief   ２つの特徴量の最小となるDTW距離を計算します
     * @param[in]   feature1    特徴量１
     * @param[in]   feature2    特徴量２
     * @return  平均移動距離（0~2000, 全移動距離をステップ数で割ったもの）
    */
    template <class T1, class T2>
    uint32_t calcDTW(const ISoundFeature<T1> &feature1, const ISoundFeature<T2> &feature2)
    {
        using namespace simplevox::detail;
        const int dimension = feature1.dimension();
        if (dimension != feature2.dimension())
        {
            return UINT32_MAX;
        }
        if (feature1.size() <= 0 || feature2.size() <= 0)
        {
            return UINT32_MAX;
        }
        if (feature1.size() > 3 * feature2.size() || 3 * feature1.size() < feature2.size())
        {
            return UINT32_MAX;
        }

        std::unique_ptr<int16_t[]> step_counts(new (std::nothrow) int16_t [feature2.size()]);
        std::unique_ptr<uint32_t[]> step_distances(new (std::nothrow) uint32_t[feature2.size()]);
        if (!step_counts || !step_distances)
        {
            return UINT32_MAX;
        }

        const auto inner1_0 = InnerProduct(feature1.feature(0), dimension);
        const auto inner2_0 = InnerProduct(feature2.feature(0), dimension);

        // f1[0], f2[0]
        step_distances[0] = 2 * CosineDistance(InnerProduct(feature1.feature(0), dimension, feature2.feature(0)), inner1_0, inner2_0);
        step_counts[0] = 0;

        for (int j = 1; j < feature2.size(); j++)   // f1[0], f2[j] | 1 <= j < N
        {
            const auto inner12_j = InnerProduct(feature1.feature(0), dimension, feature2.feature(j));
            const auto inner2_j = InnerProduct(feature2.feature(j), dimension);
            step_distances[j] = step_distances[j - 1] + CosineDistance(inner12_j, inner1_0, inner2_j);
            step_counts[j] = j; // = step_counts[j - 1] + 1
        }

        const int last = feature2.size() - 1;
        for (int i = 1; i < feature1.size(); i++)   // f1[i] | 1 <= i < N, f2[j] | 1 <= j < N
        {
            const auto inner1_i = InnerProduct(feature1.feature(i), dimension);
            const auto inner12_i0 = InnerProduct(feature1.feature(i), dimension, feature2.feature(0));
            uint32_t prev_step_dist = step_distances[0] + CosineDistance(inner12_i0, inner1_i, inner2_0);
            int prev_step_count = step_counts[0] + 1;
            for (int j = 1; j < feature2.size(); j++)
            {
                uint32_t step_dist;
                int step_count;
                if (step_distances[j] < prev_step_dist)
                {
                    step_dist = step_distances[j];
                    step_count = step_counts[j];
                }
                else
                {
                    step_dist = prev_step_dist;
                    step_count = prev_step_count;
                }
                if (step_distances[j - 1] < step_dist)
                {
                    step_dist = step_distances[j - 1];
                    step_count = step_counts[j - 1];
                }

                const auto inner12_ij = InnerProduct(feature1.feature(i), dimension, feature2.feature(j));
                const auto inner2_j = InnerProduct(feature2.feature(j), dimension);
                step_dist += CosineDistance(inner12_ij, inner1_i, inner2_j);
                step_count += 1;
                step_distances[j - 1] = prev_step_dist;
                step_counts[j - 1] = prev_step_count;
                prev_step_dist = step_dist;
                prev_step_count = step_count;
            }
            step_distances[last] = prev_step_dist;
            step_counts[last] = prev_step_count;
        }
        return step_distances[last] / step_counts[last];
    }
} // namespace simplevox


#endif // DETAIL_SIMPLEVOX_DTW_H_