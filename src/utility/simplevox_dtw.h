/*!
 * SimpleVox
 *
 * Copyright (c) 2023 MechaUma
 *
 * This software is released under the MIT.
 * see https://opensource.org/licenses/MIT
 */

#ifndef SIMPLEVOX_DTW_H_
#define SIMPLEVOX_DTW_H_

#include <stdint.h>

#include "simplevox_feature.h"

namespace simplevox
{
    template <class T1, class T2>
    uint32_t calcDTW(const ISoundFeature<T1> &feature1, const ISoundFeature<T2> &feature2);
} // namespace simplevox

#include "detail/simplevox_dtw.h"
#endif // SIMPLEVOX_DTW_H_