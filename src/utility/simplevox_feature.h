/*!
 * SimpleVox
 *
 * Copyright (c) 2023 MechaUma
 *
 * This software is released under the MIT.
 * see https://opensource.org/licenses/MIT
 */

#ifndef SIMPLEVOX_FEATURE_H_
#define SIMPLEVOX_FEATURE_H_

#include<stdint.h>

namespace simplevox
{
    template <class T>
    class ISoundFeature
    {
    public:
        int size() const { return static_cast<const T*>(this)->size(); }
        int dimension() const { return static_cast<const T*>(this)->dimension(); }
        const int16_t *feature(int number) const { return static_cast<const T*>(this)->feature(number); }
    };
} // namespace simplevox

#endif // SIMPLEVOX_FEATURE_H_