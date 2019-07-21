#pragma once

#include <cstdint>
#include <cstring>

static std::uint32_t nextPowerOfTwo(std::uint32_t v)
{
    v--;
    v |= v >> 1;
    v |= v >> 2;
    v |= v >> 4;
    v |= v >> 8;
    v |= v >> 16;
    v++;
    return v;
}

template<typename T>
static void carray_resize(const T** array, std::uint32_t* currentSize, std::uint32_t newSize)
{
    T** local = const_cast<T**>(array);

    std::uint32_t previous = nextPowerOfTwo(*currentSize);
    std::uint32_t next = nextPowerOfTwo(newSize);
    if (previous != next)
    {
        *local = reinterpret_cast<T*>(realloc(*local, next * sizeof(T)));
    }
    *currentSize = newSize;
}

template<typename T>
inline void carray_resize(T** array, std::uint32_t* currentSize, std::uint32_t newSize)
{
    carray_resize(const_cast<const T**>(array), currentSize, newSize);
}

template<typename T>
static void carray_push_back(const T** array, std::uint32_t* size, T value)
{
    T** local = const_cast<T**>(array);

    std::uint32_t previous = nextPowerOfTwo(*size);
    std::uint32_t next = nextPowerOfTwo(++(*size));
    if (previous != next)
    {
        *local = reinterpret_cast<T*>(realloc(*local, next * sizeof(T)));
    }
    (*local)[*size-1] = value;
}

template<typename T>
inline void carray_push_back(T** array, std::uint32_t* size, T value)
{
    carray_push_back(const_cast<const T**>(array), size, value);
}

template<typename T>
static void carray_copy(const T* source, const T* destination, std::uint32_t count)
{
    T* lsource = const_cast<T*>(source);
    T* ldestination = const_cast<T*>(destination);

    memcpy(ldestination, lsource, count * sizeof(T));
}

template<typename T>
inline void carray_copy(T* source, T* destination, std::uint32_t count)
{
    carray_push_back(const_cast<const T*>(source), const_cast<const T*>(destination), count);
}

template<typename T>
inline void carray_free(const T* array, std::uint32_t* size)
{
    T* local = const_cast<T*>(array);

    free(local);
    *size = 0;
}

/*
------------------------------------------------------------------------------
This software is available under 2 licenses -- choose whichever you prefer.
------------------------------------------------------------------------------
ALTERNATIVE A - MIT License
Copyright (c) 2018 Patrick Pelletier
Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
------------------------------------------------------------------------------
ALTERNATIVE B - Public Domain (www.unlicense.org)
This is free and unencumbered software released into the public domain.
Anyone is free to copy, modify, publish, use, compile, sell, or distribute this
software, either in source code form or as a compiled binary, for any purpose,
commercial or non-commercial, and by any means.
In jurisdictions that recognize copyright laws, the author or authors of this
software dedicate any and all copyright interest in the software to the public
domain. We make this dedication for the benefit of the public at large and to
the detriment of our heirs and successors. We intend this dedication to be an
overt act of relinquishment in perpetuity of all present and future rights to
this software under copyright law.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
------------------------------------------------------------------------------
*/
