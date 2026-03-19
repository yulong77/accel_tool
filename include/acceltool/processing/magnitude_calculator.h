#pragma once

#include "acceltool/core/data_types.h"

namespace acceltool
{
    class MagnitudeCalculator
    {
    public:
        ProcessedSample process(const RawSample& raw) const;
    };
}
