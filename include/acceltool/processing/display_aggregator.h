#pragma once

#include <cstddef>
#include <optional>

#include "acceltool/core/data_types.h"

namespace acceltool
{
    class DisplayAggregator
    {
    public:
        explicit DisplayAggregator(std::size_t bucketSampleCount);

        std::optional<DisplayBucket> consume(const ProcessedSample& sample);
        std::optional<DisplayBucket> flush();

    private:
        std::size_t m_bucketSampleCount = 0;
        std::uint64_t m_bucketIndex = 0;

        bool m_hasOpenBucket = false;
        DisplayBucket m_current{};
    };
}
