#include "acceltool/processing/display_aggregator.h"

#include <algorithm>
#include <stdexcept>

namespace acceltool
{
    DisplayAggregator::DisplayAggregator(std::size_t bucketSampleCount)
        : m_bucketSampleCount(bucketSampleCount)
    {
        if (m_bucketSampleCount == 0)
        {
            throw std::invalid_argument("bucketSampleCount must be > 0");
        }
    }

    std::optional<DisplayBucket> DisplayAggregator::consume(const ProcessedSample& sample)
    {
        if (!m_hasOpenBucket)
        {
            m_current = {};
            m_current.bucketIndex = m_bucketIndex;
            m_current.startSampleIndex = sample.sampleIndex;
            m_current.endSampleIndex = sample.sampleIndex;
            m_current.startTimestampSeconds = sample.hostTimestampSeconds;
            m_current.endTimestampSeconds = sample.hostTimestampSeconds;
            m_current.sampleCount = 1;
            m_current.maxMagnitudeXY = sample.magnitudeXY;
            m_current.maxMagnitudeXYZ = sample.magnitudeXYZ;
            m_hasOpenBucket = true;
        }
        else
        {
            m_current.endSampleIndex = sample.sampleIndex;
            m_current.endTimestampSeconds = sample.hostTimestampSeconds;
            ++m_current.sampleCount;
            m_current.maxMagnitudeXY = std::max(m_current.maxMagnitudeXY, sample.magnitudeXY);
            m_current.maxMagnitudeXYZ = std::max(m_current.maxMagnitudeXYZ, sample.magnitudeXYZ);
        }

        if (m_current.sampleCount >= m_bucketSampleCount)
        {
            DisplayBucket finished = m_current;
            ++m_bucketIndex;
            m_hasOpenBucket = false;
            m_current = {};
            return finished;
        }

        return std::nullopt;
    }

    std::optional<DisplayBucket> DisplayAggregator::flush()
    {
        if (!m_hasOpenBucket)
        {
            return std::nullopt;
        }

        DisplayBucket finished = m_current;
        ++m_bucketIndex;
        m_hasOpenBucket = false;
        m_current = {};
        return finished;
    }
}
