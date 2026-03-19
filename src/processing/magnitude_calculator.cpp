#include "acceltool/processing/magnitude_calculator.h"

#include <cmath>

namespace acceltool
{
    ProcessedSample MagnitudeCalculator::process(const RawSample& raw) const
    {
        ProcessedSample out{};

        out.sampleIndex = raw.sampleIndex;
        out.nodeAddress = raw.nodeAddress;
        out.timestampSeconds = raw.timestampSeconds;

        out.x = raw.x;
        out.y = raw.y;
        out.z = raw.z;

        out.baseRssi = raw.baseRssi;
        out.nodeRssi = raw.nodeRssi;

        out.magnitudeXY = std::sqrt(raw.x * raw.x + raw.y * raw.y);
        out.magnitudeXYZ = std::sqrt(raw.x * raw.x + raw.y * raw.y + raw.z * raw.z);

        return out;
    }
}

