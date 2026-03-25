#include "acceltool/processing/magnitude_calculator.h"

#include <cmath>

namespace acceltool
{
    MagnitudeCalculator::MagnitudeCalculator(const AppConfig& config)
        : m_axisMode(config.axisMode),
          m_spec(config.spec)
    {
    }

    ProcessedSample MagnitudeCalculator::process(const RawSample& sample) const
    {
        ProcessedSample out{};

        out.sampleIndex = sample.sampleIndex;
        out.nodeAddress = sample.nodeAddress;
        out.timestampSeconds = sample.timestampSeconds;

        out.x = sample.x;
        out.y = sample.y;
        out.z = sample.z;

        out.magnitudeXY = std::sqrt(sample.x * sample.x + sample.y * sample.y);
        out.magnitudeXYZ = std::sqrt(sample.x * sample.x + sample.y * sample.y + sample.z * sample.z);

        out.appliedSpec = m_spec;

        if (m_axisMode == AxisMode::XY)
        {
            out.exceedsSpec = (out.magnitudeXY > m_spec);
        }
        else
        {
            out.exceedsSpec = (out.magnitudeXYZ > m_spec);
        }

        out.baseRssi = sample.baseRssi;
        out.nodeRssi = sample.nodeRssi;

        return out;
    }
}
