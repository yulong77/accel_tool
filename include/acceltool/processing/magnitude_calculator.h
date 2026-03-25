#pragma once

#include "acceltool/core/app_config.h"
#include "acceltool/core/data_types.h"

namespace acceltool
{
    class MagnitudeCalculator
    {
    public:
        explicit MagnitudeCalculator(const AppConfig& config);

        ProcessedSample process(const RawSample& sample) const;

    private:
        AxisMode m_axisMode = AxisMode::XYZ;
        double m_spec = 0.0;
    };
}
