#pragma once

#include <cstdint>

namespace acceltool
{
    struct RawSample
    {
        std::uint64_t sampleIndex = 0;
        int nodeAddress = 0;

        double timestampSeconds = 0.0;

        double x = 0.0;
        double y = 0.0;
        double z = 0.0;

        bool hasX = false;
        bool hasY = false;
        bool hasZ = false;

        int baseRssi = 0;
        int nodeRssi = 0;
    };

    struct ProcessedSample
    {
        std::uint64_t sampleIndex = 0;
        int nodeAddress = 0;

        double timestampSeconds = 0.0;

        double x = 0.0;
        double y = 0.0;
        double z = 0.0;

        double magnitudeXY = 0.0;
        double magnitudeXYZ = 0.0;

        int baseRssi = 0;
        int nodeRssi = 0;
    };
}

