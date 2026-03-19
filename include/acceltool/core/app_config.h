#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace acceltool
{
    enum class AxisMode
    {
        XY,
        XYZ
    };

    struct AppConfig
    {
        // Wireless base station
        std::string port;
        std::uint32_t baudrate = 0;

        // Wireless node
        int nodeAddress = 0;

        // Sampling / startup
        bool forceSetToIdle = false;
        bool configureNode = false;
        bool useSyncSampling = false;
        bool useLxrsPlus = false;
        std::uint32_t sampleRateHz = 0;
        std::uint32_t inactivityTimeoutSeconds = 0;
        bool unlimitedDuration = false;

        // Acquisition
        std::size_t maxSamples = 0;
        std::uint32_t readTimeoutMs = 0;
        AxisMode axisMode = AxisMode::XYZ;

        // Output
        std::string outputCsvPath;
        bool printToConsole = false;
        std::size_t printEvery = 0;

        // Debug
        bool dumpSweepChannelsAtStartup = false;
        bool printCurrentNodeConfig = false;
    };

    bool loadConfigFromFile(const std::string& path, AppConfig& config, std::string& errorMessage);
    bool validateConfig(const AppConfig& config, std::string& errorMessage);
}
