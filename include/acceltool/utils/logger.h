#pragma once

#include <fstream>
#include <string>

namespace acceltool
{
    enum class LogLevel
    {
        Debug = 0,
        Info,
        Warn,
        Error,
        Off
    };

    void initLogger(
        LogLevel level = LogLevel::Info,
        bool enableConsole = true,
        bool enableFile = false,
        const std::string& filePath = ""
    );

    void shutdownLogger();

    void logDebug(const std::string& msg);
    void logInfo(const std::string& msg);
    void logWarn(const std::string& msg);
    void logError(const std::string& msg);
}

