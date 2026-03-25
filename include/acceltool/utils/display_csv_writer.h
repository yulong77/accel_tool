#pragma once

#include <fstream>
#include <string>

#include "acceltool/core/data_types.h"

namespace acceltool
{
    class DisplayCsvWriter
    {
    public:
        DisplayCsvWriter() = default;
        ~DisplayCsvWriter();

        void open(const std::string& path);
        void writeHeader();
        void writeRow(const DisplayBucket& bucket);
        void flush();
        bool isOpen() const noexcept;

    private:
        std::ofstream m_out;
    };
}
