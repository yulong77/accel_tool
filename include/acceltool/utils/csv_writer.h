#pragma once

#include <fstream>
#include <string>

#include "acceltool/core/data_types.h"

namespace acceltool
{
    class CsvWriter
    {
    public:
        CsvWriter() = default;
        ~CsvWriter();

        void open(const std::string& path);
        void writeHeader();
        void writeRow(const ProcessedSample& sample);
        void flush();
        bool isOpen() const noexcept;

    private:
        std::ofstream m_out;
    };
}
