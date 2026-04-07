#include "acceltool/utils/csv_writer.h"

#include <stdexcept>

namespace acceltool
{
    CsvWriter::~CsvWriter()
    {
        flush();
    }

    void CsvWriter::open(const std::string& path)
    {
        m_out.open(path, std::ios::out | std::ios::trunc);
        if (!m_out.is_open())
        {
            throw std::runtime_error("Failed to open CSV file: " + path);
        }
    }

    void CsvWriter::writeHeader()
    {
        m_out << "sample_index,"
             << "node_address,"
             << "host_timestamp_sec,"
             << "device_tick,"
             << "device_timestamp_sec,"
             << "device_timestamp_nanosec,"
             << "device_timestamp_unix_ns,"
             << "expected_timestamp_step_ns,"
             << "timestamp_gap_ns,"
             << "timestamp_gap_detected,"
             << "tick_gap_detected,"
             << "tick_gap_count,"
             << "x,"
             << "y,"
             << "z,"
             << "magnitude_xy,"
             << "magnitude_xyz,"
             << "applied_spec,"
             << "exceeds_spec\n";
    }


    void CsvWriter::writeRow(const ProcessedSample& sample)
    {
        m_out
        << sample.sampleIndex << ','
        << sample.nodeAddress << ','
        << sample.hostTimestampSeconds << ','
        << sample.deviceTick << ','
        << sample.deviceTimestampSec << ','
        << sample.deviceTimestampNanosec << ','
        << sample.deviceTimestampUnixNs << ','
        << sample.expectedTimestampStepNs << ','
        << sample.timestampGapNs << ','
        << (sample.timestampGapDetected ? 1 : 0) << ','
        << (sample.tickGapDetected ? 1 : 0) << ','
        << sample.tickGapCount << ','
        << sample.x << ','
        << sample.y << ','
        << sample.z << ','
        << sample.magnitudeXY << ','
        << sample.magnitudeXYZ << ','
        << sample.appliedSpec << ','
        << (sample.exceedsSpec ? 1 : 0) << '\n';
    }


    void CsvWriter::flush()
    {
        if (m_out.is_open())
        {
            m_out.flush();
        }
    }

    bool CsvWriter::isOpen() const noexcept
    {
        return m_out.is_open();
    }
}
