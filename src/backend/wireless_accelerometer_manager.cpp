#include "acceltool/backend/wireless_accelerometer_manager.h"

#include <chrono>
#include <cctype>
#include <cmath>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <sstream>
#include <algorithm>
#include <limits>
#include <map>

#include <mscl/Communication/Devices.h>


namespace acceltool
{
    namespace
    {
        constexpr double kTimestampGapTolerancePpm = 5.0;

        bool containsIgnoreCase(const std::string& text, const std::string& token)
        {
            auto lower = [](unsigned char c) { return static_cast<char>(std::tolower(c)); };

            std::string a;
            a.reserve(text.size());
            for (char c : text)
            {
                a.push_back(lower(static_cast<unsigned char>(c)));
            }

            std::string b;
            b.reserve(token.size());
            for (char c : token)
            {
                b.push_back(lower(static_cast<unsigned char>(c)));
            }

            return a.find(b) != std::string::npos;
        }

        bool looksLikeChannel1(const std::string& name)
        {
            return containsIgnoreCase(name, "ch1") ||
                   containsIgnoreCase(name, "channel 1");
        }

        bool looksLikeChannel2(const std::string& name)
        {
            return containsIgnoreCase(name, "ch2") ||
                   containsIgnoreCase(name, "channel 2");
        }

        bool looksLikeChannel3(const std::string& name)
        {
            return containsIgnoreCase(name, "ch3") ||
                   containsIgnoreCase(name, "channel 3");
        }

        struct RfSweepFrequencyCandidate
        {
            std::uint32_t frequency = 0;
            int strongestRssi = -32768;
            mscl::NodeDiscoveries discoveries;
        };
        
        int rfSweepKeyToFrequencyChannel(std::uint32_t key)
        {
            if (key >= 11 && key <= 26)
            {
                return static_cast<int>(key);
            }
        
            if (key >= 2405 && key <= 2480 && ((key - 2405) % 5 == 0))
            {
                return 11 + static_cast<int>((key - 2405) / 5);
            }
        
            if (key >= 2405000 && key <= 2480000 && (((key / 1000) - 2405) % 5 == 0))
            {
                return 11 + static_cast<int>(((key / 1000) - 2405) / 5);
            }
        
            if (key >= 2405000000u && key <= 2480000000u && (((key / 1000000u) - 2405) % 5 == 0))
            {
                return 11 + static_cast<int>(((key / 1000000u) - 2405) / 5);
            }
        
            return 0;
        }
    }

    WirelessAccelerometerManager::WirelessAccelerometerManager() = default;

    WirelessAccelerometerManager::~WirelessAccelerometerManager() = default;

    std::uint64_t WirelessAccelerometerManager::expectedSamplePeriodNs() const
    {
        if (m_config.sampleRateHz == 0)
        {
            return 0;
        }

        return mscl::TimeSpan::NANOSECONDS_PER_SECOND / m_config.sampleRateHz;
    }

    void WirelessAccelerometerManager::populateTimingAndLossFields(
        const mscl::DataSweep& sweep,
        RawSample& sample)
    {
        const mscl::Timestamp& ts = sweep.timestamp();

        const std::uint64_t deviceUnixNs = ts.nanoseconds(mscl::Timestamp::UNIX);
        
        sample.deviceTick = sweep.tick();
        sample.deviceTimestampUnixNs = deviceUnixNs;
        sample.expectedTimestampStepNs = expectedSamplePeriodNs();

        sample.tickGapDetected = false;
        sample.tickGapCount = 0;
        sample.timestampGapDetected = false;
        sample.timestampGapNs = 0;
    
        if (m_hasPreviousSweepMeta)
        {
            const std::uint32_t expectedTick = m_previousDeviceTick + 1;
    
            if (sample.deviceTick != expectedTick)
            {
                sample.tickGapDetected = true;
    
                if (sample.deviceTick > expectedTick)
                {
                    sample.tickGapCount = sample.deviceTick - expectedTick;
                }
                else
                {
                    // Sync packet parser currently reads tick as uint16, so handle wraparound as 16-bit.
                    sample.tickGapCount =
                        static_cast<std::uint32_t>((0x10000u - expectedTick) + sample.deviceTick);
                }
            }
    
            sample.timestampGapNs = static_cast<std::int64_t>(
                sample.deviceTimestampUnixNs - m_previousDeviceTimestampUnixNs);
    
            if (m_config.sampleRateHz > 0)
            {
                const double tolerancePercent = m_config.timestampGapTolerancePercent;
                const double toleranceFraction = tolerancePercent / 100.0;
    
                const double expectedStepNs =
                    static_cast<double>(mscl::TimeSpan::NANOSECONDS_PER_SECOND) /
                    static_cast<double>(m_config.sampleRateHz);
    
                const double minAllowedGapNs =
                    expectedStepNs * (1.0 - toleranceFraction);
    
                const double maxAllowedGapNs =
                    expectedStepNs * (1.0 + toleranceFraction);
    
                const double actualGapNs =
                    static_cast<double>(sample.timestampGapNs);
    
                // Mark anomaly if the observed timestamp step falls outside
                // the theoretical step +/- configured percentage tolerance.
                if (actualGapNs < minAllowedGapNs || actualGapNs > maxAllowedGapNs)
                {
                    sample.timestampGapDetected = true;
                }
            }
        }
    
        m_previousDeviceTick = sample.deviceTick;
        m_previousDeviceTimestampUnixNs = sample.deviceTimestampUnixNs;
        m_hasPreviousSweepMeta = true;
    }




    mscl::WirelessTypes::WirelessSampleRate
    WirelessAccelerometerManager::toMsclSampleRate(std::uint32_t hz) const
    {
        switch (hz)
        {
        case 1024:
            return mscl::WirelessTypes::sampleRate_1024Hz;
        case 2048:
            return mscl::WirelessTypes::sampleRate_2048Hz;
        case 4096:
            return mscl::WirelessTypes::sampleRate_4096Hz;
        default:
            throw std::runtime_error(
                "Unsupported sampleRateHz: " + std::to_string(hz) +
                ". Allowed values are 1024, 2048, 4096.");
        }
    }

    mscl::ChannelMask WirelessAccelerometerManager::buildAxisChannelMask() const
    {
        mscl::ChannelMask mask;

        // channel index is 1-based in MSCL
        mask.enable(1, true);   // CH1
        mask.enable(2, true);   // CH2

        if (m_config.axisMode == AxisMode::XYZ)
        {
            mask.enable(3, true);   // CH3
        }

        return mask;
    }

    std::string WirelessAccelerometerManager::channelMaskToString(const mscl::ChannelMask& mask) const
    {
        std::ostringstream oss;
        bool first = true;

        for (std::uint8_t ch = 1; ch <= mscl::ChannelMask::MAX_CHANNELS; ++ch)
        {
            if (!mask.enabled(ch))
            {
                continue;
            }

            if (!first)
            {
                oss << ",";
            }

            oss << "CH" << static_cast<int>(ch);
            first = false;
        }

        if (first)
        {
            return "(none)";
        }

        return oss.str();
    }

    std::uint32_t WirelessAccelerometerManager::fromMsclSampleRate(
        mscl::WirelessTypes::WirelessSampleRate rate) const
    {
        switch (rate)
        {
        case mscl::WirelessTypes::sampleRate_1024Hz:
            return 1024;
        case mscl::WirelessTypes::sampleRate_2048Hz:
            return 2048;
        case mscl::WirelessTypes::sampleRate_4096Hz:
            return 4096;
        default:
            return 0;
        }
    }

    std::string WirelessAccelerometerManager::commProtocolToString(
        mscl::WirelessTypes::CommProtocol protocol) const
    {
        switch (protocol)
        {
        case mscl::WirelessTypes::commProtocol_lxrs:
            return "LXRS";
        case mscl::WirelessTypes::commProtocol_lxrsPlus:
            return "LXRS+";
        default:
            return "Unknown";
        }
    }

    std::string WirelessAccelerometerManager::defaultModeToString(
        mscl::WirelessTypes::DefaultMode mode) const
    {
        switch (mode)
        {
        case mscl::WirelessTypes::defaultMode_sleep:
            return "Sleep";
        case mscl::WirelessTypes::defaultMode_idle:
            return "Idle";
        case mscl::WirelessTypes::defaultMode_sync:
            return "Sampling";
        default:
            return "Unknown";
        }
    }

    std::string WirelessAccelerometerManager::samplingModeToString(
        mscl::WirelessTypes::SamplingMode mode) const
    {
        switch (mode)
        {
        case mscl::WirelessTypes::samplingMode_nonSync:
            return "NonSync";
        case mscl::WirelessTypes::samplingMode_sync:
            return "Sync";
        case mscl::WirelessTypes::samplingMode_syncBurst:
            return "Burst";
        case mscl::WirelessTypes::samplingMode_syncEvent:
            return "Event";
        case mscl::WirelessTypes::samplingMode_armedDatalog:
            return "ArmedDatalog";
        default:
            return "Unknown";
        }
    }

    void WirelessAccelerometerManager::connect(const AppConfig& config)
    {
        m_config = config;
    
        m_connected = false;
        m_samplingStarted = false;
        m_network.reset();
        m_node.reset();
        m_baseStation.reset();
        m_connection.reset();
    
        bool connected = false;
    
        if (!isAutoPort(m_config.port))
        {
            std::cout << "Trying configured base station port: "
                      << m_config.port << "...\n";
    
            connected = connectToBaseStationPort(m_config.port);
    
            if (!connected)
            {
                std::cout << "Configured base station port failed.\n";
            }
        }
    
        if (!connected && m_config.autoFindComPort)
        {
            connected = scanAndConnectBaseStation();
        }
    
        if (!connected)
        {
            throw std::runtime_error(
                "Failed to connect to a MicroStrain base station. Check COM port, USB connection, and baudrate.");
        }
    
        m_connected = true;
    }

    bool WirelessAccelerometerManager::isAutoPort(const std::string& port) const
    {
        return port.empty() || containsIgnoreCase(port, "auto");
    }
    
    bool WirelessAccelerometerManager::connectToBaseStationPort(const std::string& port)
    {
        try
        {
            auto connection = std::make_unique<mscl::Connection>(
                mscl::Connection::Serial(port, m_config.baudrate));
    
            auto baseStation = std::make_unique<mscl::BaseStation>(*connection);
            baseStation->readWriteRetries(1);
    
            if (!baseStation->ping())
            {
                return false;
            }
    
            m_connection = std::move(connection);
            m_baseStation = std::move(baseStation);
    
            std::cout << "Connected to base station on " << port << ".\n";
    
            return true;
        }
        catch (const std::exception& e)
        {
            std::cout << "  Port " << port << " failed: "
                      << e.what() << '\n';
            return false;
        }
    }
    
    bool WirelessAccelerometerManager::scanAndConnectBaseStation()
    {
        std::cout << "Scanning for MicroStrain base station COM port...\n";
    
        mscl::Devices::DeviceList candidates = mscl::Devices::listBaseStations();
    
        if (candidates.empty())
        {
            std::cout << "No base stations found by listBaseStations(); scanning all serial ports...\n";
            candidates = mscl::Devices::listPorts();
        }
    
        for (const auto& candidate : candidates)
        {
            const std::string& port = candidate.first;
    
            std::cout << "Trying candidate port " << port
                      << " (" << candidate.second.description() << ")...\n";
    
            if (connectToBaseStationPort(port))
            {
                m_config.port = port;
                return true;
            }
        }
    
        return false;
    }


    void WirelessAccelerometerManager::initialize(bool forceSetToIdleNow)
    {
        if (!m_connected || !m_baseStation)
        {
            throw std::runtime_error("WirelessAccelerometerManager not connected.");
        }
    
        ensureNodeReady();
    
        if (forceSetToIdleNow || m_config.forceSetToIdle)
        {
            setNodeToIdle();
        }
    
        pingNode();

        // Read the current config.
        m_configReport = {};
        m_configReport.configureNodeEnabled = m_config.configureNode;
        m_configReport.requestedSampleRateHz = m_config.sampleRateHz;
        m_configReport.requestedInactivityTimeoutSeconds = m_config.inactivityTimeoutSeconds;
        m_configReport.requestedUnlimitedDuration = m_config.unlimitedDuration;
        m_configReport.requestedUseLxrsPlus = m_config.useLxrsPlus;
        m_configReport.before = readCurrentNodeConfigSnapshot();

        optionallyApplyConfig();
    }

    void WirelessAccelerometerManager::setToIdle()
    {
        if (!m_connected || !m_baseStation)
        {
            throw std::runtime_error("WirelessAccelerometerManager not connected.");
        }
    
        if (!m_node)
        {
            ensureNodeReady();
        }
    
        setNodeToIdle();
        m_samplingStarted = false;
    }

    void WirelessAccelerometerManager::startSampling()
    {
        if (!m_connected || !m_baseStation || !m_node)
        {
            throw std::runtime_error("Cannot start sampling: manager not fully connected.");
        }
    
        if (!m_config.useSyncSampling)
        {
            throw std::runtime_error("This version currently requires useSyncSampling=true.");
        }
    
        m_sampleCounter = 0;
        m_dumpedChannels = false;

        m_hasPreviousSweepMeta = false;
        m_previousDeviceTick = 0;
        m_previousDeviceTimestampUnixNs = 0;
    
        waitForNodeToStabilize(10, 300);
    
        buildAndStartSyncNetwork();
    }

    void WirelessAccelerometerManager::stopSampling()
    {
        if (!m_samplingStarted)
        {
            return;
        }
    
        if (m_node)
        {
            try
            {
                setNodeToIdle();
            }
            catch (...)
            {
            }
        }
    
        m_samplingStarted = false;
    }

    std::vector<RawSample> WirelessAccelerometerManager::readAvailableSamples(std::uint32_t timeoutMs)
    {
        std::vector<RawSample> out;
    
        if (!m_baseStation)
        {
            return out;
        }
    
        const mscl::DataSweeps sweeps = m_baseStation->getData(timeoutMs);
        out.reserve(sweeps.size());
    
        for (const mscl::DataSweep& sweep : sweeps)
        {
            if (m_config.dumpSweepChannelsAtStartup && !m_dumpedChannels)
            {
                dumpSweepChannels(sweep);
                m_dumpedChannels = true;
            }
    
            RawSample sample{};
            if (tryConvertSweepToRawSample(sweep, sample))
            {
                out.push_back(sample);
            }
        }
    
        return out;
    }

    bool WirelessAccelerometerManager::isConnected() const noexcept
    {
        return m_connected;
    }

    int WirelessAccelerometerManager::nodeAddress() const noexcept
    {
        return m_config.nodeAddress;
    }

    const AppConfig& WirelessAccelerometerManager::currentConfig() const noexcept
    {
        return m_config;
    }

    const ConfigApplyReport& WirelessAccelerometerManager::configApplyReport() const noexcept
    {
        return m_configReport;
    }

    void WirelessAccelerometerManager::pingNode()
    {
        const mscl::PingResponse response = m_node->ping();

        if (!response.success())
        {
            throw std::runtime_error(
                "Failed to ping wireless node. Check power, range, RF frequency, LXRS+ mode, and node address.");
        }

        std::cout << "Successfully pinged Node " << m_node->nodeAddress() << '\n';
        std::cout << "Base Station RSSI: " << response.baseRssi() << '\n';
        std::cout << "Node RSSI: " << response.nodeRssi() << '\n';
        std::cout << "Model Number: " << m_node->model() << '\n';
        std::cout << "Serial: " << m_node->serial() << '\n';
        std::cout << "Firmware: " << m_node->firmwareVersion().str() << "\n\n";
    }

    bool WirelessAccelerometerManager::tryPingNode()
    {
        try
        {
            const mscl::PingResponse response = m_node->ping();
            return response.success();
        }
        catch (...)
        {
            return false;
        }
    }

    bool WirelessAccelerometerManager::recoverWithRfSweepPrompt()
    {
        if (!m_baseStation)
        {
            throw std::runtime_error("Base station is not connected.");
        }
    
        while (true)
        {
            std::cout << "\nRecovery options:\n"
                      << "  D = Listen for NodeDiscovery again on RF "
                      << m_config.frequency << "\n"
                      << "  R = Scan RF signal strength 11..26\n"
                      << "  N = Manually enter nodeAddress\n"
                      << "  Q = Quit\n"
                      << "\nEnter choice [D/R/N/Q]: ";
            std::cout.flush();
    
            std::string line;
            std::getline(std::cin, line);
    
            if (line.empty())
            {
                continue;
            }
    
            const char command =
                static_cast<char>(std::toupper(static_cast<unsigned char>(line[0])));
    
            if (command == 'Q')
            {
                return false;
            }
    
            if (command == 'D')
            {
                if (discoverNodeOnConfiguredFrequency())
                {
                    return true;
                }
    
                std::cout << "No nodeAddress was discovered on RF "
                          << m_config.frequency << ".\n";
                continue;
            }
    
            if (command == 'N')
            {
                std::cout << "Enter nodeAddress to try on RF "
                          << m_config.frequency << ": ";
                std::cout.flush();
    
                std::getline(std::cin, line);
    
                int address = 0;
                try
                {
                    address = std::stoi(line);
                }
                catch (...)
                {
                    std::cout << "Invalid nodeAddress.\n";
                    continue;
                }
    
                if (address <= 0)
                {
                    std::cout << "Invalid nodeAddress.\n";
                    continue;
                }
    
                m_config.nodeAddress = address;
    
                m_node = std::make_unique<mscl::WirelessNode>(
                    m_config.nodeAddress,
                    *m_baseStation);
                m_node->readWriteRetries(3);
    
                if (tryPingNode())
                {
                    std::cout << "Node " << m_config.nodeAddress
                              << " responded on RF " << m_config.frequency << ".\n";
                    return true;
                }
    
                std::cout << "Node " << m_config.nodeAddress
                          << " did not respond on RF " << m_config.frequency << ".\n";
                m_node.reset();
                m_config.nodeAddress = 0;
                continue;
            }
    
            if (command != 'R')
            {
                std::cout << "Unknown choice. Use D, R, N, or Q.\n";
                continue;
            }
    
            std::cout << "\nScanning RF signal strength...\n";
    
            std::map<std::uint32_t, RfSweepFrequencyCandidate> byFrequency;
    
            try
            {
                (void)m_baseStation->getNodeDiscoveries();
    
                m_baseStation->startRfSweepMode();
                std::this_thread::sleep_for(std::chrono::seconds(3));
    
                const mscl::DataSweeps sweeps = m_baseStation->getData(3000);
                const mscl::NodeDiscoveries discoveries =
                    m_baseStation->getNodeDiscoveries();
    
                try
                {
                    (void)m_baseStation->ping();
                }
                catch (...)
                {
                }
    
                for (const mscl::DataSweep& sweep : sweeps)
                {
                    if (sweep.samplingType() != mscl::DataSweep::samplingType_RfSweep)
                    {
                        continue;
                    }
    
                    for (const mscl::WirelessDataPoint& dp : sweep.data())
                    {
                        if (dp.channelId() != mscl::WirelessChannel::channel_rfSweep)
                        {
                            continue;
                        }
    
                        const mscl::RfSweep rfSweep = dp.as_RfSweep();
    
                        for (const auto& item : rfSweep)
                        {
                            const int frequency =
                                rfSweepKeyToFrequencyChannel(item.first);
    
                            if (frequency < 11 || frequency > 26)
                            {
                                continue;
                            }
    
                            auto& candidate =
                                byFrequency[static_cast<std::uint32_t>(frequency)];
    
                            candidate.frequency =
                                static_cast<std::uint32_t>(frequency);
    
                            candidate.strongestRssi =
                                std::max(candidate.strongestRssi,
                                         static_cast<int>(item.second));
                        }
                    }
                }
    
                for (const mscl::NodeDiscovery& discovery : discoveries)
                {
                    const std::uint32_t frequency =
                        static_cast<std::uint32_t>(discovery.frequency());
    
                    if (frequency < 11 || frequency > 26)
                    {
                        continue;
                    }
    
                    auto& candidate = byFrequency[frequency];
                    candidate.frequency = frequency;
                    candidate.strongestRssi =
                        std::max(candidate.strongestRssi,
                                 static_cast<int>(discovery.baseRssi()));
                    candidate.discoveries.push_back(discovery);
                }
            }
            catch (const std::exception& e)
            {
                try
                {
                    (void)m_baseStation->ping();
                }
                catch (...)
                {
                }
    
                std::cout << "RF sweep failed: " << e.what() << '\n';
                continue;
            }
    
            std::vector<RfSweepFrequencyCandidate> candidates;
    
            for (const auto& item : byFrequency)
            {
                candidates.push_back(item.second);
            }
    
            std::sort(
                candidates.begin(),
                candidates.end(),
                [](const RfSweepFrequencyCandidate& a,
                   const RfSweepFrequencyCandidate& b)
                {
                    return a.strongestRssi > b.strongestRssi;
                });
    
            if (candidates.empty())
            {
                std::cout << "No RF signal was detected on frequencies 11 through 26.\n";
                continue;
            }
    
            std::cout << "\nRF frequencies by strongest signal:\n";
    
            for (std::size_t i = 0; i < candidates.size(); ++i)
            {
                const auto& candidate = candidates[i];
    
                std::cout << "  " << (i + 1)
                          << ") RF " << candidate.frequency
                          << ", RSSI " << candidate.strongestRssi;
    
                if (!candidate.discoveries.empty())
                {
                    std::cout << ", discovered nodeAddress:";
                    for (const mscl::NodeDiscovery& discovery : candidate.discoveries)
                    {
                        std::cout << ' ' << discovery.nodeAddress();
                    }
                }
                else
                {
                    std::cout << ", no nodeAddress discovered";
                }
    
                std::cout << '\n';
            }
    
            std::cout << "\nChoose a frequency number to use, R to rescan, or Q to quit: ";
            std::cout.flush();
    
            std::getline(std::cin, line);
    
            if (line.empty())
            {
                continue;
            }
    
            const char nextCommand =
                static_cast<char>(std::toupper(static_cast<unsigned char>(line[0])));
    
            if (nextCommand == 'Q')
            {
                return false;
            }
    
            if (nextCommand == 'R')
            {
                continue;
            }
    
            int selectedIndex = 0;
    
            try
            {
                selectedIndex = std::stoi(line);
            }
            catch (...)
            {
                std::cout << "Invalid choice.\n";
                continue;
            }
    
            if (selectedIndex < 1 ||
                selectedIndex > static_cast<int>(candidates.size()))
            {
                std::cout << "Invalid choice.\n";
                continue;
            }
    
            const RfSweepFrequencyCandidate& selected =
                candidates[static_cast<std::size_t>(selectedIndex - 1)];
    
            m_config.frequency = selected.frequency;
    
            std::cout << "Setting base station RF frequency to "
                      << m_config.frequency << "...\n";
    
            m_baseStation->changeFrequency(toMsclFrequency(m_config.frequency));
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
    
            if (selected.discoveries.empty())
            {
                std::cout << "RF signal was detected on RF " << m_config.frequency
                          << ", but no nodeAddress was discovered.\n"
                          << "You can now choose D to listen again on this RF, "
                          << "N to manually enter nodeAddress, or Q to quit.\n";
                continue;
            }
    
            if (selected.discoveries.size() == 1)
            {
                mscl::NodeDiscoveries oneDiscovery;
                oneDiscovery.push_back(selected.discoveries.front());
    
                selectDiscoveredNode(oneDiscovery);
                return true;
            }
    
            std::cout << "\nMultiple nodeAddress values were discovered on RF "
                      << selected.frequency << ":\n";
    
            for (std::size_t i = 0; i < selected.discoveries.size(); ++i)
            {
                const mscl::NodeDiscovery& discovery = selected.discoveries[i];
    
                std::cout << "  " << (i + 1)
                          << ") Node " << discovery.nodeAddress()
                          << ", RSSI " << discovery.baseRssi()
                          << ", Protocol "
                          << commProtocolToString(discovery.communicationProtocol())
                          << ", Serial " << discovery.serialNumber()
                          << '\n';
            }
    
            std::cout << "Choose a node number to use, or Q to quit: ";
            std::cout.flush();
    
            std::getline(std::cin, line);
    
            if (line.empty() ||
                std::toupper(static_cast<unsigned char>(line[0])) == 'Q')
            {
                return false;
            }
    
            int nodeChoice = 0;
    
            try
            {
                nodeChoice = std::stoi(line);
            }
            catch (...)
            {
                std::cout << "Invalid choice.\n";
                continue;
            }
    
            if (nodeChoice < 1 ||
                nodeChoice > static_cast<int>(selected.discoveries.size()))
            {
                std::cout << "Invalid choice.\n";
                continue;
            }
    
            mscl::NodeDiscoveries oneDiscovery;
            oneDiscovery.push_back(
                selected.discoveries[static_cast<std::size_t>(nodeChoice - 1)]);
    
            if (selectDiscoveredNode(oneDiscovery))
            {
                return true;
            }
            
            continue;
        }
    }


    mscl::WirelessTypes::Frequency
    WirelessAccelerometerManager::toMsclFrequency(std::uint32_t frequency) const
    {
        if (frequency < static_cast<std::uint32_t>(mscl::WirelessTypes::freq_11) ||
            frequency > static_cast<std::uint32_t>(mscl::WirelessTypes::freq_26))
        {
            throw std::runtime_error(
                "Invalid RF frequency: " + std::to_string(frequency) +
                ". Allowed values are 11 through 26.");
        }
    
        return static_cast<mscl::WirelessTypes::Frequency>(frequency);
    }
    
    void WirelessAccelerometerManager::ensureNodeReady()
    {
        if (!m_baseStation)
        {
            throw std::runtime_error("Base station is not connected.");
        }
    
        if (m_config.frequency == 0)
        {
            throw std::runtime_error(
                "RF frequency is not configured. Set frequency=11..26 in acceltool.ini.");
        }
    
        std::cout << "Setting base station RF frequency to configured value "
                  << m_config.frequency << "...\n";
    
        m_baseStation->changeFrequency(toMsclFrequency(m_config.frequency));
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    
        if (m_config.nodeAddress > 0)
        {
            std::cout << "Using configured Node Address "
                      << m_config.nodeAddress << ".\n";
        
            m_node = std::make_unique<mscl::WirelessNode>(
                m_config.nodeAddress,
                *m_baseStation);
            m_node->readWriteRetries(3);
        
            try
            {
                ensureNodeReachable();
                return;
            }
            catch (const std::exception& e)
            {
                std::cout << "Configured nodeAddress/frequency did not respond: "
                          << e.what() << "\n";
        
                if (!m_config.autoFindNodeAddress)
                {
                    throw;
                }
        
                std::cout << "Scanning all RF frequencies 11..26 for node discovery...\n";
        
                m_node.reset();
                m_config.nodeAddress = 0;
        
                if (scanFrequenciesForNodeDiscovery())
                {
                    ensureNodeReachable();
                    return;
                }
        
                throw;
            }
        }
    
        if (m_config.autoFindNodeAddress)
        {
            if (discoverNodeOnConfiguredFrequency())
            {
                ensureNodeReachable();
                return;
            }
        
            std::cout << "No nodeAddress was discovered on configured RF "
                      << m_config.frequency
                      << ". Scanning all RF frequencies 11..26...\n";
        
            if (scanFrequenciesForNodeDiscovery())
            {
                ensureNodeReachable();
                return;
            }
        
            throw std::runtime_error(
                "Could not discover nodeAddress on the configured RF frequency or any RF frequency 11..26.");
        }

    
        throw std::runtime_error(
            "nodeAddress is 0/auto, but autoFindNodeAddress is false.");
    }

    
    void WirelessAccelerometerManager::ensureNodeReachable()
    {
        if (!m_node)
        {
            throw std::runtime_error("Wireless node has not been created.");
        }
    
        if (tryPingNode())
        {
            return;
        }
    
        std::cout << "Initial node ping failed.\n";
    
        throw std::runtime_error(
            "Failed to ping wireless node using the configured RF frequency and node address.");
    }

    
    bool WirelessAccelerometerManager::scanForNodeFrequency()
    {
        if (!m_baseStation || !m_node)
        {
            return false;
        }
    
        for (int frequency = static_cast<int>(mscl::WirelessTypes::freq_11);
             frequency <= static_cast<int>(mscl::WirelessTypes::freq_26);
             ++frequency)
        {
            const auto msclFrequency =
                static_cast<mscl::WirelessTypes::Frequency>(frequency);
    
            try
            {
                std::cout << "  Trying RF frequency " << frequency << "...\n";
    
                m_baseStation->changeFrequency(msclFrequency);
                std::this_thread::sleep_for(std::chrono::milliseconds(250));
    
                if (tryPingNode())
                {
                    std::cout << "Found Node " << m_config.nodeAddress
                              << " on RF frequency " << frequency << ".\n";
                    m_config.frequency = static_cast<std::uint32_t>(frequency);
                    return true;
                }
            }
            catch (const std::exception& e)
            {
                std::cout << "  RF frequency " << frequency
                          << " scan attempt failed: " << e.what() << '\n';
            }
        }
    
        return false;
    }
    
    bool WirelessAccelerometerManager::discoverNodeOnConfiguredFrequency()
    {
        if (m_config.frequency == 0)
        {
            return false;
        }
    
        std::cout << "Discovering node on configured RF frequency "
                  << m_config.frequency << "...\n";
    
        return collectNodeDiscoveryOnCurrentFrequency(10);
    }
    
    bool WirelessAccelerometerManager::scanFrequenciesForNodeDiscovery()
    {
        if (!m_baseStation)
        {
            return false;
        }
    
        std::cout << "Scanning RF frequencies for node discovery...\n";

        const std::uint32_t originalFrequency = m_config.frequency;

        for (int frequency = static_cast<int>(mscl::WirelessTypes::freq_11);
             frequency <= static_cast<int>(mscl::WirelessTypes::freq_26);
             ++frequency)
        {
            try
            {
                const auto msclFrequency =
                    static_cast<mscl::WirelessTypes::Frequency>(frequency);
    
                std::cout << "  Discovering on RF frequency "
                          << frequency << "...\n";
    
                m_baseStation->changeFrequency(msclFrequency);
                m_config.frequency = static_cast<std::uint32_t>(frequency);
    
                std::this_thread::sleep_for(std::chrono::milliseconds(250));
    
                if (collectNodeDiscoveryOnCurrentFrequency(10))
                {
                    return true;
                }
            }
            catch (const std::exception& e)
            {
                std::cout << "  RF frequency " << frequency
                          << " discovery attempt failed: " << e.what() << '\n';
            }
        }

        m_config.frequency = originalFrequency;
        return false;
    }

    
    bool WirelessAccelerometerManager::collectNodeDiscoveryOnCurrentFrequency(int listenSeconds)
    {
        if (!m_baseStation)
        {
            return false;
        }
    
        try
        {
            (void)m_baseStation->getNodeDiscoveries();
            (void)m_baseStation->getData(10);
    
            constexpr int kListenSeconds = 30;
    
            std::cout << "  Listening for any wireless packets on RF frequency "
                      << m_config.frequency << " for "
                      << listenSeconds << " seconds...\n";
    
            for (int elapsed = 0; elapsed < listenSeconds; ++elapsed)
            {
                const mscl::NodeDiscoveries discoveries =
                    m_baseStation->getNodeDiscoveries();
    
                for (const mscl::NodeDiscovery& discovery : discoveries)
                {
                    if (static_cast<std::uint32_t>(discovery.frequency()) == m_config.frequency)
                    {
                        std::cout << "\n  NodeDiscovery received:\n"
                                  << "    Node Address: " << discovery.nodeAddress()
                                  << ", RF: " << static_cast<int>(discovery.frequency())
                                  << ", RSSI: " << discovery.baseRssi()
                                  << ", Protocol: " << commProtocolToString(discovery.communicationProtocol())
                                  << ", Serial: " << discovery.serialNumber()
                                  << '\n';
    
                        mscl::NodeDiscoveries selected;
                        selected.push_back(discovery);
                        if (selectDiscoveredNode(selected))
                        {
                            return true;
                        }
                        return true;
                    }
                }
    
                const mscl::DataSweeps sweeps = m_baseStation->getData(1000);
    
                for (const mscl::DataSweep& sweep : sweeps)
                {
                    const int nodeAddress = sweep.nodeAddress();
    
                    if (nodeAddress <= 0)
                    {
                        continue;
                    }
    
                    std::cout << "\n  Wireless packet received:\n"
                              << "    Node Address: " << nodeAddress
                              << ", Sampling Type: " << static_cast<int>(sweep.samplingType())
                              << ", Base RSSI: " << sweep.baseRssi()
                              << ", Node RSSI: " << sweep.nodeRssi()
                              << '\n';
    
                    std::cout << "\nUse Node " << nodeAddress
                              << " on RF " << m_config.frequency
                              << "? [Y/N]: ";
                    std::cout.flush();
    
                    std::string line;
                    std::getline(std::cin, line);
    
                    if (!line.empty() &&
                        std::toupper(static_cast<unsigned char>(line[0])) == 'Y')
                    {
                        m_config.nodeAddress = nodeAddress;
    
                        m_node = std::make_unique<mscl::WirelessNode>(
                            m_config.nodeAddress,
                            *m_baseStation);
                        m_node->readWriteRetries(3);
    
                        return true;
                    }
                }
    
                std::cout << "  Waiting for wireless packets... "
                          << (elapsed + 1) << "/" << listenSeconds << "\r";
                std::cout.flush();
            }
    
            std::cout << "\n  No NodeDiscovery or wireless data packets received on RF frequency "
                      << m_config.frequency << ".\n";
            return false;
        }
        catch (const std::exception& e)
        {
            std::cout << "\n  Wireless packet listening failed: " << e.what() << '\n';
            return false;
        }
    }

    bool WirelessAccelerometerManager::confirmUseNode(
        int nodeAddress,
        std::uint32_t frequency) const
    {
        std::cout << "\nUse Node " << nodeAddress
                  << " on RF " << frequency
                  << "? [Y/N]: ";
        std::cout.flush();
    
        std::string line;
        std::getline(std::cin, line);
    
        return !line.empty() &&
               std::toupper(static_cast<unsigned char>(line[0])) == 'Y';
    }


    bool WirelessAccelerometerManager::selectDiscoveredNode(
        const mscl::NodeDiscoveries& discoveries)
    {
        if (discoveries.empty())
        {
            throw std::runtime_error("No node discoveries were provided.");
        }
    
        std::cout << "Discovered wireless nodes:\n";
    
        for (const mscl::NodeDiscovery& discovery : discoveries)
        {
            std::cout << "  Node Address: " << discovery.nodeAddress()
                      << ", RF: " << static_cast<int>(discovery.frequency())
                      << ", RSSI: " << discovery.baseRssi()
                      << ", Protocol: " << commProtocolToString(discovery.communicationProtocol())
                      << ", Serial: " << discovery.serialNumber()
                      << '\n';
        }
    
        if (discoveries.size() > 1)
        {
            throw std::runtime_error(
                "Multiple wireless nodes were discovered. Please set nodeAddress explicitly in acceltool.ini.");
        }
    
        const mscl::NodeDiscovery& selected = discoveries.front();
    
        if (selected.communicationProtocol() != mscl::WirelessTypes::commProtocol_lxrsPlus)
        {
            throw std::runtime_error(
                "Discovered node is not in LXRS+ mode. This application expects LXRS+.");
        }
    
        const int selectedNodeAddress = selected.nodeAddress();
        const std::uint32_t selectedFrequency =
            static_cast<std::uint32_t>(selected.frequency());
        
        if (!confirmUseNode(selectedNodeAddress, selectedFrequency))
        {
            return false;
        }
        
        m_config.nodeAddress = selectedNodeAddress;
        m_config.frequency = selectedFrequency;
        
        std::cout << "Selected Node " << m_config.nodeAddress
                  << " on RF frequency " << m_config.frequency << ".\n";
        
    
        m_baseStation->changeFrequency(selected.frequency());
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    
        m_node = std::make_unique<mscl::WirelessNode>(
            m_config.nodeAddress,
            *m_baseStation);
        m_node->readWriteRetries(3);
    
        m_node->updateEepromCacheFromNodeDiscovery(selected);

        return true;
    }


    void WirelessAccelerometerManager::waitForNodeToStabilize(int maxAttempts, int sleepMs)
    {
        std::cout << "Waiting for node to stabilize...\n";

        for (int i = 0; i < maxAttempts; ++i)
        {
            if (tryPingNode())
            {
                std::cout << "Node is responsive again.\n";
                return;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(sleepMs));
        }

        throw std::runtime_error(
            "Node did not become responsive after configuration change.");
    }

    void WirelessAccelerometerManager::applyConfigWithRetry(
        const mscl::WirelessNodeConfig& config,
        int maxAttempts)
    {
        std::exception_ptr lastException;

        for (int attempt = 1; attempt <= maxAttempts; ++attempt)
        {
            try
            {
                std::cout << "applyConfig attempt " << attempt
                          << " / " << maxAttempts << "...\n";

                m_node->applyConfig(config);

                std::cout << "applyConfig succeeded.\n";
                return;
            }
            catch (const mscl::Error_NodeCommunication& e)
            {
                lastException = std::current_exception();

                std::cout << "applyConfig attempt " << attempt
                          << " failed with node communication error:\n"
                          << "  " << e.what() << '\n';

                if (attempt < maxAttempts)
                {
                    std::cout << "Waiting before retry...\n";
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

                    try
                    {
                        waitForNodeToStabilize(10, 300);
                    }
                    catch (...)
                    {
                    }
                }
            }
            catch (...)
            {
                throw;
            }
        }

        if (lastException)
        {
            std::rethrow_exception(lastException);
        }

        throw std::runtime_error("applyConfig failed after retries.");
    }

    void WirelessAccelerometerManager::setNodeToIdle()
    {
        mscl::SetToIdleStatus status = m_node->setToIdle();

        std::cout << "Setting Node to Idle";

        while (!status.complete(300))
        {
            std::cout << '.';
        }

        std::cout << '\n';

        switch (status.result())
        {
        case mscl::SetToIdleStatus::setToIdleResult_success:
            std::cout << "Successfully set Node to idle.\n";
            break;
        case mscl::SetToIdleStatus::setToIdleResult_canceled:
            throw std::runtime_error("Set to Idle was canceled.");
        case mscl::SetToIdleStatus::setToIdleResult_failed:
        default:
            throw std::runtime_error("Set to Idle failed.");
        }
    }

    NodeConfigSnapshot WirelessAccelerometerManager::readCurrentNodeConfigSnapshot() const
    {
        NodeConfigSnapshot snapshot{};

        snapshot.nodeAddress = m_node->nodeAddress();
        snapshot.modelNumber = std::to_string(static_cast<int>(m_node->model()));
        snapshot.serial = m_node->serial();
        snapshot.firmware = m_node->firmwareVersion().str();

        snapshot.communicationProtocol = commProtocolToString(m_node->communicationProtocol());
        snapshot.defaultMode = defaultModeToString(m_node->getDefaultMode());
        snapshot.samplingMode = samplingModeToString(m_node->getSamplingMode());

        snapshot.sampleRateHz = fromMsclSampleRate(m_node->getSampleRate());
        snapshot.inactivityTimeoutSeconds = m_node->getInactivityTimeout();
        snapshot.unlimitedDuration = m_node->getUnlimitedDuration();

        snapshot.numSweeps = m_node->getNumSweeps();

        const mscl::ChannelMask activeChannels = m_node->getActiveChannels();
        snapshot.activeChannelCount = activeChannels.count();
        snapshot.activeChannelMask = activeChannels.toMask();
        snapshot.activeChannelSummary = channelMaskToString(activeChannels);

        return snapshot;
    }

    void WirelessAccelerometerManager::printNodeConfigSnapshot(
        const std::string& title,
        const NodeConfigSnapshot& snapshot) const
    {
        std::cout << "\n========== " << title << " ==========\n";
        std::cout << "Node Address           : " << snapshot.nodeAddress << '\n';
        std::cout << "Model Number           : " << snapshot.modelNumber << '\n';
        std::cout << "Serial                 : " << snapshot.serial << '\n';
        std::cout << "Firmware               : " << snapshot.firmware << '\n';
        std::cout << "Communication Protocol : " << snapshot.communicationProtocol << '\n';
        std::cout << "Default Mode           : " << snapshot.defaultMode << '\n';
        std::cout << "Sampling Mode          : " << snapshot.samplingMode << '\n';
        std::cout << "Sample Rate            : " << snapshot.sampleRateHz << " Hz\n";
        std::cout << "Inactivity Timeout     : " << snapshot.inactivityTimeoutSeconds << " sec\n";
        std::cout << "Unlimited Duration     : " << (snapshot.unlimitedDuration ? "true" : "false") << '\n';
        std::cout << "Num Sweeps             : " << snapshot.numSweeps << '\n';
        std::cout << "Active Channel Count   : " << snapshot.activeChannelCount << '\n';
        std::cout << "Active Channel Mask    : 0x" << std::hex
                  << snapshot.activeChannelMask << std::dec << '\n';
        std::cout << "Active Channel Summary : " << snapshot.activeChannelSummary << '\n';
        std::cout << "=========================================\n\n";
    }

    void WirelessAccelerometerManager::printConfigApplyReport(const ConfigApplyReport& report) const
    {
        std::cout << "\n========== CONFIG APPLY REPORT ==========\n";
        std::cout << "configureNode                  : "
                  << (report.configureNodeEnabled ? "true" : "false") << '\n';
        std::cout << "verifyConfig passed            : "
                  << (report.verifyPassed ? "true" : "false") << '\n';
        std::cout << "applyConfig completed          : "
                  << (report.applyCompletedWithoutException ? "true" : "false") << '\n';

        std::cout << "\nRequested / Applied / Reported\n";
        std::cout << "Requested sample rate          : "
                  << report.requestedSampleRateHz << " Hz\n";
        std::cout << "Applied sample rate target     : "
                  << report.requestedSampleRateHz << " Hz\n";
        std::cout << "Device reported sample rate    : "
                  << report.after.sampleRateHz << " Hz\n";

        std::cout << "\nRequested inactivity timeout   : "
                  << report.requestedInactivityTimeoutSeconds << " sec\n";
        std::cout << "Device reported timeout        : "
                  << report.after.inactivityTimeoutSeconds << " sec\n";

        std::cout << "\nRequested unlimited duration   : "
                  << (report.requestedUnlimitedDuration ? "true" : "false") << '\n';
        std::cout << "Device reported unlimited dur. : "
                  << (report.after.unlimitedDuration ? "true" : "false") << '\n';

        std::cout << "\nRequested protocol             : "
                  << (report.requestedUseLxrsPlus ? "LXRS+" : "(no protocol change requested)") << '\n';
        std::cout << "Device reported protocol       : "
                  << report.after.communicationProtocol << '\n';

        std::cout << "\nBefore -> After summary\n";
        std::cout << "Sample Rate                    : "
                  << report.before.sampleRateHz << " Hz -> "
                  << report.after.sampleRateHz << " Hz\n";
        std::cout << "Sampling Mode                  : "
                  << report.before.samplingMode << " -> "
                  << report.after.samplingMode << '\n';
        std::cout << "Inactivity Timeout             : "
                  << report.before.inactivityTimeoutSeconds << " sec -> "
                  << report.after.inactivityTimeoutSeconds << " sec\n";
        std::cout << "Unlimited Duration             : "
                  << (report.before.unlimitedDuration ? "true" : "false") << " -> "
                  << (report.after.unlimitedDuration ? "true" : "false") << '\n';
        std::cout << "Communication Protocol         : "
                  << report.before.communicationProtocol << " -> "
                  << report.after.communicationProtocol << '\n';
        std::cout << "=========================================\n\n";
    }

    void WirelessAccelerometerManager::printCurrentConfig()
    {
        const NodeConfigSnapshot snapshot = readCurrentNodeConfigSnapshot();
        printNodeConfigSnapshot("Node Configuration", snapshot);
    }

    void WirelessAccelerometerManager::optionallyApplyConfig()
    {
        if (!m_config.configureNode)
        {
            std::cout << "Skipping node configuration changes (configureNode = false).\n";
            m_configReport.after = m_configReport.before;
            printConfigApplyReport(m_configReport);
            return;
        }

        std::cout << "Applying node configuration...\n";

        mscl::WirelessNodeConfig config;

        if (m_config.useLxrsPlus)
        {
            std::cout << "Setting communication protocol to LXRS+...\n";
            config.communicationProtocol(mscl::WirelessTypes::commProtocol_lxrsPlus);
        }

        std::cout << "Requested sample rate: " << m_config.sampleRateHz << " Hz\n";
        std::cout << "Requested inactivity timeout: "
                  << m_config.inactivityTimeoutSeconds << " seconds\n";
        std::cout << "Requested unlimited duration: "
                  << (m_config.unlimitedDuration ? "true" : "false") << "\n";


        const mscl::ChannelMask requestedChannels = buildAxisChannelMask();

        std::cout << "Requested active channels: "
                  << channelMaskToString(requestedChannels) << '\n';

        config.defaultMode(mscl::WirelessTypes::defaultMode_idle);
        config.inactivityTimeout(static_cast<mscl::uint16>(m_config.inactivityTimeoutSeconds));
        config.samplingMode(mscl::WirelessTypes::samplingMode_sync);
        config.sampleRate(toMsclSampleRate(m_config.sampleRateHz));
        config.activeChannels(requestedChannels);
        config.unlimitedDuration(m_config.unlimitedDuration);

        mscl::ConfigIssues issues;
        if (!m_node->verifyConfig(config, issues))
        {
            m_configReport.verifyPassed = false;

            std::string msg = "Failed to verify node configuration:\n";
            for (const mscl::ConfigIssue& issue : issues)
            {
                msg += "  - " + issue.description() + "\n";
            }
            throw std::runtime_error(msg);
        }

        m_configReport.verifyPassed = true;

        applyConfigWithRetry(config, 3);
        waitForNodeToStabilize(20, 500);

        m_configReport.applyCompletedWithoutException = true;

        std::cout << "Node configuration applied and node is stable.\n";

        // Read back the config to report what the device is actually set to.
        m_configReport.after = readCurrentNodeConfigSnapshot();

        if (m_config.printCurrentNodeConfig)
        {
            printNodeConfigSnapshot("Node Configuration", m_configReport.after);
        }

        printConfigApplyReport(m_configReport);
    }

    void WirelessAccelerometerManager::buildAndStartSyncNetwork()
    {
        m_network = std::make_unique<mscl::SyncSamplingNetwork>(*m_baseStation);

        std::cout << "Adding Node " << m_node->nodeAddress() << " to sync network...\n";
        m_network->addNode(*m_node);

        std::cout << "Network info:\n";
        std::cout << "  Network OK: " << (m_network->ok() ? "TRUE" : "FALSE") << '\n';
        std::cout << "  Percent Bandwidth: " << m_network->percentBandwidth() << "%\n";
        std::cout << "  Lossless Enabled: " << (m_network->lossless() ? "TRUE" : "FALSE") << '\n';

        std::cout << "Applying network configuration...\n";
        m_network->applyConfiguration();

        std::cout << "Starting sync sampling network...\n";
        m_network->startSampling();

        m_samplingStarted = true;
    }

    bool WirelessAccelerometerManager::tryConvertSweepToRawSample(const mscl::DataSweep& sweep, RawSample& out)
    {
        if (sweep.nodeAddress() != m_config.nodeAddress)
        {
            return false;
        }

        RawSample sample{};
        sample.sampleIndex = m_sampleCounter;
        sample.nodeAddress = sweep.nodeAddress();
        sample.baseRssi = sweep.baseRssi();
        sample.nodeRssi = sweep.nodeRssi();


        for (const mscl::WirelessDataPoint& dp : sweep.data())
        {
            const std::string name = dp.channelName();

            if (looksLikeChannel1(name))
            {
                sample.x = dp.as_float();
                sample.hasX = true;
            }
            else if (looksLikeChannel2(name))
            {
                sample.y = dp.as_float();
                sample.hasY = true;
            }
            else if (looksLikeChannel3(name))
            {
                sample.z = dp.as_float();
                sample.hasZ = true;
            }
        }

        const bool valid =
            (m_config.axisMode == AxisMode::XY)
                ? (sample.hasX && sample.hasY)
                : (sample.hasX && sample.hasY && sample.hasZ);

        if (!valid)
        {
            return false;
        }

        populateTimingAndLossFields(sweep, sample);

        out = sample;
        ++m_sampleCounter;
        return true;
    }

    void WirelessAccelerometerManager::dumpSweepChannels(const mscl::DataSweep& sweep) const
    {
        std::cout << "\n========== FIRST SWEEP DUMP ==========\n";
        std::cout << "Node " << sweep.nodeAddress()
                  << " Timestamp: " << sweep.timestamp().str()
                  << " Tick: " << sweep.tick()
                  << " Sample Rate: " << sweep.sampleRate().prettyStr()
                  << " Base RSSI: " << sweep.baseRssi()
                  << " Node RSSI: " << sweep.nodeRssi()
                  << '\n';

        for (const mscl::WirelessDataPoint& dp : sweep.data())
        {
            std::cout << "  " << dp.channelName() << " = " << dp.as_string() << '\n';
        }

        std::cout << "======================================\n\n";
    }
}

