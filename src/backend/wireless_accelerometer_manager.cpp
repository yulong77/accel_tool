#include "acceltool/backend/wireless_accelerometer_manager.h"

#include <chrono>
#include <cctype>
#include <iostream>
#include <stdexcept>
#include <string>

namespace acceltool
{
    namespace
    {
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

        bool looksLikeAccelChannel(const std::string& name)
        {
            return containsIgnoreCase(name, "accel") ||
                   containsIgnoreCase(name, "acceleration");
        }

        bool looksLikeAxisX(const std::string& name)
        {
            return containsIgnoreCase(name, "x") ||
                   containsIgnoreCase(name, "axis 1") ||
                   containsIgnoreCase(name, "ch1");
        }

        bool looksLikeAxisY(const std::string& name)
        {
            return containsIgnoreCase(name, "y") ||
                   containsIgnoreCase(name, "axis 2") ||
                   containsIgnoreCase(name, "ch2");
        }

        bool looksLikeAxisZ(const std::string& name)
        {
            return containsIgnoreCase(name, "z") ||
                   containsIgnoreCase(name, "axis 3") ||
                   containsIgnoreCase(name, "ch3");
        }
    }

    WirelessAccelerometerManager::WirelessAccelerometerManager() = default;

    WirelessAccelerometerManager::~WirelessAccelerometerManager()
    {
        try
        {
            stopSampling();
        }
        catch (...)
        {
        }
    }

    double WirelessAccelerometerManager::nowSeconds()
    {
        using clock = std::chrono::steady_clock;
        static const auto t0 = clock::now();
        const auto dt = clock::now() - t0;
        return std::chrono::duration<double>(dt).count();
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

    void WirelessAccelerometerManager::connect(const AppConfig& config)
    {
        m_config = config;

        m_connection = std::make_unique<mscl::Connection>(
            mscl::Connection::Serial(m_config.port, m_config.baudrate));

        m_baseStation = std::make_unique<mscl::BaseStation>(*m_connection);
        m_baseStation->readWriteRetries(3);

        m_node = std::make_unique<mscl::WirelessNode>(m_config.nodeAddress, *m_baseStation);
        m_node->readWriteRetries(3);

        m_connected = true;
    }

    void WirelessAccelerometerManager::initialize()
    {
        if (!m_connected || !m_node)
        {
            throw std::runtime_error("WirelessAccelerometerManager not connected.");
        }

        pingNode();

        if (m_config.forceSetToIdle)
        {
            setNodeToIdle();
        }

        if (m_config.printCurrentNodeConfig)
        {
            printCurrentConfig();
        }

        optionallyApplyConfig();
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

        buildAndStartSyncNetwork();
    }

    void WirelessAccelerometerManager::stopSampling()
    {
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

    std::optional<RawSample> WirelessAccelerometerManager::readNextSample(std::uint32_t timeoutMs)
    {
        if (!m_baseStation)
        {
            return std::nullopt;
        }

        for (const mscl::DataSweep& sweep : m_baseStation->getData(timeoutMs))
        {
            if (m_config.dumpSweepChannelsAtStartup && !m_dumpedChannels)
            {
                dumpSweepChannels(sweep);
                m_dumpedChannels = true;
            }

            RawSample sample{};
            if (tryConvertSweepToRawSample(sweep, sample))
            {
                return sample;
            }
        }

        return std::nullopt;
    }

    bool WirelessAccelerometerManager::isConnected() const noexcept
    {
        return m_connected;
    }

    int WirelessAccelerometerManager::nodeAddress() const noexcept
    {
        return m_config.nodeAddress;
    }

    void WirelessAccelerometerManager::pingNode()
    {
        const mscl::PingResponse response = m_node->ping();

        if (!response.success())
        {
            throw std::runtime_error(
                "Failed to ping wireless node. Check power, range, frequency, LXRS/LXRS+, and node address.");
        }

        std::cout << "Successfully pinged Node " << m_node->nodeAddress() << '\n';
        std::cout << "Base Station RSSI: " << response.baseRssi() << '\n';
        std::cout << "Node RSSI: " << response.nodeRssi() << '\n';
        std::cout << "Model Number: " << m_node->model() << '\n';
        std::cout << "Serial: " << m_node->serial() << '\n';
        std::cout << "Firmware: " << m_node->firmwareVersion().str() << "\n\n";
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

    void WirelessAccelerometerManager::printCurrentConfig()
    {
        std::cout << "Current Configuration Settings\n";
        std::cout << "# of Datalog Sessions: " << m_node->getNumDatalogSessions() << '\n';
        std::cout << "User Inactivity Timeout: " << m_node->getInactivityTimeout() << " seconds\n";
        std::cout << "Total active channels: " << m_node->getActiveChannels().count() << '\n';
        std::cout << "# of sweeps: " << m_node->getNumSweeps() << '\n';

        const mscl::ChannelGroups chGroups = m_node->features().channelGroups();
        for (const mscl::ChannelGroup& group : chGroups)
        {
            std::cout << "Channel Group: " << group.name() << '\n';
        }

        std::cout << '\n';
    }

    void WirelessAccelerometerManager::optionallyApplyConfig()
    {
        if (!m_config.configureNode)
        {
            std::cout << "Skipping node configuration changes (configureNode = false).\n";
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
        std::cout << "Requested inactivity timeout: " << m_config.inactivityTimeoutSeconds << " seconds\n";
        std::cout << "Requested unlimited duration: " << (m_config.unlimitedDuration ? "true" : "false") << "\n";

        config.defaultMode(mscl::WirelessTypes::defaultMode_idle);
        config.inactivityTimeout(m_config.inactivityTimeoutSeconds);
        config.samplingMode(mscl::WirelessTypes::samplingMode_sync);
        config.sampleRate(toMsclSampleRate(m_config.sampleRateHz));
        config.unlimitedDuration(m_config.unlimitedDuration);

        mscl::ConfigIssues issues;
        if (!m_node->verifyConfig(config, issues))
        {
            std::string msg = "Failed to verify node configuration:\n";
            for (const mscl::ConfigIssue& issue : issues)
            {
                msg += "  - " + issue.description() + "\n";
            }
            throw std::runtime_error(msg);
        }

        m_node->applyConfig(config);
        std::cout << "Node configuration applied.\n";
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

        out = {};
        out.sampleIndex = m_sampleCounter++;
        out.nodeAddress = sweep.nodeAddress();
        out.timestampSeconds = nowSeconds();
        out.baseRssi = sweep.baseRssi();
        out.nodeRssi = sweep.nodeRssi();

        for (const mscl::WirelessDataPoint& dp : sweep.data())
        {
            const std::string name = dp.channelName();

            if (!looksLikeAccelChannel(name))
            {
                continue;
            }

            if (looksLikeAxisX(name))
            {
                out.x = dp.as_float();
                out.hasX = true;
            }
            else if (looksLikeAxisY(name))
            {
                out.y = dp.as_float();
                out.hasY = true;
            }
            else if (looksLikeAxisZ(name))
            {
                out.z = dp.as_float();
                out.hasZ = true;
            }
        }

        if (m_config.axisMode == AxisMode::XY)
        {
            return out.hasX && out.hasY;
        }

        return out.hasX && out.hasY && out.hasZ;
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