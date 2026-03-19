#pragma once

#include <memory>
#include <optional>
#include <string>

#include <mscl/stdafx.h>
#include <mscl/MicroStrain/Wireless/BaseStation.h>
#include <mscl/MicroStrain/Wireless/Configuration/WirelessNodeConfig.h>
#include <mscl/MicroStrain/Wireless/Features/ChannelGroup.h>
#include <mscl/MicroStrain/Wireless/Features/NodeFeatures.h>
#include <mscl/MicroStrain/Wireless/SyncSamplingNetwork.h>
#include <mscl/MicroStrain/Wireless/WirelessNode.h>

#include "acceltool/core/app_config.h"
#include "acceltool/core/data_types.h"

namespace acceltool
{
    class WirelessAccelerometerManager
    {
    public:
        WirelessAccelerometerManager();
        ~WirelessAccelerometerManager();

        void connect(const AppConfig& config);
        void initialize();
        void startSampling();
        void stopSampling();

        std::optional<RawSample> readNextSample(std::uint32_t timeoutMs = 10);

        bool isConnected() const noexcept;
        int nodeAddress() const noexcept;

    private:
        void pingNode();
        void setNodeToIdle();
        void printCurrentConfig();
        void optionallyApplyConfig();
        void buildAndStartSyncNetwork();

        bool tryConvertSweepToRawSample(const mscl::DataSweep& sweep, RawSample& out);
        void dumpSweepChannels(const mscl::DataSweep& sweep) const;

        mscl::WirelessTypes::WirelessSampleRate toMsclSampleRate(std::uint32_t hz) const;

        static double nowSeconds();

    private:
        AppConfig m_config{};

        bool m_connected = false;
        bool m_samplingStarted = false;
        bool m_dumpedChannels = false;

        std::uint64_t m_sampleCounter = 0;

        std::unique_ptr<mscl::Connection> m_connection;
        std::unique_ptr<mscl::BaseStation> m_baseStation;
        std::unique_ptr<mscl::WirelessNode> m_node;
        std::unique_ptr<mscl::SyncSamplingNetwork> m_network;
    };
}
