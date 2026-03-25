#include <atomic>
#include <exception>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "acceltool/backend/wireless_accelerometer_manager.h"
#include "acceltool/core/app_config.h"
#include "acceltool/core/data_types.h"
#include "acceltool/processing/display_aggregator.h"
#include "acceltool/processing/magnitude_calculator.h"
#include "acceltool/utils/blocking_queue.h"
#include "acceltool/utils/csv_writer.h"
#include "acceltool/utils/display_csv_writer.h"
#include "acceltool/utils/logger.h"

namespace acceltool
{
    struct WritePayload
    {
        std::vector<ProcessedSample> processedSamples;
        std::vector<DisplayBucket> displayBuckets;
    };

    struct RuntimeStats
    {
        std::atomic<std::size_t> samplesReceivedFromDevice{0};   // Number of samples returned by manager.readAvailableSamples() and successfully converted
        std::atomic<std::size_t> samplesPushedToRawQueue{0};     // Number of samples successfully pushed into rawQueue
        std::atomic<std::size_t> samplesPoppedFromRawQueue{0};   // Number of samples popped from rawQueue by the processing thread
        
        std::atomic<std::size_t> samplesProcessed{0};            // Number of samples processed by calculator.process()
        std::atomic<std::size_t> samplesPushedToWriteQueue{0};   // Number of processed samples successfully pushed into writeQueue
        std::atomic<std::size_t> samplesPoppedFromWriteQueue{0}; // Number of processed samples popped from writeQueue by the writer thread
        std::atomic<std::size_t> samplesWrittenToCsv{0};         // Number of samples actually written to the main CSV file
        
        std::atomic<std::size_t> displayBucketsProduced{0};      // Number of display buckets produced by the aggregator
        std::atomic<std::size_t> displayBucketsWritten{0};       // Number of display buckets actually written to the display CSV file
        
        std::atomic<std::size_t> deviceReadCalls{0};             // Number of readAvailableSamples() calls
        std::atomic<std::size_t> emptyReadCount{0};              // Number of times readAvailableSamples() returned an empty batch

        std::atomic<bool> acquisitionThreadStarted{false};
        std::atomic<bool> acquisitionThreadFinished{false};

        std::atomic<bool> processingThreadStarted{false};
        std::atomic<bool> processingThreadFinished{false};

        std::atomic<bool> writerThreadStarted{false};
        std::atomic<bool> writerThreadFinished{false};
    };

    struct WorkerErrorState
    {
        mutable std::mutex mutex;
        std::exception_ptr exception;
        std::string source;

        void captureIfEmpty(const std::string& who, std::exception_ptr ex)
        {
            std::lock_guard<std::mutex> lock(mutex);
            if (!exception)
            {
                exception = ex;
                source = who;
            }
        }

        bool hasError() const
        {
            std::lock_guard<std::mutex> lock(mutex);
            return static_cast<bool>(exception);
        }

        std::exception_ptr getException() const
        {
            std::lock_guard<std::mutex> lock(mutex);
            return exception;
        }

        std::string getSource() const
        {
            std::lock_guard<std::mutex> lock(mutex);
            return source;
        }
    };

    void requestStop(
        std::atomic<bool>& stopRequested,
        BlockingQueue<std::vector<RawSample>>& rawQueue,
        BlockingQueue<WritePayload>& writeQueue)
    {
        stopRequested = true;
        rawQueue.close();
        writeQueue.close();
    }

    void logFinalStats(
        const RuntimeStats& stats,
        const BlockingQueue<std::vector<RawSample>>& rawQueue,
        const BlockingQueue<WritePayload>& writeQueue)
    {
        logInfo("========== FINAL STATS ==========");
        logInfo("deviceReadCalls            = " + std::to_string(stats.deviceReadCalls.load()));
        logInfo("emptyReadCount             = " + std::to_string(stats.emptyReadCount.load()));

        logInfo("samplesReceivedFromDevice  = " + std::to_string(stats.samplesReceivedFromDevice.load()));
        logInfo("samplesPushedToRawQueue    = " + std::to_string(stats.samplesPushedToRawQueue.load()));
        logInfo("samplesPoppedFromRawQueue  = " + std::to_string(stats.samplesPoppedFromRawQueue.load()));

        logInfo("samplesProcessed           = " + std::to_string(stats.samplesProcessed.load()));
        logInfo("samplesPushedToWriteQueue  = " + std::to_string(stats.samplesPushedToWriteQueue.load()));
        logInfo("samplesPoppedFromWriteQueue= " + std::to_string(stats.samplesPoppedFromWriteQueue.load()));
        logInfo("samplesWrittenToCsv        = " + std::to_string(stats.samplesWrittenToCsv.load()));

        logInfo("displayBucketsProduced     = " + std::to_string(stats.displayBucketsProduced.load()));
        logInfo("displayBucketsWritten      = " + std::to_string(stats.displayBucketsWritten.load()));

        logInfo("rawQueueCurrentBatches     = " + std::to_string(rawQueue.size()));
        logInfo("rawQueuePeakBatches        = " + std::to_string(rawQueue.peakSize()));
        logInfo("writeQueueCurrentBatches   = " + std::to_string(writeQueue.size()));
        logInfo("writeQueuePeakBatches      = " + std::to_string(writeQueue.peakSize()));

        logInfo("acquisitionThreadStarted   = " + std::string(stats.acquisitionThreadStarted.load() ? "true" : "false"));
        logInfo("acquisitionThreadFinished  = " + std::string(stats.acquisitionThreadFinished.load() ? "true" : "false"));
        logInfo("processingThreadStarted    = " + std::string(stats.processingThreadStarted.load() ? "true" : "false"));
        logInfo("processingThreadFinished   = " + std::string(stats.processingThreadFinished.load() ? "true" : "false"));
        logInfo("writerThreadStarted        = " + std::string(stats.writerThreadStarted.load() ? "true" : "false"));
        logInfo("writerThreadFinished       = " + std::string(stats.writerThreadFinished.load() ? "true" : "false"));
    }

    void validateFinalStats(
        const AppConfig& config,
        const RuntimeStats& stats,
        const BlockingQueue<std::vector<RawSample>>& rawQueue,
        const BlockingQueue<WritePayload>& writeQueue)
    {
        const std::size_t pushedToRaw     = stats.samplesPushedToRawQueue.load();
        const std::size_t poppedFromRaw   = stats.samplesPoppedFromRawQueue.load();
        const std::size_t processed       = stats.samplesProcessed.load();
        const std::size_t pushedToWrite   = stats.samplesPushedToWriteQueue.load();
        const std::size_t poppedFromWrite = stats.samplesPoppedFromWriteQueue.load();
        const std::size_t written         = stats.samplesWrittenToCsv.load();

        const std::size_t bucketsProduced = stats.displayBucketsProduced.load();
        const std::size_t bucketsWritten  = stats.displayBucketsWritten.load();

        if (!stats.acquisitionThreadFinished.load())
        {
            throw std::runtime_error("Acquisition thread did not finish cleanly.");
        }

        if (!stats.processingThreadFinished.load())
        {
            throw std::runtime_error("Processing thread did not finish cleanly.");
        }

        if (!stats.writerThreadFinished.load())
        {
            throw std::runtime_error("Writer thread did not finish cleanly.");
        }

        if (pushedToRaw != config.maxSamples)
        {
            throw std::runtime_error(
                "samplesPushedToRawQueue does not match config.maxSamples. "
                "pushedToRaw=" + std::to_string(pushedToRaw) +
                ", expected=" + std::to_string(config.maxSamples));
        }

        if (poppedFromRaw != pushedToRaw)
        {
            throw std::runtime_error(
                "samplesPoppedFromRawQueue does not match samplesPushedToRawQueue. "
                "poppedFromRaw=" + std::to_string(poppedFromRaw) +
                ", pushedToRaw=" + std::to_string(pushedToRaw));
        }

        if (processed != poppedFromRaw)
        {
            throw std::runtime_error(
                "samplesProcessed does not match samplesPoppedFromRawQueue. "
                "processed=" + std::to_string(processed) +
                ", poppedFromRaw=" + std::to_string(poppedFromRaw));
        }

        if (pushedToWrite != processed)
        {
            throw std::runtime_error(
                "samplesPushedToWriteQueue does not match samplesProcessed. "
                "pushedToWrite=" + std::to_string(pushedToWrite) +
                ", processed=" + std::to_string(processed));
        }

        if (poppedFromWrite != pushedToWrite)
        {
            throw std::runtime_error(
                "samplesPoppedFromWriteQueue does not match samplesPushedToWriteQueue. "
                "poppedFromWrite=" + std::to_string(poppedFromWrite) +
                ", pushedToWrite=" + std::to_string(pushedToWrite));
        }

        if (written != poppedFromWrite)
        {
            throw std::runtime_error(
                "samplesWrittenToCsv does not match samplesPoppedFromWriteQueue. "
                "written=" + std::to_string(written) +
                ", poppedFromWrite=" + std::to_string(poppedFromWrite));
        }

        if (bucketsWritten != bucketsProduced)
        {
            throw std::runtime_error(
                "displayBucketsWritten does not match displayBucketsProduced. "
                "written=" + std::to_string(bucketsWritten) +
                ", produced=" + std::to_string(bucketsProduced));
        }

        if (rawQueue.size() != 0)
        {
            throw std::runtime_error(
                "rawQueue is not empty at shutdown. remaining batches=" +
                std::to_string(rawQueue.size()));
        }

        if (writeQueue.size() != 0)
        {
            throw std::runtime_error(
                "writeQueue is not empty at shutdown. remaining batches=" +
                std::to_string(writeQueue.size()));
        }
    }
}

int main(int argc, char* argv[])
{
    using namespace acceltool;

    try
    {
        AppConfig config;

        std::string configPath = "config/acceltool.ini";
        if (argc >= 2)
        {
            configPath = argv[1];
        }

        std::string loadError;
        if (!loadConfigFromFile(configPath, config, loadError))
        {
            throw std::runtime_error(
                "Failed to load config file.\nPath: " + configPath + "\n" + loadError);
        }

        std::string validateError;
        if (!validateConfig(config, validateError))
        {
            throw std::runtime_error("Invalid configuration.\n" + validateError);
        }

        initLogger(LogLevel::Debug, true, true, "acceltool.log");
        logInfo("AccelTool started.");
        logInfo("Loaded config file: " + configPath);
        logInfo("Using port: " + config.port);
        logInfo("Using node address: " + std::to_string(config.nodeAddress));
        logInfo("Using baudrate: " + std::to_string(config.baudrate));
        logInfo("Requested sample rate from ini: " + std::to_string(config.sampleRateHz));
        logInfo("Opening raw/result CSV file: " + config.outputCsvPath);
        logInfo("Opening display CSV file: " + config.outputDisplayCsvPath);

        WirelessAccelerometerManager manager;
        CsvWriter writer;
        DisplayCsvWriter displayWriter;

        writer.open(config.outputCsvPath);
        writer.writeHeader();

        displayWriter.open(config.outputDisplayCsvPath);
        displayWriter.writeHeader();

        manager.connect(config);
        manager.initialize();
        manager.startSampling();

        BlockingQueue<std::vector<RawSample>> rawQueue(config.rawQueueCapacityBatches);
        BlockingQueue<WritePayload> writeQueue(config.writeQueueCapacityBatches);

        RuntimeStats stats;
        WorkerErrorState workerError;
        std::atomic<bool> stopRequested{false};

        std::thread acquisitionThread([&]() {
            stats.acquisitionThreadStarted = true;

            try
            {
                while (!stopRequested.load())
                {
                    const std::size_t pushedAlready = stats.samplesPushedToRawQueue.load();
                    if (pushedAlready >= config.maxSamples)
                    {
                        break;
                    }

                    ++stats.deviceReadCalls;

                    std::vector<RawSample> batch = manager.readAvailableSamples(config.readTimeoutMs);
                    if (batch.empty())
                    {
                        ++stats.emptyReadCount;
                        continue;
                    }

                    stats.samplesReceivedFromDevice += batch.size();

                    const std::size_t remaining = config.maxSamples - pushedAlready;
                    if (batch.size() > remaining)
                    {
                        batch.resize(remaining);
                    }

                    const std::size_t batchCount = batch.size();
                    if (batchCount == 0)
                    {
                        continue;
                    }

                    if (!rawQueue.push(std::move(batch)))
                    {
                        throw std::runtime_error("Failed to push batch into rawQueue because the queue is closed.");
                    }

                    stats.samplesPushedToRawQueue += batchCount;
                }
            }
            catch (...)
            {
                workerError.captureIfEmpty("acquisitionThread", std::current_exception());
                requestStop(stopRequested, rawQueue, writeQueue);
            }

            rawQueue.close();
            stats.acquisitionThreadFinished = true;
        });

        std::thread processingThread([&]() {
            stats.processingThreadStarted = true;

            try
            {
                MagnitudeCalculator calculator(config);
                DisplayAggregator aggregator(config.displayAggregationSamples);

                std::vector<RawSample> rawBatch;
                while (rawQueue.waitPop(rawBatch))
                {
                    stats.samplesPoppedFromRawQueue += rawBatch.size();

                    WritePayload payload;
                    payload.processedSamples.reserve(rawBatch.size());

                    for (const RawSample& raw : rawBatch)
                    {
                        const ProcessedSample processed = calculator.process(raw);
                        payload.processedSamples.push_back(processed);
                        ++stats.samplesProcessed;

                        auto bucketOpt = aggregator.consume(processed);
                        if (bucketOpt.has_value())
                        {
                            payload.displayBuckets.push_back(*bucketOpt);
                            ++stats.displayBucketsProduced;
                        }
                    }

                    if (!payload.processedSamples.empty())
                    {
                        const std::size_t payloadSampleCount = payload.processedSamples.size();

                        if (!writeQueue.push(std::move(payload)))
                        {
                            throw std::runtime_error("Failed to push payload into writeQueue because the queue is closed.");
                        }

                        stats.samplesPushedToWriteQueue += payloadSampleCount;
                    }
                }

                auto finalBucket = aggregator.flush();
                if (finalBucket.has_value())
                {
                    WritePayload tailPayload;
                    tailPayload.displayBuckets.push_back(*finalBucket);
                    ++stats.displayBucketsProduced;

                    if (!writeQueue.push(std::move(tailPayload)))
                    {
                        throw std::runtime_error("Failed to push final display bucket into writeQueue because the queue is closed.");
                    }
                }
            }
            catch (...)
            {
                workerError.captureIfEmpty("processingThread", std::current_exception());
                requestStop(stopRequested, rawQueue, writeQueue);
            }

            writeQueue.close();
            stats.processingThreadFinished = true;
        });

        std::thread writerThread([&]() {
            stats.writerThreadStarted = true;

            try
            {
                std::size_t nextConsolePrintAt = config.printEvery;

                WritePayload payload;
                while (writeQueue.waitPop(payload))
                {
                    stats.samplesPoppedFromWriteQueue += payload.processedSamples.size();

                    for (const ProcessedSample& sample : payload.processedSamples)
                    {
                        writer.writeRow(sample);
                        ++stats.samplesWrittenToCsv;
                    }

                    for (const DisplayBucket& bucket : payload.displayBuckets)
                    {
                        displayWriter.writeRow(bucket);
                        ++stats.displayBucketsWritten;
                    }

                    if (config.printToConsole && !payload.processedSamples.empty())
                    {
                        while (stats.samplesWrittenToCsv.load() >= nextConsolePrintAt)
                        {
                            const ProcessedSample& s = payload.processedSamples.back();
                            logInfo(
                                "sample=" + std::to_string(s.sampleIndex) +
                                ", x=" + std::to_string(s.x) +
                                ", y=" + std::to_string(s.y) +
                                ", z=" + std::to_string(s.z) +
                                ", magXY=" + std::to_string(s.magnitudeXY) +
                                ", magXYZ=" + std::to_string(s.magnitudeXYZ) +
                                ", spec=" + std::to_string(s.appliedSpec) +
                                ", exceedsSpec=" + std::string(s.exceedsSpec ? "1" : "0"));
                            nextConsolePrintAt += config.printEvery;
                        }
                    }
                }

                writer.flush();
                displayWriter.flush();
            }
            catch (...)
            {
                workerError.captureIfEmpty("writerThread", std::current_exception());
                requestStop(stopRequested, rawQueue, writeQueue);
            }

            stats.writerThreadFinished = true;
        });

        acquisitionThread.join();
        processingThread.join();
        writerThread.join();

        try
        {
            manager.stopSampling();
        }
        catch (const std::exception& e)
        {
            logError(std::string("Failed during manager.stopSampling(): ") + e.what());
        }
        catch (const mscl::Error& e)
        {
            logError(std::string("MSCL error during manager.stopSampling(): ") + e.what());
        }

        if (workerError.hasError())
        {
            logError("Worker thread failed: " + workerError.getSource());
            std::rethrow_exception(workerError.getException());
        }

        logFinalStats(stats, rawQueue, writeQueue);
        validateFinalStats(config, stats, rawQueue, writeQueue);

        logInfo("Program finished successfully.");
        shutdownLogger();
        return 0;
    }
    catch (const mscl::Error& e)
    {
        acceltool::logError(std::string("MSCL ERROR: ") + e.what());
        acceltool::shutdownLogger();
        return 1;
    }
    catch (const std::exception& e)
    {
        acceltool::logError(std::string("STD ERROR: ") + e.what());
        acceltool::shutdownLogger();
        return 1;
    }
}
