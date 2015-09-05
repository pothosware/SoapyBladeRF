/*
 * This file is part of the bladeRF project:
 *   http://www.github.com/nuand/bladeRF
 *
 * Copyright (C) 2015 Josh Blum
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#pragma once

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Time.hpp>
#include <libbladeRF.h>
#include <cstdio>
#include <queue>

/*!
 * Storage for rx commands and tx responses
 */
struct StreamMetadata
{
    int flags;
    long long timeNs;
    size_t numElems;
    int code;
};

/*!
 * The SoapySDR device interface for a blade RF.
 * The overloaded virtual methods calls into the blade RF C API.
 */
class bladeRF_SoapySDR : public SoapySDR::Device
{
public:

    //! initialize blade RF from device info
    bladeRF_SoapySDR(const bladerf_devinfo &devinfo);

    //! destructor shuts down and cleans up
    ~bladeRF_SoapySDR(void);

    /*******************************************************************
     * Identification API
     ******************************************************************/

    std::string getDriverKey(void) const
    {
        return "bladeRF";
    }

    std::string getHardwareKey(void) const
    {
        return "bladeRF";
    }

    SoapySDR::Kwargs getHardwareInfo(void) const;

    /*******************************************************************
     * Channels API
     ******************************************************************/

    size_t getNumChannels(const int) const
    {
        return 1;
    }

    bool getFullDuplex(const int, const size_t) const
    {
        return true;
    }

    /*******************************************************************
     * Stream API
     ******************************************************************/

    SoapySDR::Stream *setupStream(
        const int direction,
        const std::string &format,
        const std::vector<size_t> &channels = std::vector<size_t>(),
        const SoapySDR::Kwargs &args = SoapySDR::Kwargs());

    void closeStream(SoapySDR::Stream *stream);

    size_t getStreamMTU(SoapySDR::Stream *stream) const;

    int activateStream(
        SoapySDR::Stream *stream,
        const int flags = 0,
        const long long timeNs = 0,
        const size_t numElems = 0);

    int deactivateStream(
        SoapySDR::Stream *stream,
        const int flags = 0,
        const long long timeNs = 0);

    int readStream(
        SoapySDR::Stream *stream,
        void * const *buffs,
        const size_t numElems,
        int &flags,
        long long &timeNs,
        const long timeoutUs = 100000);

    int writeStream(
        SoapySDR::Stream *stream,
        const void * const *buffs,
        const size_t numElems,
        int &flags,
        const long long timeNs = 0,
        const long timeoutUs = 100000);

    int readStreamStatus(
        SoapySDR::Stream *stream,
        size_t &chanMask,
        int &flags,
        long long &timeNs,
        const long timeoutUs
    );

    /*******************************************************************
     * Antenna API
     ******************************************************************/

    std::vector<std::string> listAntennas(const int direction, const size_t channel) const;

    void setAntenna(const int direction, const size_t channel, const std::string &name);

    std::string getAntenna(const int direction, const size_t channel) const;

    /*******************************************************************
     * Gain API
     ******************************************************************/

    std::vector<std::string> listGains(const int direction, const size_t channel) const;

    void setGain(const int direction, const size_t channel, const double value);

    void setGain(const int direction, const size_t channel, const std::string &name, const double value);

    double getGain(const int direction, const size_t channel, const std::string &name) const;

    SoapySDR::Range getGainRange(const int direction, const size_t channel, const std::string &name) const;

    /*******************************************************************
     * Frequency API
     ******************************************************************/

    void setFrequency(const int direction, const size_t channel, const std::string &name, const double frequency, const SoapySDR::Kwargs &args = SoapySDR::Kwargs());

    double getFrequency(const int direction, const size_t channel, const std::string &name) const;

    std::vector<std::string> listFrequencies(const int direction, const size_t channel) const;

    SoapySDR::RangeList getFrequencyRange(const int direction, const size_t channel, const std::string &name) const;

    /*******************************************************************
     * Sample Rate API
     ******************************************************************/

    void setSampleRate(const int direction, const size_t channel, const double rate);

    double getSampleRate(const int direction, const size_t channel) const;

    std::vector<double> listSampleRates(const int direction, const size_t channel) const;

    void setBandwidth(const int direction, const size_t channel, const double bw);

    double getBandwidth(const int direction, const size_t channel) const;

    std::vector<double> listBandwidths(const int direction, const size_t channel) const;

    /*******************************************************************
     * Time API
     ******************************************************************/

    bool hasHardwareTime(const std::string &what = "") const;

    long long getHardwareTime(const std::string &what = "") const;

    void setHardwareTime(const long long timeNs, const std::string &what = "");

    /*******************************************************************
     * Register API
     ******************************************************************/

    void writeRegister(const unsigned addr, const unsigned value);

    unsigned readRegister(const unsigned addr) const;

    /*******************************************************************
     * GPIO API
     ******************************************************************/

    std::vector<std::string> listGPIOBanks(void) const;

    void writeGPIO(const std::string &bank, const unsigned value);

    unsigned readGPIO(const std::string &bank) const;

    void writeGPIODir(const std::string &bank, const unsigned dir);

    unsigned readGPIODir(const std::string &bank) const;

private:

    static bladerf_module _dir2mod(const int direction)
    {
        return (direction == SOAPY_SDR_RX)?BLADERF_MODULE_RX:BLADERF_MODULE_TX;
    }

    static std::string _err2str(const int err)
    {
        const char *msg = NULL;
        switch (err)
        {
        case BLADERF_ERR_UNEXPECTED: msg = "An unexpected failure occurred"; break;
        case BLADERF_ERR_RANGE: msg = "Provided parameter is out of range"; break;
        case BLADERF_ERR_INVAL: msg = "Invalid operation/parameter"; break;
        case BLADERF_ERR_MEM: msg = "Memory allocation error"; break;
        case BLADERF_ERR_IO: msg = "File/Device I/O error"; break;
        case BLADERF_ERR_TIMEOUT: msg = "Operation timed out"; break;
        case BLADERF_ERR_NODEV: msg = "No device(s) available"; break;
        case BLADERF_ERR_UNSUPPORTED: msg = "Operation not supported"; break;
        case BLADERF_ERR_MISALIGNED: msg = "Misaligned flash access"; break;
        case BLADERF_ERR_CHECKSUM: msg = "Invalid checksum"; break;
        case BLADERF_ERR_NO_FILE: msg = "File not found"; break;
        case BLADERF_ERR_UPDATE_FPGA: msg = "An FPGA update is required"; break;
        case BLADERF_ERR_UPDATE_FW: msg = "A firmware update is requied"; break;
        case BLADERF_ERR_TIME_PAST: msg = "Requested timestamp is in the past"; break;
        default: msg = "Unknown error code"; break;
        }
        char buff[256];
        sprintf(buff, "%d - %s", err, msg);
        return buff;
    }

    long long _rxTicksToTimeNs(const long long ticks) const
    {
        return SoapySDR::ticksToTimeNs(ticks, _rxSampRate) + _timeNsOffset;
    }

    long long _timeNsToRxTicks(const long long timeNs) const
    {
        return SoapySDR::timeNsToTicks(timeNs-_timeNsOffset, _rxSampRate);
    }

    long long _txTicksToTimeNs(const long long ticks) const
    {
        return SoapySDR::ticksToTimeNs(ticks, _txSampRate) + _timeNsOffset;
    }

    long long _timeNsToTxTicks(const long long timeNs) const
    {
        return SoapySDR::timeNsToTicks(timeNs-_timeNsOffset, _txSampRate);
    }

    void updateRxMinTimeoutMs(void)
    {
        //the 2x factor allows padding so we aren't on the fence
        _rxMinTimeoutMs = long((2*1000*_rxBuffSize)/_rxSampRate);
    }

    double _rxSampRate;
    double _txSampRate;
    bool _inTxBurst;
    bool _rxFloats;
    bool _txFloats;
    bool _rxOverflow;
    long long _rxNextTicks;
    long long _txNextTicks;
    long long _timeNsOffset;
    int16_t *_rxConvBuff;
    int16_t *_txConvBuff;
    size_t _rxBuffSize;
    size_t _txBuffSize;
    long _rxMinTimeoutMs;
    std::queue<StreamMetadata> _rxCmds;
    std::queue<StreamMetadata> _txResps;

    bladerf *_dev;
};
