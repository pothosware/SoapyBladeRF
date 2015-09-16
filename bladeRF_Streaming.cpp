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

#include "bladeRF_SoapySDR.hpp"
#include <SoapySDR/Logger.hpp>
#include <stdexcept>

//cross platform usleep()
#ifdef _MSC_VER
#include <windows.h>
#define usleep(t) Sleep((t)/1000)
#else
#include <unistd.h>
#endif

#define DEF_NUM_BUFFS 32
#define DEF_BUFF_LEN 4096

SoapySDR::Stream *bladeRF_SoapySDR::setupStream(
    const int direction,
    const std::string &format,
    const std::vector<size_t> &channels,
    const SoapySDR::Kwargs &args)
{
    //check the channel configuration
    if (channels.size() > 1 or (channels.size() > 0 and channels.at(0) != 0))
    {
        throw std::runtime_error("setupStream invalid channel selection");
    }

    //check the format
    if (format == "CF32") {}
    else if (format == "CS16") {}
    else throw std::runtime_error("setupStream invalid format " + format);

    //determine the number of buffers to allocate
    int numBuffs = (args.count("buffers") == 0)? 0 : atoi(args.at("buffers").c_str());
    if (numBuffs == 0) numBuffs = DEF_NUM_BUFFS;
    if (numBuffs == 1) numBuffs++;

    //determine the size of each buffer in samples
    int bufSize = (args.count("buflen") == 0)? 0 : atoi(args.at("buflen").c_str());
    if (bufSize == 0) bufSize = DEF_BUFF_LEN;
    if ((bufSize % 1024) != 0) bufSize = ((bufSize/1024) + 1) * 1024;

    //determine the number of active transfers
    int numXfers = (args.count("transfers") == 0)? 0 : atoi(args.at("transfers").c_str());
    if (numXfers == 0) numXfers = numBuffs/2;
    if (numXfers > numBuffs) numXfers = numBuffs; //cant have more than available buffers
    if (numXfers > 32) numXfers = 32; //libusb limit

    //setup the stream for sync tx/rx calls
    int ret = bladerf_sync_config(
        _dev,
        _dir2mod(direction),
        BLADERF_FORMAT_SC16_Q11_META,
        numBuffs,
        bufSize,
        numXfers,
        1000); //1 second timeout
    if (ret != 0)
    {
        SoapySDR::logf(SOAPY_SDR_ERROR, "bladerf_sync_config() returned %d", ret);
        throw std::runtime_error("setupStream() " + _err2str(ret));
    }

    //activate the stream here -- only call once
    ret = bladerf_enable_module(_dev, _dir2mod(direction), true);
    if (ret != 0)
    {
        SoapySDR::logf(SOAPY_SDR_ERROR, "bladerf_enable_module(true) returned %d", ret);
        throw std::runtime_error("setupStream() " + _err2str(ret));
    }

    if (direction == SOAPY_SDR_RX)
    {
        _rxOverflow = false;
        _rxFloats = (format == "CF32");
        _rxConvBuff = new int16_t[bufSize*2];
        _rxBuffSize = bufSize;
        this->updateRxMinTimeoutMs();
    }

    if (direction == SOAPY_SDR_TX)
    {
        _txFloats = (format == "CF32");
        _txConvBuff = new int16_t[bufSize*2];
        _txBuffSize = bufSize;
    }

    return (SoapySDR::Stream *)(new int(direction));
}

void bladeRF_SoapySDR::closeStream(SoapySDR::Stream *stream)
{
    const int direction = *reinterpret_cast<int *>(stream);

    //deactivate the stream here -- only call once
    const int ret = bladerf_enable_module(_dev, _dir2mod(direction), false);
    if (ret != 0)
    {
        SoapySDR::logf(SOAPY_SDR_ERROR, "bladerf_enable_module(false) returned %s", _err2str(ret).c_str());
        throw std::runtime_error("closeStream() " + _err2str(ret));
    }

    //cleanup stream convert buffers
    if (direction == SOAPY_SDR_RX)
    {
        delete [] _rxConvBuff;
    }

    if (direction == SOAPY_SDR_TX)
    {
        delete [] _txConvBuff;
    }

    delete reinterpret_cast<int *>(stream);
}

size_t bladeRF_SoapySDR::getStreamMTU(SoapySDR::Stream *stream) const
{
    const int direction = *reinterpret_cast<int *>(stream);
    return (direction == SOAPY_SDR_RX)?_rxBuffSize:_txBuffSize;
}

int bladeRF_SoapySDR::activateStream(
    SoapySDR::Stream *stream,
    const int flags,
    const long long timeNs,
    const size_t numElems)
{
    const int direction = *reinterpret_cast<int *>(stream);

    if (direction == SOAPY_SDR_RX)
    {
        StreamMetadata cmd;
        cmd.flags = flags;
        cmd.timeNs = timeNs;
        cmd.numElems = numElems;
        _rxCmds.push(cmd);
    }

    if (direction == SOAPY_SDR_TX)
    {
        if (flags != 0) return SOAPY_SDR_NOT_SUPPORTED;
    }

    return 0;
}

int bladeRF_SoapySDR::deactivateStream(
    SoapySDR::Stream *stream,
    const int flags,
    const long long)
{
    const int direction = *reinterpret_cast<int *>(stream);
    if (flags != 0) return SOAPY_SDR_NOT_SUPPORTED;

    if (direction == SOAPY_SDR_RX)
    {
        //clear all commands when deactivating
        while (not _rxCmds.empty()) _rxCmds.pop();
    }

    if (direction == SOAPY_SDR_TX)
    {
        //in a burst -> end it
        //if (_inTxBurst) this->sendTxEndBurst();
        _inTxBurst = false;
    }

    return 0;
}

int bladeRF_SoapySDR::readStream(
    SoapySDR::Stream *,
    void * const *buffs,
    size_t numElems,
    int &flags,
    long long &timeNs,
    const long timeoutUs)
{
    //clip to the available conversion buffer size
    numElems = std::min(numElems, _rxBuffSize);

    //extract the front-most command
    //no command, this is a timeout...
    if (_rxCmds.empty()) return SOAPY_SDR_TIMEOUT;
    StreamMetadata &cmd = _rxCmds.front();

    //clear output metadata
    flags = 0;
    timeNs = 0;

    //return overflow status indicator
    if (_rxOverflow)
    {
        _rxOverflow = false;
        flags |= SOAPY_SDR_HAS_TIME;
        timeNs = _rxTicksToTimeNs(_rxNextTicks);
        return SOAPY_SDR_OVERFLOW;
    }

    //initialize metadata
    bladerf_metadata md;
    md.timestamp = 0;
    md.flags = 0;
    md.status = 0;

    //without a soapy sdr time flag, set the blade rf now flag
    if ((cmd.flags & SOAPY_SDR_HAS_TIME) == 0) md.flags |= BLADERF_META_FLAG_RX_NOW;
    md.timestamp = _timeNsToRxTicks(cmd.timeNs);
    if (cmd.numElems > 0) numElems = std::min(cmd.numElems, numElems);
    cmd.flags = 0; //clear flags for subsequent calls

    //prepare buffers
    void *samples = (void *)buffs[0];
    if (_rxFloats) samples = _rxConvBuff;

    //recv the rx samples
    const long timeoutMs = std::max(_rxMinTimeoutMs, timeoutUs/1000);
    int ret = bladerf_sync_rx(_dev, samples, numElems, &md, timeoutMs);
    if (ret == BLADERF_ERR_TIMEOUT) return SOAPY_SDR_TIMEOUT;
    if (ret == BLADERF_ERR_TIME_PAST) return SOAPY_SDR_TIME_ERROR;
    if (ret != 0)
    {
        //any error when this is a finite burst causes the command to be removed
        if (cmd.numElems > 0) _rxCmds.pop();
        SoapySDR::logf(SOAPY_SDR_ERROR, "bladerf_sync_rx() returned %s", _err2str(ret).c_str());
        return SOAPY_SDR_STREAM_ERROR;
    }

    //perform the int16 to float conversion
    if (_rxFloats)
    {
        float *output = (float *)buffs[0];
        for (size_t i = 0; i < 2 * md.actual_count; i++)
        {
            output[i] = float(_rxConvBuff[i])/2048;
        }
    }

    //unpack the metadata
    flags |= SOAPY_SDR_HAS_TIME;
    timeNs = _rxTicksToTimeNs(md.timestamp);

    //parse the status
    if ((md.status & BLADERF_META_STATUS_OVERRUN) != 0)
    {
        SoapySDR::log(SOAPY_SDR_SSI, "0");
        _rxOverflow = true;
    }

    //consume from the command if this is a finite burst
    if (cmd.numElems > 0)
    {
        cmd.numElems -= md.actual_count;
        if (cmd.numElems == 0) _rxCmds.pop();
    }

    _rxNextTicks = md.timestamp + md.actual_count;
    return md.actual_count;
}

int bladeRF_SoapySDR::writeStream(
    SoapySDR::Stream *,
    const void * const *buffs,
    size_t numElems,
    int &flags,
    const long long timeNs,
    const long timeoutUs)
{
    //clip to the available conversion buffer size
    numElems = std::min(numElems, _txBuffSize);

    //initialize metadata
    bladerf_metadata md;
    md.timestamp = 0;
    md.flags = 0;
    md.status = 0;

    //time and burst start
    if (_inTxBurst)
    {
        if ((flags & SOAPY_SDR_HAS_TIME) != 0)
        {
            md.timestamp = _timeNsToTxTicks(timeNs);
            md.flags |= BLADERF_META_FLAG_TX_UPDATE_TIMESTAMP;
            _txNextTicks = md.timestamp;
        }
    }
    else
    {
        md.flags |= BLADERF_META_FLAG_TX_BURST_START;
        if ((flags & SOAPY_SDR_HAS_TIME) != 0)
        {
            md.timestamp = _timeNsToTxTicks(timeNs);
        }
        else
        {
            md.flags |= BLADERF_META_FLAG_TX_NOW;
            bladerf_get_timestamp(_dev, BLADERF_MODULE_TX, &md.timestamp);
        }
        _txNextTicks = md.timestamp;
    }

    //end of burst
    if ((flags & SOAPY_SDR_END_BURST) != 0)
    {
        md.flags |= BLADERF_META_FLAG_TX_BURST_END;
    }

    //prepare buffers
    void *samples = (void *)buffs[0];
    if (_txFloats) samples = _txConvBuff;

    //perform the float to int16 conversion
    if (_txFloats)
    {
        float *input = (float *)buffs[0];
        for (size_t i = 0; i < 2 * numElems; i++)
        {
            _txConvBuff[i] = int16_t(input[i]*2048);
        }
    }

    //send the tx samples
    int ret = bladerf_sync_tx(_dev, samples, numElems, &md, timeoutUs/1000);
    if (ret == BLADERF_ERR_TIMEOUT) return SOAPY_SDR_TIMEOUT;
    if (ret == BLADERF_ERR_TIME_PAST) return SOAPY_SDR_TIME_ERROR;
    if (ret != 0)
    {
        SoapySDR::logf(SOAPY_SDR_ERROR, "bladerf_sync_tx() returned %s", _err2str(ret).c_str());
        return SOAPY_SDR_STREAM_ERROR;
    }
    _txNextTicks += numElems;

    //always in a burst after successful tx
    _inTxBurst = true;

    //parse the status
    if ((md.status & BLADERF_META_STATUS_UNDERRUN) != 0)
    {
        SoapySDR::log(SOAPY_SDR_SSI, "U");
        StreamMetadata resp;
        resp.flags = 0;
        resp.code = SOAPY_SDR_UNDERFLOW;
        _txResps.push(resp);
    }

    //end burst status message
    if ((flags & SOAPY_SDR_END_BURST) != 0)
    {
        StreamMetadata resp;
        resp.flags = SOAPY_SDR_END_BURST | SOAPY_SDR_HAS_TIME;
        resp.timeNs = this->_txTicksToTimeNs(_txNextTicks);
        resp.code = 0;
        _txResps.push(resp);
        _inTxBurst = false;
    }

    return numElems;
}

int bladeRF_SoapySDR::readStreamStatus(
    SoapySDR::Stream *stream,
    size_t &,
    int &flags,
    long long &timeNs,
    const long timeoutUs
)
{
    const int direction = *reinterpret_cast<int *>(stream);
    if (direction == SOAPY_SDR_RX) return SOAPY_SDR_NOT_SUPPORTED;

    //wait for an event to be ready considering the timeout and time
    //this is an emulation by polling and waiting on the hardware time
    long long timeNowNs = this->getHardwareTime();
    const long long exitTimeNs = timeNowNs + (timeoutUs*1000);
    while (true)
    {
        //no status to report, sleep for a bit
        if (_txResps.empty()) goto pollSleep;

        //no time on the current status, done waiting...
        if ((_txResps.front().flags & SOAPY_SDR_HAS_TIME) == 0) break;

        //current status time expired, done waiting...
        if (_txResps.front().timeNs < timeNowNs) break;

        //sleep a bit, never more than time remaining
        pollSleep:
        usleep(std::min<long>(1000, (exitTimeNs-timeNowNs)/1000));

        //check for timeout expired
        timeNowNs = this->getHardwareTime();
        if (exitTimeNs < timeNowNs) return SOAPY_SDR_TIMEOUT;
    }

    //extract the most recent status event
    if (_txResps.empty()) return SOAPY_SDR_TIMEOUT;
    StreamMetadata resp = _txResps.front();
    _txResps.pop();

    //load the output from the response
    flags = resp.flags;
    timeNs = resp.timeNs;
    return resp.code;
}
