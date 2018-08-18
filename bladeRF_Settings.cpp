/*
 * This file is part of the bladeRF project:
 *   http://www.github.com/nuand/bladeRF
 *
 * Copyright (C) 2015-2016 Josh Blum
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
#include <algorithm> //find
#include <stdexcept>
#include <cstdio>

/*******************************************************************
 * Device init/shutdown
 ******************************************************************/

bladeRF_SoapySDR::bladeRF_SoapySDR(const bladerf_devinfo &devinfo):
    _rxSampRate(1.0),
    _txSampRate(1.0),
    _inTxBurst(false),
    _rxFloats(false),
    _txFloats(false),
    _rxOverflow(false),
    _rxNextTicks(0),
    _txNextTicks(0),
    _timeNsOffset(0),
    _rxBuffSize(0),
    _txBuffSize(0),
    _rxMinTimeoutMs(0),
    _xb200Mode("disabled"),
    _samplingMode("internal"),
    _loopbackMode("disabled"),
    _dev(NULL)

{
    bladerf_devinfo info = devinfo;
    SoapySDR::logf(SOAPY_SDR_INFO, "bladerf_open_with_devinfo()");
    int ret = bladerf_open_with_devinfo(&_dev, &info);

    if (ret < 0)
    {
        SoapySDR::logf(SOAPY_SDR_ERROR, "bladerf_open_with_devinfo() returned %s", _err2str(ret).c_str());
        throw std::runtime_error("bladerf_open_with_devinfo() failed " + _err2str(ret));
    }

    char serialStr[BLADERF_SERIAL_LENGTH];
    ret = bladerf_get_serial(_dev, serialStr);
    if (ret == 0) SoapySDR::logf(SOAPY_SDR_INFO, "bladerf_get_serial() = %s", serialStr);

    //initialize the sample rates to something
    this->setSampleRate(SOAPY_SDR_RX, 0, 4e6);
    this->setSampleRate(SOAPY_SDR_TX, 0, 4e6);

}

bladeRF_SoapySDR::~bladeRF_SoapySDR(void)
{
    SoapySDR::logf(SOAPY_SDR_INFO, "bladerf_close()");
    if (_dev != NULL) bladerf_close(_dev);
}

/*******************************************************************
 * Identification API
 ******************************************************************/

SoapySDR::Kwargs bladeRF_SoapySDR::getHardwareInfo(void) const
{
    SoapySDR::Kwargs info;

    {
        char serialStr[BLADERF_SERIAL_LENGTH];
        int ret = bladerf_get_serial(_dev, serialStr);
        if (ret == 0) info["serial"] = serialStr;
    }

    {
        bladerf_fpga_size fpgaSize = BLADERF_FPGA_UNKNOWN;
        int ret = bladerf_get_fpga_size(_dev, &fpgaSize);
        char fpgaStr[100];
        sprintf(fpgaStr, "%u", int(fpgaSize));
        if (ret == 0) info["fpga_size"] = fpgaStr;
    }

    {
        struct bladerf_version verInfo;
        int ret = bladerf_fw_version(_dev, &verInfo);
        if (ret == 0) info["fw_version"] = verInfo.describe;
    }

    {
        struct bladerf_version verInfo;
        int ret = bladerf_fpga_version(_dev, &verInfo);
        if (ret == 0) info["fpga_version"] = verInfo.describe;
    }

    return info;
}

/*******************************************************************
 * Channels API
 ******************************************************************/

size_t bladeRF_SoapySDR::getNumChannels(const int direction) const
{
    #ifndef LIBBLADERF_V2
    return 1;
    #else
    return bladerf_get_channel_count(_dev, (direction == SOAPY_SDR_RX)?BLADERF_RX:BLADERF_TX);
    #endif
}

bool bladeRF_SoapySDR::getFullDuplex(const int, const size_t) const
{
    return true;
}

/*******************************************************************
 * Antenna API
 ******************************************************************/

std::vector<std::string> bladeRF_SoapySDR::listAntennas(const int direction, const size_t) const
{
    std::vector<std::string> options;
    if (direction == SOAPY_SDR_TX) options.push_back("TX");
    if (direction == SOAPY_SDR_RX) options.push_back("RX");
    return options;
}

void bladeRF_SoapySDR::setAntenna(const int, const size_t, const std::string &)
{
    return; //nothing to set, ignore it
}

std::string bladeRF_SoapySDR::getAntenna(const int direction, const size_t channel) const
{
    if (direction == SOAPY_SDR_TX) return "TX";
    if (direction == SOAPY_SDR_RX) return "RX";
    return SoapySDR::Device::getAntenna(direction, channel);
}

/*******************************************************************
 * Calibration API
 ******************************************************************/

bool bladeRF_SoapySDR::hasDCOffset(const int, const size_t) const
{
    return true;
}

void bladeRF_SoapySDR::setDCOffset(const int direction, const size_t channel, const std::complex<double> &offset)
{
    int ret = 0;
    int16_t i = 0;
    int16_t q = 0;

    if (offset.real() > 1.0)
        i = int16_t(1.0 * 2048);
    else
        i = int16_t(offset.real() * 2048);

    if (offset.imag() > 1.0)
        q = int16_t(1.0 * 2048);
    else
        q = int16_t(offset.imag() * 2048);

    ret = bladerf_set_correction(_dev, _toch(direction, channel), BLADERF_CORR_LMS_DCOFF_I, i);
    if (ret != 0)
    {
        SoapySDR::logf(SOAPY_SDR_ERROR, "bladerf_set_correction(%f) returned %s", i, _err2str(ret).c_str());
        throw std::runtime_error("setDCOffset() " + _err2str(ret));
    }

    ret = bladerf_set_correction(_dev, _toch(direction, channel), BLADERF_CORR_LMS_DCOFF_Q, q);
    if (ret != 0)
    {
        SoapySDR::logf(SOAPY_SDR_ERROR, "bladerf_set_correction(%f) returned %s", q, _err2str(ret).c_str());
        throw std::runtime_error("setDCOffset() " + _err2str(ret));
    }
}

std::complex<double> bladeRF_SoapySDR::getDCOffset(const int direction, const size_t channel) const
{
    int ret = 0;
    int16_t i = 0;
    int16_t q = 0;

    ret = bladerf_get_correction(_dev, _toch(direction, channel), BLADERF_CORR_LMS_DCOFF_I, &i);
    if (ret != 0)
    {
        SoapySDR::logf(SOAPY_SDR_ERROR, "bladerf_get_correction() returned %s", _err2str(ret).c_str());
        throw std::runtime_error("getDCOffset() " + _err2str(ret));
    }

    ret = bladerf_get_correction(_dev, _toch(direction, channel), BLADERF_CORR_LMS_DCOFF_Q, &q);
    if (ret != 0)
    {
        SoapySDR::logf(SOAPY_SDR_ERROR, "bladerf_get_correction() returned %s", _err2str(ret).c_str());
        throw std::runtime_error("getDCOffset() " + _err2str(ret));
    }

    std::complex<double> z(i / 2048.0f, q / 2048.0f);
    return z;
}

bool bladeRF_SoapySDR::hasIQBalance(const int, const size_t channel) const
{
    return true;
}

void bladeRF_SoapySDR::setIQBalance(const int direction, const size_t channel, const std::complex<double> &balance)
{
    int ret = 0;
    int16_t gain = 0;
    int16_t phase = 0;

    if (balance.real() > 1.0)
        gain = int16_t(1.0 * 4096);
    else
        gain = int16_t(balance.real() * 4096);

    if (balance.imag() > 1.0)
        phase = int16_t(1.0 * 4096);
    else
        phase = int16_t(balance.imag() * 4096);

    ret = bladerf_set_correction(_dev, _toch(direction, channel), BLADERF_CORR_FPGA_GAIN, gain);
    if (ret != 0)
    {
        SoapySDR::logf(SOAPY_SDR_ERROR, "bladerf_set_correction(%f) returned %s", gain, _err2str(ret).c_str());
        throw std::runtime_error("setIQBalance() " + _err2str(ret));
    }

    ret = bladerf_set_correction(_dev, _toch(direction, channel), BLADERF_CORR_FPGA_PHASE, phase);
    if (ret != 0)
    {
        SoapySDR::logf(SOAPY_SDR_ERROR, "bladerf_set_correction(%f) returned %s", phase, _err2str(ret).c_str());
        throw std::runtime_error("setIQBalance() " + _err2str(ret));
    }
}

std::complex<double> bladeRF_SoapySDR::getIQBalance(const int direction, const size_t channel) const
{
    int ret = 0;
    int16_t gain = 0;
    int16_t phase = 0;

    ret = bladerf_get_correction(_dev, _toch(direction, channel), BLADERF_CORR_FPGA_GAIN, &gain);
    if (ret != 0)
    {
        SoapySDR::logf(SOAPY_SDR_ERROR, "bladerf_get_correction() returned %s", _err2str(ret).c_str());
        throw std::runtime_error("getIQBalance() " + _err2str(ret));
    }

    ret = bladerf_get_correction(_dev, _toch(direction, channel), BLADERF_CORR_FPGA_PHASE, &phase);
    if (ret != 0)
    {
        SoapySDR::logf(SOAPY_SDR_ERROR, "bladerf_get_correction() returned %s", _err2str(ret).c_str());
        throw std::runtime_error("getIQBalance() " + _err2str(ret));
    }

    std::complex<double> z(gain / 4096.0f, phase / 4096.0f);
    return z;
}

/*******************************************************************
 * Gain API
 ******************************************************************/

bool bladeRF_SoapySDR::hasGainMode(const int direction, const size_t channel) const
{
    #ifdef HAS_BLADERF_GAIN_MODE
    return _toch(direction, channel) == BLADERF_MODULE_RX ? true : false;
    #else
    return false;
    #endif
}

void bladeRF_SoapySDR::setGainMode(const int direction, const size_t channel, const bool automatic)
{
    #ifdef HAS_BLADERF_GAIN_MODE
    if (direction == SOAPY_SDR_TX) return; //not supported on tx
    bladerf_gain_mode gain_mode = automatic ? BLADERF_GAIN_AUTOMATIC : BLADERF_GAIN_MANUAL;
    const int ret = bladerf_set_gain_mode(_dev, _toch(direction, channel), gain_mode);
    if (ret != 0 and automatic) //only throw when mode is automatic, manual is default even when call bombs
    {
        SoapySDR::logf(SOAPY_SDR_ERROR, "bladerf_set_gain_mode(%s) returned %s", automatic?"automatic":"manual", _err2str(ret).c_str());
        throw std::runtime_error("setGainMode() " + _err2str(ret));
    }
    #endif
}

bool bladeRF_SoapySDR::getGainMode(const int direction, const size_t channel) const
{
    #ifdef HAS_BLADERF_GAIN_MODE
    if (direction == SOAPY_SDR_TX) return false; //not supported on tx
    int ret = 0;
    bladerf_gain_mode gain_mode;
    bool automatic;
    ret = bladerf_get_gain_mode(_dev, _toch(direction, channel), &gain_mode);
    if (ret != 0)
    {
        SoapySDR::logf(SOAPY_SDR_ERROR, "bladerf_get_gain_mode() returned %s", _err2str(ret).c_str());
        throw std::runtime_error("getGainMode() " + _err2str(ret));
    }

    automatic = gain_mode == BLADERF_GAIN_AUTOMATIC ? true : false;
    return automatic;
    #else
    return false;
    #endif
}

std::vector<std::string> bladeRF_SoapySDR::listGains(const int direction, const size_t channel) const
{
    std::vector<std::string> options;
    if (direction == SOAPY_SDR_RX) options.push_back("LNA");
    options.push_back("VGA1");
    options.push_back("VGA2");
    return options;
}

void bladeRF_SoapySDR::setGain(const int direction, const size_t channel, const double value)
{
    const int ret = bladerf_set_gain(_dev, _toch(direction, channel), int(value));
    if (ret != 0)
    {
        SoapySDR::logf(SOAPY_SDR_ERROR, "bladerf_set_gain(%f) returned %s", value, _err2str(ret).c_str());
        throw std::runtime_error("setGain() " + _err2str(ret));
    }
}

void bladeRF_SoapySDR::setGain(const int direction, const size_t channel, const std::string &name, const double value)
{
    int ret = 0;
    if (direction == SOAPY_SDR_RX and name == "LNA")
    {
        if      (value < 1.5) ret = bladerf_set_lna_gain(_dev, BLADERF_LNA_GAIN_BYPASS);
        else if (value < 4.5) ret = bladerf_set_lna_gain(_dev, BLADERF_LNA_GAIN_MID);
        else                  ret = bladerf_set_lna_gain(_dev, BLADERF_LNA_GAIN_MAX);
    }
    else if (direction == SOAPY_SDR_RX and name == "VGA1") ret = bladerf_set_rxvga1(_dev, int(value));
    else if (direction == SOAPY_SDR_RX and name == "VGA2") ret = bladerf_set_rxvga2(_dev, int(value));
    else if (direction == SOAPY_SDR_TX and name == "VGA1") ret = bladerf_set_txvga1(_dev, int(value));
    else if (direction == SOAPY_SDR_TX and name == "VGA2") ret = bladerf_set_txvga2(_dev, int(value));
    else throw std::runtime_error("setGain("+name+") -- unknown name");
    if (ret != 0)
    {
        SoapySDR::logf(SOAPY_SDR_ERROR, "bladerf_set_vga(%f) returned %s", value, _err2str(ret).c_str());
        throw std::runtime_error("setGain("+name+") " + _err2str(ret));
    }
}

double bladeRF_SoapySDR::getGain(const int direction, const size_t channel, const std::string &name) const
{
    int ret = 0;
    int gain = 0;
    if (direction == SOAPY_SDR_RX and name == "LNA")
    {
        bladerf_lna_gain lnaGain;
        ret = bladerf_get_lna_gain(_dev, &lnaGain);
        switch (lnaGain)
        {
        case BLADERF_LNA_GAIN_UNKNOWN: gain = 0; break;
        case BLADERF_LNA_GAIN_BYPASS: gain = 0; break;
        case BLADERF_LNA_GAIN_MID: gain = BLADERF_LNA_GAIN_MID_DB; break;
        case BLADERF_LNA_GAIN_MAX: gain = BLADERF_LNA_GAIN_MAX_DB; break;
        }
    }
    else if (direction == SOAPY_SDR_RX and name == "VGA1") ret = bladerf_get_rxvga1(_dev, &gain);
    else if (direction == SOAPY_SDR_RX and name == "VGA2") ret = bladerf_get_rxvga2(_dev, &gain);
    else if (direction == SOAPY_SDR_TX and name == "VGA1") ret = bladerf_get_txvga1(_dev, &gain);
    else if (direction == SOAPY_SDR_TX and name == "VGA2") ret = bladerf_get_txvga2(_dev, &gain);
    else throw std::runtime_error("getGain("+name+") -- unknown name");
    if (ret != 0)
    {
        SoapySDR::logf(SOAPY_SDR_ERROR, "bladerf_get_vga() returned %s", _err2str(ret).c_str());
        throw std::runtime_error("getGain("+name+") " + _err2str(ret));
    }
    return gain;
}

SoapySDR::Range bladeRF_SoapySDR::getGainRange(const int direction, const size_t channel, const std::string &name) const
{
    if (direction == SOAPY_SDR_RX and name == "LNA")  return SoapySDR::Range(0, BLADERF_LNA_GAIN_MAX_DB);
    if (direction == SOAPY_SDR_RX and name == "VGA1") return SoapySDR::Range(BLADERF_RXVGA1_GAIN_MIN, BLADERF_RXVGA1_GAIN_MAX);
    if (direction == SOAPY_SDR_RX and name == "VGA2") return SoapySDR::Range(BLADERF_RXVGA2_GAIN_MIN, BLADERF_RXVGA2_GAIN_MAX);
    if (direction == SOAPY_SDR_TX and name == "VGA1") return SoapySDR::Range(BLADERF_TXVGA1_GAIN_MIN, BLADERF_TXVGA1_GAIN_MAX);
    if (direction == SOAPY_SDR_TX and name == "VGA2") return SoapySDR::Range(BLADERF_TXVGA2_GAIN_MIN, BLADERF_TXVGA2_GAIN_MAX);
    else throw std::runtime_error("getGainRange("+name+") -- unknown name");
}

/*******************************************************************
 * Frequency API
 ******************************************************************/

void bladeRF_SoapySDR::setFrequency(const int direction, const size_t channel, const std::string &name, const double frequency, const SoapySDR::Kwargs &)
{
    if (name == "BB") return; //for compatibility
    if (name != "RF") throw std::runtime_error("setFrequency("+name+") unknown name");

    int ret = bladerf_set_frequency(_dev, _toch(direction, channel), (unsigned int)(frequency));
    if (ret != 0)
    {
        SoapySDR::logf(SOAPY_SDR_ERROR, "bladerf_set_frequency(%f) returned %s", frequency, _err2str(ret).c_str());
        throw std::runtime_error("setFrequency("+name+") " + _err2str(ret));
    }
}

double bladeRF_SoapySDR::getFrequency(const int direction, const size_t channel, const std::string &name) const
{
    if (name == "BB") return 0.0; //for compatibility
    if (name != "RF") throw std::runtime_error("getFrequency("+name+") unknown name");

    bladerf_frequency freq = 0;
    int ret = bladerf_get_frequency(_dev, _toch(direction, channel), &freq);
    if (ret != 0)
    {
        SoapySDR::logf(SOAPY_SDR_ERROR, "bladerf_get_frequency() returned %s", _err2str(ret).c_str());
        throw std::runtime_error("getFrequency("+name+") " + _err2str(ret));
    }
    return freq;
}

std::vector<std::string> bladeRF_SoapySDR::listFrequencies(const int, const size_t channel) const
{
    std::vector<std::string> components;
    components.push_back("RF");
    return components;
}

SoapySDR::RangeList bladeRF_SoapySDR::getFrequencyRange(const int, const size_t channel, const std::string &name) const
{
    if (name == "BB") return SoapySDR::RangeList(1, SoapySDR::Range(0.0, 0.0)); //for compatibility
    if (name != "RF") throw std::runtime_error("getFrequencyRange("+name+") unknown name");

    const bool has_xb200 = bladerf_expansion_attach(_dev, BLADERF_XB_200) != 0;
    const double minFreq = has_xb200?BLADERF_FREQUENCY_MIN_XB200:BLADERF_FREQUENCY_MIN;
    return SoapySDR::RangeList(1, SoapySDR::Range(minFreq, BLADERF_FREQUENCY_MAX));
}

/*******************************************************************
 * Sample Rate API
 ******************************************************************/

void bladeRF_SoapySDR::setSampleRate(const int direction, const size_t channel, const double rate)
{
    bladerf_rational_rate ratRate;
    ratRate.integer = uint64_t(rate);
    ratRate.den = uint64_t(1 << 14); //arbitrary denominator -- should be big enough
    ratRate.num = uint64_t(rate - ratRate.integer) * ratRate.den;

    //stash the approximate hardware time so it can be restored
    const long long timeNow = this->getHardwareTime();

    int ret = bladerf_set_rational_sample_rate(_dev, _toch(direction, channel), &ratRate, NULL);
    if (ret != 0)
    {
        SoapySDR::logf(SOAPY_SDR_ERROR, "bladerf_set_rational_sample_rate(%f) returned %s", rate, _err2str(ret).c_str());
        throw std::runtime_error("setSampleRate() " + _err2str(ret));
    }

    //stash the actual rate
    const double actual = this->getSampleRate(direction, channel);
    if (direction == SOAPY_SDR_RX)
    {
        _rxSampRate = actual;
        this->updateRxMinTimeoutMs();
    }
    if (direction == SOAPY_SDR_TX)
    {
        _txSampRate = actual;
    }

    //restore the previous hardware time setting (after rate stash)
    this->setHardwareTime(timeNow);

    SoapySDR::logf(SOAPY_SDR_INFO, "setSampleRate(%d, %f MHz), actual = %f MHz", direction, rate/1e6, actual/1e6);
}

double bladeRF_SoapySDR::getSampleRate(const int direction, const size_t channel) const
{
    bladerf_rational_rate ratRate;
    int ret = bladerf_get_rational_sample_rate(_dev, _toch(direction, channel), &ratRate);
    if (ret != 0)
    {
        SoapySDR::logf(SOAPY_SDR_ERROR, "bladerf_get_rational_sample_rate() returned %s", _err2str(ret).c_str());
        throw std::runtime_error("getSampleRate() " + _err2str(ret));
    }

    return double(ratRate.integer) + (double(ratRate.num)/double(ratRate.den));
}

std::vector<double> bladeRF_SoapySDR::listSampleRates(const int, const size_t channel) const
{
    std::vector<double> options;
    for (double r = 160e3; r <= 200e3; r += 40e3) options.push_back(r);
    for (double r = 300e3; r <= 900e3; r += 100e3) options.push_back(r);
    for (double r = 1e6; r <= 40e6; r += 1e6) options.push_back(r);
    //options.push_back(BLADERF_SAMPLERATE_MIN);
    //options.push_back(BLADERF_SAMPLERATE_REC_MAX);
    return options;
}

void bladeRF_SoapySDR::setBandwidth(const int direction, const size_t channel, const double bw)
{
    //bypass the filter when sufficiently large BW is selected
    if (bw > BLADERF_BANDWIDTH_MAX)
    {
        bladerf_set_lpf_mode(_dev, _toch(direction, channel), BLADERF_LPF_BYPASSED);
        return;
    }

    //otherwise set to normal and configure the filter bandwidth
    bladerf_set_lpf_mode(_dev, _toch(direction, channel), BLADERF_LPF_NORMAL);
    int ret = bladerf_set_bandwidth(_dev, _toch(direction, channel), (unsigned int)(bw), NULL);
    if (ret != 0)
    {
        SoapySDR::logf(SOAPY_SDR_ERROR, "bladerf_set_bandwidth(%f) returned %s", bw, _err2str(ret).c_str());
        throw std::runtime_error("setBandwidth() " + _err2str(ret));
    }
}

double bladeRF_SoapySDR::getBandwidth(const int direction, const size_t channel) const
{
    unsigned int bw = 0;
    int ret = bladerf_get_bandwidth(_dev, _toch(direction, channel), &bw);
    if (ret != 0)
    {
        SoapySDR::logf(SOAPY_SDR_ERROR, "bladerf_get_bandwidth() returned %s", _err2str(ret).c_str());
        throw std::runtime_error("getBandwidth() " + _err2str(ret));
    }
    return bw;
}

std::vector<double> bladeRF_SoapySDR::listBandwidths(const int, const size_t) const
{
    std::vector<double> options;
    options.push_back(0.75);
    options.push_back(0.875);
    options.push_back(1.25);
    options.push_back(1.375);
    options.push_back(1.5);
    options.push_back(1.92);
    options.push_back(2.5);
    options.push_back(2.75);
    options.push_back(3);
    options.push_back(3.5);
    options.push_back(4.375);
    options.push_back(5);
    options.push_back(6);
    options.push_back(7);
    options.push_back(10);
    options.push_back(14);
    for (size_t i = 0; i < options.size(); i++) options[i] *= 2e6;
    //options.push_back(BLADERF_BANDWIDTH_MIN);
    //options.push_back(BLADERF_BANDWIDTH_MAX);
    return options;
}

/*******************************************************************
 * Time API
 ******************************************************************/

bool bladeRF_SoapySDR::hasHardwareTime(const std::string &what) const
{
    if (not what.empty()) return SoapySDR::Device::hasHardwareTime(what);
    return true;
}

long long bladeRF_SoapySDR::getHardwareTime(const std::string &what) const
{
    if (not what.empty()) return SoapySDR::Device::getHardwareTime(what);
    uint64_t ticksNow = 0;
    #ifndef LIBBLADERF_V2
    const int ret = bladerf_get_timestamp(_dev, BLADERF_MODULE_RX, &ticksNow);
    #else
    const int ret = bladerf_get_timestamp(_dev, BLADERF_RX, &ticksNow);
    #endif

    if (ret != 0)
    {
        SoapySDR::logf(SOAPY_SDR_ERROR, "bladerf_get_timestamp() returned %s", _err2str(ret).c_str());
        throw std::runtime_error("getHardwareTime() " + _err2str(ret));
    }

    return _rxTicksToTimeNs(ticksNow);
}

void bladeRF_SoapySDR::setHardwareTime(const long long timeNs, const std::string &what)
{
    if (not what.empty()) return SoapySDR::Device::setHardwareTime(timeNs, what);

    //reset the counters with GPIO and stash the offset
    //this is the same as setting the time because
    //we maintain the offset math within the driver

    int ret = 0;
    uint32_t original = 0;
    ret |= bladerf_config_gpio_read(_dev, &original);
    ret |= bladerf_config_gpio_write(_dev, original & ~(BLADERF_GPIO_TIMESTAMP));
    ret |= bladerf_config_gpio_write(_dev, original | BLADERF_GPIO_TIMESTAMP);

    if (ret != 0)
    {
        SoapySDR::logf(SOAPY_SDR_ERROR, "bladerf_config_gpio_read/write() returned %s", _err2str(ret).c_str());
        throw std::runtime_error("setHardwareTime() " + _err2str(ret));
    }

    _timeNsOffset = timeNs;
}

/*******************************************************************
 * Register API
 ******************************************************************/

void bladeRF_SoapySDR::writeRegister(const unsigned addr, const unsigned value)
{
    const int ret = bladerf_lms_write(_dev, uint8_t(addr), uint8_t(value));
    if (ret != 0)
    {
        SoapySDR::logf(SOAPY_SDR_ERROR, "bladerf_lms_write(0x%x) returned %s", addr, _err2str(ret).c_str());
        throw std::runtime_error("writeRegister() " + _err2str(ret));
    }
}

unsigned bladeRF_SoapySDR::readRegister(const unsigned addr) const
{
    uint8_t value = 0;
    const int ret = bladerf_lms_read(_dev, uint8_t(addr), &value);
    if (ret != 0)
    {
        SoapySDR::logf(SOAPY_SDR_ERROR, "bladerf_lms_read(0x%x) returned %s", addr, _err2str(ret).c_str());
        throw std::runtime_error("readRegister() " + _err2str(ret));
    }
    return value;
}

/*******************************************************************
* Settings API
******************************************************************/

SoapySDR::ArgInfoList bladeRF_SoapySDR::getSettingInfo(void) const
{
    SoapySDR::ArgInfoList setArgs;

    // XB200 setting
    SoapySDR::ArgInfo xb200SettingArg;
    xb200SettingArg.key = "xb200";
    xb200SettingArg.value = "disabled";
    xb200SettingArg.name = "XB200 Transverter";
    xb200SettingArg.description = "bladeRF XB200 Transverter Board";
    xb200SettingArg.type = SoapySDR::ArgInfo::STRING;
    xb200SettingArg.options.push_back("disabled");
    xb200SettingArg.optionNames.push_back("Disabled");
    xb200SettingArg.options.push_back("50M");
    xb200SettingArg.optionNames.push_back("Filterbank: 50M");
    xb200SettingArg.options.push_back("144M");
    xb200SettingArg.optionNames.push_back("Filterbank: 144M");
    xb200SettingArg.options.push_back("222M");
    xb200SettingArg.optionNames.push_back("Filterbank: 222M");
    xb200SettingArg.options.push_back("auto1db");
    xb200SettingArg.optionNames.push_back("Filterbank: Auto (1dB)");
    xb200SettingArg.options.push_back("auto3db");
    xb200SettingArg.optionNames.push_back("Filterbank: Auto (3dB)");
    xb200SettingArg.options.push_back("auto");
    xb200SettingArg.optionNames.push_back("Filterbank: Auto");
    xb200SettingArg.options.push_back("custom");
    xb200SettingArg.optionNames.push_back("Filterbank: Custom");

    setArgs.push_back(xb200SettingArg);

    // Sampling mode
    SoapySDR::ArgInfo samplingModeArg;
    samplingModeArg.key = "sampling_mode";
    samplingModeArg.value = "internal";
    samplingModeArg.name = "Sampling Mode";
    samplingModeArg.description = "Internal = Via RX/TX connectors, External = Direct sampling from J60/J61 connectors";
    samplingModeArg.type = SoapySDR::ArgInfo::STRING;
    samplingModeArg.options.push_back("internal");
    samplingModeArg.optionNames.push_back("Internal (Default)");
    samplingModeArg.options.push_back("external");
    samplingModeArg.optionNames.push_back("Direct Sampling");

    setArgs.push_back(samplingModeArg);

    // Loopback
    SoapySDR::ArgInfo lookbackArg;
    lookbackArg.key = "loopback";
    lookbackArg.value = "disabled";
    lookbackArg.name = "Loopback Mode";
    lookbackArg.description = "Enable/disable internal loopback";
    lookbackArg.type = SoapySDR::ArgInfo::STRING;
    lookbackArg.options.push_back("disabled");
    lookbackArg.optionNames.push_back("Disabled");
    lookbackArg.options.push_back("firmware");
    lookbackArg.optionNames.push_back("FX3 Firmware");
    lookbackArg.options.push_back("bb_txlpf_rxvga2");
    lookbackArg.optionNames.push_back("Baseband: TXLPF to RXVGA2");
    lookbackArg.options.push_back("bb_txvga1_rxvga2");
    lookbackArg.optionNames.push_back("Baseband: TXVGA1 to RXVGA2");
    lookbackArg.options.push_back("bb_txlpf_rxlpf");
    lookbackArg.optionNames.push_back("Baseband: TXLPF to RXLPF");
    lookbackArg.options.push_back("bb_txvga1_rxlpf");
    lookbackArg.optionNames.push_back("Baseband: TXVGA1 to RXLPF");
    lookbackArg.options.push_back("rf_lna1");
    lookbackArg.optionNames.push_back("RF: TXMIX to LNA1");
    lookbackArg.options.push_back("rf_lna2");
    lookbackArg.optionNames.push_back("RF: TXMIX to LNA2");
    lookbackArg.options.push_back("rf_lna3");
    lookbackArg.optionNames.push_back("RF: TXMIX to LNA3");

    setArgs.push_back(lookbackArg);

    // Device reset
    SoapySDR::ArgInfo resetArg;
    resetArg.key = "reset";
    resetArg.value = "true";
    resetArg.name = "Reset Device";
    resetArg.description = "Reset the device, causing it to reload its firmware from flash.";
    resetArg.type = SoapySDR::ArgInfo::STRING;
    resetArg.options.push_back("true");
    resetArg.optionNames.push_back("True");

    setArgs.push_back(resetArg);

    // Erase stored FPGA
    SoapySDR::ArgInfo eraseArg;
    eraseArg.key = "erase_stored_fpga";
    eraseArg.value = "true";
    eraseArg.name = "Erase the FPGA region of flash";
    eraseArg.description = "Erase the FPGA region of SPI flash, effectively disabling FPGA autoloading.";
    eraseArg.type = SoapySDR::ArgInfo::STRING;
    eraseArg.options.push_back("true");
    eraseArg.optionNames.push_back("True");

    setArgs.push_back(eraseArg);

    // Flash firmware
    SoapySDR::ArgInfo firmwareArg;
    firmwareArg.key = "flash_firmware";
    firmwareArg.value = "";
    firmwareArg.name = "Write FX3 firmware to flash";
    firmwareArg.description = "Write FX3 firmware to the bladeRF's SPI flash from the provided file path. This will require a power cycle to take effect.";
    firmwareArg.type = SoapySDR::ArgInfo::STRING;

    setArgs.push_back(firmwareArg);

    // Flash FPGA
    SoapySDR::ArgInfo flashArg;
    flashArg.key = "flash_fpga";
    flashArg.value = "";
    flashArg.name = "Write to the FPGA region of flash";
    flashArg.description = "Write FPGA image to the bladeRF's SPI flash from the provided file path and enable FPGA loading from SPI flash at power on.";
    flashArg.type = SoapySDR::ArgInfo::STRING;

    setArgs.push_back(flashArg);

    // Jump to bootloader
    SoapySDR::ArgInfo bootloaderArg;
    bootloaderArg.key = "jump_to_bootloader";
    bootloaderArg.value = "true";
    bootloaderArg.name = "Clear out a firmware signature word in flash and jump to FX3 bootloader";
    bootloaderArg.description = "The device will continue to boot into the FX3 bootloader across power cycles until new firmware is written to the device.";
    bootloaderArg.type = SoapySDR::ArgInfo::STRING;
    bootloaderArg.options.push_back("true");
    bootloaderArg.optionNames.push_back("True");

    setArgs.push_back(bootloaderArg);

    // Load FPGA
    SoapySDR::ArgInfo loadArg;
    loadArg.key = "load_fpga";
    loadArg.value = "";
    loadArg.name = "Load device's FPGA";
    loadArg.description = "Load device's FPGA from the provided file path. Note that this FPGA configuration will be reset at the next power cycle.";
    loadArg.type = SoapySDR::ArgInfo::STRING;

    setArgs.push_back(loadArg);

    return setArgs;
}

std::string bladeRF_SoapySDR::readSetting(const std::string &key) const
{
    if (key == "xb200") {
        return _xb200Mode;
    } else if (key == "sampling_mode") {
        return _samplingMode;
    } else if (key == "loopback") {
        return _loopbackMode;
    } else if (key == "reset") {
        return "";
    } else if (key == "erase_stored_fpga") {
        return "";
    } else if (key == "flash_firmware") {
        return "";
    } else if (key == "flash_fpga") {
        return "";
    } else if (key == "jump_to_bootloader") {
        return "";
    } else if (key == "load_fpga") {
        return "";
    }

    SoapySDR_logf(SOAPY_SDR_WARNING, "Unknown setting '%s'", key.c_str());
    return "";
}

void bladeRF_SoapySDR::writeSetting(const std::string &key, const std::string &value)
{
    if (key == "xb200")
    {
        #ifndef LIBBLADERF_V2
        // Verify that a valid setting has arrived
        std::vector<std::string> xb200_validSettings{ "disabled", "50M", "144M", "222M", "auto1db", "auto3db", "auto", "custom" };
        if (std::find(std::begin(xb200_validSettings), std::end(xb200_validSettings), value) != std::end(xb200_validSettings))
        {
            // --> Valid setting has arrived
            _xb200Mode = value;

            // Get attached expansion device
            bladerf_xb _bladerf_xb_attached = bladerf_xb::BLADERF_XB_NONE;
            bladerf_expansion_get_attached(_dev, &_bladerf_xb_attached);

            // If "disabled," ensure board is bypassed, if present, and return
            if (value == "disabled")
            {
                if (_bladerf_xb_attached == bladerf_xb::BLADERF_XB_200)
                {
                    // Apply bypass around connected XB200
                    SoapySDR::logf(SOAPY_SDR_INFO, "bladeRF: Disabling connected XB200 by bypassing signal path");
                    bladerf_xb200_set_path(_dev, bladerf_module::BLADERF_MODULE_RX, bladerf_xb200_path::BLADERF_XB200_BYPASS);
                }

                return;
            }

            // Attach the XB200, if it isn't already attached
            if (_bladerf_xb_attached == bladerf_xb::BLADERF_XB_NONE)
            {
                if (bladerf_expansion_attach(_dev, bladerf_xb::BLADERF_XB_200))
                {
                    SoapySDR::logf(SOAPY_SDR_ERROR, "bladeRF: Could not attach to XB200");
                    return;
                }
            }
            SoapySDR::logf(SOAPY_SDR_INFO, "bladeRF: XB200 is attached");

            // Which filterbank was selected?
            bladerf_xb200_filter filter = bladerf_xb200_filter::BLADERF_XB200_AUTO_1DB;

            if (value == "50M")
            {
                // 50-54 MHz (6 meter band) filterbank
                filter = bladerf_xb200_filter::BLADERF_XB200_50M;
            }
            else if (value == "144M")
            {
                // 144-148 MHz (2 meter band) filterbank
                filter = bladerf_xb200_filter::BLADERF_XB200_144M;
            }
            else if (value == "222M")
            {
                // 222-225 MHz (1.25 meter band) filterbank
                // Note that this filter option is technically wider, covering 206-235 MHz
                filter = bladerf_xb200_filter::BLADERF_XB200_222M;
            }
            else if (value == "auto1db")
            {
                // The other filter options are automatically selected depending on the RX or TX
                // module's current frequency, based upon the 1dB points of the on-board filters
                // For frequencies outside the range of the on-board filters, the custom path is used
                filter = bladerf_xb200_filter::BLADERF_XB200_AUTO_1DB;
            }
            else if (value == "auto3db")
            {
                // The other filter options are automatically selected depending on the RX or TX
                // module's current frequency, based upon the 3dB points of the on-board filters
                // For frequencies outside the range of the on-board filters, the custom path is used
                filter = bladerf_xb200_filter::BLADERF_XB200_AUTO_3DB;
            }
            else if (value == "custom")
            {
                // The custom filter bank path across the FILT and FILT-ANT SMA connectors
                filter = bladerf_xb200_filter::BLADERF_XB200_CUSTOM;
            }
            else
            {
                // Default: Auto, 1dB points
                // The other filter options are automatically selected depending on the RX or TX
                // module's current frequency, based upon the 1dB points of the on-board filters
                // For frequencies outside the range of the on-board filters, the custom path is used
                filter = bladerf_xb200_filter::BLADERF_XB200_AUTO_1DB;
            }

            // Set the filterbank
            SoapySDR::logf(SOAPY_SDR_INFO, "bladeRF: Set XB200 filterbank '%s'", value.c_str());
            int ret = bladerf_xb200_set_filterbank(_dev, bladerf_module::BLADERF_MODULE_RX, filter);
            if (ret != 0)
            {
                SoapySDR::logf(SOAPY_SDR_ERROR, "bladerf_xb200_set_filterbank(%s) returned %s", value.c_str(), _err2str(ret).c_str());
                throw std::runtime_error("writeSetting() " + _err2str(ret));
            }

            // Check signal path
            bladerf_xb200_path _bladerf_xb200_path = bladerf_xb200_path::BLADERF_XB200_MIX;
            bladerf_xb200_get_path(_dev, bladerf_module::BLADERF_MODULE_RX, &_bladerf_xb200_path);
            if (_bladerf_xb200_path != bladerf_xb200_path::BLADERF_XB200_MIX)
            {
                // Apply mix path through XB200
                SoapySDR::logf(SOAPY_SDR_INFO, "bladeRF: Adjusting mix path through XB200");
                bladerf_xb200_set_path(_dev, bladerf_module::BLADERF_MODULE_RX, bladerf_xb200_path::BLADERF_XB200_MIX);
            }
        }
        else
        {
            // --> Invalid setting has arrived
            SoapySDR::logf(SOAPY_SDR_ERROR, "bladeRF: Invalid XB200 setting '%s'", value.c_str());
            //throw std::runtime_error("writeSetting(" + key + "," + value + ") unknown value");
        }
        #endif
    }
    else if (key == "sampling_mode")
    {
        /* Configure the sampling of the LMS6002D to be either internal or external.
        ** Internal sampling will read from the RXVGA2 driver internal to the chip.
        ** External sampling will connect the ADC inputs to the external inputs for direct sampling.
        */

        // Verify that a valid setting has arrived
        std::vector<std::string> sampling_mode_validSettings{ "internal", "external" };
        if (std::find(std::begin(sampling_mode_validSettings), std::end(sampling_mode_validSettings), value) != std::end(sampling_mode_validSettings))
        {
            // --> Valid setting has arrived
            _samplingMode = value;

            // Set the sampling mode
            int ret = 0;
            if (value == "external")
            {
                // External/direct sampling
                SoapySDR::logf(SOAPY_SDR_INFO, "bladeRF: Set sampling mode to direct/external sampling", value.c_str());
                ret = bladerf_set_sampling(_dev, bladerf_sampling::BLADERF_SAMPLING_EXTERNAL);
            }
            else
            {
                // Default: Internal
                SoapySDR::logf(SOAPY_SDR_INFO, "bladeRF: Set sampling mode to internal sampling", value.c_str());
                ret = bladerf_set_sampling(_dev, bladerf_sampling::BLADERF_SAMPLING_INTERNAL);
            }
            if (ret != 0)
            {
                SoapySDR::logf(SOAPY_SDR_ERROR, "bladerf_set_sampling(%s) returned %s", value.c_str(), _err2str(ret).c_str());
                throw std::runtime_error("writeSetting() " + _err2str(ret));
            }
        }
        else
        {
            // --> Invalid setting has arrived
            SoapySDR::logf(SOAPY_SDR_ERROR, "bladeRF: Invalid sampling mode '%s'", value.c_str());
            //throw std::runtime_error("writeSetting(" + key + "," + value + ") unknown value");
        }
    }
    else if (key == "loopback")
    {
        // Verify that a valid setting has arrived
        #ifdef LIBBLADERF_V2
        std::vector<std::string> loopback_validSettings{ "disabled", "firmware", "bb_txlpf_rxvga2", "bb_txvga1_rxvga2", "bb_txlpf_rxlpf", "bb_txvga1_rxlpf", "rf_lna1", "rf_lna2", "rf_lna3", "rfic_bist"};
        #else
        std::vector<std::string> loopback_validSettings{ "disabled", "firmware", "bb_txlpf_rxvga2", "bb_txvga1_rxvga2", "bb_txlpf_rxlpf", "bb_txvga1_rxlpf", "rf_lna1", "rf_lna2", "rf_lna3"};
        #endif
        if (std::find(std::begin(loopback_validSettings), std::end(loopback_validSettings), value) != std::end(loopback_validSettings))
        {
            // --> Valid setting has arrived
            _loopbackMode = value;

            // Which loopback mode was selected?
            bladerf_loopback loopback = bladerf_loopback::BLADERF_LB_NONE;

            if (value == "firmware")
            {
                // Firmware loopback inside of the FX3
                loopback = bladerf_loopback::BLADERF_LB_FIRMWARE;
            }
            else if (value == "bb_txlpf_rxvga2")
            {
                // Baseband loopback. TXLPF output is connected to the RXVGA2 input.
                loopback = bladerf_loopback::BLADERF_LB_BB_TXLPF_RXVGA2;
            }
            else if (value == "bb_txvga1_rxvga2")
            {
                // Baseband loopback. TXVGA1 output is connected to the RXVGA2 input.
                loopback = bladerf_loopback::BLADERF_LB_BB_TXVGA1_RXVGA2;
            }
            else if (value == "bb_txlpf_rxlpf")
            {
                // Baseband loopback. TXLPF output is connected to the RXLPF input.
                loopback = bladerf_loopback::BLADERF_LB_BB_TXLPF_RXLPF;
            }
            else if (value == "bb_txvga1_rxlpf")
            {
                // Baseband loopback. TXVGA1 output is connected to RXLPF input.
                loopback = bladerf_loopback::BLADERF_LB_BB_TXVGA1_RXLPF;
            }
            else if (value == "rf_lna1")
            {
                // RF loopback. The TXMIX output, through the AUX PA, is connected to the output of LNA1.
                loopback = bladerf_loopback::BLADERF_LB_RF_LNA1;
            }
            else if (value == "rf_lna2")
            {
                // RF loopback. The TXMIX output, through the AUX PA, is connected to the output of LNA2.
                loopback = bladerf_loopback::BLADERF_LB_RF_LNA2;
            }
            else if (value == "rf_lna3")
            {
                // RF loopback. The TXMIX output, through the AUX PA, is connected to the output of LNA3.
                loopback = bladerf_loopback::BLADERF_LB_RF_LNA3;
            }
            #ifdef LIBBLADERF_V2
            else if (value == "rfic_bist")
            {
                // RF IC Built In Self Test loopback.
                loopback = bladerf_loopback::BLADERF_LB_RFIC_BIST;
            }
            #endif
            else
            {
                // Default: Disabled
                // Disables loopback and returns to normal operation
                loopback = bladerf_loopback::BLADERF_LB_NONE;
            }
            #ifdef LIBBLADERF_V2
            // Check if the loopback mode is supported by the board
            if (bladerf_is_loopback_mode_supported(_dev, loopback))
            #else
            if (true)
            #endif
            {

                // If the loopback isn't already set, set the loopback
                bladerf_loopback _bladerf_loopback = bladerf_loopback::BLADERF_LB_NONE;
                bladerf_get_loopback(_dev, &_bladerf_loopback);
                if (_bladerf_loopback != loopback)
                {
                    SoapySDR::logf(SOAPY_SDR_INFO, "bladeRF: Loopback set '%s'", value.c_str());
                    int ret = bladerf_set_loopback(_dev, loopback);
                    if (ret != 0)
                    {
                        SoapySDR::logf(SOAPY_SDR_ERROR, "bladerf_set_loopback(%s) returned %s", value.c_str(), _err2str(ret).c_str());
                        throw std::runtime_error("writeSetting() " + _err2str(ret));
                    }
                }
            }
            else
            {
              // --> Invalid setting has arrived
              SoapySDR::logf(SOAPY_SDR_ERROR, "bladeRF: Loopback setting '%s' not available on this device.", value.c_str());
            }
        }
        else
        {
            // --> Invalid setting has arrived
            SoapySDR::logf(SOAPY_SDR_ERROR, "bladeRF: Invalid loopback setting '%s'", value.c_str());
            //throw std::runtime_error("writeSetting(" + key + "," + value + ") unknown value");
        }
    }
    else if (key == "reset")
    {
        // Verify that a valid setting has arrived
        if (value == "true") {
            // --> Valid setting has arrived
            int ret = bladerf_device_reset(_dev);
            if (ret != 0)
            {
                SoapySDR::logf(SOAPY_SDR_ERROR, "bladerf_device_reset(%s) returned %s", value.c_str(),
                               _err2str(ret).c_str());
                throw std::runtime_error("writeSetting() " + _err2str(ret));
            }
        }
        else {
            // --> Invalid setting has arrived
            SoapySDR::logf(SOAPY_SDR_ERROR, "bladeRF: Invalid reset setting '%s'", value.c_str());
        }
    }
    else if (key == "erase_stored_fpga")
    {
        // Verify that a valid setting has arrived
        if (value == "true") {
            // --> Valid setting has arrived
            int ret = bladerf_erase_stored_fpga(_dev);
            if (ret != 0)
            {
                SoapySDR::logf(SOAPY_SDR_ERROR, "bladerf_erase_stored_fpga(%s) returned %s", value.c_str(),
                               _err2str(ret).c_str());
                throw std::runtime_error("writeSetting() " + _err2str(ret));
            }
        }
        else {
            // --> Invalid setting has arrived
            SoapySDR::logf(SOAPY_SDR_ERROR, "bladeRF: Invalid erase setting '%s'", value.c_str());
        }
    }
    else if (key == "flash_firmware")
    {
        if (!value.empty()) {
            int ret = bladerf_flash_firmware(_dev, value.c_str());
            if (ret != 0) {
                SoapySDR::logf(SOAPY_SDR_ERROR, "bladerf_flash_firmware(%s) returned %s", value.c_str(),
                               _err2str(ret).c_str());
                throw std::runtime_error("writeSetting() " + _err2str(ret));
            }
        }
        else {
            // --> Invalid setting has arrived
            SoapySDR::logf(SOAPY_SDR_ERROR, "bladeRF: The provided firmware file path is empty");
        }
    }
    else if (key == "flash_fpga")
    {
        if (!value.empty()) {
            int ret = bladerf_flash_fpga(_dev, value.c_str());
            if (ret != 0) {
                SoapySDR::logf(SOAPY_SDR_ERROR, "bladerf_flash_fpga(%s) returned %s", value.c_str(),
                               _err2str(ret).c_str());
                throw std::runtime_error("writeSetting() " + _err2str(ret));
            }
        }
        else {
            // --> Invalid setting has arrived
            SoapySDR::logf(SOAPY_SDR_ERROR, "bladeRF: The provided FPGA image path is empty");
        }
    }
    else if (key == "jump_to_bootloader")
    {
        // Verify that a valid setting has arrived
        if (value == "true") {
            // --> Valid setting has arrived
            int ret = bladerf_jump_to_bootloader(_dev);
            if (ret != 0)
            {
                SoapySDR::logf(SOAPY_SDR_ERROR, "bladerf_jump_to_bootloader(%s) returned %s", value.c_str(),
                               _err2str(ret).c_str());
                throw std::runtime_error("writeSetting() " + _err2str(ret));
            }
        }
        else {
            // --> Invalid setting has arrived
            SoapySDR::logf(SOAPY_SDR_ERROR, "bladeRF: Invalid jump to bootloader setting '%s'", value.c_str());
        }
    }
    else if (key == "load_fpga")
    {
        if (!value.empty()) {
            int ret = bladerf_load_fpga(_dev, value.c_str());
            if (ret != 0) {
                SoapySDR::logf(SOAPY_SDR_ERROR, "bladerf_load_fpga(%s) returned %s", value.c_str(),
                               _err2str(ret).c_str());
                throw std::runtime_error("writeSetting() " + _err2str(ret));
            }
        }
        else {
            // --> Invalid setting has arrived
            SoapySDR::logf(SOAPY_SDR_ERROR, "bladeRF: The provided FPGA image path is empty");
        }
    }
    else
    {
        throw std::runtime_error("writeSetting(" + key + ") unknown setting");
    }
}

/*******************************************************************
 * GPIO API
 ******************************************************************/

std::vector<std::string> bladeRF_SoapySDR::listGPIOBanks(void) const
{
    std::vector<std::string> banks;
    banks.push_back("CONFIG");
    banks.push_back("EXPANSION");
    return banks;
}

void bladeRF_SoapySDR::writeGPIO(const std::string &bank, const unsigned value)
{
    int ret = 0;
    if (bank == "CONFIG")
    {
        ret = bladerf_config_gpio_write(_dev, value);
    }
    else if (bank == "EXPANSION")
    {
        ret = bladerf_expansion_gpio_write(_dev, value);
    }
    else throw std::runtime_error("writeGPIO("+bank+") unknown bank name");

    if (ret != 0) throw std::runtime_error("writeGPIO("+bank+") " + _err2str(ret));
}

void bladeRF_SoapySDR::writeGPIO(const std::string &bank, const unsigned value, const unsigned mask)
{
    #if defined(LIBBLADERF_API_VERSION) && (LIBBLADERF_API_VERSION >= 0x01050000)
    if (bank == "EXPANSION")
    {
        int ret = bladerf_expansion_gpio_masked_write(_dev, mask, value);
        if (ret != 0) throw std::runtime_error("writeGPIODir("+bank+") " + _err2str(ret));
        return;
    }
    #endif
    return SoapySDR::Device::writeGPIO(bank, value, mask);
}

unsigned bladeRF_SoapySDR::readGPIO(const std::string &bank) const
{
    uint32_t value = 0;
    int ret = 0;
    if (bank == "CONFIG")
    {
        ret = bladerf_config_gpio_read(_dev, &value);
    }
    else if (bank == "EXPANSION")
    {
        ret = bladerf_expansion_gpio_read(_dev, &value);
    }
    else throw std::runtime_error("readGPIO("+bank+") unknown bank name");

    if (ret != 0) throw std::runtime_error("readGPIO("+bank+") " + _err2str(ret));
    return value;
}

void bladeRF_SoapySDR::writeGPIODir(const std::string &bank, const unsigned dir)
{
    int ret = 0;
    if (bank == "CONFIG")
    {
        throw std::runtime_error("data direction not configurable for CONFIG bank");
    }
    else if (bank == "EXPANSION")
    {
        ret = bladerf_expansion_gpio_dir_write(_dev, dir);
    }
    else throw std::runtime_error("writeGPIODir("+bank+") unknown bank name");

    if (ret != 0) throw std::runtime_error("writeGPIODir("+bank+") " + _err2str(ret));
}

void bladeRF_SoapySDR::writeGPIODir(const std::string &bank, const unsigned dir, const unsigned mask)
{
    #if defined(LIBBLADERF_API_VERSION) && (LIBBLADERF_API_VERSION >= 0x01050000)
    if (bank == "EXPANSION")
    {
        int ret = bladerf_expansion_gpio_dir_masked_write(_dev, mask, dir);
        if (ret != 0) throw std::runtime_error("writeGPIODir("+bank+") " + _err2str(ret));
        return;
    }
    #endif
    return SoapySDR::Device::writeGPIODir(bank, dir, mask);
}

unsigned bladeRF_SoapySDR::readGPIODir(const std::string &bank) const
{
    uint32_t value = 0;
    int ret = 0;
    if (bank == "CONFIG")
    {
        throw std::runtime_error("data direction not configurable for CONFIG bank");
    }
    else if (bank == "EXPANSION")
    {
        ret = bladerf_expansion_gpio_dir_read(_dev, &value);
    }
    else throw std::runtime_error("readGPIODir("+bank+") unknown bank name");

    if (ret != 0) throw std::runtime_error("readGPIODir("+bank+") " + _err2str(ret));
    return value;
}
