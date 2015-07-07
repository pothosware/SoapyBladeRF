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
    this->setSampleRate(SOAPY_SDR_RX, 0, 1e6);
    this->setSampleRate(SOAPY_SDR_TX, 0, 1e6);
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
 * Gain API
 ******************************************************************/

std::vector<std::string> bladeRF_SoapySDR::listGains(const int direction, const size_t) const
{
    std::vector<std::string> options;
    if (direction == SOAPY_SDR_RX) options.push_back("LNA");
    options.push_back("VGA1");
    options.push_back("VGA2");
    return options;
}

void bladeRF_SoapySDR::setGain(const int direction, const size_t, const double value)
{
    const int ret = bladerf_set_gain(_dev, _dir2mod(direction), int(value));
    if (ret != 0)
    {
        SoapySDR::logf(SOAPY_SDR_ERROR, "bladerf_set_gain(%f) returned %s", value, _err2str(ret).c_str());
        throw std::runtime_error("setGain() " + _err2str(ret));
    }
}

void bladeRF_SoapySDR::setGain(const int direction, const size_t, const std::string &name, const double value)
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

double bladeRF_SoapySDR::getGain(const int direction, const size_t, const std::string &name) const
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

SoapySDR::Range bladeRF_SoapySDR::getGainRange(const int direction, const size_t, const std::string &name) const
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

void bladeRF_SoapySDR::setFrequency(const int direction, const size_t, const std::string &name, const double frequency, const SoapySDR::Kwargs &)
{
    if (name == "BB") return; //for compatibility
    if (name != "RF") throw std::runtime_error("setFrequency("+name+") unknown name");

    int ret = bladerf_set_frequency(_dev, _dir2mod(direction), (unsigned int)(frequency));
    if (ret != 0)
    {
        SoapySDR::logf(SOAPY_SDR_ERROR, "bladerf_set_frequency(%f) returned %s", frequency, _err2str(ret).c_str());
        throw std::runtime_error("setFrequency("+name+") " + _err2str(ret));
    }
}

double bladeRF_SoapySDR::getFrequency(const int direction, const size_t, const std::string &name) const
{
    if (name == "BB") return 0.0; //for compatibility
    if (name != "RF") throw std::runtime_error("getFrequency("+name+") unknown name");

    unsigned int freq = 0;
    int ret = bladerf_get_frequency(_dev, _dir2mod(direction), &freq);
    if (ret != 0)
    {
        SoapySDR::logf(SOAPY_SDR_ERROR, "bladerf_get_frequency() returned %s", _err2str(ret).c_str());
        throw std::runtime_error("getFrequency("+name+") " + _err2str(ret));
    }
    return freq;
}

std::vector<std::string> bladeRF_SoapySDR::listFrequencies(const int, const size_t) const
{
    std::vector<std::string> components;
    components.push_back("RF");
    return components;
}

SoapySDR::RangeList bladeRF_SoapySDR::getFrequencyRange(const int, const size_t, const std::string &name) const
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

    int ret = bladerf_set_rational_sample_rate(_dev, _dir2mod(direction), &ratRate, NULL);
    if (ret != 0)
    {
        SoapySDR::logf(SOAPY_SDR_ERROR, "bladerf_set_rational_sample_rate(%f) returned %s", rate, _err2str(ret).c_str());
        throw std::runtime_error("setSampleRate() " + _err2str(ret));
    }

    //stash the actual rate
    const double actual = this->getSampleRate(direction, channel);
    if (direction == SOAPY_SDR_RX) _rxSampRate = actual;
    if (direction == SOAPY_SDR_TX) _txSampRate = actual;

    //restore the previous hardware time setting (after rate stash)
    this->setHardwareTime(timeNow);

    SoapySDR::logf(SOAPY_SDR_INFO, "setSampleRate(%d, %f MHz), actual = %f MHz", direction, rate/1e6, actual/1e6);
}

double bladeRF_SoapySDR::getSampleRate(const int direction, const size_t) const
{
    bladerf_rational_rate ratRate;
    int ret = bladerf_get_rational_sample_rate(_dev, _dir2mod(direction), &ratRate);
    if (ret != 0)
    {
        SoapySDR::logf(SOAPY_SDR_ERROR, "bladerf_get_rational_sample_rate() returned %s", _err2str(ret).c_str());
        throw std::runtime_error("getSampleRate() " + _err2str(ret));
    }

    return double(ratRate.integer) + (double(ratRate.num)/double(ratRate.den));
}

std::vector<double> bladeRF_SoapySDR::listSampleRates(const int, const size_t) const
{
    std::vector<double> options;
    for (double r = 160e3; r <= 200e3; r += 40e3) options.push_back(r);
    for (double r = 300e3; r <= 900e3; r += 100e3) options.push_back(r);
    for (double r = 1e6; r <= 40e6; r += 1e6) options.push_back(r);
    //options.push_back(BLADERF_SAMPLERATE_MIN);
    //options.push_back(BLADERF_SAMPLERATE_REC_MAX);
    return options;
}

void bladeRF_SoapySDR::setBandwidth(const int direction, const size_t, const double bw)
{
    //bypass the filter when sufficiently large BW is selected
    if (bw > BLADERF_BANDWIDTH_MAX)
    {
        bladerf_set_lpf_mode(_dev, _dir2mod(direction), BLADERF_LPF_BYPASSED);
        return;
    }

    //otherwise set to normal and configure the filter bandwidth
    bladerf_set_lpf_mode(_dev, _dir2mod(direction), BLADERF_LPF_NORMAL);
    int ret = bladerf_set_bandwidth(_dev, _dir2mod(direction), (unsigned int)(bw), NULL);
    if (ret != 0)
    {
        SoapySDR::logf(SOAPY_SDR_ERROR, "bladerf_set_bandwidth(%f) returned %s", bw, _err2str(ret).c_str());
        throw std::runtime_error("setBandwidth() " + _err2str(ret));
    }
}

double bladeRF_SoapySDR::getBandwidth(const int direction, const size_t) const
{
    unsigned int bw = 0;
    int ret = bladerf_get_bandwidth(_dev, _dir2mod(direction), &bw);
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
    const int ret = bladerf_get_timestamp(_dev, BLADERF_MODULE_RX, &ticksNow);

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
