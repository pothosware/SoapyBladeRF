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
    _dev(NULL)
{
    bladerf_devinfo info = devinfo;
    int ret = bladerf_open_with_devinfo(&_dev, &info);

    if (ret < 0)
    {
        SoapySDR::logf(SOAPY_SDR_ERROR, "bladerf_open_with_devinfo() returned %d", ret);
        throw std::runtime_error("bladerf_open_with_devinfo() failed");
    }
}

bladeRF_SoapySDR::~bladeRF_SoapySDR(void)
{
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

void bladeRF_SoapySDR::setFrontendMapping(const int direction, const std::string &mapping)
{
    
}

std::string bladeRF_SoapySDR::getFrontendMapping(const int direction) const
{
    
}

/*******************************************************************
 * Antenna API
 ******************************************************************/

std::vector<std::string> bladeRF_SoapySDR::listAntennas(const int direction, const size_t channel) const
{
    
}

void bladeRF_SoapySDR::setAntenna(const int direction, const size_t channel, const std::string &name)
{
    
}

std::string bladeRF_SoapySDR::getAntenna(const int direction, const size_t channel) const
{
    
}

/*******************************************************************
 * Gain API
 ******************************************************************/

std::vector<std::string> bladeRF_SoapySDR::listGains(const int, const size_t) const
{
    std::vector<std::string> options;
    //same for rx and tx
    options.push_back("VGA1");
    options.push_back("VGA2");
    return options;
}

void bladeRF_SoapySDR::setGain(const int direction, const size_t channel, const double value)
{
    const int ret = bladerf_set_gain(_dev, _dir2mod(direction), int(value));
    //TODO ret
}

void bladeRF_SoapySDR::setGain(const int direction, const size_t channel, const std::string &name, const double value)
{
    int ret = 0;
    if      (direction == SOAPY_SDR_TX and name == "VGA1") ret = bladerf_set_txvga1(_dev, int(value));
    else if (direction == SOAPY_SDR_TX and name == "VGA2") ret = bladerf_set_txvga2(_dev, int(value));
}

double bladeRF_SoapySDR::getGain(const int direction, const size_t channel, const std::string &name) const
{
    int ret = 0;
    int gain = 0;
    if      (direction == SOAPY_SDR_TX and name == "VGA1") ret = bladerf_get_txvga1(_dev, &gain);
    else if (direction == SOAPY_SDR_TX and name == "VGA2") ret = bladerf_get_txvga2(_dev, &gain);
    
    return gain;
}

SoapySDR::Range bladeRF_SoapySDR::getGainRange(const int direction, const size_t channel, const std::string &name) const
{
    if (direction == SOAPY_SDR_TX and name == "VGA1") return SoapySDR::Range(BLADERF_TXVGA1_GAIN_MIN, BLADERF_TXVGA1_GAIN_MAX);
    if (direction == SOAPY_SDR_TX and name == "VGA2") return SoapySDR::Range(BLADERF_TXVGA2_GAIN_MIN, BLADERF_TXVGA2_GAIN_MAX);
    if (direction == SOAPY_SDR_RX and name == "VGA1") return SoapySDR::Range(BLADERF_RXVGA1_GAIN_MIN, BLADERF_RXVGA1_GAIN_MAX);
    if (direction == SOAPY_SDR_RX and name == "VGA2") return SoapySDR::Range(BLADERF_RXVGA2_GAIN_MIN, BLADERF_RXVGA2_GAIN_MAX);
}

/*******************************************************************
 * Frequency API
 ******************************************************************/

void bladeRF_SoapySDR::setFrequency(const int direction, const size_t channel, const double frequency, const SoapySDR::Kwargs &args)
{
    
}

void bladeRF_SoapySDR::setFrequency(const int direction, const size_t channel, const std::string &name, const double frequency, const SoapySDR::Kwargs &args)
{
    
}

double bladeRF_SoapySDR::getFrequency(const int direction, const size_t channel, const std::string &name) const
{
    
}

std::vector<std::string> bladeRF_SoapySDR::listFrequencies(const int direction, const size_t channel) const
{
    std::vector<std::string> components;
    components.push_back("RF");
    return components;
}

SoapySDR::RangeList bladeRF_SoapySDR::getFrequencyRange(const int direction, const size_t channel, const std::string &name) const
{
    if (name == "RF") return SoapySDR::RangeList(1, SoapySDR::Range(BLADERF_FREQUENCY_MIN, BLADERF_FREQUENCY_MAX));
    return SoapySDR::Device::getFrequencyRange(direction, channel, name);
}

/*******************************************************************
 * Sample Rate API
 ******************************************************************/

void bladeRF_SoapySDR::setSampleRate(const int direction, const size_t channel, const double rate)
{
    
}

double bladeRF_SoapySDR::getSampleRate(const int direction, const size_t channel) const
{
    
}

std::vector<double> bladeRF_SoapySDR::listSampleRates(const int, const size_t) const
{
    std::vector<double> options;
    options.push_back(BLADERF_SAMPLERATE_MIN);
    options.push_back(BLADERF_SAMPLERATE_REC_MAX);
    return options;
}

void bladeRF_SoapySDR::setBandwidth(const int direction, const size_t channel, const double bw)
{
    
}

double bladeRF_SoapySDR::getBandwidth(const int direction, const size_t channel) const
{
    
}

std::vector<double> bladeRF_SoapySDR::listBandwidths(const int, const size_t) const
{
    std::vector<double> options;
    options.push_back(BLADERF_BANDWIDTH_MIN);
    options.push_back(BLADERF_BANDWIDTH_MAX);
    return options;
}
