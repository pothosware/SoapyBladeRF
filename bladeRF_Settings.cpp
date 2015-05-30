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
