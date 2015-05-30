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
#include <libbladeRF.h>

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

private:
    bladerf *_dev;
};
