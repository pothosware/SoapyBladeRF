/*
 * This file is part of the bladeRF project:
 *   http://www.github.com/nuand/bladeRF
 *
 * Copyright (C) 2015-2018 Josh Blum
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

#include <SoapySDR/Registry.hpp>
#include "bladeRF_SoapySDR.hpp"
#include <cstdio>
#include <sstream>
#include <cstdlib>
#include <cstring>

static SoapySDR::Kwargs devinfo_to_kwargs(const bladerf_devinfo &info)
{
    SoapySDR::Kwargs args;

    args["backend"] = bladerf_backend_str(info.backend);
    args["serial"] = std::string(info.serial);

    char buff[100];
    int r = std::sprintf(buff, "0x%02X:0x%02X", int(info.usb_bus), int(info.usb_addr));
    if (r > 0) args["device"] = std::string(buff, r);

    r = std::sprintf(buff, "%u", info.instance);
    if (r > 0) args["instance"] = std::string(buff, r);

    std::string shortSerial(std::string(info.serial));
    //serial can take the value "ANY" on permissions errors
    if (shortSerial.size() > 20) shortSerial.replace(8, 16, "..");
    args["label"] = "BladeRF #" + args["instance"] + " [" + shortSerial + "]";

    return args;
}

static bladerf_devinfo kwargs_to_devinfo(const SoapySDR::Kwargs &args)
{
    std::stringstream ss;

    if (args.count("backend") != 0)
    {
        ss << args.at("backend") << ":";
    }
    else ss << "*:";

    if (args.count("device") != 0)
    {
        ss << "device=" << args.at("device") << " ";
    }

    if (args.count("instance") != 0)
    {
        ss << "instance=" << args.at("instance") << " ";
    }

    if (args.count("serial") != 0)
    {
        ss << "serial=" << args.at("serial") << " ";
    }

    bladerf_devinfo info;
    bladerf_init_devinfo(&info);
    bladerf_get_devinfo_from_str(ss.str().c_str(), &info);
    return info;
}

static std::vector<SoapySDR::Kwargs> find_bladeRF(const SoapySDR::Kwargs &matchArgs)
{
    const bladerf_devinfo matchinfo = kwargs_to_devinfo(matchArgs);

    std::vector<SoapySDR::Kwargs> results;
    bladerf_devinfo *infos = NULL;
    int ret = 0;
    ret = bladerf_get_device_list(&infos);

    for (int i = 0; i < ret; i++)
    {
        if (bladerf_devinfo_matches(infos+i, &matchinfo))
        {
            results.push_back(devinfo_to_kwargs(infos[i]));
        }
    }

    bladerf_free_device_list(infos);
    return results;
}

static SoapySDR::Device *make_bladeRF(const SoapySDR::Kwargs &args)
{
    SoapySDR::Device *bladerf = new bladeRF_SoapySDR(kwargs_to_devinfo(args));

    //apply applicable settings found in args
    for (const auto &info : bladerf->getSettingInfo())
    {
        if (args.count(info.key) == 0) continue;
        bladerf->writeSetting(info.key, args.at(info.key));
    }

    return bladerf;
}

static SoapySDR::Registry register__bladeRF("bladerf", &find_bladeRF, &make_bladeRF, SOAPY_SDR_ABI_VERSION);
