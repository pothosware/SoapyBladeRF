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

/*!
 * Stream pointer and associated data
 * This structure will be filled by setup stream,
 * and passed to various Soapy SDR stream calls
 * as the opaque pointer SoapySDR::Stream *stream.
 */
struct bladeRF_SoapySDR_stream_data
{
    struct bladerf_stream *stream;
    bladerf_module module;
};

/*!
 * Blade RF streamer callback for completed transfers.
 */
void bladeRF_SoapySDR_stream_cb(
    bladerf *dev,
    struct bladerf_stream *stream,
    bladerf_metadata *meta,
    void *samples,
    size_t num_samples,
    void *user_data)
{
    
}

SoapySDR::Stream *bladeRF_SoapySDR::setupStream(
    const int direction,
    const std::string &format,
    const std::vector<size_t> &channels,
    const SoapySDR::Kwargs &args)
{
    
}

void bladeRF_SoapySDR::closeStream(SoapySDR::Stream *stream)
{
    
}

size_t bladeRF_SoapySDR::getStreamMTU(SoapySDR::Stream *stream) const
{
    
}

int bladeRF_SoapySDR::activateStream(
    SoapySDR::Stream *stream,
    const int flags,
    const long long timeNs,
    const size_t numElems)
{
    //const int ret = bladerf_enable_module(_dev, (bladerf_module)stream, true);
}

int bladeRF_SoapySDR::deactivateStream(
    SoapySDR::Stream *stream,
    const int flags,
    const long long timeNs)
{
    //const int ret = bladerf_enable_module(_dev, (bladerf_module)stream, false);
}

int bladeRF_SoapySDR::readStream(
    SoapySDR::Stream *stream,
    void * const *buffs,
    const size_t numElems,
    int &flags,
    long long &timeNs,
    const long timeoutUs)
{
    
}

int bladeRF_SoapySDR::writeStream(
    SoapySDR::Stream *stream,
    const void * const *buffs,
    const size_t numElems,
    int &flags,
    const long long timeNs,
    const long timeoutUs)
{
    
}
