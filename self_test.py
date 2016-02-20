import SoapySDR
from SoapySDR import * #SOAPY_SDR_* constants
import numpy as np

if __name__ == "__main__":
    bladerf = SoapySDR.Device(dict(driver="bladerf"))
    print bladerf

    for i in range(5):
        print("  Make rx stream #%d"%i)
        rxStream = bladerf.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32, [0])
        for j in range(5):
            lastTimeNs = 0
            numSampsTotal = 10000
            print("    Activate, get %d samples, Deactivate #%d"%(numSampsTotal, j))
            bladerf.activateStream(rxStream, SOAPY_SDR_END_BURST, 0, numSampsTotal)
            buff = np.array([0]*1024, np.complex64)
            while numSampsTotal != 0:
                sr = bladerf.readStream(rxStream, [buff], buff.size)
                assert(sr.ret > 0)
                numSampsTotal -= sr.ret
                if not (sr.timeNs > lastTimeNs):
                    print("Fail %s, %d"%(sr, numSampsTotal))
                assert(sr.timeNs > lastTimeNs)
                lastTimeNs = sr.timeNs
            bladerf.deactivateStream(rxStream)
        bladerf.closeStream(rxStream)

    for i in range(5):
        print("  Make tx stream #%d"%i)
        txStream = bladerf.setupStream(SOAPY_SDR_TX, SOAPY_SDR_CF32, [0])
        for j in range(5):
            numSampsTotal = 10000
            print("    Activate, send %d samples, Deactivate #%d"%(numSampsTotal, j))
            bladerf.activateStream(txStream)
            buff = np.array([0]*1024, np.complex64)
            while numSampsTotal != 0:
                size = min(buff.size, numSampsTotal)
                flags = 0
                #if size == numSampsTotal: flags |= SOAPY_SDR_END_BURST
                sr = bladerf.writeStream(txStream, [buff], size, flags)
                if not (sr.ret > 0): print("Fail %s, %d"%(sr, numSampsTotal))
                assert(sr.ret > 0)
                numSampsTotal -= sr.ret
            bladerf.deactivateStream(txStream)
        bladerf.closeStream(txStream)
