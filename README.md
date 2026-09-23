# Soapy SDR plugin for Blade RF

## Dependencies

* SoapySDR - https://github.com/pothosware/SoapySDR/wiki
* LibBladeRF - http://www.github.com/nuand/bladeRF

## Documentation

* https://github.com/pothosware/SoapyBladeRF/wiki

## Licensing information

*LGPLv2.1: https://www.gnu.org/licenses/old-licenses/lgpl-2.1.txt

## Adding OVERSAMPLE Support

1. Install libbladeRF [link](https://github.com/Nuand/bladeRF)
2. Install Soapy [link](https://github.com/pothosware/SoapySDR/wiki/BuildGuide#unix-instructions) at tag 0.8.0
3. Install SoapyBladeRF plugin [link](https://github.com/pothosware/SoapyBladeRF/wiki) at master
4. Build example and run

    ```bash
    gcc -std=c99 ../example.c -L/usr/local/lib -lSoapySDR
    ./a.out
    ```

5. gnuradio-companion 3.10.9.2 will prioritize the local v0.8.0 Soapy lib after install
