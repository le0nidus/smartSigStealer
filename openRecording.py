# SoapySDR is the API for the hackrf
import SoapySDR
from SoapySDR import Device, SOAPY_SDR_TX, SOAPY_SDR_CF32
# Using pyfftw instead of numpy to calculate fft faster
from pyfftw import interfaces
from pyfftw.interfaces import numpy_fft as fastnumpyfft
# use numpy for buffers
import numpy as np
import sys
# use keyboard for getting menu choices from the user
import keyboard
# use time for creating delays (remove re-prints)
import time
# use the defaults from variable file
import configFileWrite
import glob
import matplotlib.pyplot as plt
import os


# apply initial settings to HackRF device
def initializeHackRF(fs, center_freq, bw, gain, soapyDirection):
    sdr.setSampleRate(soapyDirection, 0, fs)
    sdr.setBandwidth(soapyDirection, 0, bw)
    sdr.setFrequency(soapyDirection, 0, center_freq)
    sdr.setGain(soapyDirection, 0, gain)


def analyzeFiles():
    if not glob.glob('*.iq'):
        print("No IQ files found")
    else:
        for fname in os.listdir('.'):
            if fname.endswith('.iq'):
                print("Opening " + fname + "...")
                samplesArr = np.fromfile(fname, np.complex64)
                samplesIQ = samplesArr[::2] + 1j * samplesArr[1::2]  # convert to IQIQIQ...
                sampleLen = np.size(samplesIQ)
                print(str(sampleLen) + " samples")
                usrInput = str(input("\nStream the transmission now? [Y/N]: "))
                if usrInput == "N":
                    break
                elif usrInput == "Y":
                    for i in range(int(sampleLen/samplesPerIteration)):
                        samplesToStream = samplesIQ[(i*samplesPerIteration):((i*samplesPerIteration)-1)]
                        status = sdr.writeStream(stream_TX, [samplesToStream], samplesPerIteration)
                        if status.ret != samplesPerIteration:
                            raise Exception('transmit failed %s' % str(status))
    return



# setup a stream (complex floats)
def setStream(sdrDevice, soapyDirection):
    stream = sdrDevice.setupStream(soapyDirection, SOAPY_SDR_CF32)
    sdrDevice.activateStream(stream)  # start streaming
    return stream


# stop the stream and shutdown
def quitStream(sdrDevice, stream):
    sdrDevice.deactivateStream(stream)  # stop streaming
    sdrDevice.closeStream(stream)


if __name__ == '__main__':
    # window()

    # show soapySDR devices available
    results = SoapySDR.Device.enumerate()
    print("Available devices:")
    for result in results: print(result)

    # create device instance
    # args can be user defined or from the enumeration result
    args = dict(driver="hackrf")
    sdr = SoapySDR.Device(args)

    print(sdr.getStreamFormats(SOAPY_SDR_TX, 0))

    bandwidth_TX = configFileWrite.BANDWIDTH
    samp_rate_TX = configFileWrite.SAMPLE_RATE
    centerFreq_TX = configFileWrite.TX_FREQ + (0.1666*1e6)
    samplesPerIteration = configFileWrite.SAMPLES_PER_ITERATION
    gain_TX = configFileWrite.TX_GAIN

    # in keyboard is_pressed it re-prints if the function won't sleep
    cancelRePrintSleepTime = configFileWrite.CANCEL_REPRINT_SLEEP_TIME

    initializeHackRF(samp_rate_TX, centerFreq_TX, bandwidth_TX, gain_TX, SOAPY_SDR_TX)

    # setup a stream
    stream_TX = setStream(sdr, SOAPY_SDR_TX)

    analyzeFiles()

    # shutdown the stream
    quitStream(sdr, stream_TX)
