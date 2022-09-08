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
def initializeHackRF(fs, f_rx, bw, gain):
    sdr.setSampleRate(SOAPY_SDR_TX, 0, fs)
    sdr.setBandwidth(SOAPY_SDR_TX, 0, bw)
    sdr.setFrequency(SOAPY_SDR_TX, 0, f_rx)
    sdr.setGain(SOAPY_SDR_TX, 0, gain)


# setup a stream (complex floats)
def setStream(sdrDevice):
    stream = sdrDevice.setupStream(SOAPY_SDR_TX, SOAPY_SDR_CF32)
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

    bandwidth = configFileWrite.BANDWIDTH
    samp_rate = configFileWrite.SAMPLE_RATE
    tx_freq = configFileWrite.TX_FREQ
    samplesPerIteration = configFileWrite.SAMPLES_PER_ITERATION
    tx_gain = configFileWrite.TX_GAIN

    # in keyboard is_pressed it re-prints if the function won't sleep
    cancelRePrintSleepTime = configFileWrite.CANCEL_REPRINT_SLEEP_TIME

    tx_freq = 315 * 1e6
    tx_gain = 46
    initializeHackRF(samp_rate, tx_freq, bandwidth, tx_gain)

    # setup a stream
    txStream = setStream(sdr)

    if not glob.glob('*.iq'):
        print("No IQ files found")
    else:
        for filename in os.listdir('.'):
            if filename.endswith('.iq'):
                print("Opening " + filename + "...")
                samplesArr = np.fromfile(filename, np.complex64)
                samplesIQ = samplesArr[::2] + 1j * samplesArr[1::2]  # convert to IQIQIQ...
                sampleLen = np.size(samplesIQ)
                print(str(sampleLen) + " samples")
                usrInput = str(input("\nStream the transmission now? [Y/N]: "))
                if usrInput == "N":
                    break
                elif usrInput == "Y":
                    for i in range(int(sampleLen/samplesPerIteration)):
                        samplesToStream = samplesIQ[(i*samplesPerIteration):((i*samplesPerIteration)-1)]
                        status = sdr.writeStream(txStream, [samplesToStream], samplesPerIteration)
                        if status.ret != samplesPerIteration:
                            raise Exception('transmit failed %s' % str(status))

    # shutdown the stream
    quitStream(sdr, txStream)
