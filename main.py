# SoapySDR is the API for the hackrf
import SoapySDR
from SoapySDR import Device, SOAPY_SDR_RX, SOAPY_SDR_CF32
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
import configfile


# apply initial settings to HackRF device
def initializeHackRF(fs, f_rx, bw, gain):
    sdr.setSampleRate(SOAPY_SDR_RX, 0, fs)
    sdr.setBandwidth(SOAPY_SDR_RX, 0, bw)
    sdr.setFrequency(SOAPY_SDR_RX, 0, f_rx)
    sdr.setGain(SOAPY_SDR_RX, 0, gain)


# setup a stream (complex floats)
def setStream(sdrDevice):
    stream = sdrDevice.setupStream(SOAPY_SDR_RX, SOAPY_SDR_CF32)
    print(sdr.getStreamMTU(stream))
    sdrDevice.activateStream(stream)  # start streaming
    return stream


# stop the stream and shutdown
def quitStream(sdrDevice, stream):
    sdrDevice.deactivateStream(stream)  # stop streaming
    sdrDevice.closeStream(stream)


# Keyboard choice (menu choices)
def kbUsrChoice(mySDR, myRXFreq, myRXSampleRate, rnBool, freqVec, samplesPerIteration, silenceTime, peakThresholdVal):
    paramsChanged = False
    if keyboard.is_pressed("1"):
        myRXSampleRate = int(float(input("\nEnter desired sample rate (in MHz): ")) * 1e6)
        mySDR.setSampleRate(SOAPY_SDR_RX, 0, myRXSampleRate)
        freqVec = fastnumpyfft.fftshift(fastnumpyfft.fftfreq(samplesPerIteration, d=1 / myRXSampleRate))
        paramsChanged = True
    elif keyboard.is_pressed("2"):
        myRXFreq = int(float(input("\nEnter desired frequency (in MHz): ")) * 1e6)
        mySDR.setFrequency(SOAPY_SDR_RX, 0, myRXFreq)
        paramsChanged = True
    elif keyboard.is_pressed("3"):
        silenceTime = int(float(input("\nEnter desired wait time after peak (in seconds): ")))
        paramsChanged = True
    elif keyboard.is_pressed("4"):
        peakThresholdVal = int(float(input("\nEnter desired peak threshold value (1-2047): ")))
        paramsChanged = True
    elif keyboard.is_pressed("9"):
        print(printMenu.__doc__)
        time.sleep(cancelRePrintSleepTime)
    elif keyboard.is_pressed("0"):
        print("\nYou chose to quit, ending loop")
        rnBool = False
    return myRXFreq, myRXSampleRate, mySDR, rnBool, freqVec, silenceTime, peakThresholdVal, paramsChanged


# Get samples from sdr, but in a loop (read small number of samples every time)
def getSamples(device, stream, samplesPerScan, numOfRequestedSamples):
    samples = np.zeros(numOfRequestedSamples, dtype=np.complex64)
    iterations = int(numOfRequestedSamples / samplesPerScan)
    for j in range(iterations):
        sr = device.readStream(stream, [samples[((j-1)*samplesPerScan):]], samplesPerScan)
    # normalize the sample values
    # sr = device.readStream(stream, [samples], numOfRequestedSamples)
    return samples


def mainWhileLoop(numSamplesPerDFT, numSamplesPerSingleRead, mySDR, sampleRate, rx_freq, rxStream,
                  runBool, freqVec, peakThreshold, timeAfterLastPeak):
    # receive samples
    recordedSamples = np.zeros(numSamplesPerDFT, dtype=np.complex64)
    recordFlag = parametersChangedBool = False
    numOfRecordings = 0
    while runBool:

        # get the samples
        samples = getSamples(mySDR, rxStream, numSamplesPerSingleRead, numSamplesPerDFT)
        # dft on samples to find the freq of the detected peak
        dft = fastnumpyfft.fftshift(fastnumpyfft.fft(samples, numSamplesPerDFT))

        # detect peak and its' frequency
        peakDetectedBool = (np.argmax(np.abs(dft)) > peakThreshold) and\
                           ((freqVec[np.argmax(np.abs(dft))] + rx_freq) != rx_freq)
        if peakDetectedBool:
            time_lastPeak = time_final = time.time()  # get current time of peak + final peak happened now
            if not recordFlag:  # start recording the signal if it's the first time we detect it
                print("Started recording")
                time_initiatedRecording = time.time()
                recordFlag = True

            # declare everytime we encounter a peak
            print("Maximum received in: " + str((freqVec[np.argmax(np.abs(dft))] + rx_freq) / 1e6) + " MHz")

            # save the signal
            recordedSamples = np.append(recordedSamples, samples[:])

        # if there is no peak but recording is still running
        elif ((not peakDetectedBool) and recordFlag):
            recordedSamples = np.append(recordedSamples, samples[:])  # we still record (maybe OOK signal...)
            time_final = time.time()
            if (time_final - time_lastPeak) > timeAfterLastPeak:  # if we encounter X seconds of silence since last peak
                # stop recording to variable and save to file
                recordFlag = False
                numOfRecordings += 1  # Recording number
                recordingTime = time_final - time_initiatedRecording # How much time the recording took
                print("Finished recording, recorded " + str(round(recordingTime, 4)) + " seconds")
                recordedSamples.tofile('recording' + str(numOfRecordings) + '.iq')  # Save to file
                recordedSamples = np.zeros(numSamplesPerDFT, dtype=np.complex64)  # reset the recording variable

        rx_freq, sampleRate, mySDR0, runBool, freqVec, timeAfterLastPeak, peakThreshold, parametersChangedBool = \
            kbUsrChoice(mySDR, rx_freq, sampleRate, runBool, freqVec, samplesPerIteration, timeAfterLastPeak,
                        peakThreshold)

        # If user changed one of the parameters
        if parametersChangedBool:
            recordedSamples = np.zeros(numSamplesPerDFT, dtype=np.complex64)  # reset the recording variable
            if recordFlag:
                # if it was in the middle of a recording
                print("Recording stopped because SDR parameters changed")
                recordFlag = False

    return


# print the main menu for our spectrum analyzer
def printMenu():
    '''Choose one from the options:
    1 - Change sample rate
    2 - Change RX frequency
    3 - Change silence time
    4 - Change peak threshold value
    9 - Print menu again
    0 - Quit'''


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

    a = 0.0
    print(sdr.getStreamFormats(SOAPY_SDR_RX, 0))

    bandwidth = configfile.BANDWIDTH
    samp_rate = configfile.SAMPLE_RATE
    rx_freq = configfile.RX_FREQ
    samplesPerIteration = configfile.SAMPLES_PER_ITERATION
    samplesPerRead = configfile.SAMPLES_PER_READ
    RX_gain = configfile.RX_GAIN

    runBool = configfile.BOOL_RUN

    peakThreshold = configfile.PEAK_THRESHOLD
    timeAfterLastPeak = configfile.SILENCE_AFTER_LAST_PEAK

    #in keyboard is_pressed it re-prints if the function won't sleep
    cancelRePrintSleepTime = configfile.CANCEL_REPRINT_SLEEP_TIME

    initializeHackRF(samp_rate, rx_freq, bandwidth, RX_gain)

    # setup a stream
    rxStream = setStream(sdr)

    # print menu
    print(printMenu.__doc__)

    freqs = fastnumpyfft.fftshift(fastnumpyfft.fftfreq(samplesPerIteration, d=1 / samp_rate))

    mainWhileLoop(samplesPerIteration, samplesPerRead, sdr, samp_rate, rx_freq, rxStream, runBool, freqs,
                  peakThreshold, timeAfterLastPeak)


    # shutdown the stream
    quitStream(sdr, rxStream)