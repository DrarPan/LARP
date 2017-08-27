#!/usr/bin/env python

from scipy.io import wavfile
import argparse
import numpy as np
import pygame
import sys
import warnings
import rospy
from sensor_msgs.msg import LaserScan
from math import pi

pitchNum=18
volume_scale=1
trigger_list=[False]*pitchNum;
trigger_count=[0]*pitchNum
trigger_thresh=10
trigger_range=0.2
init_pitch=20

def speedx(snd_array, factor):
    """ Speeds up / slows down a sound, by some factor. """
    indices = np.round(np.arange(0, len(snd_array), factor))
    indices = indices[indices < len(snd_array)].astype(int)
    return snd_array[indices]

def stretch(snd_array, factor, window_size, h):
    """ Stretches/shortens a sound, by some factor. """
    phase = np.zeros(window_size)
    hanning_window = np.hanning(window_size)

    result = np.zeros(int(len(snd_array) / factor + window_size))

    for i in np.arange(0, len(snd_array) - (window_size + h), int(h*factor)):
        # Two potentially overlapping subarrays
        a1 = snd_array[i: i + window_size]
        a2 = snd_array[i + h: i + window_size + h]

        # The spectra of these arrays
        s1 = np.fft.fft(hanning_window * a1)
        s2 = np.fft.fft(hanning_window * a2)

        # Rephase all frequencies
        phase = (phase + np.angle(s2/s1)) % 2*np.pi

        a2_rephased = np.fft.ifft(np.abs(s2)*np.exp(1j*phase))
        i2 = int(i/factor)
        result[i2: i2 + window_size] += hanning_window*a2_rephased.real

    # normalize (16bit)
    result = ((2**(16-4)) * result/result.max())

    return result.astype('int16')

def pitchshift(snd_array, n, window_size=2**13, h=2**11):
    """ Changes the pitch of a sound by ``n`` semitones. """
    factor = 2**(1.0 * n / 12.0)
    stretched = stretch(snd_array, 1.0/factor, window_size, h)
    return speedx(stretched[window_size:], factor)

def createSounds(soundfile,soundrange=25):
    fps, sound = wavfile.read(soundfile)
    pygame.mixer.init(fps, -16, 1, 2048)
    tones = range(-soundrange, soundrange)
    print('Creating sounds...')
    transposed_sounds =[pitchshift(sound,n)*volume_scale for n in tones]
    print('Done')
    return map(pygame.sndarray.make_sound, transposed_sounds)

sounds=createSounds('./bowl.wav')

def playsound_callback(data):
    min_angle=data.angle_min
    max_angle=data.angle_max
    angle_increment=data.angle_increment
    trigger_count=[0]*pitchNum
    for i in range(len(data.ranges)):
        index=int(((min_angle+angle_increment*i)+pi/2)/(pi/pitchNum))
        if index>=0 and index<18:
            if(data.ranges[i]<trigger_range):
                print index, "pitch trigger!"
                trigger_count[index]+=1

    for i in range(len(trigger_count)):
        if trigger_count[i]>trigger_thresh:
            if(trigger_list[i]==False):
                sounds[i+init_pitch].play(fade_ms=100)
                trigger_list[i]=True
        else:
            trigger_list[i]=False

def main():
    pygame.display.init()
    dispimg=pygame.image.load('./guitar.jpg')
    icon=pygame.display.set_icon(dispimg)
    caption=pygame.display.set_caption("LARP")
    screen = pygame.display.set_mode((1,1))
    #print(pygame.image.tostring(dispimg,'RGB'))
    # pygame.mouse.set_visible(0)
    rospy.init_node('laserpiano')
    rospy.Subscriber('/scan_rectified',LaserScan,playsound_callback)
    rospy.spin()

if __name__=="__main__":
    main()


