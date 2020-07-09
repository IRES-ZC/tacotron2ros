#!/home/amer/miniconda3/envs/tacotron2ros/bin/python
#
# TACOTRON2ROS, 2020
# Author: Abdalrahman M. Amer (abdalrahman.m5959@gmail.com)
# Software License Agreement (MIT License)
# Simple Text to Voice talker demo utilize tacotron2 
# that accepts std_msgs/Strings messages published 
# to the 'text2voice' topic and synthesis artificial
# female sound tro speakers.


# ROS staff
import rospy
from std_msgs.msg import String

# Sound play staff
import sounddevice as sd
import time
import os

# Tacotron2 dependencies
import numpy as np
import torch
import tensorflow as tf

from include.hparams import create_hparams
from include.model import Tacotron2
from include.layers import TacotronSTFT, STFT
from include.audio_processing import griffin_lim
from include.train import load_model
from include.text import text_to_sequence
from include.denoiser import Denoiser
import types

# Get folder path    
__location__ = os.path.realpath(
    os.path.join(os.getcwd(), os.path.dirname(__file__)))

def tacotron2audio(data):
    ''' Used to feedforward string to the model and
        return audio tensor and sample rate'''
    text = str(data.data)
    #### Setup hparams
    hparams = create_hparams()
    hparams.sampling_rate = 22050

    #### Load model from checkpoint
    # checkpoint_path = "tacotron2_statedict.pt"
    checkpoint_path = os.path.join(__location__, 'models/tacotron2_statedict.pt')
    model = load_model(hparams)
    model.load_state_dict(torch.load(checkpoint_path)['state_dict'])
    _ = model.cuda().eval().half()

    #### Load WaveGlow for mel2audio synthesis and denoiser
    # waveglow_path = 'waveglow_256channels.pt'
    waveglow_path = os.path.join(__location__, 'models/waveglow_256channels_universal_v5.pt')
    waveglow = torch.load(waveglow_path)['model']
    waveglow.cuda().eval().half()
    for k in waveglow.convinv:
        k.float()
    denoiser = Denoiser(waveglow)

    #### Prepare text input
    #text = "Let us make some Intelligent Robots!"
    sequence = np.array(text_to_sequence(text, ['english_cleaners']))[None, :]
    sequence = torch.autograd.Variable(
        torch.from_numpy(sequence)).cuda().long()

    #### Decode text input 
    mel_outputs, mel_outputs_postnet, _, alignments = model.inference(sequence)

    with torch.no_grad():
        audio = waveglow.infer(mel_outputs_postnet, sigma=0.666)

    audio_data = audio[0].data.cpu().numpy()
    sample_rate = hparams.sampling_rate
    
    return audio_data , sample_rate


def audio2speaker(audio_data,sample_rate,freq_hz = 440.0,duration_s = 10.0,atten = 1):
    '''Used to output signal to the psychical speaker through soundplay lib'''
    # Samples per second
    sps = sample_rate

    # Frequency / pitch
    #freq_hz = 440.0

    # Duration
    #duration_s = 10.0

    # Attenuation so the sound is reasonable
    #atten = 1

    # NumpPy magic to calculate the waveform
    each_sample_number = np.arange(duration_s * sps)
    waveform = np.float32(audio_data)
    waveform_quiet = waveform * atten

    # Play the waveform out the speakers
    sd.play(waveform_quiet, sps)
    time.sleep(duration_s)
    sd.stop()


def callback(data):
    ''' Used to as ROS callback to get the string data from topic'''
    # pass the text to tacotron2 model and get the audio data
    audio_data , sample_rate = tacotron2audio(data)

    # pub the text
    rospy.loginfo(rospy.get_caller_id() + ' I received %s', data.data)

    # output the audio to speaker
    audio2speaker(audio_data,sample_rate)

def talker():
    '''ROS node initializer and text topic subscriber'''
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('talker', anonymous=True)

    rospy.Subscriber('text2voice', String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
