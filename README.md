# tacotron2ros 

![cover](/img/cover.png)

## Description

ROS Package to Generate very natural sounding speech from include.text (text-to-speech, TTS).
This package utilizes the tacotron2 deep learning model from the Google AI research lab DeepMind,read more [here](https://ai.googleblog.com/2017/12/tacotron-2-generating-human-like-speech.html).
You can used to give your Robot a human like voice and its completely offline.

[Demo](https://youtu.be/2grU254aAHI)


## Installation 

Assuming you have Ubuntu 18 and ROS1 Melodic installed with a catkin_ws configured.

We need to create isolated environment to install and run the package dependencies.

This will allow you to install multiple python modules eg. multiple tensorFlow/torch versions to use it with with your ROS distro.

### You have two options:

### If you use *linux-x86_64* based systems
Normal laptop or PC then you can use miniconda.

**1. Install Dependencies**
Set up the Conda Environment
First, install the [miniconda](https://docs.conda.io/en/latest/miniconda.html#linux-installers) and create a new Python 3.6 environment:


```sh

$ sudo apt-get install libportaudio2

$ cd ~/catkin_ws/src

$ git clone https://github.com/IRES-ZC/tacotron2ros

$ catkin build

$ source ~/catkin_ws/devel/setup.bash 

$ cd ~/catkin_ws/src/tacotron2ros

$ conda env create -f environment.yml

OR

$ conda create -n tacotron2ros environment --file req.txt

```

**2. Configure Dependencies**

Now we need the ROS node ```tacotron2ros.py``` to use the python interpreter from our virtual environment we created.

First, activate your env 

```sh

$ conda activate tacotron2ros

$ whereis python

```
you will find multiple version in your system. 

> /home/amer/miniconda3/envs/tacotron2ros/bin/python3.6

Next, change the hashbang (or shebang) line which indicates which interpreter should process the in ```tacotron2ros.py```.

> ```#! /usr/bin/env python```  to the one you used in the tacotron2ros environment e.g ```/home/amer/miniconda3/envs/tacotron2ros/bin/python3.6


### If you use *linux-aarch64* based systems
Nvidia Jetson Kits and Raspberry PIs

You will need to use virtual environment or [miniforge](https://github.com/conda-forge/miniforge)
scince anaconda don't have linux-aarch64 yet see this [issue](https://github.com/conda/conda/issues/8297).

**1. Install Dependencies**

Set up the Virtual Environment

First, install the virtualenv package and create a new Python 3.6 virtual environment:

```sh

$ sudo apt-get install libportaudio2

$ sudo apt-get install virtualenv

$ cd ~/catkin_ws/src

$ git clone https://github.com/IRES-ZC/tacotron2ros

$ catkin build

$ source ~/catkin_ws/devel/setup.bash 

$ cd ~/catkin_ws/src/tacotron2ros

$ python3 -m virtualenv -p python3.6 tacotron2ros


Next, activate the virtual environment:

$ source tacotron2ros/bin/activate

$ pip3 install -r requirements.txt

Deactivate the Virtual Environment

$ deactivate

```

**2. Configure Dependencies**

Now we need the ROS node ```tacotron2ros.py``` to use the python interpreter from our virtual environment we created.

First, activate your env 

```sh

$ source tacotron2ros/bin/activate

$ whereis python

```
You will find multiple versions in your system. 

Next, change the hashbang (or shebang) line which indicates which interpreter should process the in ```tacotron2ros.py```.

> ```#! /usr/bin/env python```  to the one you used in the tacotron2ros environment e.g ```#! tacotron2ros/bin/python3.6


## Get the Pertained Models

1. Download Nvidia published [Tacotron 2](https://drive.google.com/file/d/1c5ZTuT7J08wLUoVZ2KkUs_VdZuJ86ZqA/view) model
2. Download Nvidia published [WaveGlow](https://drive.google.com/file/d/1rpK8CzAAirq9sWZhe9nlfvxMF1dRgFbF/view) model
3. add these models to ```~/catkin_ws/src/tacotron2ros/src/models```

## Run

```sh
$ cd ~/catkin_ws/src

$ source ~/catkin_ws/devel/setup.bash 

$ roscore

$ rosrun tacotron2ros tacotron2ros.py 

In another terminal publish the text you want to synthesis

$ rostopic pub /text2voice std_msgs/String "data: 'Hello there!, Nice day'"

you should here female voice with the same text from your speaker.

```
If you want to close and release resources after usage:

```sh
Kill ROS 
$ rosnode kill -a & killall -9 rosmaster  

Kill GPU Processes 
$ sudo fuser -v /dev/nvidia*
$ kill -9 <<the-python-PID>>
```

## Notes

1. No need to activate any conda or virtualenv during running ROS the pkg.
2. Make sure you locate the hashbang of your python interpreter properly.
3. Added the the pretend models
4. If you want a custom model/language/voice refer to acknowledgements section. 
5. This implementation uses Nvidia Cuda to accelerate the inference and it's performance depends on your hardware.
6. To test tactron2 without ROS use the ```inference.ipynb``` notebook don't forget to activate your env and select the proper kernel.

## Acknowledgements

This work is based on Nvidia implementation of [Natural TTS Synthesis By Conditioning Wavenet On Mel Spectrogram Predictions](https://arxiv.org/pdf/1712.05884.pdf) founded [here](https://github.com/NVIDIA/tacotron2) and built as a part of Nour social robot project founded [here]()
