# mnt_surveillance

Implement a video surveillance and analysis system that harnesses the power of distributed computing, enabling multiple services to run seamlessly across various machines. The design will leverage ROS2 (Robot Operating System 2) for its robust communication framework, ensuring efficient data flow and service integration.

## Video Source(Python / CPP)

- ~~Web or USB or built in Camera~~
- File Player (for pre-recorded (**resulted from the recorder you implement**) raw data files)
    - __did not manage to create a file player in the specified time frame but i have the code infrastructure required to do it relatively quickly__

### Specifications

#### Video Acquisition

- ~~The system should be equipped to access a live feed from a Web or USB camera or retrieve frames from a pre-recorded raw data file.~~
    - __did not manage to do file reading codec in specified time__

#### Frame Processing

- ~~Each video frame should conform to a specified resolution of your choice.~~
- ~~Frames can be assumed to be grayscale.~~
- ~~The pixel depth of each frame should be adjusted to 10 bits. This might necessitate upscaling from 8 bits or downscaling from 16 bits, depending on the source.~~

#### Streaming

- ~~Processed video frames should be streamed to a predefined ROS topic: /video/raw_data.~~
- ~~The streamed data should exclusively consist of raw pixel values.~~
- ~~Each message transmitted to the topic should not exceed a size of 100 bytes.~~

#### Frame Rate

- ~~For both video sources, the system should maintain a consistent frame rate of 30 frames per second (FPS).~~

## Recorder (CPP)

- ~~The recorder process will be configured to a specific video stream and file path. It should store the video in a binary file. Store only the raw data and do it in a packed manner, meaning only the 10 active bits should be stored without any padding.~~

## Analyzer (CPP)

- ~~The analyzer framework will listen to the video stream and once a frame is ready it will apply various algorithms on it.~~
- The applied algorithms should be configurable from the outside and are provided as 3rd party either source or precompiled libraries. Please elaborate on how a new algorithm will be introduced to the system.
    - __did not manage to complete this in the time frame. With the codec object i have created this should not be an issue. just adding another loop and using the img_decoder class__
    - __The applied algorithms are extremely simple since we are working with a 9x11 pixel image and it is pretty hard to distinguish any details in such a matrix. the algorithms applied are overexposure and underexposure algorithms__
- ~~A configuration yaml file is to be used to configure which algorithms will be applied and in what order  ~~
- ~~For example one of the algorithms can be over-exposure detection to protect the system from burglars who will try to apply direct light into the camera to avoid detection. An alarm will be sounded if more than 20% of the image is fully saturated.~~

## Alarm (Python / CPP)

- ~~A listener node that will expose a ROS2 service, once an alarm is triggered a log will be displayed on the screen.~~
    - __The log which is triggered is rqt_console. it is called whenever the alarm is set off__

## Display

- This node will open a window and show real live data upon request.
    - __I have created an rqt_image_view launch file but it does not work for some reason. Since this was done on friday evening I've decided to leave it as it is for now and submit the work for review__

<br>


## Conclution
During the execution of the task my main focus was to keep a maintainable, testable code and a readable code structure. I did so by conducting an object oriented programming approach which allowed my to segment and encapsulate various espects of the code. This allowed me to conduct testing and debugging to a certain part of the code without compromising other parts of the code. 

Some of the objects present in the code were used for testing the encoding and decoding of the images from the regulat sensor_msgs::msg::Image msg type to my propryetary 10bit encoding and decoding codec.

One last note:
During the project I've had some knowledge gaps which I had to fill. I have done so by doing courses in Udemy, searching in forums and using ChatGPT as a C++ coding tutor. I have found that, since GPT is a language model, it does a good job explaining coding logic and programming methodologies.

I hope that the result is good enough. would love to talk and explain how to read the code. I know the documentation is not great but I will try to compensate in out interview (should there be one)


Have a great Weekend.

Lior S.
