So far...

I commented out unwanted code (full deletes later) in Arducopter/Copter.cpp
I made notes in Tools/scripts/build_options.py of what to disable; the file uses a binary system where...
    0 Enable
    1 Disable

Need to have:
    I2C connection with the Raspberry pi for data transmit
    Possible: connection to a separate app to enable editing to data and analyzing data

TTD for CtrlPos library
    In AP_Logger library, Define a MSG for CtrlPos library like is done for OptFlow library. Then you can use the MSG type for CtrlPos
    To test, you only need to set the Optical flow type to 0 and then set the ctrlpos type to PX4Flow.  the library should take care of the rest
