# TSL2561-with-Interrupts

A basic Arduino script (with no library dependencies) to read and configure the TSL2561 light sensor. I created this for a project that need to generate an interrupt alert when a warning light above the sensor came on.

Most of this is based on the Sparkfun & Adafuit libraries, but I still have not found a really good explanation of the threshold interrupt settings. I generally don't bother with conversion from Lux values, and just manually tweak the two threshold values until I get the behavior I'm after.
