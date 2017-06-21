I used the framework given by udacity for this project and took some code from the lessons for this one. (Had to convert between Python and C++).

I implemeted the PID and twiddle to get the car to stay on track. The PID is roughly just a standard implementation and was quite fast to get to work - twiddle on the other hand was another thing - I don't believe the implementation is the fastest, but it is (should be) correct. I used it to finetune speed and steering. If you want to use twiddle please note that you have to set the twiddle-parameter in main.cpp to 'true'.

I used PID control not only for the steering value but also for the speed value, which seems to work really well. It took a long time to get init-values manually and then use twiddle afterwards to get even better results.

I commented the code, but if there are still questions just write me back in the review so I can correct the problems. Some write-lines (std::cout) are outcommented, which can be put back into the code to see different information when run. (I would not use all lines at once, because it could be a bit confusing).

P.S: I used the lowest resolution and smallest window for the simulator, if that is important...

I hope everything works as expected.

Thanks for the review and have a nice week. :)

Greetings Frank


