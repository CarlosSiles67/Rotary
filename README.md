This is  a great rotary library from Ben Buxton and modified by Phill Fisk, who included the push button feature. Original library works but it doesn’t follow the sequence  00>10>11>01>01, hence I modified it so that it does.

A rotary encoder is a position sensor that is used to determine angular position of the rotating shaft. Usually, a rotary encoder has 5 pins: 5V, GND, push button, DT and CLK that are used for Bit A and Bit B. The encoder uses Gray Code sequence where two successive values differ in only one bit (binary digit).

The basic way of decoding these is to watch for which bits change. For example, a change from “00” to “10” indicates one direction, whereas a change from “00” to “01” indicates the other direction. The library uses a Finite State Machine (FSM) algorithm that follows a Gray Code sequence. The biggest advantage of using a state machine over other algorithms is that this has inherent debounce built in. Other algorithms emit spurious output with switch bounce, but this one will simply flip between sub-states until the bounce settles, then continue along the state machine.
Please read the description of the original library in the following link:

http://www.buxtronix.net/2011/10/rotary-encoders-done-properly.html

Phill Fisk incorporated push button features:

https://bitbucket.org/Dershum/rotary_button/src/master/

I’ve made a brief description of the FSM algorithm here:

https://carlossilesblog.wordpress.com/2019/07/27/rotary-encoder/

