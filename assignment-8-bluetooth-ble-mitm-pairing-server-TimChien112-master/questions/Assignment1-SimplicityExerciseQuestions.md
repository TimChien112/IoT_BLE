Please include your answers to the questions below with your submission, entering into the space below each question
See [Mastering Markdown](https://guides.github.com/features/mastering-markdown/) for github markdown formatting if desired.

**1. How much current does a single LED draw when the output drive is set to "Strong" with the original code?**

The average current value in the period of LED1 turning on is 4.9 mA
The average current value in the period of LED1 turning off is 4.4 mA
Therefore, the additional current value to turn on LED1 is 0.5 mA


**2. After commenting out the standard output drive and uncommenting "Weak" drive, how much current does a single LED draw?**

The value of current is "same" as the value in strong mode.
The average current value in the period of LED1 turning on is 4.9 mA
The average current value in the period of LED1 turning off is 4.4 mA
Therefore, the additional current value to turn on LED1 is 0.5 mA 

**3. Is there a meaningful difference in current between the answers for question 1 and 2? Please explain your answer, 
referencing the [Mainboard Schematic](https://www.silabs.com/documents/public/schematic-files/WSTK-Main-BRD4001A-A01-schematic.pdf) and [AEM Accuracy](https://www.silabs.com/documents/login/user-guides/ug279-brd4104a-user-guide.pdf) section of the user's guide where appropriate.**

There is no meaningful difference in current value between strong mode and weak mode in this case.
This parameter is used to control the maximum current value from certain pin.
Weak mode is to weak 1mA for GPIO, and strong mode is to strong 10mA for GPIO.
However, in this assignment, the current value is less than 1mA.
Therefore, current value does not change after switching to weak mode.

**4. Using the Energy Profiler with "weak" drive LEDs, what is the average current and energy measured with only LED1 turning on in the main loop?**

The average current value in the period of LED1 turning on is 4.92 mA, and power consumption is 16.25 mW.

**5. Using the Energy Profiler with "weak" drive LEDs, what is the average current and energy measured with both LED1 and LED0 turning on in the main loop?**

The average current value in the period of LED1 and LED0 turning on is 5.4 mA, and power consumption is 17.8 mW.