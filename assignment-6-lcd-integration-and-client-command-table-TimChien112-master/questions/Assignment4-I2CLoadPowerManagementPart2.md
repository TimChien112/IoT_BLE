Please include your answers to the questions below with your submission, entering into the space below each question
See [Mastering Markdown](https://guides.github.com/features/mastering-markdown/) for github markdown formatting if desired.

*Be sure to take measurements in the "Default" configuration of the profiler to ensure your logging logic is not impacting current/time measurements.*

*Please include screenshots of the profiler window detailing each current measurement captured.  See [Shared document](https://docs.google.com/document/d/1Ro9G2Nsr_ZXDhBYJ6YyF9CPivb--6UjhHRmVhDGySag/edit?usp=sharing) for instructions.* 

1. What is the average current per period?
   Answer: 106.25 uA  
   [Avg_current_per_period] https://drive.google.com/file/d/1FZ9AAh0f_4M7P4dTA6Wm1RJOMw5QBikC/view?usp=sharing

2. What is the average current when the Si7021 is Load Power Management OFF?
   Answer:	1.72 uA
   [Avg_current_LPM_Off] https://drive.google.com/file/d/1EFzho4CJGYBKK0_8Fro_aqpl6EL8woTo/view?usp=sharing

3. What is the average current when the Si7021 is Load Power Management ON?
   Answer:	3.8 mA
   [Avg_current_LPM_On] https://drive.google.com/file/d/1IH-uKTDliIkhijvhFLWp1jZHtnaPr-pl/view?usp=sharing 

4. How long is the Si7021 Load Power Management ON for 1 temperature reading?
   Answer:	110.5 ms
   <br>Screenshot:  
   [duration_lpm_on] https://drive.google.com/file/d/1XUizSvVL-gFLjNkHY1kR6xObov1A87pz/view?usp=sharing
   [duration_lpm_on_stamp] https://drive.google.com/file/d/1COyyHnXuYh5YN2B3kAovGbv53Bas354G/view?usp=sharing

5. What is the total operating time of your design for assignment 4 in hours assuming a 1000mAh supply?
   Answer:
	1000(mAh)/106.25(uA)= 9411 hr
	
6. How has the power consumption performance of your design changed since the previous assignment?
   Answer:
	Pervious assignment average power consumption	:	 119.86 uA
	This assignment average power consumption 	 	:	 106.25 uA
	=> power consumption decreases in 13 uAh
	
7. Describe how you have tested your code to ensure you are sleeping in EM1 mode during I2C transfers.
   1. Enter the debug mode, and set a breakpoint on WAIT_TEMP_READY function case. 
   2. Run the code to this breakpoint. 
   3. Click Resume and clik suspend right after resume clicking.
   4. The code will be stopped at Wait_us.
   5. At the window, we can see the Energy mode we are using right now.
   [Test_in_EM1] https://drive.google.com/file/d/1wEAPjsLBeYAXiRVK-T7HWEKbF5uiEoGS/view?usp=sharing
   
