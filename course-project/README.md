
course-project-TimChien112
course-project-TimChien112 created by GitHub Classroom

                        Mesh based Smart Home (Home Automation)
This project is developed as final project for the course ECEN 5823.

 Authors: 	santhosh Thummanapalli 
			Tim chien

 This Project aims at developing a secure mesh based smart home. This project tries to implement a low power smart fire alert system, smart lighting control system and app for updating these values using Bluetooth mesh.

 Board used: silicon labs Blue gecko starter kit - BRD4001A and Bluetooth Mesh App

 This project develops	1. A low power Node.
							Low power node will send the state changing request to friend node.
							Low power node will receive the changing request (presense state)from friend node, and enable smoke sensor and light sensor.
						2. Fiend node. 
							with IR beam (presense detecter) sensor and LED (bulb).
							Friend node will receive the state changing request (brightness) from LPN, and change the local state. According to the current local state, Friend will change the brightness of the bulb.
							Friend node will sent the presense state to Low power node.
							

 Sensors used 	1. MAX3010 Particle sensor for smoke detection 
				2. APDS 9301 Ambient light sensor for light intensity detection 
				3. 5mm IR break out board.
			 
4-20 process update:
	Complete the friendship between LPN and Friend.
	Can publish the request from LPN to friend for button state changing.
	Enable the GPIO interrupt for IR Beam sensor interrupt.
	Start working on the I2C communication for light ambient sensor.
	
4-29 final update:

	Complete mesh communication with Generic On/Off model and Lightness model, and show it on Friend node LCD
		Receive smoke alert from LPN with Generic On/Off model, and use it to change window state.
		Receive lightness level value from LPN with lightness model.
		Sent room state to LPN with Generic On/Off model to turn sensors on and off.
		
	Use two IR beam sensors to count the number of people in the room.
		Use people count to turn LED on and off.
		
	Use temperature sensor to control window state.
		if temperature value higher than 30 degree Celsius, open the window
		
	Implement the persistent data storage and loading functionalities
		When the room state, window state, and value of lightness change, those data will be stored to flash memory.
		If Friend node is turned off or shut down accidentally, those persistent data can be recoveried from flash memory in initialization process.

Group report Link: https://drive.google.com/file/d/1kbifmCT0BC0zGU0sI-ALf91gBCcpUDaV/view?usp=sharing
Individual report Link: https://drive.google.com/file/d/1lGOTmayz0zMeHHAO1PTn6TQOiRR2MahR/view?usp=sharing
	
	