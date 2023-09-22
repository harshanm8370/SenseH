SenseH esp32 application code
====================

This is a esp32 SenseH application to be used with [Espressif IoT Development Framework](https://github.com/espressif/esp-idf).

Firmware Release Notes
SenseH_FW_Esp32
Release V1.0 – Date: 22/09/2023
Release Name: SenseH_FW_Esp32


Features:  
•	Date/time sync with the mobile application
•	Time monitoring on the console for each test  
•	Lead off detection for ECG 
•	Polling method for ECG and PPG
•	Storage for quick and multi-vitals
•	Data syncing with mobile APK
•	CS pin of ADS now connected to IO14 of ESP32

Existing Issues and Workarounds:  
•	V leads in multi-vitals are not inverted
•	PPG lead off detection 
•	BP test (Sequential acquisition is not properly managed)


Release File Location:	 SenseH_FW_Esp32
Hardware Details	
BOARD	BOARD NO.	MODIFICATIONS	
Device Main Board:	 28	 Fibbing – Done, R123 resistor removed
Prog resistor – Present
ENTC – Present
Reset Ckt Cap - Present 	
		ADS CS pin is routed to IO14 of ESP32	
Device Display Board:	 28		
Device Power Board:	 28		
Device PPG Board	 28	 Removed R99 and R102 resistors for ECG LOD	
Software Details	
Android APK Version:	T-1.0.6	
  
