![image](images/microchip.jpg) 
## Sensorless FOC of PMSM using ATPLL estimator integrated with Power Factor Correction: MCHV-230VAC-1.5kW Development Board and dsPIC33CK64MC105 Motor Control DIM

## 1. INTRODUCTION
This document describes the setup requirements to drive a PMSM Motor with Power Factor Correction (PFC) on the hardware platform [EV78U65A](https://www.microchip.com/en-us/development-tool/ev78u65a) "MCHV-230VAC-1.5kW Development Board"  and [EV03J37A](https://www.microchip.com/en-us/development-tool/ev03j37a) "
dsPIC33CK64MC105 Motor Control Dual In-line Module (DIM)".

The firmware includes Sensorless Field Oriented Control (FOC) with ATPLL (Angle Tracking PLL) Estimator, integrated with Power Factor Correction. It also includes field weakening algorithm to support extended speed operation. 
</br>

## 2. SUGGESTED DEMONSTRATION REQUIREMENTS

### 2.1 Motor Control Application Firmware Required for the Demonstration

To clone or download this application firmware on GitHub, 
- Navigate to the [main page of this repository](https://github.com/microchip-pic-avr-solutions/mchv230vac1.5kw-33ck64mc105-pmsm-foc-atpll-pfc) and 
- On the tab **<> Code**, above the list of files in the right-hand corner, click Code, then from the menu, click **Download ZIP** or copy the repository URL to **clone.**
> **Note:** </br>
>In this document, hereinafter this firmware package is referred as **firmware.**
### 2.2 Software Tools Used for Testing the firmware

- MPLAB® X IDE **v6.20** 
- Device Family Pack (DFP): **dsPIC33CK-MP_DFP v1.8.299**
- Curiosity/Starter Kits Tool Pack : **PKOB4_TP v1.18.1438**
- MPLAB® XC-DSC Compiler **v3.10**
- MPLAB® X IDE Plugin: **X2C-Scope v1.6.6** 
> **Note:** </br>
>The software used for testing the firmware prior to release is listed above. It is recommended to use the version listed above or later versions for building the firmware. All previous versions of Device Family Packs (DFP) and Tool Packs can be downloaded from [Microchip Packs Repository.](https://packs.download.microchip.com/)
### 2.3 Hardware Tools Required for the Demonstration
- MCHV-230VAC-1.5kW Development Board  [(EV78U65A)](https://www.microchip.com/en-us/development-tool/ev78u65a) 
- dsPIC33CK64MC105 Motor Control Dual In-line Module [(EV03J37A)](https://www.microchip.com/en-us/development-tool/ev03j37a)
- Leadshine (EL5-M0400-1-24) 400W 220VAC Servo Motor[(AC300025)](https://www.microchip.com/en-us/development-tool/AC300025)
- AC Source for powering the Development Board: 150-230V<sub>ac rms</sub> , 50Hz

> **Note:** </br>
>  Items [EV78U65A](https://www.microchip.com/en-us/development-tool/ev78u65a), [EV03J37A](https://www.microchip.com/en-us/development-tool/ev03j37a) and [AC300025](https://www.microchip.com/en-us/development-tool/AC300025) are available to purchase directly from [microchip DIRECT](https://www.microchipdirect.com/) 
</br>

## 3. HARDWARE SETUP
This section describes the hardware setup required for the demonstration.
Refer ["Motor Control High Voltage 230VAC-1.5kW Development Board User's Guide"](https://ww1.microchip.com/downloads/aemDocuments/documents/MCU16/ProductDocuments/UserGuides/Motor-Control-High-Voltage-230VAC-1.5kW-Dev-Board-Users-Guide-DS70005576.pdf), before operating the unit.
> **Note:** </br>
>In this document, hereinafter the MCHV-230VAC-1.5kW Development Board is referred as **development board**.

1. Motor currents are amplified by the amplifiers internal to the dsPIC33CK64MC105 on the DIM. The firmware and DIM are configured to sample and convert internal amplifier outputs (**'internal op-amp configuration'**) by default to measure the motor currents needed to implement FOC. **Table-1** summarizes the jumper resistors(0R) on the DIM for **‘internal op-amp configuration’**.

     <p align="left" >
     <img  src="images/Table_internalopamp.jpg"width="800"/></p>

2. Ensure the development board is not powered and it is fully discharged. Verify the LEDs **LD1**(Green) and **LD4**(Red) on Power Factor Correction Board and **LD1**(Green) on Motor Control Inverter Board are not glowing. 


     <p align="left" >
     <img  src="images/MCHVnotpowered.jpg" width="500"></p>


3. Remove the thumb screw and open the top cover of the enclosure and measure the DC Bus voltage across the terminals which reads approximately '0 VDC'. If the voltage is greater than 1V, wait until it is completely discharged.
4. Insert the **dsPIC33CK64MC105 Motor Control DIM** into the DIM Interface **connector J2** on the development board. Make sure the DIM is placed correctly and oriented before going ahead. Close the top cover of the enclosure and secure it with the thumb screw.

     <p align="left" >
     <img  src="images/dimconnected.jpg" width="600"/></p>


5. Connect the 3-phase wires from the motor to **A**, **B**, and **C** (no specific order) of the connector J13(**MOTOR**) on the development board.

     <p align="left" >
      <img  src="images/motorconnection.png" width="600"/></p>

6. Power the PFC board using a controlled AC source by applying a voltage within in the range 150-230V<sub>ac rms</sub> and a frequency of 50Hz through IEC connector **connector J1** provided on the PFC board. If the supply frequency is 60Hz, refer to point No: 3 in the section [Basic Demonstration](#52-basic-demonstration) to change the input frequency in firmware parameters . 

      <p align="left">
      <img  src="images/mchv_input.png" width="600"/></p>
> **Note:** </br>
>The Development Board is designed to operate in the 90 to 230V<sub>ac rms</sub> voltage range with a maximum input current of 10A<sub>rms</sub>. In the Input AC voltage range of 90 to 150V<sub>ac rms</sub>, the maximum input power to the Development Board must be derated (<1500W) to maintain the input current through the socket to less than or equal to 10A<sub>rms</sub>.

5. The Development Board has an on-board programming tool called the Isolated PKoB4 Daughter Board. To use the on-board programmer, connect a micro-USB cable between the Host PC and connector J11(**PROGRAM**) on the development board.
      <p align="left">
      <img  src="images/program.png" width="500"/></p>
> **Note:** </br>
> Use only **shielded micro-USB** cables intended for data transfer.

 6. To establish serial communication with the host PC, connect a micro-USB cable between the host PC and connector J8(**USB-UART**) on the development board. 
      <p align="left">
      <img  src="images/usb_uart.png" width="500"/></p>

> **Note:** </br>
> Use only **shielded micro-USB** cables intended for data transfer.
 </br>

## 4. SOFTWARE SETUP AND RUN
### 4.1 Setup: MPLAB X IDE and MPLAB XC-DSC Compiler
Install **MPLAB X IDE** and **MPLAB XC-DSC Compiler** versions that support the device **dsPIC33CK64MC105** and **PKoBv4.** The MPLAB X IDE, MPLAB XC-DSC Compiler, and X2C-Scope plug-in used for testing the firmware are mentioned in the [Motor Control Application Firmware Required for the Demonstration](#21-motor-control-application-firmware-required-for-the-demonstration) section. 

To get help on  
- MPLAB X IDE installation, refer [link](https://microchipdeveloper.com/mplabx:installation)
- MPLAB XC-DSC Compiler installation steps, refer [link](https://developerhelp.microchip.com/xwiki/bin/view/software-tools/xc-dsc/install/)

If MPLAB IDE v8 or earlier is already installed on your computer, then run the MPLAB driver switcher (Installed when MPLAB®X IDE is installed) to switch from MPLAB IDE v8 drivers to MPLAB X IDE drivers. If you have Windows 8 or 10, you must run the MPLAB driver switcher in **Administrator Mode**. To run the Device Driver Switcher GUI application as administrator, right-click on the executable (or desktop icon) and select **Run as Administrator**. For more details, refer to the MPLAB X IDE help topic **“Before You Begin: Install the USB Device Drivers (For Hardware Tools): USB Driver Installation for Windows Operating Systems.”**

### 4.2 Setup: X2C-SCOPE
X2C-Scope is a MPLAB X IDE plugin that allows developers to interact with an application while it runs. X2C-Scope enables you to read, write, and plot global variables (for motor control) in real-time. It communicates with the target using the UART. To use X2C-Scope, the plugin must be installed. To set up and use X2C-Scope, refer to the instructions provided on the [web page](https://x2cscope.github.io/docs/MPLABX_Plugin.html).

## 5.  BASIC DEMONSTRATION
### 5.1 Firmware Description
The firmware version needed for the demonstration is mentioned in the section [Motor Control Application Firmware Required for the Demonstration](#21-motor-control-application-firmware-required-for-the-demonstration) section. This firmware is implemented to work on Microchip’s Digital signal controller (dsPIC® DSC) **dsPIC33CK64MC105**. For more information, see the **dsPIC33CK64MC105 Family datasheet [(DS70005399)](https://ww1.microchip.com/downloads/en/DeviceDoc/dsPIC33CK64MC105-Family-Data-Sheet-DS70005399C.pdf)**.

The Motor Control Demo application uses a push button to start or stop the motor and a potentiometer to vary the speed of the motor. This Motor Control Demo Application configures and uses peripherals like PWM, ADC, UART, etc. For more details, refer to Microchip Application note **[AN1208](https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ApplicationNotes/ApplicationNotes/01208A.pdf), “Integrated Power Factor Correction (PFC) and Sensorless Field Oriented Control (FOC) System”** available on the [Microchip website](https://www.microchip.com/en-us/application-notes).

> **Note:**</br>
> The project may not build correctly in Windows OS if the Maximum path length of any source file in the project is more than 260 characters. In case the absolute path exceeds or nears the maximum length, do any (or both) of the following:
> - Shorten the directory name containing the firmware used in this demonstration. If you renamed the directory, consider the new name while reading the instructions provided in the upcoming sections of the document.
> - Place firmware in a location such that the total path length of each file included in the projects does not exceed the Maximum Path length specified. </br>
> Refer to MPLAB X IDE help topic **“Path, File, and Folder Name Restrictions”** for details. 

### 5.2 Basic Demonstration
Follow the below instructions, step by step, to set up and run the motor control demo application:

1. Start **MPLAB X IDE** and open the project **mcapp_pmsm.X (File > Open Project)** with device selection **dsPIC33CK64MC105.**  
    <p align="left">
       <img  src="images/deviceidselect.png" width="960"/></p>
  

2. Set the project **mcapp_pmsm.X** as the main project by right-clicking on the project name and selecting **Set as Main Project** as shown. The project **mcapp_pmsm.X** will then appear in **bold.**
    <p align="left">
     <img  src="images/ideprojectsetup.png" width="400"/></p> 

3. Open <code>**pfc_userparams.h**</code> (**mcapp_pmsm.X > Header Files> pfc**) in the project **mcapp_pmsm.X** .  
     - Ensure that the macro <code>**DEBUG_BOOST</code>** is not defined in the header file <code>**pfc_userparams.h**</code>.        
          
          <p align="left" >
          <img  src="images/undef_debug_boost.png" width="200"></p>
          
     - Ensure that the macro <code>**PFC_POWER_CONTROL</code>** is defined in the header file<code> **pfc_userparams.h.**</code>
          <p align="left" >
          <img  src="images/pfc_power_control.png" width="200"></p>
	
     - Update the input supply frequency by setting the macro <code>**PFC_INPUT_FREQUENCY</code>** in the header file <code>**pfc_userparams.h**</code>
          <p align="left" >
          <img  src="images/def_pfc_input_frequency.png" width="380"></p>

4. Open <code>**mc1_user_params.h** </code> (**mcapp_pmsm.X > Header Files**) in the project **mcapp_pmsm.X.**  
     - Ensure that the macro <code>**OPEN_LOOP_FUNCTIONING**</code> is not defined for closed loop functioning.
          <p align="left"> <img  src="images/def_openloopfun.png" width="260"/></p>

    - Firmware is configured to run with Leadshine (EL5-M0400-1-24) 400W 220VAC Brushless AC Servo Motor [(AC300025)](https://www.microchip.com/en-us/development-tool/ac300025) by default.  The macro <code>**LEADSHINE_EL5_M0400**</code> should be defined in <code>**mc1_user_params.h**</code>.
          <p align="left"> <img  src="images/def_leadshine.png" width="260"/></p>


<!-- > **Note:** </br>
> The motor parameters are available in the mcapp_pll_tuning_parameters.xlsx file in the **project\docs** folder. -->

5. Open <code>**port_config.h**</code> (**mcapp_pmsm.X > Header Files > hal**) in the project **mcapp_pmsm.X.**  
     - Ensure that the macro <code>**INTERNAL_OPAMP_CONFIG**</code> is defined in <code>**port_config.h**.</code>
          <p align="left"> <img  src="images/def_int_opamp_config.png" width="260"></p>

6. Right-click on the project **mcapp_pmsm.X** and select **Properties** to open its **Project Properties** Dialog. Click the **Conf:[default]** category to reveal the general project configuration information. The development tools used for testing the firmware are listed in section [2.2 Software Tools Used for Testing the firmware.](#22-software-tools-used-for-testing-the-firmware)

     In the **Conf:[default]** category window: 
     - Ensure the selected **Device** is **dsPIC33CK64MC105.**
     - Select the **Connected Hardware Tool** to be used for programming and debugging. 
     - Select the specific Device Family Pack (DFP) from the available list of **Packs.** In this case, **dsPIC33CK-MP_DFP v1.8.299** is selected. 
     - Select the specific **Compiler Toolchain** from the available list of **XC-DSC** compilers. 
     In this case, **XC-DSC(v3.10)** is selected.
     - After selecting Hardware Tool and Compiler Toolchain, Device Pack, click the button **Apply**

     Please ensure that the selected MPLAB® XC-DSC Compiler and Device Pack support the device configured in the firmware

     <p align="left">
     <img  src="images/projectpropertiessettings.png" width="900"/></p>

7. Ensure that the checkbox **Load symbols when programming or building for production (slows process)** is checked under the **Loading** category of the **Project Properties** window.           
      <p align="left">
      <img  src="images/loadvariables.png" width="900"/></p>

     Also, go to **Tools > Options** , and
      <p align="left">
      <img  src="images/tools_options.png" width="250"/></p>
   
    Open the  **Embedded > Generic Settings** tab and ensure that the **ELF debug session symbol load methodology (MIPS/ARM)** is selected as **Pre-processed (Legacy)** from the drop down.
           
      <p align="left">
      <img  src="images/embedded_legacy.png" width="900"/></p>
8. To build the project (in this case, **mcapp_pmsm.X**) and program the device dsPIC33CK64MC105, click **Make and Program Device Main project** on the toolbar
    <p align="left">
    <img  src="images/deviceprogramming.png" width="700"/></p>
  
9. If the device is successfully programmed, **LED1** will be turned **ON**, indicating that the dsPIC® DSC is enabled.
    <p align="left">
     <img  src="images/LED_1.png" width="350"/></p>

10. Measure the DC Bus voltage across the terminals (**VDC** and **PGND**) and ensure that the voltage is approximately 380V, as specified by the macro <code>**PFC_OUPUT_VOLTAGE_NOMINAL**</code> in the **pfc_userparams.h** file. 
    <p align="left">
     <img  src="images/VDC_measure.png" width="550"/></p>

> **Note:**</br>
> Please refer to the Board Inspection Checklist section in the Motor Control High Voltage 230VAC-1.5kW Development Board User’s Guide [(DS70005576)](https://ww1.microchip.com/downloads/aemDocuments/documents/MCU16/ProductDocuments/UserGuides/Motor-Control-High-Voltage-230VAC-1.5kW-Dev-Board-Users-Guide-DS70005576.pdf)
to verify the power supply outputs of the Development Board when a functional failure is suspected or observed.
 


9. Run or stop the motor by pressing the push button **BUTTON 1**. The motor should start spinning smoothly in one direction. Ensure that the motor is spinning smoothly without any vibration. The LED **LED2** is turned **ON** to show the button is pressed to start the motor.
     <p align="left">
     <img  src="images/pushbutton1.png" width="900"/></p>
 

10. The motor speed can be varied using the potentiometer **(POT).**
    <p align="left">
    <img  src="images/potentiometer.png"  width="550"/></p>
 

11. Press the push button **BUTTON 1** to stop the motor.


>**Note:**</br>
>The macros <code>MINIMUM_SPEED_RPM</code>, <code>NOMINAL_SPEED_RPM</code>, and <code>MAXIMUM_SPEED_RPM</code> are specified in the header file <code>**mc1_user_params.h**</code> included in the project **mcapp_pmsm.X.** The macros <code>NOMINAL_SPEED_RPM</code> and <code>MAXIMUM_SPEED_RPM</code> are defined as per the Motor manufacturer’s specifications. Exceeding manufacture specifications may damage the motor or the board or both.

## 5.3  Data visualization through X2C-Scope Plug-in of MPLAB X

X2C-Scope is a third-party plug-in in MPLAB X, which helps in real-time diagnostics. The application firmware comes with the initialization needed to interface the controller with the host PC to enable data visualization through the X2C-Scope plug-in. Ensure the X2C-Scope plug-in is installed. For more information on how to set up a plug-in, refer to either the [Microchip Developer Help page](https://microchipdeveloper.com/mplabx:tools-plugins-available) or the [web page.](https://x2cscope.github.io/docs/MPLABX_Plugin.html)
 
1. To establish serial communication with the host PC, connect a micro-USB cable between the host PC and **connector J8** on the development board. 

2. Ensure the application is configured and running as described under section [5.2 Basic Demonstration](#52-basic-demonstration) by following steps 1 through 10.

3. Open the **X2C-Scope** window by selecting **Tools>Embedded>X2CScope.**
      <p align="left">
       <img  src="images/x2cselection.png" width="550"/></p>
 

4. **In the X2C-Scope Configuration** window, open the **Connection Setup** tab and click **Select Project.** This opens the drop-down menu **Select Project** with a list of opened projects. Select the specific project **mcapp_pmsm** from the list of projects and click **OK.**
    <p align="left">
    <img  src="images/x2cprojectselection.png" width="400"/></p>

5. To configure and establish the serial communication for **X2C-Scope**, open the **X2CScope Configuration** window, click on the **Connection Setup** tab and:
     - Set **Baudrate** as **115200**, which is configured in the application firmware. 
     - Click on the **Refresh** button to refresh and update the list of the available Serial COM ports connected to the Host PC. 
     - Select the specific **Serial port** detected when interfaced with the development board. The **Serial port** depends on the system settings

    <p align="left">
     <img  src="images/x2cconnectionsetup.png" width="400"/></p>
 

6. Once the **Serial port** is detected, click on **Disconnected** and turn to **Connected**, to establish serial communication between the Host PC and the board.
     <p align="left">
    <img  src="images/x2cconnectionbutton.png" width="400"/></p>


7. Open the **Project Setup** tab in the **X2CScope Configuration** window and,
     - Set **Scope Sampletime** as the interval at which <code>X2CScopeUpdate()</code> is called. In this application, it is every <code>62µs</code>. 
     - Then, click **Set Values** to save the configuration.

      <p align="left">
      <img  src="images/x2cprojectsetup.png" width="400"/></p>


8.	Click on **Open Scope View** (in the **Data Views** tab of the **X2CScope Configuration** Window); this opens **Scope Window.**
     <p align="left">
      <img  src="images/x2cdataview.png" width="400"/></p>


    	     
9. In the **Scope Window**, select the variables that must be watched. To do this, click on the **Source** against each channel, and a window **Select Variables** opens on the screen. From the available list, the required variable can be chosen. Ensure checkboxes **Enable** and **Visible** are checked for the variables to be plotted.
To view data plots continuously, uncheck **Single-shot.** When **Single-shot** is checked, it captures the data once and stops. The **Sample time factor** value multiplied by **Sample time** decides the time difference between any two consecutive data points on the plot.
    <p align="left">
    <img  src="images/x2cdatapointselection.png" width="800"/></p>

10.	Click on **SAMPLE**, then the X2C-Scope window plots variables in real-time, which updates automatically.
     <p align="left">
     <img  src="images/x2csample.png" width="800"/></p>
 

11.	Click on **ABORT** to stop.
     <p align="left">
     <img  src="images/x2cabort.png" width="800"/></p>
 
 ## 6. REFERENCES:
For additional information, refer following documents or links.
1. AN1208 Application Note ["Integrated Power Factor Correction (PFC) and Sensorless Field Oriented Control (FOC) System"](https://www.microchip.com/en-us/application-notes/an1208)
2. Motor Control High Voltage 230VAC-1.5kW Development Board User’s Guide [(DS70005576)](https://ww1.microchip.com/downloads/aemDocuments/documents/MCU16/ProductDocuments/UserGuides/Motor-Control-High-Voltage-230VAC-1.5kW-Dev-Board-Users-Guide-DS70005576.pdf)
3. dsPIC33CK64MC105 Motor Control Dual In-Line Module (DIM) Information Sheet [(DS50003344)](https://ww1.microchip.com/downloads/aemDocuments/documents/MCU16/ProductDocuments/InformationSheet/dsPIC33CK64MC105-Motor-Control-DIM-Info-Sheet-DS50003344.pdf)
4. dsPIC33CK64MC105 Family datasheet [(DS70005399D)](https://ww1.microchip.com/downloads/aemDocuments/documents/MCU16/ProductDocuments/DataSheets/dsPIC33CK64MC105-Family-Data-Sheet-DS70005399D.pdf)
5. Implementation of Speed Sensor-less Field Oriented Control for Permanent Magnet Synchronous Motor [Sensorless_FOC_for_PMSM_and_IPSM_White_Paper.pdf](https://ww1.microchip.com/downloads/aemDocuments/documents/MCU16/ProductDocuments/SupportingCollateral/Sensorless_FOC_for_PMSM_and_IPSM_White_Paper.pdf)
6. MPLAB® X IDE User’s Guide [(DS50002027)](https://ww1.microchip.com/downloads/en/DeviceDoc/50002027E.pdf) or [MPLAB® X IDE help](https://microchipdeveloper.com/xwiki/bin/view/software-tools/x/)
7. [MPLAB® X IDE installation](http://microchipdeveloper.com/mplabx:installation)
8. [MPLAB® XC-DSC Compiler installation](https://developerhelp.microchip.com/xwiki/bin/view/software-tools/xc-dsc/install/)
9. [Installation and setup of X2Cscope plugin for MPLAB X](https://x2cscope.github.io/docs/MPLABX_Plugin.html)
10. [Microchip Packs Repository](https://packs.download.microchip.com/)
