# ECSE444- Microcontroller Course
### Final Project 

Team members : 
- Rania Ouassif 
- Sansitha Panchadsaram 
- 
- 

In the final project of the semester, our group will use the B-L475EIOT01A to build an interactive application that combines the features explored in the earlier labs.
 - DAC Speaker, 
 - DFSDM Microphone,
 - QSPI Flash, 
 - UART
## Project Description
  The interactive application that we will implement is a memory game in which the user will have to memorize a sequence of digits played by the DAC speaker and enter those digits via UART. First, the user must press the pushbutton to start the game. Then, the speaker outputs a sequence of digits, which the user must memorize and input into the terminal. After which, the terminal displays a message to confirm the input validity.  

  The goal of the game is to win through the three levels of difficulty (easy, medium, hard) by entering the correct digits within a time limit. The game difficulty increases by setting a shorter timer and by adding more digits to the sequence. The DFSDM microphone will be used to record the list of digits and QSPI Flash will be used to store them in memory. The performance of this application will be enhanced by using low-power operation. The WaitForInterrupt() function will be used to put the processor sleep until an interrupt is triggered. 
