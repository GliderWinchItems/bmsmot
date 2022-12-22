# bmsmot
This board is the control for the BMS (battery monitoring system) for one string of batteries; the contactor function, thermistor inputs for motor coolant, and output drive for enables, relays, and external high current FET sub-boards. A STM32F446RET6 is the main processor on this board.

The contactor function measures the high voltage using a STM32L431RCT6, ground isolated and communicates readings to the main processor via uart with opto-isolator. It also measures battery string current using an external Hall-effect sensor.

The main board provides for the CAN bus cabling to the BMS modules to loop back and apply +12v current to both ends of the CAN cabling, as well as in & out connectors for other nodes on the CAN bus. Provision is for a second CAN bus.

The software provides the EMC (Energy Management Control) and MMC (Motor Management Control) functions for one battery/motor installation.

