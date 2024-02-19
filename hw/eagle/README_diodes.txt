README_diodes.txt
02/11/2024


1. Error on .brd footprint for diode

sub-pwr-fet-v2,v3, v4

Footprint has anode/cathode backwards on .brd, but 
on the schematic, .sch, it is correct.

Assembling the 'v2 and placing the diode as shown on
the .brd still works as the fet driver has the boostrap
diode built in (and the Vcc is > 4,5.v). The 
backwards diode does nothing.

This is not true for the 'v4 as the external diode
is needed, so it has to be installed correctly.
However, the ADP3120 has some sort of startup
problem.

