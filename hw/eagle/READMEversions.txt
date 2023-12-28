READMEversions.txt
12/27/23

pwrfet

1. Initial board (5 boards ordered): 

  TLP151A blew up twice, 

  AUIRS2110S didn't work twice. 

  Motor transients appeared to cause at least
one blowup.

  Hacked to add 12v regulator, and seemed to work.

2. pwrfet-v2 (5 boards ordered)

First version but added 
  12v regulator (two options)
  DGD05463 FET driver
  Misc improvements of caps foot prints, etc.

3. pwrfet-v3 (5 boards ordered)

V2 revised for ADP3110 or ADP3120 FET driver
which is a SOT package

4. pwrfet-v4 (layout not complete as of 12/27/23)

V3 revised to add 74HCT02 (quad NOR) and AP311 comparator
to limit max current in low-side FET source.

5. pfuart & pfCAN 

New repo started for a version using a STM32L431.
See: GliderWinchItems/isopwrfet.


