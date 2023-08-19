Robot_1 is household chores helper ground drone.

Parts that will need to be present:

-Microcontroller
-ESC
-Motors (driveline & suction impeller, probably Centrifugal Impeller/Turbine/Compressor - see Dyson)
-Battery
-Charging station
-Minimalistic HMI

########################################################################################################################################################################################################################################################################################################################################################################################

Rover System Notes

Battery Pack:
18650
3.7V (Fully charged 4.2V, Depleted 3.2V)

Voltage (V) should match to the device we are powering
(e.g. 3D printer = 24V)

Current (A) should be supplied in enough of a magnitude to drive the load

Capacity (Ah) will determine how long the device can be powered for
(e.g. 2Ah battery will supply 1A of current for 2 hours, or 0.5A for 4 hours)


Configuration: 
+ Series - cells stacked positive to negative - voltage is added, current and capacity unchanged. (2 batteries in series = 2S = 7.4V)
+ Parallel - cells positive to positive, negative to negative - voltage is the same, current and capacity added (3 batteries in parallel = 3P = 3.7V but 3x current and capacity). 
+ Series & Parallel - combination to achieve desired performance (2S3P: 7.4V, 3x Current & Capacity)

The most important bit is the Current - be aware that it would sometimes say that rated current is 40A, but it's most likely just a pulse discharge. 
Look at Continuous discharge.
Cells are either optimised for high current OR high capacity, but not both.
Highest continuous discharge: ~30A (1500mAh)
Highest capacity: 3500mAh (10A continuous)

Better to stick with established manufacturers (LG, Samsung, Sony, Panasonic


Links:
+ FAQ Lithium ion https://www.batterypowertips.com/difference-between-lithium-ion-lithium-polymer-batteries-faq/
+ Use calculators like this: https://power-calculation.com/battery-storage-calculator.php
+ Tesla Battery: https://www.evwest.com/catalog/product_info.php?products_id=463