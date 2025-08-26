# Hornet X Acoustics Public Archive

Uploaded 26/08 for archiving purposes.
Associated writeup found here: [https://tankaicong.com/projects/hornet/](https://tankaicong.com/projects/hornet/)

## Algorithm approach:
1. A timer interrupt runs at 3x the sampling frequency (3*125kHz)
	- For each timer callback, ADC conversion is started on one of the analog inputs to one of the ADC channels in a cyclical fashion (e.g. A0 -> ADC0, A1 -> ADC1, A2 -> ADC0, A0 -> ADC1, etc.)
	- Upon ADC conversion complete, the ADC interrupt callback will read the converted value and the current clock cycle ticks (measurement of reading time)
2. Readings are sent to their individual circular buffers of size 128
3. At a fixed frequency (2x of signal frequency -> 2*45kHz), DFT is performed on the earliest 64 values in the readings buffer, but only at 45kHz, not the entire freqeuncy domain.
	- If the DFT results in spikes above a specified threshold on all hydrophone channels, that indicates a ping has passed through the hydrohpone array. 
4. If ping detected, the next 64 readings (the presumably more stable portion of the ping) will be put into a sinusoidal regression to get the signal phases on each hydrophone
	- This is done by linearising the equation y = Asin(Bx+C) --> y = (Asin(c))\*cos(Bx) + (Acos(c))\*sin(Bx) --> y = αC + βS
	- This is now a bivariate linear regression on C and S which has a standard formula to determine coefficients α and β
	- Hence atan(α/β) returns c, the phase difference
5. For each pair of phase differences, there can be fixed angle(s) that can produce those phase differences (see acoustics data playground excel sheet for visualisation). By comparing the phase differences, the azimuth and elevations can be back calculated.
	- But due to aliasing (distance between hydrophones larger than one wavelength of the signal), there could be multiple possible angles that produce the phase difference
	- So narrow down range using first detected ping by comparison of pinger signal's order of arrival to the hydrophones.
	- If still have ambiguities then just take vector that is closest to last vector as the correct choice

## Testing done:
* Connected all channels to signal generator, which could read a near 0 phase difference between all channels to give elevation 90 deg as result
* Back-tested with old datalog for the 4 different positions, all seems to yield the expected result in their output(s).

## link dump:
* [4 channel ADC DMA read with claimed 500kSps](https://forum.pjrc.com/index.php?threads/t4-adc-real-time-timer-triggered-with-dma-using-adc_etc.76129/)
* [ADC DMA general thread](https://forum.pjrc.com/index.php?threads/adc-library-with-support-for-teensy-4-3-x-and-lc.25532/page-17)
* [T3.6 simul acquistion with PDB](https://forum.pjrc.com/index.php?threads/pedvides-adc-library-multiple-channel-simultaneous-continuous-acquisition.45206/)
* [Teensy4.1 IC ref manual](https://www.pjrc.com/teensy/IMXRT1060RM_rev3_annotations.pdf)