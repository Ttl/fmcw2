
FMCW radar hardware, firmware and processing software.

Folders:

hardware: KiCad hardware design files.

fmcw2: RF board.
fmcw2_mcu: Digital board.

firmware: Firmware for LPC4320 microcontroller. Based on [HackRF](https://github.com/mossmann/hackrf) firmware.

host: Host programs for recording data, programming firmware. Also from HackRF with small modifications.

processing:
fir: Resamples, filters and repackages samples for other programs.
filter.py: Output FIR filter coefficients for fir C program.
analysis.py: Various analyses about the recorded data.
sar_bicycle.py: Pre-processing of raw data.
sar_process.py: Focus raw data to SAR image.
sar_autofocus.py: Simple minimum entropy autofocusing of SAR image.

SAR focusing program is loosely based on Matlab program by Gregory L. Charvat
and it also includes some functions from [RITSAR](https://github.com/dm6718/RITSAR).
