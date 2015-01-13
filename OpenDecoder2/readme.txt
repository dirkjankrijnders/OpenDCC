OpenDecoder2
-----------------------------------------------------------------------------------------
DCC Decoder based on Atmega8515 or Atmega162
  This archive contains:
  HEX and EEP files  (to be found in folder hex)
  Sourcecode

(c) Kufer Wolfgang
http://www.opendcc.de
------------------------------------------------------------------------------------------
Version 0.12 from 14.09.2010
- added REVERSER Mode (only this version is different to V0.11)

Version 0.11 from 05.05.2009
- fixed an cv-programming issue with the delay power up from version 10
- selectable power up sequence for servos:
  (CV555, Bit 6 = 0: -> pulse chain = ___________X______X_____X_____X_____ )
  (CV555, Bit 6 = 1: -> pulse chain = XXXXXXXXXXXX______X_____X_____X_____ )

Version 0.10 from 13.01.2009
- special init sequence for servo, this gives small or nearly zero
  unintented movement on most servos.
  I didn't succeed to completely remove the initial move.
  (pulse chain = XXXXXXXXXXXX______X_____X_____X_____ )
- delayed power up of servo motors (only if switches on board are fitted)
- LED will light dimmed during delay

Version 0.9 from 15.12.2008
- added HW defines for Hardware 2.5 (small servo only decoder)
- bugfix in servo code for 2.5 (15.12.2008)
- bugfix in init: unintentionally the init for segement is performed
  and does a unwanted novement at power up.

Version 0.8 from 08.02.2008
- all: dcc preamble recognition changed from >10 to >=10 bits.

Version 0.7 from 27.01.2008
- DMX: changed defaults
  now 72 virtual decoders, 4 macros, each 18 decoders
  (reason: better mapping for simulation software - coming soon)

Version 0.6 from 09.01.2008
- DMX: Lightnings, Relay Control, Pullup for Tracers
- PORT: Line settling before read of feedback lines
- all Versions: Error code 'unprogrammed' (5x) added
