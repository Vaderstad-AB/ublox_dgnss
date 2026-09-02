# MonRFBlock - status of a single RF block
# An RF block corresponds to a GNSS band supported by the receiver, not to an antenna port.

# antenna supervisor state machine status
uint8 ANT_STATUS_INIT = 0
uint8 ANT_STATUS_DONTKNOW = 1
uint8 ANT_STATUS_OK = 2
uint8 ANT_STATUS_SHORT = 3
uint8 ANT_STATUS_OPEN = 4

# antenna power status
uint8 ANT_POWER_OFF = 0
uint8 ANT_POWER_ON = 1
uint8 ANT_POWER_DONTKNOW = 2

# jamming/interference state
uint8 JAMMING_UNKNOWN = 0
uint8 JAMMING_OK = 1
uint8 JAMMING_WARNING = 2
uint8 JAMMING_CRITICAL = 3

# GNSS band of this RF block
uint8 BAND_UNKNOWN = 0
uint8 BAND_L1 = 1
uint8 BAND_L2 = 2
uint8 BAND_L3 = 3
uint8 BAND_L5 = 4

uint8 block_id             # unique id of the RF block
uint8 jamming_state        # see JAMMING_* constants
uint8 ant_status           # see ANT_STATUS_* constants
uint8 ant_power            # see ANT_POWER_* constants
uint32 post_status         # deprecated by u-blox, always 0
uint16 noise_per_ms        # noise level measured by the GNSS core
uint16 agc_cnt             # AGC monitor, 0..8191 == 100% of maximum gain
uint8 cw_suppression       # CW interference suppression, 0 none .. 255 strong
int8 ofs_i                 # imbalance of I-part of complex signal
uint8 mag_i                # magnitude of I-part, 0 no signal .. 255 max
int8 ofs_q                 # imbalance of Q-part of complex signal
uint8 mag_q                # magnitude of Q-part, 0 no signal .. 255 max
uint8 rf_block_gnss_band   # see BAND_* constants
