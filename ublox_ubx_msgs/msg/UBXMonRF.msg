# UBXMonRF - RF front end status per RF block, used to diagnose antenna and interference problems
std_msgs/Header header # Header timestamp should be acquisition time

uint8 version          # Message version
uint8 n_blocks         # Number of RF blocks reported
uint8 msg_source       # 2 bit receiver information field, identifies the reporting receiver
MonRFBlock[] blocks    # Status of each RF block
