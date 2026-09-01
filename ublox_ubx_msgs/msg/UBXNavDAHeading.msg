# Dual antenna heading (0x01 0x45)
# Relative positioning information and heading between antenna 1 and antenna 2
std_msgs/Header header

uint8 version                  # message version
uint32 itow                    # ms - GPS Time of week of the navigation epoch
int32 rel_pos_n                # mm - North component of relative position vector
int32 rel_pos_e                # mm - East component of relative position vector
int32 rel_pos_d                # mm - Down component of relative position vector
int32 rel_pos_length           # mm - Length of the relative position vector
int32 rel_pos_heading          # deg scale 1e-5 - Heading of the relative position vector
uint32 acc_n                   # mm - Accuracy of relative position North Component
uint32 acc_e                   # mm - Accuracy of relative position East Component
uint32 acc_d                   # mm - Accuracy of relative position Down Component
uint32 acc_length              # mm - Accuracy of length of the relative position vector
uint32 acc_heading             # deg scale 1e-5 - Accuracy of heading of the relative position vector
bool gnss_fix_ok               # A valid fix (i.e within DOP & accuracy masks)
bool diff_soln                 # differential corrections were applied
bool rel_pos_valid             # relative position components and accuracies are valid
bool rel_pos_heading_valid     # relPosHeading is valid
bool rel_pos_normalized        # components of the relative position vector are normalized
