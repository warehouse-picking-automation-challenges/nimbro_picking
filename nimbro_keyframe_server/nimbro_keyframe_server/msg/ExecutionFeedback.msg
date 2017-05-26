
uint32 JS_VEL_LIMIT       = 1    # Joint space velocity limit active
uint32 JS_ACC_LIMIT       = 2    # Joint space acceleration limit active
uint32 LIN_VEL_LIMIT      = 4    # Linear velocity limit active
uint32 LIN_ACC_LIMIT      = 8    # Linear acceleration limit active
uint32 ANG_VEL_LIMIT      = 16   # Angular velocity limit active
uint32 ANG_ACC_LIMIT      = 32   # Angular acceleration limit active
uint32 CART_JS_VEL_LIMIT  = 64   # Joint space velocity limit active after IK

bool playing
uint32 keyframe_index
uint32 active_limits
