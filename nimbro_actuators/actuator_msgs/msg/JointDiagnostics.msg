# Joint name
string name

# Temperature in degree celsius
float32 temperature

# Voltage in 0.1 volts
uint16 voltage

# Number of timeouts since microcontroller startup
uint32 timeouts

# Error byte of last received status packet. Zero = Everything okay.
uint8 error

# Current load of the servo in Nm
float32 torque

# Current torque limit of the servo in proportion (0.0 .. 1.0) to maxTorque from config_momaro.yaml
float32 goalTorque

# Indicator how likely it is, that the servo had a timeout
float32 timeoutConfidence