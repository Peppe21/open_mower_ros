################################
## Hardware Specific Settings ##
################################

# The type of mower you're using, used to get some hardware parameters automatically
# Currently supported:
# YardForce500
# CUSTOM (put your configs in ~/mower_params/)
export OM_MOWER="YardForce500"

# Your Hardware Version (more a firmware version, really). Check the OpenMower docs (https://www.openmower.de/docs) for the firmware versions.
# Supported values as of today:
# 0_10_X_WT901: Use this if you have an WT901 and have a 0.10.x mainboard.
# 0_10_X_MPU9250: Use this if you have an MPU9250 and have a 0.10.x mainboard (be aware that there are many fake chips on the market. So probably not your hardware version).
# 0_9_X_WT901_INSTEAD_OF_SOUND: Use this if you have soldered the WT901 in the sound module's slot and have a 0.9.x mainboard.
# 0_9_X_MPU9250: Use this if you have an MPU9250 and have a 0.9.x mainboard (be aware that there are many fake chips on the market. So probably not your hardware version).
export OM_HARDWARE_VERSION="0_9_X_WT901"

# Select your ESC type
# Supported values as of today:
# xesc_mini: for the STM32 version (VESC)
# xesc_2040: for the RP2040 version (very experimental!)
export OM_MOWER_ESC_TYPE="xesc_2040"

# Select your gamepad
# Currently supported: ps3, xbox360
#export OM_MOWER_GAMEPAD="xbox360"
#export OM_MOWER_GAMEPAD="glabkpad"
export OM_MOWER_GAMEPAD="android"

# Set to true to record your session.
# Output will be stored in your $HOME
# export OM_ENABLE_RECORDING_ALL=False

################################
##        GPS Settings        ##
################################
# Relative Positioning vs LatLng coordinates
# If OM_USE_RELATIVE_POSITION=False, we're using an arbitrary point as map origin. This point is called the DATUM point and
# needs to be set using OM_DATUM_LAT and OM_DATUM_LONG below.
# If OM_USE_RELATIVE_POSITION=True, we're using the ublox NAVRELPOSNED messages as position.
# This makes your base station the map origin
# For it recommended to set OM_USE_RELATIVE_POSITION to False. This way you can move your base station without re-recording your maps and it's also more compatible overall.
export OM_USE_RELATIVE_POSITION=False

# If needed, uncomment and set to coordinates near you (these default coordinates are somewhere in Germany).
# This will be your map origin!
export OM_DATUM_LAT=46.137842
export OM_DATUM_LONG=6.150889

# GPS protocol. Use UBX for u-blox chipsets and NMEA for everything else
export OM_GPS_PROTOCOL=UBX

# NTRIP Settings
# Set to False if using external radio plugged into the Ardusimple board.
export OM_USE_NTRIP=True
export OM_NTRIP_HOSTNAME=caster.centipede.fr
export OM_NTRIP_PORT=2101
export OM_NTRIP_USER=centipede
export OM_NTRIP_PASSWORD=centipede
export OM_NTRIP_ENDPOINT=NCY

# If you want to use F9R's sensor fusion, set this to true (you will also need to set DATUM_LAT and DATUM_LONG.
# Consider this option unstable, since I don't have the F9R anymore, so I'm not able to test this.
# IF YOU DONT KNOW WHAT THIS IS, SET IT TO FALSE
export OM_USE_F9R_SENSOR_FUSION=False


################################
##    Mower Logic Settings    ##
################################
# The distance to drive forward AFTER reaching the second docking point
export OM_DOCKING_DISTANCE=1.0

# The distance to drive for undocking. This needs to be large enough for the robot to have GPS reception
export OM_UNDOCK_DISTANCE=2.0

# How many outlines should the mover drive. It's not recommended to set this below 4.
export OM_OUTLINE_COUNT=4

# The width of mowing paths.
# Choose it smaller than your actual mowing tool in order to have some overlap.
# 0.13 works well for the Classic 500.
export OM_TOOL_WIDTH=0.13

# Voltages for battery to be considered full or empty
export OM_BATTERY_EMPTY_VOLTAGE=23.0
export OM_BATTERY_FULL_VOLTAGE=28.0

# Mower motor temperatures to stop and start mowing
export OM_MOWING_MOTOR_TEMP_HIGH=80.0
export OM_MOWING_MOTOR_TEMP_LOW=40.0

export OM_GPS_WAIT_TIME_SEC=10.0
export OM_GPS_TIMEOUT_SEC=5.0

# Mowing Behavior Settings
# True to enable mowing motor
export OM_ENABLE_MOWER=true

# True to start mowing automatically. If this is false, you need to start manually by pressing the start button
export OM_AUTOMATIC_START=false
export OM_OUTLINE_OFFSET=0.05

################################################################
## Legacy stuff for mode use_legacy_localization = True
################################################################

# Angle which gets added to the compass heading, if your imu is in a rotated coordinate frame
export OM_IMU_OFFSET=180.0

# Offset between the point in between your wheels and the GPS antenna in m
export OM_GPS_ANTENNA_OFFSET=0.3

# Serial ports of your hardware devices
export OM_LL_SERIAL_PORT=ttyAMA0
export OM_LEFT_SERIAL_PORT=ttyAMA4
export OM_RIGHT_SERIAL_PORT=ttyAMA2
export OM_MOW_SERIAL_PORT=ttyAMA3
export OM_GPS_SERIAL_PORT=ttyAMA1
# IMU Calibration and filter settings
export OM_MAG_BIAS_X=-0.027
export OM_MAG_BIAS_Y=0.039
export OM_MAG_BIAS_Z=0.025
export OM_IMU_FILTER_GAIN=0.05