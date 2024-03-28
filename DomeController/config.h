// Configuration
#define HAS_SHUTTER  // Uncomment if the shutter controller is available
#define USE_BUTTONS  // Uncomment if you want to move the dome with push buttons

#define AZ_TIMEOUT (30000)  // Azimuth movement timeout (in ms)
#define AZ_TOLERANCE (4)    // Azimuth target tolerance in encoder ticks
#define AZ_SLOW_RANGE (16)  // The motor will run at slower speed when the
                          // dome is at this number of ticks from the target

#define ENCODER_PRE_DIV (5)  // number of ticks subdivision
#define ENCODER_TICKS_PER_TURN (600)  // for a complete dome revolution
#define HOME_POSITION (0)
