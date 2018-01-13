// user defined variables

// example variables used in Wii camera testing - replace with your own
// variables
#ifdef USERHOOK_VARIABLES
#define BUFFER_FRAME_SIZE   10

uint16_t ips_bytes;
uint16_t ips_data[3];
uint16_t c_buff;
uint16_t c_state;
char ips_char[BUFFER_FRAME_SIZE];
uint32_t ips_delay_ms; 
float air_temperature;
Vector3f ips_gyro, ips_accel;
Vector2f opt_flowRate;
Vector2f opt_bodyRate;
uint32_t opt_integration_timespan;

uint16_t s16_US_HEIGHT;

#endif  // USERHOOK_VARIABLES


