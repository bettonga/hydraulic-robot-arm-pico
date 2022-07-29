#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"

const uint LED_PIN = 25;
const uint MOTOR[3] = {16,18,20};
const uint motor_min = 600;   // 0
const uint motor_max = 3495;  // 4095
const int MOT_SIGN[3] = {-1,-1,1};
const uint SENSOR[3] = {1,2,3};
uint16_t lvl_min, lvl_max, lvl_mid;

uint CAPTURE_PIN_BASE = 1;  // 1, 2 and 3
uint CAPTURE_PIN_COUNT = 1;
uint CAPTURE_N_SAMPLES = 2000; // 2000 samples at 1MHz has at least 1 whole SENT frame
float FREQ_DIV = 125.0f; // Divide 125Mhz by this to get your freq
uint FREQUENCY = 1000000; // 1us per sample needed to reliably detect 3us ticks
bool TRIGGER = false; // true = high : false = low
uint offset;
struct pio_program *capture_prog_2;

#define ENDSTDIN 255
#define NL 10

// initialize as global variables
const double max_error[3] = {10, 10, 30};
const double Kp[3] = {130.0,60.0,50.0}; // 150 | 150 | 50
const double Ki[3] = {0.0,0.0,0.0};   // 0 | 0 | 0
const double Kd[3] = {1.0,1.0,1.0};   // 1 | 1 | 1
double ANGLE[3] = {0.0,0.0,0.0};
double last_error[3] = {0.0,0.0,0.0};
double integral[3] = {0.0,0.0,0.0};


//// Initialization sequence for the motor, computes lvl_min, lvl_max, lvl_mid
void init_motor(const uint pin) {
  /**               _________                                 _____________                        ____________________________
                   |         |                               |             |                      |                            |
  125MHz clock --> | Divider | -- 125MHz / Divider clock --> | Counter +=1 | -- Counter value --> | 1 if Counter<Level, else 0 | -- TOR -->
                   |_________|                               |_____________|                      |____________________________|
  **/

  // Pico characteristics
  uint32_t f_clock = clock_get_hz(5); // get clock frequency
  uint16_t wrap_max = 65535;          // 2**16-1, the highest value the counter goes before reset to 0

  // PWM wanted value, for a servo-like PWM we need a 50Hz signal (0.02s period)
  float T = 0.02;
  float dt_min = 0.0011;
  float dt_max = 0.0019;

  // set up pwm on the line of pin 18
  gpio_set_function(pin, GPIO_FUNC_PWM);
  uint slice=pwm_gpio_to_slice_num(pin);
  uint channel=pwm_gpio_to_channel(pin);

  // enable pwm
  pwm_set_enabled(slice, true);
  // disable the phase correction (when counter > wrap, counter = 0)
  pwm_set_phase_correct(slice, false);

  // compute PWM characteristics (without phase correction)
  uint8_t divider = ceil( (T*f_clock)/wrap_max ) +1;      // divider = 39
  uint16_t wrap = ceil( (T*f_clock)/divider );            // wrap = 64102
  lvl_min = floor( (dt_min*f_clock)/divider );   // lvl_min = 3437
  lvl_max = ceil( (dt_max*f_clock)/divider );    // lvl_max = 5938
  lvl_mid = ceil( (lvl_min+lvl_max)/2 );         // lvl_mid = 4687

  // set the characteristics
  pwm_set_wrap(slice, wrap);
  pwm_set_clkdiv(slice, divider);

  // set the counter compare value
  pwm_set_gpio_level(pin, lvl_mid);
}
//// Command the motor according to the 0<value<4095
void motor_cmd(const uint pin, int value) {
  if (value<motor_min) {value=motor_min;}
  else if (value>motor_max) {value=motor_max;}
  uint16_t lvl = ceil(((lvl_max-lvl_min)/4095.0)*value +lvl_min);   // from 0<=value<=4095 to lvl_min<=lvl<=lvl_max
  pwm_set_gpio_level(pin, lvl);
  // printf("%u %u %d\n", pin, lvl, value);
}
//// Sawtooth int in degrees
double sawtooth_deg(double diff) {
  double ret = (360.0/M_PI)*atan(tan(diff*M_PI/360.0));
  return ret;
}
int pid_update(const int id_motor, const int setpoint, const double actual, const double dt) {
  double error = MOT_SIGN[id_motor] *sawtooth_deg(setpoint -actual);
  error = (error>max_error[id_motor]) ? max_error[id_motor] : ( (error<-max_error[id_motor]) ? -max_error[id_motor] : error );
  integral[id_motor] = integral[id_motor] + error*dt;
  double derivative = (error -last_error[id_motor]) /dt;
  last_error[id_motor] = error;
  int ret = (int) 2048 + round(error*Kp[id_motor] + integral[id_motor]*Ki[id_motor] + derivative*Kd[id_motor]);
  return ret;
}

//// Load a program to capture n pins. This is just a single `in pins, instruction with a wrap.
void logic_analyser_init(PIO pio, uint sm, uint pin_base, uint pin_count, float div) {
    uint16_t capture_prog_instr = pio_encode_in(pio_pins, pin_count);
    struct pio_program capture_prog = {
        .instructions = &capture_prog_instr,
        .length = 1,
        .origin = -1
    };
    capture_prog_2 = &capture_prog;

    offset = pio_add_program(pio, &capture_prog);

    // Configure state machine to loop over this `in` instruction forever,
    // with autopush enabled.
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_in_pins(&c, pin_base);
    sm_config_set_wrap(&c, offset, offset);
    sm_config_set_clkdiv(&c, div);
    sm_config_set_in_shift(&c, true, true, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
    pio_sm_init(pio, sm, offset, &c);
}
//// Adapted to the arm
void logic_analyser_arm(PIO pio, uint sm, uint dma_chan, uint32_t *capture_buf, size_t capture_size_words,
                        uint trigger_pin, bool trigger_level) {
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_clear_fifos(pio, sm);

    dma_channel_config c = dma_channel_get_default_config(dma_chan);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, false));

    dma_channel_configure(dma_chan, &c,
        capture_buf,        // Destinatinon pointer
        &pio->rxf[sm],      // Source pointer
        capture_size_words, // Number of transfers
        true                // Start immediately
    );

    pio_sm_exec(pio, sm, pio_encode_wait_gpio(trigger_level, trigger_pin));
    pio_sm_set_enabled(pio, sm, true);
}
//// CRC calculation function from: https://electronics.stackexchange.com/a/408455
bool crc_check(int *frame) {

    char data[6];
    for (int i = 0; i < 6; ++i)
    {
        data[i] = frame[i+1] + '0';
    }

    uint8_t calculatedCRC, i;
    const uint8_t CrcLookup[16] = {0, 13, 7, 10, 14, 3, 9, 4, 1, 12, 6, 11, 15, 2, 8, 5};
    calculatedCRC = 5; // initialize checksum with seed "0101"

    for (i = 0; i < 6; i++)
    {
        calculatedCRC = CrcLookup[calculatedCRC];
        calculatedCRC = (calculatedCRC ^ data[i]) & 0x0F;
    }
    // One more round with 0 as input
    calculatedCRC = CrcLookup[calculatedCRC];

    //printf("sum %d\n", calculatedCRC);
    //printf("crc %d\n", frame[7]);

    if (calculatedCRC == frame[7])
    {
        return true;
    }
    else {
        return false;
    }
}
////
void decode_SENT(const uint32_t *buf, uint pin_base, uint pin_count, uint32_t n_samples) {
    int periods[40];
    int period_buff = 0;
    int period_index = 0;
    bool level;
    int level_old;
    bool sync;
    int msg[8];

    // Loop through the samples and calculate times between falling edges
    for (int sample = 0; sample < n_samples; ++sample) {
        uint bit_index1 = 1 + sample * pin_count;
        for (int pin = 0; pin < pin_count; ++pin) {
            uint bit_index = pin + sample * pin_count;
            level = !!(buf[bit_index / 32] & 1u << (bit_index % 32));
        }

        // Look for falling edges to stop counting period length
        if ((level_old == 1) && (level == 0))
        {
            periods[period_index] = period_buff;
            period_buff = 1; // First 0 is added to the next period
            ++period_index;
        }

        // Between falling edges samples are counted
        else
        {
            ++period_buff;
        }

        // Keep track of the last sample to detect falling edge
        level_old = level;
    }


    // Loop through the period times and find the message frames
    int msg_index;
    for (int i = 0; i < 40; ++i)
    {
        // Find beginning of a message
        if ((periods[i]>=167) && (periods[i]<=169))
        {
            sync = true;
            msg_index = -1;
        }

        // If sync pulse has been detected, start recording the SENT frame
        if (sync == true)
        {
            ++msg_index;
            msg[msg_index] = (periods[i+1]/3 - 12);
        }

        // When 8 nibbles are collected (index 7), frame is complete
        if (msg_index == 7)
        {
            sync = false;
            msg_index = -1;

            // Check if a frame is valid and print out
            if (crc_check(msg) == true)
            {
                // We are interested only in the first 3 data nibbles
                int angle = (msg[1])<<8 | (msg[2])<<4 | (msg[3]);
                //printf("12bit: %d\n",  angle );
                // printf("pin:%d %s %d %s \n", (pin_base-1) , " | ", (angle*360/4095), "deg");
                if (pin_base==1) { //pin 1 is wrist
                  ANGLE[2] = (double) (angle*360.0/4095.0)*-1.01 +172.6; //*-0.8554 +135.7
                }
                else if (pin_base==2) { //pin 2 is shoulder
                  ANGLE[0] = (double) (angle*360.0/4095.0)*1.2526 -137.31616; //*0.7792 -121.4
                }
                else if (pin_base==3) { //pin3 is elbow
                  ANGLE[1] = (double) (angle*360.0/4095.0)*0.7258 -153.9;
                }
            }
        }
    }
}
////
void get_angle(uint pin_base, uint32_t *capture_buf, PIO pio, uint sm, uint dma_chan) {
  uint32_t capture_buf_memory_size = (CAPTURE_PIN_COUNT * CAPTURE_N_SAMPLES + 31) / 32 * sizeof(uint32_t);
  capture_buf = malloc(capture_buf_memory_size);
  // if (capture_buf == NULL) {printf("Error allocating capture buffer size %d\n", capture_buf_memory_size);}
  logic_analyser_init(pio, sm, pin_base, CAPTURE_PIN_COUNT, FREQ_DIV);
  uint32_t hz = clock_get_hz(clk_sys);
  float caphz = (float)hz/FREQ_DIV;
  gpio_put(LED_PIN, 1);
  logic_analyser_arm(pio, sm, dma_chan, capture_buf,
      (CAPTURE_PIN_COUNT * CAPTURE_N_SAMPLES + 31) / 32,
      pin_base, TRIGGER);
  dma_channel_wait_for_finish_blocking(dma_chan);
  gpio_put(LED_PIN, 0);
  decode_SENT(capture_buf, pin_base, CAPTURE_PIN_COUNT, CAPTURE_N_SAMPLES);
  pio_remove_program(pio, capture_prog_2, offset);
  free(capture_buf);
}
// Boost the baud rate to try to get the data out faster
// Probably should just call the init with the baud rate option set
#undef PICO_DEFAULT_UART_BAUD_RATE
#define PICO_DEFAULT_UART_BAUD_RATE 921600



int main() {
  stdio_init_all();

  // Motor initialization
  for (int i=0; i<3; i++) {
    init_motor(MOTOR[i]);
  }
  sleep_ms(200);

  // Sensors initialization
  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);

  uint32_t *capture_buf = 0;

  PIO pio = pio0;
  uint sm = 0;
  uint dma_chan = 0;

  // Serial communication initialization
  char strg[100];
  char chr;
  int lp = 0;
  memset(strg, 0, sizeof(strg));

  int id, value;
  char SPD[4] = "SPD\0";
  char ANG[4] = "ANG\0";
  char mode[4];
  int VALUE[3] = {2048,2048,2048};    // holds the command inputs of all three motors
  char MODE[3][4] = {"SPD", "SPD", "SPD"};
  uint32_t TIME[3] = {time_us_32(), time_us_32(), time_us_32()};


  // for (int i=0; i<3; i++) {
  //   calibrate(i, capture_buf, pio, sm, dma_chan);
  // }
  uint32_t time = time_us_32();
  while (true) {

    /// get incoming command "<mode> <id_motor> <value>", mode is ANG or SPD, id_motor is 0,1 or 2, value is either an angle (ANG) or speed (SPD)
    chr = getchar_timeout_us(0);
    while(chr != ENDSTDIN) {
    	strg[lp++] = chr;
    	if(chr == NL || lp == (sizeof(strg) - 1)) {
    		strg[lp] = 0;	//terminate string
        sscanf(strg, "%s %d %d", &mode, &id, &value);
        VALUE[id] = value;
        *MODE[id] = *mode;
        // printf("mode: %s | id_motor: %d | value: %d\n", mode, id, value);
    		lp = 0;		//reset string buffer pointer
    		break;
    	}
    	chr = getchar_timeout_us(0);
    }


    /// get sensors data
    ++CAPTURE_PIN_BASE;
    if (CAPTURE_PIN_BASE > 3) {CAPTURE_PIN_BASE = 1;}
    get_angle(CAPTURE_PIN_BASE, capture_buf, pio, sm, dma_chan);


    /// send sensors data to Olimex
    // printf("%f %f %f\n", (ANGLE[0]), (ANGLE[1]), (ANGLE[2]));


    /// send the commands to the motors
    for (int i=0; i<3; i++) {

      // SPD MODE: send corresponding command to the motors
      if (*MODE[i]==*SPD) {
        motor_cmd(MOTOR[i], VALUE[i]);
      }

      // ANG MODE: regulation of the cylinder to achieve desired angles
      else if (*MODE[i]==*ANG) {
        int cmd = pid_update(i, VALUE[i], ANGLE[i], (time_us_32()-TIME[i])/1000000.0);
        TIME[i] = time_us_32();
        motor_cmd(MOTOR[i], cmd);
      }

    }

    sleep_ms(10);
  }
  return 0;
}
