#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <math.h>

#include "pico.h"
#include "pico/time.h"
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"

const uint LED_PIN = 25;
const uint MOTOR[3] = {16,18,20};
const uint motor_min = 0;   // 0
const uint motor_max = 4095;
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
#define NB_MEM 5

// initialize as global variables
double ANG[3];
double VEL[3];
double ANG_MEM[3][NB_MEM*2];
uint32_t T_MEM[3][NB_MEM*2];
uint8_t I[3] = {0,0,0};

double Kp_wrist=50;
double Ki_wrist=0;
double Kd_wrist=1;
double wrist_integral=0;
double wrist_last_error=0;

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

void push_back_double(const uint i, double a[][NB_MEM*2], double b) {
  for (int k = 0; k<NB_MEM*2-1; k++){
    a[i][k]=a[i][k+1];
  }
  a[i][NB_MEM*2-1] = b;
}
void push_back_uint32_t(const uint i, uint32_t a[][NB_MEM*2], uint32_t b) {
  for (int k = 0; k<NB_MEM*2-1; k++){
    a[i][k]=a[i][k+1];
  }
  a[i][NB_MEM*2-1] = b;
}
void update_pos_vel(const uint i){
  push_back_double(i, ANG_MEM, ANG[i]);
  push_back_uint32_t(i, T_MEM, time_us_32());
  double old = 0;
  for (int k=0; k<NB_MEM; k++){
    old = old + ANG_MEM[i][k];
  }
  old = old / (NB_MEM);
  double actual = 0;
  for (int k=NB_MEM; k<NB_MEM*2; k++){
    actual = actual + ANG_MEM[i][k];
  }
  actual = actual / (NB_MEM);
  double dt =  (T_MEM[i][NB_MEM*2-1] - T_MEM[i][NB_MEM])/1000000.0;
  VEL[i] = (actual-old) / dt;
  // printf("ANGLE %d  |  %.2f\n", i, VEL[i]);
  // printf("ANGLE %d  |  %.2f %.2f %.2f %.2f  |  %.2f\n", i, ANG_MEM[i][0], ANG_MEM[i][1], ANG_MEM[i][2], ANG_MEM[i][3], VEL[i]);
}

int pid_update_wrist(const int setpoint, const double actual, const double dt) {
  double error = MOT_SIGN[2] *(setpoint -actual);
  wrist_integral = wrist_integral + error*dt;
  double derivative = (error -wrist_last_error) /dt;
  wrist_last_error = error;
  return (int) 2048 + round(error*Kp_wrist + wrist_integral*Ki_wrist + derivative*Kd_wrist);
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
              break;
            }
        }
    }
    // We are interested only in the first 3 data nibbles
    int angle = (msg[1])<<8 | (msg[2])<<4 | (msg[3]);
    //printf("12bit: %d\n",  angle );
    // printf("pin:%d %s %d %s \n", (pin_base-1) , " | ", (angle*360/4095), "deg");
    if (pin_base==1) { //pin 1 is wrist
      ANG[2] = (double) (angle*360.0/4095.0)*-1.01 +172.6; //*-0.8554 +135.7
      update_pos_vel(2);
    }
    else if (pin_base==2) { //pin 2 is shoulder
      ANG[0] = (double) (angle*360.0/4095.0)*0.7792 -121.4;
      update_pos_vel(0);
    }
    else if (pin_base==3) { //pin 3 is elbow
      ANG[1] = (double) (angle*360.0/4095.0)*0.7258 -153.9;
      update_pos_vel(1);
    }
    printf("Shoulder  %.2f\n"
           "Elbow     %.2f\n"
           "Wrist     %.2f\n\n", VEL[0], VEL[1], VEL[2]);
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


// #undef PICO_DEFAULT_UART_BAUD_RATE
// #define PICO_DEFAULT_UART_BAUD_RATE 921600


int main() {
  stdio_init_all();

  // Motor initialization
  for (int i=0; i<3; i++) {
    init_motor(MOTOR[i]);
  }
  sleep_ms(1500);

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
  char SPD[4] = "SPD\0";
  char VEC[4] = "VEC\0";
  char mode[4] = "SPD\0";

  int val0 = 0;  // shoulder speed or vec_x in mm/s
  int val1 = 0;  // elbow speed or vec_y in mm/s
  int val2 = 0;  // wrist speed or wrist_ang_global in deg
  int a = 260;  // in mm
  int b = 247;
  double ang_wrist = 0;
  double w_shoulder, w_elbow;
  int cmd_shoulder = 2048;
  int cmd_elbow = 2048;
  uint32_t time_loop;

  while (true) {
    time_loop = time_us_32();
    /// get incoming command "<vec_double OLD_ANG[3];
    chr = getchar_timeout_us(0);
    while(chr != ENDSTDIN) {
    	strg[lp++] = chr;
    	if(chr == NL || lp == (sizeof(strg) - 1)) {
    		strg[lp] = 0;	//terminate string
        sscanf(strg, "%s %d %d %d", &mode, &val0, &val1, &val2);
        // printf("val0: %d | val1: %d | ang_wrist: %d\n", val0, val1, ang_wrist);
    		lp = 0;		//reset string buffer pointer
    		break;
    	}
    	chr = getchar_timeout_us(0);
    }

    /// get sensors data
    ++CAPTURE_PIN_BASE;
    if (CAPTURE_PIN_BASE > 3) {CAPTURE_PIN_BASE = 1; } //gpio_put(28, 1); going to 25 so it does not go too quick
    get_angle(CAPTURE_PIN_BASE, capture_buf, pio, sm, dma_chan);

    // compute desired w_shoulder and w_elbow according to desired vex_x and val1 | wrist angle according to wrist global
    w_shoulder = (180/M_PI)* ((val0 +val1*tan((M_PI/180)*(ANG[0]))) / (b*( sin((M_PI/180)*(ANG[0]+ANG[1])) +tan((M_PI/180)*(ANG[0]))*cos((M_PI/180)*(ANG[0]+ANG[1])) )));
    w_elbow = (180/M_PI)* ((-val1 -b*w_shoulder*cos((M_PI/180)*(ANG[0]+ANG[1]))) / ((a+b)*cos((M_PI/180)*(ANG[0]))));
    ang_wrist = val2 -ANG[0] -ANG[1];
    ang_wrist = (ang_wrist>90) ? 90 : ( (ang_wrist<-90) ? -90 : ang_wrist );

    /// send sensors data to Olimex
    // printf("%d %d %d\n", (int) round(ANG[0]), (int) round(ANG[1]), (int) round(ANG[2]));
    // printf("%.2f %.2f %.2f %.2f %.2f %.2f\n", (ANG[0]), (ANG[1]), (ANG[2]), (VEL[0]), (VEL[1]), (VEL[2]));

    /// send the commands to the motors: speed regulation for shoulder and elbow, wrist kept
    // SPD MODE: send corresponding command to the motors
    if (*mode==*SPD) {
      motor_cmd(MOTOR[0], val0);
      motor_cmd(MOTOR[1], val1);
      motor_cmd(MOTOR[2], val2);
    }


    if (*mode==*VEC) {
      cmd_shoulder = (int) 2048 + 100*ceil(w_shoulder -VEL[0]);
      motor_cmd(MOTOR[0], cmd_shoulder);
      printf("Setpoint %.2f\n"
             "Current  %.2f\n"
             "Command  %d\n\n", w_shoulder, VEL[0], cmd_shoulder);
      // int cmd = pid_update_wrist(ang_wrist, ANG[2], (time_us_32()-time_loop)/1000000.0 );
      motor_cmd(MOTOR[1], 2048);
      motor_cmd(MOTOR[2], 2048);

    }

    sleep_ms(10);
  }
  return 0;
}
