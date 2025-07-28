#include <Arduino.h>
float Output = 0, Err = 0, pre_Err = 0;
float pPart = 0, iPart = 0, dPart = 0;
float Kp = 1, Ki = 0.3, Kd = 3;
const float Sampling_time = 20.0;
const float inv_Sampling_time = 1000.0 / Sampling_time;
// ADC & PWM
#define SENSOR_PIN  34      
#define PWM_PIN     18      
#define PWM_CH      0
#define PWM_FREQ    50  
#define PWM_RES     16      

float measured_pos = 0;
float input_pos = 0;

char readBuffer[4];

// Timer for sampling
hw_timer_t * timer = NULL;
volatile bool flag_pid = false;
volatile uint8_t sample_count = 0;

// Calibration range
#define MIN_ANALOG_READ 0
#define MAX_ANALOG_READ 4095

void IRAM_ATTR onTimer() {
  flag_pid = true;
  sample_count++;
}

void Throttle_Pos_PID(float desired_pos) {
  Err = desired_pos - abs(measured_pos);

  // PID calculations
  pPart = Kp * Err;
  dPart = Kd * (Err - pre_Err) * inv_Sampling_time;
  iPart += Ki * Sampling_time * Err / 1000.0;

  Output += pPart + dPart + iPart;

  // Saturation
  if (Output > 65535) Output = 65535;
  if (Output < 0) Output = 0;

  pre_Err = Err;

  // Output to PWM
  ledcWrite(PWM_CH, (uint32_t)Output);
}

void handleSerialCommand(char command) {
  if (command == 'R') {
    if (Serial.available() >= 4) {
      Serial.readBytes(readBuffer, 4);
      memcpy(&input_pos, readBuffer, 4); 
    }
  }
}

void setup() {
  Serial.begin(115200);
  ledcSetup(PWM_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWM_PIN, PWM_CH);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 20000, true);
  timerAlarmEnable(timer);
}

void loop() {
  if (flag_pid) {
    flag_pid = false;

    int adc_val = analogRead(SENSOR_PIN);
    measured_pos = ((float)(adc_val - MIN_ANALOG_READ) / (MAX_ANALOG_READ - MIN_ANALOG_READ)) * 100.0;
    if (measured_pos < 0) measured_pos = 0;

    Throttle_Pos_PID(input_pos);

    if (sample_count >= 5) {
      sample_count = 0;
      Serial.print("Angle: ");
      Serial.println(measured_pos);
    }
  }

  if (Serial.available()) {
    char cmd = Serial.read();
    handleSerialCommand(cmd);
  }
}
