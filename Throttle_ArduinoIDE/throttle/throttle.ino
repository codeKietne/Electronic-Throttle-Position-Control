#include "THROTTLE.h"

// PID elements
float Output, Err, pre_Err=0;  
float pPart=0, iPart=0, dPart=0;
float Kp=1, Kd=3, Ki=0.3;	

// Control throttle position
float measured_pos = 0;
volatile uint8_t sample_count = 0;

// Read data from Labview
char readBuffer[4];	// For containing the string from LabView
float input_pos = 0;
char* ptr_input_pot = (char*)&input_pos;
//-----------------------------------------------------------------------------------
void Throttle_Pos_PID(float desired_pos)
{
  Err = desired_pos - abs(measured_pos);
  // PID element calculation
  pPart = Kp*Err;
  dPart = Kd*(Err -pre_Err)*inv_Sampling_time;
  iPart += Ki*Sampling_time*Err/1000.0;
  // chặn tích phân PID
  Output += pPart +dPart +iPart;

  // Saturation case handling
  if(Output > PWM_Period) Output = PWM_Period -1;
  if(Output < 0) Output = 0;
  
  pre_Err = Err;
  OCR1A = Output;
}

int main(void)
{
  Serial.begin(115200);
  TIMER1_INIT();
  TIMER2_INIT();
  MOTOR_INIT();
  SENSOR_ADC_READING_INIT();
  //--------------------------------------
  // Start the motor and PWM
  OCR1A = 1;
  ICR1 = PWM_Period;            // Set PWM TOP = 20000
  MOTOR_PORT |= (1<<MOTOR_POS); // CW
  sei();  
  //--------------------------------------
  while(1)
  {
    if(sample_count >= 5) // 100ms sampling data 
    {
      sample_count = 0;
      int voltage_signal = analogRead(A0);

      measured_pos = (float) ((voltage_signal - MIN_ANALOG_READ) / (MAX_ANALOG_READ - MIN_ANALOG_READ)) * 100;
      if (measured_pos < 0) measured_pos = 0;

      Serial.print("Angle: ");
      Serial.println(measured_pos);
    }

    if(Serial.available() > 0)
    {
      handleSerialCommand(Serial.read());
    }
  }
}

ISR(TIMER2_OVF_vect)
{
  ++sample_count;
  // 20ms sampling time
  Throttle_Pos_PID(input_pos);
}

void handleSerialCommand(char command)
{
  switch(command)
  {
    case 'R':   // Read input position
    {
      Serial.readBytes(readBuffer, 4);
      getFloatValue(readBuffer, (char*)&input_pos);
      break;
    }
    default:
      break;
  }
}

