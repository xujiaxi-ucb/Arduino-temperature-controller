// gain function constants
const int p = 3000; const float f0 = 0.005; const float f1 = 0.05;
const int fs = 10; const float ts = 0.1; const int a = 10;
const int analogOffset = 683; const float voltageDivCoef = 2.6686;

// set up SPI ports
#include <SPI.h>
#define SS1 (1 << 2) //Slave Select 1 PORTB
#define SS1PORT &PORTB   

#define SS2 (1 << 1) //Slave Select 2 PORTB
#define SS2PORT &PORTB

#define SS3PORT &PORTB

#define SS4 (1 << 7) //Slave Select 4 PORTD
#define SS4PORT &PORTD

const int ss1 = 10;
const int ss2 = 9;
const int ss3 = 8;
const int ss4 = 7;


int inputPin = A0;  // select the input pin

// initialize output(0V) in bits
int outputBits = 2048;

// initialize values for volt/bit conversion
float inputVolts = 0;


// initialize array for data collection
// v stands for output voltage in Volts
// k stands for input voltage in Volts
// there is no input and output at first, so v and k are set to 0V (digital value 2048)
int v[3] = {0,0,0};
int k[3] = {0,0,0};

float limit_output (float output){
  float filtered;
  if (output > 10){
    filtered = 10;
  } else if (output < -10){
    filtered = -10;
  } else {
    filtered = output;
  }
  return filtered;
 }




void setup() {
  // set the slaveSelectPins as an output:
  pinMode (10, OUTPUT);
  pinMode (9, OUTPUT);
  pinMode (8, OUTPUT);
  pinMode (7, OUTPUT);

  // initialize voltage to zero
  // maximum value for the voltage is 4096
  // desired output is -10~10V
  int voltbits0=2048;

  // initialize SPI:
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  writeMCP492x(voltbits0, SS1, SS1PORT);
  pinMode(A0, INPUT);

  // for plotting
  Serial.begin(9600);

}

void loop() {
  
  // analogRead measures voltage between 0-5V, range 0-1023
  int digitalCurrentK = analogRead(inputPin); 
  
  inputVolts = input_to_volts(digitalCurrentK);

  //问题在这和下面的函数有关，我在fb函数里能看到震荡，在这就变成0了
  outputBits = feedback_control(inputVolts); 
  //Serial.println(outputBits);

  writeMCP492x(outputBits,10);
}
  

//Method to write to the DAC,using  digitalWrite for slave select
void writeMCP492x(uint16_t data,uint8_t slave_select) {
  // Take the top 4 bits of config and the top 4 valid bits (data is actually a 12 bit number) 
  //and OR them together
  uint8_t top_msg = (0x30 & 0xF0) | (0x0F & (data >> 8));

  // Take the bottom octet of data
  uint8_t lower_msg = (data & 0x00FF);

  // Select our DAC, Active LOW
  digitalWrite(slave_select, LOW);

  // Send first 8 bits
  SPI.transfer(top_msg);
  // Send second 8 bits
  SPI.transfer(lower_msg);

  //Deselect DAC
  digitalWrite(slave_select, HIGH);
}


//Method to write to the DAC, using direct port for slave select
void writeMCP492x(uint16_t data,uint8_t ss,volatile uint8_t* slave_port) {
  // Take the top 4 bits of config and the top 4 valid bits (data is actually a 12 bit number) 
  //and OR them together
  uint8_t top_msg = (0x30 & 0xF0) | (0x0F & (data >> 8));

  // Take the bottom octet of data
  uint8_t lower_msg = (data & 0x00FF);

  // Select our DAC, Active LOW
  *slave_port &= ~ss;

  // Send first 8 bits
  SPI.transfer(top_msg);
  // Send second 8 bits
  SPI.transfer(lower_msg);

  //Deselect DAC
  *slave_port |= ss;
}

// feedback control function
float feedback_control(float voltInput) {
  // update input signal
  float tempK = k[1];
  k[1] = k[0];
  k[2] = tempK;
  k[0] = voltInput;

  // Gain function
  int outputVoltage = v[1] * (2 + f1 * ts) / (1 + f1 *ts)
                 + v[2] * (-1) / (1 + f1 *ts)
                 + p * k[0] * (a + ts * (f1 + a * f0) + f0 * f1 * pow(ts, 2)) / (1 + f1 * ts)
                 + p * k[1] * (-2 * a - ts * (a * f0 + f1)) / (1 + f1 * ts)
                 + p * k[2] * (a / (1 + f1 * ts));

  // filter the voltage
  outputVoltage = limit_output(outputVoltage);

  // update output signal
  float tempV = v[1];
  v[1] = v[0];
  v[2] = tempV;
  v[0] = outputVoltage;

  

  // output volt to bit funciton here:
  outputBits = output_to_bits(outputVoltage);
  

  

  // frequency is 10Hz, so we need a delay of 100ms
  delay(100);
}

float input_to_volts(int input) {
  float inputVolts = (float) input * 0.00488;
  return inputVolts;
}

int output_to_bits(float output) {
  int outputBits = (int) output * 136.5 + 2048;
  return outputBits;
}

  
