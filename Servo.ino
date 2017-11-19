/*
 * Servo control using timer
 */

void InitServo() {
  int temp;
  // Initialize Timer4 as Fast PWM
  TCCR4A = (1 << PWM4A) | (1 << PWM4B);
  TCCR4B = 0;
  TCCR4C = (1 << PWM4D);
  TCCR4D = 0;
  TCCR4E = (1 << ENHC4); // Enhaced -> 11 bits

  temp = 1500 >> 3;
  TC4H = temp >> 8;
  OCR4B = temp & 0xff;

  // Reset timer
  TC4H = 0;
  TCNT4 = 0;

  // Set TOP to 1023 (10 bit timer)
  TC4H = 3;
  OCR4C = 0xFF;
  
  DDRB |= (1 << 6);  // OC4B = PB6 (Pin10 on Leonardo board)

  //Enable OC4A and OC4B and OCR4D output
  TCCR4A |= (1 << COM4B1) | (1 << COM4A1);
  //TCCR4C |= (1 << COM4D1);
  // set prescaler to 256 and enable timer    16Mhz/256/1024 = 61Hz (16.3ms)
  TCCR4B = (1 << CS43) | (1 << CS40);
}

void SWrite(int _pwm) {
  _pwm = constrain(_pwm, 700, 2300) >> 3;
  TC4H = _pwm >> 8;
  OCR4B = _pwm & 0xFF;//chan so 10 tren board
}

void remoteServo(String _str){
  if(_str.substring(0,2) == "up"){
    servo_ud = 1;
  }
  else if(_str.substring(0,2) == "dw"){
    servo_ud = -1;
  }
  else if(_str.substring(0,2) == "sp"){
    servo_ud = 0;
  }
  else if(_str.substring(0,2) == "au"){
    angle_servo = 1500;
    servo_ud = 0;
  }
  angle_servo = angle_servo + servo_ud;
  if(angle_servo > 1850) angle_servo = 1850;
  if(angle_servo < 1100)  angle_servo = 1100;
  
  SWrite(angle_servo);
}

