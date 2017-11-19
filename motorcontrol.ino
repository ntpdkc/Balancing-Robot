/*
   Raspberry Control Motor
*/

void RobotControl() {
  int pin_forward = !digitalRead(13);
  int pin_back    = !digitalRead(11);
  int pin_left    = !digitalRead(6);
  int pin_right   = !digitalRead(9);

  if (pin_forward && !pin_back) {//tien
    throttle += 1;//tien len cham cham
    if (throttle > fforward) throttle = fforward;
  }
  else if (pin_back && !pin_forward) { //Lui
    throttle -= 1;
    if (throttle < -fforward) throttle = -fforward;
  }
  else if (!pin_forward && !pin_back) { //Dung
    if (throttle > 0) {
      throttle -= 1;
    } else if (throttle < 0) {
      throttle += 1;
    } else {
      throttle = 0;
    }
  }

  //dieu khien trai phai
  if (pin_left && !pin_right) {
    steering += 0.5;
    if (steering > fturn) steering = fturn;
  }
  else if (pin_right && !pin_left) {
    steering -= 0.5;
    if (steering < -fturn) steering = -fturn;
  }
  else if (!pin_left && !pin_right) {
    steering = 0;
//    if (steering > 0) {
//      steering -= 0.5;
//    }
//    else if (steering < 0) {
//      steering += 0.5;
//    }
//    else {
//      steering = 0;
//    }
  }
}

