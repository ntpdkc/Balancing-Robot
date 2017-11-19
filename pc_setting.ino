/*
 * Raspberry Control Turning
 */
void remoteSetting(String setting_) {
  if (setting_.substring(0, 2) == "fr") {
    fforward = setting_.substring(2, 5).toInt();
  }
  else if (setting_.substring(0, 2) == "tr") {
    fturn = setting_.substring(2, 5).toInt();
  }
  else if (setting_.substring(0, 6) == "enTrue") {
    robot_enable = true;
  }
  else if (setting_.substring(0, 7) == "enFalse") {
    robot_enable = false;
  }
}

