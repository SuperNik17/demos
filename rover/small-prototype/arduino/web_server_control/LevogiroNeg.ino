// Send all motors forward
// According to the defined reference system (counter-clockwise with X-axis pointing toward the front bogie):
// - LEFT side motors (SX) rotate clockwise for forward movement
// - RIGHT side motors (DX) rotate counter-clockwise for forward movement
// This corresponds to:
//   FORWARD on LEFT → clockwise (K × I = J)
//   FORWARD on RIGHT → counter-clockwise (K × I = -J)
// Note: BA = Front Bogie

void LevogiroNeg(int vel, int tpos, int tneg, int scheda) {
  int i;

  // Set LEFT side motors to run forward
  A0x6nM2->run(FORWARD); // Front Left
  A0x6nM4->run(FORWARD); // Middle Left
  A0x6nM3->run(FORWARD); // Rear Left

  // Set RIGHT side motors to run forward
  A0x60M1->run(FORWARD); // Front Right
  A0x60M4->run(FORWARD); // Middle Right
  A0x60M3->run(FORWARD); // Rear Right

  // Gradually increase motor speeds
  for (i = 20; i < vel; i++) {
    A0x6nM2->setSpeed(i / tpos);
    A0x6nM4->setSpeed(i / tpos);
    A0x6nM3->setSpeed(i / tpos);

    A0x60M1->setSpeed(i / tneg);
    A0x60M4->setSpeed(i / tneg);
    A0x60M3->setSpeed(i / tneg);
    // Optional: add delay for smoother ramp-up
    // delay(10);
  }
}
