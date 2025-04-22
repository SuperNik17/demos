// Send all motors backward
// According to the defined reference system (counter-clockwise with X-axis pointing toward the front bogie):
// - LEFT side motors (SX) rotate counter-clockwise for backward movement
// - RIGHT side motors (DX) rotate clockwise for backward movement
// This corresponds to:
//   BACKWARD on LEFT → counter-clockwise (K × I = J)
//   BACKWARD on RIGHT → clockwise (K × I = -J)
// Note: BA = Front Bogie

void LevogiroPos(int vel, int tpos, int tneg, int scheda) {
  int i;

  // Set LEFT side motors to run backward
  A0x6nM2->run(BACKWARD); // Front Left
  A0x6nM4->run(BACKWARD); // Middle Left
  A0x6nM3->run(BACKWARD); // Rear Left

  // Set RIGHT side motors to run backward
  A0x60M1->run(BACKWARD); // Front Right
  A0x60M4->run(BACKWARD); // Middle Right
  A0x60M3->run(BACKWARD); // Rear Right

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
