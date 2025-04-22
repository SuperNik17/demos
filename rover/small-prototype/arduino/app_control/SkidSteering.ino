// Skid steering control function
// According to the defined reference system (left-handed, X-axis toward front bogie):
// LEFT wheels (SX) rotate clockwise for forward → FORWARD = clockwise
// RIGHT wheels (DX) rotate counter-clockwise for forward → FORWARD = counter-clockwise
// BA = Front Bogie
// The 'skid' parameter determines rotation direction:
//   - skid == 1 → left motors forward, right motors backward (turn right)
//   - skid == 0 → left motors backward, right motors forward (turn left)

void SkidSteering (int vel, int tpos, int tneg, int scheda, int skid) {

  if (skid == 1) {
    // Left wheels FORWARD, Right wheels BACKWARD → turn right
    A0x6nM2->run(FORWARD);
    A0x6nM4->run(FORWARD);
    A0x6nM3->run(FORWARD);

    A0x60M1->run(BACKWARD);
    A0x60M4->run(BACKWARD);
    A0x60M3->run(BACKWARD);
  } else {
    // Left wheels BACKWARD, Right wheels FORWARD → turn left
    A0x6nM2->run(BACKWARD);
    A0x6nM4->run(BACKWARD);
    A0x6nM3->run(BACKWARD);

    A0x60M1->run(FORWARD);
    A0x60M4->run(FORWARD);
    A0x60M3->run(FORWARD);
  }

  // Gradually ramp up motor speeds
  int i;
  for (i = 20; i < vel; i++) {
    A0x6nM2->setSpeed(i / tpos);
    A0x6nM4->setSpeed(i / tpos);
    A0x6nM3->setSpeed(i / tpos);

    A0x60M1->setSpeed(i / tneg);
    A0x60M4->setSpeed(i / tneg);
    A0x60M3->setSpeed(i / tneg);
    // Optional: delay(10);
  }
}
