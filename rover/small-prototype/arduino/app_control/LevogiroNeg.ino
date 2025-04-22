// Send motors forward (according to left-handed reference system with X-axis toward front bogie)
// According to this convention:
// - LEFT motors (SX) rotate clockwise for FORWARD motion (K × I = J)
// - RIGHT motors (DX) rotate counter-clockwise for FORWARD motion (K × I = -J)
// BA = Front Bogie

void LevogiroNeg (int vel, int tpos, int tneg, int scheda) {
    int i;
    A0x6nM2->run(FORWARD); // Front Left motor
    A0x6nM4->run(FORWARD); // Middle Left motor
    A0x6nM3->run(FORWARD); // Rear Left motor
                           
    A0x60M1->run(FORWARD); // Front Right motor
    A0x60M4->run(FORWARD); // Middle Right motor
    A0x60M3->run(FORWARD); // Rear Right motor

    // Gradually ramp up motor speeds
    for (i = 20; i < vel; i++) {
      A0x6nM2->setSpeed(i / tpos);
      A0x6nM4->setSpeed(i / tpos);
      A0x6nM3->setSpeed(i / tpos);

      A0x60M1->setSpeed(i / tneg);
      A0x60M4->setSpeed(i / tneg);
      A0x60M3->setSpeed(i / tneg);
      // Optional delay for smoother ramp-up
      // delay(10);
    }
}
