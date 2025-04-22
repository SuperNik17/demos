// Send motors backward (according to left-handed reference system with X-axis toward front bogie)
// According to this convention:
// - LEFT motors (SX) rotate counter-clockwise for BACKWARD motion (K × I = J)
// - RIGHT motors (DX) rotate clockwise for BACKWARD motion (K × I = -J)
// BA = Front Bogie

void LevogiroPos (int vel, int tpos, int tneg, int scheda) {

    int i;
    A0x6nM2->run(BACKWARD); // Front Left motor
    A0x6nM4->run(BACKWARD); // Middle Left motor
    A0x6nM3->run(BACKWARD); // Rear Left motor

    A0x60M1->run(BACKWARD); // Front Right motor
    A0x60M4->run(BACKWARD); // Middle Right motor
    A0x60M3->run(BACKWARD); // Rear Right motor

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
