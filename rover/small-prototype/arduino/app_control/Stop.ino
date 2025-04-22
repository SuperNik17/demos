// Stop all left-side motors by releasing their control signals
void LeftWheelsStop() {
    A0x6nM2->run(RELEASE); // Front Left motor (BA = Front Bogie)
    A0x6nM4->run(RELEASE); // Middle Left motor
    A0x6nM3->run(RELEASE); // Rear Left motor
}

// Stop all right-side motors by releasing their control signals
void RightWheelsStop() {
    A0x60M1->run(RELEASE); // Front Right motor
    A0x60M4->run(RELEASE); // Middle Right motor
    A0x60M3->run(RELEASE); // Rear Right motor
}
