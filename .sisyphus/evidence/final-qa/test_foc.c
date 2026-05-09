// QA 5: foc_set_duty compilation check — verify foc.h (stub) compiles
// Compile with: cc -fsyntax-only -I stubs/ test_foc.c

#include "foc.h"

void verify_foc(void)
{
    int16 l = 100;
    int16 r = -100;
    foc_set_duty(l, r);

    volatile float la = motor_value.receive_left_angle_data;
    volatile float ra = motor_value.receive_right_angle_data;
    (void)la; (void)ra;
}

int main(void)
{
    verify_foc();
    return 0;
}
