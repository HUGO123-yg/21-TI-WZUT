// QA 4: euler_angle struct compilation check — verify euler.h compiles
// Compile with: cc -fsyntax-only -I stubs/ -I ../../project/code/ test_euler.c

#include "euler.h"

// Verify the struct has the expected members and alignment
void verify_euler(void)
{
    euler_angle_struct e;
    e.pitch = 15.0f;
    e.roll  = -5.0f;
    e.yaw   = 90.0f;

    // Use volatile to prevent optimization from removing the assignment
    volatile float p = e.pitch;
    volatile float r = e.roll;
    volatile float y = e.yaw;
    (void)p; (void)r; (void)y;
}

// Verify extern symbol resolves (link-time check only, syntax is fine)
void verify_extern(void)
{
    // euler_angle is extern — declared in euler.h, defined in euler.c
    // Only syntax check here, so just reference it
    volatile float p = euler_angle.pitch;
    (void)p;
}

int main(void)
{
    verify_euler();
    verify_extern();
    return 0;
}
