// QA 6: servo_control_table output range verification
#include <stdio.h>

// Only TCPWM pin macros needed before servo.h (servo.h defines STEER_*)
#define TCPWM_CH10_P05_0  (0)
#define TCPWM_CH10_P05_1  (1)
#define TCPWM_CH10_P05_2  (2)
#define TCPWM_CH10_P05_3  (3)

#include "../../../project/code/servo_kinematics.h"
#include "../../../project/code/servo_kinematics.c"

int main(void)
{
    int16 p1, p4;
    int all_pass = 1;

    printf("QA 6 - servo_control_table output range verification:\n");

    servo_control_table(5.5f, 0.0f, &p1, &p4);
    printf("  p=5.5, angle=0:  p1=%d, p4=%d (expected ~500)\n", p1, p4);
    if (p1 != 500 || p4 != 500) { printf("    FAIL\n"); all_pass = 0; }
    else printf("    PASS\n");

    servo_control_table(2.75f, 0.0f, &p1, &p4);
    printf("  p=2.75, angle=0: p1=%d, p4=%d (expected ~250)\n", p1, p4);
    if (p1 != 250 || p4 != 250) { printf("    FAIL\n"); all_pass = 0; }
    else printf("    PASS\n");

    servo_control_table(11.0f, 0.0f, &p1, &p4);
    printf("  p=11.0, angle=0: p1=%d, p4=%d (expected 1000)\n", p1, p4);
    if (p1 != 1000 || p4 != 1000) { printf("    FAIL\n"); all_pass = 0; }
    else printf("    PASS\n");

    servo_control_table(0.0f, 0.0f, &p1, &p4);
    printf("  p=0, angle=0:   p1=%d, p4=%d (expected 0)\n", p1, p4);
    if (p1 != 0 || p4 != 0) { printf("    FAIL\n"); all_pass = 0; }
    else printf("    PASS\n");

    servo_control_table(-5.5f, 0.0f, &p1, &p4);
    printf("  p=-5.5, angle=0: p1=%d, p4=%d (expected -500)\n", p1, p4);
    if (p1 != -500 || p4 != -500) { printf("    FAIL\n"); all_pass = 0; }
    else printf("    PASS\n");

    servo_control_table(5.5f, 30.0f, &p1, &p4);
    printf("  p=5.5, angle=30: p1=%d, p4=%d (expected 500, angle unused)\n", p1, p4);
    if (p1 != 500 || p4 != 500) { printf("    FAIL\n"); all_pass = 0; }
    else printf("    PASS\n");

    printf("\n  VERDICT: %s\n", all_pass ? "PASS" : "FAIL");
    return all_pass ? 0 : 1;
}
