// QA 3: clip2 macro boundary test — preprocessor expansion verification
#include <stdio.h>

// clip2 macro from project/code/control.h
#define clip2(x, max) (((x) > (max)) ? (max) : ((x) < -(max)) ? -(max) : (x))

// Test cases
static int test(int input, int limit, int expected)
{
    int result = clip2(input, limit);
    if (result == expected)
    {
        printf("  clip2(%6d, %6d) = %6d [expected %6d] PASS\n", input, limit, result, expected);
        return 1;
    }
    else
    {
        printf("  clip2(%6d, %6d) = %6d [expected %6d] FAIL\n", input, limit, result, expected);
        return 0;
    }
}

int main(void)
{
    int pass = 0;
    int total = 0;
    int p;

    printf("QA 3 - clip2 macro boundary test:\n");

    p = test(5000,   10000,  5000);   pass += p; total++;
    p = test(15000,  10000,  10000);  pass += p; total++;
    p = test(-15000, 10000, -10000);  pass += p; total++;

    // Additional boundary tests
    p = test(10000,  10000,  10000);  pass += p; total++;  // at positive boundary
    p = test(-10000, 10000, -10000);  pass += p; total++;  // at negative boundary
    p = test(0,      10000,  0);       pass += p; total++;  // zero
    p = test(1,      10000,  1);       pass += p; total++;  // small positive
    p = test(-1,     10000, -1);       pass += p; total++;  // small negative
    p = test(9999,   10000,  9999);    pass += p; total++;  // just inside
    p = test(-9999,  10000, -9999);    pass += p; total++;  // just inside negative
    p = test(10001,  10000,  10000);   pass += p; total++;  // just outside
    p = test(-10001, 10000, -10000);   pass += p; total++;  // just outside negative

    // Edge: large values at int boundary
    p = test(32767, 32767, 32767);  pass += p; total++;
    p = test(-32768, 32767, -32767); pass += p; total++;  // max=-32767 (limit, not input)
    // Note: -(32767) = -32767, but input -32768 < -(32767) → clipped to -32767

    printf("\n  Total: %d/%d passed\n", pass, total);
    if (pass == total)
    {
        printf("  VERDICT: PASS — all clip2 boundary tests passed\n");
        return 0;
    }
    else
    {
        printf("  VERDICT: FAIL — %d tests failed\n", total - pass);
        return 1;
    }
}
