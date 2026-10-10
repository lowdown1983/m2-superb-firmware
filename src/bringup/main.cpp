// BRINGUP v5 — bisect ModeManager::begin() step by step.
//
// v4 (which calls modeManager.begin()) prints nothing; v3 (which doesn't)
// ticks fine. This replicates begin()'s steps ONE AT A TIME with a print
// after each. The last print seen = the killer. Runs once, then ticks.
#include <Arduino.h>
#include <due_can.h>

static void busyWait(volatile uint32_t n)
{
    while (n--) { __asm__ volatile("nop"); }
}

static bool tested = false;

void setup()
{
    SerialUSB.begin(115200);
}

void loop()
{
    if (!tested) {
        tested = true;
        SerialUSB.println(F("V5-T1: pinMode(22)"));
        pinMode(22, INPUT_PULLUP);
        SerialUSB.println(F("V5-T2: T1 done"));

        SerialUSB.println(F("V5-T3: Can0.begin(500000,255)"));
        Can0.begin(500000, 255);
        SerialUSB.println(F("V5-T4: Can0.begin returned"));

        SerialUSB.println(F("V5-T5: Can1.begin(500000,255)"));
        Can1.begin(500000, 255);
        SerialUSB.println(F("V5-T6: Can1.begin returned"));

        SerialUSB.println(F("V5-T7: setRXFilter x12"));
        for (int i = 0; i < 8; i++) Can0.setRXFilter(i, 0, 0, true);
        for (int i = 0; i < 4; i++) Can1.setRXFilter(i, 0, 0, true);
        SerialUSB.println(F("V5-T8: filters done"));

        SerialUSB.println(F("V5-ALL-DONE"));
    }
    SerialUSB.print(F("V5-C: tick millis="));
    SerialUSB.println(millis());
    busyWait(30000000UL);
}
