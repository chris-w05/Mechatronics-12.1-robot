#include "L298NMotorDriverMega.h"

// Pins (same as your original)
const int ENA = 3; //Signal
const int IN1 = 4; //Dir 1
const int IN2 = 5; //Dir 2
const int ENB = 6;
const int IN3 = 7;
const int IN4 = 8;

L298NMotorDriverMega md(ENA,IN1,IN2,ENB,IN3,IN4); // create object for motor driver

// Allowed max magnitude for speed (match your original usage)
const int MAX_SPEED = 400; // keep same range you used previously

// --- Helper functions ---
bool isNumber(const char *s) {
  if (!s || !*s) return false;
  if (*s == '+' || *s == '-') s++;
  while (*s) {
    if (*s < '0' || *s > '9') return false;
    s++;
  }
  return true;
}

void printUsage() {
  Serial.println(F("Commands (case-insensitive). Examples:"));
  Serial.println(F("  1 ON F 200     -> Motor 1 ON, Forward, power 200"));
  Serial.println(F("  2 ON R 300     -> Motor 2 ON, Reverse, power 300"));
  Serial.println(F("  1 ON -200      -> Motor 1 ON, reverse sign encoded in number"));
  Serial.println(F("  1 OFF          -> Motor 1 OFF (brake)"));
  Serial.println(F("  ALL OFF        -> Both motors OFF"));
  Serial.println(F("  HELP           -> show this message"));
  Serial.println();
  Serial.println(F("Notes: power should be 0..400 (will be clipped)."));
  Serial.println(F("Direction tokens: F / FORWARD / +  OR  R / REVERSE / -"));
  Serial.println();
}

void applyMotorCommand(int motor, bool on, int speedValue) {
  // motor: 1 or 2 or 0 meaning ALL
  if (motor == 1 || motor == 0) {
    if (on) md.setM1Speed(speedValue);
    else md.setM1Brake(0); // brake (0 argument matches your prior usage)
  }
  if (motor == 2 || motor == 0) {
    if (on) md.setM2Speed(speedValue);
    else md.setM2Brake(0);
  }
}

void parseAndExecute(char *line) {
  // tokenize
  const int MAXTOK = 6;
  char *tok[MAXTOK];
  int tcount = 0;
  char *p = strtok(line, " ,\t\r\n");
  while (p && tcount < MAXTOK) {
    tok[tcount++] = p;
    p = strtok(NULL, " ,\t\r\n");
  }
  if (tcount == 0) return;

  // Uppercase the tokens for easy comparisons (we'll convert in-place)
  for (int i=0;i<tcount;i++) {
    for (char *c = tok[i]; *c; ++c) *c = toupper(*c);
  }

  // HELP
  if (strcmp(tok[0],"HELP")==0) {
    printUsage();
    return;
  }

  // First token: motor: "1", "2", or "ALL"
  int motor = -1; // 1 or 2 or 0(all)
  if (strcmp(tok[0],"1")==0) motor = 1;
  else if (strcmp(tok[0],"2")==0) motor = 2;
  else if (strcmp(tok[0],"ALL")==0) motor = 0;
  else {
    Serial.println(F("Unknown motor. Use 1, 2 or ALL. Type HELP for usage."));
    return;
  }

  // Need at least a second token (command)
  if (tcount < 2) {
    Serial.println(F("Missing command (ON/OFF). Type HELP for usage."));
    return;
  }

  if (strcmp(tok[1],"OFF")==0 || strcmp(tok[1],"BRAKE")==0) {
    applyMotorCommand(motor, false, 0);
    if (motor==0) Serial.println(F("Both motors: BRAKE"));
    else {
      Serial.print(F("Motor "));
      Serial.print(motor);
      Serial.println(F(": BRAKE"));
    }
    return;
  }

  if (strcmp(tok[1],"ON")==0) {
    // We expect direction and/or power.
    // Accept formats:
    // 1) motor ON DIR POWER  (e.g. 1 ON F 200)
    // 2) motor ON POWER      (e.g. 1 ON 200 or 1 ON -200)
    // 3) motor ON +200 / -200
    int sign = +1;
    int power = 0;
    bool powerFound = false;
    // Look for a numeric token among remaining tokens
    for (int i=2;i<tcount;i++) {
      // Accept tokens like "+", "-", "+200", "-200", "200"
      // Also accept direction tokens F/FORWARD/+/+1, R/REVERSE/-/-1
      if (isNumber(tok[i])) {
        power = atoi(tok[i]);
        powerFound = true;
        break;
      }
    }
    // If no explicit numeric token, maybe direction given then numeric after that
    if (!powerFound && tcount >= 4) {
      if (isNumber(tok[3])) {
        power = atoi(tok[3]);
        powerFound = true;
      }
    }

    // Determine direction token (if present)
    bool dirProvided = false;
    if (tcount >= 3) {
      const char *d = tok[2];
      if (strcmp(d,"F")==0 || strcmp(d,"FORWARD")==0 || strcmp(d,"+")==0) {
        sign = +1;
        dirProvided = true;
      } else if (strcmp(d,"R")==0 || strcmp(d,"REVERSE")==0 || strcmp(d,"-")==0) {
        sign = -1;
        dirProvided = true;
      } else {
        // if token 2 is numeric we handled it above
        if (isNumber(d)) {
          power = atoi(d);
          powerFound = true;
        }
      }
    }

    if (!powerFound) {
      Serial.println(F("Missing power value. Usage example: '1 ON F 200' or '2 ON -300'. Type HELP."));
      return;
    }

    // If numeric contained sign (like -200), that encodes direction already
    if (power < 0) {
      sign = -1;
      power = -power; // make positive part; we'll multiply by sign later
    }

    // Clip power
    if (power > MAX_SPEED) {
      Serial.print(F("Power clipped to "));
      Serial.println(MAX_SPEED);
      power = MAX_SPEED;
    }
    int finalSpeed = power * sign;

    applyMotorCommand(motor, true, finalSpeed);

    // Feedback
    if (motor==0) {
      Serial.print(F("Both motors ON: "));
      Serial.print(finalSpeed);
      Serial.println();
    } else {
      Serial.print(F("Motor "));
      Serial.print(motor);
      Serial.print(F(" ON: "));
      Serial.println(finalSpeed);
    }
    return;
  }

  Serial.println(F("Unknown command. Use ON or OFF. Type HELP for usage."));
}

void setup() {
  md.init();
  Serial.begin(9600);
  while (!Serial) { ; } // wait for Serial on boards that need it (optional)
  Serial.println(F("Motor control ready. Type HELP for usage."));
  printUsage();
}

void loop() {
  // Read a line from Serial (non-blocking style)
  if (Serial.available()) {
    static char buf[96];
    size_t len = Serial.readBytesUntil('\n', buf, sizeof(buf)-1);
    buf[len] = '\0';
    // remove possible trailing \r
    if (len > 0 && buf[len-1] == '\r') buf[len-1] = '\0';
    parseAndExecute(buf);
  }
  // nothing else to do in loop; you may add background tasks here
}
