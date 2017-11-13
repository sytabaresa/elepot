#ifndef LOOKUP_TABLES_H
#define LOOKUP_TABLES_H

#include <Arduino.h>

// D1(t) = 0.8333*|sin(2*pi*60*t)|
static const uint8_t D1[166] PROGMEM = {
6,13,19,25,31,37,43,49,
55,61,67,73,78,84,89,94,
99,104,109,114,118,123,127,131,
135,138,141,145,148,150,153,155,
157,159,161,162,163,164,165,166,
166,166,166,165,164,163,162,161,
159,157,155,153,150,148,145,141,
138,135,131,127,123,118,114,109,
104,99,94,89,84,78,73,67,
61,55,49,43,37,31,25,19,
13,6,0,6,13,19,25,31,37,
43,49,55,61,67,73,78,84,
89,94,99,104,109,114,118,
123,127,131,135,138,141,145,148,
150,153,155,157,159,161,162,163,
164,165,166,166,166,166,165,164,
163,162,161,159,157,155,153,150,
148,145,141,138,135,131,127,123,
118,114,109,104,99,94,89,84,
78,73,67,61,55,49,43,37,
31,25,19,13,6,0
};

// D2(t) = (5 + 5*sin(2*pi*60*t)/12
static const uint8_t D2[166] PROGMEM = {
86,89,92,95,99,102,105,108,
111,114,116,119,122,125,128,130,
133,135,138,140,142,144,146,148,
150,152,154,155,157,158,159,161,
162,163,163,164,165,165,165,166,
166,166,166,165,165,165,164,163,
163,162,161,159,158,157,155,154,
152,150,148,146,144,142,140,138,
135,133,130,128,125,122,119,116,
114,111,108,105,102,99,95,92,89,
86,83,80,77,74,70,67,64,
61,58,55,52,49,47,44,41,
38,36,33,31,28,26,24,22,
20,18,16,14,12,11,9,8,
6,5,4,3,2,2,1,1,
0,0,0,0,0,0,1,1,
2,2,3,4,5,6,8,9,
11,12,14,16,18,20,22,24,
26,28,31,33,36,38,41,44,
47,49,52,55,58,61,64,67,
70,74,77,80,83
};


#endif
