/*
MIT License

Copyright (c) 2024 controllercustom@myyahoo.com

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 */
// Reference: https://www.psdevwiki.com/ps4/DS4-USB
typedef struct __attribute__((packed)) {
  uint8_t reportid;       // 1
  uint8_t leftx;          // Left thumb joystick
  uint8_t lefty;
  uint8_t rightx;         // Right thumb joystick
  uint8_t righty;

  uint8_t dpad:4;         // Direction pad
  uint8_t square:1;
  uint8_t cross:1;
  uint8_t circle:1;
  uint8_t triangle:1;

  uint8_t l1:1;
  uint8_t r1:1;
  uint8_t l2:1;           // Also are reported as analog values
  uint8_t r2:1;           // Also are reported as analog values
  uint8_t share:1;
  uint8_t options:1;
  uint8_t l3:1;           // the joysticks are also buttons
  uint8_t r3:1;

  uint8_t ps_logo:1;
  uint8_t touchpad:1;     // the touchpad is also a button
  uint8_t reportCount:6;

  uint8_t l2_analog;
  uint8_t r2_analog;

  uint16_t timestamp;     // ?
  uint8_t battery_level;
  uint16_t gyrox;
  uint16_t gyroy;
  uint16_t gyroz;
  uint16_t accelx;
  uint16_t accely;
  uint16_t accelz;
  uint8_t unknown1[5];
  uint8_t headset;
  uint8_t tbd[33];
} DS4Report_t;

typedef struct __attribute__((packed)) {
  uint8_t reportid;       // 1
  uint8_t leftx;          // Left thumb joystick
  uint8_t lefty;
  uint8_t leftz;
  uint8_t rightz;
  uint8_t rightx;         // Right thumb joystick
  uint8_t righty;

  uint8_t reportCount;

  uint8_t dpad:4;         // Direction pad
  uint8_t square:1;
  uint8_t cross:1;
  uint8_t circle:1;
  uint8_t triangle:1;

  uint8_t l1:1;
  uint8_t r1:1;
  uint8_t l2:1;           // Also are reported as analog values
  uint8_t r2:1;           // Also are reported as analog values
  uint8_t share:1;
  uint8_t options:1;
  uint8_t l3:1;           // the joysticks are also buttons
  uint8_t r3:1;

  uint16_t ps_logo:1;
  uint16_t touchpad:1;     // the touchpad is also a button
  uint16_t button15:1;     // ??
  uint16_t vendor2:13;

  uint8_t vendor3[52];
} DS5Report_t;

typedef struct {
  DS4Report_t report;
  const uint16_t USB_VID = 0x054c;
  const uint16_t USB_PID = 0x09cc;
  const uint16_t USB_PID2 = 0x05C4; // Alt
  uint8_t dev_addr;
  uint8_t instance;
  uint8_t report_len;
  bool connected = false;
  bool available = false;
  bool debug = false;
} DS4_state_t;

typedef struct {
  DS5Report_t report;
  const uint16_t USB_VID = 0x054c;
  const uint16_t USB_PID = 0x0e5f; // Access Controller for PS5
  const uint16_t USB_PID2 = 0x0CE6; // Dual Sense for PS5
  uint8_t dev_addr;
  uint8_t instance;
  uint8_t report_len;
  bool connected = false;
  bool available = false;
  bool debug = false;
} DS5_state_t;

void print_DS4_controls(volatile DS4_state_t *DS4) {
  DBG_printf("Count:%d,", DS4->report.reportCount);
  DBG_printf("LX:%d,LY:%d,RX:%d,RY:%d,dpad:%d,",
      DS4->report.leftx, DS4->report.lefty,
      DS4->report.rightx, DS4->report.righty, DS4->report.dpad);
  DBG_printf("L2:%d,R2:%d,", DS4->report.l2_analog, DS4->report.r2_analog);
  if (DS4->report.square) DBG_print("Square,");
  if (DS4->report.cross) DBG_print("Cross,");
  if (DS4->report.circle) DBG_print("Circle,");
  if (DS4->report.triangle) DBG_print("Triangle,");
  if (DS4->report.l1) DBG_print("L1,");
  if (DS4->report.r1) DBG_print("R1,");
  if (DS4->report.l3) DBG_print("L3,");
  if (DS4->report.r3) DBG_print("R3,");
  if (DS4->report.options) DBG_print("Option,");
  if (DS4->report.share) DBG_print("Share,");
  if (DS4->report.ps_logo) DBG_print("PS logo,");
  if (DS4->report.touchpad) DBG_print("Touchpad");
  DBG_println();
}

void print_DS5_controls(volatile DS5_state_t *DS5) {
  DBG_printf("Count:%d,", DS5->report.reportCount);
  DBG_printf("LX:%d,LY:%d,LZ:%d,RX:%d,RY:%d,RZ:%d,dpad:%d,",
      DS5->report.leftx, DS5->report.lefty, DS5->report.leftz,
      DS5->report.rightx, DS5->report.righty, DS5->report.rightz,
      DS5->report.dpad);
  if (DS5->report.square) DBG_print("Square,");
  if (DS5->report.cross) DBG_print("Cross,");
  if (DS5->report.circle) DBG_print("Circle,");
  if (DS5->report.triangle) DBG_print("Triangle,");
  if (DS5->report.l1) DBG_print("L1,");
  if (DS5->report.r1) DBG_print("R1,");
  if (DS5->report.l3) DBG_print("L3,");
  if (DS5->report.r3) DBG_print("R3,");
  if (DS5->report.options) DBG_print("Option,");
  if (DS5->report.share) DBG_print("Share,");
  if (DS5->report.ps_logo) DBG_print("PS logo,");
  if (DS5->report.touchpad) DBG_print("Touchpad");
  DBG_println();
}
