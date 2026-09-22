#include "protocol.h"
#include "protocol_internal.h"
#include "config_store.h"
#include "motion_api.h"
#include "board.h"
#include "servo_pwm.h"
#include "config_defaults.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

static std::string g_out;
static std::string g_dbg;

static void io_write(const char *data, size_t n, void *ctx) {
  (void)ctx;
  g_out.append(data, n);
}

static void io_debug(const char *data, size_t n, void *ctx) {
  (void)ctx;
  g_dbg.append(data, n);
}

static void feed(const char *s) {
  for (const char *p = s; *p; ++p) {
    protocol_feed_byte((uint8_t)*p);
  }
}

static void feed_uart(const char *s) {
  for (const char *p = s; *p; ++p) {
    protocol_feed_uart_byte((uint8_t)*p);
  }
}

static int g_fail;

static void expect_contains(const char *name, const char *needle) {
  if (g_out.find(needle) == std::string::npos) {
    std::fprintf(stderr, "FAIL %s: expected '%s' in:\n%s\n", name, needle, g_out.c_str());
    ++g_fail;
  } else {
    std::printf("OK   %s\n", name);
  }
}

static void expect_empty(const char *name) {
  if (!g_out.empty()) {
    std::fprintf(stderr, "FAIL %s: expected silence, got:\n%s\n", name, g_out.c_str());
    ++g_fail;
  } else {
    std::printf("OK   %s\n", name);
  }
}

static void expect_true(const char *name, bool ok) {
  if (!ok) {
    std::fprintf(stderr, "FAIL %s\n", name);
    ++g_fail;
  } else {
    std::printf("OK   %s\n", name);
  }
}

static void expect_not_contains(const char *name, const char *needle) {
  if (g_out.find(needle) != std::string::npos) {
    std::fprintf(stderr, "FAIL %s: unexpected '%s' in:\n%s\n", name, needle, g_out.c_str());
    ++g_fail;
  } else {
    std::printf("OK   %s\n", name);
  }
}

static void expect_dbg_contains(const char *name, const char *needle) {
  if (g_dbg.find(needle) == std::string::npos) {
    std::fprintf(stderr, "FAIL %s: expected '%s' in debug:\n%s\n", name, needle,
                 g_dbg.c_str());
    ++g_fail;
  } else {
    std::printf("OK   %s\n", name);
  }
}

static void expect_dbg_empty(const char *name) {
  if (!g_dbg.empty()) {
    std::fprintf(stderr, "FAIL %s: expected empty debug, got:\n%s\n", name, g_dbg.c_str());
    ++g_fail;
  } else {
    std::printf("OK   %s\n", name);
  }
}

static void reset_out(void) {
  g_out.clear();
  g_dbg.clear();
}

static void expect_format(const char *name, float v, const char *want) {
  char got[32];
  protocol_format_num(got, sizeof(got), v);
  if (std::strcmp(got, want) != 0) {
    std::fprintf(stderr, "FAIL %s: format %s, got '%s' want '%s'\n", name, name, got, want);
    ++g_fail;
  } else {
    std::printf("OK   %s\n", name);
  }
}

int main(void) {
  expect_format("fmt 1.2345", 1.2345f, "1.23");
  expect_format("fmt 123.45", 123.45f, "123.45");
  expect_format("fmt 12345", 12345.f, "12345");
  expect_format("fmt 1234500", 1234500.f, "1234500");
  expect_format("fmt 0.12345", 0.12345f, "0.123");
  expect_format("fmt 0.0012345", 0.0012345f, "0.00123");
  expect_format("fmt 0.000012345", 0.000012345f, "0.0000123");
  expect_format("fmt 1000", 1000.f, "1000");
  expect_format("fmt 1", 1.f, "1");
  expect_format("fmt 0.1", 0.1f, "0.1");
  expect_format("fmt 0.001", 0.001f, "0.001");
  expect_format("fmt 0.1239", 0.1239f, "0.124");
  expect_format("fmt -0.1239", -0.1239f, "-0.124");
  expect_format("fmt 9.996", 9.996f, "10");
  expect_format("fmt 9.9999999999", 9.9999999999f, "10");
  expect_format("fmt -1.2345", -1.2345f, "-1.23");
  expect_format("fmt 0", 0.f, "0");
  expect_format("fmt nan", NAN, "-");

  ProtocolIo io = {io_write, io_debug, nullptr};
  protocol_init(io);
  protocol_send_banner();
  expect_contains("startup banner", "# MC V1 -");
  expect_contains("startup help hint", "['?' for help]");

  reset_out();
  feed("VH\n");
  expect_contains("VH reprints banner", "# MC V1 -");

  reset_out();
  feed("\n");
  expect_contains("empty line re-banners", "# MC V1 -");
  reset_out();
  feed("\r\n");
  expect_contains("CRLF re-banners", "# MC V1 -");

  reset_out();
  feed("GE\n");
  expect_contains("GE disabled", "GE:0");
  reset_out();
  feed("IR\n");
  expect_contains("IR not ready when disabled", "IR:0");

  reset_out();
  feed("SE\n");
  expect_empty("SE bare toggle on");
  reset_out();
  feed("GE\n");
  expect_contains("GE enabled after bare SE", "GE:1");
  reset_out();
  feed("SE\n");
  expect_empty("SE bare toggle off");
  reset_out();
  feed("GE\n");
  expect_contains("GE disabled after second bare SE", "GE:0");
  reset_out();
  feed("SE 1\n");
  expect_empty("SE 1 explicit");
  reset_out();
  feed("GE\n");
  expect_contains("GE enabled", "GE:1");
  reset_out();
  feed("IR\n");
  expect_contains("IR ready when idle enabled", "IR:1");
  reset_out();
  feed("IE\n");
  expect_contains("IE IsError clear", "IE:0");

  reset_out();
  protocol_feed_byte('#');
  expect_contains("realtime # compact", "#I ");
  expect_not_contains("realtime # 1-axis has no axis separator", " | ");
  expect_not_contains("realtime # not old format", "<I|P:");

  reset_out();
  protocol_feed_byte('?');
  expect_empty("? is not realtime");
  feed("\n");
  expect_contains("? newline is help", "Cmd  Description");

  board_camera_ctrl_inject(true);
  reset_out();
  protocol_feed_byte('#');
  expect_contains("CAMERA_CTRL low overlays T once", "#T ");
  reset_out();
  protocol_feed_byte('#');
  expect_contains("CAMERA_CTRL still low uses motion letter", "#I ");
  expect_not_contains("CAMERA_CTRL no second T while held", "#T ");
  board_camera_ctrl_inject(false);
  board_camera_ctrl_inject(true);
  reset_out();
  protocol_feed_byte('#');
  expect_contains("CAMERA_CTRL new press overlays T again", "#T ");
  board_camera_ctrl_inject(false);

  reset_out();
  feed("CS init_speed 25\n");
  expect_empty("CS silent");
  reset_out();
  feed("CG init_speed\n");
  expect_contains("CG init_speed", "CG:init_speed=25");
  reset_out();
  feed("GS\n");
  expect_contains("GS after CS", "GS:25");

  reset_out();
  feed("CR\n");
  expect_empty("CR ConfigReset silent");
  reset_out();
  feed("CG init_speed\n");
  expect_contains("CG after CR default", "CG:init_speed=50");
  reset_out();
  feed("CG max_speed_2\n");
  expect_contains("CG max_speed_2 default", "CG:max_speed_2=100");
  reset_out();
  feed("CG max_accel_2\n");
  expect_contains("CG max_accel_2 default", "CG:max_accel_2=300");
  reset_out();
  feed("GS\n");
  expect_contains("GS after CR default", "GS:50");
  reset_out();
  feed("ConfigReset\n");
  expect_contains("ConfigReset long gone", "!E:parse");

  reset_out();
  feed("CS max_accel_1 300\n");
  expect_empty("CS max_accel_1 silent");
  reset_out();
  feed("CG max_accel_1\n");
  expect_contains("CG max_accel_1", "CG:max_accel_1=300");

  reset_out();
  feed("CS DRV_STEP_1_active 0\n");
  expect_empty("CS DRV_STEP_1_active 0");
  reset_out();
  feed("CG DRV_STEP_1_active\n");
  expect_contains("CG DRV_STEP_1_active", "CG:DRV_STEP_1_active=0");
  reset_out();
  feed("CS DRV_STEP_1_active 1\n");
  expect_empty("CS DRV_STEP_1_active restore");

  reset_out();
  feed("CS SW_LIMIT_R_3_use 1\n");
  expect_empty("CS SW_LIMIT_R_3_use 1");
  reset_out();
  feed("CG SW_LIMIT_R_3_use\n");
  expect_contains("CG SW_LIMIT_R_3_use", "CG:SW_LIMIT_R_3_use=1");
  reset_out();
  feed("CS SW_LIMIT_R_3_use 0\n");
  expect_empty("CS SW_LIMIT_R_3_use restore");

  reset_out();
  feed("CS DRV_ENABLE_active 1\n");
  expect_empty("CS DRV_ENABLE_active 1");
  reset_out();
  feed("CG DRV_ENABLE_active\n");
  expect_contains("CG DRV_ENABLE_active", "CG:DRV_ENABLE_active=1");
  reset_out();
  feed("CS DRV_ENABLE_active 0\n");
  expect_empty("CS DRV_ENABLE_active restore");

  reset_out();
  feed("CS DRV_STEP_active 0\n");
  expect_contains("old DRV_STEP_active rejected", "!E:cfg bad key/value");
  reset_out();
  feed("CS SW_LIMIT_R_use_3 1\n");
  expect_contains("old SW_LIMIT_R_use_3 rejected", "!E:cfg bad key/value");
  reset_out();
  feed("CS home_mode 1\n");
  expect_contains("old home_mode rejected", "!E:cfg bad key/value");
  reset_out();
  feed("CS axis2_use 1\n");
  expect_contains("old axis2_use rejected", "!E:cfg bad key/value");
  reset_out();
  feed("CS DRV_EN_1_active 0\n");
  expect_contains("old DRV_EN_1_active rejected", "!E:cfg bad key/value");
  reset_out();
  feed("CS EXT_0_active 1\n");
  expect_contains("old EXT_0_active rejected", "!E:cfg bad key/value");

  reset_out();
  feed("SS 50\n");
  expect_empty("SS under max");
  reset_out();
  feed("SS 999\n");
  expect_contains("SS over max_speed", "!E:limit SS > max_speed");
  reset_out();
  feed("GS\n");
  expect_contains("GS unchanged after SS reject", "GS:50");

  reset_out();
  feed("SA 200\n");
  expect_empty("SA under max");
  reset_out();
  feed("SA 999\n");
  expect_contains("SA over max_accel", "!E:limit SA > max_accel");
  reset_out();
  feed("GA\n");
  expect_contains("GA unchanged after SA reject", "GA:200 200");

  reset_out();
  feed("SA 200 50\n");
  expect_empty("SA two-arg silent");
  reset_out();
  feed("GA\n");
  expect_contains("GA accel decel", "GA:200 50");
  reset_out();
  feed("SA 200 _\n");
  expect_contains("SA skip rejected", "!E:parse SA args");
  reset_out();
  feed("GA\n");
  expect_contains("GA after skip reject", "GA:200 50");
  reset_out();
  feed("SA 200 999\n");
  expect_contains("SA 2nd over max", "!E:limit SA > max_accel");
  reset_out();
  feed("GA\n");
  expect_contains("GA after 2nd reject", "GA:200 50");
  reset_out();
  feed("SA 200\n");
  expect_empty("SA one-arg sets both");
  reset_out();
  feed("GA\n");
  expect_contains("GA after one-arg both", "GA:200 200");

  reset_out();
  feed("SD 4\n");
  expect_empty("SD silent");
  reset_out();
  feed("GD\n");
  expect_contains("GD after SD", "GD:4");
  reset_out();
  feed("SD\n");
  expect_empty("SD bare silent");
  reset_out();
  feed("GD\n");
  expect_contains("GD after bare SD", "GD:3");

  reset_out();
  feed("SS50\nSA200\nMT100;WM\n");
  expect_empty("MT silent before wait done");
  reset_out();
  feed("IW\n");
  expect_contains("IW during WM", "IW:1");
  reset_out();
  feed("IR\n");
  expect_contains("IR busy during WM", "IR:0");
  for (int i = 0; i < 200; ++i) {
    protocol_poll(20);
    reset_out();
    feed("IM\n");
    if (g_out.find("IM:0") != std::string::npos) {
      break;
    }
    reset_out();
  }
  expect_contains("WM finished (IM:0)", "IM:0");
  expect_not_contains("no OK after WM", "OK");
  reset_out();
  feed("IW\n");
  expect_contains("IW idle", "IW:0");

  reset_out();
  feed("IP\n");
  expect_contains("IP after move", "IP:100");

  reset_out();
  feed("VA\n");
  expect_contains("VA about", "VA:Slider Motion Controller V1.0 by Jochen Krapf");
  reset_out();
  feed("VF\n");
  expect_contains("VF", "VF:1.0");
  reset_out();
  feed("VP\n");
  expect_contains("VP", "VP:1");

  /* Timeout cancels remainder of chain */
  reset_out();
  feed("MT0;WM 0.01;SS 12\n");
  for (int i = 0; i < 5; ++i) {
    protocol_poll(5);
  }
  expect_contains("WM timeout", "!E:timeout");
  reset_out();
  feed("GS\n");
  /* SS 12 must not have run — still prior session speed (50 from SS50, or 25 from CS) */
  expect_not_contains("timeout canceled SS", "GS:12");

  /* Halt cancels wait + following chain commands */
  reset_out();
  feed("SS 50\n");
  expect_empty("SS before halt chain");
  reset_out();
  feed("MT200;WM;SS 12\n");
  protocol_poll(5);
  reset_out();
  feed("ME\n");
  expect_empty("HT silent");
  for (int i = 0; i < 50; ++i) {
    protocol_poll(20);
  }
  reset_out();
  feed("GS\n");
  expect_not_contains("HT canceled SS", "GS:12");
  expect_contains("HT kept prior SS", "GS:50");
  reset_out();
  feed("IW\n");
  expect_contains("IW after HT", "IW:0");

  /* Bare W defaults to 1 s then continues chain */
  reset_out();
  feed("WT;SS 33\n");
  expect_empty("WT silent start");
  reset_out();
  feed("IW\n");
  expect_contains("IW during WT", "IW:1");
  for (int i = 0; i < 20; ++i) {
    protocol_poll(50); /* 1.0 s */
  }
  reset_out();
  feed("GS\n");
  expect_contains("WT then SS ran", "GS:33");

  /* WP / WC / WN / BE */
  reset_out();
  feed("WP\n");
  expect_contains("WP bare parse", "!E:parse");
  reset_out();
  feed("WP 100 50 1\n");
  expect_contains("WP extra token parse", "!E:parse");

  reset_out();
  feed("SS 50\n");
  expect_empty("SS 50 before idle WP");
  reset_out();
  feed("WP 50; SS 12\n");
  expect_empty("idle WP silent");
  reset_out();
  feed("IW\n");
  expect_contains("IW idle after WP", "IW:0");
  reset_out();
  feed("GS\n");
  expect_contains("idle WP then SS immediately", "GS:12");

  reset_out();
  feed("SS 50\nSA 200\n");
  expect_empty("SS SA for WP move");
  reset_out();
  feed("MT 0; WM\n");
  for (int i = 0; i < 50; ++i) {
    protocol_poll(20);
  }
  reset_out();
  feed("MT 100; WP 40; SS 8\n");
  expect_empty("MT WP SS silent start");
  reset_out();
  feed("IW\n");
  expect_contains("IW during WP", "IW:1");
  {
    bool ran = false;
    for (int i = 0; i < 200; ++i) {
      protocol_poll(20);
      reset_out();
      feed("GS\n");
      if (g_out.find("GS:8") != std::string::npos) {
        ran = true;
        break;
      }
    }
    expect_true("WP mid-move then SS", ran);
  }

  reset_out();
  feed("MT 0; WM\n");
  for (int i = 0; i < 80; ++i) {
    protocol_poll(20);
  }
  reset_out();
  feed("SS 50\n");
  reset_out();
  feed("MT 100; WP 80 0.01; SS 9\n");
  for (int i = 0; i < 8; ++i) {
    protocol_poll(5);
  }
  expect_contains("WP timeout", "!E:timeout");
  reset_out();
  feed("GS\n");
  expect_not_contains("WP timeout canceled SS", "GS:9");

  reset_out();
  feed("ME\nSE 1\nSS 50\nSA 200\n");
  expect_empty("re-enable after HT");
  reset_out();
  feed("MT 0; WM\n");
  for (int i = 0; i < 80; ++i) {
    protocol_poll(20);
  }
  reset_out();
  feed("MT 100; WC; SA 5\n");
  expect_empty("MT WC SA silent start");
  reset_out();
  feed("IW\n");
  expect_contains("IW during WC", "IW:1");
  {
    bool ran = false;
    for (int i = 0; i < 200; ++i) {
      protocol_poll(20);
      reset_out();
      feed("GA\n");
      if (g_out.find("GA:5") != std::string::npos) {
        ran = true;
        break;
      }
    }
    expect_true("WC then live SA", ran);
  }

  reset_out();
  feed("MS\n");
  expect_empty("MS idle before WN");
  reset_out();
  feed("WN; SS 44\n");
  expect_empty("idle WN silent");
  reset_out();
  feed("IW\n");
  expect_contains("IW idle after WN", "IW:0");
  reset_out();
  feed("GS\n");
  expect_contains("idle WN then SS immediately", "GS:44");

  reset_out();
  feed("SS 50\nSA 200\nMT 0; WM\n");
  for (int i = 0; i < 80; ++i) {
    protocol_poll(20);
  }
  reset_out();
  feed("MT 100; WC; WN; SS 6\n");
  {
    bool ran = false;
    for (int i = 0; i < 400; ++i) {
      protocol_poll(20);
      reset_out();
      feed("GS\n");
      if (g_out.find("GS:6") != std::string::npos) {
        ran = true;
        break;
      }
    }
    expect_true("WN after cruise then SS", ran);
  }

  reset_out();
  feed("SS 50\nSA 200\nMS\nSP 0\n");
  expect_empty("park origin before HT WP");
  reset_out();
  feed("MT 100; WP 40; SS 11\n");
  protocol_poll(5);
  reset_out();
  feed("ME\n");
  expect_empty("HT during WP silent");
  for (int i = 0; i < 50; ++i) {
    protocol_poll(20);
  }
  reset_out();
  feed("GS\n");
  expect_not_contains("HT canceled WP SS", "GS:11");
  reset_out();
  feed("IW\n");
  expect_contains("IW after HT WP", "IW:0");

  reset_out();
  feed("SE 1\nBE; SS 13\n");
  expect_empty("BE silent not a wait");
  reset_out();
  feed("IW\n");
  expect_contains("IW after BE", "IW:0");
  reset_out();
  feed("GS\n");
  expect_contains("BE then SS immediately", "GS:13");
  reset_out();
  feed("BE foo\n");
  expect_contains("BE junk arg parse", "!E:parse");
  reset_out();
  feed("BE 50 1\n");
  expect_contains("BE extra arg parse", "!E:parse");
  reset_out();
  feed("CG BUZZER_use\n");
  expect_contains("CG BUZZER_use default", "CG:BUZZER_use=0");
  reset_out();
  feed("CS BUZZER_use 1\n");
  expect_empty("CS BUZZER_use 1");
  reset_out();
  feed("CG BUZZER_use\n");
  expect_contains("CG BUZZER_use on", "CG:BUZZER_use=1");
  reset_out();
  feed("CS BUZZER_use 0\n");
  expect_empty("CS BUZZER_use 0 restore");

  reset_out();
  feed("SE 1\nSS 50\nSA 200\n");
  expect_empty("restore SS SA after wait tests");

  reset_out();
  feed("MT50\n");
  protocol_feed_byte('!');
  protocol_poll(5);
  feed("IM\n");
  expect_contains("! stop", "IM:0");

  reset_out();
  feed("SV 1\n");
  protocol_poll(400);
  expect_contains("verbose push", "#");

  reset_out();
  feed("HL\n");
  expect_contains("HL header", "Cmd  Description");
  expect_contains("HL Set Speed", "SS    Set Speed");
  expect_contains("HL Set Debug", "SD    Set Debug");
  expect_contains("HL Set Left", "SL    Set Left");
  expect_contains("HL Set Position", "SP    Set Position");
  expect_contains("HL Get Left", "GL    Get Left");
  expect_contains("HL Is Ready", "IR    Is Ready");
  expect_contains("HL Move Joy", "MJ    Move Joy");
  expect_contains("HL Wait Pos", "WP    Wait Pos");
  expect_contains("HL Wait Cruise", "WC    Wait Cruise");
  expect_contains("HL Wait Not", "WN    Wait Not");
  expect_contains("HL Beep", "BE    Beep");
  expect_contains("HL Halt", "ME    Move E-Stop");
  expect_contains("HL Camera", "CT    Camera Trigger");
  expect_contains("HL self", "HL/?");
  expect_contains("HL comment footer", "/ comments to end of line");
  expect_contains("HL now footer", "Now: # status");
  reset_out();
  feed("$\n");
  expect_contains("$ gone", "!E:parse");
  reset_out();
  feed("?\n");
  expect_contains("? help alias", "Version Protocol");
  reset_out();
  feed("Help\n");
  expect_contains("Help long gone", "!E:parse");
  reset_out();
  feed("H\n");
  expect_contains("H gone", "!E:parse");
  reset_out();
  feed("HT\n");
  expect_contains("HT gone", "!E:parse");
  reset_out();
  feed("ME\n");
  expect_empty("ME Halt silent");

  /* / comments */
  reset_out();
  feed("SS 41 / note; SS 99\n");
  expect_empty("comment cuts rest of line");
  reset_out();
  feed("GS\n");
  expect_contains("SS with comment ran", "GS:41");
  expect_not_contains("comment dropped SS 99", "GS:99");
  reset_out();
  feed("/ only comment\n");
  expect_empty("comment-only no banner");
  reset_out();
  feed("SS 50 / restore cruise\n");
  expect_empty("restore SS after comment tests");

  /* CT / BE pulse parse; CT does not busy-out motion */
  reset_out();
  feed("SV 0\n");
  expect_empty("SV 0 before CT tests");
  reset_out();
  feed("CT 0\n");
  expect_contains("CT 0 parse", "!E:parse");
  reset_out();
  feed("CT 60001\n");
  expect_contains("CT 60001 parse", "!E:parse");
  reset_out();
  feed("BE 0\n");
  expect_contains("BE 0 parse", "!E:parse");
  reset_out();
  feed("BE 1001\n");
  expect_contains("BE 1001 parse", "!E:parse");
  reset_out();
  feed("BE 50\n");
  expect_empty("BE 50 silent");
  reset_out();
  feed("BE\n");
  expect_empty("BE bare silent");
  reset_out();
  feed("CT\n");
  expect_empty("CT verbose off silent");
  expect_not_contains("CT verbose off no T", "#T");
  reset_out();
  feed("IM\n");
  expect_contains("CT then IM accepted", "IM:");
  reset_out();
  feed("CT 40\n");
  expect_empty("CT 40 silent");
  reset_out();
  protocol_feed_byte('#');
  expect_contains("# during CT pulse is T", "#T ");
  reset_out();
  feed("SE 1\n");
  expect_empty("SE 1 before MT during CT");
  reset_out();
  feed("MT 10\n");
  expect_empty("MT during CT not busy");
  protocol_poll(40);
  reset_out();
  protocol_feed_byte('#');
  expect_not_contains("after CT pulse no T", "#T ");
  reset_out();
  feed("SV 1\n");
  expect_empty("SV 1 silent");
  reset_out();
  feed("CT 20\n");
  expect_contains("CT verbose on sends T", "#T ");
  reset_out();
  feed("SV 0\n");
  expect_empty("SV 0 silent");
  protocol_poll(20);
  reset_out();
  feed("ME\n");
  expect_empty("ME after CT tests");
  reset_out();
  feed("SE 1\nSP 0\n");
  expect_empty("park after CT tests");

  reset_out();
  feed("EO10\n");
  expect_empty("EO10 glued = EO1 0");
  reset_out();
  feed("EO1\n");
  expect_empty("EO1 bare toggles on");
  reset_out();
  feed("EO2 1\n");
  expect_empty("EO2 on");
  reset_out();
  feed("EO3\n");
  expect_empty("EO3 bare toggles");
  reset_out();
  feed("EO0 1\n");
  expect_contains("EO0 rejected", "!E:parse");

  reset_out();
  feed("ZZZ\n");
  expect_contains("parse error", "!E:parse");
  reset_out();
  feed("X0\n");
  expect_contains("X0 gone", "!E:parse");
  reset_out();
  feed("Z\n");
  expect_contains("Z gone", "!E:parse");
  reset_out();
  feed("ML\n");
  expect_contains("ML gone", "!E:parse");
  reset_out();
  feed("MR\n");
  expect_contains("MR gone", "!E:parse");
  reset_out();
  feed("M 10\n");
  expect_contains("M gone", "!E:parse");
  reset_out();
  feed("W\n");
  expect_contains("W gone", "!E:parse");

  reset_out();
  feed("ME\nSE 1\nSP 0\n");
  expect_empty("idle before late #");
  reset_out();
  protocol_feed_byte('#');
  expect_contains("realtime # compact", "#I ");
  reset_out();
  protocol_feed_byte(0x1B);
  feed("IM\n");
  expect_contains("ESC stop", "IM:0");

  /* Terminal Mode: UART lines sniffed to write_debug before execute */
  reset_out();
  feed("ST 1\n");
  expect_empty("ST silent");
  reset_out();
  feed_uart("SS 44\n");
  expect_dbg_contains("UART sniff SS", "SS 44\n");
  reset_out();
  feed("GS\n");
  expect_contains("UART cmd executed after sniff", "GS:44");
  expect_dbg_empty("USB feed not sniffed");

  reset_out();
  feed("ST 0\n");
  feed_uart("SS 55\n");
  expect_dbg_empty("no sniff when terminal off");
  reset_out();
  feed("GS\n");
  expect_contains("UART cmd still runs without terminal", "GS:55");

  /* --- Path (PC/PD/PG/PN/PS) --- */
  reset_out();
  feed("SE 1\n");
  feed("PC\n");
  expect_empty("PC clears silently");
  reset_out();
  feed("PN\n");
  expect_contains("PN 0 after clear", "PN:0");

  reset_out();
  feed("PD 12345\n");
  feed("PD -32768\n");
  expect_empty("PD appends silently");
  reset_out();
  feed("PN\n");
  expect_contains("PN 2 after two PD", "PN:2");

  reset_out();
  feed("PD 32768\n");
  expect_contains("PD above int16 range rejected", "!E:parse");
  reset_out();
  feed("PD -32769\n");
  expect_contains("PD below int16 range rejected", "!E:parse");

  reset_out();
  feed("CS path_buffer_size 2\n");
  feed("PD 1\n");
  expect_contains("PD rejected once buffer_size reached", "!E:full");
  feed("CS path_buffer_size 32000\n");

  reset_out();
  feed("PS 500\n");
  expect_contains("PS below 1000us rejected", "!E:parse");
  reset_out();
  feed("PS 2000\n");
  expect_empty("PS accepted at 2000us");
  reset_out();
  feed("PS\n");
  expect_empty("PS bare resets to default silently");

  reset_out();
  feed("PC\n");
  feed("PG\n");
  expect_contains("PG rejected on empty buffer", "!E:empty");

  reset_out();
  feed("PD 100\n");
  feed("SE 0\n");
  feed("PG\n");
  expect_contains("PG rejected when disabled", "!E:disabled");
  feed("SE 1\n");

  reset_out();
  feed("PG\n");
  expect_empty("PG accepted");

  reset_out();
  feed("MT 10\n");
  expect_contains("MT rejected while path active", "!E:busy");
  reset_out();
  feed("MJ 50\n");
  expect_contains("MJ rejected while path active", "!E:busy");
  reset_out();
  feed("SS 10\n");
  expect_contains("SS rejected while path active", "!E:busy");
  reset_out();
  feed("SL 10\n");
  expect_contains("SL rejected while path active", "!E:busy");
  reset_out();
  feed("GL\n");
  expect_contains("GL allowed while path active", "GL:");
  reset_out();
  feed("PN\n");
  expect_contains("PN allowed while path active", "PN:1");
  reset_out();
  feed("PC\n");
  expect_contains("PC rejected while path active", "!E:busy");
  reset_out();
  feed("PG\n");
  expect_contains("PG rejected while already active", "!E:busy");

  reset_out();
  feed("PD 200\n");
  expect_empty("PD allowed while path active (live-move streaming)");
  reset_out();
  feed("PN\n");
  expect_contains("PN reflects live-streamed PD", "PN:2");

  reset_out();
  feed("MS\n");
  expect_empty("MS hands path off to planner and stops silently");
  reset_out();
  feed("MT 10\n");
  expect_empty("MT accepted again once path-mode ended");

  /* --- MJ / MoveJoy (1-axis) --- */
  reset_out();
  feed("MJ\n");
  expect_contains("MJ no args", "!E:parse MJ args");
  reset_out();
  feed("MJ foo\n");
  expect_contains("MJ junk args", "!E:parse");
  reset_out();
  feed("SE 0\n");
  feed("MJ 50\n");
  expect_contains("MJ disabled", "!E:disabled");
  feed("SE 1\n");

  reset_out();
  feed("SS 80\n");
  expect_empty("SS 80 before MJ");
  reset_out();
  feed("MJ 50\n");
  expect_empty("MJ 50 silent");
  expect_true("MJ 50 cruise 40", std::fabs(motion_host_axis_cruise(0) - 40.0f) < 0.01f);
  reset_out();
  feed("IM\n");
  expect_contains("IM during MJ", "IM:1");
  reset_out();
  feed("GS\n");
  expect_contains("MJ does not change SS", "GS:80");

  reset_out();
  feed("SS 60\n");
  expect_empty("SS during MJ silent");
  expect_true("SS rescale joy cruise", std::fabs(motion_host_axis_cruise(0) - 30.0f) < 0.01f);
  reset_out();
  feed("SA 150\n");
  expect_empty("SA during MJ silent");
  expect_true("SA during MJ", std::fabs(motion_host_axis_accel(0) - 150.0f) < 0.01f);
  expect_true("SA during MJ sets decel too",
              std::fabs(motion_host_axis_decel(0) - 150.0f) < 0.01f);
  reset_out();
  feed("GS\n");
  expect_contains("SS still 60 in joy", "GS:60");

  reset_out();
  feed("MJ 500\n");
  expect_empty("MJ over 100% silent");
  expect_true("MJ clamp to max_speed", std::fabs(motion_host_axis_cruise(0) - 100.0f) < 0.01f);

  reset_out();
  feed("MJ 0\n");
  expect_empty("MJ 0 silent");
  reset_out();
  feed("IM\n");
  expect_contains("IM after MJ 0", "IM:0");

  reset_out();
  feed("MJ 50 -20\n");
  expect_contains("1-axis MJ extra axis parse", "!E:parse");
  reset_out();
  feed("MJ 50\n");
  expect_empty("1-axis MJ 50");
  expect_true("1-axis MJ 50 cruise 30", std::fabs(motion_host_axis_cruise(0) - 30.0f) < 0.01f);

  reset_out();
  feed("MS\n");
  expect_empty("MS exits joy");
  reset_out();
  feed("GS\n");
  expect_contains("SS unchanged after MS", "GS:60");

  reset_out();
  feed("MS\n");
  expect_empty("MS after joy");
  reset_out();
  feed("IP\n");
  /* sit at current pos: set slider_max to that position and command into the rail */
  {
    McStatus st;
    motion_get_status(&st);
    char cmd[64];
    std::snprintf(cmd, sizeof(cmd), "CS MOTOR_1_max %.3f\n", (double)st.pos[0]);
    reset_out();
    feed(cmd);
    expect_empty("CS slider_max to current pos");
  }
  reset_out();
  feed("MJ 50\n");
  expect_empty("MJ into soft rail silent");
  reset_out();
  feed("CS MOTOR_1_max 600\n");
  expect_empty("CS MOTOR_1_max restore");
  reset_out();
  feed("SR\n");
  expect_empty("SR bare restores session right after CS squeeze");
  reset_out();
  feed("SS 50\n");
  expect_empty("SS restore 50");
  reset_out();
  feed("SA 200\n");
  expect_empty("SA restore 200");

  /* --- SL/SR/GL/GR session working window (1-axis) --- */
  reset_out();
  feed("SL\n");
  feed("SR\n");
  expect_empty("SL/SR bare to envelope");
  reset_out();
  feed("GL\n");
  expect_contains("GL boot envelope", "GL:0");
  reset_out();
  feed("GR\n");
  expect_contains("GR boot envelope", "GR:600");
  reset_out();
  feed("SL 120\n");
  expect_empty("SL 120 silent");
  reset_out();
  feed("GL\n");
  expect_contains("GL after SL", "GL:120");
  reset_out();
  feed("CG MOTOR_1_min\n");
  expect_contains("envelope unchanged by SL", "CG:MOTOR_1_min=0");
  reset_out();
  feed("SL\n");
  expect_empty("SL bare reset");
  reset_out();
  feed("GL\n");
  expect_contains("GL after bare SL", "GL:0");
  reset_out();
  feed("SL 700\n");
  expect_contains("SL past envelope", "!E:limit");
  reset_out();
  feed("SL 200\n");
  expect_empty("SL 200 for crossed-window test");
  reset_out();
  feed("SR 100\n");
  expect_contains("SR left of SL rejected", "!E:limit");
  reset_out();
  feed("GR\n");
  expect_contains("GR unchanged after reject", "GR:600");
  reset_out();
  feed("SL 80\n");
  expect_empty("SL 80 before CS clamp");
  reset_out();
  feed("CS MOTOR_1_min 150\n");
  expect_empty("CS MOTOR_1_min 150");
  reset_out();
  feed("GL\n");
  expect_contains("CS clamps session left", "GL:150");
  reset_out();
  feed("CS MOTOR_1_min 0\n");
  expect_empty("CS MOTOR_1_min restore 0");
  reset_out();
  feed("GL\n");
  expect_contains("CS widen envelope keeps window", "GL:150");
  reset_out();
  feed("SL\n");
  expect_empty("SL bare after CS clamp test");
  reset_out();
  feed("GL\n");
  expect_contains("GL full rail again", "GL:0");

  /* SL none: session cleared; GL/clip fall back to envelope when set */
  reset_out();
  feed("SL 120\n");
  expect_empty("SL 120 before none");
  reset_out();
  feed("SL none\n");
  expect_empty("SL none with envelope");
  reset_out();
  feed("GL\n");
  expect_contains("GL after SL none uses envelope", "GL:0");
  reset_out();
  feed("CS MOTOR_1_min none\n");
  expect_empty("CS MOTOR_1_min none");
  reset_out();
  feed("CG MOTOR_1_min\n");
  expect_contains("CG unset envelope is dash", "CG:MOTOR_1_min=-");
  reset_out();
  feed("SL none\n");
  expect_empty("SL none with open envelope");
  reset_out();
  feed("GL\n");
  expect_contains("GL open after none+none envelope", "GL:-");
  reset_out();
  feed("CS MOTOR_1_min 0\n");
  feed("SL\n");
  expect_empty("restore MOTOR_1_min and bare SL");
  reset_out();
  feed("MT none\n");
  expect_contains("MT none not a skip", "!E:parse");
  reset_out();
  feed("MT *\n");
  expect_contains("MT star not a skip", "!E:parse");
  reset_out();
  feed("MT N\n");
  expect_contains("MT N not a skip", "!E:parse");

  reset_out();
  feed("MS\n");
  feed("SP 110\n");
  expect_empty("park 110 before inward MJ");
  feed("SL 100\n");
  expect_empty("SL 100 for inward MJ");
  reset_out();
  feed("MJ -80\n");
  expect_empty("MJ negative below window silent");
  motion_stub_tick_ms(800);
  reset_out();
  feed("IP\n");
  expect_contains("MJ out clamped inward to left wall", "IP:100");
  reset_out();
  feed("SL\n");
  feed("MS\n");
  expect_empty("reset window after inward test");

  /* --- axis2 protocol (HOST_TEST builds with PIN_AXIS2_SUPPORTED) --- */
  reset_out();
  feed("IA\n");
  expect_contains("IA before axis2", "IA:1");

  reset_out();
  feed("CS motors 2\n");
  expect_empty("CS motors 2");
  reset_out();
  feed("CG axis\n");
  expect_contains("CG axis 2", "CG:axis=2");

  reset_out();
  feed("IA\n");
  expect_contains("IA after axis2", "IA:2");
  reset_out();
  feed("Axis\n");
  expect_contains("Axis long gone", "!E:parse");

  reset_out();
  protocol_send_banner();
  expect_contains("banner 2+0 axis", "- 2+0 axis");
  expect_not_contains("banner no name yet", "Foo - Slider");

  reset_out();
  feed("CS name Foo\n");
  expect_empty("CS name Foo");
  reset_out();
  feed("CG name\n");
  expect_contains("CG name", "CG:name=Foo");
  reset_out();
  feed("CG motor_1_unit\n");
  expect_contains("CG motor_1_unit default", "CG:motor_1_unit=mm");
  reset_out();
  feed("CS motor_1_unit deg\n");
  expect_empty("CS motor_1_unit deg");
  reset_out();
  feed("CG motor_1_unit\n");
  expect_contains("CG motor_1_unit deg", "CG:motor_1_unit=deg");
  reset_out();
  feed("CG servo_1_unit\n");
  expect_contains("CG servo_1_unit default", "CG:servo_1_unit=deg");
  reset_out();
  feed("CS servo_1_unit rad\n");
  expect_empty("CS servo_1_unit rad");
  reset_out();
  feed("CG servo_1_unit\n");
  expect_contains("CG servo_1_unit rad", "CG:servo_1_unit=rad");
  reset_out();
  feed("CG axis_1_unit\n");
  expect_contains("CG axis_1_unit matches motor 1", "CG:axis_1_unit=deg");
  reset_out();
  feed("CS axis_1_unit mm\n");
  expect_contains("CS axis_1_unit rejected", "!E:cfg bad key/value");
  reset_out();
  feed("CS steps_per_unit_1 200\n");
  expect_empty("CS steps_per_unit_1");
  reset_out();
  feed("CG steps_per_unit_1\n");
  expect_contains("CG steps_per_unit_1", "CG:steps_per_unit_1=200");
  reset_out();
  feed("CS steps_per_mm_1 320\n");
  expect_empty("CS steps_per_mm_1 alias");
  reset_out();
  feed("CG steps_per_mm_1\n");
  expect_contains("CG steps_per_mm_1 alias", "CG:steps_per_mm_1=320");
  reset_out();
  protocol_send_banner();
  expect_contains("named 2-axis banner", "# MC V1 - Foo");
  expect_contains("named banner has 2+0 axis", "- 2+0 axis");

  reset_out();
  feed("MT 0 0\n");
  expect_empty("MT park dual origin");
  reset_out();
  feed("MS\n");
  expect_empty("MS snap to origin");
  reset_out();
  feed("MT 100 50\n");
  expect_empty("MT dual absolute accepted");
  reset_out();
  protocol_feed_byte('#');
  expect_contains("moving ? grouped axis2", " | ");
  {
    /* |d2|/|d1| = 50/100 = 0.5 */
    float v0 = motion_host_axis_cruise(0);
    float v1 = motion_host_axis_cruise(1);
    float a0 = motion_host_axis_accel(0);
    float a1 = motion_host_axis_accel(1);
    float d0 = motion_host_axis_decel(0);
    float d1 = motion_host_axis_decel(1);
    expect_true("dual MT cruise ratio ~0.5", std::fabs(v1 - v0 * 0.5f) < 0.01f);
    expect_true("dual MT accel ratio ~0.5", std::fabs(a1 - a0 * 0.5f) < 0.01f);
    expect_true("dual MT decel ratio ~0.5", std::fabs(d1 - d0 * 0.5f) < 0.01f);
  }
  reset_out();
  feed("SS 40\n");
  expect_empty("SS mid dual move");
  {
    float v0 = motion_host_axis_cruise(0);
    float v1 = motion_host_axis_cruise(1);
    expect_true("SS mid-move axis1=40", std::fabs(v0 - 40.0f) < 0.01f);
    expect_true("SS mid-move axis2 keeps ratio", std::fabs(v1 - 20.0f) < 0.01f);
  }
  reset_out();
  feed("SA 100\n");
  expect_empty("SA mid dual move");
  {
    float a0 = motion_host_axis_accel(0);
    float a1 = motion_host_axis_accel(1);
    float d0 = motion_host_axis_decel(0);
    float d1 = motion_host_axis_decel(1);
    expect_true("SA mid-move axis1=100", std::fabs(a0 - 100.0f) < 0.01f);
    expect_true("SA mid-move axis2 keeps ratio", std::fabs(a1 - 50.0f) < 0.01f);
    expect_true("SA mid-move decel axis1=100", std::fabs(d0 - 100.0f) < 0.01f);
    expect_true("SA mid-move decel axis2 keeps ratio", std::fabs(d1 - 50.0f) < 0.01f);
  }
  reset_out();
  feed("SA 80 40\n");
  expect_empty("SA split mid dual move");
  {
    float a0 = motion_host_axis_accel(0);
    float a1 = motion_host_axis_accel(1);
    float d0 = motion_host_axis_decel(0);
    float d1 = motion_host_axis_decel(1);
    expect_true("SA split mid-move accel1=80", std::fabs(a0 - 80.0f) < 0.01f);
    expect_true("SA split mid-move accel2 ratio", std::fabs(a1 - 40.0f) < 0.01f);
    expect_true("SA split mid-move decel1=40", std::fabs(d0 - 40.0f) < 0.01f);
    expect_true("SA split mid-move decel2 ratio", std::fabs(d1 - 20.0f) < 0.01f);
  }
  reset_out();
  feed("IP\n");
  expect_contains("IP dual after MT", "IP:");

  reset_out();
  feed("MS\n");
  feed("MT 0 0\n");
  feed("MS\n");
  feed("SS 50\n");
  feed("SA 200\n");
  expect_empty("park before 2-axis WP");
  reset_out();
  feed("MT 100 80; WP 40; SS 4\n");
  {
    bool ran = false;
    for (int i = 0; i < 200; ++i) {
      protocol_poll(20);
      reset_out();
      feed("GS\n");
      if (g_out.find("GS:4") != std::string::npos) {
        ran = true;
        break;
      }
    }
    expect_true("2-axis WP keys off axis1", ran);
  }
  reset_out();
  feed("IP\n");
  {
    size_t ip = g_out.find("IP:");
    if (ip == std::string::npos) {
      std::fprintf(stderr, "FAIL IP dual: no IP: in:\n%s\n", g_out.c_str());
      ++g_fail;
    } else {
      std::string line = g_out.substr(ip);
      size_t nl = line.find('\n');
      if (nl != std::string::npos) {
        line = line.substr(0, nl);
      }
      /* "IP:a b" → one space after the colon value pair */
      int spaces = 0;
      for (size_t i = 3; i < line.size(); ++i) {
        if (line[i] == ' ') {
          ++spaces;
        }
      }
      if (spaces < 1) {
        std::fprintf(stderr, "FAIL IP dual needs two fields, got:\n%s\n", line.c_str());
        ++g_fail;
      } else {
        std::printf("OK   IP dual positions\n");
      }
    }
  }
  reset_out();
  feed("MT none 30\n");
  expect_contains("MT none not skip on dual", "!E:parse");
  reset_out();
  feed("MT _ 40\n");
  expect_empty("MT underscore skip accepted");
  reset_out();
  feed("MT * 40\n");
  expect_contains("MT star not skip on dual", "!E:parse");

  reset_out();
  feed("ML 1\n");
  expect_contains("ML gone on dual", "!E:parse");
  reset_out();
  feed("MR 2\n");
  expect_contains("MR gone on dual", "!E:parse");
  reset_out();
  feed("MH 1\n");
  expect_empty("MH axis 1");
  reset_out();
  feed("MT Y40\n");
  expect_empty("MT Y40 named");
  reset_out();
  feed("MTX20Z100\n");
  expect_contains("MT X+Z without Y on 2-axis", "!E:parse");
  reset_out();
  feed("CS motors 3\n");
  expect_empty("temp axis 3 for named XYZ");
  reset_out();
  feed("SE 1\n");
  feed("MTX20Y50Z100\n");
  expect_empty("MTX20Y50Z100 glued");
  reset_out();
  feed("MS\n");
  expect_empty("MS before SP glued");
  reset_out();
  feed("SPX20Y50Z100\n");
  expect_empty("SP glued named");
  reset_out();
  feed("IP\n");
  expect_contains("IP after glued XYZ", "IP:20 | 50 | 100");
  reset_out();
  feed("MJ Y50\n");
  expect_empty("MJ Y50 named");
  reset_out();
  feed("MT 20 Z100\n");
  expect_contains("mixed positional+named", "!E:parse");
  reset_out();
  feed("MT X20 X100\n");
  expect_contains("duplicate X", "!E:parse");
  reset_out();
  feed("MBX-5\n");
  expect_empty("MBX-5 relative");
  reset_out();
  feed("CS motors 2\n");
  expect_empty("restore axis 2 after named XYZ");
  reset_out();
  feed("SE 1\n");

  reset_out();
  feed("MS\n");
  expect_empty("MS before dual MJ");
  reset_out();
  feed("SS 80\n");
  expect_empty("SS 80 for dual MJ");
  reset_out();
  feed("CS max_speed_2 30\n");
  expect_empty("CS max_speed_2 30");
  reset_out();
  feed("CG max_speed_2\n");
  expect_contains("CG max_speed_2 30", "CG:max_speed_2=30");
  reset_out();
  feed("MJ 100 100\n");
  expect_empty("MJ 100 100 silent");
  expect_true("MJ axis0 cruise SS", std::fabs(motion_host_axis_cruise(0) - 80.0f) < 0.01f);
  expect_true("MJ axis1 clamp max_speed_2", std::fabs(motion_host_axis_cruise(1) - 30.0f) < 0.01f);

  reset_out();
  feed("CS max_speed_2 100\n");
  expect_empty("CS max_speed_2 restore");
  reset_out();
  feed("MJ 40\n");
  expect_empty("MJ 40 snapshot axis1=0");
  {
    McStatus st;
    motion_get_status(&st);
    expect_true("snapshot axis0 moving", st.vel[0] > 0.0f);
    expect_true("snapshot axis1 stopped", std::fabs(st.vel[1]) < 0.01f);
  }
  expect_true("snapshot axis0 cruise 32", std::fabs(motion_host_axis_cruise(0) - 32.0f) < 0.01f);

  reset_out();
  feed("MJ 40 -20\n");
  expect_empty("MJ independent signs");
  {
    McStatus st;
    motion_get_status(&st);
    expect_true("MJ 40 vel+", st.vel[0] > 0.0f);
    expect_true("MJ -20 vel-", st.vel[1] < 0.0f);
  }
  expect_true("MJ 40 cruise 32", std::fabs(motion_host_axis_cruise(0) - 32.0f) < 0.01f);
  expect_true("MJ -20 cruise 16", std::fabs(motion_host_axis_cruise(1) - 16.0f) < 0.01f);

  reset_out();
  feed("SS 50\n");
  expect_empty("SS during dual MJ");
  expect_true("SS rescale axis0", std::fabs(motion_host_axis_cruise(0) - 20.0f) < 0.01f);
  expect_true("SS rescale axis1", std::fabs(motion_host_axis_cruise(1) - 10.0f) < 0.01f);

  reset_out();
  feed("MS\n");
  expect_empty("MS exits dual MJ");
  reset_out();
  feed("GS\n");
  expect_contains("GS after dual MJ MS", "GS:50");

  reset_out();
  feed("PC\n");
  feed("PD 100 200\n");
  expect_empty("PD dual sample");
  reset_out();
  feed("PN\n");
  expect_contains("PN after dual PD", "PN:1");
  reset_out();
  feed("PD _ _\n");
  expect_empty("PD underscore skip become 0");
  reset_out();
  feed("PD * N\n");
  expect_contains("PD star/N not skip", "!E:parse");

  reset_out();
  feed("SL\n");
  feed("SL _ 40\n");
  expect_empty("SL skip axis1");
  reset_out();
  feed("GL\n");
  expect_contains("GL dual skip axis2", "GL:0 | 40");
  reset_out();
  feed("SL none 50\n");
  expect_empty("SL none axis1 set axis2");
  reset_out();
  feed("GL\n");
  expect_contains("GL after SL none 50", "GL:0 | 50");
  reset_out();
  feed("SR 200\n");
  feed("SL _ none\n");
  expect_empty("SL clear axis2 only");
  reset_out();
  feed("GL\n");
  expect_contains("GL after SL _ none", "GL:0 | 0");
  reset_out();
  feed("SL\n");
  expect_empty("SL bare resets both axes");
  reset_out();
  feed("GL\n");
  expect_contains("GL dual after bare SL", "GL:0 | 0");

  reset_out();
  feed("EO4 1\n");
  expect_empty("EO4 on");
  reset_out();
  feed("EO5 1\n");
  expect_contains("EO5 invalid", "!E:parse");
  reset_out();
  feed("EO6 1\n");
  expect_contains("EO6 invalid", "!E:parse");

  reset_out();
  feed("IG\n");
  expect_contains("IG has STEP_2 when axis2", "DRV_STEP_2");
  expect_contains("IG has DRV_ENABLE", "DRV_ENABLE");
  expect_not_contains("IG no per-axis EN2", "DRV_EN2");
  reset_out();
  feed("VG\n");
  expect_contains("VG has STEP_2 when axis2", "PIN_DRV_STEP_2=");
  expect_contains("VG has PIN_DRV_ENABLE", "PIN_DRV_ENABLE=");
  expect_not_contains("VG no PIN_DRV_EN2", "PIN_DRV_EN2=");

  reset_out();
  feed("SP 100 50\n");
  expect_empty("park dual non-zero for idle ?");
  reset_out();
  protocol_feed_byte('#');
  expect_contains("realtime # with pos2", "#I ");
  expect_contains("idle ? grouped axis2", " | ");
  {
    size_t hash = g_out.find("#I ");
    if (hash == std::string::npos) {
      std::fprintf(stderr, "FAIL idle ? format: no #I in:\n%s\n", g_out.c_str());
      ++g_fail;
    } else {
      std::string line = g_out.substr(hash);
      size_t nl = line.find('\n');
      if (nl != std::string::npos) {
        line = line.substr(0, nl);
      }
      /* "#I pos1 | pos2" */
      if (line.find(" | ") == std::string::npos) {
        std::fprintf(stderr, "FAIL idle ? needs pos1 | pos2, got:\n%s\n", line.c_str());
        ++g_fail;
      } else {
        std::printf("OK   idle ? dual positions\n");
      }
    }
  }

  reset_out();
  feed("CS motors 1\n");
  expect_empty("CS motors 1 restore 1-axis");
  reset_out();
  feed("IA\n");
  expect_contains("IA restored 1-axis", "IA:1");
  reset_out();
  feed("IG\n");
  expect_not_contains("IG no STEP_2 when 1-axis", "DRV_STEP_2");
  reset_out();
  feed("VG\n");
  expect_not_contains("VG no STEP_2 when 1-axis", "PIN_DRV_STEP_2=");
  reset_out();
  feed("IP\n");
  expect_contains("IP single-field 1-axis", "IP:");
  expect_not_contains("IP no second field when 1-axis", "IP:0 0");
  {
    size_t ip = g_out.find("IP:");
    if (ip != std::string::npos) {
      std::string line = g_out.substr(ip);
      size_t nl = line.find('\n');
      if (nl != std::string::npos) {
        line = line.substr(0, nl);
      }
      int spaces = 0;
      for (size_t i = 3; i < line.size(); ++i) {
        if (line[i] == ' ') {
          ++spaces;
        }
      }
      if (spaces != 0) {
        std::fprintf(stderr, "FAIL IP 1-axis should be one field, got:\n%s\n", line.c_str());
        ++g_fail;
      } else {
        std::printf("OK   IP single position\n");
      }
    }
  }
  reset_out();
  protocol_send_banner();
  expect_not_contains("1-axis banner no 2+0", "2+0 axis");
  expect_contains("named 1-axis banner", "# MC V1 - Foo");
  reset_out();
  feed("RB\n");
  expect_empty("RB silent on host");
  reset_out();
  feed("HL\n");
  expect_contains("HL lists RB", "RB");
  expect_contains("HL lists Reboot", "Reboot");

  reset_out();
  feed("SE 1\n");
  feed("SP\n");
  expect_empty("bare SP silent");
  reset_out();
  feed("IP\n");
  expect_contains("bare SP zeros pose", "IP:0");
  reset_out();
  feed("SP 100\n");
  expect_empty("SP 100 silent");
  reset_out();
  feed("IP\n");
  expect_contains("SP 100 pose", "IP:100");
  reset_out();
  feed("SP 0\n");
  expect_empty("SP 0 silent");
  reset_out();
  feed("IP\n");
  expect_contains("SP 0 pose", "IP:0");
  reset_out();
  feed("CS home_mode_1 3\n");
  expect_empty("CS home_mode_1 3 stall");
  reset_out();
  feed("CG home_mode_1\n");
  expect_contains("CS home_mode_1 3 persists", "CG:home_mode_1=3");
  reset_out();
  feed("CS home_mode_1 0\n");
  expect_empty("CS home_mode_1 0 restore");
  reset_out();
  feed("CS motors 2\n");
  feed("SP 10 20\n");
  expect_empty("SP dual silent");
  reset_out();
  feed("IP\n");
  expect_contains("SP dual pose", "IP:10 | 20");
  reset_out();
  feed("SP _ 0\n");
  expect_empty("SP skip axis1");
  reset_out();
  feed("IP\n");
  expect_contains("SP skip keeps axis1", "IP:10 | 0");
  reset_out();
  feed("MT 50\n");
  reset_out();
  feed("SP 0\n");
  expect_contains("SP rejected while moving", "!E:busy");
  reset_out();
  feed("MS\n");
  feed("CS motors 1\n");
  expect_empty("restore 1-axis after SP tests");

  /* --- axis3 protocol --- */
  reset_out();
  feed("CS motors 3\n");
  expect_empty("CS motors 3");
  reset_out();
  feed("CG axis\n");
  expect_contains("CG axis 3", "CG:axis=3");
  reset_out();
  feed("IA\n");
  expect_contains("IA after axis 3", "IA:3");
  reset_out();
  protocol_send_banner();
  expect_contains("banner 3+0 axis", "- 3+0 axis");

  reset_out();
  feed("CS motors 2\n");
  expect_empty("CS motors 2 from 3");
  reset_out();
  feed("IA\n");
  expect_contains("CS motors 2 sets IA:2", "IA:2");
  reset_out();
  feed("CS motors 3\n");
  expect_empty("CS motors 3 again");

  reset_out();
  feed("MS\n");
  feed("SP 0 0 0\n");
  expect_empty("SP triple origin");
  reset_out();
  feed("IP\n");
  expect_contains("IP triple origin", "IP:0 | 0 | 0");

  reset_out();
  feed("MT 100 50 25\n");
  expect_empty("MT triple absolute");
  {
    float v0 = motion_host_axis_cruise(0);
    float v1 = motion_host_axis_cruise(1);
    float v2 = motion_host_axis_cruise(2);
    float a0 = motion_host_axis_accel(0);
    float a1 = motion_host_axis_accel(1);
    float a2 = motion_host_axis_accel(2);
    expect_true("triple MT cruise ratio 0.5", std::fabs(v1 - v0 * 0.5f) < 0.01f);
    expect_true("triple MT cruise ratio 0.25", std::fabs(v2 - v0 * 0.25f) < 0.01f);
    expect_true("triple MT accel ratio 0.5", std::fabs(a1 - a0 * 0.5f) < 0.01f);
    expect_true("triple MT accel ratio 0.25", std::fabs(a2 - a0 * 0.25f) < 0.01f);
    float d0 = motion_host_axis_decel(0);
    float d1 = motion_host_axis_decel(1);
    float d2 = motion_host_axis_decel(2);
    expect_true("triple MT decel ratio 0.5", std::fabs(d1 - d0 * 0.5f) < 0.01f);
    expect_true("triple MT decel ratio 0.25", std::fabs(d2 - d0 * 0.25f) < 0.01f);
  }
  reset_out();
  feed("MS\n");
  feed("SP 0 0 0\n");
  feed("SP _ _ 40\n");
  expect_empty("SP skip first two");
  reset_out();
  feed("IP\n");
  expect_contains("SP skip keeps 1+2 zeros axis3", "IP:0 | 0 | 40");

  reset_out();
  feed("PC\n");
  feed("PD 10 20 30\n");
  expect_empty("PD triple sample");
  reset_out();
  feed("PD _ _ 5\n");
  expect_empty("PD skip become 0");
  reset_out();
  feed("PDZ5\n");
  expect_empty("PDZ5 named");
  reset_out();
  feed("PN\n");
  expect_contains("PN after three triple samples", "PN:3");

  reset_out();
  feed("MH 3\n");
  expect_empty("MH axis 3");
  reset_out();
  feed("ML 3\n");
  expect_contains("ML gone on triple", "!E:parse");
  reset_out();
  feed("MT Z40\n");
  expect_empty("MT Z40 named");
  reset_out();
  feed("MS\n");
  expect_empty("MS after axis3 MT");

  reset_out();
  feed("CS axis 4\n");
  expect_contains("CS axis 4 rejected", "!E:cfg bad key/value");
  reset_out();
  feed("IA\n");
  expect_contains("IA still 3 after axis 4 reject", "IA:3");

  reset_out();
  feed("SP 0 0 0\n");
  protocol_feed_byte('#');
  expect_contains("idle ? 3-axis elides zero groups", "#I||");

  reset_out();
  feed("SS 80\n");
  expect_empty("SS 80 for triple MJ");
  reset_out();
  feed("MJ 100 50 25\n");
  expect_empty("MJ triple silent");
  expect_true("MJ triple cruise 80", std::fabs(motion_host_axis_cruise(0) - 80.0f) < 0.01f);
  expect_true("MJ triple cruise 40", std::fabs(motion_host_axis_cruise(1) - 40.0f) < 0.01f);
  expect_true("MJ triple cruise 20", std::fabs(motion_host_axis_cruise(2) - 20.0f) < 0.01f);
  {
    McStatus st;
    motion_get_status(&st);
    expect_true("MJ triple vel+", st.vel[0] > 0.0f);
    expect_true("MJ triple vel2+", st.vel[1] > 0.0f);
    expect_true("MJ triple vel3+", st.vel[2] > 0.0f);
  }
  reset_out();
  feed("MS\n");
  expect_empty("MS after triple MJ");

  reset_out();
  feed("SL 10 20 30\n");
  expect_empty("SL triple");
  reset_out();
  feed("GL\n");
  expect_contains("GL triple", "GL:10 | 20 | 30");
  reset_out();
  feed("SL\n");
  expect_empty("SL bare after triple");

  reset_out();
  feed("CS motors 1\n");
  expect_empty("restore 1-axis after axis3 tests");
  reset_out();
  feed("IA\n");
  expect_contains("IA restored after axis3", "IA:1");

  /* --- servo PWM axes --- */
  reset_out();
  feed("CS axis 2\n");
  expect_contains("CS axis rejected", "!E:cfg bad key/value");
  reset_out();
  feed("CS slider_min_1 0\n");
  expect_contains("slider_min rejected", "!E:cfg bad key/value");
  reset_out();
  feed("CS motors 1\n");
  feed("CS servos 0\n");
  feed("SL\n");
  feed("SR\n");
  expect_empty("CS motors 1 servos 0");
  reset_out();
  protocol_send_banner();
  expect_contains("banner 1+0 axis", "1+0 axis");
  reset_out();
  feed("IA\n");
  expect_contains("IA 1+0", "IA:1");
  reset_out();
  feed("CG axis\n");
  expect_contains("CG axis sum 1", "CG:axis=1");
  reset_out();
  feed("CG\n");
  expect_contains("bare CG dump axis 1", "CG:axis=1");

  reset_out();
  feed("CS servos 2\n");
  expect_empty("CS servos 2");
  reset_out();
  feed("IA\n");
  expect_contains("IA 1+2", "IA:3");
  reset_out();
  feed("CG axis\n");
  expect_contains("CG axis sum 3", "CG:axis=3");
  reset_out();
  feed("CG\n");
  expect_contains("bare CG dump axis 3", "CG:axis=3");
  reset_out();
  protocol_send_banner();
  expect_contains("banner 1+2 axis", "1+2 axis");

  reset_out();
  feed("CS MOTOR_1_min 0\n");
  feed("CS SERVO_1_min -90\n");
  expect_empty("kind-stable envelopes");
  reset_out();
  feed("CG MOTOR_1_min\n");
  expect_contains("CG MOTOR_1_min", "CG:MOTOR_1_min=0");
  reset_out();
  feed("CG SERVO_1_min\n");
  expect_contains("CG SERVO_1_min", "CG:SERVO_1_min=-90");
  reset_out();
  feed("CS axis_min_2 -45\n");
  expect_empty("CS axis_min_2 write-through");
  reset_out();
  feed("CG SERVO_1_min\n");
  expect_contains("axis_min_2 wrote SERVO_1_min", "CG:SERVO_1_min=-45");
  reset_out();
  feed("CG axis_min_2\n");
  expect_contains("CG axis_min_2 synth", "CG:axis_min_2=-45");
  reset_out();
  feed("CS SERVO_1_min -135\n");
  expect_empty("restore SERVO_1_min");

  reset_out();
  feed("CG SERVO_1_min_pulse\n");
  expect_contains("CG default min_pulse", "CG:SERVO_1_min_pulse=500");
  reset_out();
  feed("CG SERVO_1_max_pulse\n");
  expect_contains("CG default max_pulse", "CG:SERVO_1_max_pulse=2500");
  reset_out();
  feed("CG SERVO_1_swap\n");
  expect_contains("CG default swap", "CG:SERVO_1_swap=0");
  expect_true("map -135 → 500 µs",
              fabsf(servo_pwm_deg_to_us(0, -135.0f) - 500.0f) < 0.5f);
  expect_true("map 135 → 2500 µs",
              fabsf(servo_pwm_deg_to_us(0, 135.0f) - 2500.0f) < 0.5f);
  expect_true("map 0 → 1500 µs",
              fabsf(servo_pwm_deg_to_us(0, 0.0f) - 1500.0f) < 0.5f);
  reset_out();
  feed("CS SERVO_1_min_pulse 1000\n");
  feed("CS SERVO_1_max_pulse 2000\n");
  expect_empty("analog 1000–2000 pulse");
  expect_true("analog map -135 → 1000 µs",
              fabsf(servo_pwm_deg_to_us(0, -135.0f) - 1000.0f) < 0.5f);
  expect_true("analog map 135 → 2000 µs",
              fabsf(servo_pwm_deg_to_us(0, 135.0f) - 2000.0f) < 0.5f);
  reset_out();
  feed("CS SERVO_1_swap 1\n");
  expect_empty("CS swap");
  expect_true("swap flips -135 → 2000 µs",
              fabsf(servo_pwm_deg_to_us(0, -135.0f) - 2000.0f) < 0.5f);
  expect_true("swap flips 135 → 1000 µs",
              fabsf(servo_pwm_deg_to_us(0, 135.0f) - 1000.0f) < 0.5f);
  reset_out();
  feed("CS SERVO_1_min_pulse 300\n");
  expect_contains("pulse 300 rejected", "!E:cfg bad key/value");
  reset_out();
  feed("CS SERVO_1_min_pulse 2000\n");
  expect_contains("min_pulse >= max_pulse rejected", "!E:cfg bad key/value");
  reset_out();
  feed("CS SERVO_1_min_pulse 500\n");
  feed("CS SERVO_1_max_pulse 2500\n");
  feed("CS SERVO_1_swap 0\n");
  expect_empty("restore digital pulse + swap");

  reset_out();
  feed("SE 1\n");
  feed("SP 0 0 0\n");
  expect_empty("SP three packed zeros");
  reset_out();
  feed("MT Y10\n");
  expect_contains("Y rejected when motors=1", "!E:parse");
  reset_out();
  feed("MT Z10\n");
  expect_contains("Z rejected when motors=1", "!E:parse");
  reset_out();
  feed("MT C10\n");
  expect_contains("C rejected when servos=2", "!E:parse");
  reset_out();
  feed("MH A\n");
  expect_contains("MH letter A parse fail", "!E:parse");

  reset_out();
  feed("CS servos 1\n");
  expect_empty("CS servos 1 for mix");
  reset_out();
  feed("SE 1\n");
  feed("SP 0 0\n");
  expect_empty("SP motor+servo origin");
  reset_out();
  feed("MT X10 A-45\n");
  expect_empty("MT X10 A-45");
  motion_stub_tick_ms(2000);
  reset_out();
  feed("IP\n");
  expect_contains("IP motor|servo after named MT", "IP:10 | -45");
  reset_out();
  feed("MS\n");
  feed("SP 10 -45\n");
  expect_empty("park 10 -45");
  reset_out();
  feed("MT 20 -90\n");
  expect_empty("positional MT motors then servos");
  motion_stub_tick_ms(2000);
  reset_out();
  feed("IP\n");
  expect_contains("IP after positional MT", "IP:20 | -90");

  reset_out();
  feed("MS\n");
  feed("SP 10 0\n");
  feed("CS servos 2\n");
  feed("SP 10 0 -45\n");
  expect_empty("park 10 0 -45");
  reset_out();
  protocol_feed_byte('#');
  expect_contains("verbose middle-zero elide", "#I 10 || -45");
  reset_out();
  feed("IP\n");
  expect_contains("IP keeps explicit 0", "IP:10 | 0 | -45");
  reset_out();
  feed("GL\n");
  expect_contains("GL pipes with zeros", "GL:");
  expect_contains("GL has pipe", " | ");

  reset_out();
  feed("CS motors 1\n");
  feed("CS servos 0\n");
  feed("SE 1\n");
  feed("SP 0\n");
  expect_empty("single channel park 0");
  reset_out();
  protocol_feed_byte('#');
  expect_contains("single-channel idle 0 kept", "#I 0");

  reset_out();
  feed("SE 0\n");
  expect_true("SE 0 stops PWM", !servo_pwm_enabled());
  reset_out();
  feed("CS servos 1\n");
  feed("SE 1\n");
  expect_true("SE 1 starts PWM with servos", servo_pwm_enabled());
  reset_out();
  feed("SE 0\n");
  expect_true("SE 0 limp again", !servo_pwm_enabled());

  reset_out();
  feed("CS servos 3\n");
  expect_empty("CS servos 3 steals EXT_4");
  reset_out();
  feed("EO4 1\n");
  expect_contains("EO4 fails when servos=3", "!E:parse");
  reset_out();
  feed("CS servos 0\n");
  expect_empty("restore servos 0");
  reset_out();
  feed("EO4 1\n");
  expect_empty("EO4 ok after servos 0");
  reset_out();
  feed("EO4 0\n");
  expect_empty("EO4 off");

  reset_out();
  {
    std::string line(CFG_LINE_MAX, 'x');
    line.push_back('\n');
    feed(line.c_str());
    expect_contains("1024-char line accepted", "!E:parse unknown command");
  }
  reset_out();
  {
    std::string line(CFG_LINE_MAX + 1, 'x');
    line.push_back('\n');
    feed(line.c_str());
    expect_contains("1025-char line rejected", "!E:parse line too long");
  }

  reset_out();
  feed("CS motors 2\n");
  feed("CS servos 0\n");
  feed("SE 1\n");
  feed("SS 50\n");
  feed("SA 200\n");
  feed("SP 10 0\n");
  expect_empty("park dest1==current setup");
  reset_out();
  feed("MT 10 80\n");
  expect_empty("MT dest1 current motor2 moves");
  expect_true("motor 2 is master", motion_master_channel() == 1);
  expect_true("master cruise is SS", std::fabs(motion_host_axis_cruise(1) - 50.0f) < 0.01f);
  reset_out();
  feed("MS\n");
  feed("CS motors 1\n");
  feed("CS servos 1\n");
  feed("SE 1\n");
  feed("SP 0 0\n");
  feed("MT A45\n");
  expect_empty("servo-only MT");
  expect_true("servo 1 is master", motion_master_channel() == 1);
  expect_true("SS as deg/s on servo master",
              std::fabs(motion_host_axis_cruise(1) - 50.0f) < 0.01f);

  reset_out();
  feed("MS\n");
  feed("CS servos 0\n");
  feed("CS motors 2\n");
  feed("SP 0 0\n");
  feed("MT 100 80\n");
  reset_out();
  feed("WP 40\n");
  expect_empty("WP keys motor 1");
  {
    bool ran = false;
    for (int i = 0; i < 200; ++i) {
      protocol_poll(20);
      reset_out();
      feed("IW\n");
      if (g_out.find("IW:0") != std::string::npos) {
        ran = true;
        break;
      }
    }
    expect_true("WP 40 on motor 1 completed", ran);
  }
  reset_out();
  feed("MS\n");
  feed("SP 0 0\n");
  feed("MT _ 80\n");
  reset_out();
  feed("WP 40\n");
  expect_empty("WP keys motor 2");
  {
    bool ran = false;
    for (int i = 0; i < 200; ++i) {
      protocol_poll(20);
      reset_out();
      feed("IW\n");
      if (g_out.find("IW:0") != std::string::npos) {
        ran = true;
        break;
      }
    }
    expect_true("WP 40 on motor 2 completed", ran);
  }
  reset_out();
  feed("MS\n");
  feed("CS motors 1\n");
  feed("CS servos 1\n");
  feed("SP 0 0\n");
  feed("MT A45\n");
  reset_out();
  feed("WP 10\n");
  expect_empty("WP servo-only");
  {
    bool ran = false;
    for (int i = 0; i < 200; ++i) {
      protocol_poll(20);
      reset_out();
      feed("IW\n");
      if (g_out.find("IW:0") != std::string::npos) {
        ran = true;
        break;
      }
    }
    expect_true("WP 10 on servo 1 completed", ran);
  }

  reset_out();
  feed("MS\n");
  feed("CS motors 1\n");
  feed("CS servos 0\n");
  expect_empty("restore 1+0 after servo tests");

  if (g_fail) {
    std::fprintf(stderr, "\n%d test(s) failed\n", g_fail);
    return 1;
  }
  std::puts("\nAll protocol tests passed.");
  return 0;
}
