#include "protocol.h"
#include "config_store.h"
#include "motion_api.h"

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

int main(void) {
  ProtocolIo io = {io_write, io_debug, nullptr};
  protocol_init(io);
  protocol_send_banner();
  expect_contains("startup banner", "# Slider Motion Controller V");
  expect_contains("startup help hint", "['$' for help]");

  reset_out();
  feed("\n");
  feed("\r\n");
  expect_empty("empty line silent");

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
  protocol_feed_byte('?');
  expect_contains("realtime ? compact", "#I ");
  expect_not_contains("realtime ? 1-axis has no axis separator", " | ");
  expect_not_contains("realtime ? not old format", "<I|P:");

  reset_out();
  feed("CS init_speed 25\n");
  expect_empty("CS silent");
  reset_out();
  feed("CG init_speed\n");
  expect_contains("CG init_speed", "CG:init_speed=25");
  reset_out();
  feed("GS\n");
  expect_contains("GS after CS", "GS:25.00");

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
  expect_contains("GS after CR default", "GS:50.00");
  reset_out();
  feed("ConfigReset\n");
  expect_empty("ConfigReset alias silent");

  reset_out();
  feed("CS max_accel 300\n");
  expect_empty("CS max_accel silent");
  reset_out();
  feed("CG max_accel\n");
  expect_contains("CG max_accel", "CG:max_accel=300");

  reset_out();
  feed("SS 50\n");
  expect_empty("SS under max");
  reset_out();
  feed("SS 999\n");
  expect_contains("SS over max_speed", "!E:limit SS > max_speed");
  reset_out();
  feed("GS\n");
  expect_contains("GS unchanged after SS reject", "GS:50.00");

  reset_out();
  feed("SA 200\n");
  expect_empty("SA under max");
  reset_out();
  feed("SA 999\n");
  expect_contains("SA over max_accel", "!E:limit SA > max_accel");
  reset_out();
  feed("GA\n");
  expect_contains("GA unchanged after SA reject", "GA:200.00");

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
  expect_contains("IP after move", "IP:100.00");

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
  expect_not_contains("timeout canceled SS", "GS:12.00");

  /* Halt cancels wait + following chain commands */
  reset_out();
  feed("SS 50\n");
  expect_empty("SS before halt chain");
  reset_out();
  feed("MT200;WM;SS 12\n");
  protocol_poll(5);
  reset_out();
  feed("HT\n");
  expect_empty("HT silent");
  for (int i = 0; i < 50; ++i) {
    protocol_poll(20);
  }
  reset_out();
  feed("GS\n");
  expect_not_contains("HT canceled SS", "GS:12.00");
  expect_contains("HT kept prior SS", "GS:50.00");
  reset_out();
  feed("IW\n");
  expect_contains("IW after HT", "IW:0");

  /* Bare W defaults to 1 s then continues chain */
  reset_out();
  feed("W;SS 33\n");
  expect_empty("W silent start");
  reset_out();
  feed("IW\n");
  expect_contains("IW during W", "IW:1");
  for (int i = 0; i < 20; ++i) {
    protocol_poll(50); /* 1.0 s */
  }
  reset_out();
  feed("GS\n");
  expect_contains("W then SS ran", "GS:33.00");

  /* WP / WC / WnC / Z */
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
  expect_contains("idle WP then SS immediately", "GS:12.00");

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
      if (g_out.find("GS:8.00") != std::string::npos) {
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
  expect_not_contains("WP timeout canceled SS", "GS:9.00");

  reset_out();
  feed("HT\nSE 1\nSS 50\nSA 200\n");
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
      if (g_out.find("GA:5.00") != std::string::npos) {
        ran = true;
        break;
      }
    }
    expect_true("WC then live SA", ran);
  }

  reset_out();
  feed("WnC; SS 44\n");
  expect_empty("idle WnC silent");
  reset_out();
  feed("IW\n");
  expect_contains("IW idle after WnC", "IW:0");
  reset_out();
  feed("GS\n");
  expect_contains("idle WnC then SS immediately", "GS:44.00");

  reset_out();
  feed("SS 50\nSA 200\nMT 0; WM\n");
  for (int i = 0; i < 80; ++i) {
    protocol_poll(20);
  }
  reset_out();
  feed("MT 100; WC; WnC; SS 6\n");
  {
    bool ran = false;
    for (int i = 0; i < 400; ++i) {
      protocol_poll(20);
      reset_out();
      feed("GS\n");
      if (g_out.find("GS:6.00") != std::string::npos) {
        ran = true;
        break;
      }
    }
    expect_true("WnC after cruise then SS", ran);
  }

  reset_out();
  feed("SS 50\nMT 100; WP 40; SS 11\n");
  protocol_poll(5);
  reset_out();
  feed("HT\n");
  expect_empty("HT during WP silent");
  for (int i = 0; i < 50; ++i) {
    protocol_poll(20);
  }
  reset_out();
  feed("GS\n");
  expect_not_contains("HT canceled WP SS", "GS:11.00");
  reset_out();
  feed("IW\n");
  expect_contains("IW after HT WP", "IW:0");

  reset_out();
  feed("SE 1\nZ; SS 13\n");
  expect_empty("Z silent not a wait");
  reset_out();
  feed("IW\n");
  expect_contains("IW after Z", "IW:0");
  reset_out();
  feed("GS\n");
  expect_contains("Z then SS immediately", "GS:13.00");
  reset_out();
  feed("Z 1\n");
  expect_contains("Z extra arg parse", "!E:parse");
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
  feed("Help\n");
  expect_contains("Help header", "Short Long");
  expect_contains("Help SetSpeed", "SS    SetSpeed");
  expect_contains("Help SetDebug", "SD    SetDebug");
  expect_contains("Help SetLeft", "SL    SetLeft");
  expect_contains("Help SetPosition", "SP    SetPosition");
  expect_contains("Help GetLeft", "GL    GetLeft");
  expect_contains("Help IsReady", "IR    IsReady");
  expect_contains("Help MoveJoy", "MJ    MoveJoy");
  expect_contains("Help WaitPos", "WP    WaitPos");
  expect_contains("Help WaitCruise", "WC    WaitCruise");
  expect_contains("Help WaitNotCruise", "WnC   WaitNotCruise");
  expect_contains("Help Buzzer", "Z     Buzzer");
  expect_contains("Help Halt", "H/HT  Halt");
  expect_contains("Help self", "$ /HL Help");
  reset_out();
  feed("$\n");
  expect_contains("$ help alias", "VersionProtocol");
  reset_out();
  feed("HL\n");
  expect_contains("HL alias", "IsError");
  reset_out();
  feed("Help\n");
  expect_contains("Help long", "MoveHome");
  reset_out();
  feed("H\n");
  expect_empty("H is Halt (silent)");

  reset_out();
  feed("X00\n");
  expect_empty("X00 glued = X0 0");
  reset_out();
  feed("X0\n");
  expect_empty("X0 bare toggles on");
  reset_out();
  feed("Ext1 1\n");
  expect_empty("Ext1 on");
  reset_out();
  feed("X2\n");
  expect_empty("X2 bare toggles");

  reset_out();
  feed("ZZZ\n");
  expect_contains("parse error", "!E:parse");

  /* Python-style '#' comments */
  reset_out();
  feed("# this is a comment\n");
  expect_empty("comment-only line silent");
  reset_out();
  feed("#M 12.5 20 0 100\n");
  expect_empty("pasted status line ignored");
  reset_out();
  feed("SS 77 # cruise\n");
  expect_empty("SS with inline comment silent");
  reset_out();
  feed("GS\n");
  expect_contains("GS after SS with comment", "GS:77.00");

  /* Terminal Mode: UART lines sniffed to write_debug before execute */
  reset_out();
  feed("ST 1\n");
  expect_empty("ST silent");
  reset_out();
  feed_uart("SS 44\n");
  expect_dbg_contains("UART sniff SS", "SS 44\n");
  reset_out();
  feed("GS\n");
  expect_contains("UART cmd executed after sniff", "GS:44.00");
  expect_dbg_empty("USB feed not sniffed");

  reset_out();
  feed("ST 0\n");
  feed_uart("SS 55\n");
  expect_dbg_empty("no sniff when terminal off");
  reset_out();
  feed("GS\n");
  expect_contains("UART cmd still runs without terminal", "GS:55.00");

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
  expect_contains("MJ does not change SS", "GS:80.00");

  reset_out();
  feed("SS 60\n");
  expect_empty("SS during MJ silent");
  expect_true("SS rescale joy cruise", std::fabs(motion_host_axis_cruise(0) - 30.0f) < 0.01f);
  reset_out();
  feed("SA 150\n");
  expect_empty("SA during MJ silent");
  expect_true("SA during MJ", std::fabs(motion_host_axis_accel(0) - 150.0f) < 0.01f);
  reset_out();
  feed("GS\n");
  expect_contains("SS still 60 in joy", "GS:60.00");

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
  expect_empty("1-axis ignores spd2");
  expect_true("1-axis MJ 50 cruise 30", std::fabs(motion_host_axis_cruise(0) - 30.0f) < 0.01f);

  reset_out();
  feed("ML\n");
  expect_empty("ML exits joy");
  reset_out();
  feed("GS\n");
  expect_contains("SS unchanged after ML", "GS:60.00");
  expect_true("ML uses session SS", std::fabs(motion_host_axis_cruise(0) - 60.0f) < 0.01f);

  reset_out();
  feed("MS\n");
  expect_empty("MS after ML");
  reset_out();
  feed("IP\n");
  /* sit at current pos: set slider_max to that position and command into the rail */
  {
    McStatus st;
    motion_get_status(&st);
    char cmd[64];
    std::snprintf(cmd, sizeof(cmd), "CS slider_max %.3f\n", (double)st.pos_mm);
    reset_out();
    feed(cmd);
    expect_empty("CS slider_max to current pos");
  }
  reset_out();
  feed("MJ 50\n");
  expect_empty("MJ into soft rail silent");
  reset_out();
  feed("CS slider_max 600\n");
  expect_empty("CS slider_max restore");
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
  expect_contains("GL boot envelope", "GL:0.00");
  reset_out();
  feed("GR\n");
  expect_contains("GR boot envelope", "GR:600.00");
  reset_out();
  feed("SL 120\n");
  expect_empty("SL 120 silent");
  reset_out();
  feed("GL\n");
  expect_contains("GL after SL", "GL:120.00");
  reset_out();
  feed("CG slider_min\n");
  expect_contains("envelope unchanged by SL", "CG:slider_min=0");
  reset_out();
  feed("SL\n");
  expect_empty("SL bare reset");
  reset_out();
  feed("GL\n");
  expect_contains("GL after bare SL", "GL:0.00");
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
  expect_contains("GR unchanged after reject", "GR:600.00");
  reset_out();
  feed("SL 80\n");
  expect_empty("SL 80 before CS clamp");
  reset_out();
  feed("CS slider_min 150\n");
  expect_empty("CS slider_min 150");
  reset_out();
  feed("GL\n");
  expect_contains("CS clamps session left", "GL:150.00");
  reset_out();
  feed("CS slider_min 0\n");
  expect_empty("CS slider_min restore 0");
  reset_out();
  feed("GL\n");
  expect_contains("CS widen envelope keeps window", "GL:150.00");
  reset_out();
  feed("SL\n");
  expect_empty("SL bare after CS clamp test");
  reset_out();
  feed("GL\n");
  expect_contains("GL full rail again", "GL:0.00");

  /* SL none: session cleared; GL/clip fall back to envelope when set */
  reset_out();
  feed("SL 120\n");
  expect_empty("SL 120 before none");
  reset_out();
  feed("SL none\n");
  expect_empty("SL none with envelope");
  reset_out();
  feed("GL\n");
  expect_contains("GL after SL none uses envelope", "GL:0.00");
  reset_out();
  feed("CS slider_min none\n");
  expect_empty("CS slider_min none");
  reset_out();
  feed("SL none\n");
  expect_empty("SL none with open envelope");
  reset_out();
  feed("GL\n");
  expect_contains("GL open after none+none envelope", "GL:-");
  reset_out();
  feed("CS slider_min 0\n");
  feed("SL\n");
  expect_empty("restore slider_min and bare SL");
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
  feed("SL 100\n");
  expect_empty("SL 100 for inward MJ");
  reset_out();
  feed("MJ -80\n");
  expect_empty("MJ negative below window silent");
  motion_stub_tick_ms(800);
  reset_out();
  feed("IP\n");
  expect_contains("MJ out clamped inward to left wall", "IP:100.00");
  reset_out();
  feed("SL\n");
  feed("MS\n");
  expect_empty("reset window after inward test");

  /* --- axis2 protocol (HOST_TEST builds with PIN_AXIS2_SUPPORTED) --- */
  reset_out();
  feed("IA\n");
  expect_contains("IA before axis2", "IA:1");

  reset_out();
  feed("CS axis2_use 1\n");
  expect_empty("CS axis2_use 1");
  reset_out();
  feed("CG axis2_use\n");
  expect_contains("CG axis2_use", "CG:axis2_use=1");

  reset_out();
  feed("IA\n");
  expect_contains("IA after axis2", "IA:2");
  reset_out();
  feed("Axis\n");
  expect_contains("Axis alias", "IA:2");

  reset_out();
  protocol_send_banner();
  expect_contains("banner 2 Axis", "- 2 Axis");
  expect_not_contains("banner no name yet", "Foo - Slider");

  reset_out();
  feed("CS name Foo\n");
  expect_empty("CS name Foo");
  reset_out();
  feed("CG name\n");
  expect_contains("CG name", "CG:name=Foo");
  reset_out();
  feed("CG unit_name\n");
  expect_contains("CG unit_name default", "CG:unit_name=mm");
  reset_out();
  feed("CS unit_name deg\n");
  expect_empty("CS unit_name deg");
  reset_out();
  feed("CG unit_name\n");
  expect_contains("CG unit_name deg", "CG:unit_name=deg");
  reset_out();
  feed("CS steps_per_unit 200\n");
  expect_empty("CS steps_per_unit");
  reset_out();
  feed("CG steps_per_unit\n");
  expect_contains("CG steps_per_unit", "CG:steps_per_unit=200");
  reset_out();
  feed("CS steps_per_mm 320\n");
  expect_empty("CS steps_per_mm alias");
  reset_out();
  feed("CG steps_per_mm\n");
  expect_contains("CG steps_per_mm alias", "CG:steps_per_mm=320");
  reset_out();
  protocol_send_banner();
  expect_contains("named 2-axis banner", "# Foo - Slider Motion Controller V");
  expect_contains("named banner has 2 Axis", "- 2 Axis");

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
  protocol_feed_byte('?');
  expect_contains("moving ? grouped axis2", " | ");
  {
    /* |d2|/|d1| = 50/100 = 0.5 */
    float v0 = motion_host_axis_cruise(0);
    float v1 = motion_host_axis_cruise(1);
    float a0 = motion_host_axis_accel(0);
    float a1 = motion_host_axis_accel(1);
    expect_true("dual MT cruise ratio ~0.5", std::fabs(v1 - v0 * 0.5f) < 0.01f);
    expect_true("dual MT accel ratio ~0.5", std::fabs(a1 - a0 * 0.5f) < 0.01f);
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
    expect_true("SA mid-move axis1=100", std::fabs(a0 - 100.0f) < 0.01f);
    expect_true("SA mid-move axis2 keeps ratio", std::fabs(a1 - 50.0f) < 0.01f);
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
      if (g_out.find("GS:4.00") != std::string::npos) {
        ran = true;
        break;
      }
    }
    expect_true("2-axis WP keys off axis1", ran);
  }
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
  expect_empty("ML axis mask 1");
  reset_out();
  feed("MR 2\n");
  expect_empty("MR axis mask 2");
  reset_out();
  feed("MH 1\n");
  expect_empty("MH axis 1");

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
    expect_true("snapshot axis0 moving", st.vel_mm_s > 0.0f);
    expect_true("snapshot axis1 stopped", std::fabs(st.vel_mm_s_2) < 0.01f);
  }
  expect_true("snapshot axis0 cruise 32", std::fabs(motion_host_axis_cruise(0) - 32.0f) < 0.01f);

  reset_out();
  feed("MJ 40 -20\n");
  expect_empty("MJ independent signs");
  {
    McStatus st;
    motion_get_status(&st);
    expect_true("MJ 40 vel+", st.vel_mm_s > 0.0f);
    expect_true("MJ -20 vel-", st.vel_mm_s_2 < 0.0f);
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
  expect_contains("GS after dual MJ MS", "GS:50.00");

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
  expect_contains("GL dual skip axis2", "GL:0.00 40.00");
  reset_out();
  feed("SL none 50\n");
  expect_empty("SL none axis1 set axis2");
  reset_out();
  feed("GL\n");
  expect_contains("GL after SL none 50", "GL:0.00 50.00");
  reset_out();
  feed("SR 200\n");
  feed("SL _ none\n");
  expect_empty("SL clear axis2 only");
  reset_out();
  feed("GL\n");
  expect_contains("GL after SL _ none", "GL:0.00 0.00");
  reset_out();
  feed("SL\n");
  expect_empty("SL bare resets both axes");
  reset_out();
  feed("GL\n");
  expect_contains("GL dual after bare SL", "GL:0.00 0.00");

  reset_out();
  feed("X4 1\n");
  expect_contains("X4 invalid", "!E:parse");
  reset_out();
  feed("X6 1\n");
  expect_contains("X6 invalid", "!E:parse");

  reset_out();
  feed("IX\n");
  expect_contains("IX has STEP2 when axis2", "DRV_STEP2");
  reset_out();
  feed("VG\n");
  expect_contains("VG has STEP2 when axis2", "PIN_DRV_STEP2=");

  reset_out();
  protocol_feed_byte('?');
  expect_contains("realtime ? with pos2", "#I ");
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
  feed("CS axis2_use 0\n");
  expect_empty("CS axis2_use 0 restore 1-axis");
  reset_out();
  feed("IA\n");
  expect_contains("IA restored 1-axis", "IA:1");
  reset_out();
  feed("IX\n");
  expect_not_contains("IX no STEP2 when 1-axis", "DRV_STEP2");
  reset_out();
  feed("VG\n");
  expect_not_contains("VG no STEP2 when 1-axis", "PIN_DRV_STEP2=");
  reset_out();
  feed("IP\n");
  expect_contains("IP single-field 1-axis", "IP:");
  expect_not_contains("IP no second field when 1-axis", "IP:0.00 0.00");
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
  expect_not_contains("1-axis banner no 2 Axis", "2 Axis");
  expect_contains("named 1-axis banner", "# Foo - Slider Motion Controller V");
  reset_out();
  feed("RB\n");
  expect_empty("RB silent on host");
  reset_out();
  feed("Help\n");
  expect_contains("Help lists RB", "RB");
  expect_contains("Help lists Reboot", "Reboot");

  reset_out();
  feed("SE 1\n");
  feed("SP\n");
  expect_empty("bare SP silent");
  reset_out();
  feed("IP\n");
  expect_contains("bare SP zeros pose", "IP:0.00");
  reset_out();
  feed("SP 100\n");
  expect_empty("SP 100 silent");
  reset_out();
  feed("IP\n");
  expect_contains("SP 100 pose", "IP:100.00");
  reset_out();
  feed("SP 0\n");
  expect_empty("SP 0 silent");
  reset_out();
  feed("IP\n");
  expect_contains("SP 0 pose", "IP:0.00");
  reset_out();
  feed("CS home_mode 3\n");
  expect_empty("CS home_mode 3 stall");
  reset_out();
  feed("CG home_mode\n");
  expect_contains("CS home_mode 3 persists", "CG:home_mode=3");
  reset_out();
  feed("CS SW_HOME_use 1\n");
  expect_empty("legacy SW_HOME_use ignored");
  config_migrate_legacy_home_modes();
  reset_out();
  feed("CG home_mode\n");
  expect_contains("legacy migrate 3 to 1", "CG:home_mode=1");
  reset_out();
  feed("CS home_mode 4\n");
  feed("CS SW_HOME_use_2 0\n");
  config_migrate_legacy_home_modes();
  reset_out();
  feed("CG home_mode\n");
  expect_contains("legacy migrate 4 to 2", "CG:home_mode=2");
  reset_out();
  feed("CS home_mode 0\n");
  expect_empty("CS home_mode 0 restore");
  reset_out();
  feed("CS axis2_use 1\n");
  feed("SP 10 20\n");
  expect_empty("SP dual silent");
  reset_out();
  feed("IP\n");
  expect_contains("SP dual pose", "IP:10.00 20.00");
  reset_out();
  feed("SP _ 0\n");
  expect_empty("SP skip axis1");
  reset_out();
  feed("IP\n");
  expect_contains("SP skip keeps axis1", "IP:10.00 0.00");
  reset_out();
  feed("MT 50\n");
  reset_out();
  feed("SP 0\n");
  expect_contains("SP rejected while moving", "!E:busy");
  reset_out();
  feed("MS\n");
  feed("CS axis2_use 0\n");
  expect_empty("restore 1-axis after SP tests");

  if (g_fail) {
    std::fprintf(stderr, "\n%d test(s) failed\n", g_fail);
    return 1;
  }
  std::puts("\nAll protocol tests passed.");
  return 0;
}
