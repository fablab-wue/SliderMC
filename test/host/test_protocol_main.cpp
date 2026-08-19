#include "protocol.h"
#include "config_store.h"
#include "motion_api.h"

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
  expect_contains("Help IsReady", "IR    IsReady");
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
  feed("CS path_buffer_size 64000\n");

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
  feed("SS 10\n");
  expect_contains("SS rejected while path active", "!E:busy");
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

  if (g_fail) {
    std::fprintf(stderr, "\n%d test(s) failed\n", g_fail);
    return 1;
  }
  std::puts("\nAll protocol tests passed.");
  return 0;
}
