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
  expect_empty("MT skip axis1 token accepted");
  reset_out();
  feed("MT _ 40\n");
  expect_empty("MT underscore skip accepted");

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
  feed("PC\n");
  feed("PD 100 200\n");
  expect_empty("PD dual sample");
  reset_out();
  feed("PN\n");
  expect_contains("PN after dual PD", "PN:1");
  reset_out();
  feed("PD * N\n");
  expect_empty("PD skip tokens become 0");

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
  /* idle: #I pos1 pos2 — at least two spaces separating three tokens after #I */
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
      /* "#I a b" → two spaces minimum in the numeric part */
      int spaces = 0;
      for (char ch : line) {
        if (ch == ' ') {
          ++spaces;
        }
      }
      if (spaces < 2) {
        std::fprintf(stderr, "FAIL idle ? needs pos1 pos2, got:\n%s\n", line.c_str());
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

  if (g_fail) {
    std::fprintf(stderr, "\n%d test(s) failed\n", g_fail);
    return 1;
  }
  std::puts("\nAll protocol tests passed.");
  return 0;
}
