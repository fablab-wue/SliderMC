#include "board_config_fs.h"
#include "config_store.h"
#include "config_defaults.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>

#ifndef HOST_TEST

#include <Arduino.h>
#include <LittleFS.h>

static const char *k_ini_path = "/mc.ini";
static const char *k_ini_tmp = "/mc.ini.tmp";

static bool ensure_mounted(void) {
  if (LittleFS.begin()) {
    return true;
  }
  LittleFS.format();
  return LittleFS.begin();
}

static void trim_inplace(char *s) {
  char *start = s;
  while (*start && isspace((unsigned char)*start)) {
    ++start;
  }
  if (start != s) {
    memmove(s, start, strlen(start) + 1);
  }
  size_t n = strlen(s);
  while (n > 0 && isspace((unsigned char)s[n - 1])) {
    s[--n] = 0;
  }
}

static void apply_ini_line(char *line) {
  trim_inplace(line);
  if (line[0] == 0 || line[0] == ';' || line[0] == '#') {
    return;
  }
  char *eq = strchr(line, '=');
  if (!eq) {
    return;
  }
  *eq = 0;
  char *key = line;
  char *val = eq + 1;
  trim_inplace(key);
  trim_inplace(val);
  if (key[0] == 0) {
    return;
  }
  (void)config_set_key(key, val); /* unknown keys ignored */
}

bool board_config_load_from_fs(void) {
  if (!ensure_mounted()) {
    return false;
  }
  File f = LittleFS.open(k_ini_path, "r");
  if (!f) {
    return false;
  }

  char line[CFG_LINE_MAX];
  size_t len = 0;
  while (f.available()) {
    int c = f.read();
    if (c < 0) {
      break;
    }
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      line[len] = 0;
      apply_ini_line(line);
      len = 0;
      continue;
    }
    if (len + 1 < sizeof(line)) {
      line[len++] = (char)c;
    }
  }
  if (len > 0) {
    line[len] = 0;
    apply_ini_line(line);
  }
  f.close();
  return true;
}

typedef struct {
  File *f;
  bool ok;
} SaveCtx;

static void save_one(const char *key, const char *value, void *ctx) {
  SaveCtx *sc = (SaveCtx *)ctx;
  if (!sc->ok || !key || !value) {
    return;
  }
  /* name may be blank; other keys must have a value (guards broken float snprintf). */
  if (value[0] == 0 && strcmp(key, "name") != 0) {
    sc->ok = false;
    return;
  }
  if (sc->f->printf("%s=%s\n", key, value) <= 0) {
    sc->ok = false;
  }
}

bool board_config_save_to_fs(void) {
  if (!ensure_mounted()) {
    return false;
  }

  File f = LittleFS.open(k_ini_tmp, "w");
  if (!f) {
    return false;
  }

  SaveCtx sc = {&f, true};
  if (f.printf("; SliderMC configuration\n") <= 0) {
    sc.ok = false;
  }
  if (sc.ok) {
    config_foreach(save_one, &sc);
  }
  f.flush();
  f.close();

  if (!sc.ok) {
    LittleFS.remove(k_ini_tmp);
    return false;
  }

  LittleFS.remove(k_ini_path);
  if (!LittleFS.rename(k_ini_tmp, k_ini_path)) {
    LittleFS.remove(k_ini_tmp);
    return false;
  }
  return true;
}

#else /* HOST_TEST */

bool board_config_load_from_fs(void) { return false; }

/* No-op success so host CS tests stay silent. */
bool board_config_save_to_fs(void) { return true; }

#endif
