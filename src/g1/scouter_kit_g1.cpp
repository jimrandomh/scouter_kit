#include "scouter_kit_g1.hpp"

// Configuration: define any of these before compiling to change it.
// SCOUTER_G1_EMULATOR: talk to two emulated arms instead of BlueZ (no
// libsystemd needed). Define it for every file that includes the header.
// SCOUTER_G1_NO_BLUEZ: no BlueZ code; Init needs Connection::transport.
// SCOUTER_G1_MIC: the microphone (Arm::MicStart); link liblc3 from
// third_party/liblc3 (C, -DLC3_PLUS=0 -DLC3_PLUS_HR=0).
#ifndef SCOUTER_G1_MAX_PACKET
#define SCOUTER_G1_MAX_PACKET 236 // seq + host code: 9 HCI fragments of 27 B
#endif
#ifndef SCOUTER_G1_IN_FLIGHT
#define SCOUTER_G1_IN_FLIGHT 12 // packets sent and not acked, per arm
#endif
#ifndef SCOUTER_G1_STAGING_BYTES
#define SCOUTER_G1_STAGING_BYTES 8000 // arena kept for black and restores
#endif
#ifndef SCOUTER_G1_POOL_WORDS
#define SCOUTER_G1_POOL_WORDS 1024 // arena words for the host code's constants
#endif
#ifndef SCOUTER_G1_LINK_MIN
#define SCOUTER_G1_LINK_MIN 6 // active link: interval asked for, x 1.25 ms
#endif
#ifndef SCOUTER_G1_LINK_MAX
#define SCOUTER_G1_LINK_MAX 6
#endif
#ifndef SCOUTER_G1_IDLE_LINK
#define SCOUTER_G1_IDLE_LINK 40 // idle link: 50 ms
#endif
#ifndef SCOUTER_G1_IDLE_AFTER_MS
#define SCOUTER_G1_IDLE_AFTER_MS 5000 // without traffic, then the idle link
#endif
#ifndef SCOUTER_G1_BLACK_OFF_MS
#define SCOUTER_G1_BLACK_OFF_MS 30000 // an all-black eye in standby this long, then off
#endif
#ifndef SCOUTER_G1_MATCH_BRIGHTNESS
#define SCOUTER_G1_MATCH_BRIGHTNESS 1 // a panel guard's cap on one eye becomes the other's BRIGHTNESS_MAX
#endif

#include <algorithm>
#include <atomic>
#include <bit>
#include <chrono>
#include <condition_variable>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <random>
#include <set>
#include <thread>
#include <unordered_map>

#include "../../third_party/concurrentqueue/concurrentqueue.h"
#include "../../third_party/tinycrypt/sha256.c"
#include "../../third_party/tinycrypt/utils.c"
#include "../vm/vm.h"

#ifdef SCOUTER_G1_MIC
#include <cmath>
#include <numbers>

#include "../../third_party/liblc3/include/lc3.h"
#endif

#if !defined(SCOUTER_G1_EMULATOR) && !defined(SCOUTER_G1_NO_BLUEZ)
#ifndef __linux__
#error "scouter_kit_g1: only Linux + BlueZ (or define SCOUTER_G1_EMULATOR or SCOUTER_G1_NO_BLUEZ)"
#endif
#include <cerrno>
#include <fcntl.h>
#include <poll.h>
#include <sys/eventfd.h>
#include <systemd/sd-bus.h>
#include <time.h>
#include <unistd.h>
#endif

static_assert(std::endian::native == std::endian::little,
              "Arm::arena holds little-endian words");

namespace scouter_kit::g1 {

Arm left, right;

int64_t NowUs() {
  using namespace std::chrono;
  return duration_cast<microseconds>(steady_clock::now().time_since_epoch())
      .count();
}

namespace {

// ---------------------------------------------------------------- constants

constexpr uint32_t kArenaBytes = 262144;
constexpr int kPanelW = 640;
constexpr int kPanelH = 480;
constexpr int kRowBytes = kPanelW / 2;

// The library's own memory at the top of the arena: constants, operands for
// a DRAW, a word for the time of a present, glyph text for GLYPHS, and the
// zero staging rows.
constexpr int kStagingRows = SCOUTER_G1_STAGING_BYTES / kRowBytes;
constexpr uint32_t kStagingBytes = uint32_t(kStagingRows) * kRowBytes;
constexpr Address kStaging = kArenaBytes - kStagingBytes;
constexpr uint32_t kTextBytes = 256;
constexpr Address kText = kStaging - kTextBytes;
constexpr Address kTimeAddr = kText - 4;
constexpr ArenaWord kTimeWord = ArenaWord(kTimeAddr / 4);
constexpr Address kDrawAddr = kTimeAddr - 32; // a DRAW's own operands
constexpr int kPoolWords = SCOUTER_G1_POOL_WORDS;
constexpr Address kPool = kDrawAddr - 4 * uint32_t(kPoolWords);
constexpr Address kHostBottom = 64; // below: r0 ... r14
constexpr Address kHostTop = kPool;
static_assert(kStagingRows >= 1 && kStagingBytes % 4 == 0 && kPoolWords >= 16 &&
              kHostTop > kHostBottom + 65536);

enum Op : uint8_t {
  OP_WRITE = 0x01,
  OP_WRITE_RLE = 0x02,
  OP_COPY = 0x03,
  OP_UNPACK = 0x04,
  OP_MOV = 0x10,
  OP_ADD = 0x11,
  OP_SUB = 0x12,
  OP_MUL = 0x13,
  OP_DIVU = 0x14,
  OP_MODU = 0x15,
  OP_DIVS = 0x16,
  OP_MODS = 0x17,
  OP_AND = 0x18,
  OP_OR = 0x19,
  OP_XOR = 0x1A,
  OP_SHL = 0x1B,
  OP_SHR = 0x1C,
  OP_SAR = 0x1D,
  OP_LD32 = 0x20,
  OP_ST32 = 0x24,
  OP_JMP = 0x30,
  OP_CALL = 0x31,
  OP_RET = 0x32,
  OP_END = 0x33,
  OP_HALT = 0x34,
  OP_BEQ = 0x35,
  OP_BNE = 0x36,
  OP_BLTU = 0x37,
  OP_BGEU = 0x38,
  OP_BLTS = 0x39,
  OP_BGES = 0x3A,
  OP_SLEEP_UNTIL = 0x40,
  OP_WAIT_EVENT = 0x41,
  OP_RAISE = 0x42,
  OP_NOTIFY = 0x43,
  OP_SYS_READ = 0x44,
  OP_SYS_WRITE = 0x45,
  OP_FILL = 0x50,
  OP_BLIT = 0x51,
  OP_GLYPHS = 0x52,
  OP_DRAW = 0x53,
  OP_LINE = 0x54,
  OP_VIDEO = 0x55,
  OP_ASSET_LOAD = 0x56,
};

enum Control : uint8_t {
  CTRL_HELLO = 0x01,
  CTRL_PING = 0x02,
  CTRL_STATUS = 0x03,
  CTRL_RESET_SESSION = 0x04,
  CTRL_LINK = 0x05,
  CTRL_LOG_READ = 0x06,
  CTRL_HEALTH_OK = 0x07,
  CTRL_REBOOT = 0x08,
  CTRL_PROPS = 0x0A,
  CTRL_PANEL_READ = 0x0B,
  CTRL_ARENA_CRC = 0x0C,
  CTRL_VM_START = 0x10,
  CTRL_VM_STOP = 0x11,
  CTRL_VM_STATUS = 0x12,
  CTRL_MIC = 0x13,
  CTRL_EVENTS = 0x14,
  CTRL_PANEL_GUARD = 0x16,
  CTRL_ASSET_INFO = 0x17,
  CTRL_ASSET_LIST = 0x18,
  CTRL_ASSET_PUT = 0x19,
  CTRL_ASSET_DELETE = 0x1A,
  CTRL_ASSET_FORMAT = 0x1B,
  CTRL_FLASH_CRC = 0x1C,
};

enum EventCode : uint8_t {
  EVT_ACK = 0x01,
  EVT_ERROR = 0x02,
  EVT_HALT = 0x03,
  EVT_NOTIFY = 0x04,
  EVT_NOTICE = 0x05,
  EVT_MIC_DATA = 0x06,
  EVT_INPUT = 0x07,
};

enum Channel { CH_CMD = 0, CH_CTRL = 1 };

constexpr int kHelloBytes = 35;
constexpr size_t kMicKeep = 200;      // frames for MicRead: 2 s
constexpr uint32_t kMicConceal = 100; // at most 1 s of concealment per gap
constexpr int64_t kMicRenewUs = 800000; // the arm holds the microphone 2 s per MIC 1
std::atomic<bool> g_hold_renewals{false}; // emulator::HoldMicRenewals

const char *const kArmName[2] = {"left", "right"};
const char *const kCodeName[14] = {"ok",       "malformed", "invalid op", "arena", "format",
                                   "argument", "window",    "safe mode",  "font",  "stack",
                                   "budget",   "panel",     "mic",        "asset"};

const char *CodeName(int code) { return code >= 0 && code < 14 ? kCodeName[code] : "?"; }

// Operand count of an instruction, -1 for WRITE, WRITE_RLE and unknown ops.
int Operands(uint8_t op) {
  switch (op) {
  case OP_COPY: return 3;
  case OP_UNPACK: return 4;
  case OP_MOV: return 2;
  case OP_JMP:
  case OP_CALL:
  case OP_RET:
  case OP_END:
  case OP_HALT: return 0;
  case OP_SLEEP_UNTIL:
  case OP_RAISE: return 1;
  case OP_WAIT_EVENT:
  case OP_NOTIFY:
  case OP_SYS_READ:
  case OP_SYS_WRITE: return 2;
  case OP_FILL:
  case OP_LINE: return 7;
  case OP_BLIT: return 12;
  case OP_GLYPHS: return 9;
  case OP_DRAW: return 8;
  case OP_VIDEO: return 10;
  case OP_ASSET_LOAD: return 4;
  default:
    if (op >= OP_ADD && op <= OP_SAR) return 3;
    if (op >= OP_LD32 && op <= OP_ST32 + 3) return 3;
    if (op >= OP_BEQ && op <= OP_BGES) return 2;
    return -1;
  }
}

bool HasTarget(uint8_t op) { return op == OP_JMP || op == OP_CALL || (op >= OP_BEQ && op <= OP_BGES); }

const char *OpName(uint8_t op) {
  static const std::map<uint8_t, const char *> names = {
      {OP_WRITE, "WRITE"},   {OP_WRITE_RLE, "WRITE_RLE"}, {OP_COPY, "COPY"},
      {OP_UNPACK, "UNPACK"}, {OP_MOV, "MOV"},             {OP_CALL, "CALL"},
      {OP_END, "END"},       {OP_SLEEP_UNTIL, "SLEEP_UNTIL"}, {OP_RAISE, "RAISE"},
      {OP_NOTIFY, "NOTIFY"}, {OP_SYS_READ, "SYS_READ"},   {OP_SYS_WRITE, "SYS_WRITE"},
      {OP_FILL, "FILL"},     {OP_BLIT, "BLIT"},           {OP_GLYPHS, "GLYPHS"},
      {OP_DRAW, "DRAW"},     {OP_LINE, "LINE"},           {OP_VIDEO, "VIDEO"},
      {OP_ASSET_LOAD, "ASSET_LOAD"}};
  auto it = names.find(op);
  if (it != names.end()) return it->second;
  if (op >= OP_ADD && op <= OP_SAR) return "arithmetic";
  if (op >= OP_LD32 && op <= OP_ST32 + 3) return op < OP_ST32 ? "LD" : "ST";
  return "instruction";
}

// One unit of host code that never spans packets: an instruction or a few.
struct Cmd {
  uint16_t size = 0;
  std::array<uint8_t, 244> b{};
  void Add(const uint8_t *p, size_t n) {
    std::memcpy(&b[size], p, n);
    size = uint16_t(size + n);
  }
};

void PutIns(std::vector<uint8_t> &out, uint8_t op, const ArenaWord *w, int k, uint32_t target) {
  out.push_back(op);
  for (int i = 0; i < k; i += 2) {
    int hi = std::min<int>(w[i], 15), lo = i + 1 < k ? std::min<int>(w[i + 1], 15) : 0;
    out.push_back(uint8_t(hi << 4 | lo));
  }
  if (HasTarget(op))
    for (int s = 0; s < 24; s += 8) out.push_back(uint8_t(target >> s));
  for (int i = 0; i < k; i++)
    if (w[i] >= 15) {
      out.push_back(uint8_t(w[i]));
      out.push_back(uint8_t(w[i] >> 8));
    }
}

void PutWrite(std::vector<uint8_t> &out, Address addr, const uint8_t *data, size_t n) {
  out.insert(out.end(), {OP_WRITE, uint8_t(addr), uint8_t(addr >> 8), uint8_t(addr >> 16), uint8_t(n)});
  out.insert(out.end(), data, data + n);
}

void PutWriteRle(std::vector<uint8_t> &out, Address addr, uint32_t len, const uint8_t *data, size_t n) {
  out.insert(out.end(), {OP_WRITE_RLE, uint8_t(addr), uint8_t(addr >> 8), uint8_t(addr >> 16),
                         uint8_t(len), uint8_t(len >> 8), uint8_t(n)});
  out.insert(out.end(), data, data + n);
}

Cmd ToCmd(const std::vector<uint8_t> &bytes) {
  Cmd c;
  if (bytes.size() <= c.b.size()) c.Add(bytes.data(), bytes.size());
  return c;
}

Cmd Ins(uint8_t op, std::initializer_list<ArenaWord> words, uint32_t target = 0) {
  std::vector<uint8_t> v;
  PutIns(v, op, words.begin(), int(words.size()), target);
  return ToCmd(v);
}

// ---------------------------------------------------------------- helpers

std::atomic<bool> g_log{false}; // Connection::log

void Log(const char *format, ...) {
  if (!g_log) return;
  char text[512];
  va_list ap;
  va_start(ap, format);
  std::vsnprintf(text, sizeof(text), format, ap);
  va_end(ap);
  std::fprintf(stderr, "[scouter_kit_g1 %.3f] %s\n", double(NowUs()) / 1e6, text);
}

uint16_t Get16(const uint8_t *p) { return uint16_t(p[0] | p[1] << 8); }
uint32_t Get32(const uint8_t *p) {
  return uint32_t(p[0]) | uint32_t(p[1]) << 8 | uint32_t(p[2]) << 16 |
         uint32_t(p[3]) << 24;
}
void Put16(std::vector<uint8_t> &v, uint32_t x) {
  v.push_back(uint8_t(x));
  v.push_back(uint8_t(x >> 8));
}
void Put32(std::vector<uint8_t> &v, uint32_t x) {
  Put16(v, x);
  Put16(v, x >> 16);
}

bool SeqBefore(uint16_t a, uint16_t b) { return uint16_t(a - b) >= 0x8000; }

// An ASSET_LIST entry (VM.md §6.10).
AssetEntry EntryOf(const uint8_t *p) {
  AssetEntry e{};
  std::memcpy(e.id.b, p, 16);
  e.offset = Get32(p + 16);
  e.size = Get32(p + 20);
  e.crc = Get32(p + 24);
  e.type = p[28];
  return e;
}

uint32_t Crc32(uint32_t crc, const uint8_t *p, size_t n) {
  crc = ~crc;
  while (n--) {
    crc ^= *p++;
    for (int k = 0; k < 8; k++)
      crc = (crc & 1) ? (crc >> 1) ^ 0xEDB88320u : crc >> 1;
  }
  return ~crc;
}

int PutVarint(uint32_t v, uint8_t *out) {
  int n = 0;
  while (v >= 0x80) {
    out[n++] = uint8_t(v | 0x80);
    v >>= 7;
  }
  out[n++] = uint8_t(v);
  return n;
}

// PackBits: runs of 2..128 equal bytes, literals of 1..128 bytes.
void PackBits(const uint8_t *d, int n, std::vector<uint8_t> &out) {
  int i = 0;
  while (i < n) {
    int run = 1;
    while (i + run < n && run < 128 && d[i + run] == d[i]) run++;
    if (run >= 2) {
      out.push_back(uint8_t(257 - run));
      out.push_back(d[i]);
      i += run;
      continue;
    }
    int start = i;
    while (i < n && i - start < 128) {
      if (i + 1 < n && d[i + 1] == d[i]) break;
      i++;
    }
    out.push_back(uint8_t(i - start - 1));
    out.insert(out.end(), d + start, d + i);
  }
}

} // namespace

// ---------------------------------------------------------------- Program

Program::Program(Address table, Address pool, int pool_words, Address code)
    : table_address(table), pool_address(pool), code_address(code), pool_words(pool_words) {
  handlers_.fill(-1);
  if (pool_words < 0 || uint64_t(pool) + 4ull * uint64_t(pool_words) > kArenaBytes ||
      uint64_t(table) + 64 > kArenaBytes || code >= kArenaBytes)
    Fail("layout outside the arena");
}

Program::Program(Address address, int pool_words)
    : Program(address, address + 64, pool_words, address + 64 + 4 * uint32_t(std::max(pool_words, 0))) {}

void Program::Fail(const std::string &text) {
  if (error_.empty()) error_ = text;
}

Program::Label Program::NewLabel() {
  labels_.push_back(-1);
  return Label{int(labels_.size()) - 1};
}

void Program::Bind(Label l) {
  if (l.id < 0 || l.id >= int(labels_.size())) return Fail("Bind: not a label of this program");
  if (labels_[size_t(l.id)] >= 0) return Fail("Bind: label bound twice");
  Address here = Here();
  labels_[size_t(l.id)] = here;
  for (const Fixup &f : fixups_)
    if (f.label == l.id)
      for (int s = 0; s < 3; s++) code[f.at + size_t(s)] = uint8_t(here >> (8 * s));
  for (int t = 0; t < 16; t++)
    if (handlers_[size_t(t)] == l.id) table[size_t(t)] = here;
}

Program::Label Program::At(Address address) {
  Label l = NewLabel();
  labels_[size_t(l.id)] = address;
  return l;
}

void Program::On(Trigger trigger, Label l) {
  if (trigger > 15 || l.id < 0 || l.id >= int(labels_.size())) return Fail("On: bad trigger or label");
  if (trigger == TRIGGER_START && policy_start_ >= 0) return PolicyStart(l);
  handlers_[trigger] = l.id;
  table[trigger] = labels_[size_t(l.id)] >= 0 ? Address(labels_[size_t(l.id)]) : 0;
}

ArenaWord Program::Const(uint32_t value) {
  for (size_t i = 0; i < pool.size(); i++)
    if (pool[i] == value && std::find(vars_.begin(), vars_.end(), i) == vars_.end())
      return ArenaWord((pool_address + 4 * i) / 4);
  if (int(pool.size()) >= pool_words || pool_address % 4) {
    Fail(pool_address % 4 ? "the pool address is not a multiple of 4" : "the pool is full");
    return 0;
  }
  pool.push_back(value);
  return ArenaWord((pool_address + 4 * (pool.size() - 1)) / 4);
}

// A pool word the program writes: never a Const, left out of the mirror's checks.
ArenaWord Program::Var() {
  if (int(pool.size()) >= pool_words || pool_address % 4) {
    Fail(pool_address % 4 ? "the pool address is not a multiple of 4" : "the pool is full");
    return 0;
  }
  vars_.push_back(pool.size());
  pool.push_back(0);
  Address at = pool_address + 4 * Address(pool.size() - 1);
  Changes(at, 4);
  return ArenaWord(at / 4);
}

void Program::PanelPolicy(Label redraw) {
  if (policy_start_ >= 0) return Fail("PanelPolicy: twice");
  hidden_ = Var();
  scratch_ = Var();
  ArenaWord s = scratch_, h = hidden_, zero = Const(0), one = Const(1);
  Label over = NewLabel(), check = NewLabel(), hide = NewLabel(), handler = NewLabel(), start = NewLabel(),
        none = NewLabel();
  Jmp(over);
  Bind(check); // s = 1 when the panel may show again
  SysRead(s, Const(PROP_IN_CASE));
  Beq(s, one, hide);
  SysRead(s, Const(PROP_WORN));
  Beq(s, zero, hide);
  Mov(s, h);
  Mov(h, zero);
  Ret();
  Bind(hide);
  Mov(h, one);
  SysWrite(Const(PROP_PANEL), zero);
  Mov(s, zero);
  Ret();
  Bind(handler);
  Call(check);
  if (redraw.id >= 0) Bne(s, zero, redraw);
  End();
  Bind(start); // START: the policy, then the program's own handler
  Call(check);
  Jmp(none);
  policy_start_ = int(fixups_.size()) - 1;
  Bind(none);
  End();
  Bind(over);
  On(TRIGGER_CASE, handler);
  On(TRIGGER_WEAR, handler);
  int user = handlers_[TRIGGER_START];
  handlers_[TRIGGER_START] = start.id;
  table[TRIGGER_START] = Address(labels_[size_t(start.id)]);
  if (user >= 0) PolicyStart(Label{user});
}

// The jump at the end of PanelPolicy's START code goes to the program's START handler.
void Program::PolicyStart(Label l) {
  Fixup &f = fixups_[size_t(policy_start_)];
  f.label = l.id;
  int64_t at = labels_[size_t(l.id)];
  if (at >= 0)
    for (int s = 0; s < 3; s++) code[f.at + size_t(s)] = uint8_t(at >> (8 * s));
}

// The destination column of a DRAW or VIDEO: off the panel while PanelPolicy holds it off.
ArenaWord Program::PolicyX(ArenaWord x) {
  if (policy_start_ < 0) return x;
  Label go = NewLabel();
  Mov(scratch_, x);
  Beq(hidden_, Const(0), go);
  Mov(scratch_, Const(0x7FFF));
  Bind(go);
  return scratch_;
}

void Program::Changes(Address address, uint32_t length) {
  if (uint64_t(address) + length > kArenaBytes) return Fail("Changes: outside the arena");
  changes.push_back({address, length});
}

Address Program::Here() const { return code_address + Address(code.size()); }

void Program::Target(Label l) {
  if (l.id < 0 || l.id >= int(labels_.size())) {
    Fail("a jump without a label of this program");
    code.insert(code.end(), {0, 0, 0});
    return;
  }
  int64_t at = labels_[size_t(l.id)];
  if (at < 0) fixups_.push_back({code.size(), l.id});
  for (int s = 0; s < 3; s++) code.push_back(uint8_t(at < 0 ? 0 : at >> (8 * s)));
}

void Program::Op(uint8_t opcode, std::initializer_list<ArenaWord> operands, Label target) {
  int k = Operands(opcode);
  if (k < 0 || size_t(k) != operands.size()) {
    char text[80];
    std::snprintf(text, sizeof(text), "instruction 0x%02x: unknown, or not %zu operands", opcode, operands.size());
    Fail(text);
    return;
  }
  std::vector<ArenaWord> ops(operands);
  if (policy_start_ >= 0 && (opcode == OP_DRAW || opcode == OP_VIDEO)) ops[4] = PolicyX(ops[4]); // dst_x, x
  std::vector<uint8_t> head;
  PutIns(head, opcode, ops.data(), k, 0);
  if (!HasTarget(opcode)) {
    code.insert(code.end(), head.begin(), head.end());
    return;
  }
  size_t fixed = 1 + size_t(k + 1) / 2;
  code.insert(code.end(), head.begin(), head.begin() + long(fixed));
  Target(target);
  code.insert(code.end(), head.begin() + long(fixed) + 3, head.end());
}

void Program::Write(Address address, std::span<const uint8_t> bytes) {
  if (bytes.size() > 255) return Fail("WRITE carries at most 255 bytes");
  PutWrite(code, address, bytes.data(), bytes.size());
}

void Program::WriteRLE(Address address, uint16_t length, std::span<const uint8_t> packbits) {
  if (packbits.size() > 255) return Fail("WRITE_RLE carries at most 255 bytes");
  PutWriteRle(code, address, length, packbits.data(), packbits.size());
}

void Program::Copy(ArenaWord dst, ArenaWord src, ArenaWord n) { Op(OP_COPY, {dst, src, n}); }
void Program::Unpack(ArenaWord dst, ArenaWord src, ArenaWord n, ArenaWord length) {
  Op(OP_UNPACK, {dst, src, n, length});
}
void Program::Mov(ArenaWord d, ArenaWord a) { Op(OP_MOV, {d, a}); }
void Program::Add(ArenaWord d, ArenaWord a, ArenaWord b) { Op(OP_ADD, {d, a, b}); }
void Program::Sub(ArenaWord d, ArenaWord a, ArenaWord b) { Op(OP_SUB, {d, a, b}); }
void Program::Mul(ArenaWord d, ArenaWord a, ArenaWord b) { Op(OP_MUL, {d, a, b}); }
void Program::DivU(ArenaWord d, ArenaWord a, ArenaWord b) { Op(OP_DIVU, {d, a, b}); }
void Program::ModU(ArenaWord d, ArenaWord a, ArenaWord b) { Op(OP_MODU, {d, a, b}); }
void Program::DivS(ArenaWord d, ArenaWord a, ArenaWord b) { Op(OP_DIVS, {d, a, b}); }
void Program::ModS(ArenaWord d, ArenaWord a, ArenaWord b) { Op(OP_MODS, {d, a, b}); }
void Program::And(ArenaWord d, ArenaWord a, ArenaWord b) { Op(OP_AND, {d, a, b}); }
void Program::Or(ArenaWord d, ArenaWord a, ArenaWord b) { Op(OP_OR, {d, a, b}); }
void Program::Xor(ArenaWord d, ArenaWord a, ArenaWord b) { Op(OP_XOR, {d, a, b}); }
void Program::Shl(ArenaWord d, ArenaWord a, ArenaWord b) { Op(OP_SHL, {d, a, b}); }
void Program::Shr(ArenaWord d, ArenaWord a, ArenaWord b) { Op(OP_SHR, {d, a, b}); }
void Program::Sar(ArenaWord d, ArenaWord a, ArenaWord b) { Op(OP_SAR, {d, a, b}); }

static int WidthIndex(int bits) {
  switch (bits) {
  case 32: return 0;
  case 16: return 1;
  case 8: return 2;
  case 4: return 3;
  default: return -1;
  }
}

void Program::Load(int bits, ArenaWord d, ArenaWord base, ArenaWord i) {
  int w = WidthIndex(bits);
  if (w < 0) return Fail("Load: width must be 32, 16, 8 or 4");
  Op(uint8_t(OP_LD32 + w), {d, base, i});
}

void Program::Store(int bits, ArenaWord base, ArenaWord i, ArenaWord a) {
  int w = WidthIndex(bits);
  if (w < 0) return Fail("Store: width must be 32, 16, 8 or 4");
  Op(uint8_t(OP_ST32 + w), {base, i, a});
}

void Program::Jmp(Label l) { Op(OP_JMP, {}, l); }
void Program::Call(Label l) { Op(OP_CALL, {}, l); }
void Program::Ret() { Op(OP_RET, {}); }
void Program::End() { Op(OP_END, {}); }
void Program::Halt() { Op(OP_HALT, {}); }
void Program::Beq(ArenaWord a, ArenaWord b, Label l) { Op(OP_BEQ, {a, b}, l); }
void Program::Bne(ArenaWord a, ArenaWord b, Label l) { Op(OP_BNE, {a, b}, l); }
void Program::BltU(ArenaWord a, ArenaWord b, Label l) { Op(OP_BLTU, {a, b}, l); }
void Program::BgeU(ArenaWord a, ArenaWord b, Label l) { Op(OP_BGEU, {a, b}, l); }
void Program::BltS(ArenaWord a, ArenaWord b, Label l) { Op(OP_BLTS, {a, b}, l); }
void Program::BgeS(ArenaWord a, ArenaWord b, Label l) { Op(OP_BGES, {a, b}, l); }
void Program::SleepUntil(ArenaWord t) { Op(OP_SLEEP_UNTIL, {t}); }
void Program::WaitEvent(ArenaWord d, ArenaWord mask) { Op(OP_WAIT_EVENT, {d, mask}); }
void Program::Raise(ArenaWord trigger) { Op(OP_RAISE, {trigger}); }
void Program::Notify(ArenaWord a, ArenaWord b) { Op(OP_NOTIFY, {a, b}); }
void Program::SysRead(ArenaWord d, ArenaWord property) { Op(OP_SYS_READ, {d, property}); }
void Program::SysWrite(ArenaWord property, ArenaWord a) { Op(OP_SYS_WRITE, {property, a}); }

void Program::Fill(ArenaWord dst, ArenaWord stride, ArenaWord x, ArenaWord y, ArenaWord w,
                   ArenaWord h, ArenaWord level) {
  Op(OP_FILL, {dst, stride, x, y, w, h, level});
}

void Program::Blit(ArenaWord src, ArenaWord src_stride, ArenaWord src_x, ArenaWord src_y,
                   ArenaWord dst, ArenaWord dst_stride, ArenaWord dst_x, ArenaWord dst_y,
                   ArenaWord w, ArenaWord h, ArenaWord mode, ArenaWord level) {
  Op(OP_BLIT, {src, src_stride, src_x, src_y, dst, dst_stride, dst_x, dst_y, w, h, mode, level});
}

void Program::Glyphs(ArenaWord font, ArenaWord dst, ArenaWord stride, ArenaWord x, ArenaWord y,
                     ArenaWord mode, ArenaWord level, ArenaWord text, ArenaWord n) {
  Op(OP_GLYPHS, {font, dst, stride, x, y, mode, level, text, n});
}

void Program::Draw(ArenaWord src, ArenaWord src_stride, ArenaWord src_x, ArenaWord src_y,
                   ArenaWord dst_x, ArenaWord dst_y, ArenaWord w, ArenaWord h) {
  Op(OP_DRAW, {src, src_stride, src_x, src_y, dst_x, dst_y, w, h});
}

void Program::Line(ArenaWord dst, ArenaWord stride, ArenaWord x0, ArenaWord y0, ArenaWord x1,
                   ArenaWord y1, ArenaWord level) {
  Op(OP_LINE, {dst, stride, x0, y0, x1, y1, level});
}

void Program::Video(ArenaWord bitmap, ArenaWord stride, ArenaWord w, ArenaWord h, ArenaWord x, ArenaWord y,
                    ArenaWord level, ArenaWord src, ArenaWord n, ArenaWord state) {
  Op(OP_VIDEO, {bitmap, stride, w, h, x, y, level, src, n, state});
}

void Program::AssetLoad(ArenaWord id, ArenaWord dst, ArenaWord off, ArenaWord n) {
  Op(OP_ASSET_LOAD, {id, dst, off, n});
}

// ---------------------------------------------------------------- VIDEO records (VM.md §3.7)

namespace {

constexpr int kVMax = 5;
constexpr uint32_t kVSame = 0, kVV0 = 3, kVIsv = kVV0 + 7 * 7 * 6, kVSign = kVIsv + 7 * 2 * 6,
                   kVMag = kVSign + 7 * 6, kVPass = kVMag + (kVMax - 1) * 7 * 6, kVRunLen = kVPass + 7,
                   kVRunBit = kVRunLen + 2 * 20, kVProbs = kVRunBit + 2 * 20 * 4;
static_assert(2 * kVProbs == kVideoState);

// The range coder of VM.md §3.7 (LZMA's, without its leading zero byte).
struct RangeEncoder {
  uint16_t *p;
  std::vector<uint8_t> &out;
  uint64_t low = 0;
  uint32_t range = 0xFFFFFFFF;
  uint8_t cache = 0;
  uint64_t pending = 0;
  bool first = true;

  void Put(uint8_t b) {
    if (first) first = false;
    else out.push_back(b);
  }
  void ShiftLow() {
    if (uint32_t(low) < 0xFF000000u || low >> 32) {
      uint8_t carry = uint8_t(low >> 32);
      Put(uint8_t(cache + carry));
      for (; pending; pending--) Put(uint8_t(0xFF + carry));
      cache = uint8_t(low >> 24);
    } else {
      pending++;
    }
    low = (low & 0x00FFFFFF) << 8;
  }
  void Bit(uint32_t i, bool b) {
    uint32_t q = p[i], bound = (range >> 12) * q;
    if (!b) {
      range = bound;
      p[i] = uint16_t(q + ((4096 - q) >> 5));
    } else {
      low += bound;
      range -= bound;
      p[i] = uint16_t(q - (q >> 5));
    }
    while (range < 1u << 24) {
      range <<= 8;
      ShiftLow();
    }
  }
  void Run(uint32_t colour, uint32_t v) {
    uint32_t x = v + 1, nb = 0;
    while (x >> (nb + 1)) nb++;
    for (uint32_t k = 0; k <= nb && k < 20; k++) Bit(kVRunLen + 20 * colour + std::min(k, 19u), k < nb);
    uint32_t c = kVRunBit + 80 * colour + 4 * std::min(nb, 19u);
    for (uint32_t i = 0; i < nb; i++) Bit(c + std::min(i, 3u), (x >> (nb - 1 - i)) & 1);
  }
  // The shortest code in [low, low + range) once the decoder reads zeros past the end.
  void Finish() {
    for (int k = 1; k <= 4; k++) {
      uint64_t m = (uint64_t(1) << (32 - 8 * k)) - 1, v = (low + m) & ~m;
      if (v < low + range) {
        low = v;
        break;
      }
    }
    for (int i = 0; i < 5; i++) ShiftLow();
    while (out.size() > 1 && out.back() == 0) out.pop_back();
  }
};

uint32_t AboveClass(uint32_t b1, uint32_t w, uint32_t code) {
  if (b1 >= w) return 6;
  if (code == 0) return 0;
  return code == 8 ? 1 : code == 9 ? 2 : code == 7 ? 3 : code > 8 ? 4 : 5;
}

// The previous frame's row (after 4 zero bytes) around b1: 1..5 for the leftmost transition of b1's kind at
// b1 - 2..b1 + 2, 0 for none.
uint32_t FrameClass(const uint8_t *old, uint32_t b1, uint32_t colour) {
  uint32_t q = b1 + 29, u = uint32_t(old[q >> 3]) << 8 | old[(q >> 3) + 1];
  uint32_t px = u >> (10 - (q & 7)) & 63, cand = ((px >> 1) ^ px) & (colour ? ~px : px) & 31;
  return cand ? uint32_t(std::countl_zero(cand)) - 26 : 0;
}

uint32_t Transitions(const uint8_t *row, uint32_t w, uint16_t *t) {
  uint32_t n = 0, c = 0;
  for (uint32_t x = 0; x < w; x++) {
    uint32_t p = (row[x >> 3] >> (7 - (x & 7))) & 1;
    if (p != c) {
      t[n++] = uint16_t(x);
      c = p;
    }
  }
  return n;
}

} // namespace

VideoEncoder::VideoEncoder(int width, int height)
    : w_(width), h_(height), nb_(width > 0 ? (width + 7) / 8 : 0), p_(kVProbs, 2048) {
  if (w_ >= 1 && w_ <= kPanelW && h_ >= 1 && h_ <= kPanelH) prev_.assign(size_t(nb_) * size_t(h_), 0);
}

std::vector<uint8_t> VideoEncoder::Encode(std::span<const uint8_t> pixels, bool redraw, size_t max_bytes) {
  if (prev_.empty()) return {};
  const uint32_t w = uint32_t(w_), nb = uint32_t(nb_);
  std::vector<uint8_t> cur(prev_.size(), 0), out = {uint8_t((started_ ? 0 : 1) | (redraw ? 2 : 0))};
  for (int y = 0; y < h_; y++)
    for (int x = 0; x < w_; x++)
      if (size_t(y) * size_t(w_) + size_t(x) < pixels.size() && pixels[size_t(y) * size_t(w_) + size_t(x)])
        cur[size_t(y) * nb + size_t(x) / 8] |= uint8_t(0x80 >> (x % 8));
  std::vector<uint16_t> p = started_ ? p_ : std::vector<uint16_t>(kVProbs, 2048);
  RangeEncoder e{p.data(), out};
  uint16_t ref[kPanelW + 1], ct[kPanelW + 1];
  uint8_t rcode[kPanelW + 1], ccode[kPanelW + 1], old[kPanelW / 8 + 8] = {};
  uint32_t nref = 0, prev_same = 2;
  for (uint32_t y = 0; y < uint32_t(h_); y++) {
    const uint8_t *pr = prev_.data() + y * nb, *cr = cur.data() + y * nb;
    bool same = std::memcmp(pr, cr, nb) == 0;
    uint32_t nc = Transitions(cr, w, ct);
    e.Bit(kVSame + prev_same, same);
    prev_same = same;
    if (same) {
      std::copy(ct, ct + nc, ref);
      std::fill(rcode, rcode + nc, 0);
      nref = nc;
      continue;
    }
    std::memcpy(old + 4, pr, nb);
    int32_t a0 = -1;
    uint32_t colour = 0, pc = 0, j = 0, i = 0;
    for (;;) {
      while (j < nref && int32_t(ref[j]) <= a0) j++;
      uint32_t jb = j + ((j & 1) != colour);
      uint32_t b1 = jb < nref ? ref[jb] : w, b2 = jb + 1 < nref ? ref[jb + 1] : w;
      uint32_t ac = AboveClass(b1, w, jb < nref ? rcode[jb] : 0), tc = b1 < w ? FrameClass(old, b1, colour) : 0;
      int32_t a1 = i < nc ? ct[i] : int32_t(w), a2 = i + 1 < nc ? ct[i + 1] : int32_t(w), d = a1 - int32_t(b1);
      uint32_t v0 = kVV0 + (pc * 7 + ac) * 6 + tc, isv = kVIsv + (pc * 2 + (ac != 0 && ac != 6)) * 6 + tc;
      if (int32_t(b2) < a1) { // pass
        e.Bit(v0, 1);
        e.Bit(isv, 0);
        e.Bit(kVPass + pc, 1);
        a0 = int32_t(b2);
        pc = 5;
        continue;
      }
      if (d >= -kVMax && d <= kVMax) { // vertical
        e.Bit(v0, d != 0);
        if (d) {
          uint32_t m = uint32_t(d < 0 ? -d : d);
          e.Bit(isv, 1);
          e.Bit(kVSign + ac * 6 + tc, d < 0);
          for (uint32_t k = 1; k < kVMax; k++) {
            e.Bit(kVMag + ((k - 1) * 7 + ac) * 6 + tc, m > k);
            if (m <= k) break;
          }
        }
        if (a1 >= int32_t(w)) break;
        ccode[i++] = uint8_t(d + 8);
        a0 = a1;
        colour ^= 1;
        pc = d == 0 ? 1 : d == 1 ? 2 : d == -1 ? 3 : 4;
        continue;
      }
      e.Bit(v0, 1); // horizontal
      e.Bit(isv, 0);
      e.Bit(kVPass + pc, 0);
      e.Run(colour, uint32_t(a1 - std::max(a0, 0)));
      e.Run(colour ^ 1, uint32_t(a2 - a1));
      if (a1 >= int32_t(w)) break;
      ccode[i++] = 0;
      if (a2 >= int32_t(w)) break;
      ccode[i++] = 0;
      a0 = a2;
      pc = 6;
    }
    std::copy(ct, ct + nc, ref);
    std::copy(ccode, ccode + nc, rcode);
    nref = nc;
  }
  e.Finish();
  if (out.size() > max_bytes) return {};
  started_ = true;
  p_ = std::move(p);
  prev_ = std::move(cur);
  return out;
}

bool Program::Valid(std::string *error) const {
  std::string e = error_;
  for (const Fixup &f : fixups_)
    if (e.empty() && labels_[size_t(f.label)] < 0) e = "a label is used but not bound";
  for (int t = 0; t < 16; t++)
    if (e.empty() && handlers_[size_t(t)] >= 0 && labels_[size_t(handlers_[size_t(t)])] < 0)
      e = "a handler's label is not bound";
  if (e.empty() && uint64_t(code_address) + code.size() > kArenaBytes) e = "the code ends past the arena";
  if (error) *error = e;
  return e.empty();
}

namespace {

// ---------------------------------------------------------------- the mirror

// ASSET_LOAD on a mirror: the bytes of a kept font or bitmap (the library
// holds every blob it loads that way).
int KeptRead(const uint8_t *id, uint32_t off, uint8_t *dst, uint32_t n);

// The arm's interpreter (src/vm) running only host code on Arm::arena, with
// a virtual clock: the arena and the panel as the arm will hold them once it
// has run everything queued so far. Programs are not mirrored (VM.md §5.6).
struct Mirror {
  struct vm vm {};
  vm_host host{};
  uint8_t *arena = nullptr;
  std::vector<uint8_t> panel = std::vector<uint8_t>(size_t(kPanelW) * kPanelH, 0);
  bool panel_on = false, standby = false;
  uint32_t brightness = 21, brightness_max = 42, gyro_rate = 0, auto_brightness = 0, peer_link = 0;
  uint16_t triggers = 0xFFFF; // the arm's HELLO.triggers: no IMU or light sensor refuses their settings
  uint32_t role = 0;
  uint32_t t = 0;
  std::vector<uint8_t> packet;
  bool has_packet = false;
  int error = 0;
  uint8_t error_op = 0;
  uint32_t error_detail = 0;
  std::vector<std::pair<Address, uint32_t>> changes; // what programs change

  static Mirror *M(void *c) { return static_cast<Mirror *>(c); }
  // AUTO_BRIGHTNESS exists with the light sensor, and on a left arm with the arm-to-arm link (it follows the
  // right arm while PEER_LINK is 1)
  bool HasAuto() const {
    return triggers & (1u << VM_T_LIGHT) || (triggers & (1u << VM_T_PEER) && role == 1);
  }

  void Init(uint8_t *a, int arm) {
    arena = a;
    role = arm == 0 ? 1 : 0;
    host = {this,
            [](void *c) { return M(c)->t; },
            [](void *c, uint32_t *len) -> const uint8_t * {
              Mirror *m = M(c);
              *len = uint32_t(m->packet.size());
              return m->has_packet ? m->packet.data() : nullptr;
            },
            [](void *c) { M(c)->has_packet = false; },
            [](void *) { return uint32_t(0); },
            [](void *c, uint32_t id) -> uint32_t {
              Mirror *m = M(c);
              if (id == VM_P_ARM) return m->role;
              if (id == VM_P_LINK) return 1;
              if (id == VM_P_PANEL) return m->panel_on ? 1 : m->standby ? 2 : 0;
              if (id == VM_P_BRIGHTNESS) return m->brightness;
              if (id == VM_P_BRIGHTNESS_MAX) return m->brightness_max;
              if (id == VM_P_GYRO_RATE) return m->triggers & (1u << VM_T_MOTION) ? m->gyro_rate : VM_UNKNOWN;
              if (id == VM_P_AUTO_BRIGHTNESS) return m->HasAuto() ? m->auto_brightness : VM_UNKNOWN;
              return VM_UNKNOWN;
            },
            [](void *c, uint8_t type, const uint8_t *body, uint32_t len) {
              Mirror *m = M(c);
              if (type == VM_EV_ERROR && len >= 9 && !m->error) {
                m->error = body[4];
                m->error_op = body[3];
                m->error_detail = uint32_t(body[5] | body[6] << 8 | body[7] << 16) | uint32_t(body[8]) << 24;
              }
              return 0;
            },
            [](void *c, int32_t, int32_t, int32_t, int32_t) {
              Mirror *m = M(c);
              if (!m->panel_on) std::fill(m->panel.begin(), m->panel.end(), 0);
              m->panel_on = true;
              m->standby = false;
              return 0;
            },
            [](void *c, int32_t x, int32_t y, int32_t w, const uint8_t *px) {
              uint8_t *p = M(c)->panel.data() + size_t(y) * kPanelW + size_t(x);
              for (int32_t i = 0; i < w; i++) p[i] = (i & 1) ? (px[i / 2] & 15) : (px[i / 2] >> 4);
            },
            [](void *) {},
            [](void *c, uint32_t on) {
              Mirror *m = M(c);
              if (on != 1 || !m->panel_on) std::fill(m->panel.begin(), m->panel.end(), 0);
              m->panel_on = on == 1;
              m->standby = on == 2;
            },
            [](void *c, uint32_t level) { M(c)->brightness = level; },
            [](void *, uint32_t) {},
            [](void *c, uint32_t id, uint32_t value) {
              Mirror *m = M(c);
              bool light = m->triggers & (1u << VM_T_LIGHT), peer = m->triggers & (1u << VM_T_PEER);
              if (id == VM_P_GYRO_RATE && m->triggers & (1u << VM_T_MOTION)) m->gyro_rate = value;
              else if (id == VM_P_BRIGHTNESS_MAX) m->brightness_max = value;
              else if (id == VM_P_AUTO_BRIGHTNESS && (light || (m->HasAuto() && (m->peer_link || !value))))
                m->auto_brightness = value;
              else if (id == VM_P_PEER_LINK && peer && value <= 1) {
                m->peer_link = value;
                if (!value && !light) m->auto_brightness = 0; // the left arm stops following the right one
              } else if (!(id == VM_P_PEER_SEND && peer && m->peer_link))
                return int(VM_E_ARG);
              return 0;
            }};
    host.asset_read = [](void *, const uint8_t *id, uint32_t off, uint8_t *dst, uint32_t n) {
      return KeptRead(id, off, dst, n);
    };
    vm_init(&vm, arena, &host, 0);
    panel_on = standby = false;
    brightness_max = 42;
    gyro_rate = auto_brightness = peer_link = 0;
    std::fill(panel.begin(), panel.end(), 0);
    changes.clear();
  }

  // Runs host code as one packet; 0 or the error code of the instruction
  // that failed (the ones before it stay done, as on the arm).
  int Run(const uint8_t *code, size_t n) {
    packet.assign({0, 0});
    packet.insert(packet.end(), code, code + n);
    has_packet = true;
    error = 0;
    for (int guard = 0; has_packet && guard < 100000; guard++) {
      uint32_t w = vm_run(&vm);
      if (!has_packet || w == VM_FOREVER) break;
      t += std::max<uint32_t>(w, 1);
    }
    has_packet = false;
    return error;
  }

  bool Changed(uint64_t a, uint64_t n) const {
    for (auto &c : changes)
      if (a < uint64_t(c.first) + c.second && uint64_t(c.first) < a + n) return true;
    return false;
  }
};

// Constants of the library's host code, in words at kPool (VM.md §2.1: no
// immediate values). A value keeps its word until the ring of words comes
// round to it again; packets run in order, so earlier code still reads the
// old value. A present's DRAWs are sent only when it fires, after the
// constants written meanwhile for it: its words stay pinned while it is built.
struct Pool {
  std::unordered_map<uint32_t, int> slot_of;
  std::vector<int64_t> value = std::vector<int64_t>(size_t(kPoolWords), -1);
  std::vector<bool> pinned = std::vector<bool>(size_t(kPoolWords), false);
  bool pinning = false;
  int next = 0;
  void Clear() {
    slot_of.clear();
    std::fill(value.begin(), value.end(), -1);
    Unpin();
    next = 0;
  }
  void Unpin() {
    std::fill(pinned.begin(), pinned.end(), false);
    pinning = false;
  }
};

// ---------------------------------------------------------------- arena allocator

class Allocator {
public:
  void Reset(uint32_t first_word, uint32_t end_word) {
    free_.clear();
    used_.clear();
    free_[first_word] = end_word - first_word;
    total_ = end_word - first_word;
  }

  int Alloc(uint32_t words) {
    if (words == 0) words = 1;
    for (auto it = free_.begin(); it != free_.end(); ++it) {
      if (it->second < words) continue;
      uint32_t start = it->first, rest = it->second - words;
      free_.erase(it);
      if (rest) free_[start + words] = rest;
      used_[start] = words;
      return int(start);
    }
    return -1;
  }

  bool Free(uint32_t start) {
    auto u = used_.find(start);
    if (u == used_.end()) return false;
    uint32_t len = u->second;
    used_.erase(u);
    auto next = free_.lower_bound(start);
    if (next != free_.end() && start + len == next->first) {
      len += next->second;
      next = free_.erase(next);
    }
    if (next != free_.begin()) {
      auto prev = std::prev(next);
      if (prev->first + prev->second == start) {
        prev->second += len;
        return true;
      }
    }
    free_[start] = len;
    return true;
  }

  uint32_t TotalWords() const { return total_; }
  uint32_t FreeWords() const {
    uint32_t n = 0;
    for (auto &f : free_) n += f.second;
    return n;
  }
  const std::map<uint32_t, uint32_t> &Used() const { return used_; }

private:
  std::map<uint32_t, uint32_t> free_, used_;
  uint32_t total_ = 0;
};

// ---------------------------------------------------------------- transport

// An arm the transport could connect: bonded, or seen advertising, with the
// pair record of its scan response if one was seen (VM.md §6.1).
struct Candidate {
  std::string path, address;
  bool paired = false, connected = false;
  int rssi = -999;
  int flags = -1; // pair record flags: bit 0 left, bit 1 no panel; -1 no record
  std::string pair_id;
};

// The pair record bytes after the company ID: format, flags, in format 2 the firmware's version (3 bytes), pair ID.
[[maybe_unused]] bool ParsePairRecord(const std::vector<uint8_t> &data, int *flags, std::string *pair_id) {
  if (data.size() < 3 || data[0] < 1 || data[0] > 2 || (data[0] == 2 && data.size() < 6)) return false;
  *flags = data[1];
  pair_id->assign(data.begin() + (data[0] == 2 ? 5 : 2), data.end());
  return true;
}

// Boards without a panel are left out, and so is an arm of other glasses
// once the partner's pair ID is known.
[[maybe_unused]] bool Eligible(const Candidate &c, const std::string &partner,
                               const std::set<std::string> &rejected) {
  if (rejected.count(c.path) || (c.flags >= 0 && (c.flags & 2))) return false;
  return partner.empty() || c.pair_id.empty() || c.pair_id == partner;
}

// Index of the best candidate in `mine`, -1 if none is eligible. `partner` is
// the other arm's pair ID when it is already chosen or given. Order: the pair
// ID matches the partner (without one: a candidate for the other arm has the
// same pair ID), connected, bonded, strongest signal.
[[maybe_unused]] int PickCandidate(const std::vector<Candidate> &mine,
                                    const std::vector<Candidate> &other, const std::string &partner,
                                    const std::set<std::string> &rejected) {
  auto counterpart = [&](const Candidate &c) {
    for (const Candidate &o : other)
      if (Eligible(o, "", rejected) && o.pair_id == c.pair_id) return true;
    return false;
  };
  auto score = [&](const Candidate &c) {
    bool match = !c.pair_id.empty() && (partner.empty() ? counterpart(c) : c.pair_id == partner);
    return std::make_tuple(match, c.connected, c.paired, c.rssi);
  };
  int best = -1;
  for (size_t i = 0; i < mine.size(); i++) {
    if (!Eligible(mine[i], partner, rejected)) continue;
    if (best < 0 || score(mine[size_t(best)]) < score(mine[i])) best = int(i);
  }
  return best;
}

// A BlueZ adapter object (org.bluez.Adapter1).
struct AdapterInfo {
  std::string path, address;
  bool powered = false;
};

// Index of the adapter to use, -1 (and `problem`) if there is none. `want`:
// a name ("hci1"), an address, or empty for the first powered adapter.
[[maybe_unused]] int PickAdapter(const std::vector<AdapterInfo> &adapters, const std::string &want,
                                 std::string *problem) {
  auto upper = [](std::string s) {
    for (char &c : s) c = char(std::toupper(uint8_t(c)));
    return s;
  };
  std::string off;
  for (size_t i = 0; i < adapters.size(); i++) {
    const AdapterInfo &a = adapters[i];
    if (!want.empty() && a.path != "/org/bluez/" + want && upper(a.address) != upper(want)) continue;
    if (a.powered) return int(i);
    if (off.empty()) off = a.path;
  }
  if (off.empty())
    *problem = want.empty() ? "no Bluetooth adapter" : "no Bluetooth adapter " + want;
  else
    *problem = "Bluetooth is off (" + off + ")";
  return -1;
}
// ---------------------------------------------------------------- queue items

struct PresentJob {
  uint64_t id = 0;
  std::vector<Cmd> draws[2];
  bool fired = false;
  bool dropped[2] = {false, false}; // left that arm's queue (its restore redraws)
};

struct Item {
  enum Kind : uint8_t { COMMAND, PRESENT } kind = COMMAND;
  bool last_draw = false;  // the end of a present: NOTIFY [T], [T]
  uint64_t present_id = 0; // set on the items of a fired present
  uint32_t at = 0;         // its time T
  Cmd cmd;
  std::shared_ptr<PresentJob> present;
  uint64_t tag = 0; // the asset job whose staging this command writes
};

struct ControlJob {
  int arm = 0;
  uint8_t op = 0;
  std::vector<uint8_t> body;
  std::promise<std::pair<int, std::vector<uint8_t>>> reply;
};

struct Reply {
  int status = -1; // -1: no reply
  const uint8_t *body = nullptr;
  size_t n = 0;
  uint32_t epoch = 0;
  int64_t at = 0;   // arrival (host µs)
  int64_t sent = 0; // when the request was sent
};

// ---------------------------------------------------------------- per-arm protocol state

struct Link {
  bool present = false;
  bool up = false;
  int mtu = 23;
  enum Phase { DOWN, HELLO, RESET, SETUP, READY } phase = DOWN;
  bool known = false; // the arm has a session of ours
  uint32_t epoch = 0;
  uint32_t unknown_epoch = 0; // the last other epoch seen
  int max_packet = 20;
  int64_t hello_next = 0;
  int hello_tries = 0;
  // window
  uint16_t next_seq = 0, rx_next = 0, exec_next = 0;
  int credits = 0;
  uint32_t sack = 0;
  struct Sent {
    uint16_t seq;
    std::vector<uint8_t> data;
    int64_t first;
    int64_t last;
    bool resent;
  };
  std::deque<Sent> unacked;
  uint64_t sent_bytes = 0; // new packets, for sharing the controller fairly
  int64_t progress = 0;
  int64_t srtt = 0, rttvar = 0; // from packets acked without a resend
  int backoff = 0;
  int64_t heard = 0; // last ACK or event from the arm
  bool probing = false;
  std::deque<Item> pending;
  struct DrawWait {
    uint16_t seq; // the packet with the present's NOTIFY
    uint64_t present;
    uint32_t at;
  };
  std::deque<DrawWait> draws;
  // control requests
  uint8_t tag = 0;
  struct Request {
    uint8_t op;
    std::vector<uint8_t> body;
    int tries;
    int timeout_us;
    int64_t sent;
    int64_t deadline;
    std::function<void(const Reply &)> done;
  };
  std::map<uint8_t, Request> waiting;
  // clock
  struct Sample {
    int64_t at;
    uint32_t offset;
    int64_t delay;
  };
  std::deque<Sample> samples;
  int setup_pings = 0;
  bool ping_busy = false;
  int64_t next_ping = 0, last_ping = 0;
  int interval_us = 15000;
  int phy = 0; // 1 = 1M, 2 = 2M, 0 unknown
  bool guard = false;     // the arm has the panel guard (it answered PANEL_GUARD)
  uint8_t cap = 42;       // its BRIGHTNESS_CAP as last reported
  uint8_t max_sent = 42;  // the BRIGHTNESS_MAX this session gave it
  int64_t link_asked = 0;
  bool slow = false;      // the idle profile is asked for
  int64_t last_busy = 0;  // traffic, for the link profile
  // read by the application thread
  std::atomic<bool> ready{false};
  std::atomic<bool> idle{true};
  std::atomic<uint64_t> emitted{0}; // items the application queued
  uint64_t taken = 0;               // of those, items the thread took
  uint64_t idle_taken = 0;          // `taken` when `idle` was last computed (Core::mutex)
  std::atomic<bool> idle_profile{false};
  std::atomic<bool> safe{false}, encrypted{false}, in_case{false}, microphone{false};
  std::atomic<uint16_t> triggers{0}, inputs{0};
  std::atomic<uint32_t> moved{0}; // counts ACKs that moved the window
  std::atomic<uint32_t> offset{0};
  std::atomic<int> clock_error_us{-1};
  std::atomic<uint64_t> done_present{0};
  std::atomic<uint32_t> packets{0}, resent{0}, reconnects{0}, epoch_changes{0}, errors{0},
      epoch_seen{0}, events_lost{0};
  std::atomic<int> queued{0}, firmware{0}, max_packet_seen{0}, interval_seen{0};
  std::atomic<uint64_t> shown_update{0};
  std::atomic<int64_t> shown_us{0};
  std::string last_error;   // guarded by Core::mutex
  std::string address;      // guarded by Core::mutex
  std::deque<g1::Event> events; // guarded by Core::mutex
  struct Mic {                  // guarded by Core::mutex
    std::atomic<bool> on{false};
    bool started = false;       // since MicStart, until MicStop
    uint32_t panel_before = 1; // PANEL before MicStart
    int bytes = 40;             // per frame, for the rate asked for
    MicStopReason stopped = MIC_STOP_HOST;
    int64_t renewed = 0; // the last MIC 1 (the arm's lease, VM.md §6.5)
    uint32_t next = 0; // the counter expected next
    MicState state{};
    std::deque<MicFrame> frames;
    std::vector<uint64_t> memory; // the decoder
    void *decoder = nullptr;
  } mic;
  // The asset store (VM.md §6.10), guarded by Core::mutex: what the arm holds,
  // listed at each session's setup and kept current from the library's jobs,
  // and those jobs, run one at a time in order.
  struct Assets {
    bool store = false; // the arm has one
    std::map<AssetId, AssetEntry> entries;
  } assets;
  struct AssetJob {
    uint64_t tag = 0; // its staging commands carry it
    uint8_t op = 0;   // CTRL_ASSET_PUT or CTRL_ASSET_DELETE
    AssetId id{};
    Address address = 0;
    uint32_t size = 0;
    uint8_t type = 0;
    int items = 0, packed = 0; // staging commands, and those sent in packets
    uint16_t seq = 0;          // the packet of the last one
    enum State { WAITING, SENT, RUNNING } state = WAITING;
    int64_t at = 0; // WAITING: not before; RUNNING: the next ASSET_INFO
    std::shared_ptr<std::promise<int>> done; // Store, Forget: 0 or the error code
  };
  std::deque<AssetJob> asset_jobs;
};

class Core;

// ---------------------------------------------------------------- application-side state

struct Geometry {
  int width = 740, height = 471;
  int ox[2] = {0, 100};
  int oy[2] = {0, 9};
};

struct ArmImpl {
  bool present = false;
  Mirror mirror;
  Pool pool;
  std::atomic<int64_t> black_off{0}; // NowUs when the all-black eye in standby goes off (0: none)
};

// A font or bitmap of Keep::kStored: the blob the arms store, at `address`
// in the arena (a bitmap's from its first Sync, empty before).
struct Kept {
  Address address = 0;
  std::vector<uint8_t> blob;
  AssetId id{};
  uint8_t type = 0;
};

struct G1Bitmap {
  Address address = 0;
  uint32_t stride = 0;
  uint32_t header = 0; // reserved before the rows: a kept bitmap's SKB1 header
  std::unique_ptr<Kept> kept;
};

struct G1Font {
  Address address = 0;
  std::unique_ptr<Kept> kept;
};

class G1Device;

struct Global {
  std::unique_ptr<Core> core;
  std::unique_ptr<G1Device> device;
  ArmImpl arms[2];
  Allocator alloc; // words, the same on both arms
  Geometry geo;
  int cmd_limit = SCOUTER_G1_MAX_PACKET - 2;
  std::atomic<bool> keep{false}; // Connection::keep_sessions
  std::vector<Kept *> kept;      // the fonts and bitmaps of Keep::kStored
  uint64_t emit_tag = 0;         // Emit tags its items with it (an asset job's staging)
  int emit_count = 0;            // and counts them
  uint64_t tags = 0;
};

Global *g = nullptr;
Arm *const kArms[2] = {&left, &right};

uint8_t *ArmBytes(int arm) { return reinterpret_cast<uint8_t *>(kArms[arm]->arena.data()); }

int KeptRead(const uint8_t *id, uint32_t off, uint8_t *dst, uint32_t n) {
  for (const Kept *k : g->kept) {
    if (k->blob.empty() || std::memcmp(k->id.b, id, 16) != 0) continue;
    if (uint64_t(off) + n > k->blob.size()) return VM_E_ARENA;
    std::memcpy(dst, k->blob.data() + off, n);
    return 0;
  }
  return VM_E_ASSET;
}

int LevelOf(uint8_t gray) { return (gray * 15 + 127) / 255; }

void Emit(int arm, const Cmd &cmd);
void RecordError(int arm, const std::string &text);
bool SafeMode(int arm);

// Runs the code on the arm's mirror and queues it if the arm would accept
// it, else records why not. The code is one instruction (or several that the
// caller checked), so a refused one changes nothing.
bool Submit(int arm, const Cmd &cmd, const char *what) {
  if (cmd.size == 0) {
    RecordError(arm, std::string(what) + ": does not fit a packet");
    return false;
  }
  Mirror &m = g->arms[arm].mirror;
  m.vm.safe = SafeMode(arm) ? 1 : 0;
  int code = m.Run(cmd.b.data(), cmd.size);
  if (code) {
    char text[200];
    std::snprintf(text, sizeof(text), "%s refused: %s (error %d, detail 0x%x)", what, CodeName(code),
                  code, unsigned(m.error_detail));
    RecordError(arm, text);
    return false;
  }
  Emit(arm, cmd);
  return true;
}

// The pool words that hold the values, writing the missing ones first;
// false (nothing written) when pinned words fill the pool.
bool Words(int arm, const uint32_t *values, int n, ArenaWord *out) {
  Pool &p = g->arms[arm].pool;
  const uint8_t *mem = ArmBytes(arm);
  bool need[16] = {};
  for (int i = 0; i < n; i++) {
    auto it = p.slot_of.find(values[i]);
    need[i] = it == p.slot_of.end() || Get32(mem + kPool + 4 * uint32_t(it->second)) != values[i];
    if (!need[i]) out[i] = ArenaWord(kPool / 4 + uint32_t(it->second));
  }
  std::vector<std::pair<int, uint32_t>> fresh;
  for (int i = 0; i < n; i++) {
    if (!need[i]) continue;
    bool shared = false;
    for (int j = 0; j < i && !shared; j++)
      if (need[j] && values[j] == values[i]) {
        out[i] = out[j];
        shared = true;
      }
    if (shared) continue;
    int s = 0;
    for (int tries = 0;; tries++) {
      if (tries == kPoolWords) {
        for (auto &f : fresh) {
          p.slot_of.erase(f.second);
          p.value[size_t(f.first)] = -1;
        }
        return false;
      }
      s = p.next;
      p.next = (p.next + 1) % kPoolWords;
      bool in_use = p.pinned[size_t(s)];
      for (int j = 0; j < n; j++)
        in_use |= (!need[j] || j < i) && out[j] == ArenaWord(kPool / 4 + uint32_t(s));
      if (!in_use) break;
    }
    if (p.value[size_t(s)] >= 0) {
      auto old = p.slot_of.find(uint32_t(p.value[size_t(s)]));
      if (old != p.slot_of.end() && old->second == s) p.slot_of.erase(old);
    }
    p.value[size_t(s)] = values[i];
    p.slot_of[values[i]] = s;
    out[i] = ArenaWord(kPool / 4 + uint32_t(s));
    fresh.push_back({s, values[i]});
  }
  std::sort(fresh.begin(), fresh.end());
  for (size_t k = 0; k < fresh.size();) {
    size_t e = k + 1;
    while (e < fresh.size() && fresh[e].first == fresh[e - 1].first + 1) e++;
    std::vector<uint8_t> bytes, cmd;
    for (size_t j = k; j < e; j++) Put32(bytes, fresh[j].second);
    PutWrite(cmd, kPool + 4 * uint32_t(fresh[k].first), bytes.data(), bytes.size());
    Submit(arm, ToCmd(cmd), "constant");
    k = e;
  }
  if (p.pinning)
    for (int i = 0; i < n; i++) p.pinned[size_t(out[i] - kPool / 4)] = true;
  return true;
}

// One instruction whose operands are these values (constants from the pool).
Cmd Instruction(int arm, uint8_t op, std::initializer_list<uint32_t> values) {
  ArenaWord w[16];
  Words(arm, values.begin(), int(values.size()), w);
  std::vector<uint8_t> v;
  PutIns(v, op, w, int(values.size()), 0);
  return ToCmd(v);
}

bool HostOp(int arm, uint8_t op, std::initializer_list<uint32_t> values, const char *what) {
  return Submit(arm, Instruction(arm, op, values), what);
}

Cmd WriteCmd(Address addr, const uint8_t *data, size_t n) {
  std::vector<uint8_t> v;
  PutWrite(v, addr, data, n);
  return ToCmd(v);
}

// WRITE / WRITE_RLE commands, each fitting one packet, that put `n` bytes
// at arena address `addr`.
std::vector<Cmd> EncodeWrite(Address addr, const uint8_t *data, uint32_t n) {
  const int limit = g ? g->cmd_limit : SCOUTER_G1_MAX_PACKET - 2;
  const size_t rle_max = size_t(std::min(255, limit - 7));
  const uint32_t raw_max = uint32_t(std::min(255, limit - 5));
  std::vector<Cmd> out;
  std::vector<uint8_t> packed, unit;
  uint32_t pos = 0;
  while (pos < n) {
    packed.clear();
    uint32_t len = 0;
    int run_byte = -1, run_len = 0;
    while (pos + len < n && len + 32 <= 65535) {
      uint32_t size = std::min<uint32_t>(32, n - pos - len);
      const uint8_t *d = data + pos + len;
      bool uniform = std::all_of(d, d + size, [&](uint8_t b) { return b == d[0]; });
      if (uniform && run_byte == d[0] && run_len + int(size) <= 128) {
        run_len += int(size);
        packed[packed.size() - 2] = uint8_t(257 - run_len);
        len += size;
        continue;
      }
      unit.clear();
      PackBits(d, int(size), unit);
      if (packed.size() + unit.size() > rle_max) break;
      packed.insert(packed.end(), unit.begin(), unit.end());
      run_byte = uniform && size >= 2 ? d[0] : -1;
      run_len = int(size);
      len += size;
    }
    uint32_t raw = std::min(n - pos, raw_max);
    bool use_rle = len > 0 && uint64_t(packed.size() + 7) * raw < uint64_t(raw + 5) * len;
    std::vector<uint8_t> v;
    if (use_rle) {
      PutWriteRle(v, addr + pos, len, packed.data(), packed.size());
      pos += len;
    } else {
      PutWrite(v, addr + pos, data + pos, raw);
      pos += raw;
    }
    out.push_back(ToCmd(v));
  }
  return out;
}

void Upload(int arm, Address addr, const uint8_t *data, uint32_t n) {
  for (const Cmd &c : EncodeWrite(addr, data, n)) Submit(arm, c, "WRITE");
}

size_t EncodedSize(Address addr, const uint8_t *data, uint32_t n) {
  size_t total = 0;
  for (const Cmd &c : EncodeWrite(addr, data, n)) total += c.size;
  return total;
}

// A DRAW for a present: checked on the mirror (which shows it) now, sent to
// the arm when the present fires. With the pool full of the present's own
// words, the DRAW brings its operands along: WRITE [kDrawAddr], then DRAW.
void PresentDraw(int arm, std::vector<Cmd> &out, Address src, uint32_t stride, int sx, int sy, int x,
                 int y, int w, int h) {
  uint32_t v[8] = {src, stride, uint32_t(sx), uint32_t(sy), uint32_t(x), uint32_t(y), uint32_t(w), uint32_t(h)};
  ArenaWord words[8];
  std::vector<uint8_t> ins;
  if (!Words(arm, v, 8, words)) {
    std::vector<uint8_t> bytes;
    for (int i = 0; i < 8; i++) {
      Put32(bytes, v[i]);
      words[i] = ArenaWord(kDrawAddr / 4 + uint32_t(i));
    }
    PutWrite(ins, kDrawAddr, bytes.data(), bytes.size());
  }
  PutIns(ins, OP_DRAW, words, 8, 0);
  Cmd c = ToCmd(ins);
  Mirror &m = g->arms[arm].mirror;
  m.vm.safe = SafeMode(arm) ? 1 : 0;
  int code = m.Run(c.b.data(), c.size);
  if (code)
    RecordError(arm, std::string("DRAW refused: ") + CodeName(code) + " (error " + std::to_string(code) + ")");
  else
    out.push_back(c);
}

// Black w x h at panel (x, y), from the staging rows (zero again first if
// host code drew there).
void BlackDraws(int arm, std::vector<Cmd> &out, int x, int y, int w, int h) {
  const uint8_t *staging = ArmBytes(arm) + kStaging;
  if (h > 0 && !std::all_of(staging, staging + kStagingBytes, [](uint8_t b) { return b == 0; })) {
    std::vector<uint8_t> zero(kStagingBytes, 0);
    Upload(arm, kStaging, zero.data(), kStagingBytes);
  }
  for (int r = 0; r < h; r += kStagingRows)
    PresentDraw(arm, out, kStaging, kRowBytes, 0, 0, x, y + r, w, std::min(kStagingRows, h - r));
}

// The panel rows y0..y0+n-1 the library expects eye `arm` to show, packed.
std::vector<uint8_t> ExpectedPanelRows(int arm, int y0, int n) {
  std::vector<uint8_t> out(size_t(n) * kRowBytes, 0);
  const std::vector<uint8_t> &p = g->arms[arm].mirror.panel;
  for (int r = 0; r < n; r++)
    for (int x = 0; x < kPanelW; x++) {
      int level = p[size_t(y0 + r) * kPanelW + size_t(x)];
      out[size_t(r) * kRowBytes + size_t(x) / 2] |= uint8_t(x % 2 == 0 ? level << 4 : level);
    }
  return out;
}

// The head and tail of a fired present: WRITE [T] = at; SLEEP_UNTIL [T]; the
// DRAWs; NOTIFY [T], [T] (its event time is when they are shown); WRITE [T] = 0
// (the mirror never sees these, so the word goes back to its value there).
Cmd PresentHead(uint32_t at) {
  std::vector<uint8_t> v, t;
  Put32(t, at);
  PutWrite(v, kTimeAddr, t.data(), 4);
  ArenaWord w = kTimeWord;
  PutIns(v, OP_SLEEP_UNTIL, &w, 1, 0);
  return ToCmd(v);
}

Cmd PresentTail() {
  std::vector<uint8_t> v, zero(4, 0);
  ArenaWord w[2] = {kTimeWord, kTimeWord};
  PutIns(v, OP_NOTIFY, w, 2, 0);
  PutWrite(v, kTimeAddr, zero.data(), 4);
  return ToCmd(v);
}

g1::Event MakeEvent(EventType type, const uint8_t *d, int64_t now) {
  g1::Event e{};
  e.type = type;
  e.epoch = Get32(d + 1);
  e.device_us = Get32(d + 5);
  e.host_us = now;
  const uint8_t *b = d + 9;
  switch (type) {
  case EVENT_ERROR:
    e.count = Get16(b);
    e.offset = b[2];
    e.op = b[3];
    e.code = b[4];
    e.detail = Get32(b + 5);
    e.address = Get32(b + 9);
    e.trigger = 0xFF;
    break;
  case EVENT_HALT:
    e.trigger = b[0];
    e.address = Get32(b + 1);
    e.op = b[5];
    e.code = b[6];
    e.detail = Get32(b + 7);
    break;
  case EVENT_NOTIFY:
    e.count = Get16(b);
    e.trigger = b[2];
    e.a = Get32(b + 3);
    e.b = Get32(b + 7);
    break;
  case EVENT_INPUT:
    e.count = Get16(b);
    e.trigger = b[2];
    e.a = Get32(b + 3);
    break;
  default:
    break;
  }
  return e;
}

// ---------------------------------------------------------------- Bluetooth thread

class Core : public Transport::Events {
public:
  Link links[2];
  moodycamel::ConcurrentQueue<Item> queues[2];
  std::unique_ptr<Transport> owned; // BlueZ or the emulator; Connection::transport is not ours
  Transport *transport = nullptr;
  std::thread thread;
  std::atomic<bool> stop{false};
  std::atomic<bool> ending{false}; // Deinit has begun: no more eye matching
  std::atomic<uint64_t> present_counter{0};
  std::atomic<int> hold_ms[2] = {0, 0}; // Arm::Disconnect, for the transport
  std::atomic<bool> started{false};      // Init returned: link profiles may go idle
  std::atomic<bool> forget[2] = {false, false}; // emulator::NewHost: the next HELLO is a new host's

  std::mutex mutex; // guards the fields below, and Link::last_error/address/events
  std::condition_variable cv;
  InitError problem = ERROR_NONE;
  std::string problem_text;
  bool found[2] = {false, false};
  bool finished = false; // the thread ended
  std::vector<std::shared_ptr<ControlJob>> controls;

  std::deque<Item> *restore_sink[2] = {nullptr, nullptr};

  bool automatic[2] = {false, false};

  void Launch(Transport *t, std::unique_ptr<Transport> own, const bool present[2],
              const bool found_automatically[2]) {
    owned = std::move(own);
    transport = t;
    for (int i = 0; i < 2; i++) {
      links[i].present = present[i];
      automatic[i] = found_automatically[i];
    }
    bool p0 = present[0], p1 = present[1];
    thread = std::thread([this, p0, p1] {
      bool p[2] = {p0, p1};
      if (transport->Start(this, p)) {
        Run();
        transport->Stop();
      }
      {
        std::lock_guard<std::mutex> lock(mutex);
        finished = true;
        if (problem == ERROR_NONE && !stop) problem = ERROR_BLUETOOTH_ERROR;
      }
      cv.notify_all();
    });
  }

  void Shutdown() {
    stop = true;
    if (transport) transport->Wake();
    if (thread.joinable()) thread.join();
  }

  void Notify() {
    { std::lock_guard<std::mutex> lock(mutex); }
    cv.notify_all();
  }

  uint64_t Presented() {
    uint64_t v = UINT64_MAX;
    for (auto &l : links)
      if (l.present) v = std::min(v, l.done_present.load());
    return v == UINT64_MAX ? 0 : v;
  }

  bool AnyUp() {
    for (auto &l : links)
      if (l.present && l.ready) return true;
    return false;
  }

  void RecordArmError(int arm, const std::string &text) {
    links[arm].errors++;
    std::lock_guard<std::mutex> lock(mutex);
    links[arm].last_error = text;
  }

  void PushEvent(int arm, const g1::Event &e) {
    {
      std::lock_guard<std::mutex> lock(mutex);
      auto &q = links[arm].events;
      if (q.size() >= 256) {
        q.pop_front();
        links[arm].events_lost++;
      }
      q.push_back(e);
    }
    cv.notify_all();
  }

  // ------------------------------------------------ main loop
  void Run() {
    while (!stop) {
      Step(NowUs());
      transport->Poll(NextTimeoutMs(NowUs()));
    }
  }

  void Drain() {
    for (int i = 0; i < 2; i++) {
      Item item;
      while (queues[i].try_dequeue(item)) {
        links[i].pending.push_back(std::move(item));
        links[i].taken++;
      }
    }
  }

  // The application queued `want` items: all of them have run.
  bool Settled(int arm, uint64_t want) {
    Link &l = links[arm];
    return l.ready && l.idle && l.idle_taken >= want;
  }

  void Step(int64_t now) {
    Drain();
    TakeControls();
    for (int i = 0; i < 2; i++)
      if (int ms = hold_ms[i].exchange(0); ms > 0 && links[i].present) transport->Hold(i, ms);
    for (int i = 0; i < 2; i++) {
      Link &l = links[i];
      if (!l.present || !l.up) continue;
      Timeouts(i, now);
      if (l.phase == Link::HELLO && now >= l.hello_next) SendHello(i, now);
      if (l.phase >= Link::SETUP) {
        Profile(i, now);
        Clock(i, now);
        KeepLinkFast(i, now);
        Resend(i, now);
        AssetJobs(i, now);
      }
    }
    for (int i = 0; i < 2; i++)
      if (int64_t at = g->arms[i].black_off; at && now >= at && links[i].present) BlackOff(i);
    FirePresents(now);
    SendPackets(now);
    for (int i = 0; i < 2; i++)
      if (links[i].present && links[i].up && links[i].phase >= Link::SETUP) Probe(i, now);
    bool changed = false;
    {
      std::lock_guard<std::mutex> lock(mutex);
      for (int i = 0; i < 2; i++) {
        Link &l = links[i];
        l.queued = int(l.pending.size());
        bool idle = !l.up || (l.pending.empty() && l.unacked.empty() &&
                              l.exec_next == l.next_seq && l.draws.empty());
        changed |= idle != l.idle || (idle && l.idle_taken != l.taken);
        l.idle = idle;
        l.idle_taken = l.taken;
      }
    }
    if (changed) cv.notify_all();
  }

  int NextTimeoutMs(int64_t now) {
    int64_t next = now + 100000;
    for (int i = 0; i < 2; i++) {
      Link &l = links[i];
      if (!l.present || !l.up) continue;
      for (auto &w : l.waiting) next = std::min(next, w.second.deadline);
      if (l.phase == Link::HELLO) next = std::min(next, l.hello_next);
      if (l.phase >= Link::SETUP) {
        if (!l.ping_busy) next = std::min(next, l.next_ping);
        if (!l.unacked.empty()) next = std::min(next, l.progress + Rto(l));
        if (!l.slow) next = std::min(next, l.last_busy + int64_t(SCOUTER_G1_IDLE_AFTER_MS) * 1000);
      }
    }
    return int(std::clamp<int64_t>((next - now + 999) / 1000, 0, 100));
  }

  // RFC 6298 (VM.md §6.2), at least max(4 x interval, 100 ms), doubled per
  // resend up to x8, at most 3 s.
  static int64_t Rto(const Link &l) {
    int64_t floor = std::max<int64_t>(4 * int64_t(l.interval_us), 100000);
    int64_t estimate = l.srtt ? l.srtt + 4 * l.rttvar : 1000000;
    return std::min<int64_t>(std::max(estimate, floor) << l.backoff, 3000000);
  }

  // Nothing unacked, but packets not run or presents not shown, and the arm
  // silent: the last ACK may be lost, so ask for the window with HELLO.
  void Probe(int arm, int64_t now) {
    Link &l = links[arm];
    bool waiting = l.exec_next != l.next_seq || !l.draws.empty();
    if (!l.unacked.empty() || !waiting || l.probing || now - l.heard < 300000) return;
    l.probing = true;
    l.heard = now;
    Request(arm, CTRL_HELLO, {}, 1, 1000, [this, arm](const Reply &r) {
      Link &l = links[arm];
      l.probing = false;
      if (r.status != 0 || r.n < kHelloBytes || l.phase < Link::SETUP) return;
      if (r.epoch != l.epoch) {
        l.phase = Link::HELLO;
        l.ready = false;
        l.hello_next = 0;
        l.hello_tries = 0;
        return;
      }
      ApplyWindow(arm, Get16(r.body + 20), Get16(r.body + 22), r.body[24], 0, r.at);
    });
  }

  // ------------------------------------------------ link profiles
  // Active (7.5 ms, 2M) while packets are queued or a present waits; idle
  // (50 ms) after SCOUTER_G1_IDLE_AFTER_MS without traffic (review §3.3).
  void Profile(int arm, int64_t now) {
    Link &l = links[arm];
    bool busy = !l.pending.empty() || !l.unacked.empty() || l.exec_next != l.next_seq || !l.draws.empty() ||
                l.mic.on; // VM.md §6.5: at most 15 ms while the microphone is on
    if (busy || !started) l.last_busy = now;
    if (busy && l.slow) {
      l.slow = false;
      l.idle_profile = false;
      AskLink(arm, now);
      if (now - l.last_ping > 1000000 && !l.ping_busy) l.next_ping = now;
    } else if (!busy && !l.slow && now - l.last_busy > int64_t(SCOUTER_G1_IDLE_AFTER_MS) * 1000) {
      l.slow = true;
      l.idle_profile = true;
      AskLink(arm, now);
    }
  }

  // ------------------------------------------------ control requests
  void Request(int arm, uint8_t op, std::vector<uint8_t> body, int tries, int timeout_ms,
               std::function<void(const Reply &)> done) {
    SendRequest(arm, Link::Request{op, std::move(body), tries, timeout_ms * 1000, 0, 0,
                                   std::move(done)});
  }

  void SendRequest(int arm, Link::Request r) {
    Link &l = links[arm];
    uint8_t tag = ++l.tag;
    if (tag == 0) tag = ++l.tag;
    int64_t now = NowUs();
    r.sent = now;
    r.deadline = now + r.timeout_us;
    if (r.op == CTRL_PING) {
      r.body.clear();
      Put32(r.body, uint32_t(now));
    }
    std::vector<uint8_t> msg = {r.op, tag};
    msg.insert(msg.end(), r.body.begin(), r.body.end());
    l.waiting.erase(tag);
    l.waiting.emplace(tag, std::move(r));
    transport->Write(arm, CH_CTRL, msg.data(), msg.size());
  }

  void Timeouts(int arm, int64_t now) {
    Link &l = links[arm];
    std::vector<uint8_t> expired;
    for (auto &w : l.waiting)
      if (now >= w.second.deadline) expired.push_back(w.first);
    for (uint8_t tag : expired) {
      auto it = l.waiting.find(tag);
      if (it == l.waiting.end()) continue;
      Link::Request r = std::move(it->second);
      l.waiting.erase(it);
      if (--r.tries > 0) {
        SendRequest(arm, std::move(r));
      } else if (r.done) {
        Reply none;
        none.at = now;
        none.sent = r.sent;
        r.done(none);
      }
    }
  }

  void FailRequests(int arm) {
    auto waiting = std::move(links[arm].waiting);
    links[arm].waiting.clear();
    for (auto &w : waiting)
      if (w.second.done) w.second.done(Reply{});
  }

  void TakeControls() {
    std::vector<std::shared_ptr<ControlJob>> jobs;
    {
      std::lock_guard<std::mutex> lock(mutex);
      jobs.swap(controls);
    }
    for (auto &job : jobs) {
      Link &l = links[job->arm];
      if (!l.present || !l.up || l.phase < Link::SETUP) {
        job->reply.set_value({-1, {}});
        continue;
      }
      Request(job->arm, job->op, job->body, 3, 1500, [job](const Reply &r) {
        job->reply.set_value({r.status, std::vector<uint8_t>(r.body, r.body ? r.body + r.n : r.body)});
      });
    }
  }

  // ------------------------------------------------ session
  void LinkUp(int arm, int mtu) override {
    Link &l = links[arm];
    l.up = true;
    l.mtu = mtu;
    l.phase = Link::HELLO;
    l.hello_next = 0;
    l.hello_tries = 0;
    Log("%s arm: link up, MTU %d", kArmName[arm], mtu);
  }

  void LinkDown(int arm) override {
    Link &l = links[arm];
    if (!l.up) return;
    l.up = false;
    l.phase = Link::DOWN;
    l.ready = false;
    l.ping_busy = false;
    l.reconnects++;
    l.probing = false;
    FailRequests(arm);
    uint64_t done = l.done_present;
    for (auto &d : l.draws) done = std::max(done, d.present);
    l.draws.clear();
    l.done_present = done;
    Log("%s arm: link down", kArmName[arm]);
    MicStopped(arm, MIC_STOP_LINK);
    l.cap = 42; // its cap is no longer known: the other eye stops following it
    MatchBrightness(1 - arm);
  }

  void MicStopped(int arm, MicStopReason reason) {
    {
      std::lock_guard<std::mutex> lock(mutex);
      Link::Mic &m = links[arm].mic;
      if (m.on) {
        m.on = false;
        m.stopped = reason;
        Log("%s arm: microphone stopped (%d)", kArmName[arm], int(reason));
      }
    }
    cv.notify_all();
  }

  void Found(int arm, const std::string &address) override {
    {
      std::lock_guard<std::mutex> lock(mutex);
      found[arm] = true;
      links[arm].address = address;
    }
    Log("%s arm: %s", kArmName[arm], address.c_str());
    cv.notify_all();
  }

  void Problem(InitError error, const std::string &message) override {
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (problem == ERROR_NONE) {
        problem = error;
        problem_text = message;
      }
    }
    Log("problem: %s", message.c_str());
    cv.notify_all();
  }

  void SendHello(int arm, int64_t now) {
    Link &l = links[arm];
    if (++l.hello_tries > 12) {
      Problem(ERROR_BLUETOOTH_ERROR, std::string("the ") + kArmName[arm] +
                                         " arm connects but does not answer (not bonded with "
                                         "this computer? pair in the open case)");
      l.hello_tries = 0;
      transport->Disconnect(arm);
      return;
    }
    l.hello_next = now + 1000000;
    Request(arm, CTRL_HELLO, {}, 1, 1000, [this, arm](const Reply &r) {
      if (r.status == 0 && r.n >= kHelloBytes) OnHello(arm, r);
    });
  }

  void Firmware(int arm, int protocol) {
    Problem(ERROR_G1_FIRMWARE, std::string("the ") + kArmName[arm] + " arm runs protocol " +
                                   std::to_string(protocol) + "; this library needs protocol 5");
    links[arm].phase = Link::DOWN;
  }

  void ApplyHello(int arm, const uint8_t *b) {
    Link &l = links[arm];
    l.firmware = b[1] << 16 | b[2] << 8 | b[3];
    l.safe = (b[5] & 0x01) != 0;
    l.encrypted = (b[5] & 0x40) != 0;
    l.in_case = (b[5] & 0x80) != 0;
    l.microphone = (b[6] & 0x04) != 0;
    l.triggers = Get16(b + 7);
  }

  void OnHello(int arm, const Reply &r) {
    Link &l = links[arm];
    const uint8_t *b = r.body;
    if (l.phase != Link::HELLO) return;
    if (forget[arm].exchange(false)) {
      l.known = false;
      l.unacked.clear();
    }
    if (b[0] != 5) return Firmware(arm, b[0]);
    if (automatic[arm] && (b[6] & 0x01) && transport->Reject(arm)) {
      Log("%s arm: no panel (a development board?), trying another", kArmName[arm]);
      l.phase = Link::DOWN;
      return;
    }
    ApplyHello(arm, b);
    if ((b[4] == 1) != (arm == 0))
      Log("%s arm: it reports the %s role", kArmName[arm], b[4] == 1 ? "left" : "right");
    l.max_packet = std::min({int(Get16(b + 18)), l.mtu - 3, SCOUTER_G1_MAX_PACKET});
    l.max_packet_seen = l.max_packet;
    uint16_t rx = Get16(b + 20), ex = Get16(b + 22);
    uint16_t oldest = l.unacked.empty() ? l.next_seq : l.unacked.front().seq;
    bool consistent = uint16_t(rx - oldest) <= uint16_t(l.next_seq - oldest);
    if (l.known && r.epoch == l.epoch && consistent) {
      l.rx_next = rx;
      l.exec_next = ex;
      l.credits = b[24];
      l.sack = 0;
      while (!l.unacked.empty() && SeqBefore(l.unacked.front().seq, rx)) l.unacked.pop_front();
      for (auto &s : l.unacked) {
        transport->Write(arm, CH_CMD, s.data.data(), s.data.size());
        s.last = r.at;
        s.resent = true;
      }
      l.progress = r.at;
      while (!l.draws.empty() && SeqBefore(l.draws.front().seq, ex)) {
        l.done_present = std::max(l.done_present.load(), l.draws.front().present);
        l.draws.pop_front();
      }
      Log("%s arm: same epoch %08x, resent %zu packets", kArmName[arm], r.epoch, l.unacked.size());
      // its panel went off meanwhile (the 30 s rule, a case hold that ended): shown again
      if (!(b[5] & 0x08) && !(b[5] & 0x80)) RedrawPanel(arm, true);
      ReadAssets(arm, nullptr);
      StartSetup(arm, 2);
      return;
    }
    if (g->keep && !l.known) return Adopt(arm, r);
    if (l.known && r.epoch != l.unknown_epoch) {
      l.epoch_changes++;
      Log("%s arm: new epoch %08x (was %08x): its memory is gone", kArmName[arm], r.epoch, l.epoch);
    }
    l.unknown_epoch = r.epoch;
    l.phase = Link::RESET;
    Request(arm, CTRL_RESET_SESSION, {1}, 4, 1000, [this, arm](const Reply &r) {
      if (r.status == 0 && r.n >= 2 + kHelloBytes)
        OnReset(arm, r);
      else if (links[arm].up)
        transport->Disconnect(arm);
    });
  }

  // Connection::keep_sessions: the arm's session goes on as it is, with its
  // program and memory; the library's packets follow the window it reports.
  void Adopt(int arm, const Reply &r) {
    Link &l = links[arm];
    const uint8_t *b = r.body;
    l.known = true;
    l.epoch = l.unknown_epoch = r.epoch;
    l.epoch_seen = r.epoch;
    l.rx_next = l.next_seq = Get16(b + 20);
    l.exec_next = Get16(b + 22);
    l.credits = b[24];
    l.sack = 0;
    l.unacked.clear();
    l.progress = r.at;
    Log("%s arm: kept its session %08x", kArmName[arm], r.epoch);
    ReadAssets(arm, nullptr);
    StartSetup(arm, 8);
  }

  void OnReset(int arm, const Reply &r) {
    Link &l = links[arm];
    const uint8_t *hello = r.body + 2;
    if (l.phase != Link::RESET) return;
    bool restore = l.known;
    ApplyHello(arm, hello);
    l.known = true;
    l.epoch = r.epoch;
    l.epoch_seen = r.epoch;
    l.rx_next = Get16(hello + 20);
    l.exec_next = Get16(hello + 22);
    l.next_seq = l.rx_next;
    l.credits = hello[24];
    l.sack = 0;
    l.unacked.clear();
    l.progress = r.at;
    uint64_t done = l.done_present;
    for (auto &d : l.draws) done = std::max(done, d.present);
    l.draws.clear();
    l.done_present = done;
    uint32_t epoch = r.epoch;
    int64_t at = r.at;
    ReadAssets(arm, [this, arm, restore, epoch, at] {
      if (links[arm].phase != Link::RESET || links[arm].epoch != epoch) return;
      Restore(arm, !restore);
      if (restore) {
        g1::Event e{};
        e.type = EVENT_RESET;
        e.epoch = epoch;
        e.host_us = at;
        e.trigger = 0xFF;
        PushEvent(arm, e);
      }
      StartSetup(arm, 8);
    });
  }

  void StartSetup(int arm, int pings) {
    Link &l = links[arm];
    l.phase = Link::SETUP;
    l.setup_pings = pings;
    l.ping_busy = false;
    l.next_ping = 0;
    l.slow = false;
    l.idle_profile = false;
    l.last_busy = NowUs();
    AskLink(arm, NowUs());
    Request(arm, CTRL_STATUS, {}, 2, 1500, [this, arm](const Reply &r) {
      if (r.status == 0 && r.n >= 33) SetInterval(arm, Get16(r.body + 31));
      if (r.status == 0 && r.n >= 38 && (r.body[37] & 0x0F)) links[arm].phy = r.body[37] & 0x0F;
    });
    if (uint16_t mask = l.inputs) {
      std::vector<uint8_t> body;
      Put16(body, mask);
      Request(arm, CTRL_EVENTS, body, 2, 1500, nullptr);
    }
    // The panel guard (0.5.52+; an older firmware answers 2, and its BRIGHTNESS_MAX is never written), then
    // its cap now, which the other eye matches.
    l.guard = false;
    l.max_sent = 42;
    Request(arm, CTRL_PANEL_GUARD, {}, 2, 1500, [this, arm](const Reply &r) {
      links[arm].guard = r.status == 0 && r.n >= 1;
      if (!links[arm].guard) return;
      Request(arm, CTRL_PROPS, {VM_P_BRIGHTNESS_CAP}, 2, 1500, [this, arm](const Reply &r) {
        if (r.status == 0 && r.n >= 4 && Get32(r.body) <= 42) OnCap(arm, uint8_t(Get32(r.body)));
      });
    });
  }

  // ------------------------------------------------ the asset store (VM.md §6.10)
  // ASSET_INFO, then the directory page by page; `then` runs after it, also
  // without a store or a reply (the inventory then stays as it was).
  void ReadAssets(int arm, std::function<void()> then) {
    Request(arm, CTRL_ASSET_INFO, {}, 2, 1500, [this, arm, then](const Reply &r) {
      if (r.status == 0 && r.n >= 20 && (r.body[15] & 1)) return ListAssets(arm, 0, {}, then);
      if (r.status >= 0) {
        std::lock_guard<std::mutex> lock(mutex);
        links[arm].assets.store = false;
        links[arm].assets.entries.clear();
      }
      if (then) then();
    });
  }

  void ListAssets(int arm, int first, std::map<AssetId, AssetEntry> got, std::function<void()> then) {
    Request(arm, CTRL_ASSET_LIST, {uint8_t(first)}, 2, 1500,
            [this, arm, first, got = std::move(got), then](const Reply &r) mutable {
              int k = r.status == 0 && r.n >= 2 ? int(r.n - 2) / 32 : 0;
              for (int i = 0; i < k; i++) {
                AssetEntry e = EntryOf(r.body + 2 + 32 * i);
                got[e.id] = e;
              }
              if (k > 0 && first + k < r.body[0]) return ListAssets(arm, first + k, std::move(got), then);
              if (r.status == 0) {
                std::lock_guard<std::mutex> lock(mutex);
                links[arm].assets.store = true;
                links[arm].assets.entries = std::move(got);
              }
              if (then) then();
            });
  }

  // 1: the arm holds the asset, 0: it does not, -1: it has no store.
  int HasAsset(int arm, const AssetId &id) {
    std::lock_guard<std::mutex> lock(mutex);
    const Link::Assets &a = links[arm].assets;
    return !a.store ? -1 : a.entries.count(id) ? 1 : 0;
  }

  std::future<int> PushJob(int arm, Link::AssetJob job, bool wait) {
    std::future<int> f;
    if (wait) {
      job.done = std::make_shared<std::promise<int>>();
      f = job.done->get_future();
    }
    {
      std::lock_guard<std::mutex> lock(mutex);
      links[arm].asset_jobs.push_back(std::move(job));
    }
    transport->Wake();
    return f;
  }

  void SetItems(int arm, uint64_t tag, int items) {
    std::lock_guard<std::mutex> lock(mutex);
    for (auto &j : links[arm].asset_jobs)
      if (j.tag == tag) j.items = items;
  }

  // A staging command of the job went out in packet seq.
  void Tagged(int arm, uint64_t tag, uint16_t seq) {
    std::lock_guard<std::mutex> lock(mutex);
    for (auto &j : links[arm].asset_jobs)
      if (j.tag == tag) {
        j.packed++;
        j.seq = seq;
      }
  }

  Link::AssetJob *FrontJob(int arm, uint64_t tag) {
    auto &q = links[arm].asset_jobs;
    return !q.empty() && q.front().tag == tag ? &q.front() : nullptr;
  }

  // The front job: its request once its staging has run on the arm (ASSET_PUT
  // reads the arena on arrival), then ASSET_INFO every second until it ends,
  // in case NOTICE 8 is lost.
  void AssetJobs(int arm, int64_t now) {
    Link &l = links[arm];
    std::unique_lock<std::mutex> lock(mutex);
    if (l.asset_jobs.empty() || l.phase != Link::READY) return;
    Link::AssetJob &j = l.asset_jobs.front();
    uint64_t tag = j.tag;
    if (now < j.at || j.state == Link::AssetJob::SENT) return;
    if (j.state == Link::AssetJob::RUNNING) {
      j.at = now + 1000000;
      lock.unlock();
      Request(arm, CTRL_ASSET_INFO, {}, 1, 1500, [this, arm, tag](const Reply &r) {
        if (r.status != 0 || r.n < 20) return;
        if (r.body[13] == 4) Finish(arm, tag, 0);
        if (r.body[13] == 5) Finish(arm, tag, r.body[14] ? int(r.body[14]) : int(VM_E_ASSET));
      });
      return;
    }
    if (j.items < 0 || j.packed < j.items || (j.items > 0 && !SeqBefore(j.seq, l.exec_next))) return;
    std::vector<uint8_t> body(j.id.b, j.id.b + 16);
    if (j.op == CTRL_ASSET_PUT) {
      Put32(body, j.address);
      Put32(body, j.size);
      body.push_back(j.type);
    }
    j.state = Link::AssetJob::SENT;
    uint8_t op = j.op;
    lock.unlock();
    Request(arm, op, body, 3, 1500, [this, arm, tag](const Reply &r) { OnJobReply(arm, tag, r); });
  }

  void OnJobReply(int arm, uint64_t tag, const Reply &r) {
    if (r.status == 0 && r.n >= 1 && r.body[0] == 1) return Finish(arm, tag, 0); // already stored
    if (r.status == 0 || r.status < 0) {
      std::lock_guard<std::mutex> lock(mutex);
      if (Link::AssetJob *j = FrontJob(arm, tag)) {
        j->state = r.status == 0 ? Link::AssetJob::RUNNING : Link::AssetJob::WAITING;
        j->at = NowUs() + (r.status == 0 ? 1000000 : 500000);
      }
      return;
    }
    if (r.status != VM_E_ASSET) return Finish(arm, tag, r.status);
    // 13: a job of another host runs, or no space (a delete: no such asset, so it is gone): ASSET_INFO tells
    bool del = false;
    {
      std::lock_guard<std::mutex> lock(mutex);
      Link::AssetJob *j = FrontJob(arm, tag);
      del = j && j->op == CTRL_ASSET_DELETE;
    }
    Request(arm, CTRL_ASSET_INFO, {}, 2, 1500, [this, arm, tag, del](const Reply &q) {
      if (!(q.status == 0 && q.n >= 20 && q.body[13] >= 1 && q.body[13] <= 3))
        return Finish(arm, tag, del ? 0 : int(VM_E_ASSET));
      std::lock_guard<std::mutex> lock(mutex);
      if (Link::AssetJob *j = FrontJob(arm, tag)) {
        j->state = Link::AssetJob::WAITING;
        j->at = NowUs() + 300000;
      }
    });
  }

  // NOTICE 8: u8 job (1 put, 2 delete), u8 result, the ID.
  void OnAssetNotice(int arm, int job, int result, const uint8_t *id) {
    uint64_t tag = 0;
    {
      std::lock_guard<std::mutex> lock(mutex);
      auto &q = links[arm].asset_jobs;
      if (q.empty() || q.front().state != Link::AssetJob::RUNNING || job < 1 || job > 2 ||
          (job == 1) != (q.front().op == CTRL_ASSET_PUT) || std::memcmp(q.front().id.b, id, 16) != 0)
        return;
      tag = q.front().tag;
    }
    Finish(arm, tag, result);
  }

  void Finish(int arm, uint64_t tag, int result) {
    std::shared_ptr<std::promise<int>> done;
    {
      std::lock_guard<std::mutex> lock(mutex);
      Link::AssetJob *j = FrontJob(arm, tag);
      if (!j) return;
      auto &e = links[arm].assets.entries;
      if (result == 0 && j->op == CTRL_ASSET_PUT) e[j->id] = AssetEntry{j->id, 0, j->size, 0, j->type};
      if (result == 0 && j->op == CTRL_ASSET_DELETE) e.erase(j->id);
      if (result)
        Log("%s arm: asset %s failed (%d %s)", kArmName[arm], j->op == CTRL_ASSET_PUT ? "put" : "delete", result,
            CodeName(result));
      done = std::move(j->done);
      links[arm].asset_jobs.pop_front();
    }
    if (done) done->set_value(result);
    cv.notify_all();
  }

  // The arm lost its session: its jobs end (their staging is gone).
  void DropJobs(int arm) {
    std::deque<Link::AssetJob> q;
    {
      std::lock_guard<std::mutex> lock(mutex);
      q.swap(links[arm].asset_jobs);
    }
    for (auto &j : q)
      if (j.done) j.done->set_value(-1);
    cv.notify_all();
  }

  // NOTICE 7 or the cap read at setup: the other eye gets it as its BRIGHTNESS_MAX (VM.md §6.6).
  void OnCap(int arm, uint8_t cap) {
    if (cap != links[arm].cap) Log("%s arm: panel guard cap %u", kArmName[arm], cap);
    links[arm].cap = cap;
    MatchBrightness(1 - arm);
  }

  void SetInterval(int arm, int units) {
    if (units <= 0) return;
    if (links[arm].interval_us != units * 1250)
      Log("%s arm: interval %.2f ms", kArmName[arm], units * 1.25);
    links[arm].interval_us = units * 1250;
    links[arm].interval_seen = units * 1250;
  }

  // PING every second while active, every 30 s on the idle link.
  void Clock(int arm, int64_t now) {
    Link &l = links[arm];
    if (l.ping_busy || now < l.next_ping) return;
    l.ping_busy = true;
    l.last_ping = now;
    Request(arm, CTRL_PING, {}, 1, 1000, [this, arm](const Reply &r) {
      Link &l = links[arm];
      l.ping_busy = false;
      if (r.status == 0 && r.n >= 12) AddSample(arm, r);
      if (l.setup_pings > 0) l.setup_pings--;
      l.next_ping = r.at + (l.setup_pings > 0 ? 0 : l.slow ? 30000000 : 1000000);
      if (l.setup_pings == 0 && l.phase == Link::SETUP && l.clock_error_us >= 0) {
        l.phase = Link::READY;
        l.ready = true;
        l.moved++;
        Log("%s arm: ready, clock error %d us", kArmName[arm], l.clock_error_us.load());
        MatchBrightness(arm);
        Notify();
      }
    });
  }

  void AddSample(int arm, const Reply &r) {
    Link &l = links[arm];
    uint32_t t_rx = Get32(r.body + 4), t_tx = Get32(r.body + 8);
    uint32_t d1 = t_rx - uint32_t(r.sent);
    uint32_t d2 = t_tx - uint32_t(r.at);
    uint32_t offset = d1 + uint32_t(int32_t(d2 - d1) / 2);
    int64_t delay = (r.at - r.sent) - int64_t(uint32_t(t_tx - t_rx));
    if (delay < 0) return;
    l.samples.push_back({r.at, offset, delay});
    if (l.samples.size() > 16) l.samples.pop_front();
    double best = 1e18;
    for (auto &x : l.samples) {
      double err = double(x.delay) / 2.0 + double(r.at - x.at) * 130e-6;
      if (err < best) {
        best = err;
        l.offset = x.offset;
      }
    }
    l.clock_error_us = int(best);
  }

  void Restore(int arm, bool fresh);
  void PanelFromMirror(int arm, bool fresh);
  void RedrawPanel(int arm, bool if_shown);
  void MatchBrightness(int arm);
  void EndMatch(int arm);
  void BlackOff(int arm);

  // ------------------------------------------------ packets
  bool Sacked(const Link &l, uint16_t seq) {
    uint16_t i = uint16_t(seq - l.rx_next - 1);
    return i < 32 && ((l.sack >> i) & 1);
  }

  // The oldest unacked packet again when no ACK moved rx_next for RTO.
  void Resend(int arm, int64_t now) {
    Link &l = links[arm];
    if (l.unacked.empty() || now - l.progress <= Rto(l)) return;
    auto &s = l.unacked.front();
    if (!Sacked(l, s.seq)) {
      transport->Write(arm, CH_CMD, s.data.data(), s.data.size());
      s.last = now;
      s.resent = true;
      l.resent++;
      l.backoff = std::min(l.backoff + 1, 3);
    }
    l.progress = now;
  }

  bool Sending(int arm) {
    Link &l = links[arm];
    return l.present && l.up && l.phase >= Link::SETUP && !l.pending.empty() &&
           (l.pending.front().kind == Item::COMMAND || l.pending.front().present->fired);
  }

  // New packets within each arm's window, the arms taking turns. Both links
  // share the PC's controller and its few buffers: while both arms have packets
  // to send, one that sent more bytes than the other waits, so both uploads end
  // together. An arm whose own queue is full (SLEEP_UNTIL) or that went silent
  // does not hold the other back.
  void SendPackets(int64_t now) {
    bool busy[2], open[2];
    for (int i = 0; i < 2; i++) {
      open[i] = Sending(i);
      busy[i] = open[i] && links[i].credits >= SCOUTER_G1_IN_FLIGHT && now - links[i].heard < 500000;
    }
    for (int i = 0; i < 2; i++)
      if (!busy[i]) links[i].sent_bytes = std::max(links[i].sent_bytes, links[1 - i].sent_bytes);
    for (bool sent = true; sent;) {
      sent = false;
      for (int i = 0; i < 2; i++) {
        int j = 1 - i;
        if (!open[i] || (busy[j] && links[i].sent_bytes > links[j].sent_bytes + SCOUTER_G1_MAX_PACKET))
          continue;
        open[i] = SendOne(i, now);
        busy[i] = busy[i] && Sending(i);
        sent |= open[i];
      }
    }
  }

  // One new packet if the arm's window allows it: false when none was sent.
  bool SendOne(int arm, int64_t now) {
    Link &l = links[arm];
    int window = std::min(l.credits, SCOUTER_G1_IN_FLIGHT);
    while (int(uint16_t(l.next_seq - l.rx_next)) < window && !l.pending.empty()) {
      Item &front = l.pending.front();
      if (front.kind == Item::PRESENT) {
        if (!front.present->fired) return false;
        auto job = front.present;
        l.pending.pop_front();
        PushDraws(arm, *job, uint32_t(now) + l.offset.load()); // late: at once
        continue;
      }
      std::vector<uint8_t> pkt = {uint8_t(l.next_seq), uint8_t(l.next_seq >> 8)};
      while (!l.pending.empty() && l.pending.front().kind == Item::COMMAND) {
        Item &it = l.pending.front();
        if (pkt.size() + it.cmd.size > size_t(l.max_packet)) break;
        pkt.insert(pkt.end(), it.cmd.b.begin(), it.cmd.b.begin() + it.cmd.size);
        if (it.last_draw && it.present_id) l.draws.push_back({l.next_seq, it.present_id, it.at});
        if (it.tag) Tagged(arm, it.tag, l.next_seq);
        l.pending.pop_front();
      }
      if (pkt.size() == 2) {
        Log("%s arm: a %d-byte command does not fit a %d-byte packet; dropped", kArmName[arm],
            l.pending.front().cmd.size, l.max_packet);
        l.pending.pop_front();
        continue;
      }
      transport->Write(arm, CH_CMD, pkt.data(), pkt.size());
      if (l.unacked.empty()) l.progress = now;
      l.sent_bytes += pkt.size();
      l.unacked.push_back({l.next_seq, std::move(pkt), now, now, false});
      l.next_seq++;
      l.packets++;
      return true;
    }
    return false;
  }

  // While active, asks again for 7.5 ms on the 2M PHY when a bulk transfer
  // finds the link slower, at most every 5 s (an arm sends the request at the
  // earliest 5 s after connecting).
  void KeepLinkFast(int arm, int64_t now) {
    Link &l = links[arm];
    bool slow = l.interval_us > SCOUTER_G1_LINK_MAX * 1250 || l.phy == 1;
    bool bulk = !l.pending.empty() && !l.unacked.empty();
    if (!l.slow && slow && bulk && now - l.link_asked > 5000000) AskLink(arm, now);
  }

  void AskLink(int arm, int64_t now) {
    Link &l = links[arm];
    l.link_asked = now;
    std::vector<uint8_t> link;
    Put16(link, l.slow ? SCOUTER_G1_IDLE_LINK : SCOUTER_G1_LINK_MIN);
    Put16(link, l.slow ? SCOUTER_G1_IDLE_LINK : SCOUTER_G1_LINK_MAX);
    Put16(link, 0);
    Put16(link, 400);
    link.push_back(2);
    Log("%s arm: asking for the %s link", kArmName[arm], l.slow ? "idle" : "active");
    Request(arm, CTRL_LINK, link, 2, 1500, nullptr);
  }

  // Puts the arm's part of a present at the front of its queue.
  void PushDraws(int arm, PresentJob &job, uint32_t at) {
    Link &l = links[arm];
    auto &draws = job.draws[arm];
    if (draws.empty()) {
      l.done_present = std::max(l.done_present.load(), job.id);
      Notify();
      return;
    }
    Item tail;
    tail.cmd = PresentTail();
    tail.last_draw = true;
    tail.present_id = job.id;
    tail.at = at;
    l.pending.push_front(std::move(tail));
    for (size_t k = draws.size(); k-- > 0;) {
      Item item;
      item.cmd = draws[k];
      item.present_id = job.id;
      l.pending.push_front(std::move(item));
    }
    Item head;
    head.cmd = PresentHead(at);
    head.present_id = job.id;
    l.pending.push_front(std::move(head));
  }

  bool Usable(int arm) {
    Link &l = links[arm];
    return l.present && l.up && l.phase >= Link::SETUP && l.clock_error_us >= 0;
  }

  bool AtFront(int arm, const std::shared_ptr<PresentJob> &job) {
    Link &l = links[arm];
    return !l.pending.empty() && l.pending.front().kind == Item::PRESENT &&
           l.pending.front().present == job;
  }

  // A present waits until every usable arm has received all its earlier
  // packets, then both get SLEEP_UNTIL(T) + their DRAWs.
  void FirePresents(int64_t now) {
    for (int i = 0; i < 2; i++) {
      Link &l = links[i];
      if (!l.present || l.pending.empty() || l.pending.front().kind != Item::PRESENT) continue;
      auto job = l.pending.front().present;
      if (job->fired) continue;
      bool ok = true;
      for (int j = 0; j < 2 && ok; j++)
        if (Usable(j) && !job->dropped[j]) ok = AtFront(j, job) && links[j].rx_next == links[j].next_seq;
      if (!ok) continue;
      int64_t interval = 0, error = 0;
      for (int j = 0; j < 2; j++) {
        if (!Usable(j) || job->dropped[j]) continue;
        interval = std::max<int64_t>(interval, links[j].interval_us);
        error = std::max<int64_t>(error, links[j].clock_error_us);
      }
      int64_t at = now + 2 * interval + error + 15000;
      job->fired = true;
      for (int j = 0; j < 2; j++) {
        Link &o = links[j];
        if (!o.present) continue;
        if (Usable(j) && AtFront(j, job)) {
          o.pending.pop_front();
          PushDraws(j, *job, uint32_t(at) + o.offset.load());
        } else {
          o.done_present = std::max(o.done_present.load(), job->id);
        }
      }
      Notify();
    }
  }

  // ------------------------------------------------ events
  void Data(int arm, const uint8_t *d, size_t n) override {
    if (n < 2) return;
    Link &l = links[arm];
    int64_t now = NowUs();
    uint8_t type = d[0];
    if (type == (0x80 | CTRL_HELLO) && n > 11 && d[11] == 4) {
      auto it = l.waiting.find(d[1]);
      if (it != l.waiting.end() && it->second.op == CTRL_HELLO && l.phase == Link::HELLO) {
        l.waiting.erase(it);
        Firmware(arm, 4);
        return;
      }
    }
    if (n < 9) return;
    uint32_t epoch = Get32(d + 1);
    if (type & 0x80) {
      if (n < 11) return;
      auto it = l.waiting.find(d[9]);
      if (it == l.waiting.end() || it->second.op != (type & 0x7F)) return;
      Link::Request req = std::move(it->second);
      l.waiting.erase(it);
      l.heard = now;
      Reply r;
      r.status = d[10];
      r.body = d + 11;
      r.n = n - 11;
      r.epoch = epoch;
      r.at = now;
      r.sent = req.sent;
      if (req.done) req.done(r);
      return;
    }
    switch (type) {
    case EVT_ACK:
      if (n >= 18) OnAck(arm, epoch, d + 9, now);
      break;
    case EVT_ERROR:
      if (n >= 22 && epoch == l.epoch) {
        l.heard = now;
        g1::Event e = MakeEvent(EVENT_ERROR, d, now);
        char text[200];
        std::snprintf(text, sizeof(text), "arm ERROR: %s in packet %u (offset %d, address 0x%x): %s (%d)",
                      OpName(uint8_t(e.op)), unsigned(e.count), e.offset, unsigned(e.address),
                      CodeName(e.code), e.code);
        Log("%s arm: %s", kArmName[arm], text);
        RecordArmError(arm, text);
        PushEvent(arm, e);
      }
      break;
    case EVT_HALT:
      if (n >= 20 && epoch == l.epoch) {
        l.heard = now;
        g1::Event e = MakeEvent(EVENT_HALT, d, now);
        Log("%s arm: program HALT at 0x%x (%s, %s)", kArmName[arm], unsigned(e.address),
            OpName(uint8_t(e.op)), e.code ? CodeName(e.code) : "HALT");
        PushEvent(arm, e);
      }
      break;
    case EVT_NOTIFY:
      if (n >= 20 && epoch == l.epoch) {
        l.heard = now;
        g1::Event e = MakeEvent(EVENT_NOTIFY, d, now);
        if (e.trigger == 0xFF && e.a == e.b && OnDrawDone(arm, e.a, e.device_us, now)) break;
        PushEvent(arm, e);
      }
      break;
    case EVT_INPUT:
      if (n >= 16 && epoch == l.epoch) PushEvent(arm, MakeEvent(EVENT_INPUT, d, now));
      break;
    case EVT_MIC_DATA:
      if (n > 12 && epoch == l.epoch) OnMicData(arm, d, n, now);
      break;
    case EVT_NOTICE:
      if (n >= 10) {
        if (d[9] == 1 && n >= 16) SetInterval(arm, Get16(d + 10));
        if (d[9] == 2 && n >= 12) {
          l.phy = d[10];
          Log("%s arm: PHY %dM", kArmName[arm], l.phy);
        }
        if (d[9] == 4) {
          l.encrypted = true;
          if (l.phase == Link::HELLO) l.hello_next = 0;
        }
        if (d[9] == 5 && n >= 11) {
          if (d[10] == 2) l.microphone = false; // HELLO.board bit 2 is clear until a reboot
          MicStopped(arm, d[10] == 1 || d[10] == 4 ? MIC_STOP_PANEL_OFF
                          : d[10] == 3              ? MIC_STOP_LEASE
                                                    : MIC_STOP_FAULT);
        }
        if (d[9] == 6 && n >= 11) {
          Log("%s arm: case hold %s", kArmName[arm], d[10] ? "on (its panel off in the case)" : "off");
          if (!d[10] && epoch == l.epoch) RedrawPanel(arm, false);
        }
        if (d[9] == 7 && n >= 12 && d[10] <= 42) OnCap(arm, d[10]);
        if (d[9] == 8 && n >= 28 && epoch == l.epoch) OnAssetNotice(arm, d[10], d[11], d + 12);
      }
      break;
    default:
      break;
    }
  }

  // MIC_DATA: u16 counter of the first frame, u8 n, n frames (VM.md §6.5).
  // A gap in the counter is concealed, up to kMicConceal frames.
  // A frame that arrives renews the arm's lease at most every kMicRenewUs: a
  // host that stops reading (killed, hung) lets the microphone go off by itself.
  void OnMicData(int arm, const uint8_t *d, size_t n, int64_t now) {
    int count = d[11];
    if (count == 0 || (n - 12) % size_t(count)) return;
    int bytes = int(n - 12) / count;
    bool renew = false;
    {
      std::lock_guard<std::mutex> lock(mutex);
      Link::Mic &m = links[arm].mic;
      if (!m.on || bytes != m.bytes) return;
      if (m.started && !g_hold_renewals && now - m.renewed >= kMicRenewUs) {
        m.renewed = now;
        renew = true;
      }
      uint16_t ahead = uint16_t(Get16(d + 9) - uint16_t(m.next));
      if (ahead >= 0x8000) return;
      m.state.events++;
      if (ahead) {
        m.state.gaps++;
        m.state.missing += ahead;
      }
      uint32_t skip = ahead > kMicConceal ? ahead - kMicConceal : 0;
      m.next += skip;
      for (uint32_t k = skip; k < ahead; k++) MicFrameOut(m, nullptr, 0, Get32(d + 5), now);
      for (int k = 0; k < count; k++) MicFrameOut(m, d + 12 + k * bytes, bytes, Get32(d + 5), now);
    }
    cv.notify_all();
    if (renew) Request(arm, CTRL_MIC, {1, uint8_t(bytes == 20 ? 1 : 0)}, 1, 1000, [](const Reply &) {});
  }

  void MicFrameOut(Link::Mic &m, const uint8_t *frame, int bytes, uint32_t device_us, int64_t now) {
    MicFrame f{};
    f.counter = m.next++;
    f.device_us = device_us;
    f.host_us = now;
#ifdef SCOUTER_G1_MIC
    int r = lc3_decode(static_cast<lc3_decoder_t>(m.decoder), frame, bytes, LC3_PCM_FORMAT_S16,
                       f.pcm.data(), 1);
#else
    (void)bytes;
    int r = 1;
#endif
    f.concealed = r != 0;
    if (frame) {
      m.state.frames++;
      if (r != 0) m.state.bad++;
    }
    if (m.frames.size() >= kMicKeep) {
      m.frames.pop_front();
      m.state.dropped++;
    }
    m.frames.push_back(f);
  }

  void OnAck(int arm, uint32_t epoch, const uint8_t *b, int64_t now) {
    Link &l = links[arm];
    if (l.phase < Link::SETUP) return;
    if (epoch != l.epoch) {
      Log("%s arm: ack with epoch %08x, expected %08x", kArmName[arm], epoch, l.epoch);
      l.phase = Link::HELLO;
      l.ready = false;
      l.hello_next = 0;
      l.hello_tries = 0;
      return;
    }
    ApplyWindow(arm, Get16(b), Get16(b + 2), b[4], Get32(b + 5), now);
  }

  void ApplyWindow(int arm, uint16_t rx, uint16_t ex, int credits, uint32_t sack, int64_t now) {
    Link &l = links[arm];
    if (uint16_t(rx - l.rx_next) > uint16_t(l.next_seq - l.rx_next)) return;
    l.heard = now;
    if (rx != l.rx_next) {
      l.progress = now;
      l.backoff = 0;
    }
    if (rx != l.rx_next || ex != l.exec_next) l.moved++;
    l.rx_next = rx;
    l.exec_next = ex;
    l.credits = credits;
    l.sack = sack;
    if (sack) // a hole delays the ACK of every packet after it: no RTT samples
      for (auto &s : l.unacked) s.resent = true;
    int64_t sample = -1;
    while (!l.unacked.empty() && SeqBefore(l.unacked.front().seq, rx)) {
      if (!l.unacked.front().resent) sample = now - l.unacked.front().first;
      l.unacked.pop_front();
    }
    if (sample >= 0) {
      if (!l.srtt) {
        l.srtt = sample;
        l.rttvar = sample / 2;
      } else {
        l.rttvar = (3 * l.rttvar + std::abs(l.srtt - sample)) / 4;
        l.srtt = (7 * l.srtt + sample) / 8;
      }
    }
    bool lost_done = false;
    while (!l.draws.empty() && SeqBefore(l.draws.front().seq, ex)) {
      l.done_present = std::max(l.done_present.load(), l.draws.front().present);
      l.draws.pop_front();
      lost_done = true;
    }
    if (lost_done) Notify();
    if (!l.sack) return;
    int top = 32 - std::countl_zero(l.sack);
    for (int i = 0; i < top; i++) {
      uint16_t seq = uint16_t(rx + i);
      if (i > 0 && ((l.sack >> (i - 1)) & 1)) continue;
      for (auto &s : l.unacked) {
        if (s.seq != seq) continue;
        if (now - s.last > 20000) {
          transport->Write(arm, CH_CMD, s.data.data(), s.data.size());
          s.last = now;
          s.resent = true;
          l.resent++;
        }
        break;
      }
    }
  }

  // The NOTIFY at the end of a present: when the eye showed it.
  bool OnDrawDone(int arm, uint32_t at, uint32_t device_us, int64_t now) {
    Link &l = links[arm];
    for (auto it = l.draws.begin(); it != l.draws.end(); ++it) {
      if (it->at != at) continue;
      l.done_present = std::max(l.done_present.load(), it->present);
      l.shown_update = it->present;
      l.shown_us = now + int32_t(device_us - (uint32_t(now) + l.offset.load()));
      l.moved++;
      l.draws.erase(it);
      Notify();
      return true;
    }
    return false;
  }
};

void RecordError(int arm, const std::string &text) {
  Log("%s arm: %s", kArmName[arm], text.c_str());
  g->core->RecordArmError(arm, text);
}

bool SafeMode(int arm) { return g && g->core && g->core->links[arm].safe; }

// ---------------------------------------------------------------- gray display on the G1

// ---------------------------------------------------------------- kept fonts and bitmaps

// Stages the blob at `at` with WRITEs and queues ASSET_PUT of that range for
// when they have run (API lock held).
std::future<int> StagedPut(int arm, Address at, std::span<const uint8_t> blob, uint8_t type, bool wait) {
  Core *core = g->core.get();
  Link::AssetJob j;
  j.tag = ++g->tags;
  j.op = CTRL_ASSET_PUT;
  j.id = AssetIdOf(blob);
  j.address = at;
  j.size = uint32_t(blob.size());
  j.type = type;
  j.items = -1;
  uint64_t tag = j.tag;
  std::future<int> f = core->PushJob(arm, std::move(j), wait);
  g->emit_tag = tag;
  g->emit_count = 0;
  Upload(arm, at, blob.data(), uint32_t(blob.size()));
  g->emit_tag = 0;
  core->SetItems(arm, tag, g->emit_count);
  return f;
}

// A kept object onto one arm: ASSET_LOAD if the arm holds it (its ID through
// the library's text area), else WRITE, and with a store a background put.
void Place(int arm, const Kept &k) {
  int have = g->core->HasAsset(arm, k.id);
  if (have > 0) {
    Submit(arm, WriteCmd(kText, k.id.b, 16), "WRITE");
    HostOp(arm, OP_ASSET_LOAD, {kText, k.address, 0, uint32_t(k.blob.size())}, "ASSET_LOAD");
  } else if (have == 0) {
    StagedPut(arm, k.address, k.blob, k.type, false);
  } else {
    Upload(arm, k.address, k.blob.data(), uint32_t(k.blob.size()));
  }
}

// Restore: the allocation at `at` holds a kept object, unchanged since it was
// placed. False: it does not (the caller uploads it).
bool PlaceKept(int arm, Address at, uint32_t n) {
  const uint8_t *mem = ArmBytes(arm);
  for (const Kept *k : g->kept) {
    size_t size = k->blob.size();
    if (k->address != at || !size || size > n || std::memcmp(mem + at, k->blob.data(), size) != 0) continue;
    Place(arm, *k);
    if (n > size) Upload(arm, at + uint32_t(size), mem + at + size, n - uint32_t(size));
    return true;
  }
  return false;
}

void Unkeep(const Kept *k) {
  if (k) g->kept.erase(std::remove(g->kept.begin(), g->kept.end(), k), g->kept.end());
}

// A bitmap's asset (type 2): "SKB1", u16 width, height, stride, 0, then the
// 4-bpp rows of its contents, left pixel in the high nibble.
std::vector<uint8_t> BitmapBlob(const gray::BitmapView &c, uint32_t stride) {
  std::vector<uint8_t> b = {'S', 'K', 'B', '1'};
  Put16(b, uint32_t(c.width));
  Put16(b, uint32_t(c.height));
  Put16(b, stride);
  Put16(b, 0);
  b.resize(12 + size_t(stride) * size_t(c.height), 0);
  for (int y = 0; y < c.height; y++)
    for (int x = 0; x < c.width; x++)
      b[12 + size_t(y) * stride + size_t(x) / 2] |= uint8_t(LevelOf(c.row(y)[x]) << (x % 2 == 0 ? 4 : 0));
  return b;
}

class G1Device : public device::Device {
public:
  static G1Bitmap *Data(const gray::Bitmap &b) {
    return static_cast<G1Bitmap *>(device::DeviceData(const_cast<gray::Bitmap &>(b)));
  }

  static bool Has(int arm) { return g->arms[arm].present; }

  bool AddBitmap(gray::Bitmap &bitmap, std::string *error) override {
    int w = bitmap.contents.width, h = bitmap.contents.height;
    uint32_t stride = uint32_t(w + 1) / 2;
    uint64_t bytes = uint64_t(stride) * uint64_t(h);
    uint32_t header = device::KeptAs(bitmap) == Keep::kStored ? 12 : 0;
    if (bytes + header > kArenaBytes) {
      if (error) *error = "bitmap too large";
      return false;
    }
    int at = g->alloc.Alloc(uint32_t((bytes + header + 3) / 4));
    if (at < 0) {
      if (error) *error = g->keep ? "the arms' sessions are kept: no memory is the library's" : "out of device memory";
      return false;
    }
    auto gb = std::make_unique<G1Bitmap>();
    gb->address = 4 * uint32_t(at) + header;
    gb->stride = stride;
    gb->header = header;
    if (header) {
      gb->kept = std::make_unique<Kept>();
      gb->kept->address = 4 * uint32_t(at);
      gb->kept->type = ASSET_BITMAP;
      g->kept.push_back(gb->kept.get());
    }
    for (int i = 0; i < 2; i++) {
      if (!Has(i)) continue;
      const uint8_t *m = ArmBytes(i) + gb->address;
      if (std::all_of(m, m + bytes, [](uint8_t b) { return b == 0; })) continue;
      HostOp(i, OP_FILL, {gb->address, stride, 0, 0, uint32_t(w), uint32_t(h), 0}, "FILL");
    }
    device::DeviceData(bitmap) = gb.release();
    return true;
  }

  void RemoveBitmap(gray::Bitmap &bitmap) override {
    G1Bitmap *gb = Data(bitmap);
    if (!gb) return;
    Release(gb->address - gb->header);
    Unkeep(gb->kept.get());
    delete gb;
    device::DeviceData(bitmap) = nullptr;
  }

  // Frees arena memory; what programs changed in it is known again.
  static void Release(Address address) {
    auto used = g->alloc.Used().find(address / 4);
    if (used == g->alloc.Used().end()) return;
    uint64_t end = uint64_t(address) + 4ull * used->second;
    for (auto &a : g->arms) {
      auto &c = a.mirror.changes;
      c.erase(std::remove_if(c.begin(), c.end(),
                             [&](const std::pair<Address, uint32_t> &r) {
                               return r.first >= address && uint64_t(r.first) + r.second <= end;
                             }),
              c.end());
    }
    g->alloc.Free(address / 4);
  }

  bool AddFont(Font &font, std::span<const uint8_t> blob, std::string *error) override {
    int at = g->alloc.Alloc(uint32_t((blob.size() + 3) / 4));
    if (at < 0) {
      if (error) *error = g->keep ? "the arms' sessions are kept: no memory is the library's" : "out of device memory";
      return false;
    }
    auto gf = std::make_unique<G1Font>();
    gf->address = 4 * uint32_t(at);
    if (device::KeptAs(font) == Keep::kStored) {
      gf->kept = std::make_unique<Kept>();
      Kept &k = *gf->kept;
      k.address = gf->address;
      k.blob.assign(blob.begin(), blob.end());
      k.id = AssetIdOf(blob);
      k.type = ASSET_FONT;
      g->kept.push_back(&k);
      for (int i = 0; i < 2; i++)
        if (Has(i)) Place(i, k);
    } else {
      for (int i = 0; i < 2; i++)
        if (Has(i)) Upload(i, gf->address, blob.data(), uint32_t(blob.size()));
    }
    device::DeviceData(font) = gf.release();
    return true;
  }

  void RemoveFont(Font &font) override {
    auto *gf = static_cast<G1Font *>(device::DeviceData(font));
    if (!gf) return;
    Release(gf->address);
    Unkeep(gf->kept.get());
    delete gf;
    device::DeviceData(font) = nullptr;
  }

  void Fill(gray::Bitmap &bitmap, int x, int y, int w, int h, int level) override {
    G1Bitmap *gb = Data(bitmap);
    for (int i = 0; i < 2; i++)
      if (Has(i))
        HostOp(i, OP_FILL,
               {gb->address, gb->stride, uint32_t(x), uint32_t(y), uint32_t(w), uint32_t(h), uint32_t(level)},
               "FILL");
  }

  // GLYPHS over the glyph text, written to the library's text area first.
  void GlyphRun(Font &font, gray::Bitmap &bitmap, int baseline_y,
                std::span<const device::GlyphPosition> glyphs, int level) override {
    G1Bitmap *gb = Data(bitmap);
    auto *gf = static_cast<G1Font *>(device::DeviceData(font));
    const size_t max_n = size_t(std::min<int>(int(kTextBytes), g->cmd_limit - 5));
    for (int i = 0; i < 2; i++) {
      if (!Has(i)) continue;
      size_t k = 0;
      while (k < glyphs.size()) {
        uint8_t v[256];
        size_t n = 0, first = k;
        while (k < glyphs.size()) {
          uint8_t one[3];
          int len = PutVarint(glyphs[k].index, one);
          if (n + size_t(len) > max_n) break;
          std::memcpy(v + n, one, size_t(len));
          n += size_t(len);
          k++;
        }
        Cmd glyph = Instruction(i, OP_GLYPHS,
                                {gf->address, gb->address, gb->stride, uint32_t(glyphs[first].pen_x),
                                 uint32_t(baseline_y), MODE_MASK, uint32_t(level), kText, uint32_t(n)});
        Submit(i, WriteCmd(kText, v, n), "WRITE");
        Submit(i, glyph, "GLYPHS");
      }
    }
  }

  // Uploads the rows of the bitmap whose pixels differ from the arm's mirror.
  void Sync(gray::Bitmap &bitmap) override {
    G1Bitmap *gb = Data(bitmap);
    const auto &c = bitmap.contents;
    int w = c.width, h = c.height;
    if (gb->kept && gb->kept->blob.empty()) {
      Kept &k = *gb->kept;
      k.blob = BitmapBlob(c, gb->stride);
      k.id = AssetIdOf(k.blob);
      for (int i = 0; i < 2; i++)
        if (Has(i)) Place(i, k);
    }
    for (int i = 0; i < 2; i++) {
      if (!Has(i)) continue;
      const uint8_t *mem = ArmBytes(i) + gb->address;
      std::vector<int> x0(size_t(h), -1), x1(size_t(h), -1);
      bool any = false;
      for (int y = 0; y < h; y++) {
        const uint8_t *row = c.row(y);
        const uint8_t *m = mem + uint32_t(y) * gb->stride;
        for (int x = 0; x < w; x++) {
          int have = x % 2 == 0 ? m[x / 2] >> 4 : m[x / 2] & 15;
          if (have == LevelOf(row[x])) continue;
          if (x0[size_t(y)] < 0) x0[size_t(y)] = x;
          x1[size_t(y)] = x + 1;
          any = true;
        }
      }
      if (!any) continue;
      for (int y = 0; y < h;) {
        if (x0[size_t(y)] < 0) {
          y++;
          continue;
        }
        int end = y + 1;
        while (end < h && (x0[size_t(end)] >= 0 || (end + 1 < h && x0[size_t(end + 1)] >= 0))) end++;
        while (x0[size_t(end - 1)] < 0) end--;
        SyncBand(i, bitmap, gb->address, gb->stride, y, end, x0, x1);
        y = end;
      }
    }
  }

  // Bytes [s, e) as the arm should hold them: the mirror with the bitmap's
  // rows y0..y1-1 from its contents.
  static std::vector<uint8_t> Target(int arm, const gray::Bitmap &bitmap, Address base,
                                     uint32_t stride, Address s, Address e, int y0, int y1) {
    std::vector<uint8_t> t(ArmBytes(arm) + s, ArmBytes(arm) + e);
    const auto &c = bitmap.contents;
    for (int y = y0; y < y1; y++) {
      const uint8_t *row = c.row(y);
      for (int x = 0; x < c.width; x++) {
        Address at = base + uint32_t(y) * stride + uint32_t(x) / 2;
        if (at < s || at >= e) continue;
        uint8_t &b = t[at - s];
        int v = LevelOf(row[x]);
        b = x % 2 == 0 ? uint8_t((b & 0x0F) | v << 4) : uint8_t((b & 0xF0) | v);
      }
    }
    return t;
  }

  // Whole rows as one range, or one segment per row: whichever is smaller.
  void SyncBand(int arm, const gray::Bitmap &bitmap, Address base, uint32_t stride, int y0, int y1,
                const std::vector<int> &x0, const std::vector<int> &x1) {
    Address s = base + uint32_t(y0) * stride;
    Address e = base + uint32_t(y1) * stride;
    std::vector<uint8_t> whole = Target(arm, bitmap, base, stride, s, e, y0, y1);
    size_t cost_whole = EncodedSize(s, whole.data(), uint32_t(whole.size()));
    struct Segment {
      Address s;
      std::vector<uint8_t> bytes;
    };
    std::vector<Segment> segments;
    size_t cost_rows = 0;
    for (int y = y0; y < y1 && cost_rows < cost_whole; y++) {
      if (x0[size_t(y)] < 0) continue;
      Address row = base + uint32_t(y) * stride;
      Address rs = row + uint32_t(x0[size_t(y)]) / 2;
      Address re = row + (uint32_t(x1[size_t(y)]) + 1) / 2;
      Segment seg{rs, Target(arm, bitmap, base, stride, rs, re, y, y + 1)};
      cost_rows += EncodedSize(rs, seg.bytes.data(), uint32_t(seg.bytes.size()));
      segments.push_back(std::move(seg));
    }
    if (cost_rows < cost_whole) {
      for (auto &seg : segments) Upload(arm, seg.s, seg.bytes.data(), uint32_t(seg.bytes.size()));
    } else {
      Upload(arm, s, whole.data(), uint32_t(whole.size()));
    }
  }

  uint64_t Present(std::span<const device::Piece> pieces) override {
    auto job = std::make_shared<PresentJob>();
    job->id = ++g->core->present_counter;
    for (int i = 0; i < 2; i++) {
      if (!Has(i)) continue;
      int ox = g->geo.ox[i], oy = g->geo.oy[i];
      g->arms[i].pool.pinning = true;
      bool was_off = !g->arms[i].mirror.panel_on && !g->arms[i].mirror.standby;
      for (const auto &p : pieces) {
        int x0 = std::max(p.x, ox), x1 = std::min(p.x + p.w, ox + kPanelW);
        if (x0 >= x1) continue;
        int dst_x = x0 - ox, dst_y = p.y + oy;
        if (!p.source) {
          BlackDraws(i, job->draws[i], dst_x, dst_y, x1 - x0, p.h);
          continue;
        }
        G1Bitmap *gb = Data(*p.source);
        PresentDraw(i, job->draws[i], gb->address, gb->stride, p.source_x + (x0 - p.x), p.source_y,
                    dst_x, dst_y, x1 - x0, p.h);
      }
      Mirror &m = g->arms[i].mirror;
      std::atomic<int64_t> &black_off = g->arms[i].black_off;
      if (!job->draws[i].empty() && m.panel_on &&
          std::all_of(m.panel.begin(), m.panel.end(), [](uint8_t v) { return v == 0; })) {
        // all black: standby, not a lit black panel, then off (Core::BlackOff); an eye that was off stays off
        Cmd c = Instruction(i, OP_SYS_WRITE, {VM_P_PANEL, was_off ? 0u : 2u});
        job->draws[i].clear();
        if (c.size && m.Run(c.b.data(), c.size) == 0) job->draws[i].push_back(c);
        if (m.standby && !black_off && SCOUTER_G1_BLACK_OFF_MS > 0) // 0: standby until lit again
          black_off = NowUs() + int64_t(SCOUTER_G1_BLACK_OFF_MS) * 1000;
      } else if (!job->draws[i].empty()) {
        black_off = 0;
      }
      g->arms[i].pool.Unpin();
      if (!job->draws[i].empty()) std::memset(ArmBytes(i) + kTimeAddr, 0, 4); // as its tail leaves it
    }
    for (int i = 0; i < 2; i++) {
      if (!Has(i)) continue;
      Item item;
      item.kind = Item::PRESENT;
      item.present = job;
      g->core->links[i].emitted++;
      g->core->queues[i].enqueue(std::move(item));
    }
    g->core->transport->Wake();
    return job->id;
  }

  void WaitPresented(uint64_t id) override {
    Core *core = g ? g->core.get() : nullptr;
    if (!core) return;
    std::unique_lock<std::mutex> lock(core->mutex);
    core->cv.wait_for(lock, std::chrono::seconds(60),
                      [&] { return core->Presented() >= id || !core->AnyUp() || core->stop; });
  }

  int TotalMemory() override { return int(g->alloc.TotalWords() * 4); }
  int FreeMemory() override { return int(g->alloc.FreeWords() * 4); }
};

void Emit(int arm, const Cmd &cmd) {
  Core *core = g->core.get();
  Item item;
  item.cmd = cmd;
  item.tag = g->emit_tag;
  if (item.tag) g->emit_count++;
  if (core->restore_sink[arm]) {
    core->restore_sink[arm]->push_back(std::move(item));
    return;
  }
  core->links[arm].emitted++;
  core->queues[arm].enqueue(std::move(item));
  core->transport->Wake();
}

// A new session. fresh: the library's first with this arm, whose panel is
// switched off (the library's display starts black). Else the arm lost its
// memory: the allocated memory again from the mirror, then the panel as the
// mirror shows it, band by band through the staging rows.
void Core::Restore(int arm, bool fresh) {
  std::lock_guard<std::mutex> api(device::Lock());
  Drain();
  Link &l = links[arm];
  uint64_t done = l.done_present;
  for (auto &it : l.pending) {
    if (it.kind == Item::PRESENT) {
      done = std::max(done, it.present->id);
      it.present->dropped[arm] = true;
    }
    done = std::max(done, it.present_id);
  }
  l.pending.clear();
  l.done_present = done;
  DropJobs(arm);
  std::deque<Item> out;
  restore_sink[arm] = &out;
  ArmImpl &a = g->arms[arm];
  Mirror &m = a.mirror;
  uint8_t *mem = ArmBytes(arm);
  std::vector<uint8_t> kept(kArenaBytes, 0);
  for (auto &u : g->alloc.Used())
    std::memcpy(kept.data() + 4 * u.first, mem + 4 * u.first, 4 * size_t(u.second));
  std::memcpy(mem, kept.data(), kArenaBytes);
  a.pool.Clear();
  for (auto &u : g->alloc.Used())
    if (!PlaceKept(arm, 4 * u.first, 4 * u.second)) Upload(arm, 4 * u.first, mem + 4 * u.first, 4 * u.second);
  m.triggers = l.triggers;
  PanelFromMirror(arm, fresh);
  restore_sink[arm] = nullptr;
  l.pending = std::move(out);
  Log("%s arm: %s with %zu commands", kArmName[arm], fresh ? "new session" : "restoring", l.pending.size());
  Notify();
}

// The panel as the mirror shows it, band by band through the staging rows
// (fresh: off, the library's display starts black).
void Core::PanelFromMirror(int arm, bool fresh) {
  Mirror &m = g->arms[arm].mirror;
  std::vector<uint8_t> panel = m.panel;
  bool on = m.panel_on && !fresh;
  HostOp(arm, OP_SYS_WRITE, {VM_P_PANEL, m.standby && !fresh ? 2u : 0u}, "SYS_WRITE");
  bool drawn = false;
  for (int y = 0; on && y < kPanelH; y += kStagingRows) {
    int n = std::min(kStagingRows, kPanelH - y);
    bool lit = std::any_of(panel.begin() + long(y) * kPanelW, panel.begin() + long(y + n) * kPanelW,
                           [](uint8_t v) { return v != 0; });
    if (!lit) continue;
    std::vector<uint8_t> rows(size_t(n) * kRowBytes, 0);
    for (int r = 0; r < n; r++)
      for (int x = 0; x < kPanelW; x++)
        rows[size_t(r) * kRowBytes + size_t(x) / 2] |=
            uint8_t(panel[size_t(y + r) * kPanelW + size_t(x)] << (x % 2 == 0 ? 4 : 0));
    Upload(arm, kStaging, rows.data(), uint32_t(rows.size()));
    HostOp(arm, OP_DRAW, {kStaging, kRowBytes, 0, 0, 0, uint32_t(y), kPanelW, uint32_t(n)}, "DRAW");
    drawn = true;
  }
  if (on && !drawn) HostOp(arm, OP_SYS_WRITE, {VM_P_PANEL, 1}, "SYS_WRITE");
  if (drawn) {
    std::vector<uint8_t> zero(kStagingBytes, 0);
    Upload(arm, kStaging, zero.data(), uint32_t(zero.size()));
  }
}

// The panel again from the mirror, after what is queued: at NOTICE 6 0 (the
// arm left the case hold, VM.md §6.6, during which it drew nothing) and when a
// kept session finds the panel off (if_shown: only if the mirror shows it).
void Core::RedrawPanel(int arm, bool if_shown) {
  std::lock_guard<std::mutex> api(device::Lock());
  const Mirror &m = g->arms[arm].mirror;
  if (if_shown && !m.panel_on && !m.standby) return;
  PanelFromMirror(arm, false);
  Log("%s arm: its panel drawn again from the mirror", kArmName[arm]);
}

// The other eye's panel guard cap as this eye's BRIGHTNESS_MAX, so both show the lower level; BRIGHTNESS
// itself stays the application's. Only to an arm with the guard, once per change.
void Core::MatchBrightness(int arm) {
  if (!SCOUTER_G1_MATCH_BRIGHTNESS) return;
  std::lock_guard<std::mutex> api(device::Lock());
  if (ending) return;
  Link &l = links[arm];
  uint8_t want = links[1 - arm].guard ? links[1 - arm].cap : 42;
  if (!l.guard || !l.ready || want == l.max_sent) return;
  if (HostOp(arm, OP_SYS_WRITE, {VM_P_BRIGHTNESS_MAX, want}, "SYS_WRITE")) {
    l.max_sent = want;
    Log("%s arm: BRIGHTNESS_MAX %u, the %s arm's cap", kArmName[arm], want, kArmName[1 - arm]);
  }
}

// At Deinit: an eye this session gave a lower BRIGHTNESS_MAX gets 42 again, so it does not stay dimmed for the
// programs or hosts after this one. Device lock held.
void Core::EndMatch(int arm) {
  Link &l = links[arm];
  if (l.guard && l.ready && l.max_sent != 42 && HostOp(arm, OP_SYS_WRITE, {VM_P_BRIGHTNESS_MAX, 42}, "SYS_WRITE"))
    l.max_sent = 42;
}

// An eye all black in standby for SCOUTER_G1_BLACK_OFF_MS: off, about 1 mA
// less (a DRAW then wakes it in about 71 ms instead of 4 ms, VM.md §6.6). The
// arm writes PANEL & 1, so a panel that a program or host code lit meanwhile
// stays on; [T] is the scratch word, 0 again after.
void Core::BlackOff(int arm) {
  std::lock_guard<std::mutex> api(device::Lock());
  std::atomic<int64_t> &at = g->arms[arm].black_off;
  if (!at || NowUs() < at) return;
  at = 0;
  if (!g->arms[arm].mirror.standby || SafeMode(arm)) return;
  uint32_t values[2] = {VM_P_PANEL, 1};
  ArenaWord c[2];
  if (!Words(arm, values, 2, c)) return;
  std::vector<uint8_t> v, zero(4, 0);
  ArenaWord read[2] = {kTimeWord, c[0]}, mask[3] = {kTimeWord, kTimeWord, c[1]}, write[2] = {c[0], kTimeWord};
  PutIns(v, OP_SYS_READ, read, 2, 0);
  PutIns(v, OP_AND, mask, 3, 0);
  PutIns(v, OP_SYS_WRITE, write, 2, 0);
  PutWrite(v, kTimeAddr, zero.data(), 4);
  if (Submit(arm, ToCmd(v), "SYS_WRITE"))
    Log("%s arm: all black for %d s: panel off", kArmName[arm], SCOUTER_G1_BLACK_OFF_MS / 1000);
}

// ---------------------------------------------------------------- emulated arms

#ifdef SCOUTER_G1_EMULATOR
// Two arms behind one emulated adapter: the window, ACKs and control requests
// of VM.md §6, and the arms' own interpreter (src/vm) for host code and
// programs.
class EmulatorTransport : public Transport {
public:
  struct Msg {
    int64_t at;
    int channel;
    std::vector<uint8_t> data;
  };

  struct Arm {
    EmulatorTransport *owner = nullptr;
    int index = 0;
    std::vector<uint8_t> arena = std::vector<uint8_t>(kArenaBytes, 0);
    std::vector<uint8_t> panel = std::vector<uint8_t>(size_t(kPanelW) * kPanelH, 0);
    struct vm vm {};
    vm_host host{};
    uint32_t epoch = 1;
    int64_t boot = 0;
    double ppm = 0;
    uint32_t clock_offset = 0;
    uint16_t rx_next = 0, exec_next = 0;
    std::vector<uint8_t> slot[32];
    bool held[32] = {};
    int64_t ack_at = -1, vm_at = 0, repeat_at = -1;
    bool link = false;
    bool report_down = false;
    int64_t up_at = -1;
    bool panel_on = false, standby = false;
    int panel_held = -1; // PANEL = 0 or 2 written while the microphone holds the panel
    bool guard = true; // PANEL_GUARD; tests set BRIGHTNESS_CAP to act as the guard
    uint32_t brightness = 21;
    int interval = 12, latency = 0, timeout = 400, phy = 1;
    uint32_t boot_count = 1;
    uint32_t sensor[64] = {};
    uint32_t pct = VM_UNKNOWN;
    int64_t pct_since = -1;
    uint16_t events_mask = 0, input_count = 0;
    uint32_t input_pct = VM_UNKNOWN, input_charge = VM_UNKNOWN;
    std::deque<Msg> to_arm, to_host;
    bool mic = false;
    int mic_bytes = 40;
    uint16_t mic_counter = 0;
    int64_t mic_start = 0, mic_at = -1, mic_lease = -1;
    uint64_t mic_events = 0, mic_sample = 0;
    double mic_hz = 440, mic_peak = 0.25;
    std::vector<uint64_t> mic_memory;
    void *mic_encoder = nullptr;
    // the asset store (VM.md §6.10), kept across reboots; one job at a time
    struct Asset {
      AssetId id;
      uint32_t offset;
      uint8_t type;
      std::vector<uint8_t> body;
    };
    std::vector<Asset> assets;
    bool store = true;
    uint32_t asset_seq = 0;
    uint8_t asset_state = 0, asset_error = 0;
    int64_t asset_at = -1; // the running job ends
    uint8_t job = 0, job_type = 0;
    AssetId job_id{};
    uint32_t job_addr = 0, job_size = 0, job_offset = 0;
  };
  static constexpr uint32_t kStoreBytes = 0xEC0000; // the arm's g1_assets partition
  static constexpr uint32_t kSector = 4096;

  static constexpr uint16_t kTriggers = 0xF063; // as g1v0 0.5.0, plus LIGHT on the right arm (Triggers)
  static uint16_t Triggers(const Arm &a) { return uint16_t(kTriggers | (a.index == 1 ? 1u << VM_T_LIGHT : 0)); }
  static constexpr int kMicPerEvent = 5;
  static constexpr int64_t kMicLeaseUs = 2000000;
#ifdef SCOUTER_G1_MIC
  static bool HasMic(const Arm &a) { return a.index == 1; } // as g1v0 0.5.8+: the right arm
#else
  static bool HasMic(const Arm &) { return false; }
#endif

  explicit EmulatorTransport(const std::string &adapter) : adapter_(adapter) {}

  bool Start(Events *events, const bool present[2]) override {
    events_ = events;
    // The emulated arms are behind one emulated adapter, hci0.
    std::string problem;
    if (PickAdapter({{"/org/bluez/hci0", "00:00:00:00:00:00", true}}, adapter_, &problem) < 0) {
      events_->Problem(ERROR_BLUETOOTH_ERROR, problem);
      return false;
    }
    int64_t now = NowUs();
    for (int i = 0; i < 2; i++) {
      Arm &a = arms_[i];
      a.owner = this;
      a.index = i;
      a.boot = now - 5000000;
      a.ppm = i == 0 ? 40 : -25;
      a.clock_offset = i == 0 ? 123456789u : 3456789012u;
      a.epoch = NewEpoch();
      for (uint32_t &v : a.sensor) v = VM_UNKNOWN;
      a.sensor[VM_P_BATT_MV] = 3950;
      a.sensor[VM_P_BATT_PCT] = 72;
      a.sensor[VM_P_CHARGE] = 0;
      a.sensor[VM_P_IN_CASE] = 0;
      a.sensor[VM_P_BATT_MA] = 6;
      a.sensor[VM_P_BATT_TEMP] = 265;
      a.sensor[VM_P_AUTO_BRIGHTNESS] = i == 1 ? 0 : VM_UNKNOWN;
      a.sensor[VM_P_BRIGHTNESS_MAX] = 42;
      a.sensor[VM_P_BRIGHTNESS_CAP] = 42;
      Boot(a);
      present_[i] = present[i];
      if (present[i]) {
        a.up_at = now + 20000;
        events_->Found(i, i == 0 ? "emulated left arm" : "emulated right arm");
      }
    }
    return true;
  }

  bool Write(int arm, int channel, const uint8_t *data, size_t n) override {
    std::lock_guard<std::mutex> lock(mutex_);
    Arm &a = arms_[arm];
    if (!a.link) return false;
    if (Lose(write_loss_)) return true;
    int64_t at = NowUs() + 3000 + int64_t(rng_() % 1500);
    if (!a.to_arm.empty()) at = std::max(at, a.to_arm.back().at);
    a.to_arm.push_back({at, channel, std::vector<uint8_t>(data, data + n)});
    return true;
  }

  void Wake() override {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      woken_ = true;
    }
    cv_.notify_all();
  }

  void Disconnect(int arm) override { DropLink(arm, 300); }
  void Hold(int arm, int ms) override { DropLink(arm, ms); }

  void Stop() override {}

  void Poll(int timeout_ms) override {
    std::vector<std::pair<int, Msg>> deliver;
    std::vector<int> ups, downs;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      int64_t now = NowUs();
      int64_t until = std::min(now + int64_t(timeout_ms) * 1000, NextEvent());
      if (until > now && !woken_)
        cv_.wait_for(lock, std::chrono::microseconds(until - now), [&] { return woken_; });
      woken_ = false;
      now = NowUs();
      for (int i = 0; i < 2; i++) {
        Arm &a = arms_[i];
        if (!present_[i]) continue;
        if (a.report_down) {
          a.report_down = false;
          downs.push_back(i);
        }
        if (a.up_at >= 0 && now >= a.up_at) {
          a.up_at = -1;
          a.link = true;
          ups.push_back(i);
        }
        while (!a.to_arm.empty() && a.to_arm.front().at <= now) {
          Msg m = std::move(a.to_arm.front());
          a.to_arm.pop_front();
          if (m.channel == CH_CMD)
            Receive(a, m.data, now);
          else
            Control(a, m.data, now);
        }
        Execute(a, now);
        if (a.mic && now >= a.mic_lease) {
          vm_set_mic(&a.vm, 0);
          MicStopped(a, 3);
        }
        while (a.mic && now >= a.mic_at) MicEvent(a);
        if (a.ack_at >= 0 && now >= a.ack_at) SendAck(a, now);
        if (a.repeat_at >= 0 && now >= a.repeat_at) SendAck(a, now, true);
        if (a.asset_at >= 0 && now >= a.asset_at) AssetJobDone(a, now);
        while (!a.to_host.empty() && a.to_host.front().at <= now) {
          deliver.push_back({i, std::move(a.to_host.front())});
          a.to_host.pop_front();
        }
      }
    }
    for (int i : downs) events_->LinkDown(i);
    for (int i : ups) events_->LinkUp(i, 247);
    for (auto &d : deliver) events_->Data(d.first, d.second.data.data(), d.second.data.size());
  }

  const uint8_t *Panel(int arm) {
    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_[arm] = arms_[arm].panel;
    return snapshot_[arm].data();
  }

  void SetLoss(double write_loss, double notify_loss) {
    std::lock_guard<std::mutex> lock(mutex_);
    write_loss_ = write_loss;
    notify_loss_ = notify_loss;
  }

  void SetProperty(int arm, uint32_t id, uint32_t value) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      Arm &a = arms_[arm];
      if (id == VM_P_BRIGHTNESS_CAP && a.guard && a.sensor[id] != value && a.link) { // as the guard: NOTICE 7
        int64_t now = NowUs();
        std::vector<uint8_t> m = Header(a, EVT_NOTICE, now);
        m.push_back(7);
        m.push_back(uint8_t(value));
        m.push_back(uint8_t(std::min({a.brightness, a.sensor[VM_P_BRIGHTNESS_MAX], value})));
        Notify(a, std::move(m), now);
      }
      if (id < 64) a.sensor[id] = value;
    }
    Wake();
  }

  void SetMicSignal(int arm, double hz, double peak) {
    std::lock_guard<std::mutex> lock(mutex_);
    arms_[arm].mic_hz = hz;
    arms_[arm].mic_peak = std::clamp(peak, 0.0, 1.0);
  }

  void SetAssetStore(int arm, bool available) {
    std::lock_guard<std::mutex> lock(mutex_);
    arms_[arm].store = available;
  }

  void DropLink(int arm, int ms) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      Arm &a = arms_[arm];
      if (!present_[arm]) return;
      if (a.link) a.report_down = true;
      a.link = false;
      a.to_arm.clear();
      a.to_host.clear();
      a.ack_at = -1;
      a.repeat_at = -1;
      a.events_mask = 0;
      MicOff(a);
      a.up_at = NowUs() + int64_t(ms) * 1000;
    }
    Wake();
  }

  void Reboot(int arm) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      Arm &a = arms_[arm];
      if (!present_[arm]) return;
      if (a.link) a.report_down = true;
      a.link = false;
      a.to_arm.clear();
      a.to_host.clear();
      a.epoch = NewEpoch();
      a.boot = NowUs();
      a.boot_count++;
      a.mic = false;
      a.mic_at = -1;
      if (a.asset_at >= 0) a.asset_state = 0; // a job never survives a reboot
      a.asset_at = -1;
      Boot(a);
      a.up_at = NowUs() + 300000;
    }
    Wake();
  }

private:
  static Arm *A(void *c) { return static_cast<Arm *>(c); }

  // Power-on state: arena zero, panel off, program stopped, empty window.
  void Boot(Arm &a) {
    a.host = {&a,
              [](void *c) { return A(c)->owner->DeviceNow(*A(c), NowUs()); },
              [](void *c, uint32_t *len) -> const uint8_t * {
                Arm *a = A(c);
                if (!a->held[a->exec_next % 32]) return nullptr;
                *len = uint32_t(a->slot[a->exec_next % 32].size());
                return a->slot[a->exec_next % 32].data();
              },
              [](void *c) {
                Arm *a = A(c);
                a->held[a->exec_next % 32] = false;
                a->exec_next++;
                if (a->ack_at < 0) a->ack_at = NowUs() + 2000;
              },
              [](void *) { return uint32_t(0); },
              [](void *c, uint32_t id) { return A(c)->owner->Prop(*A(c), id); },
              [](void *c, uint8_t type, const uint8_t *body, uint32_t len) {
                Arm *a = A(c);
                if (!a->link) return 1;
                std::vector<uint8_t> m = a->owner->Header(*a, type, NowUs());
                m.insert(m.end(), body, body + len);
                a->owner->Notify(*a, std::move(m), NowUs());
                return 0;
              },
              [](void *c, int32_t, int32_t, int32_t, int32_t) {
                Arm *a = A(c);
                if (!a->panel_on) std::fill(a->panel.begin(), a->panel.end(), 0);
                a->panel_on = true;
                a->standby = false;
                a->panel_held = -1;
                return 0;
              },
              [](void *c, int32_t x, int32_t y, int32_t w, const uint8_t *px) {
                uint8_t *p = A(c)->panel.data() + size_t(y) * kPanelW + size_t(x);
                for (int32_t i = 0; i < w; i++) p[i] = (i & 1) ? (px[i / 2] & 15) : (px[i / 2] >> 4);
              },
              [](void *) {},
              [](void *c, uint32_t v) { // VM.md §6.5: while the microphone is on, 0 and 2 blank to the indicator
                Arm *a = A(c);
                if (v != 1 && a->mic && a->panel_on) {
                  std::fill(a->panel.begin(), a->panel.end(), 0);
                  Indicator(*a);
                  a->panel_held = int(v);
                  return;
                }
                if (v != 1 || !a->panel_on) std::fill(a->panel.begin(), a->panel.end(), 0);
                a->panel_on = v == 1;
                a->standby = v == 2;
                a->panel_held = -1;
              },
              [](void *c, uint32_t level) { A(c)->brightness = level; },
              [](void *, uint32_t) {},
              [](void *c, uint32_t id, uint32_t value) { // BRIGHTNESS_MAX, the right arm's AUTO_BRIGHTNESS
                Arm *a = A(c);
                if (id == VM_P_BRIGHTNESS_MAX) {
                  a->sensor[id] = value;
                  return 0;
                }
                if (id != VM_P_AUTO_BRIGHTNESS || !(Triggers(*a) & (1u << VM_T_LIGHT))) return int(VM_E_ARG);
                a->sensor[VM_P_AUTO_BRIGHTNESS] = value;
                return 0;
              }};
    a.host.asset_read = [](void *c, const uint8_t *id, uint32_t off, uint8_t *dst, uint32_t n) {
      Arm *a = A(c);
      for (auto &x : a->assets) {
        if (!a->store || std::memcmp(x.id.b, id, 16) != 0) continue;
        if (uint64_t(off) + n > x.body.size()) return int(VM_E_ARENA);
        std::memcpy(dst, x.body.data() + off, n);
        return 0;
      }
      return int(VM_E_ASSET);
    };
    vm_init(&a.vm, a.arena.data(), &a.host, 0);
    std::fill(a.panel.begin(), a.panel.end(), 0);
    a.panel_on = a.standby = false;
    a.panel_held = -1;
    a.brightness = 21;
    a.rx_next = a.exec_next = 0;
    for (bool &h : a.held) h = false;
    a.ack_at = -1;
    a.repeat_at = -1;
    a.vm_at = 0;
    a.events_mask = 0;
    a.pct = VM_UNKNOWN;
    a.pct_since = -1;
  }

  uint32_t Prop(const Arm &a, uint32_t id) {
    switch (id) {
    case VM_P_ARM: return a.index == 0 ? 1 : 0;
    case VM_P_LINK: return a.link ? 1 : 0;
    case VM_P_LINK_INTERVAL: return a.link ? uint32_t(a.interval) * 1250 : 0;
    case VM_P_BATT_PCT: return a.pct;
    case VM_P_BRIGHTNESS: return a.brightness;
    case VM_P_PANEL:
      return a.panel_held >= 0 ? uint32_t(a.panel_held) : a.panel_on ? 1 : a.standby ? 2 : 0;
    default: return id < 64 ? a.sensor[id] : VM_UNKNOWN;
    }
  }

  uint32_t NewEpoch() {
    uint32_t e = uint32_t(rng_());
    return e ? e : 1;
  }

  uint32_t DeviceNow(const Arm &a, int64_t now) const {
    double t = double(now - a.boot) * (1.0 + a.ppm * 1e-6);
    return uint32_t(int64_t(t)) + a.clock_offset;
  }

  bool Lose(double p) { return p > 0 && std::uniform_real_distribution<double>(0, 1)(rng_) < p; }

  int64_t NextEvent() {
    int64_t next = INT64_MAX;
    for (int i = 0; i < 2; i++) {
      Arm &a = arms_[i];
      if (!present_[i]) continue;
      if (!a.to_arm.empty()) next = std::min(next, a.to_arm.front().at);
      if (!a.to_host.empty()) next = std::min(next, a.to_host.front().at);
      if (a.ack_at >= 0) next = std::min(next, a.ack_at);
      if (a.repeat_at >= 0) next = std::min(next, a.repeat_at);
      if (a.vm_at >= 0) next = std::min(next, a.vm_at);
      if (a.pct_since >= 0) next = std::min(next, a.pct_since + 30000000);
      if (a.up_at >= 0) next = std::min(next, a.up_at);
      if (a.mic) next = std::min(next, std::min(a.mic_at, a.mic_lease));
      if (a.asset_at >= 0) next = std::min(next, a.asset_at);
      if (a.report_down) next = 0;
    }
    return next;
  }

  void Notify(Arm &a, std::vector<uint8_t> msg, int64_t at) {
    if (!a.link || Lose(notify_loss_)) return;
    at += 3000 + int64_t(rng_() % 1500);
    if (!a.to_host.empty()) at = std::max(at, a.to_host.back().at);
    a.to_host.push_back({at, 0, std::move(msg)});
  }

  std::vector<uint8_t> Header(Arm &a, uint8_t type, int64_t now) {
    std::vector<uint8_t> m = {type};
    Put32(m, a.epoch);
    Put32(m, DeviceNow(a, now));
    return m;
  }

  // BATT_PCT as the firmware keeps it: a 1 % change only after 30 s (VM.md §2.5).
  void Sensors(Arm &a, int64_t now) {
    uint32_t raw = a.sensor[VM_P_BATT_PCT];
    if (raw == VM_UNKNOWN || a.pct == VM_UNKNOWN || raw > a.pct + 1 || raw + 1 < a.pct) {
      a.pct = raw;
      a.pct_since = -1;
    } else if (raw == a.pct) {
      a.pct_since = -1;
    } else if (a.pct_since < 0) {
      a.pct_since = now;
    } else if (now - a.pct_since >= 30000000) {
      a.pct = raw;
      a.pct_since = -1;
    }
    uint32_t charge = a.sensor[VM_P_CHARGE];
    if ((a.events_mask >> VM_T_BATTERY & 1) && (a.pct != a.input_pct || charge != a.input_charge)) {
      a.input_pct = a.pct;
      a.input_charge = charge;
      std::vector<uint8_t> m = Header(a, EVT_INPUT, now);
      Put16(m, ++a.input_count);
      m.push_back(VM_T_BATTERY);
      Put32(m, a.pct);
      Notify(a, std::move(m), now);
    }
  }

  void Execute(Arm &a, int64_t now) {
    Sensors(a, now);
    uint32_t w = vm_run(&a.vm);
    a.vm_at = w == VM_FOREVER ? -1 : now + std::max<int64_t>(int64_t(w), 1);
  }

  // The microphone as g1v0 0.5.12 runs it (VM.md §6.5): the indicator first,
  // the first 40 ms dropped, then a MIC_DATA of 5 frames every 50 ms of the
  // arm's clock, counted from 0; a MIC 1 holds it for kMicLeaseUs, and one
  // with the same format renews that.
  void MicOn(Arm &a, int bytes, int64_t now) {
    a.mic_lease = now + kMicLeaseUs;
    if (a.mic && bytes == a.mic_bytes) return;
    vm_set_mic(&a.vm, 1);
    a.mic = true;
    a.mic_bytes = bytes;
    a.mic_counter = 0;
    a.mic_events = 0;
    a.mic_sample = 0;
    a.mic_start = now + 40000;
    a.mic_at = a.mic_start + MicEventUs(a, 1);
#ifdef SCOUTER_G1_MIC
    a.mic_memory.assign(lc3_encoder_size(10000, kMicSampleRate) / 8 + 1, 0);
    a.mic_encoder = lc3_setup_encoder(10000, kMicSampleRate, 0, a.mic_memory.data());
#endif
  }

  void MicOff(Arm &a) {
    if (!a.mic) return;
    a.mic = false;
    a.mic_at = -1;
    vm_set_mic(&a.vm, 0);
    PendingOff(a);
  }

  // A PANEL = 0 or 2 that waited for the microphone to stop.
  void PendingOff(Arm &a) {
    if (a.panel_held < 0) return;
    a.standby = a.panel_held == 2;
    a.panel_held = -1;
    a.panel_on = false;
    std::fill(a.panel.begin(), a.panel.end(), 0);
  }

  static void Indicator(Arm &a) {
    int x = a.index == 0 ? 4 : 620;
    for (int y = 4; y < 20; y++) std::fill_n(a.panel.begin() + y * kPanelW + x, 16, 15);
  }

  void MicStopped(Arm &a, uint8_t reason) {
    PendingOff(a);
    a.mic = false;
    a.mic_at = -1;
    std::vector<uint8_t> m = Header(a, EVT_NOTICE, NowUs());
    m.insert(m.end(), {5, reason});
    Notify(a, std::move(m), NowUs());
  }

  int64_t MicEventUs(const Arm &a, uint64_t events) const {
    return int64_t(double(events) * kMicPerEvent * 10000 / (1 + a.ppm * 1e-6));
  }

  void MicEvent(Arm &a) {
    int64_t at = a.mic_at;
    std::vector<uint8_t> m = Header(a, EVT_MIC_DATA, at);
    Put16(m, a.mic_counter);
    m.push_back(uint8_t(kMicPerEvent));
#ifdef SCOUTER_G1_MIC
    for (int f = 0; f < kMicPerEvent; f++) {
      int16_t pcm[kMicFrameSamples];
      for (int16_t &v : pcm)
        v = int16_t(std::lround(32767 * a.mic_peak *
                                std::sin(2 * std::numbers::pi * a.mic_hz * double(a.mic_sample++) / kMicSampleRate)));
      size_t end = m.size();
      m.resize(end + size_t(a.mic_bytes));
      lc3_encode(static_cast<lc3_encoder_t>(a.mic_encoder), LC3_PCM_FORMAT_S16, pcm, 1, a.mic_bytes,
                 m.data() + end);
    }
#endif
    a.mic_counter = uint16_t(a.mic_counter + kMicPerEvent);
    a.mic_events++;
    a.mic_at = a.mic_start + MicEventUs(a, a.mic_events + 1);
    Notify(a, std::move(m), at);
  }

  // Repeated once 100 ms after an ACK that reports a drained queue (VM.md §6.2).
  void SendAck(Arm &a, int64_t now, bool repeat = false) {
    a.ack_at = -1;
    a.repeat_at = !repeat && a.exec_next == a.rx_next ? now + 100000 : -1;
    std::vector<uint8_t> m = Header(a, EVT_ACK, now);
    Put16(m, a.rx_next);
    Put16(m, a.exec_next);
    m.push_back(uint8_t(a.exec_next + 32 - a.rx_next));
    uint32_t sack = 0;
    for (int i = 0; i < 32; i++) {
      uint16_t seq = uint16_t(a.rx_next + 1 + i);
      if (uint16_t(seq - a.exec_next) < 32 && a.held[seq % 32]) sack |= 1u << i;
    }
    Put32(m, sack);
    Notify(a, std::move(m), now);
  }

  void WindowError(Arm &a, uint16_t seq, int op, int64_t now) {
    std::vector<uint8_t> m = Header(a, EVT_ERROR, now);
    Put16(m, seq);
    m.push_back(0);
    m.push_back(uint8_t(op));
    m.push_back(VM_E_WINDOW);
    Put32(m, 0);
    Put32(m, 0);
    Notify(a, std::move(m), now);
  }

  void Receive(Arm &a, const std::vector<uint8_t> &p, int64_t now) {
    if (p.size() < 2 || p.size() > 244) return;
    uint16_t seq = Get16(p.data());
    uint16_t off = uint16_t(seq - a.exec_next);
    if (off >= 32 || a.held[seq % 32]) {
      if (off >= 32 && !SeqBefore(seq, a.exec_next)) WindowError(a, seq, p.size() > 2 ? p[2] : 0, now);
      SendAck(a, now);
      return;
    }
    bool hole = seq != a.rx_next;
    a.slot[seq % 32] = p;
    a.held[seq % 32] = true;
    while (uint16_t(a.rx_next - a.exec_next) < 32 && a.held[a.rx_next % 32]) a.rx_next++;
    if (hole)
      SendAck(a, now);
    else if (a.ack_at < 0)
      a.ack_at = now + 2000;
  }

  void HelloBody(Arm &a, std::vector<uint8_t> &m) {
    m.insert(m.end(), {5, 0, 5, 0});
    m.push_back(a.index == 0 ? 1 : 0);
    m.push_back(uint8_t(0x02 | 0x04 | (a.panel_on ? 0x08 : 0) | 0x40 | (a.sensor[VM_P_IN_CASE] == 1 ? 0x80 : 0)));
    m.push_back(HasMic(a) ? 0x04 : 0);
    Put16(m, Triggers(a));
    Put16(m, kPanelW);
    Put16(m, kPanelH);
    Put32(m, kArenaBytes);
    m.push_back(32);
    Put16(m, 244);
    Put16(m, a.rx_next);
    Put16(m, a.exec_next);
    m.push_back(uint8_t(a.exec_next + 32 - a.rx_next));
    Put32(m, a.boot_count);
    Put32(m, 0);
    m.push_back(0);
    m.push_back(0);
  }

  void Control(Arm &a, const std::vector<uint8_t> &r, int64_t now) {
    if (r.size() < 2) return;
    uint8_t op = r[0], tag = r[1];
    const uint8_t *body = r.data() + 2;
    size_t n = r.size() - 2;
    uint8_t status = 0;
    std::vector<uint8_t> out;
    std::vector<uint8_t> after;
    switch (op) {
    case CTRL_HELLO:
      HelloBody(a, out);
      break;
    case CTRL_PING:
      if (n < 4) {
        status = VM_E_MALFORMED;
        break;
      }
      out.insert(out.end(), body, body + 4);
      Put32(out, DeviceNow(a, now));
      Put32(out, DeviceNow(a, now) + 40);
      break;
    case CTRL_STATUS:
      out.assign(48, 0);
      for (int k = 0; k < 2; k++) {
        out[size_t(31 + k)] = uint8_t(a.interval >> (8 * k));
        out[size_t(33 + k)] = uint8_t(a.latency >> (8 * k));
        out[size_t(35 + k)] = uint8_t(a.timeout >> (8 * k));
      }
      out[37] = uint8_t(a.phy | a.phy << 4);
      out[38] = 247;
      break;
    case CTRL_RESET_SESSION: {
      int flushed = 0;
      for (bool &h : a.held) {
        flushed += h;
        h = false;
      }
      a.rx_next = a.exec_next = 0;
      a.ack_at = a.repeat_at = -1;
      vm_reset_session(&a.vm, n >= 1 && (body[0] & 1));
      if (a.sensor[VM_P_AUTO_BRIGHTNESS] != VM_UNKNOWN) a.sensor[VM_P_AUTO_BRIGHTNESS] = 0;
      a.sensor[VM_P_BRIGHTNESS_MAX] = 42;
      a.epoch = NewEpoch();
      Put16(out, uint32_t(flushed));
      HelloBody(a, out);
      break;
    }
    case CTRL_LINK: {
      if (n < 9) {
        status = VM_E_MALFORMED;
        break;
      }
      out = {0, 0};
      a.interval = std::max<int>(6, Get16(body));
      a.latency = Get16(body + 4);
      a.timeout = Get16(body + 6);
      std::vector<uint8_t> e = Header(a, EVT_NOTICE, now);
      e.push_back(1);
      Put16(e, uint32_t(a.interval));
      Put16(e, uint32_t(a.latency));
      Put16(e, uint32_t(a.timeout));
      Notify(a, std::move(e), now + 20000);
      if (body[8] && body[8] != a.phy) {
        a.phy = body[8];
        std::vector<uint8_t> p = Header(a, EVT_NOTICE, now);
        p.insert(p.end(), {2, uint8_t(a.phy), uint8_t(a.phy)});
        Notify(a, std::move(p), now + 20000);
      }
      break;
    }
    case CTRL_LOG_READ:
      Put16(out, 0);
      Put16(out, n >= 2 ? Get16(body) : 0);
      break;
    case CTRL_HEALTH_OK:
      out.push_back(0);
      break;
    case CTRL_PROPS:
      if (n > 12) {
        status = VM_E_MALFORMED;
        break;
      }
      for (size_t k = 0; k < n && !status; k++) {
        uint32_t v = 0;
        status = uint8_t(vm_prop(&a.vm, body[k], &v));
        Put32(out, v);
      }
      if (status) out.clear();
      break;
    case CTRL_PANEL_READ: {
      if (n < 4) {
        status = VM_E_MALFORMED;
        break;
      }
      if (!a.panel_on) {
        status = VM_E_PANEL;
        break;
      }
      int y0 = Get16(body), rows = Get16(body + 2), done = 0;
      uint32_t crc = 0;
      std::vector<uint8_t> first;
      for (int y = y0; y < y0 + rows && y < kPanelH; y++, done++) {
        uint8_t row[kRowBytes];
        for (int x = 0; x < kPanelW; x += 2)
          row[x / 2] = uint8_t(a.panel[size_t(y) * kPanelW + size_t(x)] << 4 |
                               a.panel[size_t(y) * kPanelW + size_t(x) + 1]);
        if (first.empty()) first.assign(row, row + 16);
        crc = Crc32(crc, row, kRowBytes);
      }
      first.resize(16, 0);
      Put32(out, crc);
      Put16(out, uint32_t(done));
      out.insert(out.end(), first.begin(), first.end());
      break;
    }
    case CTRL_ARENA_CRC: {
      if (n < 8) {
        status = VM_E_MALFORMED;
        break;
      }
      uint64_t start = Get32(body), length = Get32(body + 4);
      uint64_t end = length ? start + length : kArenaBytes;
      if (end > kArenaBytes || start > end) {
        status = VM_E_ARENA;
        break;
      }
      Put32(out, Crc32(0, a.arena.data() + start, size_t(end - start)));
      for (uint64_t k = 0; k < 16; k++) out.push_back(start + k < end ? a.arena[start + k] : 0);
      break;
    }
    case CTRL_VM_START:
      if (n < 4) {
        status = VM_E_MALFORMED;
        break;
      }
      status = uint8_t(vm_start(&a.vm, Get32(body)));
      a.vm_at = now;
      break;
    case CTRL_VM_STOP:
      vm_stop(&a.vm);
      break;
    case CTRL_VM_STATUS:
      out.assign(32, 0);
      vm_status(&a.vm, out.data());
      break;
    case CTRL_MIC: {
      bool on = n >= 1 && body[0];
      uint8_t rate = n >= 2 ? body[1] : 0;
      if (rate > 1) {
        status = VM_E_ARG;
        break;
      }
      if (on && !HasMic(a)) {
        status = VM_E_MIC; // HELLO.board bit 2 is clear
        break;
      }
      if (on)
        MicOn(a, rate ? 20 : 40, now);
      else
        MicOff(a);
      out = {uint8_t(on), uint8_t(rate ? 20 : 40), uint8_t(kMicPerEvent)};
      Put16(out, 16000);
      out.push_back(10);
      Put16(out, uint16_t(kMicLeaseUs / 1000));
      break;
    }
    case CTRL_EVENTS:
      if (n >= 2) {
        a.events_mask = Get16(body) & Triggers(a);
        a.input_pct = a.pct;
        a.input_charge = a.sensor[VM_P_CHARGE];
      }
      Put16(out, a.events_mask);
      break;
    case CTRL_PANEL_GUARD:
      if (n >= 1 && body[0] > 1) {
        status = VM_E_ARG;
        break;
      }
      if (n >= 1) a.guard = body[0] == 1;
      if (!a.guard) a.sensor[VM_P_BRIGHTNESS_CAP] = 42;
      out.push_back(a.guard ? 1 : 0);
      break;
    case CTRL_ASSET_INFO:
    case CTRL_ASSET_LIST:
    case CTRL_ASSET_PUT:
    case CTRL_ASSET_DELETE:
    case CTRL_ASSET_FORMAT:
      status = a.store ? uint8_t(AssetControl(a, op, body, n, out, now)) : uint8_t(VM_E_OP);
      break;
    default:
      status = VM_E_OP;
      break;
    }
    std::vector<uint8_t> m = Header(a, uint8_t(0x80 | op), now);
    m.push_back(tag);
    m.push_back(status);
    m.insert(m.end(), out.begin(), out.end());
    Notify(a, std::move(m), now);
  }

  // VM.md §6.10, as g1v0 0.5.56 answers (a job takes 20 ms plus 15 ms per sector)
  static uint32_t Extent(uint32_t size) { return (size + kSector - 1) / kSector * kSector; }

  int64_t FirstFit(const Arm &a, uint32_t size) {
    uint32_t at = 2 * kSector, need = Extent(size);
    for (bool moved = true; moved;) {
      moved = false;
      for (auto &x : a.assets)
        if (at < x.offset + Extent(uint32_t(x.body.size())) && x.offset < at + need) {
          at = x.offset + Extent(uint32_t(x.body.size()));
          moved = true;
        }
    }
    return at + need <= kStoreBytes ? at : -1;
  }

  int AssetControl(Arm &a, uint8_t op, const uint8_t *body, size_t n, std::vector<uint8_t> &out, int64_t now) {
    bool busy = a.asset_at >= 0;
    auto find = [&](const uint8_t *id) {
      for (size_t i = 0; i < a.assets.size(); i++)
        if (std::memcmp(a.assets[i].id.b, id, 16) == 0) return int(i);
      return -1;
    };
    auto start = [&](uint8_t job, uint8_t state, int64_t us) {
      a.job = job;
      a.asset_state = state;
      a.asset_at = now + us;
    };
    switch (op) {
    case CTRL_ASSET_INFO: {
      uint32_t data = 0, used = 0, largest = 0;
      std::vector<std::pair<uint32_t, uint32_t>> ext;
      for (auto &x : a.assets) {
        data += uint32_t(x.body.size());
        used += Extent(uint32_t(x.body.size()));
        ext.push_back({x.offset, x.offset + Extent(uint32_t(x.body.size()))});
      }
      std::sort(ext.begin(), ext.end());
      uint32_t at = 2 * kSector;
      for (auto &e : ext) {
        largest = std::max(largest, e.first - at);
        at = e.second;
      }
      largest = std::max(largest, kStoreBytes - at);
      Put32(out, data);
      Put32(out, kStoreBytes - 2 * kSector - used);
      Put32(out, largest);
      out.push_back(uint8_t(a.assets.size()));
      out.push_back(a.asset_state);
      out.push_back(a.asset_error);
      out.push_back(1);
      Put32(out, a.asset_seq);
      return 0;
    }
    case CTRL_ASSET_LIST: {
      size_t first = n >= 1 ? body[0] : 0;
      out.push_back(uint8_t(a.assets.size()));
      out.push_back(uint8_t(first));
      for (size_t i = first; i < a.assets.size() && i < first + 7; i++) {
        const auto &x = a.assets[i];
        uint32_t crc = Crc32(0, x.body.data(), x.body.size());
        out.insert(out.end(), x.id.b, x.id.b + 16);
        Put32(out, x.offset);
        Put32(out, uint32_t(x.body.size()));
        Put32(out, crc);
        out.insert(out.end(), {x.type, 0, 0, 0});
      }
      return 0;
    }
    case CTRL_ASSET_PUT: {
      if (n < 25) return VM_E_MALFORMED;
      uint32_t addr = Get32(body + 16), size = Get32(body + 20);
      if (size == 0 || size > kArenaBytes) return VM_E_ARG;
      if (uint64_t(addr) + size > kArenaBytes) return VM_E_ARENA;
      if (int k = find(body); k >= 0) {
        if (a.assets[size_t(k)].body.size() != size) return VM_E_ARG;
        out.push_back(1);
        return 0;
      }
      int64_t off = busy || a.assets.size() >= 120 ? -1 : FirstFit(a, size);
      if (off < 0) return VM_E_ASSET;
      std::memcpy(a.job_id.b, body, 16);
      a.job_addr = addr;
      a.job_size = size;
      a.job_type = body[24];
      a.job_offset = uint32_t(off);
      start(1, 1, 20000 + 15000 * int64_t(Extent(size) / kSector));
      out.push_back(0);
      return 0;
    }
    case CTRL_ASSET_DELETE:
      if (n < 16 || find(body) < 0 || busy) return n < 16 ? VM_E_MALFORMED : VM_E_ASSET;
      std::memcpy(a.job_id.b, body, 16);
      start(2, 3, 30000);
      return 0;
    default: // CTRL_ASSET_FORMAT
      if (n < 4 || Get32(body) != 0x534B4153u) return VM_E_ARG;
      if (busy) return VM_E_ASSET;
      std::memset(a.job_id.b, 0, 16);
      start(3, 1, 50000);
      return 0;
    }
  }

  void AssetJobDone(Arm &a, int64_t now) {
    uint8_t result = 0;
    if (a.job == 1) {
      std::span<const uint8_t> staged(a.arena.data() + a.job_addr, a.job_size);
      AssetId got = AssetIdOf(staged);
      if (std::memcmp(got.b, a.job_id.b, 16) == 0) {
        a.assets.push_back({a.job_id, a.job_offset, a.job_type, {staged.begin(), staged.end()}});
        a.asset_seq++;
      } else {
        result = VM_E_ASSET;
      }
    } else if (a.job == 2) {
      for (size_t i = 0; i < a.assets.size(); i++)
        if (a.assets[i].id == a.job_id) a.assets.erase(a.assets.begin() + long(i));
      a.asset_seq++;
    } else {
      a.assets.clear();
      a.asset_seq = 0;
    }
    a.asset_at = -1;
    a.asset_state = result ? 5 : 4;
    a.asset_error = result;
    std::vector<uint8_t> m = Header(a, EVT_NOTICE, now);
    m.insert(m.end(), {8, a.job, result});
    m.insert(m.end(), a.job_id.b, a.job_id.b + 16);
    Notify(a, std::move(m), now);
  }

  std::string adapter_;
  Events *events_ = nullptr;
  bool present_[2] = {false, false};
  Arm arms_[2];
  std::vector<uint8_t> snapshot_[2];
  std::mutex mutex_;
  std::condition_variable cv_;
  bool woken_ = false;
  double write_loss_ = 0, notify_loss_ = 0;
  std::mt19937 rng_{12345};
};

EmulatorTransport *g_emulator = nullptr;

#elif !defined(SCOUTER_G1_NO_BLUEZ) // ------------------------------------ BlueZ (sd-bus)

const char *const kService = "f3da4d59-277a-4fc9-a1ed-41c9c22d4400";
const char *const kCharUuid[3] = {"f3da4d59-277a-4fc9-a1ed-41c9c22d4401",
                                  "f3da4d59-277a-4fc9-a1ed-41c9c22d4402",
                                  "f3da4d59-277a-4fc9-a1ed-41c9c22d4403"};
const char *const kAgentPath = "/org/scouter_kit/agent";

struct Value {
  std::string s;
  int64_t i = 0;
  bool b = false;
  std::vector<std::string> list;
  std::map<int, std::vector<uint8_t>> data; // ManufacturerData
};
using Props = std::map<std::string, Value>;
using Interfaces = std::map<std::string, Props>;
using Objects = std::map<std::string, Interfaces>;

std::string Lower(std::string s) {
  for (char &c : s) c = char(std::tolower(uint8_t(c)));
  return s;
}

std::string Upper(std::string s) {
  for (char &c : s) c = char(std::toupper(uint8_t(c)));
  return s;
}

int ReadVariant(sd_bus_message *m, Value *v) {
  const char *sig = nullptr;
  int r = sd_bus_message_peek_type(m, nullptr, &sig);
  if (r < 0) return r;
  std::string type = sig ? sig : "";
  if ((r = sd_bus_message_enter_container(m, 'v', sig)) < 0) return r;
  if (type == "s" || type == "o") {
    const char *s = nullptr;
    r = sd_bus_message_read_basic(m, type[0], &s);
    v->s = s ? s : "";
  } else if (type == "b") {
    int b = 0;
    r = sd_bus_message_read_basic(m, 'b', &b);
    v->b = b;
  } else if (type == "n") {
    int16_t x = 0;
    r = sd_bus_message_read_basic(m, 'n', &x);
    v->i = x;
  } else if (type == "q") {
    uint16_t x = 0;
    r = sd_bus_message_read_basic(m, 'q', &x);
    v->i = x;
  } else if (type == "as") {
    char **list = nullptr;
    r = sd_bus_message_read_strv(m, &list);
    for (char **p = list; p && *p; p++) {
      v->list.push_back(*p);
      free(*p);
    }
    free(list);
  } else if (type == "a{qv}") {
    if ((r = sd_bus_message_enter_container(m, 'a', "{qv}")) < 0) return r;
    while ((r = sd_bus_message_enter_container(m, 'e', "qv")) > 0) {
      uint16_t key = 0;
      const void *bytes = nullptr;
      size_t size = 0;
      if ((r = sd_bus_message_read_basic(m, 'q', &key)) < 0 ||
          (r = sd_bus_message_enter_container(m, 'v', "ay")) < 0 ||
          (r = sd_bus_message_read_array(m, 'y', &bytes, &size)) < 0 ||
          (r = sd_bus_message_exit_container(m)) < 0 || (r = sd_bus_message_exit_container(m)) < 0)
        return r;
      const uint8_t *p = static_cast<const uint8_t *>(bytes);
      v->data[key].assign(p, p + size);
    }
    if (r >= 0) r = sd_bus_message_exit_container(m);
  } else {
    r = sd_bus_message_skip(m, sig);
  }
  if (r < 0) return r;
  return sd_bus_message_exit_container(m);
}

int ReadProps(sd_bus_message *m, Props *props) {
  int r = sd_bus_message_enter_container(m, 'a', "{sv}");
  if (r < 0) return r;
  while ((r = sd_bus_message_enter_container(m, 'e', "sv")) > 0) {
    const char *name = nullptr;
    if ((r = sd_bus_message_read_basic(m, 's', &name)) < 0) return r;
    Value v;
    if ((r = ReadVariant(m, &v)) < 0) return r;
    (*props)[name] = std::move(v);
    if ((r = sd_bus_message_exit_container(m)) < 0) return r;
  }
  if (r < 0) return r;
  return sd_bus_message_exit_container(m);
}

int ReadInterfaces(sd_bus_message *m, Interfaces *ifaces) {
  int r = sd_bus_message_enter_container(m, 'a', "{sa{sv}}");
  if (r < 0) return r;
  while ((r = sd_bus_message_enter_container(m, 'e', "sa{sv}")) > 0) {
    const char *name = nullptr;
    if ((r = sd_bus_message_read_basic(m, 's', &name)) < 0) return r;
    if ((r = ReadProps(m, &(*ifaces)[name])) < 0) return r;
    if ((r = sd_bus_message_exit_container(m)) < 0) return r;
  }
  if (r < 0) return r;
  return sd_bus_message_exit_container(m);
}

int ReadObjects(sd_bus_message *m, Objects *objects) {
  int r = sd_bus_message_enter_container(m, 'a', "{oa{sa{sv}}}");
  if (r < 0) return r;
  while ((r = sd_bus_message_enter_container(m, 'e', "oa{sa{sv}}")) > 0) {
    const char *path = nullptr;
    if ((r = sd_bus_message_read_basic(m, 'o', &path)) < 0) return r;
    if ((r = ReadInterfaces(m, &(*objects)[path])) < 0) return r;
    if ((r = sd_bus_message_exit_container(m)) < 0) return r;
  }
  if (r < 0) return r;
  return sd_bus_message_exit_container(m);
}

// Finds the arms (given addresses, else bonded or advertising G1 arms), bonds
// through its own agent when needed, connects, and acquires the CMD/CTRL
// write and EVT notify sockets.
class BluezTransport : public Transport {
public:
  BluezTransport(const std::string addresses[2], int timeout_ms, const std::string &adapter) {
    for (int i = 0; i < 2; i++) dev_[i].address = Upper(addresses[i]);
    timeout_ms_ = timeout_ms;
    wanted_adapter_ = adapter;
  }

  bool Start(Events *events, const bool present[2]) override {
    events_ = events;
    wake_fd_ = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
    if (sd_bus_open_system(&bus_) < 0) {
      events_->Problem(ERROR_BLUETOOTH_ERROR, "cannot open the system D-Bus");
      return false;
    }
    if (!FindAdapter()) return false;
    sd_bus_add_match(bus_, &match_props_,
                     "type='signal',sender='org.bluez',interface='org.freedesktop.DBus."
                     "Properties',member='PropertiesChanged',path_namespace='/org/bluez'",
                     &BluezTransport::OnProperties, this);
    sd_bus_add_match(bus_, &match_added_,
                     "type='signal',sender='org.bluez',interface='org.freedesktop.DBus."
                     "ObjectManager',member='InterfacesAdded'",
                     &BluezTransport::OnAdded, this);
    deadline_ = NowUs() + int64_t(timeout_ms_) * 1000;
    for (int i = 0; i < 2; i++) {
      Dev &d = dev_[i];
      d.arm = i;
      d.self = this;
      d.wanted = present[i];
      if (d.wanted && !d.address.empty()) d.path = PathOf(d.address);
      if (d.wanted) d.st = Dev::FIND;
    }
    Objects objects;
    if (!Managed(&objects)) return false;
    for (auto &o : objects) {
      auto it = o.second.find("org.bluez.Device1");
      if (it != o.second.end()) Merge(o.first, it->second);
    }
    return true;
  }

  bool Reject(int arm) override {
    Dev &d = dev_[arm];
    rejected_.insert(d.path);
    auto &list = candidates_[arm];
    std::string partner = dev_[1 - arm].path.empty() ? "" : PairIdOf(dev_[1 - arm].path);
    bool other = std::any_of(list.begin(), list.end(),
                             [&](const Candidate &c) { return Eligible(c, partner, rejected_); });
    if (!other) return false;
    Disconnect(arm);
    d.path.clear();
    d.address.clear();
    d.reported = false;
    d.st = Dev::FIND;
    return true;
  }

  bool Write(int arm, int channel, const uint8_t *data, size_t n) override {
    Dev &d = dev_[arm];
    if (d.st != Dev::UP) return false;
    return ::write(d.fd[channel], data, n) == ssize_t(n);
  }

  void Wake() override {
    uint64_t one = 1;
    if (wake_fd_ >= 0) (void)::write(wake_fd_, &one, sizeof(one));
  }

  void Disconnect(int arm) override {
    Dev &d = dev_[arm];
    if (d.path.empty()) return;
    sd_bus_call_method_async(bus_, nullptr, "org.bluez", d.path.c_str(), "org.bluez.Device1",
                             "Disconnect", nullptr, nullptr, "");
    Down(d);
  }

  void Hold(int arm, int ms) override {
    Dev &d = dev_[arm];
    if (d.path.empty()) return;
    Disconnect(arm);
    d.retry_at = NowUs() + int64_t(ms) * 1000;
  }

  void Stop() override {
    for (auto &d : dev_) {
      if (d.path.empty() || d.st == Dev::IDLE) continue;
      CloseFds(d);
      d.call = sd_bus_slot_unref(d.call);
      sd_bus_call_method(bus_, "org.bluez", d.path.c_str(), "org.bluez.Device1", "Disconnect",
                         nullptr, nullptr, "");
    }
    StopDiscovery();
    for (auto &d : dev_) d.st = Dev::IDLE;
    UnregisterAgent();
    match_props_ = sd_bus_slot_unref(match_props_);
    match_added_ = sd_bus_slot_unref(match_added_);
    bus_ = sd_bus_flush_close_unref(bus_);
    if (wake_fd_ >= 0) close(wake_fd_);
    wake_fd_ = -1;
  }

  void Poll(int timeout_ms) override {
    std::vector<pollfd> fds;
    fds.push_back({wake_fd_, POLLIN, 0});
    fds.push_back({sd_bus_get_fd(bus_), short(sd_bus_get_events(bus_)), 0});
    uint64_t bus_at = 0;
    if (sd_bus_get_timeout(bus_, &bus_at) >= 0 && bus_at != UINT64_MAX) {
      timespec ts;
      clock_gettime(CLOCK_MONOTONIC, &ts);
      uint64_t now = uint64_t(ts.tv_sec) * 1000000 + uint64_t(ts.tv_nsec) / 1000;
      timeout_ms = std::min(timeout_ms, bus_at > now ? int((bus_at - now + 999) / 1000) : 0);
    }
    size_t first[2] = {0, 0};
    for (int i = 0; i < 2; i++) {
      Dev &d = dev_[i];
      if (d.st != Dev::UP) continue;
      first[i] = fds.size();
      fds.push_back({d.fd[2], POLLIN, 0});
      fds.push_back({d.fd[0], 0, 0});
      fds.push_back({d.fd[1], 0, 0});
    }
    poll(fds.data(), fds.size(), std::max(timeout_ms, 0));
    if (fds[0].revents) {
      uint64_t v;
      while (::read(wake_fd_, &v, sizeof(v)) > 0) {
      }
    }
    while (sd_bus_process(bus_, nullptr) > 0) {
    }
    for (int i = 0; i < 2; i++) {
      Dev &d = dev_[i];
      if (!first[i] || d.st != Dev::UP) continue;
      bool broken = false;
      for (size_t k = 0; k < 3; k++)
        if (fds[first[i] + k].revents & (POLLHUP | POLLERR | POLLNVAL)) broken = true;
      if (fds[first[i]].revents & POLLIN) {
        uint8_t buf[600];
        while (true) {
          ssize_t n = ::read(d.fd[2], buf, sizeof(buf));
          if (n > 0) {
            events_->Data(i, buf, size_t(n));
            continue;
          }
          if (n == 0 || (errno != EAGAIN && errno != EINTR)) broken = true;
          break;
        }
      }
      if (broken && d.st == Dev::UP) Down(d);
    }
    Timers();
  }

private:
  struct Dev {
    BluezTransport *self = nullptr;
    int arm = 0;
    bool wanted = false;
    std::string address, path;
    enum State { IDLE, FIND, PAIR, CONNECT, RESOLVE, ACQUIRE, UP, BACKOFF } st = IDLE;
    sd_bus_slot *call = nullptr;
    int fd[3] = {-1, -1, -1}; // CMD, CTRL, EVT
    int mtu = 23;
    int acquired = 0;
    std::string chars[3];
    int64_t retry_at = 0;
    int64_t state_deadline = 0;
    int backoff_ms = 500;
    bool paired = false, connected = false, resolved = false;
    int pair_tries = 0;
    bool reported = false;
  };

  std::string PathOf(const std::string &address) {
    std::string p = adapter_ + "/dev_" + address;
    for (size_t i = adapter_.size(); i < p.size(); i++)
      if (p[i] == ':') p[i] = '_';
    return p;
  }

  bool Managed(Objects *objects) {
    sd_bus_message *reply = nullptr;
    sd_bus_error error = SD_BUS_ERROR_NULL;
    int r = sd_bus_call_method(bus_, "org.bluez", "/", "org.freedesktop.DBus.ObjectManager",
                               "GetManagedObjects", &error, &reply, "");
    if (r >= 0) r = ReadObjects(reply, objects);
    sd_bus_message_unref(reply);
    std::string text = error.message ? error.message : strerror(-r);
    sd_bus_error_free(&error);
    if (r < 0) {
      events_->Problem(ERROR_BLUETOOTH_ERROR, "BlueZ does not answer: " + text);
      return false;
    }
    return true;
  }

  bool FindAdapter() {
    Objects objects;
    if (!Managed(&objects)) return false;
    std::vector<AdapterInfo> adapters;
    for (auto &o : objects) {
      auto it = o.second.find("org.bluez.Adapter1");
      if (it != o.second.end())
        adapters.push_back({o.first, it->second["Address"].s, it->second["Powered"].b});
    }
    std::string problem;
    int k = PickAdapter(adapters, wanted_adapter_, &problem);
    if (k < 0) {
      events_->Problem(ERROR_BLUETOOTH_ERROR, problem);
      return false;
    }
    adapter_ = adapters[size_t(k)].path;
    Log("adapter %s (%s)", adapter_.c_str(), adapters[size_t(k)].address.c_str());
    return true;
  }

  bool Searching() const {
    for (auto &d : dev_)
      if (d.wanted && d.address.empty()) return true;
    return false;
  }

  // Keeps every device's properties up to date (signals carry only changes).
  void Merge(const std::string &path, const Props &changes) {
    Props &p = devices_[path];
    for (auto &c : changes) p[c.first] = c.second;
    Consider(path, p);
  }

  // A device object: one of ours, or (no address given) a candidate arm.
  void Consider(const std::string &path, Props &p) {
    if (p.count("Adapter") && p["Adapter"].s != adapter_) return;
    known_.insert(path);
    for (auto &d : dev_) {
      if (!d.wanted || d.path != path) continue;
      if (p.count("Paired")) d.paired = p["Paired"].b;
      if (p.count("Connected")) d.connected = p["Connected"].b;
      if (p.count("ServicesResolved")) d.resolved = p["ServicesResolved"].b;
    }
    if (!Searching()) return;
    bool ours = false;
    for (auto &u : p["UUIDs"].list)
      if (Lower(u) == kService) ours = true;
    std::string name = p["Name"].s.empty() ? p["Alias"].s : p["Name"].s;
    int role = -1;
    if (name.size() >= 2 && name.compare(name.size() - 2, 2, "_L") == 0) role = 0;
    if (name.size() >= 2 && name.compare(name.size() - 2, 2, "_R") == 0) role = 1;
    // Since 0.5.66 an arm advertises its name and pair record, without the service UUID.
    if (name.rfind("G1CFW_L_", 0) == 0) ours = true, role = 0;
    if (name.rfind("G1CFW_R_", 0) == 0) ours = true, role = 1;
    Candidate c{path, p["Address"].s, p["Paired"].b, p["Connected"].b,
                p.count("RSSI") ? int(p["RSSI"].i) : -999, -1, ""};
    if (ParsePairRecord(p["ManufacturerData"].data[0xFFFF], &c.flags, &c.pair_id)) role = c.flags & 1 ? 0 : 1;
    if (!ours || role < 0) return;
    for (auto &list : candidates_)
      std::erase_if(list, [&](const Candidate &x) { return x.path == path; });
    candidates_[role].push_back(c);
  }

  std::string PairIdOf(const std::string &path) {
    auto it = devices_.find(path);
    int flags = -1;
    std::string id;
    if (it != devices_.end()) ParsePairRecord(it->second["ManufacturerData"].data[0xFFFF], &flags, &id);
    return id;
  }

  // With several eligible candidates, an active discovery runs for 4 s first:
  // it refreshes the pair records (scan responses) and the signal strengths.
  void Choose(Dev &d, int64_t now) {
    std::string partner = dev_[1 - d.arm].path.empty() ? "" : PairIdOf(dev_[1 - d.arm].path);
    auto &mine = candidates_[d.arm];
    int eligible = int(std::count_if(mine.begin(), mine.end(),
                                     [&](const Candidate &c) { return Eligible(c, partner, rejected_); }));
    if (eligible == 0) {
      StartDiscovery();
      return;
    }
    if (eligible > 1 && !ambiguous_until_) {
      StartDiscovery();
      ambiguous_until_ = now + 4000000;
      return;
    }
    if (now < ambiguous_until_) return;
    int k = PickCandidate(mine, candidates_[1 - d.arm], partner, rejected_);
    if (k < 0) return;
    const Candidate &best = mine[size_t(k)];
    d.path = best.path;
    d.address = best.address;
    d.paired = best.paired;
    d.connected = best.connected;
    Log("%s arm: %d of %zu candidates eligible, chose %s (pair ID %s, RSSI %d)", kArmName[d.arm], eligible,
        mine.size(), best.address.c_str(), best.pair_id.empty() ? "?" : best.pair_id.c_str(), best.rssi);
  }

  void StartDiscovery() {
    if (discovering_) return;
    sd_bus_message *m = nullptr;
    sd_bus_message_new_method_call(bus_, &m, "org.bluez", adapter_.c_str(), "org.bluez.Adapter1",
                                   "SetDiscoveryFilter");
    if (Searching())
      sd_bus_message_append(m, "a{sv}", 2, "Transport", "s", "le", "Pattern", "s", "G1");
    else
      sd_bus_message_append(m, "a{sv}", 1, "Transport", "s", "le");
    sd_bus_call(bus_, m, 0, nullptr, nullptr);
    sd_bus_message_unref(m);
    discovering_ = sd_bus_call_method(bus_, "org.bluez", adapter_.c_str(), "org.bluez.Adapter1",
                                      "StartDiscovery", nullptr, nullptr, "") >= 0;
    Log("discovery %s", discovering_ ? "started" : "failed");
  }

  void StopDiscovery() {
    if (!discovering_) return;
    sd_bus_call_method(bus_, "org.bluez", adapter_.c_str(), "org.bluez.Adapter1", "StopDiscovery",
                       nullptr, nullptr, "");
    discovering_ = false;
  }

  void Timers() {
    int64_t now = NowUs();
    bool searching = false;
    for (auto &d : dev_) {
      if (!d.wanted) continue;
      switch (d.st) {
      case Dev::FIND:
        if (d.path.empty()) Choose(d, now);
        if (!d.path.empty() && known_.count(d.path)) {
          if (!d.reported) {
            d.reported = true;
            events_->Found(d.arm, d.address);
          }
          Connect(d);
          break;
        }
        searching = true;
        if (!d.path.empty()) StartDiscovery();
        if (now > deadline_ && !d.reported) {
          d.reported = true;
          events_->Problem(ERROR_NO_G1_FOUND, std::string("the ") + kArmName[d.arm] +
                                                  " arm was not found (switched on, in range?)");
        }
        break;
      case Dev::RESOLVE:
        if (d.resolved) {
          Acquire(d);
        } else if (now > d.state_deadline) {
          Log("%s arm: services not resolved, reconnecting", kArmName[d.arm]);
          Disconnect(d.arm);
        }
        break;
      case Dev::BACKOFF:
        if (now >= d.retry_at) Connect(d);
        break;
      default:
        break;
      }
    }
    if (!searching && discovering_) StopDiscovery();
  }

  void Connect(Dev &d) {
    d.call = sd_bus_slot_unref(d.call);
    if (!d.paired) {
      Pair(d);
      return;
    }
    if (d.connected && d.resolved) {
      Acquire(d);
      return;
    }
    d.st = Dev::CONNECT;
    sd_bus_call_method_async(bus_, &d.call, "org.bluez", d.path.c_str(), "org.bluez.Device1",
                             "Connect", &BluezTransport::OnConnect, &d, "");
  }

  static int OnConnect(sd_bus_message *m, void *userdata, sd_bus_error *) {
    Dev &d = *static_cast<Dev *>(userdata);
    d.call = sd_bus_slot_unref(d.call);
    const sd_bus_error *e = sd_bus_message_get_error(m);
    if (e && !sd_bus_error_has_name(e, "org.bluez.Error.AlreadyConnected")) {
      Log("%s arm: connect failed: %s", kArmName[d.arm], e->message ? e->message : e->name);
      d.self->Backoff(d);
      return 0;
    }
    d.st = Dev::RESOLVE;
    d.state_deadline = NowUs() + 15000000;
    return 0;
  }

  void Pair(Dev &d) {
    if (!RegisterAgent()) {
      events_->Problem(ERROR_G1_BONDING_ERROR, "cannot register the pairing agent");
      d.st = Dev::IDLE;
      return;
    }
    d.st = Dev::PAIR;
    Log("%s arm: pairing with %s", kArmName[d.arm], d.address.c_str());
    sd_bus_call_method_async(bus_, &d.call, "org.bluez", d.path.c_str(), "org.bluez.Device1", "Pair",
                             &BluezTransport::OnPair, &d, "");
  }

  static int OnPair(sd_bus_message *m, void *userdata, sd_bus_error *) {
    Dev &d = *static_cast<Dev *>(userdata);
    BluezTransport *t = d.self;
    d.call = sd_bus_slot_unref(d.call);
    const sd_bus_error *e = sd_bus_message_get_error(m);
    if (e) {
      Log("%s arm: pairing failed: %s %s", kArmName[d.arm], e->name ? e->name : "",
          e->message ? e->message : "");
      if (sd_bus_error_has_name(e, "org.bluez.Error.AlreadyExists")) {
        d.paired = true;
        t->Connect(d);
        return 0;
      }
      if (++d.pair_tries >= 3) {
        d.st = Dev::IDLE;
        t->UnregisterAgent();
        t->events_->Problem(ERROR_G1_BONDING_ERROR,
                            std::string("pairing with the ") + kArmName[d.arm] +
                                " arm failed; pair with the glasses in the open charging case");
        return 0;
      }
      t->Backoff(d);
      return 0;
    }
    d.paired = true;
    sd_bus_set_property(t->bus_, "org.bluez", d.path.c_str(), "org.bluez.Device1", "Trusted",
                        nullptr, "b", 1);
    d.st = Dev::RESOLVE;
    d.state_deadline = NowUs() + 15000000;
    t->UnregisterAgent();
    Log("%s arm: bonded", kArmName[d.arm]);
    return 0;
  }

  bool RegisterAgent() {
    if (agent_) return true;
    if (sd_bus_add_object_vtable(bus_, &agent_slot_, kAgentPath, "org.bluez.Agent1", kAgentVtable,
                                 this) < 0)
      return false;
    if (sd_bus_call_method(bus_, "org.bluez", "/org/bluez", "org.bluez.AgentManager1",
                           "RegisterAgent", nullptr, nullptr, "os", kAgentPath,
                           "NoInputNoOutput") < 0) {
      agent_slot_ = sd_bus_slot_unref(agent_slot_);
      return false;
    }
    sd_bus_call_method(bus_, "org.bluez", "/org/bluez", "org.bluez.AgentManager1",
                       "RequestDefaultAgent", nullptr, nullptr, "o", kAgentPath);
    sd_bus_message *reply = nullptr;
    int on = 0;
    if (sd_bus_get_property(bus_, "org.bluez", adapter_.c_str(), "org.bluez.Adapter1", "Pairable",
                            nullptr, &reply, "b") >= 0 &&
        sd_bus_message_read(reply, "b", &on) >= 0)
      pairable_before_ = on;
    sd_bus_message_unref(reply);
    if (!pairable_before_)
      sd_bus_set_property(bus_, "org.bluez", adapter_.c_str(), "org.bluez.Adapter1", "Pairable",
                          nullptr, "b", 1);
    agent_ = true;
    return true;
  }

  void UnregisterAgent() {
    if (!agent_) return;
    for (auto &d : dev_)
      if (d.st == Dev::PAIR) return;
    sd_bus_call_method(bus_, "org.bluez", "/org/bluez", "org.bluez.AgentManager1",
                       "UnregisterAgent", nullptr, nullptr, "o", kAgentPath);
    agent_slot_ = sd_bus_slot_unref(agent_slot_);
    if (pairable_before_ == 0)
      sd_bus_set_property(bus_, "org.bluez", adapter_.c_str(), "org.bluez.Adapter1", "Pairable",
                          nullptr, "b", 0);
    pairable_before_ = -1;
    agent_ = false;
  }

  static int AgentAccept(sd_bus_message *m, void *userdata, sd_bus_error *) {
    auto *t = static_cast<BluezTransport *>(userdata);
    const char *device = nullptr;
    sd_bus_message_read_basic(m, 'o', &device);
    for (auto &d : t->dev_)
      if (d.st == Dev::PAIR && device && d.path == device) return sd_bus_reply_method_return(m, "");
    return sd_bus_reply_method_errorf(m, "org.bluez.Error.Rejected", "not a device being paired");
  }

  static int AgentReject(sd_bus_message *m, void *, sd_bus_error *) {
    return sd_bus_reply_method_errorf(m, "org.bluez.Error.Rejected", "no input or output");
  }

  static int AgentOk(sd_bus_message *m, void *, sd_bus_error *) { return sd_bus_reply_method_return(m, ""); }

  static const sd_bus_vtable kAgentVtable[];

  void Acquire(Dev &d) {
    Objects objects;
    if (!Managed(&objects)) {
      Backoff(d);
      return;
    }
    std::string service;
    for (auto &o : objects) {
      auto it = o.second.find("org.bluez.GattService1");
      if (it != o.second.end() && it->second["Device"].s == d.path &&
          Lower(it->second["UUID"].s) == kService)
        service = o.first;
    }
    for (auto &c : d.chars) c.clear();
    for (auto &o : objects) {
      auto it = o.second.find("org.bluez.GattCharacteristic1");
      if (it == o.second.end() || service.empty() || it->second["Service"].s != service) continue;
      std::string uuid = Lower(it->second["UUID"].s);
      for (int k = 0; k < 3; k++)
        if (uuid == kCharUuid[k]) d.chars[k] = o.first;
    }
    if (d.chars[0].empty() || d.chars[1].empty() || d.chars[2].empty()) {
      events_->Problem(ERROR_G1_FIRMWARE, std::string("the ") + kArmName[d.arm] +
                                              " arm does not offer the scouter service (stock "
                                              "firmware?)");
      Disconnect(d.arm);
      return;
    }
    d.st = Dev::ACQUIRE;
    d.acquired = 0;
    d.mtu = 517;
    AcquireNext(d);
  }

  static int CharOf(int step) { return step == 0 ? 2 : step - 1; } // EVT first, then CMD, CTRL

  void AcquireNext(Dev &d) {
    int k = CharOf(d.acquired);
    sd_bus_message *m = nullptr;
    sd_bus_message_new_method_call(bus_, &m, "org.bluez", d.chars[k].c_str(),
                                   "org.bluez.GattCharacteristic1",
                                   k == 2 ? "AcquireNotify" : "AcquireWrite");
    sd_bus_message_append(m, "a{sv}", 0);
    d.call = sd_bus_slot_unref(d.call);
    sd_bus_call_async(bus_, &d.call, m, &BluezTransport::OnAcquired, &d, 0);
    sd_bus_message_unref(m);
  }

  static int OnAcquired(sd_bus_message *m, void *userdata, sd_bus_error *) {
    Dev &d = *static_cast<Dev *>(userdata);
    BluezTransport *t = d.self;
    d.call = sd_bus_slot_unref(d.call);
    int k = CharOf(d.acquired);
    int fd = -1;
    uint16_t mtu = 0;
    const sd_bus_error *e = sd_bus_message_get_error(m);
    if (e || sd_bus_message_read(m, "hq", &fd, &mtu) < 0) {
      Log("%s arm: acquiring a socket failed: %s", kArmName[d.arm], e && e->message ? e->message : "?");
      t->CloseFds(d);
      t->Disconnect(d.arm);
      return 0;
    }
    d.fd[k] = fcntl(fd, F_DUPFD_CLOEXEC, 3);
    fcntl(d.fd[k], F_SETFL, fcntl(d.fd[k], F_GETFL) | O_NONBLOCK);
    d.mtu = std::min<int>(d.mtu, mtu);
    if (++d.acquired < 3) {
      t->AcquireNext(d);
      return 0;
    }
    d.st = Dev::UP;
    d.backoff_ms = 500;
    t->events_->LinkUp(d.arm, d.mtu);
    return 0;
  }

  void CloseFds(Dev &d) {
    for (int &fd : d.fd) {
      if (fd >= 0) close(fd);
      fd = -1;
    }
  }

  void Down(Dev &d) {
    bool was_up = d.st == Dev::UP;
    d.call = sd_bus_slot_unref(d.call);
    CloseFds(d);
    Backoff(d);
    if (was_up) events_->LinkDown(d.arm);
  }

  void Backoff(Dev &d) {
    d.st = Dev::BACKOFF;
    d.retry_at = NowUs() + int64_t(d.backoff_ms) * 1000;
    d.backoff_ms = std::min(d.backoff_ms * 2, 4000);
  }

  static int OnProperties(sd_bus_message *m, void *userdata, sd_bus_error *) {
    auto *t = static_cast<BluezTransport *>(userdata);
    const char *path = sd_bus_message_get_path(m);
    const char *iface = nullptr;
    if (!path || sd_bus_message_read_basic(m, 's', &iface) < 0 || !iface ||
        std::strcmp(iface, "org.bluez.Device1") != 0)
      return 0;
    Props p;
    if (ReadProps(m, &p) < 0) return 0;
    for (auto &d : t->dev_) {
      if (d.path != path) continue;
      if (p.count("Connected") && !p["Connected"].b) {
        d.connected = false;
        d.resolved = false;
        if (d.st == Dev::UP || d.st == Dev::ACQUIRE || d.st == Dev::RESOLVE) t->Down(d);
      }
    }
    t->Merge(path, p);
    return 0;
  }

  static int OnAdded(sd_bus_message *m, void *userdata, sd_bus_error *) {
    auto *t = static_cast<BluezTransport *>(userdata);
    const char *path = nullptr;
    if (sd_bus_message_read_basic(m, 'o', &path) < 0 || !path) return 0;
    Interfaces ifaces;
    if (ReadInterfaces(m, &ifaces) < 0) return 0;
    auto it = ifaces.find("org.bluez.Device1");
    if (it != ifaces.end()) t->Merge(path, it->second);
    return 0;
  }

  Events *events_ = nullptr;
  sd_bus *bus_ = nullptr;
  sd_bus_slot *match_props_ = nullptr, *match_added_ = nullptr, *agent_slot_ = nullptr;
  std::string adapter_, wanted_adapter_;
  int wake_fd_ = -1;
  int timeout_ms_ = 30000;
  int64_t deadline_ = 0;
  int64_t ambiguous_until_ = 0;
  bool discovering_ = false;
  bool agent_ = false;
  int pairable_before_ = -1;
  Dev dev_[2];
  std::set<std::string> known_;
  std::vector<Candidate> candidates_[2];
  std::map<std::string, Props> devices_;
  std::set<std::string> rejected_;
};

const sd_bus_vtable BluezTransport::kAgentVtable[] = {
    SD_BUS_VTABLE_START(0),
    SD_BUS_METHOD("Release", "", "", &BluezTransport::AgentOk, SD_BUS_VTABLE_UNPRIVILEGED),
    SD_BUS_METHOD("RequestPinCode", "o", "s", &BluezTransport::AgentReject, SD_BUS_VTABLE_UNPRIVILEGED),
    SD_BUS_METHOD("DisplayPinCode", "os", "", &BluezTransport::AgentOk, SD_BUS_VTABLE_UNPRIVILEGED),
    SD_BUS_METHOD("RequestPasskey", "o", "u", &BluezTransport::AgentReject, SD_BUS_VTABLE_UNPRIVILEGED),
    SD_BUS_METHOD("DisplayPasskey", "ouq", "", &BluezTransport::AgentOk, SD_BUS_VTABLE_UNPRIVILEGED),
    SD_BUS_METHOD("RequestConfirmation", "ou", "", &BluezTransport::AgentAccept,
                  SD_BUS_VTABLE_UNPRIVILEGED),
    SD_BUS_METHOD("RequestAuthorization", "o", "", &BluezTransport::AgentAccept,
                  SD_BUS_VTABLE_UNPRIVILEGED),
    SD_BUS_METHOD("AuthorizeService", "os", "", &BluezTransport::AgentAccept,
                  SD_BUS_VTABLE_UNPRIVILEGED),
    SD_BUS_METHOD("Cancel", "", "", &BluezTransport::AgentOk, SD_BUS_VTABLE_UNPRIVILEGED),
    SD_BUS_VTABLE_END};
#endif

// ---------------------------------------------------------------- helpers for Arm

int IndexOf(const Arm *arm) { return arm == &left ? 0 : 1; }

bool Ready(int arm) { return g && g->arms[arm].present; }

std::pair<int, std::vector<uint8_t>> Control(int arm, uint8_t op, std::vector<uint8_t> body) {
  if (!Ready(arm)) return {-1, {}};
  auto job = std::make_shared<ControlJob>();
  job->arm = arm;
  job->op = op;
  job->body = std::move(body);
  auto future = job->reply.get_future();
  {
    std::lock_guard<std::mutex> lock(g->core->mutex);
    g->core->controls.push_back(job);
  }
  g->core->transport->Wake();
  if (future.wait_for(std::chrono::seconds(6)) != std::future_status::ready) return {-1, {}};
  return future.get();
}

// Waits until the arm has run every command queued so far, however long that
// takes while its window keeps moving: false after 15 s without progress (the
// link stays down, say) or after 10 minutes in all.
bool WaitIdle(int arm) {
  using clock = std::chrono::steady_clock;
  Core *core = g->core.get();
  Link &l = core->links[arm];
  uint64_t want = l.emitted;
  auto drained = [&] { return core->Settled(arm, want); };
  auto end = clock::now() + std::chrono::minutes(10), stall = clock::now() + std::chrono::seconds(15);
  uint32_t moved = l.moved;
  std::unique_lock<std::mutex> lock(core->mutex);
  while (!drained()) {
    core->cv.wait_for(lock, std::chrono::milliseconds(500), drained);
    auto now = clock::now();
    if (l.moved != moved) {
      moved = l.moved;
      stall = now + std::chrono::seconds(15);
    }
    if (!drained() && (now > stall || now > end)) return false;
  }
  return true;
}

// Host code must leave the library's own memory alone.
bool Outside(int arm, uint64_t address, uint64_t n, const char *what) {
  if (address + n <= kHostTop) return true;
  char text[120];
  std::snprintf(text, sizeof(text), "%s refused: the library's own memory (from 0x%x)", what, unsigned(kHostTop));
  RecordError(arm, text);
  return false;
}

std::vector<uint32_t> Decode(std::string_view s) {
  std::vector<uint32_t> out;
  for (size_t i = 0; i < s.size();) out.push_back(device::NextCodepoint(s, i));
  return out;
}

} // namespace

// ---------------------------------------------------------------- Arm

void Arm::WriteArena(Address address, std::span<const uint8_t> bytes) {
  std::lock_guard<std::mutex> lock(device::Lock());
  int i = IndexOf(this);
  if (!Ready(i) || !Outside(i, address, bytes.size(), "WRITE")) return;
  if (uint64_t(address) + bytes.size() > kArenaBytes) return RecordError(i, "WRITE refused: arena (error 3)");
  Upload(i, address, bytes.data(), uint32_t(bytes.size()));
}

void Arm::WriteArenaRLE(Address address, uint16_t out_length, std::span<const uint8_t> packbits) {
  std::lock_guard<std::mutex> lock(device::Lock());
  int i = IndexOf(this);
  if (!Ready(i) || !Outside(i, address, out_length, "WRITE_RLE")) return;
  if (7 + packbits.size() <= size_t(g->cmd_limit)) {
    std::vector<uint8_t> v;
    PutWriteRle(v, address, out_length, packbits.data(), packbits.size());
    Submit(i, ToCmd(v), "WRITE_RLE");
    return;
  }
  if (!vm_packbits_ok(packbits.data(), uint32_t(packbits.size()), out_length))
    return RecordError(i, "WRITE_RLE refused: format (error 4)");
  if (uint64_t(address) + out_length > kArenaBytes) return RecordError(i, "WRITE_RLE refused: arena (error 3)");
  std::vector<uint8_t> plain(out_length);
  vm_packbits_decode(packbits.data(), uint32_t(packbits.size()), plain.data());
  Upload(i, address, plain.data(), out_length);
}

void Arm::CopyArena(Address src, Address dst, uint32_t n) {
  std::lock_guard<std::mutex> lock(device::Lock());
  int i = IndexOf(this);
  if (Ready(i) && Outside(i, dst, n, "COPY")) HostOp(i, OP_COPY, {dst, src, n}, "COPY");
}

void Arm::Unpack(Address dst, Address src, uint32_t n, uint32_t out_length) {
  std::lock_guard<std::mutex> lock(device::Lock());
  int i = IndexOf(this);
  if (Ready(i) && Outside(i, dst, out_length, "UNPACK")) HostOp(i, OP_UNPACK, {dst, src, n, out_length}, "UNPACK");
}

void Arm::Fill(Address dst, uint32_t stride, int32_t x, int32_t y, uint32_t w, uint32_t h, U8 level) {
  std::lock_guard<std::mutex> lock(device::Lock());
  int i = IndexOf(this);
  if (Ready(i))
    HostOp(i, OP_FILL, {dst, stride, uint32_t(x), uint32_t(y), w, h, level}, "FILL");
}

void Arm::Blit(Address src, uint32_t src_stride, int32_t src_x, int32_t src_y, Address dst,
               uint32_t dst_stride, int32_t dst_x, int32_t dst_y, uint32_t w, uint32_t h, Mode mode,
               U8 level) {
  std::lock_guard<std::mutex> lock(device::Lock());
  int i = IndexOf(this);
  if (Ready(i))
    HostOp(i, OP_BLIT,
           {src, src_stride, uint32_t(src_x), uint32_t(src_y), dst, dst_stride, uint32_t(dst_x),
            uint32_t(dst_y), w, h, uint32_t(mode), level},
           "BLIT");
}

void Arm::BlitCompressed(Address src, uint32_t src_stride, int32_t src_x, int32_t src_y, Address dst,
                         uint32_t dst_stride, int32_t dst_x, int32_t dst_y, uint32_t w, uint32_t h,
                         Mode mode, U8 level) {
  std::lock_guard<std::mutex> lock(device::Lock());
  int i = IndexOf(this);
  if (Ready(i))
    HostOp(i, OP_BLIT,
           {src, src_stride, uint32_t(src_x), uint32_t(src_y), dst, dst_stride, uint32_t(dst_x),
            uint32_t(dst_y), w, h, uint32_t(mode) | 4, level},
           "BLIT");
}

void Arm::GlyphRunCompressed(Address font, Address dst, uint32_t dst_stride, int32_t x, int32_t y,
                             Mode mode, std::span<const U8> varints, U8 level) {
  std::lock_guard<std::mutex> lock(device::Lock());
  int i = IndexOf(this);
  if (!Ready(i)) return;
  if (varints.size() > kTextBytes) return RecordError(i, "GLYPHS refused: more than 256 bytes of glyphs");
  Cmd glyph = Instruction(i, OP_GLYPHS,
                          {font, dst, dst_stride, uint32_t(x), uint32_t(y), uint32_t(mode), level, kText,
                           uint32_t(varints.size())});
  const size_t step = size_t(g->cmd_limit - 5);
  for (size_t pos = 0; pos < varints.size(); pos += step)
    Submit(i, WriteCmd(kText + Address(pos), varints.data() + pos, std::min(step, varints.size() - pos)),
           "WRITE");
  Submit(i, glyph, "GLYPHS");
}

void Arm::Draw(Address src, uint32_t src_stride, int32_t src_x, int32_t src_y, int32_t dst_x, int32_t dst_y,
               uint32_t w, uint32_t h) {
  std::lock_guard<std::mutex> lock(device::Lock());
  int i = IndexOf(this);
  if (Ready(i))
    HostOp(i, OP_DRAW,
           {src, src_stride, uint32_t(src_x), uint32_t(src_y), uint32_t(dst_x), uint32_t(dst_y), w, h},
           "DRAW");
}

void Arm::Line(Address dst, uint32_t stride, int32_t x0, int32_t y0, int32_t x1, int32_t y1, U8 level) {
  std::lock_guard<std::mutex> lock(device::Lock());
  int i = IndexOf(this);
  if (Ready(i))
    HostOp(i, OP_LINE, {dst, stride, uint32_t(x0), uint32_t(y0), uint32_t(x1), uint32_t(y1), level}, "LINE");
}

void Arm::Video(Address bitmap, uint32_t stride, uint32_t w, uint32_t h, int32_t x, int32_t y, U8 level,
                Address src, uint32_t n, Address state) {
  std::lock_guard<std::mutex> lock(device::Lock());
  int i = IndexOf(this);
  if (Ready(i))
    HostOp(i, OP_VIDEO, {bitmap, stride, w, h, uint32_t(x), uint32_t(y), level, src, n, state}, "VIDEO");
}

void Arm::WaitUntil(uint32_t device_time_us) {
  std::lock_guard<std::mutex> lock(device::Lock());
  int i = IndexOf(this);
  if (Ready(i)) HostOp(i, OP_SLEEP_UNTIL, {device_time_us}, "SLEEP_UNTIL");
}

void Arm::SetPanel(bool on) { SysWrite(PROP_PANEL, on ? 1 : 0); }

void Arm::SysWrite(Property property, uint32_t value) {
  std::lock_guard<std::mutex> lock(device::Lock());
  int i = IndexOf(this);
  if (Ready(i)) HostOp(i, OP_SYS_WRITE, {property, value}, "SYS_WRITE");
}

void Arm::Fence(uint32_t token) { Notify(token, 0); }

void Arm::Notify(uint32_t a, uint32_t b) {
  std::lock_guard<std::mutex> lock(device::Lock());
  int i = IndexOf(this);
  if (Ready(i)) HostOp(i, OP_NOTIFY, {a, b}, "NOTIFY");
}

void Arm::Raise(Trigger trigger) {
  std::lock_guard<std::mutex> lock(device::Lock());
  int i = IndexOf(this);
  if (Ready(i)) HostOp(i, OP_RAISE, {trigger}, "RAISE");
}

// The mirror runs the stored code too, but it may read values only the arm
// has, so the packet is sent whatever the mirror makes of it.
void Arm::Call(Address code) {
  std::lock_guard<std::mutex> lock(device::Lock());
  int i = IndexOf(this);
  if (!Ready(i)) return;
  Cmd c = Ins(OP_CALL, {}, code);
  Mirror &m = g->arms[i].mirror;
  m.vm.safe = SafeMode(i) ? 1 : 0;
  m.Run(c.b.data(), c.size);
  Emit(i, c);
}

void Arm::Run(std::span<const uint8_t> host_code) {
  std::lock_guard<std::mutex> lock(device::Lock());
  int i = IndexOf(this);
  if (!Ready(i)) return;
  if (host_code.empty() || host_code.size() > size_t(g->cmd_limit))
    return RecordError(i, "Run refused: the code does not fit a packet");
  Mirror &m = g->arms[i].mirror;
  std::vector<uint8_t> arena(m.arena, m.arena + kArenaBytes), panel = m.panel;
  struct vm saved = m.vm;
  bool on = m.panel_on, standby = m.standby;
  uint32_t brightness = m.brightness;
  m.vm.safe = SafeMode(i) ? 1 : 0;
  int code = m.Run(host_code.data(), host_code.size());
  if (code) {
    std::memcpy(m.arena, arena.data(), kArenaBytes);
    m.panel = std::move(panel);
    m.vm = saved;
    m.panel_on = on;
    m.standby = standby;
    m.brightness = brightness;
    char text[160];
    std::snprintf(text, sizeof(text), "Run refused: %s at %s (error %d)", CodeName(code), OpName(m.error_op), code);
    return RecordError(i, text);
  }
  Cmd c;
  c.Add(host_code.data(), host_code.size());
  Emit(i, c);
}

bool Arm::Load(const Program &program) {
  std::lock_guard<std::mutex> lock(device::Lock());
  int i = IndexOf(this);
  if (!Ready(i)) return false;
  std::string error;
  if (!program.Valid(&error)) {
    RecordError(i, "Load refused: " + error);
    return false;
  }
  std::vector<uint8_t> pool, table;
  for (uint32_t v : program.pool) Put32(pool, v);
  for (Address a : program.table) Put32(table, a);
  if (!Outside(i, program.pool_address, pool.size(), "Load") ||
      !Outside(i, program.code_address, program.code.size(), "Load") ||
      !Outside(i, program.table_address, 64, "Load"))
    return false;
  for (auto &c : program.changes)
    if (!Outside(i, c.first, c.second, "Load")) return false;
  Upload(i, program.pool_address, pool.data(), uint32_t(pool.size()));
  Upload(i, program.code_address, program.code.data(), uint32_t(program.code.size()));
  Upload(i, program.table_address, table.data(), uint32_t(table.size()));
  auto &changes = g->arms[i].mirror.changes;
  changes.push_back({0, kHostBottom});
  changes.insert(changes.end(), program.changes.begin(), program.changes.end());
  return true;
}

bool Arm::Start(Address table) {
  int i = IndexOf(this);
  if (!Ready(i) || !WaitIdle(i)) return false;
  std::vector<uint8_t> body;
  Put32(body, table);
  auto r = Control(i, CTRL_VM_START, body);
  if (r.first > 0) RecordError(i, std::string("VM_START refused: ") + CodeName(r.first));
  return r.first == 0;
}

bool Arm::Stop() { return Control(IndexOf(this), CTRL_VM_STOP, {}).first == 0; }

bool Arm::Status(VmStatus *s) {
  auto r = Control(IndexOf(this), CTRL_VM_STATUS, {});
  if (r.first != 0 || r.second.size() < 32) return false;
  const uint8_t *b = r.second.data();
  if (s)
    *s = VmStatus{b[0],          b[1],          Get16(b + 2),  Get32(b + 4), Get32(b + 8),  b[12],
                  b[13],         b[14],         b[15],         Get32(b + 16), Get32(b + 20), Get16(b + 24),
                  Get16(b + 26), Get16(b + 29), b[31],         (b[28] & 1) != 0};
  return true;
}

bool Arm::Props(std::span<const uint8_t> ids, std::span<uint32_t> values) {
  if (ids.empty() || ids.size() > 12 || values.size() < ids.size()) return false;
  auto r = Control(IndexOf(this), CTRL_PROPS, std::vector<uint8_t>(ids.begin(), ids.end()));
  if (r.first != 0 || r.second.size() < 4 * ids.size()) return false;
  for (size_t k = 0; k < ids.size(); k++) values[k] = Get32(r.second.data() + 4 * k);
  return true;
}

bool Arm::Prop(Property id, uint32_t *value) {
  uint8_t ids[1] = {id};
  uint32_t v[1] = {0};
  if (!Props(ids, v)) return false;
  if (value) *value = v[0];
  return true;
}

bool Arm::ReportInputs(uint16_t triggers, uint16_t *in_force) {
  int i = IndexOf(this);
  if (!Ready(i)) return false;
  g->core->links[i].inputs = triggers;
  std::vector<uint8_t> body;
  Put16(body, triggers);
  auto r = Control(i, CTRL_EVENTS, body);
  if (r.first != 0 || r.second.size() < 2) return false;
  if (in_force) *in_force = Get16(r.second.data());
  return true;
}

bool Arm::NextEvent(Event *event, int timeout_ms) {
  if (!g) return false;
  Core *core = g->core.get();
  Link &l = core->links[IndexOf(this)];
  std::unique_lock<std::mutex> lock(core->mutex);
  core->cv.wait_for(lock, std::chrono::milliseconds(std::max(timeout_ms, 0)),
                    [&] { return !l.events.empty() || core->stop; });
  if (l.events.empty()) return false;
  if (event) *event = l.events.front();
  l.events.pop_front();
  return true;
}

int64_t Arm::Alloc(uint32_t bytes) { return g1::Alloc(bytes); }

void Arm::Free(Address address) { g1::Free(address); }

uint32_t Arm::DeviceTime(int64_t in_us) {
  if (!g) return 0;
  return uint32_t(NowUs() + in_us) + g->core->links[IndexOf(this)].offset.load();
}

void Arm::Disconnect(int ms) {
  int i = IndexOf(this);
  if (!Ready(i)) return;
  g->core->hold_ms[i] = std::max(ms, 1);
  g->core->transport->Wake();
}

Arm::State Arm::GetState() {
  State s{};
  if (!g) return s;
  int i = IndexOf(this);
  Link &l = g->core->links[i];
  s.present = g->arms[i].present;
  s.connected = l.ready;
  int fw = l.firmware;
  s.firmware_major = fw >> 16;
  s.firmware_minor = (fw >> 8) & 0xFF;
  s.firmware_patch = fw & 0xFF;
  s.epoch = l.epoch_seen;
  s.max_packet = l.max_packet_seen;
  s.interval_us = l.interval_seen;
  s.clock_error_us = l.clock_error_us;
  s.idle = l.idle_profile;
  s.queued = l.queued + int(g->core->queues[i].size_approx());
  s.packets = l.packets;
  s.resent = l.resent;
  s.reconnects = l.reconnects;
  s.epoch_changes = l.epoch_changes;
  s.errors = l.errors;
  s.shown_update = l.shown_update;
  s.shown_us = l.shown_us;
  s.safe_mode = l.safe;
  s.encrypted = l.encrypted;
  s.in_case = l.in_case;
  s.microphone = l.microphone;
  s.triggers = l.triggers;
  s.events_lost = l.events_lost;
  {
    std::lock_guard<std::mutex> api(device::Lock());
    s.panel_on = g->arms[i].mirror.panel_on;
  }
  std::lock_guard<std::mutex> lock(g->core->mutex);
  s.last_error = l.last_error;
  s.address = l.address;
  return s;
}

bool Arm::SetBrightness(int level) {
  if (level < 0 || level > 42 || !Ready(IndexOf(this))) return false;
  SysWrite(PROP_BRIGHTNESS, uint32_t(level));
  return true;
}

bool Arm::Battery(int *millivolts, int *percent, bool *charging) {
  uint8_t ids[3] = {PROP_BATT_MV, PROP_BATT_PCT, PROP_CHARGE};
  uint32_t v[3] = {0, 0, 0};
  if (!Props(ids, v)) return false;
  if (millivolts) *millivolts = v[0] == UNKNOWN ? -1 : int(v[0]);
  if (percent) *percent = v[1] == UNKNOWN ? -1 : int(v[1]);
  if (charging) *charging = v[2] == 1;
  return true;
}

bool Arm::PanelCrc(int y0, int rows, uint32_t *arm_crc, uint32_t *expected_crc) {
  int i = IndexOf(this);
  if (!Ready(i) || y0 < 0 || rows <= 0 || y0 + rows > kPanelH || !WaitIdle(i)) return false;
  std::vector<uint8_t> body;
  Put16(body, uint32_t(y0));
  Put16(body, uint32_t(rows));
  auto r = Control(i, CTRL_PANEL_READ, body);
  if (r.first != 0 || r.second.size() < 6) return false;
  if (arm_crc) *arm_crc = Get32(r.second.data());
  if (expected_crc) {
    std::lock_guard<std::mutex> lock(device::Lock());
    std::vector<uint8_t> expected = ExpectedPanelRows(i, y0, rows);
    *expected_crc = Crc32(0, expected.data(), expected.size());
  }
  return true;
}

bool Arm::ArenaCrc(Address address, uint32_t length, uint32_t *arm_crc, uint32_t *expected_crc) {
  int i = IndexOf(this);
  uint64_t end = length ? uint64_t(address) + length : kArenaBytes;
  if (!Ready(i) || end > kArenaBytes || address > end) return false;
  {
    std::lock_guard<std::mutex> lock(device::Lock());
    if (g->arms[i].mirror.Changed(address, end - address)) return false;
  }
  if (!WaitIdle(i)) return false;
  std::vector<uint8_t> body;
  Put32(body, address);
  Put32(body, length);
  auto r = Control(i, CTRL_ARENA_CRC, body);
  if (r.first != 0 || r.second.size() < 4) return false;
  if (arm_crc) *arm_crc = Get32(r.second.data());
  if (expected_crc) {
    std::lock_guard<std::mutex> lock(device::Lock());
    *expected_crc = Crc32(0, ArmBytes(i) + address, size_t(end - address));
  }
  return true;
}

bool Arm::ReadArena(Address address, std::span<uint8_t> bytes) {
  int i = IndexOf(this);
  if (!Ready(i) || uint64_t(address) + bytes.size() > kArenaBytes || !WaitIdle(i)) return false;
  for (size_t pos = 0; pos < bytes.size(); pos += 16) {
    uint32_t n = uint32_t(std::min<size_t>(16, bytes.size() - pos));
    std::vector<uint8_t> body;
    Put32(body, address + uint32_t(pos));
    Put32(body, n);
    auto r = Control(i, CTRL_ARENA_CRC, body);
    if (r.first != 0 || r.second.size() < 4 + n) return false;
    std::memcpy(bytes.data() + pos, r.second.data() + 4, n);
  }
  return true;
}

// ---------------------------------------------------------------- microphone

// MIC acts on arrival: after what was queued before (a PANEL = 0 would stop it).
bool Arm::MicStart(MicRate rate) {
  int i = IndexOf(this);
  if (!Ready(i)) return false;
  if (!WaitIdle(i)) {
    RecordError(i, "MicStart: the arm did not run what was queued before");
    return false;
  }
#ifndef SCOUTER_G1_MIC
  (void)rate;
  RecordError(i, "MicStart refused: built without SCOUTER_G1_MIC");
  return false;
#else
  Core *core = g->core.get();
  Link::Mic &m = core->links[i].mic;
  int bytes = rate == MIC_16KBPS ? 20 : 40;
  uint32_t panel = UNKNOWN;
  if (!m.on && !Prop(PROP_PANEL, &panel)) {
    RecordError(i, "MicStart: no reply to PROPS");
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(core->mutex);
    if (m.on && m.bytes == bytes) return true;
    if (!m.on) m.panel_before = panel;
    m.memory.assign(lc3_decoder_size(10000, kMicSampleRate) / 8 + 1, 0);
    m.decoder = lc3_setup_decoder(10000, kMicSampleRate, 0, m.memory.data());
    m.bytes = bytes;
    m.next = 0;
    m.frames.clear();
    m.state = MicState{};
    m.on = true;
  }
  auto r = Control(i, CTRL_MIC, {1, uint8_t(rate == MIC_16KBPS ? 1 : 0)});
  bool ok = r.first == 0 && r.second.size() >= 6 && r.second[0] == 1;
  {
    std::lock_guard<std::mutex> lock(core->mutex);
    if (ok) {
      m.started = true;
      m.renewed = NowUs();
      m.bytes = r.second[1];
      m.state.frame_bytes = r.second[1];
      m.state.frames_per_event = r.second[2];
    } else {
      m.on = false;
      m.stopped = MIC_STOP_HOST;
    }
  }
  core->cv.notify_all();
  if (ok) return true;
  if (r.first < 0) Control(i, CTRL_MIC, {0, 0}); // it may have started all the same
  RecordError(i, r.first < 0 ? std::string("MIC: no reply") : std::string("MIC refused: ") + CodeName(r.first));
  return false;
#endif
}

bool Arm::MicStop() {
  int i = IndexOf(this);
  if (!Ready(i)) return false;
  Core *core = g->core.get();
  Link::Mic &m = core->links[i].mic;
  bool restore;
  uint32_t before = m.panel_before;
  {
    std::lock_guard<std::mutex> lock(core->mutex);
    if (m.on) m.stopped = MIC_STOP_HOST;
    m.on = false;
    restore = m.started && (before == 0 || before == 2);
    m.started = false;
    m.frames.clear();
  }
  core->cv.notify_all();
  // Without a session the arm is off: it stops at every disconnection.
  bool ok = Control(i, CTRL_MIC, {0, 0}).first == 0 || !core->links[i].ready;
  std::lock_guard<std::mutex> api(device::Lock());
  if (restore && !g->arms[i].mirror.panel_on) HostOp(i, OP_SYS_WRITE, {VM_P_PANEL, before}, "SYS_WRITE");
  return ok;
}

bool Arm::MicRead(MicFrame *frame, int timeout_ms) {
  if (!g) return false;
  Core *core = g->core.get();
  Link::Mic &m = core->links[IndexOf(this)].mic;
  std::unique_lock<std::mutex> lock(core->mutex);
  core->cv.wait_for(lock, std::chrono::milliseconds(std::max(timeout_ms, 0)),
                    [&] { return !m.frames.empty() || !m.on || core->stop; });
  if (m.frames.empty()) return false;
  if (frame) *frame = m.frames.front();
  m.frames.pop_front();
  return true;
}

MicState Arm::GetMicState() {
  if (!g) return MicState{};
  Core *core = g->core.get();
  Link::Mic &m = core->links[IndexOf(this)].mic;
  std::lock_guard<std::mutex> lock(core->mutex);
  MicState s = m.state;
  s.on = m.on;
  s.stopped = m.stopped;
  return s;
}

// ---------------------------------------------------------------- memory and fonts

int64_t Alloc(uint32_t bytes) {
  std::lock_guard<std::mutex> lock(device::Lock());
  if (!g) return -1;
  int at = g->alloc.Alloc((bytes + 3) / 4);
  free_memory = g->device ? g->device->FreeMemory() : 0;
  return at < 0 ? -1 : 4 * int64_t(at);
}

void Free(Address address) {
  std::lock_guard<std::mutex> lock(device::Lock());
  if (!g) return;
  G1Device::Release(address);
  free_memory = g->device ? g->device->FreeMemory() : 0;
}

Address AddressOf(const gray::Bitmap &bitmap, int *stride) {
  std::lock_guard<std::mutex> lock(device::Lock());
  G1Bitmap *gb = bitmap.pimpl && g ? G1Device::Data(bitmap) : nullptr;
  if (stride) *stride = gb ? int(gb->stride) : 0;
  return gb ? gb->address : 0;
}

Address AddressOf(const Font &font) {
  std::lock_guard<std::mutex> lock(device::Lock());
  if (!font.pimpl || !g) return 0;
  auto *gf = static_cast<G1Font *>(device::DeviceData(const_cast<Font &>(font)));
  return gf ? gf->address : 0;
}

// By position for 0-9, A-Z, a-z, else by codepoint (VM.md §4.1).
std::vector<uint32_t> GlyphIndices(const Font &font, std::string_view text) {
  std::lock_guard<std::mutex> lock(device::Lock());
  std::vector<uint32_t> out;
  if (!font.pimpl) return out;
  std::span<const uint8_t> blob = device::FontBlob(font);
  if (blob.size() < 16) return out;
  const uint8_t *p = blob.data();
  int count = Get16(p + 8);
  if (16 + 16 * size_t(count) > blob.size()) return out;
  auto codepoint = [&](int i) { return Get32(p + 16 + 12 * size_t(count) + 4 * size_t(i)); };
  auto find = [&](uint32_t cp) -> int {
    int slot = cp >= '0' && cp <= '9' ? int(cp - '0')
               : cp >= 'A' && cp <= 'Z' ? int(cp - 'A') + 10
               : cp >= 'a' && cp <= 'z' ? int(cp - 'a') + 36
                                        : -1;
    if (slot >= 0) return slot < count && codepoint(slot) == cp ? slot : -1;
    int lo = std::min(62, count), hi = count - 1;
    while (lo <= hi) {
      int mid = (lo + hi) / 2;
      if (codepoint(mid) == cp) return mid;
      if (codepoint(mid) < cp)
        lo = mid + 1;
      else
        hi = mid - 1;
    }
    return -1;
  };
  int replacement = find(0xFFFD);
  for (uint32_t cp : Decode(text)) {
    int k = find(cp);
    if (k < 0) k = replacement;
    if (k >= 0) out.push_back(uint32_t(k));
  }
  return out;
}

// ---------------------------------------------------------------- the asset store

AssetId AssetIdOf(std::span<const uint8_t> blob) {
  tc_sha256_state_struct s;
  uint8_t digest[TC_SHA256_DIGEST_SIZE];
  tc_sha256_init(&s);
  tc_sha256_update(&s, blob.data(), blob.size());
  tc_sha256_final(digest, &s);
  AssetId id;
  std::memcpy(id.b, digest, sizeof(id.b));
  return id;
}

namespace {

bool Fail(std::string *error, const std::string &text) {
  if (error) *error = text;
  return false;
}

// Waits for the arms' jobs: "" or why one failed.
std::string Wait(std::vector<std::pair<int, std::future<int>>> &jobs, int seconds) {
  std::string why;
  for (auto &j : jobs) {
    bool ready = j.second.wait_for(std::chrono::seconds(seconds)) == std::future_status::ready;
    int r = ready ? j.second.get() : -1;
    if (r) why = std::string("the ") + kArmName[j.first] + " arm: " + (r < 0 ? "no answer" : CodeName(r));
  }
  return why;
}

} // namespace

bool Store(std::span<const uint8_t> blob, uint8_t type, std::string *error) {
  if (!g) return Fail(error, "not initialized");
  if (blob.empty() || blob.size() > kArenaBytes) return Fail(error, "an asset has 1 ... 262144 bytes");
  AssetId id = AssetIdOf(blob);
  std::vector<std::pair<int, std::future<int>>> jobs;
  int64_t staging = -1;
  {
    std::lock_guard<std::mutex> api(device::Lock());
    for (int i = 0; i < 2; i++) {
      if (!g->arms[i].present) continue;
      std::string arm = std::string("the ") + kArmName[i] + " arm";
      if (!g->core->links[i].ready) return Fail(error, arm + " is not connected");
      int have = g->core->HasAsset(i, id);
      if (have < 0) return Fail(error, arm + " has no asset store (a firmware before 0.5.56)");
      if (have > 0) continue;
      if (staging < 0 && (staging = g->alloc.Alloc(uint32_t((blob.size() + 3) / 4))) < 0)
        return Fail(error, "no free arena memory to stage the asset");
      jobs.push_back({i, StagedPut(i, 4 * uint32_t(staging), blob, type, true)});
    }
  }
  std::string why = Wait(jobs, 120);
  if (staging >= 0) {
    std::lock_guard<std::mutex> api(device::Lock());
    G1Device::Release(4 * uint32_t(staging));
  }
  if (!why.empty()) return Fail(error, "not stored: " + why);
  if (error) error->clear();
  return true;
}

std::vector<AssetEntry> Assets(const Arm &arm) {
  std::vector<AssetEntry> out;
  if (!g) return out;
  int i = IndexOf(&arm);
  auto info = Control(i, CTRL_ASSET_INFO, {});
  if (info.first != 0 || info.second.size() < 20 || !(info.second[15] & 1)) return out;
  for (size_t first = 0;;) {
    auto r = Control(i, CTRL_ASSET_LIST, {uint8_t(first)});
    if (r.first != 0 || r.second.size() < 2) return {};
    size_t k = (r.second.size() - 2) / 32;
    for (size_t n = 0; n < k; n++) out.push_back(EntryOf(r.second.data() + 2 + 32 * n));
    first += k;
    if (k == 0 || first >= r.second[0]) break;
  }
  std::lock_guard<std::mutex> lock(g->core->mutex);
  auto &a = g->core->links[i].assets;
  a.store = true;
  a.entries.clear();
  for (auto &e : out) a.entries[e.id] = e;
  return out;
}

bool Forget(AssetId id, std::string *error) {
  if (!g) return Fail(error, "not initialized");
  std::vector<std::pair<int, std::future<int>>> jobs;
  {
    std::lock_guard<std::mutex> api(device::Lock());
    for (int i = 0; i < 2; i++) {
      if (!g->arms[i].present || g->core->HasAsset(i, id) <= 0) continue;
      Link::AssetJob j;
      j.tag = ++g->tags;
      j.op = CTRL_ASSET_DELETE;
      j.id = id;
      jobs.push_back({i, g->core->PushJob(i, std::move(j), true)});
    }
  }
  std::string why = Wait(jobs, 60);
  if (!why.empty()) return Fail(error, "not deleted: " + why);
  if (error) error->clear();
  return true;
}

// ---------------------------------------------------------------- Init / Deinit

InitError Init(GrayConfig *config, std::string *error_message, const Connection *connection) {
  auto fail = [&](InitError e, const std::string &text) {
    if (error_message) *error_message = text;
    return e;
  };
  if (g) return fail(ERROR_BLUETOOTH_ERROR, "already initialized");
  GrayConfig gc = config ? *config : GrayConfig{};
  if (std::abs(gc.dx) > 640 || std::abs(gc.dy) > 240)
    return fail(ERROR_BLUETOOTH_ERROR, "GrayConfig out of range");
  Connection conn = connection ? *connection : Connection{};
  g_log = conn.log;
  bool present[2] = {conn.left != "none", conn.right != "none"};
  if (!present[0] && !present[1]) return fail(ERROR_NO_G1_FOUND, "both arms are \"none\"");
#if defined(SCOUTER_G1_NO_BLUEZ) && !defined(SCOUTER_G1_EMULATOR)
  if (!conn.transport)
    return fail(ERROR_BLUETOOTH_ERROR, "built without BlueZ: Connection::transport is required");
#endif

  auto glob = std::make_unique<Global>();
  g = glob.get();
  Geometry &geo = g->geo;
  geo.width = kPanelW + std::abs(gc.dx);
  geo.height = kPanelH - std::abs(gc.dy);
  geo.ox[0] = std::max(0, -gc.dx);
  geo.oy[0] = std::max(0, -gc.dy);
  geo.ox[1] = std::max(0, gc.dx);
  geo.oy[1] = std::max(0, gc.dy);
  g->keep = conn.keep_sessions;
  g->alloc.Reset(kHostBottom / 4, (conn.keep_sessions ? kHostBottom : kHostTop) / 4); // kept: none is ours
  for (int i = 0; i < 2; i++) {
    g->arms[i].present = present[i];
    g->arms[i].mirror.Init(ArmBytes(i), i);
    g->arms[i].pool.Clear();
    kArms[i]->pimpl = &g->arms[i];
  }
  g->core = std::make_unique<Core>();
  std::unique_ptr<Transport> owned;
#ifdef SCOUTER_G1_EMULATOR
  if (!conn.transport) {
    auto emulated = std::make_unique<EmulatorTransport>(conn.adapter);
    g_emulator = emulated.get();
    owned = std::move(emulated);
  }
#elif !defined(SCOUTER_G1_NO_BLUEZ)
  std::string addresses[2] = {present[0] ? conn.left : "", present[1] ? conn.right : ""};
  if (!conn.transport)
    owned = std::make_unique<BluezTransport>(addresses, conn.timeout_ms, conn.adapter);
#endif
  Core *core = g->core.get();
  bool automatic[2] = {conn.left.empty(), conn.right.empty()};
  Transport *transport = conn.transport ? conn.transport : owned.get();
  core->Launch(transport, std::move(owned), present, automatic);

  auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(conn.timeout_ms);
  InitError result = ERROR_NONE;
  std::string text;
  {
    std::unique_lock<std::mutex> lock(core->mutex);
    auto all_ready = [&] {
      for (int i = 0; i < 2; i++)
        if (present[i] && !core->links[i].ready) return false;
      return true;
    };
    core->cv.wait_until(lock, deadline, [&] {
      return all_ready() || core->finished || core->problem == ERROR_G1_FIRMWARE ||
             core->problem == ERROR_G1_BONDING_ERROR || core->problem == ERROR_NO_G1_FOUND;
    });
    if (!all_ready()) {
      result = core->problem;
      text = core->problem_text;
      if (result == ERROR_NONE) {
        bool found = (!present[0] || core->found[0]) && (!present[1] || core->found[1]);
        result = found ? ERROR_BLUETOOTH_ERROR : ERROR_NO_G1_FOUND;
        text = found ? "the glasses were found but did not become ready in time"
                     : "the glasses were not found in time";
      }
    }
  }
  if (result != ERROR_NONE) {
    core->Shutdown();
#ifdef SCOUTER_G1_EMULATOR
    g_emulator = nullptr;
#endif
    for (auto *a : kArms) a->pimpl = nullptr;
    g = nullptr;
    return fail(result, text.empty() ? "Bluetooth error" : text);
  }
  {
    std::lock_guard<std::mutex> lock(device::Lock());
    int limit = SCOUTER_G1_MAX_PACKET;
    for (int i = 0; i < 2; i++)
      if (present[i]) limit = std::min(limit, core->links[i].max_packet_seen.load());
    g->cmd_limit = limit - 2;
    g->device = std::make_unique<G1Device>();
    device::Attach(g->device.get(), geo.width, geo.height, 16);
  }
  core->started = true;
  glob.release();
  if (error_message) error_message->clear();
  return ERROR_NONE;
}

void Deinit() {
  if (!g) return;
  Core *core = g->core.get();
  {
    std::lock_guard<std::mutex> api(device::Lock());
    core->ending = true;
    for (int i = 0; i < 2; i++) core->EndMatch(i);
  }
  {
    std::unique_lock<std::mutex> lock(core->mutex);
    uint64_t want[2] = {core->links[0].emitted, core->links[1].emitted};
    core->cv.wait_for(lock, std::chrono::seconds(3), [&] {
      for (int i = 0; i < 2; i++) {
        Link &l = core->links[i];
        if (l.present && l.ready && !core->Settled(i, want[i])) return false;
      }
      return true;
    });
    // the background puts of kept fonts and bitmaps finish (their staging is the objects themselves)
    auto end = std::chrono::steady_clock::now() + std::chrono::seconds(30);
    while (std::chrono::steady_clock::now() < end &&
           ((core->links[0].ready && !core->links[0].asset_jobs.empty()) ||
            (core->links[1].ready && !core->links[1].asset_jobs.empty())))
      core->cv.wait_for(lock, std::chrono::milliseconds(200));
  }
  {
    std::lock_guard<std::mutex> lock(device::Lock());
    device::Detach();
  }
  core->Shutdown();
#ifdef SCOUTER_G1_EMULATOR
  g_emulator = nullptr;
#endif
  for (auto *a : kArms) a->pimpl = nullptr;
  Global *old = g;
  g = nullptr;
  delete old;
}

#ifdef SCOUTER_G1_EMULATOR
namespace emulator {

const uint8_t *Panel(int arm) {
  return g_emulator && (arm == 0 || arm == 1) ? g_emulator->Panel(arm) : nullptr;
}

void SetLoss(double write_loss, double notify_loss) {
  if (g_emulator) g_emulator->SetLoss(write_loss, notify_loss);
}

void Disconnect(int arm, int ms) {
  if (g_emulator && (arm == 0 || arm == 1)) g_emulator->DropLink(arm, ms);
}

void Reboot(int arm) {
  if (g_emulator && (arm == 0 || arm == 1)) g_emulator->Reboot(arm);
}

void SetProperty(int arm, Property id, uint32_t value) {
  if (g_emulator && (arm == 0 || arm == 1)) g_emulator->SetProperty(arm, id, value);
}

void SetMicSignal(int arm, double hz, double peak) {
  if (g_emulator && (arm == 0 || arm == 1)) g_emulator->SetMicSignal(arm, hz, peak);
}

void HoldMicRenewals(bool hold) { g_hold_renewals = hold; }

void SetAssetStore(int arm, bool available) {
  if (g_emulator && (arm == 0 || arm == 1)) g_emulator->SetAssetStore(arm, available);
}

void NewHost(bool keep_sessions) {
  if (!g_emulator || !g) return;
  g->keep = keep_sessions;
  for (int i = 0; i < 2; i++) {
    if (!g->arms[i].present) continue;
    g->core->forget[i] = true;
    g_emulator->DropLink(i, 300);
  }
}

} // namespace emulator
#endif

} // namespace scouter_kit::g1
