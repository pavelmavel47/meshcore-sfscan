#ifdef SFSCAN_ENABLED

#define RADIOLIB_STATIC_ONLY 1
#include "SFScanSX1262Wrapper.h"
#include <math.h>

static const uint8_t sfscan_pattern[] = { SFSCAN_PATTERN };
#define SFSCAN_PATTERN_LEN  (sizeof(sfscan_pattern) / sizeof(sfscan_pattern[0]))

#define CAD_WATCHDOG_MS  600   // CAD should finish in < 150ms even on SF12/BW62.5
#define TX_GAP_EST_MS    10    // estimated gap between the per-SF transmissions
#define RX_MARGIN_MS     30

static volatile bool sfscan_irq = false;

static
#if defined(ESP8266) || defined(ESP32)
  ICACHE_RAM_ATTR
#endif
void sfscanSetFlag(void) {
  sfscan_irq = true;
}

static uint32_t cad_started_at = 0;

SFScanSX1262Wrapper::SFScanSX1262Wrapper(CustomSX1262& radio, mesh::MainBoard& board)
  : CustomSX1262Wrapper(radio, board) {
  _state = S_IDLE;
  _freq = LORA_FREQ; _bw = LORA_BW; _home_sf = LORA_SF;
#ifdef LORA_CR
  _cr = LORA_CR;
#else
  _cr = 5;
#endif
  _cur_sf = 0;
  _pattern_idx = 0;
  _rx_start = _rx_window = 0;
  memset(_heard_at, 0, sizeof(_heard_at));
  memset(_rx_count, 0, sizeof(_rx_count));
  memset(_seen, 0, sizeof(_seen));
  _seen_next = 0;
  _last_rx_sf = LORA_SF;
  _last_snr = _last_rssi = 0;
  _rx_airtime_pending = false;
  _tx_len = 0; _tx_num = _tx_idx = 0;
  n_recv_errors = 0;

  // distinct SFs of the scan pattern
  _num_scan_sfs = 0;
  for (size_t i = 0; i < SFSCAN_PATTERN_LEN; i++) {
    uint8_t sf = sfscan_pattern[i];
    if (sf < SFSCAN_MIN_SF || sf > SFSCAN_MAX_SF) continue;
    bool dup = false;
    for (uint8_t j = 0; j < _num_scan_sfs; j++) if (_scan_sfs[j] == sf) dup = true;
    if (!dup && _num_scan_sfs < SFSCAN_NUM_SF) _scan_sfs[_num_scan_sfs++] = sf;
  }
}

// LoRa time-on-air (Semtech AN1200.13), explicit header, CRC on
uint32_t SFScanSX1262Wrapper::airtimeMs(uint8_t sf, int len) const {
  float tsym = (float)(1UL << sf) / _bw;             // ms (bw in kHz)
  int de = tsym >= 16.0f ? 1 : 0;                    // low data rate optimisation
  int cr = (_cr >= 5 && _cr <= 8) ? _cr - 4 : 1;
  int num = 8 * len - 4 * sf + 28 + 16;
  int den = 4 * (sf - 2 * de);
  int n_payload = 8 + max(0, (int)ceilf((float)num / den)) * (cr + 4);
  float t = (preambleLengthForSF(sf) + 4.25f) * tsym + n_payload * tsym;
  return (uint32_t)ceilf(t);
}

// FNV-1a over payload type + payload. Both stay the same when a packet is forwarded
// (only the path changes), so this identifies "the same packet" across hops.
bool SFScanSX1262Wrapper::payloadHash(const uint8_t* raw, int len, uint32_t& hash) {
  if (len < 2) return false;
  int i = 1;
  uint8_t route = raw[0] & PH_ROUTE_MASK;
  if (route == ROUTE_TYPE_TRANSPORT_FLOOD || route == ROUTE_TYPE_TRANSPORT_DIRECT) i += 4;
  if (i >= len) return false;
  uint8_t path_len = raw[i++];
  if ((path_len >> 6) == 3) return false;
  i += (path_len & 63) * ((path_len >> 6) + 1);
  if (i > len) return false;

  uint32_t h = 2166136261UL;
  h = (h ^ ((raw[0] >> PH_TYPE_SHIFT) & PH_TYPE_MASK)) * 16777619UL;
  for (; i < len; i++) h = (h ^ raw[i]) * 16777619UL;
  hash = h;
  return true;
}

void SFScanSX1262Wrapper::applySF(uint8_t sf) {
  if (sf == _cur_sf) return;
  sx()->standby();
  sx()->setSpreadingFactor(sf);
  updatePreamble(sf);   // sets preamble length (same rule as all MeshCore nodes)

  float tsym = (float)(1UL << sf) / _bw;
  uint32_t pre_ms = (uint32_t)ceilf((preambleLengthForSF(sf) + 12.25f) * tsym);
  uint32_t total_ms = airtimeMs(sf, MAX_TRANS_UNIT);
  sx()->setPreambleMillis(pre_ms);
  sx()->setMaxPayloadMillis(total_ms > pre_ms ? total_ms - pre_ms : 4000);
  _cur_sf = sf;
}

void SFScanSX1262Wrapper::begin() {
  _radio->setPacketReceivedAction(sfscanSetFlag);   // DIO1: RX done/timeout, TX done and CAD done
  _noise_floor = 0;
  _threshold = 0;
  _cad_enabled = false;
  _num_floor_samples = 0;
  _floor_sample_sum = 0;
  sfscan_irq = false;
  _cur_sf = 0;
  applySF(_home_sf);
  _state = S_IDLE;
}

void SFScanSX1262Wrapper::setParams(float freq, float bw, uint8_t sf, uint8_t cr) {
  _freq = freq; _bw = bw; _home_sf = sf; _cr = cr;
  sx()->standby();
  sx()->setFrequency(freq);
  sx()->setBandwidth(bw);
  sx()->setCodingRate(cr);
  _cur_sf = 0;           // force re-apply of SF dependent settings
  applySF(sf);
  sfscan_irq = false;
  if (_state != S_TX) { _state = S_IDLE; startCAD(); }
}

void SFScanSX1262Wrapper::startCAD() {
  uint8_t sf = sfscan_pattern[_pattern_idx];
  if (sf < SFSCAN_MIN_SF || sf > SFSCAN_MAX_SF) sf = _home_sf;
  applySF(sf);
  sfscan_irq = false;
  // 2-symbol CAD (RadioLib default is 4) halves the time per SF; detPeak values per
  // Semtech AN1200.48 recommendations for 2 symbols, SF7..SF12
  static const uint8_t det_peak_2sym[SFSCAN_NUM_SF] = { 22, 22, 23, 24, 25, 28 };
  ChannelScanConfig_t cfg;
  cfg.cad.symNum = RADIOLIB_SX126X_CAD_ON_2_SYMB;
  cfg.cad.detPeak = det_peak_2sym[sf - SFSCAN_MIN_SF];
  cfg.cad.detMin = RADIOLIB_SX126X_CAD_PARAM_DET_MIN;
  cfg.cad.exitMode = RADIOLIB_SX126X_CAD_GOTO_STDBY;
  cfg.cad.timeout = 0;
  cfg.cad.irqFlags = RADIOLIB_IRQ_CAD_DEFAULT_FLAGS;
  cfg.cad.irqMask = RADIOLIB_IRQ_CAD_DEFAULT_MASK;
  int16_t err = sx()->startChannelScan(cfg);
  if (err != RADIOLIB_ERR_NONE) {
    MESH_DEBUG_PRINTLN("SFScan: startChannelScan(%d) failed on SF%d", err, sf);
    _pattern_idx = (_pattern_idx + 1) % SFSCAN_PATTERN_LEN;
    _state = S_IDLE;
    return;
  }
  cad_started_at = millis();
  _state = S_CAD;
}

void SFScanSX1262Wrapper::startRxLocked() {
  float tsym = (float)(1UL << _cur_sf) / _bw;
  // CAD fired somewhere inside the preamble: wait for the rest of it + header.
  // Single RX with hardware timeout: the SX126x stops this timer once a header is
  // detected, so it only fires on a false CAD alarm. DIO1 rises either way.
  _rx_window = (uint32_t)ceilf((preambleLengthForSF(_cur_sf) + 12.25f) * tsym) + RX_MARGIN_MS;

  sfscan_irq = false;
  uint32_t timeout_raw = (uint32_t)(sx()->calculateRxTimeout((RadioLibTime_t)_rx_window * 1000UL));
  // DIO1 on RX done, header error and timeout -> the MCU is woken for every outcome.
  // (RADIOLIB_IRQ_RX_DEFAULT_MASK is RX_DONE only: without TIMEOUT in the mask a false
  // CAD alarm left DIO1 low, so a light-sleeping MCU stayed asleep for up to 30 s
  // while the radio sat in standby = deaf.)
  int16_t err = sx()->PhysicalLayer::startReceive(timeout_raw,
      RADIOLIB_IRQ_RX_DEFAULT_FLAGS | (1UL << RADIOLIB_IRQ_PREAMBLE_DETECTED),
      RADIOLIB_IRQ_RX_DEFAULT_MASK | (1UL << RADIOLIB_IRQ_TIMEOUT) | (1UL << RADIOLIB_IRQ_HEADER_ERR), 0);
  if (err != RADIOLIB_ERR_NONE) {
    MESH_DEBUG_PRINTLN("SFScan: startReceive(%d) failed on SF%d", err, _cur_sf);
    resumeScan();
    return;
  }
  _rx_start = millis();
  _state = S_RX;
}

// Restart the scan right away (not via S_IDLE + next loop()): the main loop may put
// the MCU into light sleep before the next loop(), and an idle radio never raises
// DIO1, so the node would be deaf until the 30 s wake-up timer.
void SFScanSX1262Wrapper::resumeScan() {
  sx()->standby();
  sfscan_irq = false;
  _pattern_idx = (_pattern_idx + 1) % SFSCAN_PATTERN_LEN;
  _state = S_IDLE;
  startCAD();
}

// After an ESP32 light sleep the edge interrupt can be missed; DIO1 stays HIGH until the
// IRQ status is cleared, so polling the pin is a reliable fallback.
void SFScanSX1262Wrapper::pollDio1() {
#ifdef P_LORA_DIO_1
  if (!sfscan_irq && _state != S_IDLE && digitalRead(P_LORA_DIO_1) == HIGH) sfscan_irq = true;
#endif
}

void SFScanSX1262Wrapper::loop() {
  _rx_airtime_pending = false;   // only valid right after recvRaw() in the same Dispatcher cycle
  pollDio1();

  switch (_state) {
    case S_TX:
      return;

    case S_IDLE:
      startCAD();
      return;

    case S_CAD:
      if (sfscan_irq) {
        sfscan_irq = false;
        int16_t res = sx()->getChannelScanResult();
        if (res == RADIOLIB_LORA_DETECTED) {
          startRxLocked();       // stay on this SF and receive
        } else {
          _pattern_idx = (_pattern_idx + 1) % SFSCAN_PATTERN_LEN;
          startCAD();
        }
      } else if (millis() - cad_started_at > CAD_WATCHDOG_MS) {
        MESH_DEBUG_PRINTLN("SFScan: CAD watchdog on SF%d", _cur_sf);
        resumeScan();
      }
      return;

    case S_RX:
      if (sfscan_irq) {
        uint32_t irq = sx()->getIrqFlags();
        if ((irq & RADIOLIB_SX126X_IRQ_TIMEOUT) ||                    // false CAD alarm
            ((irq & RADIOLIB_SX126X_IRQ_HEADER_ERR) && !(irq & RADIOLIB_SX126X_IRQ_RX_DONE))) {
          if (irq & RADIOLIB_SX126X_IRQ_HEADER_ERR) n_recv_errors++;
          resumeScan();
        }
        return;                                     // else: packet ready for recvRaw()
      }
      if (sx()->isReceiving()) return;              // preamble / header in progress
      if (millis() - _rx_start > _rx_window + 2 * RX_MARGIN_MS) {   // software backup
        resumeScan();
      }
      return;
  }
}

int SFScanSX1262Wrapper::recvRaw(uint8_t* bytes, int sz) {
  pollDio1();
  if (_state != S_RX || !sfscan_irq) return 0;
  sfscan_irq = false;

  uint32_t irq = sx()->getIrqFlags();
  if (!(irq & RADIOLIB_SX126X_IRQ_RX_DONE)) {   // timeout / header error: nothing received
    resumeScan();
    return 0;
  }

  int len = sx()->getPacketLength();
  if (len > 0) {
    if (len > sz) len = sz;
    int err = sx()->readData(bytes, len);
    if (err != RADIOLIB_ERR_NONE) {
      MESH_DEBUG_PRINTLN("SFScan: readData(%d) on SF%d", err, _cur_sf);
      n_recv_errors++;
      len = 0;
    } else {
      n_recv++;
      int i = _cur_sf - SFSCAN_MIN_SF;
      if (i >= 0 && i < SFSCAN_NUM_SF) {
        _heard_at[i] = millis() | 1;   // never 0
        _rx_count[i]++;
      }
      _last_rx_sf = _cur_sf;
      _last_snr = sx()->getSNR();
      _last_rssi = sx()->getRSSI();
      _rx_airtime_pending = true;

      uint32_t h;
      if (payloadHash(bytes, len, h)) {
        _seen[_seen_next].hash = h;
        _seen[_seen_next].sf = _cur_sf;
        _seen_next = (_seen_next + 1) % SFSCAN_SEEN_SIZE;
      }
    }
  } else {
    len = 0;
  }
  resumeScan();
  return len;
}

// home SF first, then the other scanned SFs
uint8_t SFScanSX1262Wrapper::addScanSFs(uint8_t* out, uint8_t n) const {
  out[n++] = _home_sf;
  for (uint8_t j = 0; j < _num_scan_sfs; j++) {
    if (_scan_sfs[j] != _home_sf) out[n++] = _scan_sfs[j];
  }
  return n;
}

uint8_t SFScanSX1262Wrapper::buildTxList(const uint8_t* raw, int len, uint8_t* out) const {
#ifdef SFSCAN_TX_SAME_SF
  // forwarded packet -> only the SF it was received on
  uint32_t h;
  if (raw && payloadHash(raw, len, h)) {
    for (int k = 0; k < SFSCAN_SEEN_SIZE; k++) {
      int idx = (_seen_next + SFSCAN_SEEN_SIZE - 1 - k) % SFSCAN_SEEN_SIZE;   // newest first
      if (_seen[idx].sf != 0 && _seen[idx].hash == h) {
        out[0] = _seen[idx].sf;
        return 1;
      }
    }
  }
  // own packet (advert, reply, ...) -> every scanned SF
  return addScanSFs(out, 0);
#else
  uint8_t n = 0;
  out[n++] = _home_sf;
  uint32_t now = millis();
  for (uint8_t sf = SFSCAN_MIN_SF; sf <= SFSCAN_MAX_SF; sf++) {
    if (sf == _home_sf) continue;
    uint32_t t = _heard_at[sf - SFSCAN_MIN_SF];
    if (t != 0 && now - t < SFSCAN_TX_WINDOW_MS) out[n++] = sf;
  }
  return n;
#endif
}

uint32_t SFScanSX1262Wrapper::getEstAirtimeFor(int len_bytes) {
  if (_rx_airtime_pending) {   // Dispatcher asks for the airtime of the packet just received
    _rx_airtime_pending = false;
    return airtimeMs(_last_rx_sf, len_bytes);
  }
#ifdef SFSCAN_TX_SAME_SF
  // a forwarded packet goes out once -> airtime on the slowest scanned SF
  uint32_t worst = airtimeMs(_home_sf, len_bytes);
  for (uint8_t j = 0; j < _num_scan_sfs; j++) {
    uint32_t t = airtimeMs(_scan_sfs[j], len_bytes);
    if (t > worst) worst = t;
  }
  return worst;
#else
  return getEstTxAirtimeFor(NULL, len_bytes);
#endif
}

uint32_t SFScanSX1262Wrapper::getEstTxAirtimeFor(const uint8_t* raw, int len) {
  uint8_t sfs[SFSCAN_NUM_SF + 1];
  uint8_t n = buildTxList(raw, len, sfs);
  uint32_t total = 0;
  for (uint8_t i = 0; i < n; i++) total += airtimeMs(sfs[i], len) + (i > 0 ? TX_GAP_EST_MS : 0);
  return total;
}

bool SFScanSX1262Wrapper::txCurrent() {
  applySF(_tx_sfs[_tx_idx]);
  sfscan_irq = false;
  int err = sx()->startTransmit(_tx_buf, _tx_len);
  if (err != RADIOLIB_ERR_NONE) {
    MESH_DEBUG_PRINTLN("SFScan: startTransmit(%d) on SF%d", err, _cur_sf);
    return false;
  }
  return true;
}

bool SFScanSX1262Wrapper::startSendRaw(const uint8_t* bytes, int len) {
  if (len > (int)sizeof(_tx_buf)) len = sizeof(_tx_buf);
  memcpy(_tx_buf, bytes, len);
  _tx_len = len;
  _tx_num = buildTxList(_tx_buf, _tx_len, _tx_sfs);
  _tx_idx = 0;

  sx()->standby();     // abort any CAD / RX in progress
  sfscan_irq = false;
  _state = S_TX;

  _board->onBeforeTransmit();
  if (!txCurrent()) {
    _board->onAfterTransmit();
    resumeScan();
    return false;
  }
  return true;
}

bool SFScanSX1262Wrapper::isSendComplete() {
  pollDio1();
  if (_state != S_TX || !sfscan_irq) return false;
  sfscan_irq = false;

  _tx_idx++;
  if (_tx_idx < _tx_num) {
    sx()->finishTransmit();
    if (txCurrent()) return false;   // next SF on its way
  }
  n_sent++;
  return true;
}

void SFScanSX1262Wrapper::onSendFinished() {
  sx()->finishTransmit();
  _board->onAfterTransmit();
  resumeScan();
}

void SFScanSX1262Wrapper::resetAGC() {
  if (_state == S_RX || _state == S_TX) return;   // never in the middle of a packet
  doResetAGC();
  sfscan_irq = false;
  _cur_sf = 0;          // re-apply modulation settings after the frontend reset
  _state = S_IDLE;
  startCAD();
}

#endif
