#pragma once

// Multi-SF scanning radio driver for SX1262 (Heltec V3 etc).
//
// RX: the SX1262 can only demodulate one spreading factor at a time, so this
//     driver cycles hardware CAD (channel activity detection) through a list of
//     spreading factors (SFSCAN_PATTERN). When CAD detects a LoRa preamble on an
//     SF, the radio receives on that SF (single RX with hardware timeout, so the
//     MCU may light-sleep meanwhile: every step ends with a DIO1 interrupt).
//
// TX, two modes:
//   default (companion):   every packet goes out on the "home" SF (configured radio
//                          SF) plus every scanned SF on which a valid packet was
//                          received within the last SFSCAN_TX_WINDOW_MS.
//   SFSCAN_TX_SAME_SF (repeater): a packet that was received is forwarded only on
//                          the SF it arrived on (matched by payload hash). Packets
//                          the node creates itself (adverts, replies) go out on all
//                          scanned SFs.
//
// Frequency, bandwidth and coding rate stay fixed (the configured values).

#include "CustomSX1262Wrapper.h"

#ifndef SFSCAN_TX_WINDOW_MS
  #define SFSCAN_TX_WINDOW_MS  (30UL * 60UL * 1000UL)   // 30 minutes
#endif

#ifndef SFSCAN_PATTERN
  // scan order: lower SFs have short preambles, so they are checked more often
  #define SFSCAN_PATTERN  8,7,9,8,7,10,8,7,9,8,7,11,8,7,9,8,7,10,8,7,9,8,7,12
#endif

#ifndef SFSCAN_LABEL
  #define SFSCAN_LABEL  "7-12"    // shown on the display
#endif

#define SFSCAN_MIN_SF  7
#define SFSCAN_MAX_SF  12
#define SFSCAN_NUM_SF  (SFSCAN_MAX_SF - SFSCAN_MIN_SF + 1)
#define SFSCAN_SEEN_SIZE  48      // remembered received packets (for same-SF forwarding)

class SFScanSX1262Wrapper : public CustomSX1262Wrapper {
  enum ScanState : uint8_t { S_IDLE, S_CAD, S_RX, S_TX };

  ScanState _state;
  float _freq, _bw;
  uint8_t _home_sf, _cr;
  uint8_t _cur_sf;             // SF the radio is currently configured for
  uint8_t _pattern_idx;
  uint32_t _rx_start;
  uint32_t _rx_window;
  uint8_t _scan_sfs[SFSCAN_NUM_SF];   // distinct SFs from the pattern
  uint8_t _num_scan_sfs;

  // receive bookkeeping
  uint32_t _heard_at[SFSCAN_NUM_SF];   // millis() of last valid packet per SF (0 = never)
  uint32_t _rx_count[SFSCAN_NUM_SF];
  uint8_t _last_rx_sf;
  float _last_snr, _last_rssi;
  bool _rx_airtime_pending;

  struct Seen { uint32_t hash; uint8_t sf; };
  Seen _seen[SFSCAN_SEEN_SIZE];
  uint8_t _seen_next;

  // transmit bookkeeping
  uint8_t _tx_buf[256];
  int _tx_len;
  uint8_t _tx_sfs[SFSCAN_NUM_SF];
  uint8_t _tx_num, _tx_idx;

  CustomSX1262* sx() const { return (CustomSX1262*)_radio; }

  void applySF(uint8_t sf);
  void startCAD();
  void startRxLocked();
  void resumeScan();
  bool txCurrent();
  void pollDio1();
  uint8_t buildTxList(const uint8_t* raw, int len, uint8_t* out) const;
  uint8_t addScanSFs(uint8_t* out, uint8_t n) const;
  uint32_t airtimeMs(uint8_t sf, int len) const;
  static bool payloadHash(const uint8_t* raw, int len, uint32_t& hash);

public:
  SFScanSX1262Wrapper(CustomSX1262& radio, mesh::MainBoard& board);

  bool isReceivingPacket() override { return _state == S_RX; }

  void begin() override;
  void setParams(float freq, float bw, uint8_t sf, uint8_t cr) override;

  int recvRaw(uint8_t* bytes, int sz) override;
  uint32_t getEstAirtimeFor(int len_bytes) override;
  uint32_t getEstTxAirtimeFor(const uint8_t* raw, int len) override;
  bool startSendRaw(const uint8_t* bytes, int len) override;
  bool isSendComplete() override;
  void onSendFinished() override;
  bool isInRecvMode() const override { return _state != S_TX; }
  bool isReceiving() override { return _state == S_RX; }
  void resetAGC() override;
  void loop() override;

  float getLastRSSI() const override { return _last_rssi; }
  float getLastSNR() const override { return _last_snr; }
  float packetScore(float snr, int packet_len) override { return packetScoreInt(snr, _last_rx_sf, packet_len); }
  uint8_t getSpreadingFactor() const override { return _home_sf; }

  // extra info (for display / debug)
  uint8_t getLastRxSF() const { return _last_rx_sf; }
  uint32_t getRxCountForSF(uint8_t sf) const {
    return (sf >= SFSCAN_MIN_SF && sf <= SFSCAN_MAX_SF) ? _rx_count[sf - SFSCAN_MIN_SF] : 0;
  }
};
