/// @file
/// @brief Support for the Xiaomi MI  

// Supports:
//   Brand: Xiaomi,  Model: MI

// Based almost entirely on the ir_RCMM

#include <algorithm>
#include "IRrecv.h"
#include "IRsend.h"
#include "IRtimer.h"
#include "IRutils.h"

// Constants
const uint16_t kXMiTick = 26;
const uint16_t kXMiHdrMarkTicks = 39;
const uint16_t kXMiHdrMark = 1014;
const uint16_t kXMiHdrSpaceTicks = 23;
const uint16_t kXMiHdrSpace = 598;
const uint16_t kXMiBitMarkTicks = 23;
const uint16_t kXMiBitMark = 598;
const uint16_t kXMiBitSpace0Ticks = 23;
const uint16_t kXMiBitSpace0 = 598;
const uint16_t kXMiBitSpace1Ticks = 35;
const uint16_t kXMiBitSpace1 = 910;
const uint16_t kXMiBitSpace2Ticks = 46;
const uint16_t kXMiBitSpace2 = 1196;
const uint16_t kXMiBitSpace3Ticks = 57;
const uint16_t kXMiBitSpace3 = 1482;
const uint16_t kXMiRptLengthTicks = 992;
const uint32_t kXMiRptLength = 27778;
const uint16_t kXMiMinGapTicks = 400;
const uint32_t kXMiMinGap = 10400;
// Use a tolerance of +/-10% when matching some data spaces.
const uint8_t kXMiTolerance = 10;
const uint16_t kXMiExcess = 50;

#if SEND_XMI
/// Send a Xiaomi MI packet.
/// Status: UNKNOWN
/// @param[in] data The message to be sent.
/// @param[in] nbits The number of bits of message to be sent.
/// @param[in] repeat The number of times the command is to be repeated.
void IRsend::sendXMI(uint64_t data, uint16_t nbits, uint16_t repeat) {
  // Set 38kHz IR carrier frequency & a 1/3 (33%) duty cycle.
  enableIROut(38, 33);
  IRtimer usecs = IRtimer();

  for (uint16_t r = 0; r <= repeat; r++) {
    usecs.reset();
    // Header
    mark(kXMiHdrMark);
    space(kXMiHdrSpace);
    // Data
    uint64_t mask = 0b11ULL << (nbits - 2);
    for (int32_t i = nbits; i > 0; i -= 2) {
      mark(kXMiBitMark);
      // Grab the next Most Significant Bits to send.
      switch ((data & mask) >> (i - 2)) {
        case 0b00:
          space(kXMiBitSpace0);
          break;
        case 0b01:
          space(kXMiBitSpace1);
          break;
        case 0b10:
          space(kXMiBitSpace2);
          break;
        case 0b11:
          space(kXMiBitSpace3);
          break;
      }
      mask >>= 2;
    }
    // Footer
    mark(kXMiBitMark);
    // Protocol requires us to wait at least kXMiRptLength usecs from the
    // start or kXMiMinGap usecs.
    space(std::max(kXMiRptLength - usecs.elapsed(), kXMiMinGap));
  }
}
#endif  // SEND_XMI

#if DECODE_XMI
/// Decode a Xiaomi MI packet, 20bits
/// Status:  UNKNOWN
/// @param[in,out] results Ptr to the data to decode & where to store the result
/// @param[in] offset The starting index to use when attempting to decode the
///   raw data. Typically/Defaults to kStartOffset.
/// @param[in] nbits The number of data bits to expect.
/// @param[in] strict Flag indicating if we should perform strict matching.
/// @return True if it can decode it, false if it can't.
bool IRrecv::decodeXMI(decode_results *results, uint16_t offset,
                        const uint16_t nbits, const bool strict) {
  uint64_t data = 0;

  if (results->rawlen <= 4 + offset - 1)
    return false;  // Not enough entries to ever be XMI.

  // Calc the maximum size in bits, the message can be, or that we can accept.
  int16_t maxBitSize =
      std::min((uint16_t)results->rawlen - 5, (uint16_t)sizeof(data) * 8);
  // Compliance
  if (strict) {
    if (maxBitSize < 20) return false;
    if (maxBitSize < nbits)
      return false;  // Short cut, we can never reach the expected nr. of bits.
  }
  // Header decode
  if (!matchMark(results->rawbuf[offset], kXMiHdrMark)) return false;
  // Calculate how long the common tick time is based on the header mark.
  uint32_t m_tick = results->rawbuf[offset++] * kRawTick / kXMiHdrMarkTicks;
  if (!matchSpace(results->rawbuf[offset], kXMiHdrSpace)) return false;
  // Calculate how long the common tick time is based on the header space.
  uint32_t s_tick = results->rawbuf[offset++] * kRawTick / kXMiHdrSpaceTicks;

  // Data decode
  uint16_t actualBits;
  for (actualBits = 0; actualBits < nbits; actualBits += 2, offset++) {
    if (!match(results->rawbuf[offset++], kXMiBitMarkTicks * m_tick))
      return false;

    data <<= 2;
    if (match(results->rawbuf[offset], kXMiBitSpace0Ticks * s_tick))
      data += 0;
    else if (match(results->rawbuf[offset], kXMiBitSpace1Ticks * s_tick))
      data += 1;
    else if (match(results->rawbuf[offset], kXMiBitSpace2Ticks * s_tick,
                   kXMiTolerance))
      data += 2;
    else if (match(results->rawbuf[offset], kXMiBitSpace3Ticks * s_tick,
                   kXMiTolerance))
      data += 3;
    else
      return false;
  }
  // Footer decode
  if (!match(results->rawbuf[offset++], kXMiBitMarkTicks * m_tick))
    return false;
  if (offset < results->rawlen &&
      !matchAtLeast(results->rawbuf[offset], kXMiMinGapTicks * s_tick))
    return false;

  // Compliance
  if (strict && actualBits != nbits) return false;

  // Success
  results->value = data;
  results->decode_type = XMI;
  results->bits = actualBits;
  results->address = 0;
  results->command = 0;
  return true;
}
#endif  // DECODE_XMI
