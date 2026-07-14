#ifndef UART_BUILDER_H
#define UART_BUILDER_H

#include <Arduino.h>
#include <vector>
#include <type_traits>

class UartBuilder {
public:
  UartBuilder() { clear(); }
  void clear() { data.clear(); }
  void appendByte(uint8_t b) { data.push_back(b); }

  // Append integer value as raw bytes, with an explicit byte count override
  // (e.g. to send a uint32_t truncated to 3 bytes). byteCount is clamped to
  // 1..4 since `value` is a 32-bit integer. bigEndian true => MSB first.
  void appendIntAsBytes(uint32_t value, uint8_t byteCount, bool bigEndian = true) {
    if (byteCount < 1) byteCount = 1;
    if (byteCount > 4) byteCount = 4;
    for (uint8_t i = 0; i < byteCount; ++i) {
      uint8_t shift = bigEndian ? (8 * (byteCount - 1 - i)) : (8 * i);
      data.push_back((uint8_t)((value >> shift) & 0xFF));
    }
  }

  // Preferred overload: infers the byte count from the type of `value`
  // (e.g. uint16_t -> 2 bytes, int32_t -> 4 bytes), so callers never need
  // to pass a byte count. `bigEndian` defaults to true (network byte order);
  // pass false only if the receiver expects little-endian.
  template <typename T>
  void appendIntAsBytes(T value, bool bigEndian = true) {
    static_assert(std::is_integral<T>::value, "appendIntAsBytes(T) requires an integral type (e.g. uint8_t/int16_t/uint32_t)");
    static_assert(sizeof(T) <= 8, "appendIntAsBytes(T) only supports integral types up to 8 bytes");

    // Use an unsigned 64-bit intermediate to avoid UB when shifting signed types.
    uint64_t v = (uint64_t)(typename std::make_unsigned<T>::type)value;
    const uint8_t byteCount = sizeof(T);
    for (uint8_t i = 0; i < byteCount; ++i) {
      uint8_t shift = bigEndian ? (8 * (byteCount - 1 - i)) : (8 * i);
      data.push_back((uint8_t)((v >> shift) & 0xFF));
    }
  }

  // Convenience named helpers for common sizes.
  void appendUint16(uint16_t v, bool bigEndian = true) { appendIntAsBytes<uint16_t>(v, bigEndian); }
  void appendUint32(uint32_t v, bool bigEndian = true) { appendIntAsBytes<uint32_t>(v, bigEndian); }
  void appendInt16(int16_t v, bool bigEndian = true) { appendIntAsBytes<int16_t>(v, bigEndian); }
  void appendInt32(int32_t v, bool bigEndian = true) { appendIntAsBytes<int32_t>(v, bigEndian); }

  // Append string characters as ASCII bytes
  void appendStringAsAscii(const String &s) {
    for (size_t i = 0; i < s.length(); ++i) data.push_back((uint8_t)s.charAt(i));
  }

  // Append float as ASCII decimal string (e.g. "12.34"). Uses dtostrf.
  void appendFloatAsAscii(float f, uint8_t decimals = 2) {
    char buf[32];
    dtostrf(f, 0, decimals, buf);
    appendStringAsAscii(String(buf));
  }

  // Append float as raw 4 IEEE-754 bytes. bigEndian true => MSB first.
  void appendFloatAsIEEE32(float f, bool bigEndian = true) {
    appendFloatAsBytes(f, bigEndian);
  }

  // Preferred name: append a float/double as its raw IEEE-754 bytes
  // (4 bytes for float, 8 bytes for double). Byte count is inferred from
  // the type, matching the appendIntAsBytes(T) convention. bigEndian
  // defaults to true (MSB first).
  template <typename T>
  void appendFloatAsBytes(T value, bool bigEndian = true) {
    static_assert(std::is_floating_point<T>::value, "appendFloatAsBytes(T) requires a floating point type (float/double)");

    uint8_t bytes[sizeof(T)];
    memcpy(bytes, &value, sizeof(T));
    if (bigEndian) {
      for (int i = sizeof(T) - 1; i >= 0; --i) appendByte(bytes[i]);
    } else {
      for (size_t i = 0; i < sizeof(T); ++i) appendByte(bytes[i]);
    }
  }

  // Return comma-separated decimal bytes (e.g. "65,66,67")
  String toPayload() const {
    String out;
    for (size_t i = 0; i < data.size(); ++i) {
      if (i) out += ',';
      out += String(data[i]);
    }
    return out;
  }

  size_t size() const { return data.size(); }

private:
  std::vector<uint8_t> data;
};

#endif // UART_BUILDER_H
