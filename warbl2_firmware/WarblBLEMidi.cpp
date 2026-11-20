/**************************************************************************/
/*!
    @file     WarblBLEMidi (a copy of Adafruit's BLEMidi with one addition, sendSysExMessage).cpp
    @author   hathach (tinyusb.org) & toddtreece

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2018, Adafruit Industries (adafruit.com)
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**************************************************************************/

#include "bluefruit.h"
#include "WarblBLEMidi.h"

// GCC 5x new feature to detect optional include
#ifdef __has_include
#if __has_include("MIDI.h")
  #include <MIDI.h>
  #define MIDI_LIB_INCLUDED
#endif
#endif


/* MIDI Service: 03B80E5A-EDE8-4B33-A751-6CE34EC4C700
 * MIDI I/O    : 7772E5DB-3868-4112-A1A9-F2669D106BF3
 */

const uint8_t WARBL_BLEMIDI_UUID_SERVICE[] =
{
    0x00, 0xC7, 0xC4, 0x4E, 0xE3, 0x6C, 0x51, 0xA7,
    0x33, 0x4B, 0xE8, 0xED, 0x5A, 0x0E, 0xB8, 0x03
};

const uint8_t WARBL_BLEMIDI_UUID_CHR_IO[] =
{
    0xF3, 0x6B, 0x10, 0x9D, 0x66, 0xF2, 0xA9, 0xA1,
    0x12, 0x41, 0x68, 0x38, 0xDB, 0xE5, 0x72, 0x77
};

/*------------------------------------------------------------------*/
/* MIDI Data Type
 *------------------------------------------------------------------*/
typedef union ATTR_PACKED
{
  struct {
    uint8_t timestamp_hi : 6;
    uint8_t              : 1;
    uint8_t start_bit    : 1;
  };

  uint8_t byte;
} midi_header_t;

VERIFY_STATIC ( sizeof(midi_header_t) == 1 );

typedef union ATTR_PACKED
{
  struct {
    uint8_t timestamp_low : 7;
    uint8_t start_bit : 1;
  };

  uint8_t byte;
} midi_timestamp_t;

VERIFY_STATIC ( sizeof(midi_timestamp_t) == 1 );

typedef struct ATTR_PACKED
{
  midi_header_t header;
  midi_timestamp_t timestamp;
  uint8_t data[BLE_MIDI_TX_BUFFER_SIZE];
} midi_event_packet_t;

VERIFY_STATIC ( sizeof(midi_event_packet_t) == (BLE_MIDI_TX_BUFFER_SIZE + 2) );

typedef struct ATTR_PACKED
{
  midi_header_t header;
  uint8_t data[BLE_MIDI_TX_BUFFER_SIZE];
} midi_split_packet_t;

VERIFY_STATIC ( sizeof(midi_split_packet_t) == (BLE_MIDI_TX_BUFFER_SIZE + 1) );

/*------------------------------------------------------------------*/
/* IMPLEMENTATION
 *------------------------------------------------------------------*/
WarblBLEMidi::WarblBLEMidi(uint16_t fifo_depth)
  : BLEService(WARBL_BLEMIDI_UUID_SERVICE), _io(WARBL_BLEMIDI_UUID_CHR_IO), _rxd_fifo(1, fifo_depth)
{
  _write_cb    = NULL;
  _midilib_obj = NULL;
}

bool WarblBLEMidi::notifyEnabled(void)
{
  return _io.notifyEnabled();
}

bool WarblBLEMidi::notifyEnabled(uint16_t conn_hdl)
{
  return _io.notifyEnabled(conn_hdl);
}

void WarblBLEMidi::setWriteCallback(midi_write_cb_t fp)
{
  _write_cb = fp;
}

void WarblBLEMidi::autoMIDIread(void* midi_obj)
{
  _midilib_obj = midi_obj;
}
void WarblBLEMidi::begin(int baudrate)
{
  (void) baudrate;
  begin();
}

err_t WarblBLEMidi::begin(void)
{
  _rxd_fifo.begin();

  // Invoke base class begin()
  VERIFY_STATUS( BLEService::begin() );

  // IO characteristic
  _io.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP | CHR_PROPS_NOTIFY);
  _io.setPermission(SECMODE_ENC_NO_MITM, SECMODE_ENC_NO_MITM);
  _io.setWriteCallback(WarblBLEMidi::blemidi_write_cb);
  _io.setMaxLen(244);  // WARBL: allow full-size notifications (the default, 20 bytes, silently cut longer ones short)

  VERIFY_STATUS( _io.begin() );

  // Attempt to change the connection interval to 11.25-15 ms when starting HID
  Bluefruit.Periph.setConnInterval(9, 12);

  return ERROR_NONE;
}
  
bool WarblBLEMidi::beginTransmission(uint8_t type)
{
  return true;
};

void WarblBLEMidi::endTransmission()
{
}

/*------------------------------------------------------------------*/
/* Callbacks
 *------------------------------------------------------------------*/
void WarblBLEMidi::blemidi_write_cb(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len)
{
  if ( len < 3 ) return;

  WarblBLEMidi& midi_svc = (WarblBLEMidi&) chr->parentService();
  midi_svc._write_handler(conn_hdl, data, len);
}

void WarblBLEMidi::_write_handler(uint16_t conn_hdl, uint8_t* data, uint16_t len)
{
  // WARBL: bounds-checked (Adafruit's original read data[1] past the end when a packet ended in a single byte
  // with bit 7 set, and then let len wrap around), and a full FIFO drops the rest of the packet.
  if (len < 2) return;  // A header and nothing else, or malformed

  // drop the BLE MIDI header byte
  data++;
  len--;

  while (len)
  {
    // A byte with bit 7 set followed by another byte is a timestamp (before a status byte, or on its own before
    // running-status data): drop it. A lone one at the end of the packet is dropped too.
    if ( isStatusByte(data[0]) )
    {
      data++;
      len--;
      if (len == 0) break;
    }

    // write the status or MIDI data to the FIFO
    if ( !_rxd_fifo.write(data++, 1) ) break;  // Full: drop the rest of the packet
    len--;
  }

  // Call write callback if configured
  if ( _write_cb ) _write_cb(conn_hdl);

#ifdef MIDI_LIB_INCLUDED
  // read while possible if configured
  if ( _midilib_obj )
  {
    while( ((midi::MidiInterface<WarblBLEMidi>*)_midilib_obj)->read() ) { }
  }
#endif

}

/*------------------------------------------------------------------*/
/* Stream API
 *------------------------------------------------------------------*/
int WarblBLEMidi::read ( void )
{
  uint8_t ch;
  return _rxd_fifo.read(&ch) ? (int) ch : EOF;
}


size_t WarblBLEMidi::write ( uint8_t b )
{
  // MIDI Library will write event byte by byte.
  // We need to buffer the data until we have a full event,
  // or until we reach the TX buffer limit.
  static uint8_t count = 0;
  static uint8_t buf[BLE_MIDI_TX_BUFFER_SIZE] = { 0 };

  // the current byte is a sysex end status message,
  // and we still have an existing buffer. send the
  // existing buffer and clear it so we can send
  // the sysex end status message with the appropriate
  // BLE header and timestamp bytes.
  if(b == 0xF7 && count > 0)
  {
    // send and clear the last of the existing buffer.
    // it should contain the final bytes in the sysex payload.
    if (isStatusByte(buf[0]))
      send(buf, count);
    else
      sendSplit(buf, count);

    // reset buffer
    buf[0] = 0;
    count = 0;
  }

  // add the current byte to the buffer
  buf[count++] = b;

  // send matching 1, 2, or 3 byte messages
  // and clear the buffer
  if ( (oneByteMessage(buf[0]) && count == 1) ||
       (twoByteMessage(buf[0]) && count == 2) ||
       (threeByteMessage(buf[0]) && count == 3) )
  {
    send(buf, count);
    // reset buffer
    buf[0] = 0;
    count = 0;
  }

  // do we have a full buffer at this point?
  if(count == BLE_MIDI_TX_BUFFER_SIZE)
  {
    // send a full or split message depending
    // on the type of the first byte in the buffer
    if (isStatusByte(buf[0]))
      send(buf, count);
    else
      sendSplit(buf, count);

    // reset buffer
    buf[0] = 0;
    count = 0;
  }

  return 1;
}

int WarblBLEMidi::available ( void )
{
  return _rxd_fifo.count();
}

int WarblBLEMidi::peek ( void )
{
  uint8_t ch;
  return _rxd_fifo.peek(&ch) ? (int) ch : EOF;
}

void WarblBLEMidi::flush ( void )
{
  _rxd_fifo.clear();
}

/*------------------------------------------------------------------*/
/* Message Type Helpers
 *------------------------------------------------------------------*/
bool WarblBLEMidi::isStatusByte( uint8_t b )
{
  // if the bit 7 is set, then it's a MIDI status message
  if (bitRead(b, 7))
    return true;
  else
    return false;
}

bool WarblBLEMidi::oneByteMessage( uint8_t status )
{
  // system messages
  if (status >= 0xF4) return true;

  // system common
  if (status == 0xF1) return true;

  // sysex end
  if (status == 0xF7) return true;

  return false;
}

bool WarblBLEMidi::twoByteMessage( uint8_t status )
{
  // program change, aftertouch
  if (status >= 0xC0 && status <= 0xDF) return true;

  // song select
  if (status == 0xF3) return true;

  return false;
}

bool WarblBLEMidi::threeByteMessage( uint8_t status )
{
  // note off, note on, aftertouch, control change
  if (status >= 0x80 && status <= 0xBF) return true;

  // pitch wheel change
  if (status >= 0xE0 && status <= 0xEF) return true;

  // song position pointer
  if (status == 0xF2) return true;

  return false;
}

/*------------------------------------------------------------------*/
/* Send Event (notify)
 *------------------------------------------------------------------*/
bool WarblBLEMidi::send(uint8_t data[], uint8_t len)
{
  uint32_t tstamp = millis();

  midi_event_packet_t event =
  {
      .header = {{
          .timestamp_hi = (uint8_t) ((tstamp & 0x1F80UL) >> 7),
          .start_bit    = 1
      }},

      .timestamp = {{
          .timestamp_low = (uint8_t) (tstamp & 0x7FUL),
          .start_bit     = 1
      }}
  };

  memcpy(event.data, data, len);

  // send data length + 1 byte for header + 1 byte for timestamp
  return _io.notify(&event, len + 2);
}

bool WarblBLEMidi::sendSplit(uint8_t data[], uint8_t len)
{
  uint32_t tstamp = millis();

  midi_split_packet_t event =
  {
      .header = {{
          .timestamp_hi = (uint8_t) ((tstamp & 0x1F80UL) >> 7),
          .start_bit    = 1
      }}
  };

  memcpy(event.data, data, len);

  // send data length + 1 byte for header
  // don't include the second timestamp byte
  return _io.notify(&event, len + 1);
}

/*------------------------------------------------------------------*/
/* WARBL addition
 *------------------------------------------------------------------*/
// BLE MIDI packet layout for SysEx (MIDI over Bluetooth LE spec): every packet starts with a header byte
// (bit 7 set, the timestamp's high 6 bits); a timestamp byte (bit 7 set, low 7 bits) comes before F0 and before
// F7; packets in between carry only the header and data bytes.
uint16_t WarblBLEMidi::maxPacketLength(void)
{
  BLEConnection* connection = Bluefruit.Connection(Bluefruit.connHandle());
  const uint16_t mtu = connection ? connection->getMtu() : 23;
  return (uint16_t) min(244, max(0, (int) mtu - 3));  // ATT notification payload
}

bool WarblBLEMidi::sendSysExMessage(const uint8_t* message, uint16_t length)
{
  if (length < 2 || message[0] != 0xF0 || message[length - 1] != 0xF7) return false;

  const uint16_t maxPacket = maxPacketLength();

  const uint32_t tstamp = millis();
  const uint8_t header = (uint8_t) (0x80 | ((tstamp >> 7) & 0x3F));
  const uint8_t timestamp = (uint8_t) (0x80 | (tstamp & 0x7F));

  uint8_t packet[244];
  uint16_t n = 0;
  uint16_t i = 0;
  bool ok = true;
  packet[n++] = header;
  packet[n++] = timestamp;  // Before F0
  while (i < length - 1)    // Everything but the final F7
  {
    if (n == maxPacket)
    {
      ok = _io.notify(packet, n) && ok;
      n = 0;
      packet[n++] = header;
    }
    packet[n++] = message[i++];
  }
  if (n + 2 > maxPacket)  // No room for the timestamp and F7: they go in a packet of their own
  {
    ok = _io.notify(packet, n) && ok;
    n = 0;
    packet[n++] = header;
  }
  packet[n++] = timestamp;
  packet[n++] = 0xF7;
  ok = _io.notify(packet, n) && ok;
  return ok;
}

// Atomic one-notification SysEx for sensor batches. BLE MIDI framing is
// header, timestamp, F0 ... data ..., timestamp, F7 (3 extra bytes).
bool WarblBLEMidi::sendSysExSinglePacket(const uint8_t* message, uint16_t length)
{
  if (!message || length < 2 || message[0] != 0xF0 || message[length - 1] != 0xF7)
    return false;
  if (!notifyEnabled() || length + 3 > maxPacketLength()) return false;

  const uint32_t tstamp = millis();
  const uint8_t header = (uint8_t)(0x80 | ((tstamp >> 7) & 0x3F));
  const uint8_t timestamp = (uint8_t)(0x80 | (tstamp & 0x7F));
  uint8_t packet[244];
  packet[0] = header;
  packet[1] = timestamp;
  memcpy(packet + 2, message, length - 1); // Everything through last data byte
  packet[length + 1] = timestamp;
  packet[length + 2] = 0xF7;
  return _io.notify(packet, length + 3);
}
