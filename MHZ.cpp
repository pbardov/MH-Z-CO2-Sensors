/* MHZ library

    By Tobias Schürg
*/

#include "MHZ.h"

const int MHZ14A = 14;
const int MHZ19B = 19;

const int MHZ14A_RESPONSE_TIME = 60;
const int MHZ19B_RESPONSE_TIME = 120;

const int STATUS_NO_RESPONSE = -2;
const int STATUS_CHECKSUM_MISMATCH = -3;
const int STATUS_INCOMPLETE = -4;
const int STATUS_NOT_READY = -5;

unsigned long lastRequest = 0;

MHZ::MHZ(uint8_t rxpin, uint8_t txpin, uint8_t pwmpin, uint8_t type) {
  SoftwareSerial *ss = new SoftwareSerial(rxpin, txpin);
  _pwmpin = pwmpin;
  _type = type;

  ss->begin(9600);
  _serial = ss;
}

MHZ::MHZ(Stream *serial, uint8_t pwmpin, uint8_t type) {
  _serial = serial;
  _pwmpin = pwmpin;
  _type = type;
}

/**
 * Enables or disables the debug mode (more logging).
 */
void MHZ::setDebug(boolean enable) {
  debug = enable;

#ifdef MHZ_DEBUG
  if (debug) {
    Serial.println(F("MHZ: debug mode ENABLED"));
  } else {
    Serial.println(F("MHZ: debug mode DISABLED"));
  }
#endif
}

boolean MHZ::isPreHeating() {
  if (_type == MHZ14A) {
    return millis() < MHZ_PREHEAT_TIME;
  } else if (_type == MHZ19B) {
    return millis() < MHZ_PREHEAT_TIME;
  } else {

#ifdef MHZ_DEBUG
    Serial.println(F("MHZ::isPreHeating() => UNKNOWN SENSOR"));
#endif

    return false;
  }
}

boolean MHZ::isReady() {
  if (_type == MHZ14A)
    return lastRequest < millis() - MHZ14A_RESPONSE_TIME;
  else if (_type == MHZ19B)
    return lastRequest < millis() - MHZ19B_RESPONSE_TIME;
  else {

#ifdef MHZ_DEBUG
    Serial.print(F("MHZ::isReady() => UNKNOWN SENSOR \""));
    Serial.print(_type);
    Serial.println(F("\""));
#endif

    return true;
  }
}

int MHZ::readCO2UART() {
  if (!isReady())
    return STATUS_NOT_READY;

#ifdef MHZ_DEBUG
  if (debug)
    Serial.println(F("-- read CO2 uart ---"));
#endif

  byte cmd[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
  byte response[9]; // for answer

#ifdef MHZ_DEBUG
  if (debug)
    Serial.print(F("  >> Sending CO2 request"));
#endif

  _serial->write(cmd, 9); // request PPM CO2
  lastRequest = millis();

  // clear the buffer
  memset(response, 0, 9);

  int waited = 0;
  while (_serial->available() == 0) {

#ifdef MHZ_DEBUG
    if (debug)
      Serial.print(".");
#endif

    delay(100); // wait a short moment to avoid false reading
    if (waited++ > 10) {

#ifdef MHZ_DEBUG
      if (debug)
        Serial.println(F("No response after 10 seconds"));
#endif

      _serial->flush();
      return STATUS_NO_RESPONSE;
    }
  }

#ifdef MHZ_DEBUG
  if (debug)
    Serial.println();
#endif

  // The serial stream can get out of sync. The response starts with 0xff, try
  // to resync.
  // TODO: I think this might be wrong any only happens during initialization?
  boolean skip = false;
  while (_serial->available() > 0 && (unsigned char)_serial->peek() != 0xFF) {
    if (!skip) {

#ifdef MHZ_DEBUG
      Serial.print(F("MHZ: - skipping unexpected readings:"));
#endif

      skip = true;
    }

#ifdef MHZ_DEBUG
    Serial.print(" ");
    Serial.print(_serial->peek(), HEX);
#endif

    _serial->read();
  }

#ifdef MHZ_DEBUG
  if (skip)
    Serial.println();
#endif

  if (_serial->available() > 0) {
    int count = _serial->readBytes(response, 9);
    if (count < 9) {
      _serial->flush();
      return STATUS_INCOMPLETE;
    }
  } else {
    _serial->flush();
    return STATUS_INCOMPLETE;
  }

#ifdef MHZ_DEBUG
  if (debug) {
    // print out the response in hexa
    Serial.print(F("  << "));
    for (int i = 0; i < 9; i++) {
      Serial.print(response[i], HEX);
      Serial.print(F("  "));
    }
    Serial.println(F(""));
  }
#endif

  // checksum
  byte check = getCheckSum(response);
  if (response[8] != check) {

#ifdef MHZ_DEBUG
    Serial.println(F("MHZ: Checksum not OK!"));
    Serial.print(F("MHZ: Received: "));
    Serial.println(response[8], HEX);
    Serial.print(F("MHZ: Should be: "));
    Serial.println(check, HEX);
#endif

    temperature = STATUS_CHECKSUM_MISMATCH;
    _serial->flush();
    return STATUS_CHECKSUM_MISMATCH;
  }

  int ppm_uart = 256 * (int)response[2] + response[3];

  temperature = response[4] - 44; // - 40;

  byte status = response[5];

#ifdef MHZ_DEBUG
  if (debug) {
    Serial.print(F(" # PPM UART: "));
    Serial.println(ppm_uart);
    Serial.print(F(" # Temperature? "));
    Serial.println(temperature);
  }
#endif

#ifdef MHZ_DEBUG
  // Is always 0 for version 14a  and 19b
  // Version 19a?: status != 0x40
  if (debug && status != 0) {
    Serial.print(F(" ! Status maybe not OK ! "));
    Serial.println(status, HEX);
  } else if (debug) {
    Serial.print(F(" Status  OK: "));
    Serial.println(status, HEX);
  }
#endif

  _serial->flush();
  return ppm_uart;
}

uint8_t MHZ::getLastTemperature() {
  if (isPreHeating())
    return STATUS_NOT_READY;
  return temperature;
}

byte MHZ::getCheckSum(byte *packet) {

#ifdef MHZ_DEBUG
  if (debug)
    Serial.println(F("  getCheckSum()"));
#endif

  byte i;
  unsigned char checksum = 0;
  for (i = 1; i < 8; i++) {
    checksum += packet[i];
  }
  checksum = 0xff - checksum;
  checksum += 1;
  return checksum;
}

int MHZ::readCO2PWM() {
  // if (!isReady()) return STATUS_NOT_READY; not needed?

#ifdef MHZ_DEBUG
  if (debug)
    Serial.print(F("-- reading CO2 from pwm "));
#endif

  unsigned long th, tl, ppm_pwm = 0;
  do {

#ifdef MHZ_DEBUG
    if (debug)
      Serial.print(".");
#endif

    th = pulseIn(_pwmpin, HIGH, 1004000) / 1000;
    tl = 1004 - th;
    ppm_pwm = 5000 * (th - 2) / (th + tl - 4);
  } while (th == 0);

#ifdef MHZ_DEBUG
  if (debug) {
    Serial.print(F("\n # PPM PWM: "));
    Serial.println(ppm_pwm);
  }
#endif

  return ppm_pwm;
}
