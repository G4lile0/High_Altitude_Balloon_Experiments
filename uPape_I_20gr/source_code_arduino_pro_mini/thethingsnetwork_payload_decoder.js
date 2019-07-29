function Decoder(bytes, port) {
  // Decode an uplink message from a buffer
  // (array) of bytes to an object of fields.
  var decoded = {};

  // if (port === 1) decoded.led = bytes[0];

lat_decode = ((bytes[0]) << 24)
+ ((bytes[1]) << 16)
+ ((bytes[2]) << 8)
+ ((bytes[3]));

lon_decode = ((bytes[4]) << 24)
+ ((bytes[5]) << 16)
+ ((bytes[6]) << 8)
+ ((bytes[7]));

if (port === 1) decoded.latitude = lat_decode / 1000000;
if (port === 1) decoded.longitude = lon_decode / 1000000;

if (port === 1) decoded.altitude = ((bytes[8]) << 8) + ((bytes[9]));
if (port === 1) decoded.hdop = bytes[10] / 10;
//if (port === 1) decoded.sats = bytes[11];

if (port === 2) decoded.temp = bytes[0]-127;
if (port === 2) decoded.Vcc = bytes[1] * 25 / 1000;

  return decoded;
}
