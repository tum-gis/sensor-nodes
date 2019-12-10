function Decoder(bytes, port) {
  var result = {};
  var transformers = {};

  if (port == 9) {
    transformers = {
      temperature: function transform(bytes) {
        value = bytes[0] * 256 + bytes[1];
        if (value >= 32768) value = value - 65536;
        return value / 100.0;
      },
      humidity: function transform(bytes) {
        return (bytes[0] * 256 + bytes[1]) / 100.0;
      },
      lat: function transform(bytes) {
        return (
          (bytes[0] * 16777216 + bytes[1] * 65536 + bytes[2] * 256 + bytes[3]) /
            1000000.0 -
          90.0
        );
      },
      lon: function transform(bytes) {
        return (
          (bytes[0] * 16777216 + bytes[1] * 65536 + bytes[2] * 256 + bytes[3]) /
            1000000.0 -
          180.0
        );
      },
      altitude: function transform(bytes) {
        return bytes[0] * 256 + bytes[1];
      },
      sat: function transform(bytes) {
        return bytes[0];
      },
      vbattery: function transform(bytes) {
        return (bytes[0] * 256 + bytes[1]) / 100.0;
      }
    };

    result["temperature"] = {
      value: transformers["temperature"](bytes.slice(0, 2)),
      uom: "Celsius"
    };

    result["humidity"] = {
      value: transformers["humidity"](bytes.slice(2, 4)),
      uom: "Percent"
    };

    result["lat"] = {
      value: transformers["lat"](bytes.slice(4, 8)),
      uom: "Degree"
    };

    result["lon"] = {
      value: transformers["lon"](bytes.slice(8, 12)),
      uom: "Degree"
    };

    result["altitude"] = {
      value: transformers["altitude"](bytes.slice(12, 14)),
      uom: "Meter"
    };

    result["sat"] = {
      value: transformers["sat"](bytes.slice(14, 15)),
      uom: "Count"
    };

    result["vbattery"] = {
      value: transformers["vbattery"](bytes.slice(15, 17)),
      uom: "Volt"
    };

    return result;
  }
}
