# Full SoftAP MC-Link bridge

This is the full bridge, not just the isolated UDP echo bench. ESP32 UART1 pins
must be configured explicitly: TX17, RX18, 921600 baud. COM/UART0 remains the debug
console. PC Tenda connects to ESP SoftAP separately from its internet adapter.

The AP disconnect epoch invalidates the old socket/session even across a rapid
leave/rejoin. A new HELLO is required; it must never re-enable old motion.
Keep the 750ms ESP session and independent 150ms MCU command lease.

No real SSID password, sdkconfig, NVS or Flash dump is included. Root
`scripts/build_esp32_softap.sh` creates private ignored configuration and builds
only after an explicit new local password. Do not flash the default STA build
over an AP deployment or assume N16R8 means PSRAM is automatically used.
