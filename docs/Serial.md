# Serial

Betaflight has enhanced serial port flexibility but configuration is slightly more complex as a result.

Betaflight has the concept of a function (MSP, GPS, Serial RX, etc) and a port (VCP, UARTx, SoftSerial x).
Not all functions can be used on all ports due to hardware pin mapping, conflicting features, hardware, and software
constraints.

## Serial port types

* USB Virtual Com Port (VCP) - USB pins on a USB port connected directly to the processor without requiring
a dedicated USB to UART adapter.  VCP does not 'use' a physical UART port.
* UART - A pair of dedicated hardware transmit and receive pins with signal detection and generation done in hardware.
* SoftSerial - A pair of hardware transmit and receive pins with signal detection and generation done in software.

UART is the most efficient in terms of CPU usage.
SoftSerial is the least efficient and slowest, SoftSerial should only be used for low-bandwidth usages, such as telemetry transmission.

UART ports are sometimes exposed via on-board USB to UART converters, such as the CP2102 as found on the Naze and Flip32 boards.
If the flight controller does not have an on-board USB to UART converter and doesn't support VCP then an external USB to UART board is required.
These are sometimes referred to as FTDI boards.  FTDI is just a common manufacturer of a chip (the FT232RL) used on USB to UART boards.

When selecting a USB to UART converter choose one that has DTR exposed as well as a selector for 3.3v and 5v since they are more useful.

Examples:
 
 * [FT232RL FTDI USB To TTL Serial Converter Adapter](http://www.banggood.com/FT232RL-FTDI-USB-To-TTL-Serial-Converter-Adapter-Module-For-Arduino-p-917226.html)
 * [USB To TTL / COM Converter Module buildin-in CP2102](http://www.banggood.com/Wholesale-USB-To-TTL-Or-COM-Converter-Module-Buildin-in-CP2102-New-p-27989.html)

Both SoftSerial and UART ports can be connected to your computer via USB to UART converter boards. 

## Serial Configuration

Serial port configuration is best done via the configurator.

Configure serial ports first, then enable/disable features that use the ports.  To configure SoftSerial ports the SOFTSERIAL feature must be enabled. 

### Constraints

If the configuration is invalid the serial port configuration will reset to its defaults and features may be disabled.

* There must always be a port available to use for MSP/CLI.
* There is a maximum of 3 MSP ports.
* To use a port for a function, the function's corresponding feature must be also be enabled.
e.g. after configuring a port for GPS enable the GPS feature.
* If SoftSerial is used, then all SoftSerial ports must use the same baudrate.
* Softserial is limited to 19200 baud.
* All telemetry systems except MSP will ignore any attempts to override the baudrate.
* MSP/CLI can be shared with EITHER Blackbox OR telemetry.  In shared mode blackbox or telemetry will be output only when armed.
* Smartport telemetry cannot be shared with MSP.
* No other serial port sharing combinations are valid.
* You can use as many different telemetry systems as you like at the same time.
* You can only use each telemetry system once.  e.g.  FrSky telemetry cannot be used on two port, but MSP Telemetry + FrSky on different ports is fine.

### Configuration via CLI

You can use the CLI for configuration but the commands are reserved for developers and advanced users.

The `serial` CLI command takes 6 arguments:
```
serial <port identifier> <port function> <msp baudrate> <gps baudrate> <telemetry baudrate> <blackbox baudrate>
```

| Serial cli command arguments |
| ---------------------------- |
| 1. Serial Port Identifier    |
| 2. Serial Port Function      |
| 3. MSP baud rate             |
| 4. GPS baud rate             |
| 5. Telemetry baud rate       |
| 6. Blackbox baudrate         |

Note: for Identifier see serialPortIdentifier_e in the source; for Function bitmask see serialPortFunction_e in the source code.

### 1. Serial Port Identifier

| Identifier                 | Value |
| -------------------------- | ----- |
| SERIAL_PORT_NONE           | -1    |
| SERIAL_PORT_USART1         | 0     |
| SERIAL_PORT_USART2         | 1     |
| SERIAL_PORT_USART3         | 2     |
| SERIAL_PORT_UART4          | 3     |
| SERIAL_PORT_UART5          | 4     |
| SERIAL_PORT_USART6         | 5     |
| SERIAL_PORT_USART7         | 6     |
| SERIAL_PORT_USART8         | 7     |
| SERIAL_PORT_UART9          | 8     |
| SERIAL_PORT_USART10        | 9     |
| SERIAL_PORT_USB_VCP        | 20    |
| SERIAL_PORT_SOFTSERIAL1    | 30    |
| SERIAL_PORT_SOFTSERIAL2    | 31    |
| SERIAL_PORT_LPUART1        | 40    |

ID's 0-19 reserved for UARTS 1-20
ID's 20-29 reserved for USB 1-10
ID's 30-39 reserved for SoftSerial 1-10
ID's 40-49 reserved for LPUART 1-10
Other devices can be added starting from id 50.

### 2. Serial Port Function

| Function                     | Value |
| ---------------------------- | ----- |
| FUNCTION_NONE                | 0     |
| FUNCTION_MSP                 | 1     |
| FUNCTION_GPS                 | 2     |
| FUNCTION_TELEMETRY_FRSKY_HUB | 4     |
| FUNCTION_TELEMETRY_HOTT      | 8     |
| FUNCTION_TELEMETRY_LTM       | 16    |
| FUNCTION_TELEMETRY_SMARTPORT | 32    |
| FUNCTION_RX_SERIAL           | 64    |
| FUNCTION_BLACKBOX            | 128   |
| FUNCTION_TELEMETRY_MAVLINK   | 512   |
| FUNCTION_ESC_SENSOR          | 1024  |
| FUNCTION_VTX_SMARTAUDIO      | 2048  |
| FUNCTION_TELEMETRY_IBUS      | 4096  |
| FUNCTION_VTX_TRAMP           | 8192  |
| FUNCTION_RCDEVICE            | 16384 |
| FUNCTION_LIDAR_TF            | 32768 |
| FUNCTION_FRSKY_OSD           | 65536 |
| FUNCTION_PRINTF              | 131072 |
| FUNCTION_SBUS_OUT            | 262144 |
| FUNCTION_FBUS_MASTER         | 524288 |
| FUNCTION_SPORT_MASTER        | 1048576 |
| FUNCTION_SRXL2_ESC           | 2097152 |
| FUNCTION_RX_INPUT_BACKUP     | 4194304 |

Note: values above `FUNCTION_LIDAR_TF` require more than 16 bits. `FUNCTION_SPORT_MASTER` = `(1<<20)` requires 21 bits.

`FUNCTION_RX_INPUT_BACKUP` assigns a UART to a secondary, independent RX input
("backup RX"). It is not the main RX link - it exists as a fallback: if the main RX
link's signal is lost, the FC takes all RC channels (including aux/mode switches)
from this port instead, bypassing the staged failsafe machinery (hold/land/cut)
entirely, and reverts back automatically once the main link recovers. If the main
link is present, this port's data has no effect. None of this applies if no
backup protocol is configured (`provider` is `NONE`) or the backup link isn't
currently up itself - in either case the main RX's normal staged failsafe
(hold/land/cut) remains the fallback, exactly as it would without this feature.
Takeover/revert is bounded by the
main RX's own existing ~100ms signal-loss detection window (`rxSignalReceived`,
`DELAY_100_MS` in `rx.c`'s `rxFrameCheck()`), not per-missed-frame. See
`drivers/rx_input_backup.c` and `rx/rx.c`'s `detectAndApplySignalLossBehaviour()`.
Diagnostics are available read-only via `MSP2_GET_RX_INPUT_BACKUP_STATUS`, and
provider/inverted/halfDuplex/pinSwap config can be read and written from the
configurator via `MSP2_GET_RX_INPUT_BACKUP_CONFIG`/`MSP2_SET_RX_INPUT_BACKUP_CONFIG`
(a changed value only takes effect after save+reboot, same as any other
serial-port function/config change).

Which protocol this port speaks is selected via `rx_input_backup_provider`:
`NONE`, `SBUS`, `FBUS`, `FPORT`, `FPORT2`, `EXBUS`, or `CRSF`
(`pg/rx_input_backup.h`'s
`provider` field). `NONE` (value `0`, matching this codebase's usual
"zero-init means off" convention) is the default for a freshly reset config -
assigning a port `FUNCTION_RX_INPUT_BACKUP` alone no longer silently starts
decoding SBUS on it; the port is reserved but never opened until a real
protocol is chosen. FBUS is decoded as 16-channel frames only (its 8ch/24ch
variants aren't supported yet). `EXBUS` (Jeti EX
Bus) decodes only the fixed 16-channel data frame; unlike the other
providers here its own reference driver (rx/jetiexbus.c) always opens the
port bidirectionally since a real Jeti receiver may need a telemetry reply -
this backup link never replies (receive-only, as always), which is expected
to be fine since the receiver broadcasts channel data on its own schedule
regardless, but hasn't been hardware-verified specifically for this
protocol's stricter documented bus-master expectations. `CRSF`
(Crossfire/ELRS) decodes only the `RC_CHANNELS_PACKED` frame type off a
generic address+length-prefixed byte stream - unlike the other length-
prefixed providers here, a real CRSF link legitimately interleaves other
frame types (e.g. `LINK_STATISTICS`, sent unprompted) on the same wire, so
this provider tracks dynamic frame length the same way `rx/crsf.c` itself
does rather than assuming one fixed shape, to avoid losing byte sync
whenever a different frame type arrives. Adding another
protocol is a small, additive
change (see `drivers/rx_input_backup_sbus.c`/`_fbus.c`/`_fport.c`/`_exbus.c`/`_crsf.c`
for the template); there is deliberately no telemetry on this link, ever, for any
protocol - it exists purely to hand over channel data, same as a physical
backup satellite receiver would.

Electrical settings for this port are independent of the main RX's own
`serialrx_inverted`/`serialrx_halfduplex`/`serialrx_pinswap` (different
physical UART): `rx_input_backup_inverted`, `rx_input_backup_halfduplex`, and
`rx_input_backup_pinswap` (all `OFF`/`ON`, default `OFF`), in
`pg/rx_input_backup.h`. `pinSwap` behaves identically for every protocol, but
`inverted` and `halfDuplex` do not - each protocol's own native wiring
convention differs (SBUS is natively inverted, FBUS/FPort/FPort2 are not; SBUS
uses plain half-duplex, FBUS/FPort/FPort2 use push-pull half-duplex), so
`OFF` (the default, "normal wiring for whichever protocol is selected") maps
to a different underlying UART configuration depending on
`rx_input_backup_provider` - each provider's own driver file handles this
translation, exactly mirroring how `rx/sbus.c` and `rx/fbus.c`/`rx/fport.c`
apply the main RX's own `serialrx_inverted`/`serialrx_halfduplex` in opposite
directions for the same reason.

### 3. MSP Baudrates

| Baudrate |
| -------- |
| 9600     |
| 19200    |
| 38400    |
| 57600    |
| 115200   |
| 230400   |
| 250000   |
| 500000   |
| 1000000  |

### 4 GPS Baudrates

| Baudrate |
| -------- |
| 9600     |
| 19200    |
| 38400    |
| 57600    |
| 115200   |

Note: Also has a boolean AUTOBAUD. It is recommended to use a fixed baudrate. Configure GPS baudrate according to device documentation.

### 5. Telemetry Baudrates

| Baudrate |
| -------- |
| AUTO     |
| 9600     |
| 19200    |
| 38400    |
| 57600    |
| 115200   |

### 6. Blackbox Baudrates

| Baudrate |
| -------- |
| 19200    |
| 38400    |
| 57600    |
| 115200   |
| 230400   |
| 250000   |
| 400000   |
| 460800   |
| 500000   |
| 921600   |
| 1000000  |
| 1500000  |
| 2000000  |
| 2470000  |

### Serial Port Baud Rates

The Serial Port baudrates are defined as follows:

| ID | Baudrate  |
| -- | --------- |
| 0  | Auto      |
| 1  | 9600      |
| 2  | 19200     |
| 3  | 38400     |
| 4  | 57600     |
| 5  | 115200    |
| 6  | 230400    |
| 7  | 250000    |
| 8  | 400000    |
| 9  | 460800    |
| 10 | 500000    |
| 11 | 921600    |
| 12 | 1000000   |
| 13 | 1500000   |
| 14 | 2000000   |
| 15 | 2470000   |


### Passthrough

Betaflight can enter a special passthrough mode whereby it passes serial data through to a device connected to a UART/SoftSerial port. This is useful to change the configuration of a Betaflight peripheral such as an OSD, bluetooth dongle, serial RX etc.

To initiate passthrough mode, use the CLI command `serialpassthrough` This command takes four arguments.

    serialpassthrough <port1 id> [port1 baud] [port1 mode] [port1 DTR PINIO] [port2 id] [port2 baud] [port2 mode]

`PortX ID` is the internal identifier of the serial port from Betaflight source code (see serialPortIdentifier_e in the source). For instance UART1-UART4 are 0-3 and SoftSerial1/SoftSerial2 are 30/31 respectively. PortX Baud is the desired baud rate, and portX mode is a combination of the keywords rx and tx (rxtx is full duplex). The baud and mode parameters can be used to override the configured values for the specified port. `port1 DTR PINIO` identifies the PINIO resource which is optionally connected to a DTR line of the attached device.

If port2 config(the last three arguments) is not specified, the passthrough will run between port1 and VCP. The last three arguments are used for `Passthrough between UARTs`, see that section to get detail.

For example. If you have your MWOSD connected to UART 2, you could enable communicaton to this device using the following command. This command does not specify the baud rate or mode, using the one configured for the port (see above).

    serialpassthrough 1

If a baud rate is not specified, or is set to 0, then `serialpassthrough` supports changing of the baud rate over USB. This allows tools such as the MWOSD GUI to dynamically set the baud rate to, for example 57600 for reflashing the MWOSD firmware and then 115200 for adjusting settings without having to powercycle your flight control board between the two.

_To use a tool such as the MWOSD GUI, it is necessary to disconnect or exit Betaflight configurator._

**To exit serial passthrough mode, power cycle your flight control board.**

In order to reflash an Arduino based device such as a MWOSD via `serialpassthrough` if is necessary to connect the DTR line in addition to the RX and TX serial lines. The DTR is used as a reset line to invoke the bootloader. The DTR line may be connected to any GPIO pin on the flight control board. This pin must then be associated with a PINIO resource, the instance of which is then passed to the serialpassthrough command. If you don't need it, you can ignore it or set it to `none`. The DTR line associated with any given UART may be set using the CLI command `resource` specifying it as a PINIO resource.

For example, the following configuration for an OpenPilot Revolution shows the UART6 serial port to be configured with TX on pin C06, RX on pin C07 and a DTR connection using PINIO on pin C08.

```
resource SERIAL_TX 1 A09
resource SERIAL_TX 3 B10
resource SERIAL_TX 4 A00
resource SERIAL_TX 6 C06
resource SERIAL_RX 1 A10
resource SERIAL_RX 3 B11
resource SERIAL_RX 6 C07

resource PINIO 1 C08
```

To assign the DTR line to another pin use the following command.

```
resource PINIO 1 c05
```

To disassociate DTR from a pin use the following command.

```
resource PINIO 1 none
```

Having configured a PINIO resource assocaited with a DTR line as per the above example, connection to an MWOSD attached to an Openpilot Revolution could be achieved using the following command.

```serialpassthrough 5 0 rxtx 1```

This will connect using UART 6, with the baud rate set over USB, full duplex, and with DTR driven on PINIO resource 1.

A (desirable) side effect of configuring the DTR line to be associated with a PINIO resource, is that when the FC is reset, the attached Arduino device will also be reset.

Note that if DTR is left configured on a port being used with a standard build of MWOSD firmware, the display will break-up when the flight controller is reset. This is because, by default, the MWOSD does not correctly handle resets from DTR. There are two solutions to this:

1. Assign the DTR pin using the resource command above prior to reflashing MWOSD, and then dissasociate DTR from the pin.
2. Rebuild MWOSD with MAX_SOFTRESET defined. The MWOSD will then be reset correctly every time the flight controller is reset.

### Passthrough between UARTs

in BetaFlight 4.1 or later, you can make a serial passthrough between UARTs.

the last three arguments of `serialpassthrough` are used to the passthrough between UARTs: `[port2 id]` `[port2 baud]` `[port2 mode]`, if you don't need passthrough between UARTs, just ignore them, and use `serialpassthrough` according to above description.
if you want passthrough between UARTs, `[port2 id]` is a required argument, the value range is same with `port1 ID` argument, it is the internal identifier of the serial port. `[port2 baud]`and`[port2 mode]` is optional argument, the default of them are `57600` and `MODE_RXTX`.

For example. If you using a filght controller built-in BLE chip, and the BLE chip was inner connected to a UART, you can use the following command to let the UART to talk with other UART:
```
serialpassthrough 0 115200 rxtx none 4 19200
```
the command will run a serial passthrough between UART1 and UART5, UART1 baud is 115200, mode is MODE_RXTX, DTR is none, UART5 baud is 19200, mode is not specific, it will take default value MODE_RXTX.
