# PicoATP

## Elevator Pitch

It's a tiny AppleTalk stack for 8-bit PICs, coming in at under 1024 words of PIC assembly, and while the code targets the PIC12F1840, it is compatible with a variety of different PICs with the 8-bit mid-range core.  It's a companion project to [TashTalk](https://github.com/lampmerchant/tashtalk), a LocalTalk interface.  It advertises its presence by responding to NBP (Name Binding Protocol) queries and implements a system of mailboxes that can be used by the mainline program to respond to ATP (AppleTalk Transaction Protocol) queries.

## Setup

PicoATP responds to NBP queries for exactly one NBP entity with object name and type defined in flash with the `PA_OBJ` and `PA_TYPE` `#define`s.  It requires 16 bytes of SRAM for state, plus 16 bytes for each mailbox the programmer wishes to allocate.  The number of mailboxes is defined with the `PA_MBOX` build-time constant.  At least one mailbox must be allocated, but up to 256 are possible.  The location in linear SRAM where state variables and mailboxes begin is defined by the `PA_VARH` and `PA_VARL` build-time constants.  The location must be on a 16-byte boundary.

## Usage

The mainline program should periodically poll the mailboxes, looking for mailboxes where the first byte has the `MBINUSE` flag set and both the `MBRESP` and `MBNBP` flags clear - this indicates that the mailbox contains an ATP TReq, with its payload in the 8th through 15th bytes of the mailbox.  The actual length of the payload is defined in the low three bits of the first byte of the mailbox - payload length is represented decremented by one, so it can range from one byte (0b000) to eight bytes (0b111).  When the mainline program has replaced the request payload with a response payload, it should set the `MBRESP` flag on the mailbox and enable the UART transmitter interrupt (the `TXIE` flag of the `PIE1` register).  The interrupt handler will take care of sending and resending the payload.

## Example

The provided example mainline code gives control over three of the PIC12F1840's pins to the AppleTalk stack, RA0 and RA1 as simple GPIO lines, RA2 as a soft output-only UART running at a fixed rate of 9600 Hz.

The first byte in any received ATP request is used to set the GPIO lines:

| Bit(s) | Function                                         |
| ------ | ------------------------------------------------ |
| 7:4    | Ignored                                          |
| 3      | Data direction of GPIO 1 (1 = input, 0 = output) |
| 2      | Data direction of GPIO 0 (1 = input, 0 = output) |
| 1      | Latch value of GPIO 1 when used as an output     |
| 0      | Latch value of GPIO 0 when used as an output     |

The second through eighth bytes, if present, are queued for the soft UART to transmit.

The first byte in the ATP response indicates the state of the GPIO lines:

| Bit(s) | Function               |
| ------ | ---------------------- |
| 7:2    | Unchanged from request |
| 1      | Value read on GPIO 1   |
| 0      | Value read on GPIO 0   |

The second through eighth bytes, if present, are unchanged from the request.

## Building Firmware

Building the firmware requires Microchip MPASM, which is included with their development environment, MPLAB. Note that you must use MPLAB X version 5.35 or earlier or MPLAB 8 as later versions of MPLAB X have removed MPASM.
