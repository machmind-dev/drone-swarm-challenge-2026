# Shared Protocol

UART binary frame protocol shared between `drone-vision-esp32p4` (transmitter) and `drone-comms-esp32s3` (receiver).

## Contents

- `p4_link_protocol.h` — frame constants, packed structs, and CRC-8 helper

## Protocol Summary

```
Frame: SOF(1B) | LEN(1B) | TYPE(1B) | PAYLOAD | CRC8(1B)
```

| Constant | Value |
|----------|-------|
| `P4_LINK_SOF` | `0xAB` |
| `P4_LINK_TYPE_COMBINED` | `0x03` |
| `P4_LINK_PAYLOAD_LEN` | 47 bytes |
| `P4_LINK_FRAME_LEN` | 51 bytes |

### Payload layout (47 bytes)

```c
typedef struct { uint16_t dist_mm[6]; uint8_t status[6]; } p4_tof_t;   // 18 B
typedef struct { uint8_t valid; float x,y,z,qx,qy,qz,qw; } p4_pose_t;  // 29 B
typedef struct { p4_tof_t tof; p4_pose_t pose; } p4_combined_t;         // 47 B
```

CRC-8 (polynomial 0x07) is computed over `[TYPE, PAYLOAD...]`.

## Usage

Both the P4 transmitter (`p4_link_tx.c`) and S3 receiver (`p4_link.c`) include this header via the Docker bind-mount at `/code/shared/p4_link_protocol.h`.
