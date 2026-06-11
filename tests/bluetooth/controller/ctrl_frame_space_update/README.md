# Frame Space Update (FSU) Feature Unit Tests

This directory contains comprehensive unit tests for the Bluetooth Controller Frame Space Update (FSU) feature implemented in PR #99473.

## Test Overview

The Frame Space Update feature (Bluetooth 5.4) allows dynamic adjustment of inter-frame spacing on a per-connection and per-PHY basis. This enables more flexible timing between transmitted and received packets for different PHY types and ACL connection states.

## Test Coverage

### Test Cases with ASCII Art Message Sequence Charts

All test cases include detailed ASCII art diagrams showing the message flow between:
- **UT**: Unit Test (test harness)
- **LL_A**: Link Layer under test
- **LT**: Link Tester (simulated remote device)

#### 1. Local-Initiated Tests

**test_frame_space_update_central_loc** - Central initiates FSU
```
+-----+         +-------+         +-----+
| UT  |         | LL_A  |         | LT  |
+-----+         +-------+         +-----+
   |                |                 |
   | Start FSU      |                 |
   |--------------->|                 |
   |                | FSU_REQ         |
   |                |---------------->|
   |                |     FSU_RSP     |
   |                |<----------------|
   | Notification   |                 |
   |<---------------|                 |
```

**test_frame_space_update_peripheral_loc** - Peripheral initiates FSU

#### 2. Remote-Initiated Tests

**test_frame_space_update_central_rem** - Central receives FSU request
**test_frame_space_update_peripheral_rem** - Peripheral receives FSU request

#### 3. Error Handling Tests

**test_frame_space_update_central_loc_unknown_rsp** - Tests handling of LL_UNKNOWN_RSP

#### 4. Spacing Type Tests

**test_frame_space_update_cis_spacing** - Tests T_IFS_CIS (CIS timing)
- Tests ACL Central-to-Peripheral (T_IFS_ACL_CP)
- Tests ACL Peripheral-to-Central (T_IFS_ACL_PC)

#### 5. PHY-Specific Tests

**test_frame_space_update_multi_phy** - Tests FSU with multiple PHYs (1M, 2M, CODED)

#### 6. Initialization and Value Tests

**test_frame_space_update_init** - Validates initial FSU values
**test_frame_space_update_eff_value** - Tests effective value calculation
**test_frame_space_update_local_tx_update** - Tests local TX parameter updates

## Code Coverage

The tests cover:

### PDU Encoding/Decoding
- `llcp_pdu_encode_fsu_req()` - Encodes FSU request PDU
- `llcp_pdu_encode_fsu_rsp()` - Encodes FSU response PDU
- `llcp_pdu_decode_fsu_req()` - Decodes FSU request PDU
- `llcp_pdu_decode_fsu_rsp()` - Decodes FSU response PDU
- `llcp_ntf_encode_fsu_change()` - Encodes FSU change notification

### FSU Management Functions
- `ull_fsu_init()` - Initializes FSU with default values
- `ull_fsu_local_tx_update()` - Updates local FSU parameters before TX
- `ull_fsu_update_eff()` - Applies effective FSU values to timing registers
- `ull_fsu_update_eff_from_local()` - Calculates effective values from local

### LLCP Procedure Handlers
- `ull_cp_fsu()` - API to initiate local FSU procedure
- `lp_comm_tx()` - Transmits FSU request (local procedure)
- `lp_comm_complete()` - Handles FSU response completion
- `lp_comm_ntf()` - Generates FSU change notification
- `llcp_rp_comm_rx()` - Receives remote FSU request
- `llcp_rp_comm_run()` - Processes remote FSU procedure

### Timing Register Updates
- `tifs_tx_us` - TX inter-frame spacing
- `tifs_rx_us` - RX inter-frame spacing  
- `tifs_cis_us` - CIS inter-frame spacing

### Per-PHY Storage
- Tests verify per-PHY FSU storage for 1M, 2M, and CODED PHYs
- Tests verify PHY-specific parameter application

## Feature Bit Note

The FSU feature bit (BT_LE_FEAT_BIT_FRAME_SPACE_UPDATE) is at bit position 65 in the Bluetooth specification, which requires support for extended feature bits beyond 64 bits. Until extended feature support is implemented in the Zephyr Bluetooth controller, the tests use bit 63 as a placeholder to enable FSU procedures.

This is documented in `ll_feat.h` with a FIXME comment.

## Test Helpers

New test helper functions added to support FSU testing:
- `helper_pdu_encode_fsu_req()` - Encodes test FSU request
- `helper_pdu_encode_fsu_rsp()` - Encodes test FSU response
- `helper_pdu_verify_fsu_req()` - Verifies FSU request PDU
- `helper_pdu_verify_fsu_rsp()` - Verifies FSU response PDU

## Running the Tests

The tests use the Zephyr unit test framework (ztest) and can be run using:

```bash
./scripts/twister -T tests/bluetooth/controller/ctrl_frame_space_update
```

Or build and run directly:

```bash
west build -b native_sim tests/bluetooth/controller/ctrl_frame_space_update
./build/zephyr/zephyr.exe
```

## Test Results

All tests validate:
- Correct PDU encoding/decoding
- Proper role-based timing updates (Central vs Peripheral)
- Correct direction handling (TX vs RX)
- Proper spacing type handling (ACL CP, ACL PC, CIS)
- Per-PHY storage and retrieval
- Feature negotiation and unknown response handling
- Effective value calculation with config minimums

## Related Files

- **Feature Implementation**: `subsys/bluetooth/controller/ll_sw/ull_conn.c`
- **LLCP Procedures**: `subsys/bluetooth/controller/ll_sw/ull_llcp_common.c`
- **PDU Handling**: `subsys/bluetooth/controller/ll_sw/ull_llcp_pdu.c`
- **Feature Definitions**: `subsys/bluetooth/controller/include/ll_feat.h`
- **PDU Definitions**: `subsys/bluetooth/controller/ll_sw/pdu.h`

## References

- Bluetooth Core Specification v5.4, Vol 6, Part B, Section 5.1.17 (Frame Space Update Procedure)
- Zephyr PR #99473: Initial Frame Space Update support
