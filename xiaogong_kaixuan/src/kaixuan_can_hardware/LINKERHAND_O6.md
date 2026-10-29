# LinkerHand O6 integration

Master0 Slave7 (right hand) and Master1 Slave7 (left hand) can each be integrated
into their existing IgH master, domain, 1 ms realtime thread, and DC
synchronization. Slave0 remains the reference clock on each master. The original
seven CiA402 axes are unchanged. Either master can still start with only those
seven slaves; an O6 ROS interface is created only when the strict Slave7 identity
is present.

## ROS interface

- Right hand (Master0): `/linkerhand_o6/...`.
- Left hand (Master1): `/linkerhand_o6_left/...`.
- Each namespace has `enable` (`std_srvs/srv/SetBool`) for explicit
  enable/disable.
- `/linkerhand_o6/target_position` (`std_msgs/msg/UInt8MultiArray`): six uint8
  targets in thumb-flex, thumb-abduct, index, middle, ring, little order.
- `/linkerhand_o6/actual_position`, `temperature`, `joint_error`: six uint8 values.
- `/linkerhand_o6/pressure` (`UInt8MultiArray`): 200 uint8 pressure samples.
  Sample `(finger, row, column)` is at `finger * 40 + row * 4 + column`, where
  fingers 0..4 are thumb through little finger, rows are 0..9, and columns 0..3.
- `/linkerhand_o6/pressure_status` (`UInt16MultiArray`): pressure sequence,
  finger-valid mask, pressure state, and pressure age in milliseconds.
- `/linkerhand_o6/pressure_valid`: true when the finger mask is `0x1f`, state is
  2, and age is below 500 ms.
- `/linkerhand_o6/online`: EtherCAT OP/WC and O6 online state.
- `/linkerhand_o6/ready`: the enable handshake is ready and all O6 safety
  inputs are currently valid. It becomes false while a joint error is active
  and returns true automatically after the error clears if the handshake state
  remains ready.
- `/linkerhand_o6/status` (`UInt16MultiArray`): status word, applied sequence,
  position sequence, command sequence, valid mask, active CAN ID, CAN TX count,
  CAN RX count, CAN error count, timeout count, handshake state.

Every topic listed below `/linkerhand_o6` also exists below
`/linkerhand_o6_left` when the left O6 is connected. The normal test, latency,
and continuous pressure scripts accept `--namespace /linkerhand_o6_left`.

The gateway is disabled at process startup. A target is rejected until the
explicit current-position handshake is complete. The default direct mode allows
the full uint8 target range (`o6_max_step=255`) and disables the ROS command
timeout (`o6_command_timeout_ms=0`). One accepted target remains enabled and is
held in the 1 ms PDO loop with the same CommandSequence. Transient EtherCAT,
working-counter, or O6 status faults freeze the handshake while preserving the
last enabled PDO frame. Only an explicit `enable=false` request or hardware
lifecycle shutdown sends disabled frames. The EtherCAT process-data watchdog
remains enabled and can still disable the gateway if cyclic PDO delivery stops.
Pressure scanning is requested independently with RequestMask bit 2 and does not
change the position enable handshake. O6 requires a 1,000,000 ns master cycle;
configuration is rejected when a different cycle is selected.

## Safe checks

```bash
python3 /home/agxorin/cxl/kaixuan/xiaogong_kaixuan/src/normal/test_linkerhand_o6.py
python3 /home/agxorin/cxl/kaixuan/xiaogong_kaixuan/src/normal/test_linkerhand_o6.py --baseline
python3 /home/agxorin/cxl/kaixuan/xiaogong_kaixuan/src/normal/test_linkerhand_o6.py --baseline --exercise-index
```

The first command is monitor-only. The second enables only at the reported
current position and leaves O6 enabled. The third is the explicit opt-in index
-5/restore test and also leaves O6 enabled. Add `--disable-on-exit` when explicit
disable-on-exit behavior is required.
