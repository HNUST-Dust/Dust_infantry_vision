# Gimbal Protocol Facts

- Configuration keys are `com_port` and optional `skip_cboard_crc`; the documented default is `/dev/gimbal`.
- `GimbalToVision` and `VisionToGimbal` are packed structs with a two-byte `SP` header and a trailing CRC16. Receive quaternion order is `wxyz`.
- `Gimbal::read_thread()` scans for the header, reads the remaining packed frame, checks CRC, queues quaternion/timestamp pairs, and updates `GimbalState` under a mutex.
- `Gimbal::q(t)` interpolates queued quaternions with slerp. Invalid or sparse timestamps can look like calibration or control errors.
- State yaw, pitch and angular velocity are stored in radians/radians per second even though the incoming velocity fields are degrees per second.
- The production entry points send mode `0` for idle, `1` for control without fire, and `2` for control plus fire.

## Non-destructive commands

```bash
stat -c '%A %U:%G %n' /dev/gimbal
ls -l build-ninja/gimbal_test build-ninja/gimbal_response_test
./build-ninja/gimbal_test -h
./build-ninja/gimbal_response_test -h
```

These commands do not prove the protocol works; they only establish safe prerequisites and CLI availability.
