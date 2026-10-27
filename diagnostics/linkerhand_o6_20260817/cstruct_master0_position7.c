/* Master 0, Slave 7, "O6-ECAT-CAN"
 * Vendor ID:       0x00000000
 * Product code:    0x00000601
 * Revision number: 0x00010000
 */

ec_pdo_entry_info_t slave_7_pdo_entries[] = {
    {0x7000, 0x01, 16}, /* ControlWord */
    {0x7000, 0x02, 16}, /* CommandSequence */
    {0x7000, 0x03, 8}, /* UpdateMask */
    {0x7000, 0x04, 8}, /* RequestMask */
    {0x7000, 0x05, 16}, /* RxReserved */
    {0x7010, 0x01, 8}, /* TargetPositionThumbBend */
    {0x7010, 0x02, 8}, /* TargetPositionThumbSwing */
    {0x7010, 0x03, 8}, /* TargetPositionIndexBend */
    {0x7010, 0x04, 8}, /* TargetPositionMiddleBend */
    {0x7010, 0x05, 8}, /* TargetPositionRingBend */
    {0x7010, 0x06, 8}, /* TargetPositionLittleBend */
    {0x7020, 0x01, 8}, /* TorqueLimitThumbBend */
    {0x7020, 0x02, 8}, /* TorqueLimitThumbSwing */
    {0x7020, 0x03, 8}, /* TorqueLimitIndexBend */
    {0x7020, 0x04, 8}, /* TorqueLimitMiddleBend */
    {0x7020, 0x05, 8}, /* TorqueLimitRingBend */
    {0x7020, 0x06, 8}, /* TorqueLimitLittleBend */
    {0x7030, 0x01, 8}, /* SpeedLimitThumbBend */
    {0x7030, 0x02, 8}, /* SpeedLimitThumbSwing */
    {0x7030, 0x03, 8}, /* SpeedLimitIndexBend */
    {0x7030, 0x04, 8}, /* SpeedLimitMiddleBend */
    {0x7030, 0x05, 8}, /* SpeedLimitRingBend */
    {0x7030, 0x06, 8}, /* SpeedLimitLittleBend */
    {0x7040, 0x01, 8}, /* AccelerationThumbBend */
    {0x7040, 0x02, 8}, /* AccelerationThumbSwing */
    {0x7040, 0x03, 8}, /* AccelerationIndexBend */
    {0x7040, 0x04, 8}, /* AccelerationMiddleBend */
    {0x7040, 0x05, 8}, /* AccelerationRingBend */
    {0x7040, 0x06, 8}, /* AccelerationLittleBend */
    {0x6000, 0x01, 16}, /* StatusWord */
    {0x6000, 0x02, 16}, /* AppliedCommandSequence */
    {0x6000, 0x03, 16}, /* PositionSequence */
    {0x6000, 0x04, 8}, /* ValidMask */
    {0x6000, 0x05, 8}, /* ActiveCanId */
    {0x6000, 0x06, 8}, /* ActualPositionThumbBend */
    {0x6000, 0x07, 8}, /* ActualPositionThumbSwing */
    {0x6000, 0x08, 8}, /* ActualPositionIndexBend */
    {0x6000, 0x09, 8}, /* ActualPositionMiddleBend */
    {0x6000, 0x0a, 8}, /* ActualPositionRingBend */
    {0x6000, 0x0b, 8}, /* ActualPositionLittleBend */
    {0x6000, 0x0c, 8}, /* TemperatureThumbBend */
    {0x6000, 0x0d, 8}, /* TemperatureThumbSwing */
    {0x6000, 0x0e, 8}, /* TemperatureIndexBend */
    {0x6000, 0x0f, 8}, /* TemperatureMiddleBend */
    {0x6000, 0x10, 8}, /* TemperatureRingBend */
    {0x6000, 0x11, 8}, /* TemperatureLittleBend */
    {0x6000, 0x12, 8}, /* JointErrorThumbBend */
    {0x6000, 0x13, 8}, /* JointErrorThumbSwing */
    {0x6000, 0x14, 8}, /* JointErrorIndexBend */
    {0x6000, 0x15, 8}, /* JointErrorMiddleBend */
    {0x6000, 0x16, 8}, /* JointErrorRingBend */
    {0x6000, 0x17, 8}, /* JointErrorLittleBend */
    {0x6000, 0x18, 16}, /* CanTxCount */
    {0x6000, 0x19, 16}, /* CanRxCount */
    {0x6000, 0x1a, 8}, /* CanErrorCount */
    {0x6000, 0x1b, 8}, /* O6TimeoutCount */
    {0x60f0, 0x01, 16}, /* PressureSequence */
    {0x60f0, 0x02, 8}, /* FingerValidMask */
    {0x60f0, 0x03, 8}, /* PressureState */
    {0x6100, 0x01, 32}, /* Row0 */
    {0x6100, 0x02, 32}, /* Row1 */
    {0x6100, 0x03, 32}, /* Row2 */
    {0x6100, 0x04, 32}, /* Row3 */
    {0x6100, 0x05, 32}, /* Row4 */
    {0x6100, 0x06, 32}, /* Row5 */
    {0x6100, 0x07, 32}, /* Row6 */
    {0x6100, 0x08, 32}, /* Row7 */
    {0x6100, 0x09, 32}, /* Row8 */
    {0x6100, 0x0a, 32}, /* Row9 */
    {0x6101, 0x01, 32}, /* Row0 */
    {0x6101, 0x02, 32}, /* Row1 */
    {0x6101, 0x03, 32}, /* Row2 */
    {0x6101, 0x04, 32}, /* Row3 */
    {0x6101, 0x05, 32}, /* Row4 */
    {0x6101, 0x06, 32}, /* Row5 */
    {0x6101, 0x07, 32}, /* Row6 */
    {0x6101, 0x08, 32}, /* Row7 */
    {0x6101, 0x09, 32}, /* Row8 */
    {0x6101, 0x0a, 32}, /* Row9 */
    {0x6102, 0x01, 32}, /* Row0 */
    {0x6102, 0x02, 32}, /* Row1 */
    {0x6102, 0x03, 32}, /* Row2 */
    {0x6102, 0x04, 32}, /* Row3 */
    {0x6102, 0x05, 32}, /* Row4 */
    {0x6102, 0x06, 32}, /* Row5 */
    {0x6102, 0x07, 32}, /* Row6 */
    {0x6102, 0x08, 32}, /* Row7 */
    {0x6102, 0x09, 32}, /* Row8 */
    {0x6102, 0x0a, 32}, /* Row9 */
    {0x6103, 0x01, 32}, /* Row0 */
    {0x6103, 0x02, 32}, /* Row1 */
    {0x6103, 0x03, 32}, /* Row2 */
    {0x6103, 0x04, 32}, /* Row3 */
    {0x6103, 0x05, 32}, /* Row4 */
    {0x6103, 0x06, 32}, /* Row5 */
    {0x6103, 0x07, 32}, /* Row6 */
    {0x6103, 0x08, 32}, /* Row7 */
    {0x6103, 0x09, 32}, /* Row8 */
    {0x6103, 0x0a, 32}, /* Row9 */
    {0x6104, 0x01, 32}, /* Row0 */
    {0x6104, 0x02, 32}, /* Row1 */
    {0x6104, 0x03, 32}, /* Row2 */
    {0x6104, 0x04, 32}, /* Row3 */
    {0x6104, 0x05, 32}, /* Row4 */
    {0x6104, 0x06, 32}, /* Row5 */
    {0x6104, 0x07, 32}, /* Row6 */
    {0x6104, 0x08, 32}, /* Row7 */
    {0x6104, 0x09, 32}, /* Row8 */
    {0x6104, 0x0a, 32}, /* Row9 */
    {0x60f0, 0x04, 16}, /* PressureAgeMs */
    {0x60f0, 0x05, 16}, /* PressureReserved */
};

ec_pdo_info_t slave_7_pdos[] = {
    {0x1600, 5, slave_7_pdo_entries + 0}, /* Gateway command header */
    {0x1601, 6, slave_7_pdo_entries + 5}, /* TargetPosition */
    {0x1602, 6, slave_7_pdo_entries + 11}, /* TorqueLimit */
    {0x1603, 6, slave_7_pdo_entries + 17}, /* SpeedLimit */
    {0x1604, 6, slave_7_pdo_entries + 23}, /* Acceleration */
    {0x1a00, 27, slave_7_pdo_entries + 29}, /* Gateway fast status */
    {0x1a01, 55, slave_7_pdo_entries + 56}, /* Five-finger pressure */
};

ec_sync_info_t slave_7_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 5, slave_7_pdos + 0, EC_WD_DISABLE},
    {3, EC_DIR_INPUT, 2, slave_7_pdos + 5, EC_WD_DISABLE},
    {0xff}
};

