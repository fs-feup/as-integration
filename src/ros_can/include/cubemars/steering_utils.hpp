#ifndef STEERING_UTILS
#define STEERING_UTILS

// void comm_can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len) {
//     uint8_t i=0;
//     if (len > 8) {
//         len = 8;
//     }
//     CanTxMsg TxMessage;
//     TxMessage.StdId = 0;
//     TxMessage.IDE = CAN_ID_EXT;
//     TxMessage.ExtId = id;
//     TxMessage.RTR = CAN_RTR_DATA;
//     TxMessage.DLC = len;
//     //memcpy(txmsg.data8, data, len);
//     for(i=0;i<len;i++)
//         TxMessage.Data[i]=data[i];
//     CAN_Transmit(CHASSIS_CAN, &TxMessage);
// }

// void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index) {
//     buffer[(*index)++] = number >> 24;
//     buffer[(*index)++] = number >> 16;
//     buffer[(*index)++] = number >> 8;
//     buffer[(*index)++] = number;
// }

// void buffer_append_int16(uint8_t* buffer, int16_t number, int16_t *index) {
//     buffer[(*index)++] = number >> 8;
//     buffer[(*index)++] = number;
// }

// void comm_can_set_pos(uint8_t controller_id, float pos) {
//     int32_t send_index = 0;
//     uint8_t buffer[4];
//     buffer_append_int32(buffer, (int32_t)(pos * 1000000.0), &send_index);
//     comm_can_transmit_eid(controller_id |
//     ((uint32_t)CAN_PACKET_SET_POS << 8), buffer, send_index);
// }

void send_steering_angle_command(char controller_id, float angle, unsigned int* send_id, char* buffer) {
    int send_index = 0;
    int converted_angle = static_cast<int>((angle * 1000000.0)); // Indicated by documentation
    for (unsigned int i = 3; i >= 0; i--) {
        buffer[i] = converted_angle >> (8 * i);
    }
    send_index = controller_id | static_cast<unsigned int>(4 << 8);
}

#endif // STEERING_UTILS_HPP