#include "vesc.h"

static vesc_instance_t vesc[VESC_CNTMAX];

void vesc_callback(const uint8_t* rx_data, uint32_t id, void* arg);
void vesc_init(const uint32_t mcuid, CAN_HandleTypeDef *hcan)
{
    uint8_t cnt = 0;
    for (uint8_t i = 0; i < VESC_CNTMAX; i++)
        if (!vesc[i].init)
        {
            cnt = i;
            break;
        }
    vesc[cnt].can_instance = BSP_CAN_Init(hcan);
    vesc[cnt].mcuid = mcuid;
    const uint32_t id = vesc[cnt].mcuid | CAN_PACKET_STATUS << 8;
    BSP_CAN_RegisterExtCallback(vesc[cnt].can_instance, id, 0x1FFFFFFF, vesc_callback, &vesc[cnt].packet);
    vesc[cnt].init = 1;
}

void vesc_current_ctrl(const vesc_instance_t* vesc_ins, const fp32 current)
{
    if (!vesc_ins->init) return;
    const CAN_Instance_t* can_ins = vesc_ins->can_instance;
    int32_t cur_temp = (int32_t)(current * 1000);
    uint8_t txdata[4];
    txdata[0] = ((uint8_t *)&cur_temp)[3];
    txdata[1] = ((uint8_t *)&cur_temp)[2];
    txdata[2] = ((uint8_t *)&cur_temp)[1];
    txdata[3] = ((uint8_t *)&cur_temp)[0];
    const uint32_t id = vesc_ins->mcuid | CAN_PACKET_SET_CURRENT << 8;
    BSP_CAN_Transmit(can_ins, id, CAN_ID_EXT, txdata, 4);
}

void vesc_speed_ctrl(const vesc_instance_t* vesc_ins, int32_t erpm)
{
    if (!vesc_ins->init) return;
    const CAN_Instance_t* can_ins = vesc_ins->can_instance;
    uint8_t txdata[4];
    txdata[0] = ((uint8_t *)&erpm)[3];
    txdata[1] = ((uint8_t *)&erpm)[2];
    txdata[2] = ((uint8_t *)&erpm)[1];
    txdata[3] = ((uint8_t *)&erpm)[0];
    const uint32_t id = vesc_ins->mcuid | CAN_PACKET_SET_RPM << 8;
    BSP_CAN_Transmit(can_ins, id, CAN_ID_EXT, txdata, 4);
}

void vesc_pos_ctrl(const vesc_instance_t* vesc_ins, const fp32 angle)
{
    if (!vesc_ins->init) return;
    const CAN_Instance_t* can_ins = vesc_ins->can_instance;
    int32_t pos = (int32_t)(angle * 1000000.0f);
    uint8_t txdata[4];
    txdata[0] = ((uint8_t *)&pos)[3];
    txdata[1] = ((uint8_t *)&pos)[2];
    txdata[2] = ((uint8_t *)&pos)[1];
    txdata[3] = ((uint8_t *)&pos)[0];
    const uint32_t id = vesc_ins->mcuid | CAN_PACKET_SET_POS << 8;
    BSP_CAN_Transmit(can_ins, id, CAN_ID_EXT, txdata, 4);
}

void vesc_packet_1_handler(const uint8_t* rx_data, vesc_packet_t* vesc_packet);
void vesc_callback(const uint8_t* rx_data, uint32_t id, void* arg)
{
    const uint16_t packet_id = (CAN_PACKET_ID)((id >> 8) & 0xFF);
    vesc_packet_t *packet = (vesc_packet_t *)arg;
    switch (packet_id)
    {
        case CAN_PACKET_STATUS:
            vesc_packet_1_handler(rx_data, packet);
            break;
        default:
            break;
    }
}

void vesc_packet_1_handler(const uint8_t* rx_data, vesc_packet_t* vesc_packet)
{
    vesc_packet->packet_1.erpm = (rx_data[0] << 24 | (rx_data[1] << 16) | (rx_data[2] << 8) | (rx_data[3]));
    vesc_packet->packet_1.current = (int16_t)(((rx_data[4] << 8) & 0xFF) | (rx_data[5] & 0xFF));
    vesc_packet->packet_1.duty = (int16_t)((rx_data[6] << 8) | (rx_data[7]));
}

vesc_instance_t* Get_Vesc_Ptr(const uint32_t mcuid)
{
    for (uint8_t i = 0; i < VESC_CNTMAX; i++)
        if (vesc[i].init && vesc[i].mcuid == mcuid)
            return &vesc[i];
    return NULL;
}