#ifndef DM8009P_H
#define DM8009P_H

#include "can/bsp_can.h"

#define DM8009P_MIT_BASE_ID     0x000   
#define DM8009P_POSVEL_BASE_ID  0x100  
#define DM8009P_VEL_BASE_ID     0x200  

#define DM8009P_KP_MAX          500.0f
#define DM8009P_KD_MAX          5.0f

#define DM8009P_POS_FACTOR      (2.0f * 3.1415926535f / 65536.0f)   
#define DM8009P_VEL_FACTOR      (1.0f)                               
#define DM8009P_TORQUE_FACTOR   (1.0f)                             

typedef enum {
    DM8009P_MODE_MIT = 0,
    DM8009P_MODE_POS_VEL,
    DM8009P_MODE_VEL
} DM8009P_Mode_t;

typedef enum {
    DM8009P_OK = 0,
    DM8009P_ERROR_INVALID_PARAM = 1,
    DM8009P_ERROR_NOT_INIT = 2,
    DM8009P_ERROR_CAN_TX = 3,
    DM8009P_ERROR_INVALID_MODE = 4
} DM8009P_Status_t;

typedef struct {
    uint16_t pos_raw;   
    int16_t  vel_raw;   
    int16_t  tor_raw;  
    uint8_t  mos_temp;   
    uint8_t  rotor_temp; 
    uint8_t  err;       
} dm8009p_ecd_t;

typedef struct {
    CAN_Instance_t     *can_ins;    
    uint8_t            motor_id; 
    DM8009P_Mode_t     mode;     
    dm8009p_ecd_t      ecd;
} dm8009p_instance_t;

DM8009P_Status_t dm8009p_init(dm8009p_instance_t *ins, CAN_HandleTypeDef *hcan, uint8_t motor_id);

DM8009P_Status_t dm8009p_set_mode(dm8009p_instance_t *ins, DM8009P_Mode_t mode);

DM8009P_Status_t dm8009p_ctrl_mit(const dm8009p_instance_t *ins,
                                  float p_des, float v_des,
                                  float kp, float kd, float t_ff);

DM8009P_Status_t dm8009p_ctrl_pos_vel(const dm8009p_instance_t *ins,
                                      float p_des, float v_des);

DM8009P_Status_t dm8009p_ctrl_vel(const dm8009p_instance_t *ins, float v_des);

#endif /* DM8009P_H */