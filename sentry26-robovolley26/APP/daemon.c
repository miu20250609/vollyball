#include "daemon.h"

#include "cmsis_os.h"

#define RGB_FLOW_COLOR_CHANGE_TIME  500
#define RGB_FLOW_COLOR_LENGTH   6
uint32_t RGB_flow_color[RGB_FLOW_COLOR_LENGTH + 1] = {0xFF0000FF, 0x0000FF00, 0xFFFF0000, 0x000000FF, 0xFF00FF00, 0x00FF0000, 0xFF0000FF};

static daemon_t g_daemon;
void Daemon_Init(void)
{
    LED_Init(&g_daemon.led_ins);
}

static void Daemon_AllOnline();
void Daemon_Task(void const * argument)
{
    Daemon_Init();
    while(1)
    {
        Daemon_AllOnline();
    }
}

static void Daemon_AllOnline()
{
    for(int i = 0; i < RGB_FLOW_COLOR_LENGTH; i++)
    {
        fp32 alpha = (fp32)((RGB_flow_color[i] & 0xFF000000) >> 24);
        fp32 red = (fp32)((RGB_flow_color[i] & 0x00FF0000) >> 16);
        fp32 green = (fp32)((RGB_flow_color[i] & 0x0000FF00) >> 8);
        fp32 blue = (fp32)((RGB_flow_color[i] & 0x000000FF) >> 0);

        fp32 delta_alpha = (fp32)((RGB_flow_color[i + 1] & 0xFF000000) >> 24) - (fp32)((RGB_flow_color[i] & 0xFF000000) >> 24);
        fp32 delta_red = (fp32)((RGB_flow_color[i + 1] & 0x00FF0000) >> 16) - (fp32)((RGB_flow_color[i] & 0x00FF0000) >> 16);
        fp32 delta_green = (fp32)((RGB_flow_color[i + 1] & 0x0000FF00) >> 8) - (fp32)((RGB_flow_color[i] & 0x0000FF00) >> 8);
        fp32 delta_blue = (fp32)((RGB_flow_color[i + 1] & 0x000000FF) >> 0) - (fp32)((RGB_flow_color[i] & 0x000000FF) >> 0);

        delta_alpha /= RGB_FLOW_COLOR_CHANGE_TIME;
        delta_red /= RGB_FLOW_COLOR_CHANGE_TIME;
        delta_green /= RGB_FLOW_COLOR_CHANGE_TIME;
        delta_blue /= RGB_FLOW_COLOR_CHANGE_TIME;
        for(int j = 0; j < RGB_FLOW_COLOR_CHANGE_TIME; j++)
        {
            alpha += delta_alpha;
            red += delta_red;
            green += delta_green;
            blue += delta_blue;

            const uint32_t aRGB = ((uint32_t)(alpha)) << 24 | ((uint32_t)(red)) << 16 | ((uint32_t)(green)) << 8 | ((uint32_t)(blue))
                << 0;
            LED_SetColor(&g_daemon.led_ins, aRGB);
            osDelay(3);
        }
    }
}