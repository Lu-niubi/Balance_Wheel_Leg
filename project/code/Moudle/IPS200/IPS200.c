#include "IPS200.h"
#include "stdio.h"
// IPS200 显示模块封装
// 底层由 zf_device_ips200 驱动实现（并口 8080 模式）
// GNSS 数据来源于 zf_device_gnss 的全局结构体 gnss

// IPS200_Init: 初始化 IPS200 屏幕
void IPS200_Init(void)
{
    ips200_init(IPS200_MODULE_TYPE);
}

// IPS200_ShowGNSS: 将当前 gnss 结构体数据刷新到屏幕
// 布局（行高16像素）:
//   Row 0: 年  月  日
//   Row 1: 时  分  秒
//   Row 2: 定位状态  纬度
//   Row 3: 经度      速度
//   Row 4: 航向角    卫星数
//   Row 5: 海拔高度
void IPS200_ShowGNSS(void)
{
    ips200_show_uint(  0, 16*0, gnss.time.year,   4);
    ips200_show_uint( 80, 16*0, gnss.time.month,  2);
    ips200_show_uint(160, 16*0, gnss.time.day,    2);

    ips200_show_uint(  0, 16*1, gnss.time.hour,   2);
    ips200_show_uint( 80, 16*1, gnss.time.minute, 2);
    ips200_show_uint(160, 16*1, gnss.time.second, 2);

    ips200_show_uint(  0, 16*2, gnss.state,         5);
    ips200_show_float(120, 16*2, gnss.latitude,  4, 6);

    ips200_show_float(  0, 16*3, gnss.longitude, 4, 6);
    ips200_show_float(120, 16*3, gnss.speed,     4, 6);

    ips200_show_float(  0, 16*4, gnss.direction, 4, 6);
    ips200_show_uint(120, 16*4, gnss.satellite_used, 5);

    ips200_show_float(  0, 16*5, gnss.height,    4, 6);
    printf("GNSS: %d-%d-%d %d:%d:%d state=%d lat=%d.%d.%d lon=%d.%d.%d spd=%.2f dir=%.2f sat=%d alt=%.2f\n",
           gnss.time.year, gnss.time.month, gnss.time.day,
           gnss.time.hour, gnss.time.minute, gnss.time.second,
           gnss.state,
           gnss.latitude_degree, gnss.latitude_cent, gnss.latitude_second,
           gnss.longitude_degree, gnss.longitude_cent, gnss.longitude_second,
           gnss.speed,
           gnss.direction,
           gnss.satellite_used,
           gnss.height);

}
