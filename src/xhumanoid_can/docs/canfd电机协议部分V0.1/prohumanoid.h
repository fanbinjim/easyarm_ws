#ifndef PROHUMANOID_H
#define PROHUMANOID_H

#include <QObject>
#include <QDebug>
#include "canthread.h"
#include "gb_para.h"

// 提前声明结构体（避免重复定义，若其他文件已定义可删除）
typedef struct
{
    uint32_t id;
    UINT channel;
    uint8_t buff[64];
    uint32_t len;
} CanFd_data;

// 指令码枚举（替换魔法数字，提升可读性）
enum class CmdCode : uint8_t
{
    // 关节基本设置
    QueryId     = 0x00,
    ResetId     = 0x01,
    SetId       = 0x02,
    SetZero     = 0x03,
    FanCtl      = 0x04,
    CpuId       = 0x05,

    // 电机控制模式
    MotorCtl    = 0x10,
    FpMix       = 0x11,
    Position    = 0x12,
    Speed       = 0x13,
    Current     = 0x14,
    VfMode      = 0x15,
    VfModeAngle = 0x16,

    // 关节升级（指令码独立，避免冲突）
    VerQuery    = 0x01,
    UpRequest   = 0x02,
    UpInfo      = 0x03,
    UpData      = 0x04,
    UpDataEnd   = 0x05,
    UpDevReset  = 0x06,
    UpEndQuery  = 0x07,

};

// 全局常量（集中管理，方便修改）
namespace CanConst
{
    constexpr uint8_t MAX_BUFF_LEN    = 64;    // 缓冲区最大长度（匹配CanFd_data::buff）
    constexpr uint32_t MIN_UPGRADE_ID = 0x400; // 升级功能最小ID阈值
    constexpr uint8_t MAX_UP_DATA_LEN = 12;    // 升级数据最大长度
}

class prohumanoid : public QObject
{
    Q_OBJECT // 必须添加，支持信号槽（若无需信号可省略，但建议保留）
public:
    explicit prohumanoid(QObject *parent = nullptr);
    ~prohumanoid();

    // ===== 关节基本设置 =====
    void QueryId(CanFd_data *cf);
    void ResetId(CanFd_data *cf);
    void SetId(CanFd_data *cf, uint32_t id, uint8_t OldId, uint8_t NewId);
    void SetZero(CanFd_data *cf, uint32_t id);

    // ===== 电机控制设置 =====
    void MotorCtl(CanFd_data *cf, uint32_t id, uint8_t sta); // sta:0=启动,1=关闭     //使能
    void FpMix(CanFd_data *cf, uint32_t id, uint16_t Kp, uint16_t Kd, float posi, float spd, uint16_t Ftoque, uint8_t inc);   //力位混控
    void Position(CanFd_data *cf, uint32_t id, float posi, float spd, float cur, uint8_t inc);    //位置模式
    void Speed(CanFd_data *cf, uint32_t id, float spd, float cur, uint8_t inc);   //速度模式
    void Current(CanFd_data *cf, uint32_t id, float cur, uint8_t inc);   //电流模式

    // ===== 关节升级 =====
    void VerQuery(CanFd_data *cf, uint32_t id);    //版本查询
    void UpRequest(CanFd_data *cf, uint32_t id, uint32_t ver);   //升级请求
    void UpInfo(CanFd_data *cf, uint32_t id, uint16_t crc16, uint32_t sizebin);   //升级信息
    void UpData(CanFd_data *cf, uint32_t id, uint16_t binindex, uint8_t *bindata, uint8_t bindata_len);   //升级数据
    void UdataEnd(CanFd_data *cf, uint32_t id);   //升级结束
    void UdevReset(CanFd_data *cf, uint32_t id);   //设备重启
    void UgEnd(CanFd_data *cf, uint32_t id);   //升级结束

    
    void FanCtl(CanFd_data *cf, uint32_t id, uint8_t sta); // sta:0=关闭,1=开启    //风扇控制
    void CpuId(CanFd_data *cf, uint32_t id);    //芯片ID查询

    

signals:
    // 错误通知信号（上层可连接弹窗/日志，不影响原有逻辑）
    void sendError(const QString &errMsg);

private:
    // 联合体：float ↔ 4字节（大端序适配CAN协议）
    typedef union
    {
        uint8_t Byte[4];
        float ft;
        uint32_t u32t;
    } FloatToByte;

    // ===== 私有辅助函数（减少重复代码）=====
    // 检查CanFd_data和gbcan有效性（返回true=有效）
    bool checkValidity(CanFd_data *cf);
    // float转大端序字节数组
    void floatToBigEndian(float value, uint8_t *outBytes);
    // 检查缓冲区是否溢出（返回true=安全）
    bool checkBuffOverflow(uint8_t currentLen, uint8_t addLen = 1);

    // 统一CAN发送逻辑（封装重复的ID/通道设置+发送调用）
    bool sendCan(CanFd_data *cf, uint32_t id, CmdCode cmd, const uint8_t *data = nullptr, uint8_t dataLen = 0);
};

#endif // PROHUMANOID_H
