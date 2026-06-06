#include "prohumanoid.h"

using namespace CanConst;

prohumanoid::prohumanoid(QObject *parent)
    : QObject(parent)
{
    // 保持原有逻辑：不创建局部CANThread，依赖全局GbPara::gbcan
}

prohumanoid::~prohumanoid()
{
    // 无动态分配资源，保持原有逻辑
}

// ===== 私有辅助函数实现 =====
bool prohumanoid::checkValidity(CanFd_data *cf)
{
    // 检查CanFd_data指针非空
    if (!cf)
    {
        QString err = "CanFd_data is null pointer!";
        qCritical() << "[prohumanoid] " << err;
        emit sendError(err);
        return false;
    }

    // 检查全局gbcan初始化完成
    if (!GbPara::instance().CanfdHandle)
    {
        QString err = "Global CANThread (gbcan) not initialized!";
        qCritical() << "[prohumanoid] " << err;
        emit sendError(err);
        return false;
    }

    return true;
}

void prohumanoid::floatToBigEndian(float value, uint8_t *outBytes)
{
    if (!outBytes) return;

    FloatToByte ftb;
    ftb.ft = value;
    // 协议要求大端序：高位字节在前（Byte[3]是最高位）
    outBytes[0] = ftb.Byte[3];
    outBytes[1] = ftb.Byte[2];
    outBytes[2] = ftb.Byte[1];
    outBytes[3] = ftb.Byte[0];
}

bool prohumanoid::checkBuffOverflow(uint8_t currentLen, uint8_t addLen)
{
    if (currentLen + addLen > MAX_BUFF_LEN)
    {
        QString err = QString("Buffer overflow! current=%1, add=%2, max=%3")
                        .arg(currentLen).arg(addLen).arg(MAX_BUFF_LEN);
        qCritical() << "[prohumanoid] " << err;
        emit sendError(err);
        return false;
    }
    return true;
}

bool prohumanoid::sendCan(CanFd_data *cf, uint32_t id, CmdCode cmd, const uint8_t *data, uint8_t dataLen)
{
    // 前置校验：指针+缓冲区安全
    if (!checkValidity(cf) || !checkBuffOverflow(1 + dataLen))
        return false;

    // 初始化CAN数据结构
    cf->id = id;
    cf->channel = GbPara::instance().Canfdlink.channel;
    cf->len = 0;

    // 填充指令码（第1字节）
    cf->buff[cf->len++] = static_cast<uint8_t>(cmd);

    // 填充数据段（若有）
    if (data && dataLen > 0)
    {
        memcpy(&cf->buff[cf->len], data, dataLen);
        cf->len += dataLen;
    }

    // 发送CAN数据（返回发送结果）
    bool sendOk = GbPara::instance().CanfdHandle->canfd_send(
        cf->id, cf->channel, reinterpret_cast<char*>(cf->buff), cf->len
    );

    if (!sendOk)
    {
        QString err = QString("CAN send failed! id=0x%1, cmd=0x%2")
                        .arg(id, 8, 16, QChar('0')).arg(static_cast<uint8_t>(cmd), 2, 16, QChar('0'));
        qWarning() << "[prohumanoid] " << err;
        emit sendError(err);
    }

    return sendOk;
}

// ===== 关节基本设置实现 =====
void prohumanoid::QueryId(CanFd_data *cf)
{
    uint8_t data[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    sendCan(cf, 0, CmdCode::QueryId, data, 7);
}

void prohumanoid::ResetId(CanFd_data *cf)
{
    uint8_t data[7] = {0xD5, 0xA3, 0xF1, 0xE8, 0xC2, 0xB7, 0x34};
    sendCan(cf, 0, CmdCode::ResetId, data, 7);
}

void prohumanoid::SetId(CanFd_data *cf, uint32_t id, uint8_t OldId, uint8_t NewId)
{
    uint8_t data[2] = {OldId, NewId};
    sendCan(cf, id, CmdCode::SetId, data, 2);
}

void prohumanoid::FanCtl(CanFd_data *cf, uint32_t id, uint8_t sta)
{
    sendCan(cf, id, CmdCode::FanCtl, &sta, 1);
}

void prohumanoid::CpuId(CanFd_data *cf, uint32_t id)
{
    sendCan(cf, id, CmdCode::CpuId, nullptr, 0);
}

void prohumanoid::SetZero(CanFd_data *cf, uint32_t id)
{
    sendCan(cf, id, CmdCode::SetZero, nullptr, 0);
}

// ===== 电机控制设置实现 =====
void prohumanoid::MotorCtl(CanFd_data *cf, uint32_t id, uint8_t sta)
{
    sendCan(cf, id, CmdCode::MotorCtl, &sta, 1);
}

void prohumanoid::FpMix(CanFd_data *cf, uint32_t id, uint16_t Kp, uint16_t Kd, float posi, float spd, uint16_t Ftoque, uint8_t inc)
{
    uint8_t data[14] = {0};
    uint8_t idx = 0;

    // Kp（大端序）
    data[idx++] = (Kp >> 8) & 0xFF;
    data[idx++] = Kp & 0xFF;
    // Kd（大端序）
    data[idx++] = (Kd >> 8) & 0xFF;
    data[idx++] = Kd & 0xFF;
    // 位置（大端序）
    floatToBigEndian(posi, &data[idx]);
    idx += 4;
    // 速度（大端序）
    floatToBigEndian(spd, &data[idx]);
    idx += 4;
    // 力矩（大端序）
    data[idx++] = (Ftoque >> 8) & 0xFF;
    data[idx++] = Ftoque & 0xFF;
    // 增量模式
    data[idx++] = inc;

    sendCan(cf, id, CmdCode::FpMix, data, idx);
}

void prohumanoid::Position(CanFd_data *cf, uint32_t id, float posi, float spd, float cur, uint8_t inc)
{
    uint8_t data[13] = {0};
    uint8_t idx = 0;

    floatToBigEndian(posi, &data[idx]); idx += 4; // 位置
    floatToBigEndian(spd, &data[idx]); idx += 4;  // 速度
    floatToBigEndian(cur, &data[idx]); idx += 4;  // 电流
    data[idx++] = inc;                            // 增量模式

    sendCan(cf, id, CmdCode::Position, data, idx);
}

void prohumanoid::Speed(CanFd_data *cf, uint32_t id, float spd, float cur, uint8_t inc)
{
    uint8_t data[9] = {0};
    uint8_t idx = 0;

    floatToBigEndian(spd, &data[idx]); idx += 4; // 速度
    floatToBigEndian(cur, &data[idx]); idx += 4; // 电流
    data[idx++] = inc;                            // 增量模式

    sendCan(cf, id, CmdCode::Speed, data, idx);
}

void prohumanoid::Current(CanFd_data *cf, uint32_t id, float cur, uint8_t inc)
{
    uint8_t data[5] = {0};
    uint8_t idx = 0;

    floatToBigEndian(cur, &data[idx]); idx += 4; // 电流
    data[idx++] = inc;                            // 增量模式

    sendCan(cf, id, CmdCode::Current, data, idx);
}


// ===== 关节升级实现 =====
void prohumanoid::VerQuery(CanFd_data *cf, uint32_t id)
{
    if (id < MIN_UPGRADE_ID)
    {
        QString err = QString("Invalid upgrade ID: 0x%1 (min 0x%2)").arg(id, 8, 16).arg(MIN_UPGRADE_ID, 8, 16);
        qWarning() << "[prohumanoid] " << err;
        emit sendError(err);
        return;
    }
    sendCan(cf, id, CmdCode::VerQuery, nullptr, 0);
}

void prohumanoid::UpRequest(CanFd_data *cf, uint32_t id, uint32_t ver)
{
    if (id < MIN_UPGRADE_ID)
    {
        emit sendError(QString("Invalid upgrade ID: 0x%1").arg(id, 8, 16));
        return;
    }

    uint8_t data[4] = {0};
    // 版本号（大端序）
    data[0] = (ver >> 24) & 0xFF;
    data[1] = (ver >> 16) & 0xFF;
    data[2] = (ver >> 8) & 0xFF;
    data[3] = ver & 0xFF;

    sendCan(cf, id, CmdCode::UpRequest, data, 4);
}

void prohumanoid::UpInfo(CanFd_data *cf, uint32_t id, uint16_t crc16, uint32_t sizebin)
{
    if (id < MIN_UPGRADE_ID)
    {
        emit sendError(QString("Invalid upgrade ID: 0x%1").arg(id, 8, 16));
        return;
    }

    uint8_t data[6] = {0};
    uint8_t idx = 0;

    // CRC16（大端序）
    data[idx++] = (crc16 >> 8) & 0xFF;
    data[idx++] = crc16 & 0xFF;
    // 固件大小（原代码i从1开始，可能少传1字节，此处保持原逻辑；若需完整4字节，可改为idx从0循环4次）
    FloatToByte ftb;
    ftb.u32t = sizebin;
    for (int i = 1; i < 4; i++)
    {
        data[idx++] = ftb.Byte[3 - i];
    }

    sendCan(cf, id, CmdCode::UpInfo, data, idx);
}

void prohumanoid::UpData(CanFd_data *cf, uint32_t id, uint16_t binindex, uint8_t *bindata, uint8_t bindata_len)
{
    if (id < MIN_UPGRADE_ID)
    {
        emit sendError(QString("Invalid upgrade ID: 0x%1").arg(id, 8, 16));
        return;
    }
    if (!bindata)
    {
        emit sendError("Upgrade data is null pointer!");
        return;
    }
    if (bindata_len > MAX_UP_DATA_LEN)
    {
        emit sendError(QString("Upgrade data too long: %1 (max %2)").arg(bindata_len).arg(MAX_UP_DATA_LEN));
        return;
    }

    uint8_t data[2 + MAX_UP_DATA_LEN] = {0};
    uint8_t idx = 0;

    // 数据索引（大端序）
    data[idx++] = (binindex >> 8) & 0xFF;
    data[idx++] = binindex & 0xFF;
    // 固件数据
    memcpy(&data[idx], bindata, bindata_len);
    idx += bindata_len;

    sendCan(cf, id, CmdCode::UpData, data, idx);
}

void prohumanoid::UdataEnd(CanFd_data *cf, uint32_t id)
{
    if (id < MIN_UPGRADE_ID)
    {
        emit sendError(QString("Invalid upgrade ID: 0x%1").arg(id, 8, 16));
        return;
    }
    sendCan(cf, id, CmdCode::UpDataEnd, nullptr, 0);
}

void prohumanoid::UdevReset(CanFd_data *cf, uint32_t id)
{
    if (id < MIN_UPGRADE_ID)
    {
        emit sendError(QString("Invalid upgrade ID: 0x%1").arg(id, 8, 16));
        return;
    }
    sendCan(cf, id, CmdCode::UpDevReset, nullptr, 0);
}

void prohumanoid::UgEnd(CanFd_data *cf, uint32_t id)
{
    if (id < MIN_UPGRADE_ID)
    {
        emit sendError(QString("Invalid upgrade ID: 0x%1").arg(id, 8, 16));
        return;
    }
    sendCan(cf, id, CmdCode::UpEndQuery, nullptr, 0);
}

