xSight协议部分



float CANThread::uint8tofloat(uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3)
{
    Uint8FloatUnion fto8;
    fto8.Byte[0] = d3;
    fto8.Byte[1] = d2;
    fto8.Byte[2] = d1;
    fto8.Byte[3] = d0;
    return fto8.ft;
}


float CANThread::uint8touint32(uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3)
{
    Uint8FloatUnion fto8;
    fto8.Byte[0] = d3;
    fto8.Byte[1] = d2;
    fto8.Byte[2] = d1;
    fto8.Byte[3] = d0;
    return fto8.u32t;
}

//数据解析
int CANThread::RecvAnalysis(uint32_t id,uint8_t *rdata,uint16_t rdata_len)
{
    uint8_t func = rdata[0];
    if(rdata_len > 16)
    {
        return -1;
    }
    if(id == 0X00)
    {
        switch(func)
        {
            case 0X80:  //关节ID查询
              GbPara::instance().Mot.id = rdata[1];
            break;
            case 0X81:
              GbPara::instance().Mot.id = rdata[1];
            break;
        }

    }
    if(id <= 0XFF)
    {
        switch(func)
        {

            case 0X82:
               GbPara::instance().Mot.oldid = rdata[1];
               GbPara::instance().Mot.newid = rdata[2];
               GbPara::instance().Mot.id = GbPara::instance().Mot.newid;
            break;
            case 0X83:
               GbPara::instance().Mot.zerofalg = rdata[1];
               GbPara::instance().Mot.zeroval = uint8tofloat(rdata[2],rdata[3],rdata[4],rdata[5]);
            break;
            case 0X84:
                 GbPara::instance().Mot.fanctl= rdata[1];
            break;
            case 0X85:
                GbPara::instance().Mot.mcuid = uint8tofloat(rdata[1],rdata[2],rdata[3],rdata[4]);
            break;
            case 0X90:
                GbPara::instance().Crun.ctx.Dev_flag = rdata[1];
            break;

            case 0X80:
            {
               // qDebug() << "rdata:" <<rdata[1] << rdata[2];
                GbPara::instance().Crun.crx.mod = (rdata[1] >> 4) &0XFF;
                GbPara::instance().Crun.crx.err = ((rdata[1] << 8) &0XF00) + rdata[2];

                float posi = uint8tofloat(rdata[3],rdata[4],rdata[5],rdata[6]);
               // qDebug() << "1:" <<fto8.ft;
                while (posi < -180.00 || posi >= 180.00) {
                    posi = (posi < -180.00) ? (posi + 360.00) : (posi - 360.00);
                }
                //qDebug() << "2:" <<fto8.ft;
                GbPara::instance().Crun.crx.posi = posi;

                GbPara::instance().Crun.crx.spd = uint8tofloat(rdata[7],rdata[8],rdata[9],rdata[10]);

                GbPara::instance().Crun.crx.cur = float((int16_t)(((rdata[11] << 8) &0XFF00) + rdata[12]) / 100.0);

                GbPara::instance().Crun.crx.tmotor = int8_t(rdata[13] - 50);
                GbPara::instance().Crun.crx.tmos = int8_t(rdata[14] - 50);
                //GbPara::instance().Crun.R_Vol = float(rdata[15] / 2.0);
            }
            break;

            case 0XB0:
            {
                GbPara::instance().Spara.resule_enable = rdata[1];
            }
            break;
            case 0XB1:
            {
                GbPara::instance().Spara.read_fun1 = rdata[1];
                GbPara::instance().Spara.read_fun2 = rdata[2];
            }
            break;
            case 0XB2:
            {
                GbPara::instance().Spara.read_fun1 = rdata[1];
                GbPara::instance().Spara.read_fun2 = rdata[2];
                GbPara::instance().Spara.read_value = uint8tofloat(rdata[3],rdata[4],rdata[5],rdata[6]);
                GbPara::instance().Spara.read_old_value = uint8tofloat(rdata[7],rdata[8],rdata[9],rdata[10]);
            }
            break;
            case 0XB3:
            {
                GbPara::instance().Spara.read_fun1 = rdata[1];
                GbPara::instance().Spara.read_fun2 = rdata[2];
                GbPara::instance().Spara.read_value = uint8tofloat(rdata[3],rdata[4],rdata[5],rdata[6]);
            }
            break;
        }
    }
    else if((id > 0X400) &&(id <= 0X4FF))
    {
        switch(func)
        {
            case 0X81:

                GbPara::instance().Ug.rx.mcuver = uint8touint32(rdata[1],rdata[2],rdata[3],rdata[4]);
            break;
            case 0X82:

                GbPara::instance().Ug.rx.mcuver = uint8touint32(rdata[1],rdata[2],rdata[3],rdata[4]);
            break;
            case 0X83:
                GbPara::instance().Ug.rx.fwcrc16 = (rdata[1] << 8 & 0XFF00)+ rdata[2];
                GbPara::instance().Ug.rx.fwFileLen =(rdata[3] << 16 & 0XFF0000) + (rdata[4] << 8 & 0XFF00)+ rdata[5];
                GbPara::instance().Ug.rx.fwearse = rdata[6];
            break;
            case 0X84:

                GbPara::instance().Ug.rx.fwindex = (rdata[1] << 8 & 0XFF00)+ rdata[2];
                for(int i = 0;i < rdata_len - 4;i++)
                {
                    GbPara::instance().Ug.rx.fwupdata[i] = rdata[3 + i];
                }
            break;
            case 0X85:
                GbPara::instance().Ug.rx.fwcrc16 = (rdata[1] << 8 & 0XFF00)+ rdata[2];

                GbPara::instance().Ug.rx.mcuver= uint8touint32(rdata[3],rdata[4],rdata[5],rdata[6]);
            break;
            case 0X86:

                GbPara::instance().Ug.rx.mcuver = uint8touint32(rdata[1],rdata[2],rdata[3],rdata[4]);
            break;
            case 0X87:

                GbPara::instance().Ug.rx.mcuver = uint8touint32(rdata[1],rdata[2],rdata[3],rdata[4]);
                GbPara::instance().Ug.rx.fw_succes = rdata[5];
            break;
        }
    }
    return 0;
}









