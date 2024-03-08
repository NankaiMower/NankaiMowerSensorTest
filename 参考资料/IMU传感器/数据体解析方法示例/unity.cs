using System;

public class IMUParser
{
    private static short ParseShort(byte[] buffer, int index)
    {
        return BitConverter.ToInt16(buffer, index);
    }

    private static int ParseInt24(byte[] buffer, int index)
    {
        int value = buffer[index] | buffer[index + 1] << 8 | buffer[index + 2] << 16;
        if ((value & 0x800000) != 0)  // ����Ƿ�Ϊ����
        {
            value |= unchecked((int)0xff000000);  // ת��Ϊ32λ����
        }
        return value;
    }

    private static uint ParseUInt32(byte[] buffer, int index)
    {
        return BitConverter.ToUInt32(buffer, index);
    }

    public static void ParseIMU(byte[] buf)
    {
        float scaleAccel = 0.00478515625f;        // ���ٶȱ���
        float scaleQuat = 0.000030517578125f;     // ��Ԫ������
        float scaleAngle = 0.0054931640625f;      // �Ƕȱ���
        float scaleAngleSpeed = 0.06103515625f;   // ���ٶȱ���
        float scaleMag = 0.15106201171875f;       // �ų�����
        float scaleTemperature = 0.01f;           // �¶ȱ���
        float scaleAirPressure = 0.0002384185791f;// ��ѹ����
        float scaleHeight = 0.0010728836f;        // �߶ȱ���

        float[] imu_dat = new float[34];

        if (buf[0] == 0x11)
        {
            int ctl = (buf[2] << 8) | buf[1];
            Debug.Log("\n���ı�ǩ: 0x{0:X4}", ctl);
            // ����ʱ���
            uint ms = (uint)((buf[6] << 24) | (buf[5] << 16) | (buf[4] << 8) | buf[3]);
            Debug.Log(" ms: {0}", ms);

            int L = 7; // �ӵ�7���ֽڿ�ʼ��������

            // ���ݿ���λ������ͬ������
            if ((ctl & 0x0001) != 0)
            {
                float tmpX = ParseShort(buf, L) * scaleAccel; L += 2;
                float tmpY = ParseShort(buf, L) * scaleAccel; L += 2;
                float tmpZ = ParseShort(buf, L) * scaleAccel; L += 2;

                Debug.Log("\taX: {0:F3}", tmpX); // x���ٶ�
                Debug.Log("\taY: {0:F3}", tmpY); // y���ٶ�
                Debug.Log("\taZ: {0:F3}", tmpZ); // z���ٶ�

                imu_dat[0] = tmpX;
                imu_dat[1] = tmpY;
                imu_dat[2] = tmpZ;
            }

            if ((ctl & 0x0002) != 0)
            {
                float tmpX = ParseShort(buf, L) * scaleAccel; L += 2;
                float tmpY = ParseShort(buf, L) * scaleAccel; L += 2;
                float tmpZ = ParseShort(buf, L) * scaleAccel; L += 2;

                Debug.Log("\tAX: {0:F3}", tmpX); // x���ٶ�AX
                Debug.Log("\tAY: {0:F3}", tmpY); // y���ٶ�AY
                Debug.Log("\tAZ: {0:F3}", tmpZ); // z���ٶ�AZ

                imu_dat[3] = tmpX;
                imu_dat[4] = tmpY;
                imu_dat[5] = tmpZ;
            }
            if ((ctl & 0x0004) != 0)
            {
                float tmpX = ParseShort(buf, L) * scaleAngleSpeed; L += 2;
                float tmpY = ParseShort(buf, L) * scaleAngleSpeed; L += 2;
                float tmpZ = ParseShort(buf, L) * scaleAngleSpeed; L += 2;

                Debug.Log("\tGX: {0:F3}", tmpX); // x���ٶ�GX
                Debug.Log("\tGY: {0:F3}", tmpY); // y���ٶ�GY
                Debug.Log("\tGZ: {0:F3}", tmpZ); // z���ٶ�GZ

                imu_dat[6] = tmpX;
                imu_dat[7] = tmpY;
                imu_dat[8] = tmpZ;
            }

            if ((ctl & 0x0008) != 0)
            {
                float tmpX = ParseShort(buf, L) * scaleMag; L += 2;
                float tmpY = ParseShort(buf, L) * scaleMag; L += 2;
                float tmpZ = ParseShort(buf, L) * scaleMag; L += 2;

                Debug.Log("\tCX: {0:F3}", tmpX); // x�ų�CX
                Debug.Log("\tCY: {0:F3}", tmpY); // y�ų�CY
                Debug.Log("\tCZ: {0:F3}", tmpZ); // z�ų�CZ

                imu_dat[9] = tmpX;
                imu_dat[10] = tmpY;
                imu_dat[11] = tmpZ;
            }

            // ����������¶ȡ���ѹ���߶�
            if ((ctl & 0x0010) != 0)
            {// �¶� ��ѹ �߶�
                float temperature = ParseShort(buf, L) * scaleTemperature; L += 2; // �¶�
                float airPressure = ParseInt24(buf, L) * scaleAirPressure; L += 3; // ��ѹ
                float height = ParseInt24(buf, L) * scaleHeight; L += 3; // �߶�

                Debug.Log("\ttemperature: {0:F2}", temperature);
                Debug.Log("\tairPressure: {0:F3}", airPressure);
                Debug.Log("\theight: {0:F3}", height);

                imu_dat[12] = temperature;
                imu_dat[13] = airPressure;
                imu_dat[14] = height;
            }
            // �����������Ԫ��
            if ((ctl & 0x0020) != 0)
            {
                float w = ParseShort(buf, L) * scaleQuat; L += 2;
                float x = ParseShort(buf, L) * scaleQuat; L += 2;
                float y = ParseShort(buf, L) * scaleQuat; L += 2;
                float z = ParseShort(buf, L) * scaleQuat; L += 2;

                Debug.Log("\tw: {0:F3}", w);
                Debug.Log("\tx: {0:F3}", x);
                Debug.Log("\ty: {0:F3}", y);
                Debug.Log("\tz: {0:F3}", z);

                imu_dat[15] = w;
                imu_dat[16] = x;
                imu_dat[17] = y;
                imu_dat[18] = z;
            }

            // ��������˽Ƕ�
            if ((ctl & 0x0040) != 0)
            {
                float angleX = ParseShort(buf, L) * scaleAngle; L += 2;
                float angleY = ParseShort(buf, L) * scaleAngle; L += 2;
                float angleZ = ParseShort(buf, L) * scaleAngle; L += 2;

                Debug.Log("\tangleX: {0:F3}", angleX);
                Debug.Log("\tangleY: {0:F3}", angleY);
                Debug.Log("\tangleZ: {0:F3}", angleZ);

                imu_dat[19] = angleX;
                imu_dat[20] = angleY;
                imu_dat[21] = angleZ;
            }

            // ���������λ��
            if ((ctl & 0x0080) != 0)
            {
                float offsetX = ParseShort(buf, L) / 1000.0f; L += 2;
                float offsetY = ParseShort(buf, L) / 1000.0f; L += 2;
                float offsetZ = ParseShort(buf, L) / 1000.0f; L += 2;

                Debug.Log("\toffsetX: {0:F3}", offsetX);
                Debug.Log("\toffsetY: {0:F3}", offsetY);
                Debug.Log("\toffsetZ: {0:F3}", offsetZ);

                imu_dat[22] = offsetX;
                imu_dat[23] = offsetY;
                imu_dat[24] = offsetZ;
            // ��������˲������˶�״̬
            if ((ctl & 0x0100) != 0)
            {
                uint steps = ParseUInt32(buf, L); L += 4;
                byte activity = buf[L]; L++;

                Debug.Log("\tsteps: {0}", steps);
                Debug.Log("\twalking: {0}", (activity & 0x01) != 0 ? "yes" : "no");
                Debug.Log("\trunning: {0}", (activity & 0x02) != 0 ? "yes" : "no");
                Debug.Log("\tbiking: {0}", (activity & 0x04) != 0 ? "yes" : "no");
                Debug.Log("\tdriving: {0}", (activity & 0x08) != 0 ? "yes" : "no");

                imu_dat[25] = (activity & 0x01) != 0 ? 100 : 0;
                imu_dat[26] = (activity & 0x02) != 0 ? 100 : 0;
                imu_dat[27] = (activity & 0x04) != 0 ? 100 : 0;
                imu_dat[28] = (activity & 0x08) != 0 ? 100 : 0;
            }

            // ��������˵������ٶ�
            if ((ctl & 0x0200) != 0)
            {
                float asX = ParseShort(buf, L) * scaleAccel; L += 2;
                float asY = ParseShort(buf, L) * scaleAccel; L += 2;
                float asZ = ParseShort(buf, L) * scaleAccel; L += 2;

                Debug.Log("\tasX: {0:F3}", asX);
                Debug.Log("\tasY: {0:F3}", asY);
                Debug.Log("\tasZ: {0:F3}", asZ);

                imu_dat[29] = asX;
                imu_dat[30] = asY;
                imu_dat[31] = asZ;
            }

            // ���������ADCֵ
            if ((ctl & 0x0400) != 0)
            {
                ushort adc = BitConverter.ToUInt16(buf, L); L += 2;

                Debug.Log("\tadc: {0}", adc);
                imu_dat[32] = adc;
            }

            // ���������GPIO״̬
            if ((ctl & 0x0800) != 0)
            {
                byte gpio = buf[L]; L++;
                Debug.Log("\tGPIO1 M:{0:X}, N:{1:X}", (gpio >> 4) & 0x0f, gpio & 0x0f);
                imu_dat[33] = gpio;
            }
        }
        else
        {
            Debug.Log("[����] ����ͷδ����");
        }
    }
}

public class Program
{
    public static void Main()
    {
        // data ������ʮ����������
        byte[] data = new byte[] { 0x11, 0xFF, 0x0F, 0xF5, 0xF5, 0x49, 0x47, 0x10, 0x00, 0xF6, 0xFF, 0xD5, 0xFF, 0x5A, 0xFF, 0x1F, 0xFD, 0x4A, 0x07, 0x09, 0x00, 0x0A, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3A, 0x0B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x50, 0x78, 0x9F, 0xEB, 0x5C, 0x0C, 0x64, 0xDB, 0x2C, 0xF1, 0xB0, 0x03, 0x40, 0xE7, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0xFC, 0xFF, 0xE4, 0xFF, 0xDA, 0xFF, 0xFE, 0x09, 0x01 };
        IMUParser.ParseIMU(data);
    }
}
