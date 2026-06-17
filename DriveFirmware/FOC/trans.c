#include "trans.h"
#include "tim.h"
volatile float theta_angle;
volatile float theta_hudu;
volatile float sin_val;
volatile float cos_val;

//volatile float Iq, Id, Ialpha, Ibeta;
//volatile float Vd, Vq, Valpha, Vbeta;
volatile float Ia, Ib, Ic;
volatile float Tpwm_temp, Udc_temp;
volatile float Tcmp1, Tcmp2, Tcmp3;

//角度自增，而后传给tran_angle()，再调用sin_cos_val()，进而调用反PARK变换，最后调用SVPWM计算，Vq和Vd也需要提前指定；

void tran_angle(float angle)
{
    theta_angle = angle;
	theta_hudu = theta_angle * PI / 180.0F;
}

//sinf(x),参数x应为弧度值；如果参数x为角度值，应乘上(π/180)转换成对应的弧度值。
void sin_cos_val()
{
	sin_val = sinf(theta_hudu);
	cos_val = cosf(theta_hudu);
}

/***************************************
功能：PARK变换
形参：alpha_beta电流、COS_SIN值、DQ轴电流
说明：交流变直流
***************************************/
//Id = Ialpha*cos(theta) + Ibeta*sin(theta)
//Iq = -Ialpha*sin(theta) + Ibeta*cos(theta)

void park_transf()
{
	Iq = Ialpha * cos_val + Ibeta * sin_val;
	Id = -Ialpha * sin_val + Ibeta * cos_val;
    //Iq =-Iq; //交流变直流
    //Id = -Id;
}
/***************************************
功能：反PARK变换
形参：DQ轴电压、COS_SIN值、alpha_beta电压
说明：直流变交流
***************************************/
//Valpha = cos(theta)*Vd - sin(theta)*Vq
//Vbeta = sin(theta)*Vd + cos(theta)*Vq
void anti_park_transf()
{
	Valpha = cos_val * Vd - sin_val * Vq;
	Vbeta = sin_val * Vd + cos_val * Vq;
}

/***************************************
功能：Clark变换
形参：三相电流以及alpha_beta电流
说明：由三相互差120度变换到两相互差90度
***************************************/
//Ialpha = (Ia - (Ib + Ic)/2)*2/3
//Ibeta = (Ib - Ic)*sqrt(3)/2*2/3
void clark_transf()
{
	Ialpha = (Ia - (Ib + Ic) * 0.5F) * 2.0F / 3.0F;
	Ibeta = (Ib - Ic) * SQRT3 / 2 * 2.0F / 3.0F;
}

/***************************************
功能：SVPWM计算
形参：alpha_beta电压以及母线电压、定时器周期
说明：根据alpha_beta电压计算三相占空比
***************************************/

void svpwm_calc()
{
    Udc_temp = 12.0F; //母线电压
    Tpwm_temp = __HAL_TIM_GET_AUTORELOAD(&htim1); //计算定时器周期
    int sector;
    float Tx, Ty, f_temp, Ta, Tb, Tc;     //三个Tcmp是比较值，sector仅仅是N值
    sector = 0;
    Tcmp1 = 0.0F;
    Tcmp2 = 0.0F;
    Tcmp3 = 0.0F;
    /*=== 扇区判断 ===*/
    if (Vbeta > 0.0F) 
    {
        sector = 1;
    }

    if ((SQRT3 * Valpha - Vbeta) / 2.0F > 0.0F) 
    {
        sector += 2;
    }

    if ((-SQRT3 * Valpha - Vbeta) / 2.0F > 0.0F) 
    {
        sector += 4;
    }

    /*=== 矢量作用时间计算 ===*/
    switch (sector) //计算出Tx和Ty
    {
        case 1:
            Tx = (-1.5F * Valpha + SQRT3 / 2 * Vbeta) * (Tpwm_temp / Udc_temp);
            Ty = (1.5F * Valpha + SQRT3 / 2 * Vbeta) * (Tpwm_temp / Udc_temp);
            break;

        case 2:
            Tx = (1.5F * Valpha + SQRT3 / 2 * Vbeta) * (Tpwm_temp / Udc_temp);
            Ty = -(SQRT3 * Vbeta * Tpwm_temp / Udc_temp);
            break;

        case 3:
            Tx = -((-1.5F * Valpha + SQRT3 / 2 * Vbeta) * (Tpwm_temp / Udc_temp));
            Ty = SQRT3 * Vbeta * Tpwm_temp / Udc_temp;
            break;

        case 4:
            Tx = -(SQRT3 * Vbeta * Tpwm_temp / Udc_temp);
            Ty = (-1.5F * Valpha + SQRT3 / 2 * Vbeta) * (Tpwm_temp / Udc_temp);
            break;

        case 5:
            Tx = SQRT3 * Vbeta * Tpwm_temp / Udc_temp;
            Ty = -((1.5F * Valpha + SQRT3 / 2 * Vbeta) * (Tpwm_temp / Udc_temp));
            break;

        default:
            Tx = -((1.5F * Valpha + SQRT3 / 2 * Vbeta) * (Tpwm_temp / Udc_temp));
            Ty = -((-1.5F * Valpha + SQRT3 / 2 * Vbeta) * (Tpwm_temp / Udc_temp));
            break;
    }
    /*=== 这一步问题有点大，改成摘要佬的 ===*/
    /*
    f_temp = Tx + Ty;
    if (f_temp > Tpwm_temp)
    {
      Tx /= f_temp;
      Ty /= (Tx + Ty);
    }
    */
    //限幅
    f_temp = Tx + Ty;
    if (f_temp > Tpwm_temp)
    {
        Tx = Tx / f_temp * Tpwm_temp;
        Ty = Ty / f_temp * Tpwm_temp;
    }

    //计算Ta、Tb、Tc
    Ta = (Tpwm_temp - (Tx + Ty)) / 4.0F;
    Tb = Tx / 2.0F + Ta;
    Tc = Ty / 2.0F + Tb;    //此处可以看出，此为高电平持续时间
    
    switch (sector) //Ta、Tb、Tc分配给三个通道
    {
        case 1:
            Tcmp1 = Tb;
            Tcmp2 = Ta;
            Tcmp3 = Tc;
            break;

        case 2:
            Tcmp1 = Ta;
            Tcmp2 = Tc;
            Tcmp3 = Tb;
            break;

        case 3:
            Tcmp1 = Ta;
            Tcmp2 = Tb;
            Tcmp3 = Tc;
            break;

        case 4:
            Tcmp1 = Tc;
            Tcmp2 = Tb;
            Tcmp3 = Ta;
            break;

        case 5:
            Tcmp1 = Tc;
            Tcmp2 = Ta;
            Tcmp3 = Tb;
            break;

        case 6:
            Tcmp1 = Tb;
            Tcmp2 = Tc;
            Tcmp3 = Ta;
            break;
    }
    /*=== 施加比较值给定时器 ===*/
    //FOC_Output.Tcmp1 = Tcmp1;
    //FOC_Output.Tcmp2 = Tcmp2;
    //FOC_Output.Tcmp3 = Tcmp3;
}