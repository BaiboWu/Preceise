#include "Timer.h"

#define TIMforTASK TIM2

/*TIM2:ÖÐ¶ÏÈÎÎñ´¦ÀíÅäÖÃ*/
void TIM_Task_Config(void)
{
	TIM_TimeBaseInitTypeDef TIM_TASK_TimeBase;
	NVIC_InitTypeDef NVIC_TIM_TASK;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);//´Ë´¦Èç¸ü¸ÄTIMforTASKÐèÊÖ¶¯¸ü¸Ä
	
	/**/
	TIM_DeInit(TIMforTASK);
	TIM_TASK_TimeBase.TIM_Prescaler=20-1;	
	TIM_TASK_TimeBase.TIM_Period=4200-1;//84MHz/20/4200=1KHzÖÐ¶ÏÆµÂÊ
	TIM_TASK_TimeBase.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TASK_TimeBase.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TASK_TimeBase.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIMforTASK,&TIM_TASK_TimeBase);

	TIM_ClearITPendingBit(TIMforTASK,TIM_IT_Update);//ÏÈÇå¸üÐÂÖÐ¶Ï±êÖ¾Î»£¬±ÜÃâÒ»¿ªÍê¸üÐÂÖÐ¶Ï¾ÍÁ¢Âí½ø¸üÐÂÖÐ¶Ï
	TIM_ITConfig(TIMforTASK,TIM_IT_Update,ENABLE);
	/*TIMforTASKÖÐ¶ÏÅäÖÃ*/
	NVIC_TIM_TASK.NVIC_IRQChannel=TIM2_IRQn;//´Ë´¦Èç¸ü¸ÄTIMforTASKÐèÊÖ¶¯¸ü¸Ä
	NVIC_TIM_TASK.NVIC_IRQChannelPreemptionPriority=3;
	NVIC_TIM_TASK.NVIC_IRQChannelSubPriority=0;
	NVIC_TIM_TASK.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_TIM_TASK);	
	
	TIM_Cmd(TIMforTASK,ENABLE);
	
}

float alpha_1[150]={-30.0000000000000,
-29.9779396385059,
-29.9117390169495,
-29.8013399789855,
-29.6466471195523,
-29.4475300886911,
-29.2038268573113,
-28.9153479836968,
-28.5818819282106,
-28.2032014703179,
-27.7790712860908,
-27.3092567450318,
-26.7935339815401,
-26.2317012877061,
-25.6235918593626,
-24.9690879054108,
-24.2681361003947,
-23.5207643212481,
-22.7270995604786,
-21.8873868495914,
-21.0020089586643,
-20.0715065618537,
-19.0965984763872,
-18.0782014975947,
-17.0174492693158,
-15.9157095534382,
-14.7745992013245,
-13.5959960912387,
-12.3820472875702,
-11.1351727071739,
-9.85806365152011,
-8.55367568412542,
-7.22521550099476,
-5.87612165334447,
-4.51003922782765,
-3.13078885643320,
-1.74233069898380,
-0.348724296149912,
1.04591458989196,
2.43745886485944,
3.82181534273276,
5.19496893819743,
6.55302447123095,
7.89224487635390,
9.20908480334109,
10.5002188357357,
11.7625638189058,
12.9932950605721,
14.1898564251253,
15.3499645732000,
16.4716077886840,
17.5530399801021,
18.5927705401449,
19.5895507979324,
20.5423578083426,
21.4503761982663,
22.3129787388249,
23.1297062433728,
23.9002473108975,
24.6244183496723,
25.3021442319114,
25.9334398506591,
26.5183927779320,
27.0571471598634,
27.5498889310278,
27.9968323863294,
28.3982081143800,
28.7542522704376,
29.0651971487685,
29.3312630027271,
29.5526510548929,
29.7295376382931,
29.8620694122004,
29.9503596014273,
29.9944852157799,
29.9944852157799,
29.9503596014273,
29.8620694122004,
29.7295376382931,
29.5526510548929,
29.3312630027271,
29.0651971487685,
28.7542522704376,
28.3982081143800,
27.9968323863294,
27.5498889310278,
27.0571471598634,
26.5183927779320,
25.9334398506591,
25.3021442319114,
24.6244183496723,
23.9002473108975,
23.1297062433729,
22.3129787388249,
21.4503761982663,
20.5423578083426,
19.5895507979324,
18.5927705401449,
17.5530399801021,
16.4716077886840,
15.3499645732001,
14.1898564251253,
12.9932950605721,
11.7625638189058,
10.5002188357357,
9.20908480334111,
7.89224487635391,
6.55302447123098,
5.19496893819745,
3.82181534273277,
2.43745886485944,
1.04591458989198,
-0.348724296149911,
-1.74233069898377,
-3.13078885643318,
-4.51003922782765,
-5.87612165334448,
-7.22521550099474,
-8.55367568412541,
-9.85806365152008,
-11.1351727071738,
-12.3820472875702,
-13.5959960912387,
-14.7745992013245,
-15.9157095534382,
-17.0174492693158,
-18.0782014975947,
-19.0965984763872,
-20.0715065618537,
-21.0020089586643,
-21.8873868495914,
-22.7270995604786,
-23.5207643212481,
-24.2681361003947,
-24.9690879054108,
-25.6235918593626,
-26.2317012877061,
-26.7935339815401,
-27.3092567450318,
-27.7790712860908,
-28.2032014703179,
-28.5818819282106,
-28.9153479836968,
-29.2038268573113,
-29.4475300886911,
-29.6466471195523,
-29.8013399789855,
-29.9117390169495,
-29.9779396385059,
-30.0000000000000};

float belta_1[150]={0,
1.20778513722787,
2.41395823483008,
3.61690368463490,
4.81499877970698,
6.00661026183295,
7.19009098816361,
8.36377676154877,
9.52598337313873,
10.6750039107468,
11.8091063921600,
12.9265317889035,
14.0254925126969,
15.1041714437216,
16.1607215864878,
17.1932664451104,
18.1999012146276,
19.1786948879816,
20.1276933786729,
21.0449237560793,
21.9283996830993,
22.7761281332491,
23.5861174457747,
24.3563867520426,
25.0849767740152,
25.7699619559345,
26.4094638438703,
27.0016655756037,
27.5448272871624,
28.0373021847019,
28.4775529745240,
28.8641682935928,
29.1958787420202,
29.4715720917139,
29.6903072353737,
29.8513264500726,
29.9540655813184,
29.9981618066854,
29.9834587110316,
29.9100084943905,
29.7780712337996,
29.5881112254509,
29.3407905370097,
29.0369599952834,
28.6776479159693,
28.2640469456103,
27.7974994284287,
27.2794817314652,
26.7115879611928,
26.0955134857164,
25.4330386420944,
24.7260129621425,
23.9763401964115,
23.1859643588386,
22.3568569573187,
21.4910055210110,
20.5904034856936,
19.6570414553118,
18.6928998217772,
17.6999426962288,
16.6801130831080,
15.6353292129493,
14.5674819399658,
13.4784331054467,
12.3700147667769,
11.2440291936788,
10.1022495372655,
8.94642108299616,
7.77826300502494,
6.59947054626602,
5.41171755533757,
4.21665931810577,
3.01593562757635,
1.81117404120378,
0.603993279178619,
-0.603993279178625,
-1.81117404120377,
-3.01593562757634,
-4.21665931810577,
-5.41171755533756,
-6.59947054626602,
-7.77826300502493,
-8.94642108299615,
-10.1022495372655,
-11.2440291936788,
-12.3700147667769,
-13.4784331054467,
-14.5674819399658,
-15.6353292129493,
-16.6801130831080,
-17.6999426962288,
-18.6928998217772,
-19.6570414553118,
-20.5904034856936,
-21.4910055210110,
-22.3568569573187,
-23.1859643588386,
-23.9763401964115,
-24.7260129621425,
-25.4330386420944,
-26.0955134857164,
-26.7115879611928,
-27.2794817314652,
-27.7974994284287,
-28.2640469456103,
-28.6776479159693,
-29.0369599952834,
-29.3407905370097,
-29.5881112254509,
-29.7780712337996,
-29.9100084943905,
-29.9834587110316,
-29.9981618066854,
-29.9540655813184,
-29.8513264500726,
-29.6903072353737,
-29.4715720917139,
-29.1958787420202,
-28.8641682935928,
-28.4775529745240,
-28.0373021847019,
-27.5448272871624,
-27.0016655756037,
-26.4094638438703,
-25.7699619559345,
-25.0849767740152,
-24.3563867520426,
-23.5861174457748,
-22.7761281332491,
-21.9283996830993,
-21.0449237560793,
-20.1276933786729,
-19.1786948879816,
-18.1999012146276,
-17.1932664451104,
-16.1607215864879,
-15.1041714437216,
-14.0254925126969,
-12.9265317889035,
-11.8091063921600,
-10.6750039107468,
-9.52598337313875,
-8.36377676154875,
-7.19009098816362,
-6.00661026183294,
-4.81499877970699,
-3.61690368463491,
-2.41395823483008,
-1.20778513722786,
0};

/*TIM2:ÖÐ¶ÏÈÎÎñ´¦Àíº¯Êý£¬ÖÐ¶ÏÆµÂÊ1KHz*/
PidTypeDef PID_Pos1;//Î»ÖÃ»·PID²ÎÊý½á¹¹Ìå
PidTypeDef PID_Pos2;
PidTypeDef PID_Pos3;

static uint16_t count=0;
uint16_t interval=10;

float q1_real=0.0, q2_real=0.0, q3_real=0.0;//q1 2 3Êµ¼ÊÖµ
float alpha_targ=0, belta_targ=0, alpha_step = 0, belta_step=0, alpha_real = 0, belta_real=0;

float delta_q1=0, delta_q2=0.0, delta_q3=0.0;

float deg_Node2_0_Last=0, deg_Node2_1_Last=0;
float P2 = 0.5, P3 = 0.5;

u8 send_flag=0;
float deg_yuzhi[2]={0.3, 0.3}, length_yuzhi_shen=3, length_yuzhi_la=8, deg_thred[2] = {0.1, 0.2};
float bengjindu_adjust[3] = {0.4, 0.4, 0.4}, bengjindu = 0.0, length_yuzhi_la_adjust[2] = {0.4, 0.8}, coffience[2] = {0.0, 0.0};
float bengjindu1 = 0.0, bengjindu2 = 0.0, bengjindu3 = 0.0;
float P=0.1;
float alpha_targ0=0, belta_targ0=0;
	
u8 print_flag=0, stable_flag = 0, stable_num = 0, run_flag = 0;//´òÓ¡flag
int32_t qc_actu_q1,qc_actu_q2,qc_actu_q3;//Êµ¼Êµç»úEPOS·µ»ØÇý¶¯qcÁ¿
float pos_actu_q1=0, pos_actu_q2=0.0, pos_actu_q3=0.0;

float delta_q11=0.0, delta_q22=0.0, delta_q33=0.0;//ÊÖ¶¯µ÷½Úµç»úÓÃ

float deg_kuadu=3, phi_n = -30;

u8 adjust_flag[3] = {0, 0, 0}, addeg_flag[2] = {0, 0};

uint32_t kk=0;
u8 start_flag=0;//¿ªÊ¼×ßÔ²
u8 count_1=0;//×ßÔ²ÖÜÆÚ¿ØÖÆ±äÁ¿
u8 speed = 15;//¿ØÖÆÃ¿¼¸¸öÖÜÆÚ×ßÒ»¸öµã

float lashenq1, lashenq2, lashenq3, d_l = 24.7, bjin_yuzhi = 0.0005;

void TIM2_IRQHandler(void)//´Ë´¦Èç¸ü¸ÄTIMforTASKÐèÊÖ¶¯¸ü¸Ä
{	
	
	if(TIM_GetITStatus(TIMforTASK,TIM_IT_Update)!=RESET)
	{
		TIM_ClearITPendingBit(TIMforTASK,TIM_IT_Update);		
		
		if(count==interval) 
			{		
				/*»ñÈ¡µçÎ»¼Æ¶ÁÊý£¬²¢×ª»»³É½Ç¶ÈÖµ*/
				//PotPin_Node2_GetValue();
				PotPin_Node1_GetValue();
				
				{//¶ÔµçÎ»¼Æ¶ÁÊýÏÞ·ù
					if( fabs(deg_Node2[0]-deg_Node2_0_Last) >=deg_kuadu)
						deg_Node2[0]=deg_Node2_0_Last;
					if( fabs(deg_Node2[1]-deg_Node2_1_Last) >=deg_kuadu)
						deg_Node2[1]=deg_Node2_1_Last;		
					
					deg_Node2_0_Last=deg_Node2[0];
					deg_Node2_1_Last=deg_Node2[1];
					alpha_real = deg_Node2[0];
					belta_real = deg_Node2[1];
					
				}
//				qc_actu_q1=EPOS_SDOReadActualPos(7);
//				qc_actu_q2=EPOS_SDOReadActualPos(8);
//				qc_actu_q3=EPOS_SDOReadActualPos(9);
				
				//ÅÜÔ²¸øÄ¿±êalpha belta
				if(start_flag==1)
				{
					if(count_1==speed)
					{
						count_1=0;
						alpha_targ = alpha_1[kk];
						belta_targ = belta_1[kk];
						
						kk=kk+1;
						if(kk==150) kk=0;
					}
					count_1+=1;
				} 
				
				if(start_flag==2)
				{
					if(count_1==speed)
					{
						count_1=0;
						alpha_targ = 30 * sin(kk/150.0 * 2 * PI);
						//belta_targ = 20 * sin(kk/150.0 * 2 * PI);
						
						kk=kk+1;
						if(kk==150) kk=0;
					}
					count_1+=1;
				}
				
				if(start_flag==3)
				{
					if(count_1==speed)
					{
						count_1=0;
						//alpha_targ = 20 * sin(kk/150.0 * 2 * PI);
						belta_targ = 30 * sin(kk/150.0 * 2 * PI);
						
						kk=kk+1;
						if(kk==150) kk=0;
					}
					count_1+=1;
				}
				
				//¼ÆËãÊµ¼Ê3¸ùÉþ ¿×¼ä¾àÀë
				q1_real=q_calc(alpha_real,belta_real, phi_n, 0);
				q2_real=q_calc(alpha_real,belta_real, phi_n, 1);
				q3_real=q_calc(alpha_real,belta_real, phi_n, 2);
				
				alpha_step = alpha_real + P * (alpha_targ - alpha_real);
					belta_step = belta_real + P * (belta_targ - belta_real);
					
					delta_q1= q1_real - q_calc(alpha_step,belta_step, phi_n, 0);
					delta_q2= q2_real - q_calc(alpha_step,belta_step, phi_n, 1);
					delta_q3= q3_real - q_calc(alpha_step,belta_step, phi_n, 2);
				
				if(fabs(alpha_real-alpha_targ) < deg_yuzhi[0])
				{
					addeg_flag[0] = 1;
				}
				else
				{
					addeg_flag[0] = 0;
				}
				
				if(addeg_flag[0] == 0)
				{
					deg_yuzhi[0] = deg_thred[0];
				}
				if(addeg_flag[0] == 1)
				{
					deg_yuzhi[0] = deg_thred[1];
				}
				
				if(fabs(belta_real-belta_targ) < deg_yuzhi[1])
				{
					addeg_flag[1] = 1;
				}
				else
				{
					addeg_flag[1] = 0;
				}
				
				if(addeg_flag[1] == 0)
				{
					deg_yuzhi[1] = deg_thred[0];
				}
				if(addeg_flag[1] == 1)
				{
					deg_yuzhi[1] = deg_thred[1];
				}
				
				//½Ç¶ÈÅÐ¶Ï=0µÄãÐÖµ£¬ÔÚÎó²îãÐÖµ·¶Î§¼´²»¸øµç»úÇý¶¯Á¿
				if( (fabs(alpha_real-alpha_targ)< deg_yuzhi[0]) && (fabs(belta_real-belta_targ)< deg_yuzhi[1]))
				{
					delta_q1=0;
					delta_q2=0;
					delta_q3=0;
					
					if(stable_flag == 0)
					{
						stable_flag = 1;
						if(stable_num == 0)
						{
							alpha_targ0 = alpha_targ;
							belta_targ0 = belta_targ;
							delta_q1=- P3 * (lashenq1 - bengjindu);
							delta_q2=- P3 * (lashenq2 - bengjindu);
							delta_q3=- P3 * (lashenq3 - bengjindu);
						}
						stable_num++;
						if(stable_num > 3)
						{
							stable_num = 0;
							run_flag = 1;
						}
					}
				}
				else
				{
					if((fabs(alpha_targ0-alpha_targ)<0.01) && (fabs(belta_targ0-belta_targ)<0.01) && (stable_flag == 1))
					{
						stable_flag = 0;
					}
					else if((fabs(alpha_targ0-alpha_targ)>0.01) || (fabs(belta_targ0-belta_targ)>0.01))
					{
						stable_num = 0;
						run_flag = 0;
						stable_flag = 0;
					}
				}
					
				pos_actu_q1=(float)qc_actu_q1*12.0/4/512/157.464;
				lashenq1 = pos_actu_q1-(d_l-q1_real);
				
				pos_actu_q2=(float)qc_actu_q2*12.0/4/512/157.464;
				lashenq2 = pos_actu_q2-(d_l-q2_real);
					
				pos_actu_q3=(float)qc_actu_q3*12.0/4/512/157.464;
				lashenq3 = pos_actu_q3-(d_l-q3_real);
				
				//¶Ôq1ÏÞ·ù
				if(delta_q1<0)
				{
//					if( -lashenq1 >length_yuzhi_shen)
//					{
//						delta_q1=0;
//					}
					delta_q1 = delta_q1 - P2 * (lashenq1 - bengjindu);
				}
				else if(delta_q1>0)
				{		
					if( lashenq1 >length_yuzhi_la)
					{
						delta_q1=0;
					}		
				}

		    //¶Ôq2ÏÞ·ù
				if(delta_q2<0)
				{
//					if( -lashenq2 >length_yuzhi_shen)
//					{
//						delta_q2=0;
//					}
					delta_q2 = delta_q2 - P2 * (lashenq2 - bengjindu);
				}
				else if(delta_q2>0)
				{		
					if( lashenq2 >length_yuzhi_la)
					{
						delta_q2=0;
					}		
				}
				//¶Ôq3ÏÞ·ù
				if(delta_q3<0)
				{
//					if( -lashenq3 >length_yuzhi_shen)
//					{
//						delta_q3=0;
//					}
					delta_q3 = delta_q3 - P2 * (lashenq3 - bengjindu);
				}
				else if(delta_q3>0)
				{		
					if( lashenq3 >length_yuzhi_la)
					{
						delta_q3=0;
					}		
				}
				
				
				//¿ØÖÆÈýÉþµÄ±Á½ô¶È
				if((fabs(delta_q1) < bjin_yuzhi) && (fabs(delta_q2) < bjin_yuzhi) && (fabs(delta_q3) < bjin_yuzhi) && (run_flag == 0))
				{
					if(fabs(lashenq1-bengjindu) < bengjindu_adjust[0])
					{
						adjust_flag[0] = 1;
					}
					else
					{
						adjust_flag[0] = 0;
					}
					
					if(adjust_flag[0] == 0)
					{
						bengjindu_adjust[0] = length_yuzhi_la_adjust[0];
					}
					if(adjust_flag[0] == 1)
					{
						bengjindu_adjust[0] = length_yuzhi_la_adjust[1];
					}
					
					if(fabs(lashenq1-bengjindu) < bengjindu_adjust[0])
					{
						delta_q1 = 0;
					}
					else
					{
						if( lashenq1 > bengjindu)
						{
								delta_q1 = - coffience[0] * (lashenq1 - bengjindu);
						}
						else
						{
								delta_q1 = - coffience[1] * (lashenq1 - bengjindu);
							
						}
				  }
					
					if(fabs(lashenq2-bengjindu) < bengjindu_adjust[1])
					{
						adjust_flag[1] = 1;
					}
					else
					{
						adjust_flag[1] = 0;
					}
					
					if(adjust_flag[1] == 0)
					{
						bengjindu_adjust[1] = length_yuzhi_la_adjust[0];
					}
					if(adjust_flag[1] == 1)
					{
						bengjindu_adjust[1] = length_yuzhi_la_adjust[1];
					}
					
					if(fabs(lashenq2-bengjindu) < bengjindu_adjust[1])
					{
						delta_q2 = 0;
					}
					else
					{
						if( lashenq2 > bengjindu)
						{
								delta_q2 = - coffience[0] * (lashenq2 - bengjindu);
						}
						else
						{
								delta_q2 = - coffience[1] * (lashenq2 - bengjindu);
						}
			  	}

					if(fabs(lashenq3-bengjindu) < bengjindu_adjust[2])
					{
						adjust_flag[2] = 1;
					}
					else
					{
						adjust_flag[2] = 0;
					}
					
					if(adjust_flag[2] == 0)
					{
						bengjindu_adjust[2] = length_yuzhi_la_adjust[0];
					}
					if(adjust_flag[2] == 1)
					{
						bengjindu_adjust[2] = length_yuzhi_la_adjust[1];
					}
					
					if(fabs(lashenq3-bengjindu) < bengjindu_adjust[2])
					{
						delta_q3 = 0;
					}
					else
					{
						if( lashenq3 > bengjindu)
						{
								delta_q3 = - coffience[0] * (lashenq3 - bengjindu);
						}
						else
						{
								delta_q3 = - coffience[1] * (lashenq3 - bengjindu);
						}
				}
			}
				
			  if((fabs(alpha_targ) > 30) || (fabs(belta_targ) > 30))
				{
					delta_q1 = 0;
					delta_q2 = 0;
					delta_q3 = 0;
				}
				
				if(send_flag == 1)
				{
					
				/*node Îªµç»ú½Úµã£¬delta_qÎªµç»úÎ»ÖÃÔöÁ¿ mm*/
					Motor_StartPos(7,delta_q1);
					Motor_StartPos(8,delta_q2);
					Motor_StartPos(9,delta_q3);
				}
				if(send_flag == 2)
				{
					
				/*node Îªµç»ú½Úµã£¬delta_qÎªµç»úÎ»ÖÃÔöÁ¿ mm*/
					Motor_StartPos(7,delta_q11);
					Motor_StartPos(8,delta_q22);
					Motor_StartPos(9,delta_q33);
				}
				
				if(print_flag==1)
				{
					VS4Channal_Send(100*alpha_real,100*belta_real, 100*alpha_targ,100*belta_targ); 
				}
				else if(print_flag == 2)
				{
					VS4Channal_Send(0 + 1000 * lashenq1, 0 + 1000 * lashenq2, 0 + 1000 * lashenq3, 0 + 1000 * bengjindu1);
				}
				if(print_flag==3)
				{
					VS4Channal_Send(value1/10,value2/10, value3/10,value4/10); 
				}

				count=0;

			}
			
		GPIOE->ODR^=(1<<13);
		count++;
		
	}
	
}
