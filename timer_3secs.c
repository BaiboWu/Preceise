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
	TIM_TimeBaseInit(TIMforTASK,&TIM_TASK_TimeBase);//

	TIM_ClearITPendingBit(TIMforTASK,TIM_IT_Update);//ÏÈÇå¸üÐÂÖÐ¶Ï±êÖ¾Î»£¬±ÜÃâÒ»¿ªÍê¸üÐÂÖÐ¶Ï¾ÍÁ¢Âí½ø¸üÐÂÖÐ¶Ï
	TIM_ITConfig(TIMforTASK,TIM_IT_Update,ENABLE);
	/*TIMforTASKÖÐ¶ÏÅäÖÃ*/
	NVIC_TIM_TASK.NVIC_IRQChannel=TIM2_IRQn;//´Ë´¦Èç¸ü¸ÄTIMforTASKÐèÊÖ¶¯¸ü¸Ä
	NVIC_TIM_TASK.NVIC_IRQChannelPreemptionPriority=3;
	NVIC_TIM_TASK.NVIC_IRQChannelSubPriority=0;
	NVIC_TIM_TASK.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_TIM_TASK);	
		
		TIM_Cmd(TIMforTASK, ENABLE);
}
//¹ì¼£Êý¾Ýµã
float sec1_ang[2][441]={0}, sec2_ang[2][441]={0}, sec3_ang[2][441]={0};

//Î»ÖÃ»·PID²ÎÊý½á¹¹Ìå
PidTypeDef PID_Pos1, PID_Pos2, PID_Pos3;

/*TIM2:ÖÐ¶ÏÈÎÎñ´¦Àíº¯Êý£¬ÖÐ¶ÏÆµÂÊ1KHz*/
static uint16_t count=0, interval=10;

float sec_ang_targ[3][2] = {0}, sec_ang_real[3][2] = {0}, sec_ang_step[3][2] = {0};
float sec_dst_real[3][3] = {0}, sec_delta_dst[3][3] = {0}, sec_dst_step[3][3] = {0}, sec_dst_motor[3][3] = {0};
float sec_ang0[3][2] = {0};

float sec_delta_dst1[3][3] = {0};
float deg_Node1_Last[6] = {0}, sec_phi[3] = {30, -90, -30};

float deg_yuzhi[3][2]={0.3, 0.3, 0.3, 0.3, 0.3, 0.3},  deg_thred[2] = {0.3, 0.8};
float lashen[3][3], bengjindu_adjust[3][3] = {0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3}, length_yuzhi_la_adjust[2] = {0.3, 0.8};
float P_ang=0.2, P_bjin = 0.05, P_bjin1 = 0.2;

int32_t qc_actu_q[3][3] = {0};//Êµ¼Êµç»úEPOS·µ»ØÇý¶¯qcÁ¿
float length_yuzhi_shen=3, length_yuzhi_la=6, bengjindu = 0.0, deg_kuadu=15, d_l = 24.7, bjin_yuzhi = 0.0005;

u8 send_flag=0, print_flag=0, stable_flag[3] = {0}, stable_num[3] = {0}, run_flag[3] = {0};//´òÓ¡flag

u8 adjust_flag[3][3] = {0}, addeg_flag[3][2] = {0};

uint32_t kk=0;
u8 start_flag=0;//¿ªÊ¼×ßÔ²
u8 count_1=0;//×ßÔ²ÖÜÆÚ¿ØÖÆ±äÁ¿
u8 speed = 10;//¿ØÖÆÃ¿¼¸¸öÖÜÆÚ×ßÒ»¸öµã

void TIM2_IRQHandler(void)//´Ë´¦Èç¸ü¸ÄTIMforTASKÐèÊÖ¶¯¸ü¸Ä
{	
	u8 i,j;
	if(TIM_GetITStatus(TIMforTASK,TIM_IT_Update)!=RESET)
	{
		TIM_ClearITPendingBit(TIMforTASK,TIM_IT_Update);		
		
		if(count==interval) 
			{		
				/*»ñÈ¡µçÎ»¼Æ¶ÁÊý£¬²¢×ª»»³É½Ç¶ÈÖµ*/
				//ÐèÔö¼ÓµÚÎå¡¢Áù¸öµçÎ»Æ÷µÄ¶ÁÈ¡
				PotPin_Node1_GetValue();	
				PotPin_Node2_GetValue();
				
				//¶ÔµçÎ»¼Æ¶ÁÊýÏÞ·ù
				for(i = 0; i < 6; i++)
				{
					if(i > 3)
					{
						if( fabs(deg_Node2[i-4]-deg_Node1_Last[i]) >=deg_kuadu)
							deg_Node2[i-4]=deg_Node1_Last[i];	
					
						deg_Node1_Last[i]=deg_Node2[i-4];
					}
					else
					{
						if( fabs(deg_Node1[i]-deg_Node1_Last[i]) >=deg_kuadu)
							deg_Node1[i]=deg_Node1_Last[i];	
					
						deg_Node1_Last[i]=deg_Node1[i];
					}
				}
				
				//¶ÁÈ¡µç»úÇý¶¯Á¿£¬Ðè¾¡Á¿Óë½Ç¶È¶ÁÈ¡Í¬²½
				for(i = 0; i < 3; i++)
				{
					for(j = 0; j < 3; j++)
					{
						sec_dst_motor[i][j] = EPOS_SDOReadActualPos(3 * i + j + 1);
					}
				}
				
				//Êµ¼Ê½Ç¶È¶ÔÓ¦¹ØÏµ£ºÐè¸ü¸Ä£¿£¿£¿
				for(i = 0; i < 2; i++)
				{
					for(j = 0; j < 2; j++)
					{
						sec_ang_real[j][i] = deg_Node1[2*j+1-i];
					}
					sec_ang_real[2][i] = deg_Node2[1-i];
				}
				sec_ang_real[1][0] = deg_Node1[2];
				sec_ang_real[1][1] = -deg_Node1[3];
				sec_ang_real[2][0] = deg_Node2[1];
				sec_ang_real[2][1] = deg_Node2[0];
				
				//ÅÜÔ²¸øÄ¿±êalpha belta
				if(start_flag==1)
				{
					if(count_1==speed)
					{
						count_1=0;
						for(i = 0; i < 2; i++)
						{
							sec_ang_targ[0][i] = sec1_ang[i][kk];
							sec_ang_targ[1][i] = sec2_ang[i][kk];
							sec_ang_targ[2][i] = sec3_ang[i][kk];
						}
						kk=kk+1;
						if(kk==441) kk=0;
					}
					count_1+=1;
				} 
				
				if(start_flag==2)
				{
					if(count_1==speed)
					{
						count_1=0;
						for(i = 0; i < 2; i++)
						{
//							sec_ang_targ[i] = 20 * sin(kk /441.0 * 2 * PI);
						}
						kk=kk+1;
						if(kk==441) kk=0;
					}
					count_1+=1;
				}
				
				//PD Control
				for(i = 0; i < 3; i++)
				{
					for(j = 0; j < 2; j++)
					{
						sec_ang_step[i][j] = sec_ang_real[i][j] + P_ang * (sec_ang_targ[i][j] - sec_ang_real[i][j]);
					}
				}
				
				//i´ú±íµÚ¼¸½Ú£¬j´ú±íµÚ¼¸¸ùÉþ
				for(i = 0; i < 3; i++)
				{
					for(j = 0; j < 3; j++)
					{
						sec_dst_real[i][j] = q_calc(sec_ang_real[i][0], sec_ang_real[i][1], sec_phi[i], j);
						sec_dst_step[i][j] = q_calc(sec_ang_step[i][0], sec_ang_step[i][1], sec_phi[i], j);
						sec_delta_dst[i][j] = sec_dst_real[i][j] - sec_dst_step[i][j];
					}
				}
				//ºóÃæ½ÚÊýÐèÒª²¹³¥Ç°Ãæ½ÚÊý¾àÀëµÄ±ä»¯
				for(j = 0; j < 3; j++)
				{
					//#x2
					sec_dst_real[1][j] = sec_dst_real[1][j] + q_calc(sec_ang_real[0][0], sec_ang_real[0][1], 0, j);
					sec_dst_step[1][j] = sec_dst_step[1][j] + q_calc(sec_ang_step[0][0], sec_ang_step[0][1], 0, j);
					sec_delta_dst[1][j] = sec_dst_real[1][j] - sec_dst_step[1][j];
					
					//#x3
					sec_dst_real[2][j] = sec_dst_real[2][j] + q_calc(sec_ang_real[0][0], sec_ang_real[0][1], -30, j) + q_calc(sec_ang_real[1][0], sec_ang_real[1][1], -120, j);
					sec_dst_step[2][j] = sec_dst_step[2][j] + q_calc(sec_ang_step[0][0], sec_ang_step[0][1], -30, j) + q_calc(sec_ang_step[1][0], sec_ang_step[1][1], -120, j);
					sec_delta_dst[2][j] = sec_dst_real[2][j] - sec_dst_step[2][j];
				}
				
				//#1
				for(j = 0; j < 2; j++)
				{
					if(fabs(sec_ang_real[0][j]-sec_ang_targ[0][j]) < deg_yuzhi[0][j])
					{
						addeg_flag[0][j] = 1;
					}
					else
					{
						addeg_flag[0][j] = 0;
					}
					
					deg_yuzhi[0][j] = deg_thred[addeg_flag[0][j]];
				}
				//½Ç¶ÈÅÐ¶Ï=0µÄãÐÖµ£¬ÔÚÎó²îãÐÖµ·¶Î§¼´²»¸øµç»úÇý¶¯Á¿
				if((fabs(sec_ang_targ[0][0]-sec_ang_real[0][0])< deg_yuzhi[0][0]) && (fabs(sec_ang_targ[0][1]-sec_ang_real[0][1])< deg_yuzhi[0][1]))
				{
					for(j = 0; j < 3; j++)
					{
						sec_delta_dst[0][j] = 0;
					}
					
					//#2
					for(j = 0; j < 2; j++)
					{
						if(fabs(sec_ang_real[1][j]-sec_ang_targ[1][j]) < deg_yuzhi[1][j])
						{
							addeg_flag[1][j] = 1;
						}
						else
						{
							addeg_flag[1][j] = 0;
						}
						
						deg_yuzhi[1][j] = deg_thred[addeg_flag[1][j]];
					}
					//½Ç¶ÈÅÐ¶Ï=0µÄãÐÖµ£¬ÔÚÎó²îãÐÖµ·¶Î§¼´²»¸øµç»úÇý¶¯Á¿
					if((fabs(sec_ang_targ[1][0]-sec_ang_real[1][0])< deg_yuzhi[1][0]) && (fabs(sec_ang_targ[1][1]-sec_ang_real[1][1])< deg_yuzhi[1][1]))
					{
						for(j = 0; j < 3; j++)
						{
							sec_delta_dst[1][j] = 0;
						}
						
						//#3
						for(j = 0; j < 2; j++)
						{
							if(fabs(sec_ang_real[2][j]-sec_ang_targ[2][j]) < deg_yuzhi[2][j])
							{
								addeg_flag[2][j] = 1;
							}
							else
							{
								addeg_flag[2][j] = 0;
							}
							
							deg_yuzhi[2][j] = deg_thred[addeg_flag[2][j]];
						}
						//½Ç¶ÈÅÐ¶Ï=0µÄãÐÖµ£¬ÔÚÎó²îãÐÖµ·¶Î§¼´²»¸øµç»úÇý¶¯Á¿
						if((fabs(sec_ang_targ[2][0]-sec_ang_real[2][0])< deg_yuzhi[2][0]) && (fabs(sec_ang_targ[2][1]-sec_ang_real[2][1])< deg_yuzhi[2][1]))
						{
							for(j = 0; j < 3; j++)
							{
								sec_delta_dst[2][j] = 0;
							}
							
							if(stable_flag[2] == 0)
							{
								stable_flag[2] = 1;
								if(stable_num[2] == 0)
								{
									sec_ang0[2][0] = sec_ang_targ[2][0];
									sec_ang0[2][1] = sec_ang_targ[2][1];
								}
								stable_num[2]++;
								if(stable_num[2] > 3)
								{
									stable_num[2] = 0;
									run_flag[2] = 1;
								}
							}
						}
						else
						{
							if((fabs(sec_ang0[2][0] - sec_ang_targ[2][0])<0.01) && (fabs(sec_ang0[2][1] - sec_ang_targ[2][1])<0.01) && (stable_flag[2] == 1))
							{
								stable_flag[2] = 0;
							}
							else if((fabs(sec_ang0[2][0] - sec_ang_targ[2][0])>0.01) || (fabs(sec_ang0[2][1] - sec_ang_targ[2][1])>0.01))
							{
								stable_num[2] = 0;
								run_flag[2] = 0;
								stable_flag[2] = 0;
							}
						}//end#3
					
						if(stable_flag[1] == 0)
						{
							stable_flag[1] = 1;
							if(stable_num[1] == 0)
							{
								sec_ang0[1][0] = sec_ang_targ[1][0];
								sec_ang0[1][1] = sec_ang_targ[1][1];
							}
							stable_num[1]++;
							if(stable_num[1] > 3)
							{
								stable_num[1] = 0;
								run_flag[1] = 1;
							}
						}
					}
					else
					{
						if((fabs(sec_ang0[1][0] - sec_ang_targ[1][0])<0.01) && (fabs(sec_ang0[1][1] - sec_ang_targ[1][1])<0.01) && (stable_flag[1] == 1))
						{
							stable_flag[1] = 0;
						}
						else if((fabs(sec_ang0[1][0] - sec_ang_targ[1][0])>0.01) || (fabs(sec_ang0[1][1] - sec_ang_targ[1][1])>0.01))
						{
							stable_num[1] = 0;
							run_flag[1] = 0;
							stable_flag[1] = 0;
							stable_num[2] = 0;
							run_flag[2] = 0;
							stable_flag[2] = 0;
						}
					}//end#2
				
					if(stable_flag[0] == 0)
					{
						stable_flag[0] = 1;
						if(stable_num[0] == 0)
						{
							sec_ang0[0][0] = sec_ang_targ[0][0];
							sec_ang0[0][1] = sec_ang_targ[0][1];
						}
						stable_num[0]++;
						if(stable_num[0] > 3)
						{
							stable_num[0] = 0;
							run_flag[0] = 1;
						}
					}
				}
				else
				{
					if((fabs(sec_ang0[0][0] - sec_ang_targ[0][0])<0.01) && (fabs(sec_ang0[0][1] - sec_ang_targ[0][1])<0.01) && (stable_flag[0] == 1))
					{
						stable_flag[0] = 0;
					}
					else if((fabs(sec_ang0[0][0] - sec_ang_targ[0][0])>0.01) || (fabs(sec_ang0[0][1] - sec_ang_targ[0][1])>0.01))
					{
						stable_num[0] = 0;
						run_flag[0] = 0;
						stable_flag[0] = 0;
						stable_num[1] = 0;
						run_flag[1] = 0;
						stable_flag[1] = 0;
						stable_num[2] = 0;
						run_flag[2] = 0;
						stable_flag[2] = 0;
					}
					
				}//end#1
				
				//Êµ¼Êµç»úÇý¶¯Á¿
				for(i = 0; i < 3; i++)
				{
					for(j = 0; j < 3; j++)
					{
						sec_dst_motor[i][j] = sec_dst_motor[i][j]*12.0/4/512/157.464;
						lashen[i][j] = sec_dst_motor[i][j] - ((i+1) * d_l - sec_dst_real[i][j]);
						
						//×î´óÀ­ÉìÁ¿ÏÞÖÆ
						if(sec_delta_dst[i][j] < 0)
						{
							if(-lashen[i][j] > length_yuzhi_shen)
							{
								sec_delta_dst[i][j] = 0;
							}
						  
							//ËÉ³ÚÉþ×ÓÕÅ½ôÁ¦¿ØÖÆ
							if(i == 2)
							{
								sec_delta_dst[i][j] = -P_bjin1 * (lashen[i][j] - bengjindu);
							}
						}
						else if(sec_delta_dst[i][j] > 0)
						{
							if(lashen[i][j] > length_yuzhi_la)
							{
								sec_delta_dst[i][j] = 0;
							}
						}
					}
				}
				
				//µ½´ï¾«¶È·¶Î§Ê±£¬¿ªÆô±Á½ôÁ¿¿ØÖÆ
//				for(i = 0; i < 2; i++)
//				{
//					if((fabs(sec_delta_dst[i][0]) < bjin_yuzhi) && (fabs(sec_delta_dst[i][1]) < bjin_yuzhi) && (fabs(sec_delta_dst[i][2]) < bjin_yuzhi) && run_flag[i] == 0)
//					{
//						for(j = 0; j < 3; j++)
//						{
//							if(fabs(lashen[i][j]-bengjindu) < bengjindu_adjust[i][j])
//							{
//								adjust_flag[i][j] = 1;
//							}
//							else
//							{
//								adjust_flag[i][j] = 0;
//							}
//							bengjindu_adjust[i][j] = length_yuzhi_la_adjust[adjust_flag[i][j]];
//							
//							if(fabs(lashen[i][j] - bengjindu) < bengjindu_adjust[i][j])
//							{
//								sec_delta_dst[i][j] = 0;
//							}
//							else
//							{
//								sec_delta_dst[i][j] = -P_bjin * (lashen[i][j] - bengjindu);
//							}
//						}
//					}
//				}
				
				//·¢ËÍÇý¶¯Á¿¸øµç»ú
				if(send_flag == 1)
				{
					for(i = 0; i < 3; i++)
					{
						for(j = 0; j < 3; j++)
						{
							//°²È«ÉèÖÃ£¬Ã¿ÖÜÆÚ²»³¬¹ý5mm
							if(fabs(sec_delta_dst[i][j]) > 5)
							{
								sec_delta_dst[i][j] = 0;
							}
							//·¢ËÍÃüÁî£¬ÐèÐ£¶Ô¸ü¸Ä£¿£¿£¿
							Motor_StartPos(3 * i + j + 1, sec_delta_dst[i][j]);
						}
					}
				}
				
				if(send_flag == 2)
				{
					for(i = 0; i < 3; i++)
					{
						for(j = 0; j < 3; j++)
						{
							//·¢ËÍÃüÁî£¬ÐèÐ£¶Ô¸ü¸Ä£¿£¿£¿
							Motor_StartPos(3 * i + j + 1, sec_delta_dst1[i][j]);
						}
					}
				}
				
				if(print_flag==1)
				{
					VS4Channal_Send(100*sec_ang_real[0][0],100*sec_ang_real[0][1],100*sec_ang_real[1][0], 100*sec_ang_real[1][1]); 
				}
				else if(print_flag == 2)
				{
					VS4Channal_Send(100*sec_ang_real[2][0],100*sec_ang_real[2][1],100*sec_ang_real[1][0], 100*sec_ang_real[1][1]);
				}

				count=0;
			}
				
		GPIOE->ODR^=(1<<13);
		count++;
	}
}
