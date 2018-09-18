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
//Desired trajectory data points
float sec1_ang[2][441]={0}, sec2_ang[2][441]={0}, sec3_ang[2][441]={0};

//PID structure
PidTypeDef PID_Pos1, PID_Pos2, PID_Pos3;

//TIM2 interrupt function 1kHz
static uint16_t count=0, interval=20;

//corresponding arrays for 3 sections
float sec_ang_targ[3][2] = {0}, sec_ang_real[3][2] = {0}, sec_ang_step[3][2] = {0};
float sec_dst_real[3][3] = {0}, sec_delta_dst[3][3] = {0}, sec_dst_step[3][3] = {0}, sec_dst_motor[3][3] = {0};
float sec_ang0[3][2] = {0};

float sec_delta_dst1[3][3] = {0};
float deg_Node1_Last[8] = {0}, sec_phi[3] = {30, -90, -30};

//degree threshold and lashen limits
float deg_yuzhi[3][2]={0.1, 0.1, 0.1, 0.1, 0.1, 0.1},  deg_thred[2] = {0.1, 0.3};
float lashen[3][3];
//P parameters adjustment
float P_ang=0.1, P_bjin = 0.1, P_bjin1 = 0.2;

int32_t qc_actu_q[3][3] = {0};//actual encoders of motors
float length_yuzhi_shen=3, length_yuzhi_la=8, bengjindu = 0.0, deg_kuadu=3, d_l = 24.7;

u8 send_flag=0, print_flag=0, stable_flag[3] = {0}, stable_num[3] = {0}, addeg_flag[3][2] = {0};

uint32_t kk=0;
u8 start_flag=0, count_1=0, speed = 15;//¿ØÖÆÃ¿¼¸¸öÖÜÆÚ×ßÒ»¸öµã

void TIM2_IRQHandler(void)//´Ë´¦Èç¸ü¸ÄTIMforTASKÐèÊÖ¶¯¸ü¸Ä
{	
	u8 i, j, k;
	if(TIM_GetITStatus(TIMforTASK,TIM_IT_Update)!=RESET)
	{
		TIM_ClearITPendingBit(TIMforTASK,TIM_IT_Update);		
		
		if(count==interval) 
			{		
				//Initial degree sensor
				PotPin_Node1_GetValue();	
				PotPin_Node2_GetValue();
				
				//Filter out big jump
				for(i = 0; i < 8; i++)
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
				
				//Read actual encoders
				for(i = 0; i < 3; i++)
				{
					for(j = 0; j < 3; j++)
					{
						sec_dst_motor[i][j] = EPOS_SDOReadActualPos(3 * i + j + 1);
					}
				}
				
				//read real degree by sensor
				sec_ang_real[0][0] = deg_Node1[0];
				sec_ang_real[0][1] = deg_Node1[1];
				sec_ang_real[1][0] = deg_Node1[2];
				sec_ang_real[1][1] = -deg_Node1[3];
				sec_ang_real[2][0] = deg_Node2[0];
				sec_ang_real[2][1] = deg_Node2[1];
				
				//set desired trajectory
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
						kk++;
						if(kk>=441) kk=0;
					}
					count_1++;
				} 
				
				if(start_flag==2)
				{
					if(count_1==speed)
					{
						count_1=0;
						sec_ang_targ[2][0] = 20 * sin(kk /151.0 * 2 * PI);
						
						kk++;
						if(kk>=151) kk=0;
					}
					count_1++;
				}
				if(start_flag==3)
				{
					if(count_1==speed)
					{
						count_1=0;
						sec_ang_targ[2][1] = 20 * sin(kk /151.0 * 2 * PI);
						
						kk++;
						if(kk>=151) kk=0;
					}
					count_1++;
				}
				
				//PD Control with degree
				for(i = 0; i < 3; i++)
				{
					for(j = 0; j < 2; j++)
					{
						sec_ang_step[i][j] = sec_ang_real[i][j] + P_ang * (sec_ang_targ[i][j] - sec_ang_real[i][j]);
					}
				}
				
				//calculate delta motion for next circle
				for(i = 0; i < 3; i++)
				{
					for(j = 0; j < 3; j++)
					{
						sec_dst_real[i][j] = q_calc(sec_ang_real[i][0], sec_ang_real[i][1], sec_phi[i], j);
						sec_dst_step[i][j] = q_calc(sec_ang_step[i][0], sec_ang_step[i][1], sec_phi[i], j);
						if(i == 0)
							sec_delta_dst[i][j] = sec_dst_real[i][j] - sec_dst_step[i][j];
					}
				}
				//Add extra motion for behind sections
				for(j = 0; j < 3; j++)
				{
					//#section 2
					sec_dst_real[1][j] = sec_dst_real[1][j] + q_calc(sec_ang_real[0][0], sec_ang_real[0][1], 0, j);
					sec_dst_step[1][j] = sec_dst_step[1][j] + q_calc(sec_ang_step[0][0], sec_ang_step[0][1], 0, j);
					sec_delta_dst[1][j] = sec_dst_real[1][j] - sec_dst_step[1][j];
					
					//#section 3
					sec_dst_real[2][j] = sec_dst_real[2][j] + q_calc(sec_ang_real[0][0], sec_ang_real[0][1], -30, j) + q_calc(sec_ang_real[1][0], sec_ang_real[1][1], -120, j);
					sec_dst_step[2][j] = sec_dst_step[2][j] + q_calc(sec_ang_step[0][0], sec_ang_step[0][1], -30, j) + q_calc(sec_ang_step[1][0], sec_ang_step[1][1], -120, j);
					sec_delta_dst[2][j] = sec_dst_real[2][j] - sec_dst_step[2][j];
				}
				
				//degree limit for stability
				for(i = 0; i < 3; i++)
				{
					//Vary between small and big threshold 
					for(j = 0; j < 2; j++)
					{
						if(fabs(sec_ang_real[i][j]-sec_ang_targ[i][j]) < deg_yuzhi[i][j])
						{
							addeg_flag[i][j] = 1;
						}
						else
						{
							addeg_flag[i][j] = 0;
						}
						deg_yuzhi[i][j] = deg_thred[addeg_flag[i][j]];
					}
					
					//Check whether lies in the threshold or not
					if((fabs(sec_ang_targ[i][0]-sec_ang_real[i][0])< deg_yuzhi[i][0]) && (fabs(sec_ang_targ[i][1]-sec_ang_real[i][1])< deg_yuzhi[i][1]))
					{
						for(j = 0; j < 3; j++)
						{
							sec_delta_dst[i][j] = 0;
						}
					}
					else
					{
						break;
					}
				}
				
				//tension control and limits
				for(i = 0; i < 3; i++)
				{
					for(j = 0; j < 3; j++)
					{
						sec_dst_motor[i][j] = sec_dst_motor[i][j]*12.0/4/512/157.464;
						lashen[i][j] = sec_dst_motor[i][j] - ((i+1) * d_l - sec_dst_real[i][j]);
						
						if(sec_delta_dst[i][j] < 0)
						{
							//Control tensions for each cable
							sec_delta_dst[i][j] = sec_delta_dst[i][j] - P_bjin * (lashen[i][j] - bengjindu);
						}
						else if(sec_delta_dst[i][j] > 0)
						{
							//Max limits for stretching
							if(lashen[i][j] > length_yuzhi_la)
							{
								sec_delta_dst[i][j] = 0;
							}
						}
					}
				}
				
				
				//Send motor servo command
				if(send_flag == 1)
				{
					for(i = 0; i < 3; i++)
					{
						for(j = 0; j < 3; j++)
						{
							//The max limit for delta motion
							if(fabs(sec_delta_dst[i][j]) > 5)
							{
								sec_delta_dst[i][j] = 0;
							}
							//Send command automatic
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
							//Send command manually
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
					VS4Channal_Send(100*sec_ang_real[2][0],100*sec_ang_real[2][1],100*sec_ang_targ[2][0], 100*sec_ang_targ[2][1]);
				}

				count=0;
			}
				
		GPIOE->ODR^=(1<<13);
		count++;
	}
}
