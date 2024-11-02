/*  Copyright(c) 2009-2018 Shenzhen TP-LINK Technologies Co.Ltd.
 *
 * file		sched_optimize.h
 * brief		Monitor the wait time of process for scheduling.
 * details	
 *
 * author	Wang Hao
 * version	
 * date		03May18
 *
 * history 	\arg	
 */

#ifndef SCHED_OPTIMIZE_H
#define SCHED_OPTIMIZE_H

#include <linux/timer.h>
#include <linux/times.h>
#include <linux/sysctl.h>
#include <linux/syscalls.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/netdevice.h>


/**************************************************************************************************/
/* 										  DEFINES											   */
/**************************************************************************************************/
//SDT
#define MAX_SOFTIRQ_HANDLE_TIME		msecs_to_jiffies(50)
#define LOAD_COUNT_TIME				msecs_to_jiffies(200)

//debug info
//#define SCHED_DEBUG

/**************************************************************************************************/
/* 										  TYPES 											   */
/**************************************************************************************************/

/**************************************************************************************************/
/* 										  VARIABLES 										   */
/**************************************************************************************************/
extern unsigned long g_softirqUsage;
extern unsigned char forceDropFlag;
extern struct net_device *br_dev;
extern uint32_t localIpAddr;

//SDT
extern int maxSoftirqRestart;
extern unsigned long maxSoftirqTime;
extern unsigned long softirqTime;
extern unsigned long loadCountTime;

/**************************************************************************************************/
/* 										  FUNCTIONS									   */
/**************************************************************************************************/
uint8_t isSoftirqBusy();
uint8_t isTaskWait();
void stopWaitTaskTimer(struct task_struct *process, uint8_t debug);
void checkWaitTime(struct task_struct *process);
void filterWaitTask(struct task_struct *process);
void cleanExitWaitTask(struct task_struct *process);
void checkTaskInWaitQueue(wait_queue_head_t *queue);

//SDT
void restoreMaxSoftirqRestart();
void maxSoftirqRestartTunning(const unsigned long jiffiesCounter);


#endif
