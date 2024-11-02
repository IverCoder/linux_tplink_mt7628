/*  Copyright(c) 2009-2018 Shenzhen TP-LINK Technologies Co.Ltd.
 *
 * file		sched_optimize.c
 * brief		Monitor the wait time of process for scheduling.
 * details	
 *
 * author	Wang Hao
 * version	
 * date		03May18
 *
 * history 	\arg	
 */

#include <linux/sched_optimize.h>

/**************************************************************************************************/
/* 										  DEFINES											   */
/**************************************************************************************************/
#ifdef SCHED_DEBUG
#define printErr(fmt, args...) printk("\033[1m[ %s ] %03d: "fmt"\033[0m", __FUNCTION__, __LINE__, ##args)
#define printWar(fmt, args...) printk("\033[4m[ %s ] %03d: "fmt"\033[0m", __FUNCTION__, __LINE__, ##args)

#define printWar_ratelimit(fmt, args...)	\
	do{						\
		if (printk_ratelimit()){		\
			printk("\033[4m[ %s ] %03d: "fmt"\033[0m", __FUNCTION__, __LINE__, ##args);	\
		}				\
	}while(0)

#define printErr_ratelimit(fmt, args...)	\
	do{ 					\
		if (printk_ratelimit()){		\
			printk("\033[1m[ %s ] %03d: "fmt"\033[0m", __FUNCTION__, __LINE__, ##args); 	\
		}				\
	}while(0)
#else
#define printErr(fmt, args...)
#define printWar(fmt, args...)

#define printWar_ratelimit(fmt, args...)
#define printErr_ratelimit(fmt, args...)
#endif

#define MAX_TASK_MONITOR				10
#define MAX_LOAD_RESERVED				15
#define LOAD_STEP						5
#define MAX_DECREASE_LOAD_COUNT		10
#define SOFTIRQ_USAGE_THRESHOLD		50

#define LOAD_COUNT_INTERVAL			msecs_to_jiffies(200)
#define SCHED_SDT_WAIT_TIME			msecs_to_jiffies(100)
#define SCHED_AFC_WAIT_TIME			2* HZ//msecs_to_jiffies(2000)


//SDT
#define MAX_HIGH_LOAD_COUNT			6
#define MAX_LOW_LOAD_COUNT			6
#define HIGH_LOAD_THRESHOLD			85//scheduler will tell us this value now.
#define MAX_LOAD						110
#define LOAD_INTERVAL					20

#define MAX_SOFTIRQ_RESTART			10
/* add end  */

/**************************************************************************************************/
/* 										  TYPES 											   */
/**************************************************************************************************/

/**************************************************************************************************/
/* 										  EXTERN_PROTOTYPES 								   */
/**************************************************************************************************/
extern unsigned long loadReserved;

/**************************************************************************************************/
/* 										  LOCAL_PROTOTYPES									   */
/**************************************************************************************************/
static void startWaitTaskTimer(struct task_struct *process);

/**************************************************************************************************/
/* 										  VARIABLES 										   */
/**************************************************************************************************/
unsigned long g_softirqUsage = 0;
unsigned char forceDropFlag = 0;
EXPORT_SYMBOL(forceDropFlag);
struct net_device *br_dev = NULL;
EXPORT_SYMBOL(br_dev);
uint32_t localIpAddr = 0;
EXPORT_SYMBOL(localIpAddr);


static struct task_struct *waitTask = NULL;
static struct task_struct *taskArray[MAX_TASK_MONITOR] = {NULL};
const static char *waitTaskName[MAX_TASK_MONITOR] = {//These are the key processes we want to monitor, add new here.
		"pppd",
		"httpd",
		"dhcpd",
		"dhcpc",
		NULL
	};

//for socket monitor
static unsigned long waitJiffies = 0;
static unsigned long taskWaitTime = 0;

static unsigned long decreaseLoadCountJiffies = 0;
static int decreaseLoadCount = 0;

static DEFINE_SPINLOCK(sched_lock);

void (*enableForceDropPtr)() = NULL;
EXPORT_SYMBOL(enableForceDropPtr);
void (*disableForceDropPtr)() = NULL;
EXPORT_SYMBOL(disableForceDropPtr);


//SDT
int maxSoftirqRestart = MAX_SOFTIRQ_RESTART;
unsigned long maxSoftirqTime = 0;

unsigned long softirqTime = 0;
unsigned long loadCountTime = 0;
static int highLoadCount = 0;
static int lowLoadCount = 0;

/*static unsigned int netrxCounter = 0;
static unsigned int taskletCounter = 0;
static unsigned int timerCounter = 0;
static unsigned int nettxCounter = 0;
static unsigned int blockCounter = 0;
static unsigned int blockpollCounter = 0;
static unsigned int schedCounter = 0;
static unsigned int hrtimerCounter = 0;
static unsigned int rcuCounter = 0;*/

static unsigned long testJiffies = 0;

unsigned long loadReserved = 0;

/**************************************************************************************************/
/* 										  LOCAL_FUNCTIONS									   */
/**************************************************************************************************/
static void startWaitTaskTimer(struct task_struct *process)
{
	int8_t index = 0;
	unsigned long flags = 0;

	if (!process)
		return;

	for (index = 0; index < MAX_TASK_MONITOR; index++)
	{
		if (taskArray[index] == process)
		{
			printWar("%s is been waking up...\n", process->comm);
			if (waitJiffies == 0)
			{
				spin_lock_bh(&sched_lock);//spin_lock_irqsave(&sched_lock, flags);
				waitJiffies = jiffies;
				waitTask = process;
				spin_unlock_bh(&sched_lock);//spin_unlock_irqrestore(&sched_lock, flags);
				printWar("Caution!!! %s start wait timer!\n", process->comm);
			}
		}
	}
}

/**************************************************************************************************/
/* 										  PUBLIC_FUNCTIONS									   */
/**************************************************************************************************/
uint8_t isSoftirqBusy()
{
	return (g_softirqUsage >= SOFTIRQ_USAGE_THRESHOLD) ? 1 : 0;
}
EXPORT_SYMBOL(isSoftirqBusy);

uint8_t isTaskWait()
{
	return (waitTask == NULL) ? 0 : 1;
}
EXPORT_SYMBOL(isTaskWait);

void stopWaitTaskTimer(struct task_struct *process, uint8_t debug)
{
	unsigned long flags = 0;
	
	if (!process)
		return;
	
	if (process == waitTask && waitJiffies > 0)
	{
		if (debug)
		{
			printWar("Clear %s wait flags cause packet has been taken!\n", waitTask->comm);
		}
		else
		{
			printWar("Clear %s wait flags before context switch!taskWaitTime is %lu.\n", waitTask->comm, taskWaitTime);
		}
		
		spin_lock_bh(&sched_lock);//spin_lock_irqsave(&sched_lock, flags);
		waitJiffies = 0;
		waitTask = NULL;
		taskWaitTime = 0;
		spin_unlock_bh(&sched_lock);//spin_unlock_irqrestore(&sched_lock, flags);

		//close AFC
		if (disableForceDropPtr)
		{
			disableForceDropPtr();
		}
	}
}

void checkWaitTime(struct task_struct *process)
{
	unsigned long flags = 0;
	
	if (!process)
		return;

	spin_lock_bh(&sched_lock);
	if (waitJiffies > 0 && waitTask != NULL)
	{
		//spin_lock_bh(&sched_lock);//spin_lock_irqsave(&sched_lock, flags);
		taskWaitTime += jiffies - waitJiffies;
		waitJiffies = jiffies;
		//spin_unlock_bh(&sched_lock);//spin_unlock_irqrestore(&sched_lock, flags);

		if (waitTask->state == TASK_INTERRUPTIBLE)
		{
			//clear wait flags
			spin_unlock_bh(&sched_lock);
			stopWaitTaskTimer(waitTask, 0);
			return;
		}

	#ifdef CONFIG_ACTIVE_FLOW_CONTROL
		if (taskWaitTime >= SCHED_AFC_WAIT_TIME)
		{
			if (g_softirqUsage >= SOFTIRQ_USAGE_THRESHOLD)
			{
				printErr("%s wait too long, trigger AFC!	taskWaitTime is %lu	SCHED_AFC_WAIT_TIME is %lu.\n", waitTask->comm, taskWaitTime, SCHED_AFC_WAIT_TIME);
				if (enableForceDropPtr)
				{
					enableForceDropPtr();
				}
			}
			else
			{
				printErr("%s wait too long, but AFC cannot be triggered due to g_softirqUsage is %lu.\n", waitTask->comm, g_softirqUsage);
			}

			//spin_lock_bh(&sched_lock);//spin_lock_irqsave(&sched_lock, flags);
			taskWaitTime = 0;
			//spin_unlock_bh(&sched_lock);//spin_unlock_irqrestore(&sched_lock, flags);
		}
	#endif

	#ifdef CONFIG_SOFTIRQ_DYNAMIC_TUNNING
		if (taskWaitTime >= SCHED_SDT_WAIT_TIME)
		{
			if (g_softirqUsage >= SOFTIRQ_USAGE_THRESHOLD)
			{
				//start increase
				decreaseLoadCount = 0;
				if (loadReserved + LOAD_STEP <= MAX_LOAD_RESERVED)
				{
					printErr("%s wait too long, trigger SDT!	taskWaitTime is %lu.	SCHED_SDT_WAIT_TIME is %lu.\n", waitTask->comm, taskWaitTime, SCHED_SDT_WAIT_TIME);
					loadReserved += LOAD_STEP;
					printWar("loadReserved increase to %lu.\n", loadReserved);
				}
			}
			/*else
			{
				printErr_ratelimit("%s wait too long, but SDT cannot be triggered due to g_softirqUsage is %lu.\n", waitTask->comm, g_softirqUsage);
			}*/

			if (loadReserved < MAX_LOAD_RESERVED)
			{
				//spin_lock_bh(&sched_lock);//spin_lock_irqsave(&sched_lock, flags);
				taskWaitTime = 0;
				//spin_unlock_bh(&sched_lock);//spin_unlock_irqrestore(&sched_lock, flags);
			}
		}
	#endif
	}
	else
	{
		//start slow decrease
		if (time_after(jiffies, decreaseLoadCountJiffies))
		{
			decreaseLoadCount++;
			if (decreaseLoadCount >= MAX_DECREASE_LOAD_COUNT)
			{
				decreaseLoadCount = 0;
				if (loadReserved >= LOAD_STEP)
				{
					loadReserved -= LOAD_STEP;
					printWar("loadReserved decrease to %lu.\n", loadReserved);
				}
			}
			
			decreaseLoadCountJiffies = jiffies + LOAD_COUNT_INTERVAL;
		}
	}

	spin_unlock_bh(&sched_lock);
}
EXPORT_SYMBOL(checkWaitTime);

void filterWaitTask(struct task_struct *process)
{
	int8_t index = 0;
	int8_t index2 = 0;
	unsigned long flags = 0;

	if (!process)
		return;

	for (index = 0; waitTaskName[index] != NULL; index++)
	{
		if (strncmp(process->comm, waitTaskName[index], strlen(waitTaskName[index])) == 0)
		{
			for (index2 = 0; index2 < MAX_TASK_MONITOR; index2++)
			{
				if (taskArray[index2] == NULL)
				{
					spin_lock_bh(&sched_lock);//spin_lock_irqsave(&sched_lock, flags);
					taskArray[index2] = process;
					spin_unlock_bh(&sched_lock);//spin_unlock_irqrestore(&sched_lock, flags);
 					printWar("Add process %s pid %d to taskArray!\n", process->comm, process->pid);
					break;
 				}
			}

			if (index2 == MAX_TASK_MONITOR)
			{
				printErr("taskArray is full, cannot add more task...\n");
				break;
			}

			break;
		}
	}
}

void cleanExitWaitTask(struct task_struct *process)
{
	int8_t index = 0;
	unsigned long flags = 0;

	if (!process)
		return;

	for (index = 0; index < MAX_TASK_MONITOR; index++)
	{
		if (taskArray[index] == process)
		{
			spin_lock_bh(&sched_lock);//spin_lock_irqsave(&sched_lock, flags);
			taskArray[index] = NULL;
			spin_unlock_bh(&sched_lock);//spin_unlock_irqrestore(&sched_lock, flags);
			printWar("Delete process %s pid %d from taskArray!\n", process->comm, process->pid);

			if (waitTask == process)
			{
				spin_lock_bh(&sched_lock);//spin_lock_irqsave(&sched_lock, flags);
				waitJiffies = 0;
				taskWaitTime = 0;
				waitTask = NULL;
				spin_unlock_bh(&sched_lock);//spin_unlock_irqrestore(&sched_lock, flags);
			}
			
			break;
		}
	}
}

void checkTaskInWaitQueue(wait_queue_head_t *queue)
{
	struct task_struct *process = NULL;
	struct poll_wqueues *pwq = NULL;
	wait_queue_t *curr, *next;

	if (!queue)
		return;
	
	list_for_each_entry_safe(curr, next, &queue->task_list, task_list) {
		pwq = curr->private;

		if (!pwq)
			continue;
		
		process = pwq->polling_task;

		startWaitTaskTimer(process);
	}
}

//SDT
void restoreMaxSoftirqRestart()
{
	highLoadCount = 0;
	if (time_after(jiffies, loadCountTime))
	{
		lowLoadCount++;
		loadCountTime = jiffies + LOAD_COUNT_TIME;
	}
	/* push level  */
	if (lowLoadCount >= MAX_LOW_LOAD_COUNT)
	{
		lowLoadCount = 0;
		if (maxSoftirqRestart < MAX_SOFTIRQ_RESTART)
		{
			maxSoftirqRestart = maxSoftirqRestart + 1;
			maxSoftirqRestart = maxSoftirqRestart > MAX_SOFTIRQ_RESTART ? MAX_SOFTIRQ_RESTART : maxSoftirqRestart;
			printWar("increase softirq level!	%d %lu %lu\n", maxSoftirqRestart, maxSoftirqTime, loadReserved);
		}
	}
	softirqTime = jiffies + MAX_SOFTIRQ_HANDLE_TIME;
	testJiffies = 0;
}

void maxSoftirqRestartTunning(const unsigned long jiffiesCounter)
{
	unsigned long softirqUsage = 0;
	
	testJiffies += jiffies - jiffiesCounter;
	if (time_after(jiffies, softirqTime))
	{
		softirqUsage = testJiffies * 100 / MAX_SOFTIRQ_HANDLE_TIME;//t'/T
		g_softirqUsage = softirqUsage;
		if (softirqUsage >= (MAX_LOAD - loadReserved))//HIGH_LOAD_THRESHOLD)
		{
			lowLoadCount = 0;
			if (time_after(jiffies, loadCountTime))
			{
				highLoadCount++;
				loadCountTime = jiffies + LOAD_COUNT_TIME;
			}
			/* pull level  */
			if (highLoadCount >= MAX_HIGH_LOAD_COUNT)
			{
				highLoadCount = 0;
				if (maxSoftirqRestart > 1)
				{
					maxSoftirqRestart = maxSoftirqRestart / 2;
					maxSoftirqRestart = maxSoftirqRestart ? maxSoftirqRestart : 1;
					printWar("decrease softirq level!	%d %lu %lu\n", maxSoftirqRestart, maxSoftirqTime, loadReserved);
				}
			}
		}
		else if (softirqUsage < (MAX_LOAD - LOAD_INTERVAL - loadReserved))
		{
			highLoadCount = 0;
			if (time_after(jiffies, loadCountTime))
			{
				lowLoadCount++;
				loadCountTime = jiffies + LOAD_COUNT_TIME;
			}
			/* push level  */
			if (lowLoadCount >= MAX_LOW_LOAD_COUNT)
			{
				lowLoadCount = 0;
				if (maxSoftirqRestart < MAX_SOFTIRQ_RESTART)
				{
					maxSoftirqRestart = maxSoftirqRestart + 1;
					maxSoftirqRestart = maxSoftirqRestart > MAX_SOFTIRQ_RESTART ? MAX_SOFTIRQ_RESTART : maxSoftirqRestart;
					printWar("increase softirq level!	%d %lu %lu\n", maxSoftirqRestart, maxSoftirqTime, loadReserved);
				}
			}
		}
		softirqTime = jiffies + MAX_SOFTIRQ_HANDLE_TIME;
		testJiffies = 0;
	}
}

