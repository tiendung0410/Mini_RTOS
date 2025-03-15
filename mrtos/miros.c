/****************************************************************************
* MInimal Real-time Operating System (MiROS), ARM-CLANG port.
* version 1.26 (matching lesson 26, see https://youtu.be/kLxxXNCrY60)
*
* This software is a teaching aid to illustrate the concepts underlying
* a Real-Time Operating System (RTOS). The main goal of the software is
* simplicity and clear presentation of the concepts, but without dealing
* with various corner cases, portability, or error handling. For these
* reasons, the software is generally NOT intended or recommended for use
* in commercial applications.
*
* Copyright (C) 2018 Miro Samek. All Rights Reserved.
*
* SPDX-License-Identifier: GPL-3.0-or-later
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program. If not, see <https://www.gnu.org/licenses/>.
*
* Git repo:
* https://github.com/QuantumLeaps/MiROS
****************************************************************************/
#include <stdint.h>
#include "miros.h"
#include <stdbool.h>

//----------------------------------------------------------------------
OSThread * volatile OS_curr; /* pointer to the current thread */
OSThread * volatile OS_next; /* pointer to the next thread to run */

OSThread *OS_thread[32 + 1]; /* array of threads started so far */
uint32_t OS_readySet; /* bitmask of threads that are ready to run */
uint32_t OS_delayedSet; /* bitmask of threads that are delayed */
uint32_t OS_BlockSet; // bitmask of threads that are blocked with WaitForever Timeout



#define LOG2(x) (32U - __builtin_clz(x))

OSThread idleThread;
void main_idleThread() {
    while (1) {
        OS_onIdle();
    }
}

void OS_init(void *stkSto, uint32_t stkSize) {
    /* set the PendSV interrupt priority to the lowest level 0xFF */
    *(uint32_t volatile *)0xE000ED20 |= (0xFFU << 16);

    /* start idleThread thread */
    OSThread_start(&idleThread,
                   0U, /* idle thread priority */
                   &main_idleThread,
                   stkSto, stkSize);
}

void OS_sched(void) {
    /* choose the next thread to execute... */
    OSThread *next;
    if (OS_readySet == 0U) { /* idle condition? */
        next = OS_thread[0]; /* the idle thread */
    }
    else {
        next = OS_thread[LOG2(OS_readySet)];
    }

    /* trigger PendSV, if needed */
    if (next != OS_curr) {
        OS_next = next;
        *(uint32_t volatile *)0xE000ED04 = (1U << 28);
    }
}

void OS_run(void) {
    /* callback to configure and start interrupts */
    OS_onStartup();

    __asm volatile ("cpsid i");
    OS_sched();
    __asm volatile ("cpsie i");

    /* the following code should never execute */
}

void OS_tick(void) {

	uint32_t workingSet = OS_delayedSet;
	uint32_t bit;
    uint8_t block_bit;
	while (workingSet != 0U) {
		OSThread *t = OS_thread[LOG2(workingSet)];
		bit = (1U << (t->prio - 1U));
        block_bit=    (t->Task_Semaphore_State.SemaphoreBlock <<0)           | (t->Task_Mutex_State.MutexBlock <<1)
                    | (t->Task_Queue_State.QueueFullBlock << 2)              | (t->Task_Queue_State.QueueEmptyBlock <<3)
                    | (t->Event_Group_State.Event_Group_Wait_All_Block<<4)   | (t->Event_Group_State.Event_Group_Wait_One_Block <<5)
                    | (t->Stream_Buffer_State.Stream_Buffer_Sender_Block<<6) | (t->Stream_Buffer_State.Stream_Buffer_Receive_Block<<7);
        --t->timeout;
        if (t->timeout == 0U) {
            OS_readySet   |= bit;  /* insert to set */
            OS_delayedSet &= ~bit; /* remove from set */
            if (block_bit !=0)
                {
                    OS_BlockSet &= ~bit;
                    switch (block_bit)
                    {
                    case 0b00000001:
                        t->Task_Semaphore_State.SemaphoreBlock=0;
                        break;
                    case 0b00000010:
                        t->Task_Mutex_State.MutexBlock=0;
                        break;
                    case 0b00000100:
                        t->Task_Queue_State.QueueFullBlock=0;
                        break;
                    case 0b00001000:
                        t->Task_Queue_State.QueueEmptyBlock =0;
                        break;
                    case 0b00010000:
                        t->Event_Group_State.Event_Group_Wait_All_Block=0;
                        break;
                    case 0b00100000:
                        t->Event_Group_State.Event_Group_Wait_One_Block=0;
                        break;
                    case 0b01000000:
                        t->Stream_Buffer_State.Stream_Buffer_Sender_Block=0;
                        break;
                    case 0b10000000:
                        t->Stream_Buffer_State.Stream_Buffer_Receive_Block=0;
                        t->Stream_Buffer_State.Stream_Buffer_UnblockFromTimeout=1;
                        break;
                    default:
                        break;
                    }
                }
        }
        workingSet &= ~bit; /* remove from working set */
		
	}
}

void OS_delay(uint32_t ticks) {
    uint32_t bit;
    __asm volatile ("cpsid i");

    /* never call OS_delay from the idleThread */

    OS_curr->timeout = ticks;
    bit = (1U << (OS_curr->prio - 1U));
    OS_readySet &= ~bit;
    OS_delayedSet |= bit;
    OS_sched();
    __asm volatile ("cpsie i");
}

void OSThread_start(
    OSThread *me,
    uint8_t prio, /* thread priority */
    OSThreadHandler threadHandler,
    void *stkSto, uint32_t stkSize)
{
    /* round down the stack top to the 8-byte boundary
    * NOTE: ARM Cortex-M stack grows down from hi -> low memory
    */
    uint32_t *sp = (uint32_t *)((((uint32_t)stkSto + stkSize) / 8) * 8);
    uint32_t *stk_limit;

    /* priority must be in ragne
    * and the priority level must be unused
    */


    *(--sp) = (1U << 24);  /* xPSR */
    *(--sp) = (uint32_t)threadHandler; /* PC */
    *(--sp) = 0x0000000EU; /* LR  */
    *(--sp) = 0x0000000CU; /* R12 */
    *(--sp) = 0x00000003U; /* R3  */
    *(--sp) = 0x00000002U; /* R2  */
    *(--sp) = 0x00000001U; /* R1  */
    *(--sp) = 0x00000000U; /* R0  */
    /* additionally, fake registers R4-R11 */
    *(--sp) = 0x0000000BU; /* R11 */
    *(--sp) = 0x0000000AU; /* R10 */
    *(--sp) = 0x00000009U; /* R9 */
    *(--sp) = 0x00000008U; /* R8 */
    *(--sp) = 0x00000007U; /* R7 */
    *(--sp) = 0x00000006U; /* R6 */
    *(--sp) = 0x00000005U; /* R5 */
    *(--sp) = 0x00000004U; /* R4 */

    /* save the top of the stack in the thread's attibute */
    me->sp = sp;

    /* round up the bottom of the stack to the 8-byte boundary */
    stk_limit = (uint32_t *)(((((uint32_t)stkSto - 1U) / 8) + 1U) * 8);

    /* pre-fill the unused part of the stack with 0xDEADBEEF */
    for (sp = sp - 1U; sp >= stk_limit; --sp) {
        *sp = 0xDEADBEEFU;
    }

    /* register the thread with the OS */
    OS_thread[prio] = me;
    me->prio = prio;
    /* make the thread ready to run */
    if (prio > 0U) {
        OS_readySet |= (1U << (prio - 1U));
    }
}


/* inline assembly syntax for Compiler 6 (ARMCLANG) */
__attribute__ ((naked))
void PendSV_Handler(void) {
__asm volatile (
    /* __disable_irq(); */
    "  CPSID         I                 \n"

    /* if (OS_curr != (OSThread *)0) { */
    "  LDR           r1,=OS_curr       \n"
    "  LDR           r1,[r1,#0x00]     \n"
    "  CMP           r1,#0             \n"
    "  BEQ           PendSV_restore    \n"

    /*     push registers r4-r11 on the stack */
#if (__ARM_ARCH == 6)               // if ARMv6-M...
    "  SUB           sp,sp,#(8*4)     \n" // make room for 8 registers r4-r11
    "  MOV           r0,sp            \n" // r0 := temporary stack pointer
    "  STMIA         r0!,{r4-r7}      \n" // save the low registers
    "  MOV           r4,r8            \n" // move the high registers to low registers...
    "  MOV           r5,r9            \n"
    "  MOV           r6,r10           \n"
    "  MOV           r7,r11           \n"
    "  STMIA         r0!,{r4-r7}      \n" // save the high registers
#else                               // ARMv7-M or higher
    "  PUSH          {r4-r11}          \n"
#endif                              // ARMv7-M or higher

    /*     OS_curr->sp = sp; */
    "  LDR           r1,=OS_curr       \n"
    "  LDR           r1,[r1,#0x00]     \n"
    "  MOV           r0,sp             \n"
    "  STR           r0,[r1,#0x00]     \n"
    /* } */

    "PendSV_restore:                   \n"
    /* sp = OS_next->sp; */
    "  LDR           r1,=OS_next       \n"
    "  LDR           r1,[r1,#0x00]     \n"
    "  LDR           r0,[r1,#0x00]     \n"
    "  MOV           sp,r0             \n"

    /* OS_curr = OS_next; */
    "  LDR           r1,=OS_next       \n"
    "  LDR           r1,[r1,#0x00]     \n"
    "  LDR           r2,=OS_curr       \n"
    "  STR           r1,[r2,#0x00]     \n"

    /* pop registers r4-r11 */
#if (__ARM_ARCH == 6)               // if ARMv6-M...
    "  MOV           r0,sp             \n" // r0 := top of stack
    "  MOV           r2,r0             \n"
    "  ADDS          r2,r2,#(4*4)      \n" // point r2 to the 4 high registers r7-r11
    "  LDMIA         r2!,{r4-r7}       \n" // pop the 4 high registers into low registers
    "  MOV           r8,r4             \n" // move low registers into high registers
    "  MOV           r9,r5             \n"
    "  MOV           r10,r6            \n"
    "  MOV           r11,r7            \n"
    "  LDMIA         r0!,{r4-r7}       \n" // pop the low registers
    "  ADD           sp,sp,#(8*4)      \n" // remove 8 registers from the stack
#else                               // ARMv7-M or higher
    "  POP           {r4-r11}          \n"
#endif                              // ARMv7-M or higher

    /* __enable_irq(); */
    "  CPSIE         I                 \n"

    /* return to the next thread */
    "  BX            lr                \n"
    );
}

//---------------------------Semaphore----------------------------------
uint8_t semaphore_base_id=1;
void OS_Counting_Semaphore_Init(Semaphore_t* semaphore,uint8_t semaphoreNum)
{
    __asm volatile ("cpsid i");
    semaphore->semaphoreCount= semaphoreNum;
    semaphore->semaphore_ID= semaphore_base_id; // gan id cho sema, moi lan tao 1 sema moi thi id tang 1
    semaphore_base_id +=1;
    __asm volatile ("cpsie i");
}

void OS_Binary_Semaphore_Init(Semaphore_t* semaphore)
{
	__asm volatile ("cpsid i");
    semaphore->semaphoreCount= 1;
    semaphore->semaphore_ID= semaphore_base_id;
    semaphore_base_id +=1;
    __asm volatile ("cpsie i");
}

uint8_t OS_Semaphore_Get(Semaphore_t* semaphore,TickType_t timeout)
{
    __asm volatile ("cpsid i");
        if (semaphore->semaphoreCount != 0)// neu van con sema
        {
            semaphore->semaphoreCount-=1;
        }
        else  // neu het sema roi thi block
        { 
            if (timeout == waitForever) 
            {
                OS_curr->Task_Semaphore_State.SemaphoreBlock=1;
                OS_curr->Task_Semaphore_State.SemaphoreBlock_ID=semaphore->semaphore_ID;
                OS_readySet &= ~(1U << (OS_curr->prio - 1U));
                OS_BlockSet |= (1U << (OS_curr->prio - 1U));
                OS_sched();
            }
            else // neu timeout ko phai forever thi them ca vao delaySet, neu timeout ve 0 thi unblock du cho ko get dc sema
            {
                OS_curr->Task_Semaphore_State.SemaphoreBlock=1;
                OS_curr->Task_Semaphore_State.SemaphoreBlock_ID=semaphore->semaphore_ID;
                OS_readySet &= ~(1U << (OS_curr->prio - 1U));
                OS_BlockSet |= (1U << (OS_curr->prio - 1U));
                OS_delayedSet |= (1U << (OS_curr->prio - 1U));
                OS_curr->timeout = timeout;
                OS_sched();
                return 0;
            }
        }
        return 1;
    __asm volatile ("cpsie i");
}

void OS_Semaphore_Release(Semaphore_t* semaphore)
{
    __asm volatile ("cpsid i");
    semaphore->semaphoreCount+=1;

    uint32_t workingSet = OS_BlockSet;
    uint32_t bit;
    OSThread *t = OS_thread[LOG2(workingSet)];
    /* Tim xem co task nao dang bi block do sema vua release ra khong ?*/
	while (  (!( (t->Task_Semaphore_State.SemaphoreBlock == 1) && (t->Task_Semaphore_State.SemaphoreBlock_ID== semaphore->semaphore_ID) )) && workingSet != 0U)
	{
		bit = (1U << (t->prio - 1U));
		workingSet &= ~bit; /* remove from working set */
		t = OS_thread[LOG2(workingSet)];
	}
    /* neu tim thay thi cho task do get sema luon*/
	if( (t->Task_Semaphore_State.SemaphoreBlock == 1) && (t->Task_Semaphore_State.SemaphoreBlock_ID == semaphore->semaphore_ID) )
	{
		semaphore->semaphoreCount-=1;
		t->Task_Semaphore_State.SemaphoreBlock =0;
		bit = (1U << (t->prio - 1U));
		OS_readySet   |= bit;  /* insert to set */
		OS_BlockSet &= ~bit; /* remove from set */
		OS_sched();
	}
    __asm volatile ("cpsie i");
}

void OS_Semaphore_Release_FromISR(Semaphore_t* semaphore)
{
    __asm volatile ("cpsid i");
    semaphore->semaphoreCount+=1;
    uint32_t workingSet = OS_BlockSet;
    uint32_t bit;
    OSThread *t = OS_thread[LOG2(workingSet)];
	while (  (!( (t->Task_Semaphore_State.SemaphoreBlock == 1) && (t->Task_Semaphore_State.SemaphoreBlock_ID== semaphore->semaphore_ID) )) && workingSet != 0U)
	{
		bit = (1U << (t->prio - 1U));
		workingSet &= ~bit; /* remove from working set */
		t = OS_thread[LOG2(workingSet)];
	}
	if((t->Task_Semaphore_State.SemaphoreBlock == 1) && (t->Task_Semaphore_State.SemaphoreBlock_ID== semaphore->semaphore_ID))
	{
		semaphore->semaphoreCount-=1;
		t->Task_Semaphore_State.SemaphoreBlock =0;
		bit = (1U << (t->prio - 1U));
		OS_readySet   |= bit;  /* insert to set */
		OS_BlockSet &= ~bit; /* remove from set */
		OS_sched();
	}
    __asm volatile ("cpsie i");
}
//---------------------------------------------------------Mutex-------------------------------------------------------------------

uint8_t mutex_base_id=1;
void OS_Mutex_Init(Mutex_t* mutex)
{
    mutex->mutexCount = 1;
    mutex->mutex_ID= mutex_base_id;
    mutex_base_id +=1;
}
uint8_t OS_Mutex_Get(Mutex_t* mutex,uint8_t priority,TickType_t timeout)
{
    if (mutex->mutexCount != 0) // neu mutex chua bi task nao chiem
    {
        if(OS_curr->Task_Mutex_State.HoldingAmount == 0) // luu lai prio ban dau de sau khi release thi tra lai cho no
        {
            OS_curr->Task_Mutex_State.initial_priority= OS_curr->prio;
        }
        OS_curr->Task_Mutex_State.HoldingAmount+=1;
        mutex->mutexCount=0;
        mutex->Holding_Mutex_Task = OS_curr;

        //--------- chuyen task hien tai sang vi tri moi tuong ung voi ceilling prio trong array OS_Threads, xoa bit ready cua vi tri cu va set ready cho vi tri moi
        OS_readySet &= ~(1U << (OS_curr->prio - 1U));
        OS_readySet |=  (1U << (priority - 1U));

        OS_thread[OS_curr->prio] = NULL;
        OS_thread[priority] = OS_curr;
        OS_curr->prio= priority;

        //---------
    }
    else{
        if (timeout == waitForever)
        {
            OS_curr->Task_Mutex_State.MutexBlock=1;
            OS_curr->Task_Mutex_State.MutexBlock_ID= mutex->mutex_ID;
            OS_curr->Task_Mutex_State.peding_priority= priority;
            OS_readySet &= ~(1U << (OS_curr->prio - 1U));
            OS_BlockSet |= (1U << (OS_curr->prio - 1U));
            OS_sched();
        }
        else
        {
            OS_curr->Task_Mutex_State.MutexBlock=1;
            OS_curr->Task_Mutex_State.MutexBlock_ID= mutex->mutex_ID;
            OS_curr->Task_Mutex_State.peding_priority= priority;
            OS_readySet &= ~(1U << (OS_curr->prio - 1U));
            OS_BlockSet |= (1U << (OS_curr->prio - 1U));
            OS_delayedSet |= (1U << (OS_curr->prio - 1U));
            OS_curr->timeout = timeout;
            OS_sched();
            return 0;
        }
    }
    return 1;
}

uint8_t OS_Mutex_Release(Mutex_t* mutex)
{
    /* Kiem tra neu task hien tai dang giu mutex nay thi moi co quyen release*/
    if (mutex->Holding_Mutex_Task == OS_curr)
    {
        mutex->mutexCount = 1;
        mutex->Holding_Mutex_Task= NULL;
        OS_curr->Task_Mutex_State.HoldingAmount-=1;

        OS_thread[OS_curr->prio] = NULL;
        OS_thread[OS_curr->Task_Mutex_State.initial_priority] = OS_curr;
        OS_readySet &= ~(1U << (OS_curr->prio - 1U));
        OS_readySet |=  (1U << (OS_curr->Task_Mutex_State.initial_priority - 1U));
        OS_curr->prio= OS_curr->Task_Mutex_State.initial_priority;

        uint32_t workingSet = OS_BlockSet;
        uint32_t bit;
        OSThread *t = OS_thread[LOG2(workingSet)];
        /* Tim xem co task nao dang bi block boi mutex nay khong?*/
        while ( (!( (t->Task_Mutex_State.MutexBlock == 1) && (t->Task_Mutex_State.MutexBlock_ID == mutex->mutex_ID) )) && workingSet != 0U)
        {
            bit = (1U << (t->prio - 1U));
            workingSet &= ~bit; /* remove from working set */
            t = OS_thread[LOG2(workingSet)];
        }
        /* Neu tim thay task bi block boi mutex nay thi cho no get mutex luon*/
        if( (t->Task_Mutex_State.MutexBlock == 1) && (t->Task_Mutex_State.MutexBlock_ID == mutex->mutex_ID))
        {
            mutex->mutexCount= 0;
            mutex->Holding_Mutex_Task = t;
            t->Task_Mutex_State.MutexBlock =0;
            t->Task_Mutex_State.initial_priority= t->prio;
            t->Task_Mutex_State.HoldingAmount+=1;
            // unblock task
            bit = (1U << (t->prio - 1U));
            OS_readySet   |= bit;  /* insert to set */
            OS_BlockSet &= ~bit; /* remove from set */
            // move task to pending priority position ( tuong tu nhu mieu ta trong phan mutex_get)
            OS_readySet &= ~(1U << (t->prio - 1U));
            OS_readySet |=  (1U << (t->Task_Mutex_State.peding_priority - 1U));
            OS_thread[t->prio] = NULL;
            OS_thread[t->Task_Mutex_State.peding_priority] = t;
            t->prio= t->Task_Mutex_State.peding_priority;
            OS_sched();
        }
    }
    else
    {
        return 0;
    }
    return 1;

}

//-------------------------------RTOS QUEUE--------------------------------

uint8_t queue_base_id=1;
void OS_Queue_Init(OS_Queue_t* OS_Queue, uint32_t* buffer, uint8_t buff_len)
{
    OS_Queue->buff = buffer;
    OS_Queue->index=0;
    OS_Queue->buff_len=buff_len;
    OS_Queue->id= queue_base_id;
    queue_base_id+=1;
}

bool OS_Queue_Empty(OS_Queue_t* queue)
{
   if (queue->index != 0)
   {
        return 0;
   }
   return 1;
}

bool OS_Queue_Full(OS_Queue_t* queue)
{
    if (queue->index == queue->buff_len)
    {
        return 1;
    }
    return 0;
}

uint8_t OS_Queue_Push(OS_Queue_t* OS_Queue, uint32_t val,TickType_t timeout)
{
    if (!OS_Queue_Full(OS_Queue) && !OS_Queue_Empty(OS_Queue)) // neu queue ko full ko empty thi push binh thuong
    {
        OS_Queue->buff[OS_Queue->index]=val;
        OS_Queue->index +=1;
    }
    else if (OS_Queue_Full(OS_Queue)) // Neu queue full thi block task, doi den khi duoc unblock ( queue ko con full) thi push vao
    {
        if (timeout != waitForever)
        {
            OS_delayedSet |= (1U << (OS_curr->prio - 1U));
            OS_curr->timeout = timeout;
        }
        OS_curr->Task_Queue_State.QueueFullBlock=1;
        OS_curr->Task_Queue_State.Block_ID=OS_Queue->id;
        OS_readySet &= ~(1U << (OS_curr->prio - 1U));
        OS_BlockSet |= (1U << (OS_curr->prio - 1U));
        OS_sched();
        
        // sau khi duoc unblock thi task se tiep tuc chay tu day
        if (!OS_Queue_Full(OS_Queue))
        {
            OS_Queue->buff[OS_Queue->index]=val;
            OS_Queue->index +=1;
        }
        else
        {
            return 0;
        }
    }
    else if (OS_Queue_Empty(OS_Queue)) // queue is not empty anymore so need to check for tasks blocked by pop empty queue
    {
    	OS_Queue->buff[OS_Queue->index]=val;
    	OS_Queue->index +=1;
        uint32_t workingSet = OS_BlockSet;
        uint32_t bit;
        OSThread *t = OS_thread[LOG2(workingSet)];
        // tim xem co task nao dang bi block do pop empty queue khong thi unblock cho no
        while (  (!((t->Task_Queue_State.QueueEmptyBlock == 1) && (t->Task_Queue_State.Block_ID==OS_Queue->id))) && workingSet != 0U) // find the task blocked by pop empty queue with highest prio
        {
            bit = (1U << (t->prio - 1U));
            workingSet &= ~bit; /* remove from working set */
            t = OS_thread[LOG2(workingSet)];
        }
        if(t->Task_Queue_State.QueueEmptyBlock == 1 && t->Task_Queue_State.Block_ID==OS_Queue->id) // neu tim thay
        {
            t->Task_Queue_State.QueueEmptyBlock =0;
            bit = (1U << (t->prio - 1U));
            OS_readySet   |= bit;  /* insert to set */
            OS_BlockSet &= ~bit; /* remove from set */
            OS_sched();
        }
    }
    return 1;
}

uint8_t OS_Queue_Pop(OS_Queue_t* OS_Queue,uint32_t*val,TickType_t timeout)
{
    if (!OS_Queue_Empty(OS_Queue) && !OS_Queue_Full(OS_Queue))// neu queue ko full ko empty thi pop binh thuong
    {
        *val = OS_Queue->buff[0];
        for (uint8_t i = 0; i < OS_Queue->index - 1; i++)
        {
            OS_Queue->buff[i] = OS_Queue->buff[i + 1];
        }
        OS_Queue->index-=1;
    }
    else if (OS_Queue_Empty(OS_Queue))// Neu queue empty thi block task, doi den khi duoc unblock ( queue ko con empty) thi pop ra
    {
        if (timeout != waitForever)
        {
            OS_delayedSet |= (1U << (OS_curr->prio - 1U));
            OS_curr->timeout = timeout;
        }
        
        OS_curr->Task_Queue_State.QueueEmptyBlock=1;
        OS_curr->Task_Queue_State.Block_ID=OS_Queue->id;
        OS_readySet &= ~(1U << (OS_curr->prio - 1U));
        OS_BlockSet |= (1U << (OS_curr->prio - 1U));
        OS_sched();
        
        // sau khi duoc unblock thi task se tiep tuc chay tu day
        if (!OS_Queue_Empty(OS_Queue))
        {
            *val = OS_Queue->buff[0];
            for (uint8_t i = 0; i < OS_Queue->index - 1; i++)
            {
                OS_Queue->buff[i] = OS_Queue->buff[i + 1];
            }
            OS_Queue->index-=1;
        }
        else
        {
            return 0;
        }
    }
    else if (OS_Queue_Full(OS_Queue)) // queue is not empty anymore so need to check for tasks blocked by pop empty queue
    {
    	*val = OS_Queue->buff[0];
        for (uint8_t i = 0; i < OS_Queue->index - 1; i++)
        {
            OS_Queue->buff[i] = OS_Queue->buff[i + 1];
        }
        OS_Queue->index-=1;
        uint32_t workingSet = OS_BlockSet;
        uint32_t bit;
        OSThread *t = OS_thread[LOG2(workingSet)];
        // tim xem co task nao bi block do push full queue khong de unblock cho no
        while ((!(t->Task_Queue_State.QueueFullBlock == 1 && t->Task_Queue_State.Block_ID==OS_Queue->id)) && workingSet != 0U) // find the task blocked by pop empty queue with highest prio
        {
            bit = (1U << (t->prio - 1U));
            workingSet &= ~bit; /* remove from working set */
            t = OS_thread[LOG2(workingSet)];
        }
        if(t->Task_Queue_State.QueueFullBlock == 1 && t->Task_Queue_State.Block_ID==OS_Queue->id) // neu tim thay
        {
            t->Task_Queue_State.QueueFullBlock  =0;
            bit = (1U << (t->prio - 1U));
            OS_readySet   |= bit;  /* insert to set */
            OS_BlockSet &= ~bit; /* remove from set */
            OS_sched();
        }
    }
    return 1;
}

uint8_t OS_Queue_Push_FromISR(OS_Queue_t* OS_Queue, uint32_t val)
{
    if (!OS_Queue_Full(OS_Queue))
    {
        if (!OS_Queue_Empty(OS_Queue))
        {
            OS_Queue->buff[OS_Queue->index]=val;
            OS_Queue->index +=1;
        }
        else
        {
            OS_Queue->buff[OS_Queue->index]=val;
            OS_Queue->index +=1;
            uint32_t workingSet = OS_BlockSet;
            uint32_t bit;
            OSThread *t = OS_thread[LOG2(workingSet)];
            while (  (!((t->Task_Queue_State.QueueEmptyBlock == 1) && (t->Task_Queue_State.Block_ID==OS_Queue->id))) && workingSet != 0U) // find the task blocked by pop empty queue with highest prio
            {
                bit = (1U << (t->prio - 1U));
                workingSet &= ~bit; /* remove from working set */
                t = OS_thread[LOG2(workingSet)];
            }
            if(t->Task_Queue_State.QueueEmptyBlock == 1 && t->Task_Queue_State.Block_ID==OS_Queue->id)
            {
                t->Task_Queue_State.QueueEmptyBlock =0;
                bit = (1U << (t->prio - 1U));
                OS_readySet   |= bit;  /* insert to set */
                OS_BlockSet &= ~bit; /* remove from set */
                OS_sched();
            }
        }
        return 1;
    }
    else{
        return 0;
    }
}
//-------------------------------SOFTWARE TIMER--------------------------------
soft_timer_t* timer_list[10];
uint8_t timer_idx=0;
void OS_SoftwareTimer_Init(soft_timer_t* soft_timer, uint32_t time, void* callback)
{
    soft_timer->preset = time;
    soft_timer->current= 0;
    soft_timer->callback = callback;
    soft_timer->start=0;
    timer_list[timer_idx]= soft_timer;
    timer_idx +=1;
}

void OS_SoftwareTimer_Start(soft_timer_t* soft_timer)
{
    soft_timer->start=1;
}

void OS_SoftwareTimer_Stop(soft_timer_t* soft_timer)
{
    soft_timer->start=0;
}

void OS_SoftwareTimer_Tick()// kiem tra xem timer nao dang chay thi tang current_time them 1 tick
{
    for (int i=0;i<timer_idx;i++)
    {
        if( timer_list[i]->start ==1 )
        {
            timer_list[i]->current+=1;
            if (timer_list[i]->current == timer_list[i]->preset) // neu timer dem den gia tri preset thi trigger ham callback
            {
                timer_list[i]->callback();
                timer_list[i]->current=0;
            }
        }
    }
}


//---------------------------------------------EVENT GROUP-----------------------------------------------
uint8_t event_group_base_id=1;
void OS_EventGroup_Init(Event_Group_t * event_group)
{
    event_group->event_flags =0;
    event_group->event_group_id = event_group_base_id;
    event_group_base_id +=1;
}


uint8_t OS_EventGroup_WaitBits( Event_Group_t* event_group, uint8_t uxBitsToWaitFor, bool xWaitForAllBits, bool resetEventFlags,TickType_t timeout)
{
    if ( xWaitForAllBits == 1)
    {
        if ((event_group->event_flags &  uxBitsToWaitFor) != uxBitsToWaitFor ) // kiem tra xem toan bo bit muon doi da duoc set len ko ?
        {
            if (timeout== waitForever)
            {
                OS_curr->Event_Group_State.Event_Group_Wait_All_Block= 1;
                OS_curr->Event_Group_State.WaitingBits= uxBitsToWaitFor; // gan nhung bit muon doi vao waitingbits cua task
                OS_curr->Event_Group_State.Event_Group_Block_ID= event_group->event_group_id;
                OS_readySet &= ~(1U << (OS_curr->prio - 1U));
                OS_BlockSet |= (1U << (OS_curr->prio - 1U));
                OS_sched();
            }
            else
            {
                OS_curr->Event_Group_State.Event_Group_Wait_All_Block= 1;
                OS_curr->Event_Group_State.WaitingBits= uxBitsToWaitFor; // gan nhung bit muon doi vao waitingbits cua task
                OS_curr->Event_Group_State.Event_Group_Block_ID= event_group->event_group_id;
                OS_readySet &= ~(1U << (OS_curr->prio - 1U));
                OS_BlockSet |= (1U << (OS_curr->prio - 1U));
                OS_delayedSet |= (1U << (OS_curr->prio - 1U));
                OS_curr->timeout = timeout;
                OS_sched();
            }
            // sau khi duoc unblock thi task se chay tiep tu day, tra ve trang thai event flags
            if (resetEventFlags) // neu can clear flags sau khi wait
            {
                uint8_t flags = event_group->event_flags;
                event_group->event_flags=0;
                return flags;
            }
            // neu ko can clear
            return event_group->event_flags;
        }
        else
        {
            if (resetEventFlags)// neu can clear flags sau khi wait
            {
                uint8_t flags = event_group->event_flags;
                event_group->event_flags=0;
                return flags;
            }
            return event_group->event_flags;
        }
    }
    else
    {
        if ((event_group->event_flags &  uxBitsToWaitFor) ==0 ) // kiem tra xem co bit nao trong so waitbits duoc set len ko ?
        {
            if (timeout== waitForever)
            {
                OS_curr->Event_Group_State.Event_Group_Wait_One_Block= 1;
                OS_curr->Event_Group_State.WaitingBits= uxBitsToWaitFor; // gan nhung bit muon doi vao waitingbits cua task
                OS_curr->Event_Group_State.Event_Group_Block_ID= event_group->event_group_id;
                OS_readySet &= ~(1U << (OS_curr->prio - 1U));
                OS_BlockSet |= (1U << (OS_curr->prio - 1U));
                OS_sched();
            }
            else
            {
                OS_curr->Event_Group_State.Event_Group_Wait_One_Block= 1;
                OS_curr->Event_Group_State.WaitingBits= uxBitsToWaitFor; // gan nhung bit muon doi vao waitingbits cua task
                OS_curr->Event_Group_State.Event_Group_Block_ID= event_group->event_group_id;
                OS_readySet &= ~(1U << (OS_curr->prio - 1U));
                OS_BlockSet |= (1U << (OS_curr->prio - 1U));
                OS_delayedSet |= (1U << (OS_curr->prio - 1U));
                OS_curr->timeout = timeout;
                OS_sched();
            }
            // sau khi duoc unblock thi task se chay tiep tu day, tra ve trang thai event flags
            if (resetEventFlags)// neu can clear flags sau khi wait
            {
                uint8_t flags = event_group->event_flags;
                event_group->event_flags=0;
                return flags;
            }
            return event_group->event_flags;
        }
        else
        {
            if (resetEventFlags)// neu can clear flags sau khi wait
            {
                uint8_t flags = event_group->event_flags;
                event_group->event_flags=0;
                return flags;
            }
            return event_group->event_flags;
        }
    }
}

void OS_EventGroup_SetBits( Event_Group_t* event_group, uint8_t uxBitsToSet)
{
    event_group->event_flags |= uxBitsToSet; // cap nhap trang thai event flags
    uint32_t workingSet = OS_BlockSet;
    uint32_t bit;
    OSThread *t = OS_thread[LOG2(workingSet)];
    // duyet tung task dang bi block xem co task nao dang wait cac bit vua duoc set khong
    while ( workingSet != 0U) // find the task blocked by pop empty queue with highest prio
    {
        if ((t->Event_Group_State.Event_Group_Wait_All_Block == 1) && (t->Event_Group_State.Event_Group_Block_ID==event_group->event_group_id))
        {
            // neu task dang bi wait_all_block thi kiem tra xem toan bo cac bit dang wait da duoc set len trong event flags chua
            if (event_group->event_flags == t->Event_Group_State.WaitingBits )
            {
                t->Event_Group_State.Event_Group_Wait_All_Block =0;
                t->Event_Group_State.WaitingBits =0;
                bit = (1U << (t->prio - 1U));
                OS_readySet   |= bit;  /* insert to set */
                OS_BlockSet &= ~bit; /* remove from set */
                OS_sched();
            }
        }
        if ((t->Event_Group_State.Event_Group_Wait_One_Block == 1) && (t->Event_Group_State.Event_Group_Block_ID==event_group->event_group_id))
        {
            // neu task dang bi wait_one_block thi kiem tra xem co bit nao trong waitingbits duoc set len trong event flags chua ? 
            if((event_group->event_flags & t->Event_Group_State.WaitingBits) !=0)
            {
                t->Event_Group_State.Event_Group_Wait_One_Block =0;
                t->Event_Group_State.WaitingBits =0;
                bit = (1U << (t->prio - 1U));
                OS_readySet   |= bit;  /* insert to set */
                OS_BlockSet &= ~bit; /* remove from set */
                OS_sched();
            }
        }
        bit = (1U << (t->prio - 1U));
        workingSet &= ~bit; /* remove from working set */
        t = OS_thread[LOG2(workingSet)];
    }
}


//------------------------------------------------------Stream Buffer----------------------------------------------------------
uint8_t stream_buffer_base_id=0;

void OS_Stream_Buffer_Init(Stream_Buffer_t * stream_buffer,uint32_t buffer_size, uint32_t trigger_level)
{
    stream_buffer->stream_buffer_id = stream_buffer_base_id;
    stream_buffer_base_id +=1;
    stream_buffer->buffer = (uint8_t*)malloc(buffer_size);
    stream_buffer->buff_size= buffer_size;
    stream_buffer->trigger_level= trigger_level;
    stream_buffer->current_idx =0;
}

void OS_Stream_Buffer_Delete(Stream_Buffer_t * stream_buffer)
{
    free(stream_buffer->buffer);
}

bool OS_Stream_Buffer_Empty(Stream_Buffer_t * stream_buffer)
{
   if (stream_buffer->current_idx != 0)
   {
        return 0;
   }
   return 1;
}

bool OS_Stream_Buffer_Full(Stream_Buffer_t * stream_buffer)
{
    if (stream_buffer->current_idx == stream_buffer->buff_size)
    {
        return 1;
    }
    return 0;
}

uint8_t OS_Stream_Buffer_Send(Stream_Buffer_t * stream_buffer, void * data, uint32_t data_len,TickType_t timeout)
{
    if( stream_buffer->current_idx + data_len <= stream_buffer->buff_size)// neu send trong gioi han size cua buffer
    {
        for (int i= 0; i<data_len;i++) // ghi data vao buffer cua stream buffer
        {
            stream_buffer->buffer[stream_buffer->current_idx]= ((uint8_t*)data)[i];
            stream_buffer->current_idx+=1;
        }
        /*Phan nay su dung giai thoat khac cac phan tren mot chut
        Khi send se unblock task dang bi block voi stream buffer nay luon, trong ham receive se co vong while de
        check lai lan nua neu sau khi duoc unblock van chua thoa man dieu kien receive thi tiep tuc block lai*/ 
        uint32_t workingSet = OS_BlockSet;
        uint32_t bit;
        OSThread *t = OS_thread[LOG2(workingSet)];
        while (  (!( (t->Stream_Buffer_State.Stream_Buffer_Receive_Block == 1) && (t->Stream_Buffer_State.Stream_Buffer_Block_ID== stream_buffer->stream_buffer_id) )) && workingSet != 0U)
        {
            bit = (1U << (t->prio - 1U));
            workingSet &= ~bit; /* remove from working set */
            t = OS_thread[LOG2(workingSet)];
        }
        if( (t->Stream_Buffer_State.Stream_Buffer_Receive_Block == 1) && (t->Stream_Buffer_State.Stream_Buffer_Block_ID== stream_buffer->stream_buffer_id) )
        {
            t->Stream_Buffer_State.Stream_Buffer_Receive_Block = 0;
            bit = (1U << (t->prio - 1U));
            OS_readySet   |= bit;  /* insert to set */
            OS_BlockSet &= ~bit; /* remove from set */
            OS_sched();
        }
    }
    else
    {   
        /* Block Task, khi co task receive bot data khoi buffer thi se unblock, vong while kiem tra xem thoa man dieu kien
        send chua, neu van chua thi tiep tuc block*/
        if (timeout != waitForever)
        {
            OS_delayedSet |= (1U << (OS_curr->prio - 1U));
            OS_curr->timeout = timeout;
        }
        while(stream_buffer->current_idx + data_len > stream_buffer->buff_size)
        {
            OS_curr->Stream_Buffer_State.Stream_Buffer_Sender_Block= 1;
            OS_curr->Stream_Buffer_State.Stream_Buffer_Block_ID= stream_buffer->stream_buffer_id;
            OS_readySet &= ~(1U << (OS_curr->prio - 1U));
            OS_BlockSet |= (1U << (OS_curr->prio - 1U));
            OS_sched();
            if (OS_curr->Stream_Buffer_State.Stream_Buffer_UnblockFromTimeout) // neu delay timeout thi cho thoat khoi block while
            {
                OS_curr->Stream_Buffer_State.Stream_Buffer_UnblockFromTimeout=0;
                break;
            }
        }
        if (stream_buffer->current_idx + data_len <= stream_buffer->buff_size)// kiem tra xem thoat khoi while la do thoa man dieu kien send hay la do timeout
        {
            for (int i= 0; i<data_len;i++)
            {
                stream_buffer->buffer[stream_buffer->current_idx]= ((uint8_t*)data)[i];
                stream_buffer->current_idx+=1;
            }
        }
        else
        {
            return 0;
        }
    }
    return 1;
}   


uint8_t OS_Stream_Buffer_Receive(Stream_Buffer_t * stream_buffer, void * data, uint32_t data_len,bool clearAferReceive,TickType_t timeout)// tuong tu ham send
{
    if (data_len <= stream_buffer->current_idx && stream_buffer->current_idx >= stream_buffer->trigger_level)
    {
        if (clearAferReceive)
        {
            for (int i= 0; i<data_len;i++)
            {
                ((uint8_t*)data)[i] = stream_buffer->buffer[i];
            }
            stream_buffer->current_idx =0;
        }
        else
        {
            stream_buffer->current_idx -= data_len;
            for (int i= 0; i<data_len;i++)
            {
                ((uint8_t*)data)[i] = stream_buffer->buffer[i];
                if (i<stream_buffer->current_idx)
                {
                    stream_buffer->buffer[i]= stream_buffer->buffer[i+data_len];
                }
            }
        }
        uint32_t workingSet = OS_BlockSet;
        uint32_t bit;
        OSThread *t = OS_thread[LOG2(workingSet)];
        while (  (!( (t->Stream_Buffer_State.Stream_Buffer_Sender_Block == 1) && (t->Stream_Buffer_State.Stream_Buffer_Block_ID== stream_buffer->stream_buffer_id) )) && workingSet != 0U)
        {
            bit = (1U << (t->prio - 1U));
            workingSet &= ~bit; /* remove from working set */
            t = OS_thread[LOG2(workingSet)];
        }
        if( (t->Stream_Buffer_State.Stream_Buffer_Sender_Block == 1) && (t->Stream_Buffer_State.Stream_Buffer_Block_ID== stream_buffer->stream_buffer_id) )
        {
            t->Stream_Buffer_State.Stream_Buffer_Sender_Block = 0;
            bit = (1U << (t->prio - 1U));
            OS_readySet   |= bit;  /* insert to set */
            OS_BlockSet &= ~bit; /* remove from set */
            OS_sched();
        }   
    }
    else
    {
        if (timeout != waitForever)
        {
            OS_delayedSet |= (1U << (OS_curr->prio - 1U));
            OS_curr->timeout = timeout;
        }
        while(data_len > stream_buffer->current_idx || stream_buffer->current_idx < stream_buffer->trigger_level)
        {
            OS_curr->Stream_Buffer_State.Stream_Buffer_Receive_Block= 1;
            OS_curr->Stream_Buffer_State.Stream_Buffer_Block_ID= stream_buffer->stream_buffer_id;
            OS_readySet &= ~(1U << (OS_curr->prio - 1U));
            OS_BlockSet |= (1U << (OS_curr->prio - 1U));
            OS_sched();
            if (OS_curr->Stream_Buffer_State.Stream_Buffer_UnblockFromTimeout)// neu delay timeout thi cho thoat khoi block while
            {
                OS_curr->Stream_Buffer_State.Stream_Buffer_UnblockFromTimeout =0;
                break;
            }
        }
        // kiem tra xem thoat khoi while la do thoa man dieu kien receive hay la do timeout
        if(data_len <= stream_buffer->current_idx && stream_buffer->current_idx >= stream_buffer->trigger_level)
        {
            if (clearAferReceive)
            {
                for (int i= 0; i<data_len;i++)
                {
                    ((uint8_t*)data)[i] = stream_buffer->buffer[i];
                }
                stream_buffer->current_idx =0;
            }
            else
            {
                stream_buffer->current_idx -= data_len;
                for (int i= 0; i<data_len;i++)
                {
                    ((uint8_t*)data)[i] = stream_buffer->buffer[i];
                    if (i<stream_buffer->current_idx)
                    {
                        stream_buffer->buffer[i]= stream_buffer->buffer[i+data_len];
                    }
                }
            }
        }
        else
        {
            return 0;
        }
    }
    return 1;
}
