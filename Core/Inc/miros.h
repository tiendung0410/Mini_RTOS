/****************************************************************************
* MInimal Real-time Operating System (MiROS)
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
#ifndef MIROS_H
#define MIROS_H
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

typedef uint32_t TickType_t;
#define waitForever ( TickType_t ) 0xffffffffUL


typedef struct 
{
    uint8_t QueueFullBlock;
    uint8_t QueueEmptyBlock;
    uint8_t Block_ID;
}Task_Queue_State_t;

typedef struct 
{   
    uint8_t SemaphoreBlock;
    uint8_t SemaphoreBlock_ID; 
}Task_Semaphore_State_t;


typedef struct 
{
    uint8_t MutexBlock;
    uint8_t MutexBlock_ID;
    uint8_t initial_priority; 
    uint8_t peding_priority;
    uint8_t HoldingAmount ;
}Task_Mutex_State_t;

typedef struct 
{
    uint8_t Event_Group_Wait_All_Block;
    uint8_t Event_Group_Wait_One_Block;
    uint8_t Event_Group_Block_ID;
    uint8_t WaitingBits;
}Event_Group_State_t;

typedef struct 
{
    uint8_t Stream_Buffer_Sender_Block;
    uint8_t Stream_Buffer_Receive_Block;
    uint8_t Stream_Buffer_Block_ID;
    uint8_t Stream_Buffer_UnblockFromTimeout;
}Stream_Buffer_State_t;


/* Thread Control Block (TCB) */
typedef struct {
    void *sp; /* stack pointer */
    Task_Semaphore_State_t Task_Semaphore_State;
    Task_Mutex_State_t Task_Mutex_State;
    Task_Queue_State_t Task_Queue_State;
    Event_Group_State_t Event_Group_State;
    Stream_Buffer_State_t Stream_Buffer_State;
    uint32_t timeout; /* timeout delay down-counter */
    uint8_t prio; /* thread priority */
    /* ... other attributes associated with a thread */
} OSThread;

typedef void (*OSThreadHandler)();

void OS_init(void *stkSto, uint32_t stkSize);

/* callback to handle the idle condition */
void OS_onIdle(void);

/* this function must be called with interrupts DISABLED */
void OS_sched(void);

/* transfer control to the RTOS to run the threads */
void OS_run(void);

/* blocking delay */
void OS_delay(uint32_t ticks);

/* process all timeouts */
void OS_tick(void);

/* callback to configure and start interrupts */
void OS_onStartup(void);

void OSThread_start(
    OSThread *me,
    uint8_t prio, /* thread priority */
    OSThreadHandler threadHandler,
    void *stkSto, uint32_t stkSize);
/* -----------------------------------------------------Semaphore API=/----------------------------------------------------------- */
typedef struct 
{
    uint8_t semaphore_ID;
    uint8_t semaphoreCount;
}Semaphore_t;

void OS_Counting_Semaphore_Init(Semaphore_t* semaphore,uint8_t semaphoreNum);

void OS_Binary_Semaphore_Init(Semaphore_t* semaphore);

uint8_t OS_Semaphore_Get(Semaphore_t* semaphore,TickType_t timeout);

void OS_Semaphore_Release(Semaphore_t* semaphore);

/* -------------------------------------------------------Mutex API---------------------------------------------------------------- */
typedef struct 
{
    uint8_t mutex_ID;
    uint8_t mutexCount;
    OSThread* Holding_Mutex_Task;
}Mutex_t;

void OS_Mutex_Init(Mutex_t* mutex);

uint8_t OS_Mutex_Get(Mutex_t* mutex,uint8_t priority,TickType_t timeout);

uint8_t OS_Mutex_Release(Mutex_t* mutex);

/* --------------------------------------------------------Queue API----------------------------------------------------------------- */
typedef struct
{
    uint32_t *buff;
    uint8_t buff_len;
    uint8_t index;
    uint8_t id;
}OS_Queue_t;
void OS_Queue_Init(OS_Queue_t* OS_Queue, uint32_t* buffer, uint8_t buff_len);

bool OS_Queue_Empty(OS_Queue_t* queue);

bool OS_Queue_Full(OS_Queue_t* queue);

uint8_t OS_Queue_Push(OS_Queue_t* OS_Queue, uint32_t val,TickType_t timeout);

uint8_t OS_Queue_Pop(OS_Queue_t* OS_Queue, uint32_t*val, TickType_t timeout);

/* ----------------------------------------------------Software Timer API-------------------------------------------------------------*/
typedef void(*timer_callback_t)(void);
typedef struct 
{
    uint32_t preset;
    uint32_t current;
    bool start;
    timer_callback_t callback;
}soft_timer_t;

void OS_SoftwareTimer_Init(soft_timer_t* soft_timer, uint32_t time, void* callback);

void OS_SoftwareTimer_Start(soft_timer_t* soft_timer);

void OS_SoftwareTimer_Stop(soft_timer_t* soft_timer);

void OS_SoftwareTimer_Tick();

/* ------------------------------------------------------Event Group API -------------------------------------------------------------*/
typedef struct 
{
    uint8_t event_group_id;
    uint8_t event_flags;
}Event_Group_t;

void OS_EventGroup_Init(Event_Group_t * event_group);

uint8_t OS_EventGroup_WaitBits( Event_Group_t* event_group, uint8_t uxBitsToWaitFor, bool xWaitForAllBits, bool resetEventFlags,TickType_t timeout);

void OS_EventGroup_SetBits( Event_Group_t* event_group, uint8_t uxBitsToSet);


/* ------------------------------------------------------Stream Buffer API -------------------------------------------------------------*/
typedef struct 
{
    uint8_t *buffer;
    uint8_t stream_buffer_id;
    uint8_t trigger_level;
    uint8_t current_idx;
    uint8_t buff_size;
}Stream_Buffer_t;

void OS_Stream_Buffer_Init(Stream_Buffer_t * stream_buffer,uint32_t buffer_size, uint32_t trigger_level);

void OS_Stream_Buffer_Delete(Stream_Buffer_t * stream_buffer);

bool OS_Stream_Buffer_Empty(Stream_Buffer_t * stream_buffer);

bool OS_Stream_Buffer_Full(Stream_Buffer_t * stream_buffer);

uint8_t OS_Stream_Buffer_Send(Stream_Buffer_t * stream_buffer, void * data, uint32_t data_len,TickType_t timeout);

uint8_t OS_Stream_Buffer_Receive(Stream_Buffer_t * stream_buffer, void * data, uint32_t data_len,bool clearAferReceive,TickType_t timeout);

#endif /* MIROS_H */
