/*
 * Copyright (c) 2014-2016 IBM Corporation.
 * All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of the <organization> nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <device.h>
#include <errno.h>
#include <sys/util.h>
#include <zephyr.h>

#include "lmic.h"

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(lora_app);


#define STACKSIZE 1024

struct k_thread run_loop_thread;
K_THREAD_STACK_DEFINE(run_loop_stack, STACKSIZE);

static k_tid_t tRun_loop;

//////////////////////////////////////////////////
// CONFIGURATION (FOR APPLICATION CALLBACKS BELOW)
//////////////////////////////////////////////////

// application router ID (LSBF)
static const u8_t APPEUI[8]  = { 0x02, 0x00, 0x00, 0x00, 0x00, 0xEE, 0xFF, 0xC0 };

// unique device ID (LSBF)
static const u8_t DEVEUI[8]  = { 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF };

// device-specific AES key (derived from device EUI)
static const u8_t DEVKEY[16] = { 0xAB, 0x89, 0xEF, 0xCD, 0x23, 0x01, 0x67, 0x45, 
                                 0x54, 0x76, 0x10, 0x32, 0xDC, 0xFE, 0x98, 0xBA };


//////////////////////////////////////////////////
// APPLICATION CALLBACKS
//////////////////////////////////////////////////

// provide application router ID (8 bytes, LSBF)
void os_getArtEui(u8_t * buf)
{
    memcpy(buf, APPEUI, sizeof(APPEUI));
}

// provide device ID (8 bytes, LSBF)
void os_getDevEui(u8_t * buf)
{
    memcpy(buf, DEVEUI, sizeof(DEVEUI));
}

// provide device key (16 bytes)
void os_getDevKey(u8_t * buf)
{
    memcpy(buf, DEVKEY, sizeof(DEVKEY));
}


//////////////////////////////////////////////////
// MAIN - INITIALIZATION AND STARTUP
//////////////////////////////////////////////////

// initial job
static void initfunc(osjob_t * j)
{
    // reset MAC state
    LMIC_reset();

    // start joining
    LMIC_startJoining();

    // init done - onEvent() callback will be invoked...
}

static void run_loop(void)
{
    LOG_INF("%s started", __func__);

    // execute scheduled jobs and events
    os_runloop();
}

// application entry point
int lora_app_init(void)
{
    osjob_t initjob;

    // initialize runtime env
    os_init();

    // setup initial job
    os_setCallback(&initjob, initfunc);

    // setup run_loop
    tRun_loop = k_thread_create(&run_loop_thread, 
                                run_loop_stack, STACKSIZE,
                                (k_thread_entry_t) run_loop, 
                                NULL, NULL, NULL, -1, K_USER, K_FOREVER);

    k_thread_start(&run_loop_thread);

    return 0;
}


//////////////////////////////////////////////////
// LMIC EVENT CALLBACK
//////////////////////////////////////////////////

void onEvent(ev_t ev)
{
    switch(ev) {
   
        // network joined, session established
        case EV_JOINED:
            LOG_INF("netid = %d", LMIC.netid);
            goto tx;
          
        // scheduled data sent (optionally data received)
        case EV_TXCOMPLETE:
            if (LMIC.dataLen) { // data received in rx slot after tx
                LOG_HEXDUMP_INF(LMIC.frame+LMIC.dataBeg, LMIC.dataLen, "EV_TXCOMPLETE");
            }
        tx:
            // immediately prepare next transmission
            LMIC.frame[0] = LMIC.snr;

            // schedule transmission (port 1, datalen 1, no ack requested)
            LMIC_setTxData2(1, LMIC.frame, 1, 0);

            // (will be sent as soon as duty cycle permits)
            break;

        default:
            break;
    }
}
