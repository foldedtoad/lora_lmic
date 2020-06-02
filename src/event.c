/* 
 *  events.c 
 */
#include <zephyr.h>
#include <sys/printk.h>

#include "buttons.h"
#include "event.h"
#include "ble_policy.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(events, 3);

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*---------------------------------------------------------------------------*/
void event_buttons(buttons_id_t btn_id)
{
    LOG_INF("%s", __func__);

    switch (btn_id) {

        case BTN1_ID:
            LOG_INF("%s: Button-1", __func__);
            break;

        case BTN2_ID:
            LOG_INF("%s: Button-2", __func__);
            break;

        case BTN3_ID:
            LOG_INF("%s: Button-3", __func__);
            break;

        case BTN4_ID:
            LOG_INF("%s: Button-4", __func__);
            break;

        default:
            LOG_INF("%s: unknown: %d", __func__, btn_id);
            break;
    }
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*---------------------------------------------------------------------------*/
void event_stepper(buttons_id_t btn_id)
{
    LOG_INF("%s", __func__);

    ble_button_event(btn_id);
}
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*---------------------------------------------------------------------------*/
void event_init(void)
{
    LOG_INF("%s", __func__);
    
    /* 
     * Register for button press notifications.
     */
    buttons_register_notify_handler(event_buttons);
}
