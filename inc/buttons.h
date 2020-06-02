/*
 *   buttons.h
 */
#ifndef __BUTTONS_H
#define __BUTTONS_H

#include "buttons_id.h"

typedef void (*buttons_notify_t)(buttons_id_t id);

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*---------------------------------------------------------------------------*/
void buttons_init(void);
void buttons_register_notify_handler(buttons_notify_t notify);
void buttons_unregister_notify_handler(void);

#endif  /* __BUTTONS_H */