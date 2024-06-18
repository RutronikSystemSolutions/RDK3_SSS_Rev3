/*
 * watchdog.h
 *
 *  Created on: 2024-06-18
 *      Author: GDR
 */

#ifndef WATCHDOG_H_
#define WATCHDOG_H_

void watchdog_init(void);
void watchdog_feed(void);
void watchdog_stop(void);
void watchdog_start(void);
void watchdog_deinit(void);

#endif /* WATCHDOG_H_ */
