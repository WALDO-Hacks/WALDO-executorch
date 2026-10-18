/* WALDO two-brain, M55-HE image. */
#ifndef HE_H
#define HE_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
int ethos_setup(void);            /* ethos_setup.c: 0 = the U55 driver is up */
void he_log_enable(void);         /* he_log.c: printf goes to the mailbox from now on */
void he_mailbox_flush(void);      /* main.c: push the mailbox out of the data cache */
int he_app_main(void);            /* he_app.cpp: never returns */
uint32_t he_millis(void);
#ifdef __cplusplus
}
#endif
#endif
