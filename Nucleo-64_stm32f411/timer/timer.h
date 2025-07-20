
#ifndef TIMER_H_
#define TIMER_H_

void timer2_1_sec_init(void);
void timer2_compare(void);
void timer3_capture(void);
#define SR_UIF (1u<<0)
#define SR_CC1IF (1u<<1)
#endif /* TIMER_H_ */
