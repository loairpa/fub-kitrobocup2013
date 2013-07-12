/*
 * rand.h
 *
 *  Created on: 11.03.2013
 *      Author: Lovisa
 */

#ifndef RAND_H_
#define RAND_H_

void init_rand(u32);
u32 rand_cmwc(void);
u32 random(u32);
u32 random2(u32,u32);
#endif /* RAND_H_ */
