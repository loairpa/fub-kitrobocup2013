#define PHI 0x9e3779b9
#include "stm32f10x_lib.h"
u16 		Q[4096];
u16    	  c = 362436;


void init_rand(u32 x)
{
        int i;

        Q[0] = x;
        Q[1] = x + PHI;
        Q[2] = x + PHI + PHI;

        for (i = 3; i < 4096; i++)
                Q[i] = Q[i - 3] ^ Q[i - 2] ^ PHI ^ i;
}

u32 rand_cmwc(void)
{
        u32 t, a = 18782LL;
        static u32 i = 4095;
        u32 x, r = 0xfffffffe;
        i = (i + 1) & 4095;
        t = a * Q[i] + c;
        c = (t >> 32);
        x = t + c;
        if (x < c) {
                x++;
                c++;
        }
        return (Q[i] = r - x);
}

u32 random(u32 x){

	init_rand(x);
	return rand_cmwc();

}

u32 random2(u32 min, u32 max){
	u32 x = max - min;
	init_rand(x);
	return rand_cmwc()+min;
}
/*
 * rand.c
 *
 *  Created on: 11.03.2013
 *      Author: Lovisa
 */


