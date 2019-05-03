/*
 * gen_utils.c
 *
 *  Created on: Apr 11, 2013
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "adjust_sys.h"
//#include "rll_main.h"
//#include "rll_router.h"
#include "gen_utils.h"
//#include "modbus_gen.h"
//#include "arm_sin_cos_f32_my.h"
//#include "main.h"

#define EXP10_TABLE_SIZE 100
// exp10_table_size[i]= 10^(i/10)
static const float32_t exp10_table_size[EXP10_TABLE_SIZE]= {
1.00000000,
1.25892541,
1.58489319,
1.99526231,
2.51188643,
3.16227766,
3.98107171,
5.01187234,
6.30957344,
7.94328235,
10.00000000,
12.58925412,
15.84893192,
19.95262315,
25.11886432,
31.62277660,
39.81071706,
50.11872336,
63.09573445,
79.43282347,
100.00000000,
125.89254120,
158.48931920,
199.52623150,
251.18864320,
316.22776600,
398.10717060,
501.18723360,
630.95734450,
794.32823470,
1000.00000000,
1258.92541200,
1584.89319200,
1995.26231500,
2511.88643200,
3162.27766000,
3981.07170600,
5011.87233600,
6309.57344500,
7943.28234700,
10000.00000000,
12589.25412000,
15848.93192000,
19952.62315000,
25118.86432000,
31622.77660000,
39810.71706000,
50118.72336000,
63095.73445000,
79432.82347000,
100000.00000000,
125892.54120000,
158489.31920000,
199526.23150000,
251188.64320000,
316227.76600000,
398107.17060000,
501187.23360000,
630957.34450000,
794328.23470000,
1000000.00000000,
1258925.41200000,
1584893.19200000,
1995262.31500000,
2511886.43200000,
3162277.66000000,
3981071.70600000,
5011872.33600000,
6309573.44500000,
7943282.34700000,
10000000.00000000,
12589254.12000000,
15848931.92000000,
19952623.15000000,
25118864.32000000,
31622776.60000000,
39810717.06000000,
50118723.36000000,
63095734.45000000,
79432823.47000000,
100000000.00000000,
125892541.20000000,
158489319.20000000,
199526231.50000000,
251188643.20000000,
316227766.00000000,
398107170.60000000,
501187233.60000000,
630957344.50000000,
794328234.70000000,
1000000000.00000000,
1258925412.00000000,
1584893192.00000000,
1995262315.00000000,
2511886432.00000000,
3162277660.00000000,
3981071706.00000000,
5011872336.00000000,
6309573445.00000000,
7943282347.00000000,
};

////////////////////////////////////////////////////////////////////////////////////////////

// Compares strings. Returns true if same
bool compare_str(const char *str_sample, char *str_to_compare, uint8_t size)
{uint8_t i;

  for(i=0;i<size;i++)
   {
	 if ((*str_sample)!=(*str_to_compare)) return false;
	 str_sample++;
	 str_to_compare++;
   }

  return true;
}// compare_str

///////////////////////////////////////////////////////

// Calculate CRC-16 for Modbus
void set_crc16(uint8_t *buf, uint8_t size)
{uni_8x2_16 sum;
 uint8_t shift_cnt;
 uint8_t byte_cnt;

  sum.w=0xffffU;

  for(byte_cnt=size; byte_cnt>0; byte_cnt--)
   {
	 sum.w=(uint16_t) ((sum.w/256U)*256U + ((sum.w%256U)^(buf[size-byte_cnt])));

     for(shift_cnt=0; shift_cnt<8; shift_cnt++)
       {
         if((sum.w&0x1)==1) sum.w=(uint16_t)((sum.w>>1)^0xa001U);
                       else sum.w>>=1;
       }
   }

  buf[size]=sum.b[0];
  buf[size+1]=sum.b[1];// little endian
}// set_crc16

///////////////////////////////////////////////////////

float32_t sqr_v(float32_t x)
{
  return (x*x);
}

float32_t abs_v(float32_t x)
{
  if (x>=0.0) return x;
  return (-x);
}

///////////////////////////////////////////////////////

// returns angle in range (0..180), x2 degrees, by floating-point any angle
uint8_t get_angle_x2degrees(float32_t fangle)
{
  while(fangle<0.0) fangle+= 360.0;
  while(fangle>=360.0) fangle-= 360.0;

  return (fangle/2.0);
}// get_angle_x2degrees

///////////////////////////////////////////////////////

// returns 10^(db_val/10)
float32_t exp10_table_value(uint8_t db_val)
{
  if (db_val>=EXP10_TABLE_SIZE) db_val= EXP10_TABLE_SIZE-1;
  return exp10_table_size[db_val];
}

///////////////////////////////////////////////////////

void copy_4bytes_to_ptr(uni_8x4_32 fv, uint8_t *ptr)
{uint8_t i;

  for(i=0;i<4;i++)
	ptr[i]= fv.b[i];
}

void copy_4bytes_from_ptr(uni_8x4_32 *pfv, uint8_t *ptr)
{uint8_t i;

  for(i=0;i<4;i++)
	  pfv->b[i]= ptr[i];
}

void copy_2bytes_to_ptr(uni_8x2_16 iv16, uint8_t *ptr)
{
  ptr[0]= iv16.b[0];
  ptr[1]= iv16.b[1];
}

void copy_2bytes_from_ptr(uni_8x2_16 *piv16, uint8_t *ptr)
{
  piv16->b[0]= ptr[0];
  piv16->b[1]= ptr[1];
}

///////////////////////////////////////////////////////

float32_t calc_triangle_angle(float32_t r1, float32_t r2, float32_t r_opposite)
{float32_t calfa;

  calfa= (sqr_v(r1) + sqr_v(r2)- sqr_v(r_opposite))/(2.0*r1*r2);
  if (calfa<-1.0) calfa= -1.0;
   else if (calfa>1.0) calfa= 1.0;

  calfa= acosf(calfa)*(180.0/3.14159);//arm_acos_f32_my(calfa);// angle, degrees (0..180)

  return calfa;
}//calc_triangle_angle

#define STM32F4_UUID ((uint32_t *)0x1FFF7A10)
// Returns 32 bits of 96-bit device unique ID
uint32_t uuid_v32(uint8_t ofs)
{
  return STM32F4_UUID[ofs];
}



