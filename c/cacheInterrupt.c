#include <stdio.h>
#include <stdlib.h>

#include "intcachelib.h"

#define TEST_ADDR 0x44444440
#define TAG_ADDR 0x90000910
#define CACHE_ADDR 0x90000440

#define TEST_ADDR2 0x00001440 //same cache and tag address as TEST_ADDR

#define TEST_ADDR3 0x00002110 //totally independant address
#define TAG_ADDR3 0x90000844
#define CACHE_ADDR3 0x90000110

int main() {
  cacheInt_setup(); //opt_cacheInt_setup(); for optimized inthandler.
  int ret=0;
  int tag=0;
  int cache1=0;
  int cache2=0;
  int cache3=0;
  int cache4=0;

  //save different values in test addresses
  *(volatile _UNCACHED int *)(TEST_ADDR)=0x01020304;
  *(volatile _UNCACHED int *)(TEST_ADDR+4)=0x05060708;
  *(volatile _UNCACHED int *)(TEST_ADDR+8)=110;
  *(volatile _UNCACHED int *)(TEST_ADDR+12)=937;
  *(volatile _UNCACHED int *)(TEST_ADDR2)=42;
  *(volatile _UNCACHED int *)(TEST_ADDR3)=27;

  mode_ptr=1;

  //fetch tag and cache values before miss
  tag=*(int *)(TAG_ADDR);
  cache1=*(int *)(CACHE_ADDR);
  cache2=*(int *)(CACHE_ADDR+4);
  cache3=*(int *)(CACHE_ADDR+8);
  cache4=*(int *)(CACHE_ADDR+12);

  mode_ptr=0;
  printf("before addr1 miss, tag:%#010x cache1:%#010x cache2:%#010x cache3:%d cache4:%d\n",tag,cache1,cache2,cache3,cache4);
  mode_ptr=1;

  //provoke cache miss

  ret=*(int *)(TEST_ADDR);
  asm volatile("nop;"::); //need a nop before reading tag/cache

  tag=*(int *)(TAG_ADDR);
  cache1=*(int *)(CACHE_ADDR);
  cache2=*(int *)(CACHE_ADDR+4);
  cache3=*(int *)(CACHE_ADDR+8);
  cache4=*(int *)(CACHE_ADDR+12);

  mode_ptr=0;
  printf("after addr1 miss, return:%#010x tag:%#010x cache1:%#010x cache2:%#010x cache3:%d cache4:%d\n",
    ret,tag,cache1,cache2,cache3,cache4);
  mode_ptr=1;

  //cache hit
  ret=*(int *)(TEST_ADDR);

  mode_ptr=0;
  printf("after addr1 hit, return:%#010x\n",ret);
  mode_ptr=1;

  //cache hit+4
  ret=*(int *)(TEST_ADDR+4);

  mode_ptr=0;
  printf("after addr1+4 hit, return:%#010x\n",ret);
  mode_ptr=1;

  //cache hit+8
  ret=*(int *)(TEST_ADDR+8);

  mode_ptr=0;
  printf("after addr1+8 hit, return:%d\n",ret);
  mode_ptr=1;

  //cache hit+12
  ret=*(int *)(TEST_ADDR+12);

  mode_ptr=0;
  printf("after addr1+12 hit, return:%d\n",ret);
  mode_ptr=1;

  //save new value to first test address
  *(int *)(TEST_ADDR)=20;

  asm volatile("nop;"::); //need a nop before reading tag/cache

  //fetch tag and cache value
  tag=*(int *)(TAG_ADDR);
  cache1=*(int *)(CACHE_ADDR);
  cache2=*(int *)(CACHE_ADDR+4);
  cache3=*(int *)(CACHE_ADDR+8);
  cache4=*(int *)(CACHE_ADDR+12);

  mode_ptr=0;
  printf("after addr1 write hit, tag:%#010x cache1:%d cache2:%#010x cache3:%d cache4:%d\n",
    tag,cache1,cache2,cache3,cache4);
  mode_ptr=1;

  //provoke irrelevant cache miss
  ret=*(int *)(TEST_ADDR3);

  asm volatile("nop;"::); //need a nop before reading tag/cache

  //fetch tag and cache value
  tag=*(int *)(TAG_ADDR3);
  cache1=*(int *)(CACHE_ADDR3);

  mode_ptr=0;
  printf("after addr3 miss, return:%d tag:%#010x cache:%d\n",ret,tag,cache1); 
  mode_ptr=1;

  //verify that the other tag/cache stuff is still correct
  tag=*(int *)(TAG_ADDR);
  cache1=*(int *)(CACHE_ADDR);
  cache2=*(int *)(CACHE_ADDR+4);
  cache3=*(int *)(CACHE_ADDR+8);
  cache4=*(int *)(CACHE_ADDR+12);

  mode_ptr=0;
  printf("addr1 contents, tag:%#010x cache1:%d cache2:%#010x cache3:%d cache4:%d\n",
    tag,cache1,cache2,cache3,cache4);
  mode_ptr=1; 

  //provoke cache miss second address
  ret=*(int *)(TEST_ADDR2);

  asm volatile("nop;"::); //need a nop before reading tag/cache

  //fetch tag and cache value
  tag=*(int *)(TAG_ADDR);
  cache1=*(int *)(CACHE_ADDR);
  cache2=*(int *)(CACHE_ADDR+4);
  cache3=*(int *)(CACHE_ADDR+8);
  cache4=*(int *)(CACHE_ADDR+12);

  mode_ptr=0;
  printf("addr2 miss, return:%d tag:%#010x cache1:%d cache2:%d cache3:%d cache4:%d\n",
    ret,tag,cache1,cache2,cache3,cache4);

  return 0;
}
