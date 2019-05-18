#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "cmd_def.h"

char *str_class[ble_cls_last+1];

#define M(X) str_class[X] = #X
void dump_init(void) {
  M(ble_cls_system);
  M(ble_cls_flash);
  M(ble_cls_attributes);
  M(ble_cls_connection);
  M(ble_cls_attclient);
  M(ble_cls_sm);
  M(ble_cls_gap);
  M(ble_cls_hardware);
  M(ble_cls_test);
  M(ble_cls_dfu);
  M(ble_cls_last);
}

#define P(X) printf("  uint8 " #X ": %u\n",(int)p->X)
#define Pcls(X) printf("  uint8 " #X ": %u (%s)\n",(int)p->X,str_class[p->X])
void dump_ble_header(struct ble_header *p) {
  printf("struct ble_header (%p) {\n",p);
  P(type_hilen);
  P(lolen);
  Pcls(cls);
  P(command);
  printf("}\n");
}
#undef P

void dump_ble_msg(struct ble_msg *p,void *data) {
  printf("struct ble_msg (%p) {\n",p);
  printf("  uint32 params: 0x%08lx\n",p->params);
  uint32_t t = p->params;
  uint8_t len = 0, dlen;
  uint8_t vflag = 0;
  char buf[1024], *ptr = buf;
  while(t) {
    char *fmt = NULL;
    int v;
    switch(t & 0xf) {
    case 2:
      fmt = "    uint8: %u\n";
      v = *(uint8_t*)data;
    case 3:
      if(!fmt) {
	fmt = "    int8 %d\n";
	v = *(int8_t*)data;
      }
      ptr += sprintf(ptr,fmt,v);
      data += 1;
      len += 1;
      break;
    case 4:
      fmt = "    uint16: %u\n";
      v = *(uint16_t*)data;
    case 5:
      if(!fmt) {
	fmt = "    int16 %d\n";
	v = *(int16_t*)data;
      }
      ptr += sprintf(ptr,fmt,v);
      data += 2;
      len += 2;
      break;
    case 6:
      fmt = "    uint32: %u\n";
      v = *(uint32_t*)data;
    case 7:
      if(!fmt) {
	fmt = "    int32 %d\n";
	v = *(int32_t*)data;
      }
      ptr += sprintf(ptr,fmt,v);
      data += 4;
      len += 4;
      break;
    case 8:
    case 9:
      dlen = *((uint8_t*)data);
      data += 1;
      ptr += sprintf(ptr,"    uint8array: %u{",(int)dlen);
      for(int i = 0; i < dlen; i++) {
	ptr += sprintf(ptr,"%s%02x",(i)?",":"",((uint8_t*)data)[i]);
      }
      data += dlen;
      ptr += sprintf(ptr,"}\n");
      len += 1;
      vflag = 1;
      break;
    case 10:
      ptr += sprintf(ptr,"    bd_addr: {.addr: ");
      for(int i = 0; i < 6; i++) {
	ptr += sprintf(ptr,"%s%02x",(i)?":":"",((uint8_t*)data)[5-i]);
      }
      ptr += sprintf(ptr,"}\n");
      data += 6;
      len += 6;
      break;
    default:
      fprintf(stderr,"Unhandled nibble: %x\n",t&0xf);
      exit(1);
    }
    t >>= 4;
  }
  printf("} len %s %u\n",(vflag)?">=":"==",len);
  printf("%s",buf);
}
