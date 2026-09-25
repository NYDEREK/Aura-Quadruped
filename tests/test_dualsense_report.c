#include <assert.h>
#include <string.h>
#include "dualsense_report.h"
int main(void) {
    dualsense_controls_t out;
    const uint8_t neutral[] = {0x7d,0x7e,0x83,0x82,8,0,0,0,0};
    assert(dualsense_decode(1,neutral,sizeof(neutral),&out));
    assert(out.axes[0]==0x7d && out.buttons[0]==8 && out.triggers[0]==0);
    uint8_t pressed[] = {1,2,3,4,0x28,0x10,0xff,200,255};
    assert(dualsense_decode(1,pressed,9,&out));
    assert(out.buttons[0]==0x28 && out.buttons[1]==0x10 && out.buttons[2]==3);
    assert(out.triggers[0]==200 && out.triggers[1]==255);
    assert(!dualsense_decode(1,pressed,8,&out));
    uint8_t full[77]={0}; full[0]=0x51; full[1]=123;full[5]=210;full[8]=0x48;full[10]=4;
    assert(dualsense_decode(0x31,full,77,&out));
    assert(out.axes[0]==123 && out.triggers[0]==210 && out.buttons[0]==0x48 && out.buttons[2]==4);
    assert(!dualsense_decode(0x31,full,76,&out));
    assert(!dualsense_decode(0xff,full,77,&out));
}
