//
// Created by Will Gulian on 10/27/19.
//

#include "testbench.h"
#include "Vtl45_comp.h"

int main(int argc, char **argv) {
  Verilated::commandArgs(argc, argv);
  TESTBENCH<Vtl45_comp> *tb = new TESTBENCH<Vtl45_comp>();

  auto &ram = tb->m_core->tl45_comp__DOT__my_mem__DOT__mem;

  // TODO jump encoding should not be shifted now
  // TODO SW encoding wrong

//  uint32_t init[] = {
//      0x0D100000,
//      0x0D200001,
//      0x0D30000A,
//      0x0E500100,
//      0x65400034,
//      0x08112000,
//      0x08420000,
//      0x08210000,
//      0x08140000,
//      0xA9250000,
//      0x0D33FFFF,
//      0x65F00010,
//      0x08402000,
//      0x65F00034,
//  };
//
//  memcpy(ram, init, sizeof(init));

  FILE *f = fopen("/Users/will/Work/transfer-learning/llvm-tl45/llvm/bbb/a.out", "r");
  if (!f) {
    printf("Fail!\n");
    return 1;
  }

  size_t mem_ptr = 0;

  unsigned char temp[4];
  int read = 0;
  while (true) {

    int r = fread(temp, 1, 4 - read, f);
    if (r == 0) {
      break;
    }

    read += r;

    if (read == 4) {
      ram[mem_ptr] = temp[0u] << 24u | temp[1u] << 16u | temp[2u] << 8u | temp[3u];
      printf("0x%08x\n", ram[mem_ptr]);
      read = 0;
      mem_ptr++;
    }
  }

  fclose(f);


//  ram[0] = 0x0d100001; // ADD r1, r0, 1
//  ram[1] = 0x08111000; // ADD r1, r1, r1
//  ram[2] = 0x08111000; // ADD r1, r1, r1
//  ram[3] = 0x08111000; // ADD r1, r1, r1

//  ram[0] = 0x0d106969; // ADD r1, r0, 0x6969
//  ram[1] = 0b10101001000100000100001001000000;
//  ram[5] = 0b10100001001000000100001001000100;
//  ram[6] = 0b10100001001100000100001001000000;


  tb->opentrace("trace.vcd");

  while(!tb->done() && tb->m_tickcount < 100 * 20) {
    tb->tick();
  }

  exit(EXIT_SUCCESS);
}