/**
  *******************************************************************
  * Chip: LM3S9B96
  * NOTE: This file is generated by CoIDE and Included by link.ld
  *       It is used to describe which memory regions may be used
  *       by the linker.
  *******************************************************************
  * Internal memory map
  *   Region      Start           Size
  *   flash0      0x00000000      0x00040000
  *   sram0       0x20000000      0x00018000
  *******************************************************************
*/

MEMORY
{
	rom (rx)  : ORIGIN = 0x00000000, LENGTH = 0x00040000
	ram (rwx) : ORIGIN = 0x20000000, LENGTH = 0x00018000
}

_eram = 0x20000000 + 0x00018000;