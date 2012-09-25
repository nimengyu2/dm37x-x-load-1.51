/*
 * Copyright (C) 2005 Texas Instruments.
 *
 * (C) Copyright 2004
 * Jian Zhang, Texas Instruments, jzhang@ti.com.
 *
 * (C) Copyright 2002
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 *
 * (C) Copyright 2002
 * Sysgo Real-Time Solutions, GmbH <www.elinos.com>
 * Marius Groeger <mgroeger@sysgo.de>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <asm/arch/mem.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/bits.h>
#include <asm/arch/cpu.h>
#include <asm/arch/bits.h>
#include <asm/arch/mux.h>
#include <asm/arch/sys_info.h>
#include <asm/arch/clocks.h>

#define __raw_readl(a)    (*(volatile unsigned int *)(a))
#define __raw_writel(v,a) (*(volatile unsigned int *)(a) = (v))
#define __raw_readw(a)    (*(volatile unsigned short *)(a))
#define __raw_writew(v,a) (*(volatile unsigned short *)(a) = (v))

extern int misc_init_r (void);
extern u32 get_mem_type(void);

#ifdef CFG_PRINTF
int print_info(void)
{
	printf("\n\nYU Texas Instruments X-Loader 1.51 ("
			__DATE__ " - " __TIME__ ")\n");
	return 0;
}
#endif

typedef int (init_fnc_t) (void);

init_fnc_t *init_sequence[] = {
	cpu_init,		/* basic cpu dependent setup */
	board_init,		/* basic board dependent setup */
#ifdef CFG_PRINTF
 	serial_init,		/* serial communications setup */
	print_info,
#endif
  	nand_init,		/* board specific nand init */
  	NULL,
};
extern unsigned int is_ddr_166M;
void start_armboot (void)
{
  	init_fnc_t **init_fnc_ptr;
 	int i;
	uchar *buf;

   	for (init_fnc_ptr = init_sequence; *init_fnc_ptr; ++init_fnc_ptr) {
		if ((*init_fnc_ptr)() != 0) {
			hang ();
		}
	}
	#ifdef CFG_PRINTF
	printf("after init_sequence\n");
	#endif

	misc_init_r();
	#ifdef CFG_PRINTF
	printf("after misc_init_r\n");
	#endif
	buf =  (uchar*) CFG_LOADADDR;
	#ifdef CFG_PRINTF
	printf("after (uchar*) CFG_LOADADDR\n");
	#endif

	/* Always first try mmc without checking boot pins */
#ifndef CONFIG_OMAP3_BEAGLE
	#ifdef CFG_PRINTF
	printf("after ndef CONFIG_OMAP3_BEAGLE\n");
	#endif
	//if ((get_mem_type() == MMC_ONENAND) || (get_mem_type() == MMC_NAND))
#endif	/* CONFIG_OMAP3_BEAGLE */
		buf += mmc_boot(buf);

	#ifdef CFG_PRINTF
	printf("after mmc_boot(buf)\n");
	#endif
	if (buf == (uchar *)CFG_LOADADDR) {
		if (get_mem_type() == GPMC_NAND){
#ifdef CFG_PRINTF
			printf("Booting from nand . . .\n");
#endif
			for (i = NAND_UBOOT_START; i < NAND_UBOOT_END; i+= NAND_BLOCK_SIZE){
				if (!nand_read_block(buf, i))
					buf += NAND_BLOCK_SIZE; /* advance buf ptr */
			}
		}

		if (get_mem_type() == GPMC_ONENAND){
#ifdef CFG_PRINTF
			printf("Booting from onenand . . .\n");
#endif
			for (i = ONENAND_START_BLOCK; i < ONENAND_END_BLOCK; i++){
				if (!onenand_read_block(buf, i))
					buf += ONENAND_BLOCK_SIZE;
			}
		}
	}

#if defined (CONFIG_AM3517EVM)
	/*
	 * FIXME: Currently coping uboot image,
	 * ideally we should leverage XIP feature here
	 */
	#ifdef CFG_PRINTF
	printf("after defined (CONFIG_AM3517EVM)\n");
	#endif
	if (get_mem_type() == GPMC_NOR) {
		int size;
		printf("Booting from NOR Flash...\n");
		size = nor_read_boot(buf);
		if (size > 0)
			buf += size;
	}
#endif
	#ifdef CFG_PRINTF
	printf("before buf == (uchar *)CFG_LOADADDR\n");
	#endif
	if (buf == (uchar *)CFG_LOADADDR)
		hang();
	#ifdef CFG_PRINTF
	printf("after buf == (uchar *)CFG_LOADADDR\n");
	#endif
	printf("-----------------------------------------------------------------\n");
	printf("AAAAAA:is_cpu_family()=%d\n",is_cpu_family());
	printf("AAAAAA:cpu_is_3410()=%d\n",cpu_is_3410());
	printf("AAAAAA:get_mem_type()=%d\n",get_mem_type());
	printf("AAAAAA:get_device_type()=%d\n",get_device_type());
	printf("AAAAAA:get_sysboot_value()=%d\n",get_sysboot_value());
	printf("AAAAAA:get_cpu_rev()=%d\n",get_cpu_rev());
	printf("AAAAAA:is_ddr_166M=%d\n",is_ddr_166M);

	printf("-----------------------------------------------------------------\n");
#if defined(PRCM_CLK_CFG2_200MHZ)
	printf("AAAAAA:PRCM_CLK_CFG2_200MHZ\n");
#elif defined (PRCM_CLK_CFG2_266MHZ)
	printf("AAAAAA:PRCM_CLK_CFG2_266MHZ\n");
#elif  defined(PRCM_CLK_CFG2_332MHZ)
	printf("AAAAAA:PRCM_CLK_CFG2_332MHZ\n");
#endif	
	printf("-----------------------------------------------------------------\n");
#if defined(L3_100MHZ)
	printf("AAAAAA:L3_100MHZ\n");
#elif defined(L3_133MHZ)
	printf("AAAAAA:L3_133MHZ\n");
#elif  defined(L3_165MHZ)
	printf("AAAAAA:L3_165MHZ\n");
#endif
	printf("-----------------------------------------------------------------\n");
	printf("AAAAAA:SDP_SDRC_RFR_CTRL =0x%08x\n",SDP_SDRC_RFR_CTRL );
	printf("AAAAAA:SDP_3430_SDRC_RFR_CTRL_100MHz=0x%08x\n",SDP_3430_SDRC_RFR_CTRL_100MHz);
	printf("AAAAAA:SDP_3430_SDRC_RFR_CTRL_133MHz=0x%08x\n",SDP_3430_SDRC_RFR_CTRL_133MHz);
	printf("AAAAAA:SDP_3430_SDRC_RFR_CTRL_165MHz=0x%08x\n",SDP_3430_SDRC_RFR_CTRL_165MHz);
	printf("-----------------------------------------------------------------\n");
	printf("AAAAAA:INFINEON_SDRC_ACTIM_CTRLA_0=0x%08x\n",INFINEON_SDRC_ACTIM_CTRLA_0);
	printf("AAAAAA:INFINEON_SDRC_ACTIM_CTRLB_0=0x%08x\n",INFINEON_SDRC_ACTIM_CTRLB_0);
	printf("AAAAAA:INFINEON_V_ACTIMA_100=0x%08x\n",INFINEON_V_ACTIMA_100);
	printf("AAAAAA:INFINEON_V_ACTIMB_100=0x%08x\n",INFINEON_V_ACTIMB_100);
	printf("AAAAAA:INFINEON_V_ACTIMA_133=0x%08x\n",INFINEON_V_ACTIMA_133);
	printf("AAAAAA:INFINEON_V_ACTIMB_133=0x%08x\n",INFINEON_V_ACTIMB_133);
	printf("AAAAAA:INFINEON_V_ACTIMA_165=0x%08x\n",INFINEON_V_ACTIMA_165);
	printf("AAAAAA:INFINEON_V_ACTIMB_165=0x%08x\n",INFINEON_V_ACTIMB_165);
	printf("-----------------------------------------------------------------\n");
	printf("AAAAAA:MICRON_SDRC_ACTIM_CTRLA_0=0x%08x\n",MICRON_SDRC_ACTIM_CTRLA_0);
	printf("AAAAAA:MICRON_SDRC_ACTIM_CTRLB_0=0x%08x\n",MICRON_SDRC_ACTIM_CTRLB_0);
	printf("AAAAAA:MICRON_V_ACTIMA_100=0x%08x\n",MICRON_V_ACTIMA_100);
	printf("AAAAAA:MICRON_V_ACTIMB_100=0x%08x\n", MICRON_V_ACTIMB_100);
	printf("AAAAAA:MICRON_V_ACTIMA_133=0x%08x\n",MICRON_V_ACTIMA_133);
	printf("AAAAAA:MICRON_V_ACTIMB_133=0x%08x\n", MICRON_V_ACTIMB_133);
	printf("AAAAAA:MICRON_V_ACTIMA_165=0x%08x\n",MICRON_V_ACTIMA_165);
	printf("AAAAAA:MICRON_V_ACTIMB_165=0x%08x\n", MICRON_V_ACTIMB_165);
	printf("-----------------------------------------------------------------\n");
	printf("AAAAAA:REG SDRC_ACTIM_CTRLA_0=0x%08x\n",__raw_readl(SDRC_ACTIM_CTRLA_0));
	printf("AAAAAA:REG SDRC_ACTIM_CTRLB_0=0x%08x\n",__raw_readl(SDRC_ACTIM_CTRLB_0));
	printf("AAAAAA:REG SDRC_RFR_CTRL_0=0x%08x\n",__raw_readl(SDRC_RFR_CTRL_0));
	printf("AAAAAA:REG SDRC_POWER=0x%08x\n",__raw_readl(SDRC_POWER));
	printf("AAAAAA:REG SDRC_MANUAL_0=0x%08x\n",__raw_readl(SDRC_MANUAL_0));
	printf("AAAAAA:REG SDRC_MR_0=0x%08x\n",__raw_readl(SDRC_MR_0));
	printf("AAAAAA:REG SDRC_DLLA_CTRL=0x%08x\n",__raw_readl(SDRC_DLLA_CTRL));
	//printf("AAAAAA:REG xxxx=0x%08x\n",__raw_readl(xxxx));
	printf("-----------------------------------------------------------------\n");
	//printf("AAAAAA:=%d\n",);
	//printf("AAAAAA:REG xxxx=0x%08x\n",__raw_readl(xxxx));
	/* go run U-Boot and never return */
  	printf("Starting OS Bootloader...\n");
	//while(1);
 	((init_fnc_t *)CFG_LOADADDR)();

	/* should never come here */
}

void hang (void)
{
	/* call board specific hang function */
	board_hang();

	/* if board_hang() returns, hange here */
	printf("X-Loader hangs\n");
	for (;;);
}
