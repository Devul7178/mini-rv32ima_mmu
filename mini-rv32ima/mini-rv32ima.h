// Copyright 2022 Charles Lohr, you may use this file or any portions herein under any of the BSD, MIT, or CC0 licenses.

// Some resources I used:
// Mini-rv32IMA:
// https://github.com/cnlohr/mini-rv32ima

// CSR:
// https://five-embeddev.com/quickref/csrs.html
// https://book.rvemu.app/hardware-components/03-csrs.html#:~:text=The%20trap%20delegation%20registers%2C%20medeleg%20for%20machine-level%20exception,at%200x302%20and%20mideleg%20is%20allocated%20at%200x303.
// https://domipheus.com/blog/designing-a-risc-v-cpu-in-vhdl-part-18-control-and-status-register-unit/
// https://raw.githubusercontent.com/riscv/virtual-memory/main/specs/663-Svpbmt.pdf

// I/M/A extentions:
// https://riscv.org/wp-content/uploads/2017/05/riscv-spec-v2.2.pdf#page=21
// https://github.com/johnwinans/rvalp/releases
// https://msyksphinz-self.github.io/riscv-isadoc/html/rva.html

// Linux:
// https://github.com/torvalds/linux

// CLINT (interrupts and exceptions):
// https://chromitem-soc.readthedocs.io/en/latest/clint.html
// https://mullerlee.cyou/2020/07/09/riscv-exception-interrupt/#:~:text=We%20can%20enable%20interrupts%20in%20every%20mode%20by,and%20waiting%20the%20interrupt%20handler%20to%20reset%20it. 
// https://github.com/pulp-platform/clint/tree/master
// https://github.com/riscv/riscv-aclint

// TRAPS:
// https://stackoverflow.com/questions/64863737/risc-v-software-interrupts
// https://mi-v-ecosystem.github.io/SoftConsole-Documentation/SoftConsole-v2021.3/troubleshooting/riscv_trap.html 


#ifndef _MINI_RV32IMAH_H
#define _MINI_RV32IMAH_H

/**
    To use mini-rv32ima.h for the bare minimum, the following:

	#define MINI_RV32_RAM_SIZE ram_amt
	#define MINIRV32_IMPLEMENTATION

	#include "mini-rv32ima.h"

	Though, that's not _that_ interesting. You probably want I/O!


	Notes:
		* There is a dedicated CLNT at 0x10000000.
		* There is free MMIO from there to 0x12000000.
		* You can put things like a UART, or whatever there.
		* Feel free to override any of the functionality with macros.

	CLNT is simply a dedicated memory-mapped I/O (MMIO) region at the physical memory 
	address 0x10000000 in the mini-rv32ima system that is dedicated for a 
	"client" device, such as a UART, to communicate with the mini-rv32ima processor.

	CLNT is responsible for maintaining memory mapped control and status registers which 
	are associated with the software and timer interrupts.

	users can define their own peripherals in this region, or use existing ones, and interact
	with them through the memory-mapped I/O interface 
*/

#ifndef MINIRV32WARN
	#define MINIRV32WARN( x... );
#endif

#ifndef MINIRV32_DECORATE
	#define MINIRV32_DECORATE static
#endif

// The RAM image starts at 0x80000000 
#ifndef MINIRV32_RAM_IMAGE_OFFSET
	#define MINIRV32_RAM_IMAGE_OFFSET  0x80000000
#endif

#ifndef MINIRV32_POSTEXEC
	#define MINIRV32_POSTEXEC(...);
#endif

#ifndef MINIRV32_HANDLE_MEM_STORE_CONTROL
	#define MINIRV32_HANDLE_MEM_STORE_CONTROL(...);
#endif

#ifndef MINIRV32_HANDLE_MEM_LOAD_CONTROL
	#define MINIRV32_HANDLE_MEM_LOAD_CONTROL(...);
#endif

#ifndef MINIRV32_OTHERCSR_WRITE
	#define MINIRV32_OTHERCSR_WRITE(...);
#endif

#ifndef MINIRV32_OTHERCSR_READ
	#define MINIRV32_OTHERCSR_READ(...);
#endif

// They are used to read and write data from/to memory locations based on 
// the byte offset (ofs) within a memory image.
#ifndef MINIRV32_CUSTOM_MEMORY_BUS
	#define MINIRV32_STORE4( ofs, val ) *(uint32_t*)(image + ofs) = val		// sw
	#define MINIRV32_STORE2( ofs, val ) *(uint16_t*)(image + ofs) = val		// sh/sl
	#define MINIRV32_STORE1( ofs, val ) *(uint8_t*)(image + ofs) = val		// sb
	#define MINIRV32_LOAD4( ofs ) *(uint32_t*)(image + ofs)					// lw
	#define MINIRV32_LOAD2( ofs ) *(uint16_t*)(image + ofs)					// lh/ll
	#define MINIRV32_LOAD1( ofs ) *(uint8_t*)(image + ofs)					// lb
#endif

// As a note: We quouple-ify these, because in HLSL, we will be operating with
// uint4's.  We are going to uint4 data to/from system RAM.
// We're going to try to keep the full processor state to 12 x uint4.
struct MiniRV32IMAState
{
	uint32_t regs[32];		// 32 general purpose registers

	uint32_t pc;			// stores the address of the next instruction to be executed
	uint32_t mstatus;		// machine status register (which holds the current operating state of the hart)
	uint32_t cyclel;		// stores lower 32 bits of cycle count 
	uint32_t cycleh;		// stores upper 32 bits of cycle count 

	uint32_t timerl;		// stores lower 32 bits of timer
	uint32_t timerh;		// stores upper 32 bits of timer
	uint32_t timermatchl;	// stores lower 32 bits of the timer match register
							// Used to trigger an interrupt when the timer value matches the match value.
	uint32_t timermatchh;	// stores upper 32 bits of timer match register

	uint32_t mscratch;		// machine scratch register for machine trap handlers
	uint32_t mtvec;			// Machine trap-handler base address.
	uint32_t mie;			// machine interrupt-enable register (enables/disables various interrupt sources)
	uint32_t mip;			// machine interrupt-pending register, which stores the pending status of various interrupt sources.

	uint32_t mepc;			// machine exception program counter register (holds the address of the instruction 
							// that caused the most recent trap or exception.
	uint32_t mtval;			// machine trap value register, which holds the value (bad address or instruction)
							// that caused the most recent trap or exception.
	uint32_t mcause;		// machine trap cause register, which stores the cause of the most recent trap or exception.

	uint32_t satp; 
	// Note: only a few bits are used.  (Machine = 3, User = 0)
	// Bits 0..1 = privilege.
	// Bit 2 = WFI (Wait for interrupt)
	// Bit 3+ = Load/Store reservation LSBs.
	uint32_t extraflags;
};

MINIRV32_DECORATE int32_t MiniRV32IMAStep( struct MiniRV32IMAState * state, uint8_t * image, uint32_t vProcAddress, uint32_t elapsedUs, int count );

#ifdef MINIRV32_IMPLEMENTATION

#define CSR( x ) state->x
#define SETCSR( x, val ) { state->x = val; }
#define REG( x ) state->regs[x]
#define REGSET( x, val ) { state->regs[x] = val; }

#define LEVELS 2
#define PAGESIZE 4096
#define PTESIZE 4

#define LOAD 0
#define STORE 1
#define EXECUTE 2
#define ENV_CALL 3
#define ILLEGAL_INSTR 4
#define BREAKPOINT 5

#define PRIV_USER 0
#define PRIV_SUPERVISOR 1
#define PRIV_MACHINE 3

#define MISALIGNED 0
#define ACCESS 1
#define PAGEFAULT 2


uint32_t trap_exit_code (uint32_t type_of_fault, uint32_t type_of_access){
	if (type_of_access == LOAD){ \
		if (type_of_fault == MISALIGNED) { 
			return 0x5;
		} else if (type_of_fault == ACCESS) { 
			return 0x6;
		} else if (type_of_fault == PAGEFAULT) {
			return 0xE;
		}
	} else if (type_of_access == STORE) { 
		if (type_of_fault == MISALIGNED) { 
			return 0x7;
		} else if (type_of_fault == ACCESS) { 
			return 0x8;
		} else if (type_of_fault == PAGEFAULT) {
			return 0x10;
		}
	} else if (type_of_access == EXECUTE) { 
		if (type_of_fault == MISALIGNED) { 
			return 0x1;
		} else if (type_of_fault == ACCESS) { 
			return 0x2;
		} else if (type_of_fault == PAGEFAULT) {
			return 0xD;
		}	
	} else if (type_of_access == ENV_CALL) {
		if (type_of_fault == PRIV_USER) { 
			return 0x9;
		} else if (type_of_fault == PRIV_SUPERVISOR) { 
			return 0xA;
		} else if (type_of_fault == PRIV_MACHINE) {
			return 0xC;
		}
	} else if (type_of_access == ILLEGAL_INSTR) {
		return 0x3;
	} else if (type_of_access == BREAKPOINT) {
		return 0x4;
	}

	return 0;
}

typedef struct {
    uint8_t v, r, w, x, u, g, a, d;
    uint32_t rsw;
    uint32_t ppn0;
    uint32_t ppn1;
} mmu_pte;


mmu_pte load_page(uint32_t addr, uint8_t * image) {
    uint32_t data = MINIRV32_LOAD4( addr );
    mmu_pte ret;
    #define BOOL(name, bit) ret.name = (data >> bit) & 0x1;
    BOOL(v, 0)
    BOOL(r, 1)
    BOOL(w, 2)
    BOOL(x, 3)
    BOOL(u, 4)
    BOOL(g, 5)
    BOOL(a, 6)
    BOOL(d, 7)
    #undef BOOL
    ret.rsw = (data >> 8) & 0x3;
    ret.ppn0 = (data >> 10) & 0x3ff;
    ret.ppn1 = (data >> 20) & 0xfff;
    return ret;
}

uint64_t mmu_translate(struct MiniRV32IMAState * state, uint32_t vpn_addr, uint32_t access_type, uint8_t * image, uint32_t * trap) {
    uint32_t satp = CSR( satp );
	uint32_t mode = satp >> 30;
	uint32_t ppn = satp & 0x3fffff;

	// No MMU
    if (mode == 0) {
        return vpn_addr;
    }

	uint32_t mstatus = CSR( mstatus );

    uint8_t sum = (mstatus >> 18) & 0x1;
    uint8_t mxr = (mstatus >> 19) & 0x1;
	
	uint8_t curr_priv = CSR( extraflags ) & 0x3; 
	uint8_t mprv = (mstatus >> 17) & 0x1;	
	uint8_t mpp = (mstatus >> 11) & 0x3;

	uint8_t mprv_priv;

	//MPRV = Modify Privilege (allows M-mode to temporarily change the privilege level for load and store operations)
    if (mprv) mprv_priv = mpp;
	else mprv_priv = curr_priv;

	// For fetch instructions in machine mode, the MMU will not be used because the privilege 
	// level is machine and the access mode is fetch. This ensures that the machine mode can 
	// always execute code without being affected by the MMU settings. 
    if ((mprv_priv == PRIV_MACHINE || curr_priv == PRIV_MACHINE) && access_type == EXECUTE) {
        return vpn_addr;
    }

	// Otherwise, if the current privilege mode is not machine mode or the access type is not fetch, 
	// then the MMU translation will be used, and the value of 'mxr' will determine whether a page 
	// marked as read-only can also be executable. The translated physical address will be returned.

	// We now use MMU for load/stores for machine, supervisor, and user mode and fetch for supervisor and user mode.

	// find the top of level 2's page table
    uint32_t page_table_addr = ppn * PAGESIZE;
	mmu_pte pte;
	uint8_t superpage = 0;

    for (uint8_t i = LEVELS - 1; i >= 0; i--) {
		uint32_t pt_idx = (vpn_addr >> (22 - 10 * i)) & 0x3FF;
        uint32_t pte_addr = page_table_addr + pt_idx * PTESIZE;
        pte = load_page(pte_addr, image);

		if(pte.v == 0 || (pte.w == 1 && pte.r == 0)){
			*trap = trap_exit_code(access_type, PAGEFAULT);
			return 0;
		}

		if(pte.x == 0 && pte.r == 0 && pte.w == 0 && i != 0){
			// branch entry
			if (i != 0) {
				page_table_addr = (pte.ppn1 << 22) | (pte.ppn0 << 12);
			}
			else {
				*trap = trap_exit_code(access_type, ACCESS); 
				return 0;
			}
		} else {
			// leaf entry
			// misaligned super page (a superpage that does not start at a multiple of its size in the physical address space)
			if (i > 0) {
				superpage = 1;
				if (pte.ppn0 != 0) {
					*trap = trap_exit_code(access_type, MISALIGNED);
					return 0;
				}
			}
			
			break;
		}
    }
	
	if ((access_type == LOAD && !pte.r) || 						// load page fault
		(access_type == STORE && pte.w == 0) || 				// store page fault
		(access_type == EXECUTE && (pte.x == 0 || mxr == 0))){  // instruction page fault
		*trap = trap_exit_code(access_type, PAGEFAULT);
		return 0;
	}
	// PTE has been found, permission check. If perm not there, access page fault for corresponding access_type
	if ((mprv_priv == PRIV_MACHINE) || 							// machine can read/store everything
		(mprv_priv == PRIV_USER && pte.u) || 					// this is a user pte
		(mprv_priv == PRIV_SUPERVISOR && (!pte.u || sum))){		// supervisor pte or SUM) 
		*trap = trap_exit_code(access_type, ACCESS);	
		return 0;
	}	

	if (!pte.a || (access_type == STORE && pte.d == 0)) {
		*trap = trap_exit_code(access_type, ACCESS);
		return 0;
    }

	// translation success
    uint64_t pa = vpn_addr & 0xfff;
    pa |= superpage ? ((vpn_addr >> 12) & 0x3ff) << 12 : pte.ppn0 << 12;
    pa |= pte.ppn1 << 22;

	return pa;
}

// vProcAddress is the virtual address (set to 0 for now since nommu), count is instr_per_flip it is either 0 or 1024
MINIRV32_DECORATE int32_t MiniRV32IMAStep( struct MiniRV32IMAState * state, uint8_t * image, uint32_t vProcAddress, uint32_t elapsedUs, int count )
{
	//. If the timer value overflows, the upper 32 bits are incremented. 
	// For instance if timerl is 0xFFFFFFFE and time elapsed is 4. Then timerh will be 0x00000001 and timerl is 0x00000002 (overflow wrap around 0)
	// final timer is 0x0000000100000002
	uint32_t new_timer = CSR( timerl ) + elapsedUs;
	if( new_timer < CSR( timerl ) ) CSR( timerh )++;
	CSR( timerl ) = new_timer;

	// Handle Timer interrupt.
	// Checks if a timer interrupt needs to be fired based on whether the current timer 
	// value exceeds the timer match value (don't do this if timermatch has not been set yet). 
	// If it does, it clears the WFI (Wait for interrupt) bit and sets the MTIP (Machine Timer 
	// Interrupt Pending) bit in the mip (Machine Interrupt Pending) register to fire an interrupt.
	if( ( CSR( timerh ) > CSR( timermatchh ) || ( CSR( timerh ) == CSR( timermatchh ) && CSR( timerl ) > CSR( timermatchl ) ) ) && ( CSR( timermatchh ) || CSR( timermatchl ) ) )
	{
		// Do not execute WFI (Wait for interrupt). WFI is set when we are waiting for an interrupt
		// to put the processor into a low-power state until an interrupt occurs. In this case we know
		// that the interrupt has occured so we must continue executing normally. 
		CSR( extraflags ) &= ~4; // Clear WFI

		// The MTIP bit in MIP is a status flag that indicates whether a timer interrupt is pending or not. 
		// When the MTIP bit is set to 1, it means that an interrupt from the machine timer has been requested 
		// and is waiting to be serviced by the processor. 
		CSR( mip ) |= 1<<7; //MTIP of MIP // https://stackoverflow.com/a/61916199/2926815  Fire interrupt.
	}
	else
		// No timer interrupt is waiting, so we clear the MTIP bit in mip.
		CSR( mip ) &= ~(1<<7);

	// If WFI, prevent the processor from executing any further instructions until the next interrupt occurs.
	if( CSR( extraflags ) & 4 )
		return 1;

	uint32_t trap = 0;
	uint32_t rval = 0;
	uint32_t pc = CSR( pc );
	uint32_t cycle = CSR( cyclel );

	// mip is set above
	// mie is set when you do csrrw x1, 0x304, (new value of mie with 4th bit set to 1)
	// mstatus is set when you do csrrw x1, 0x300, (new value of mstatus with 3rd bit set to 1)
	if( ( CSR( mip ) & (1<<7) ) && ( CSR( mie ) & (1<<7) /*mtie*/ ) && ( CSR( mstatus ) & 0x8 /*mie*/) )
	{
		// Timer interrupt.
		trap = 0x80000007;
		pc -= 4;
	}
	else 
	// No timer interrupt?  Execute a bunch of instructions.
	// Note that in the code above the timer interrupt is not fired yet. For that we need to set mie and mtie, linux does that through
	// code. My assumption is that the linux code checks and sees if a timer interrupt is pending through mip. If it is then it goes 
	// into a subroutine and executes instructions to set the mie and mstatus respectively. 
	for( int icount = 0; icount < count; icount++ )
	{
		// over here we are assuing that each instruction takes 1 clock cycle (risc5 pipelines processor)
		uint32_t ir = 0;
		rval = 0;
		cycle++;
		uint32_t ofs_pc = pc - MINIRV32_RAM_IMAGE_OFFSET;

		// Handle access violation on instruction read.
		if( ofs_pc  >= MINI_RV32_RAM_SIZE )
		{
			trap = trap_exit_code(EXECUTE, ACCESS);
			break;
		}
		//Handle PC-misaligned access
		else if( ofs_pc & 3 )
		{
			trap = trap_exit_code(EXECUTE, MISALIGNED);
			break;
		}
		else
		{	
			// if the pc is correct, then we load the instruction into ir
			uint64_t paddr = mmu_translate(state, ofs_pc, EXECUTE, image, &trap);

			if (trap != 0){
				rval = paddr;
				break;
			}

			ir = MINIRV32_LOAD4( paddr );

			if (icount == 992){
				printf("hello");
			}

			printf("ir: %x  icount: %d\n", ir, icount)
			
			// destination register
			uint32_t rdid = (ir >> 7) & 0x1f;
			
			// 7f is last 7 
			switch( ir & 0x7f )
			{
				case 0b0110111: // LUI (rd←immu, pc←pc+4)
					rval = ( ir & 0xfffff000 );
					break;
				case 0b0010111: // AUIPC (rd←pc+immu, pc←pc+4)
					rval = pc + ( ir & 0xfffff000 );	
					break;
				case 0b1101111: // JAL (rd←pc+4, pc←pc+immj)
				{
					int32_t reladdy = ((ir & 0x80000000)>>11) | ((ir & 0x7fe00000)>>20) | ((ir & 0x00100000)>>9) | ((ir&0x000ff000));
					if( reladdy & 0x00100000 ) reladdy |= 0xffe00000; // Sign extension.
					rval = pc + 4;
					pc = pc + reladdy - 4;
					break;
				}
				case 0b1100111: // JALR (rd←pc+4, pc←(rs1+immi)∧∼1)
				{
					uint32_t imm = ir >> 20;
					int32_t imm_se = imm | (( imm & 0x800 )?0xfffff000:0);
					rval = pc + 4;
					pc = ( (REG( (ir >> 15) & 0x1f ) + imm_se) & ~1) - 4;
					break;
				}
				case 0b1100011: // Branch (pc←pc+((rs1 {math op} rs2)?immb:4))
				{
					uint32_t immm4 = ((ir & 0xf00)>>7) | ((ir & 0x7e000000)>>20) | ((ir & 0x80) << 4) | ((ir >> 31)<<12);
					if( immm4 & 0x1000 ) immm4 |= 0xffffe000;
					int32_t rs1 = REG((ir >> 15) & 0x1f);
					int32_t rs2 = REG((ir >> 20) & 0x1f);
					immm4 = pc + immm4 - 4;
					// set to 00, because we don't want to write anything to rd in branches
					rdid = 0;
					switch( ( ir >> 12 ) & 0x7 )
					{
						case 0b000: if( rs1 == rs2 ) pc = immm4; break;						//BEQ
						case 0b001: if( rs1 != rs2 ) pc = immm4; break;						//BNE
						case 0b100: if( rs1 < rs2 ) pc = immm4; break;						//BLT
						case 0b101: if( rs1 >= rs2 ) pc = immm4; break; 					//BGE
						case 0b110: if( (uint32_t)rs1 < (uint32_t)rs2 ) pc = immm4; break;	//BLTU
						case 0b111: if( (uint32_t)rs1 >= (uint32_t)rs2 ) pc = immm4; break;	//BGEU
						default: trap = trap_exit_code(ILLEGAL_INSTR, 0);					//Illegal instruction

						
					}
					break;
				}
				case 0b0000011: // Load 
				// lb -> rd←sx(m8(rs1+immi)),pc←pc+4
				// lh -> rd←sx(m16(rs1+immi)),pc←pc+4
				// lw -> rd←sx(m32(rs1+immi)),pc←pc+4
				// lbu -> rd←zx(m8(rs1+immi)),pc←pc+4
				// lhu -> rd←zx(m16(rs1+immi)),pc←pc+4
				{
					uint32_t rs1 = REG((ir >> 15) & 0x1f);
					uint32_t imm = ir >> 20;
					int32_t imm_se = imm | (( imm & 0x800 )?0xfffff000:0);
					uint32_t rsval = rs1 + imm_se;

					/***********************************************************************************************/
					/** Loading MMIO reg values into a register:
					 * CLINT-
						* To load the value of a memory mapped control and status registers which are associated with 
						* the software and timer interrupts into a register (rdid):
						* such as the register that stores upper/lower bits of a timer, we can do any of the load 
						* instruction at the memeory address for that MMIO register.
					 * UART-
						* Remember UART ports are also memory mapped in the same space as 0x10000000 to 0x12000000.
						* So to store the value of data / control status register into a register we simply use the
						* addresses 0x10000000 and 0x10000005 respectively. Similar to CLNT, we can use any of the load
						* instructions as long as the memory address is valid. 
					**/
					/***********************************************************************************************/

					rsval -= MINIRV32_RAM_IMAGE_OFFSET;
					if( rsval >= MINI_RV32_RAM_SIZE-3 )
					{
						rsval += MINIRV32_RAM_IMAGE_OFFSET;
						if( rsval >= 0x10000000 && rsval < 0x12000000 ) 
						{
							// the mtime clint control register provides the current timer value. 
							// lower 32 bits goes to 0xbff8 and upper bits goes to 0xbffc
							// We assume that 0x1100 is the base 
							if( rsval == 0x1100bffc ) // https://chromitem-soc.readthedocs.io/en/latest/clint.html
								rval = CSR( timerh );
							else if( rsval == 0x1100bff8 )
								rval = CSR( timerl );
							
							// If the address is 0x10000005 or 0x10000000, then we read from UART MMIO reg by using UART load.
							else
								MINIRV32_HANDLE_MEM_LOAD_CONTROL( rsval, rval );
						}
						// If the address being accessed is outside the valid range of memory (defined as the region starting 
						// at MINIRV32_RAM_IMAGE_OFFSET with a size of MINI_RV32_RAM_SIZE)
						// load access fault
						else
						{
							trap = trap_exit_code(LOAD, ACCESS);
							rval = rsval;
						}
					}
					// However, if the address being accessed is within the valid range, the memory access is 
					// performed using the appropriate MINIRV32_LOAD function based on the instruction type.
					else
					{
						uint64_t paddr = mmu_translate(state, rsval, LOAD, image, &trap);

						if (trap != 0){
							rval = rsval;
							break;
						}

						switch( ( ir >> 12 ) & 0x7 )
						{
							case 0b000: rval = (int8_t)MINIRV32_LOAD1( paddr ); break;	//LB
							case 0b001: rval = (int16_t)MINIRV32_LOAD2( paddr ); break;	//LH
							case 0b010: rval = MINIRV32_LOAD4( paddr ); break;			//LW
							case 0b100: rval = MINIRV32_LOAD1( paddr ); break;			//LBU
							case 0b101: rval = MINIRV32_LOAD2( paddr ); break;			//LHU
							default: trap = trap_exit_code(ILLEGAL_INSTR, 0);										//Illegal instruction
						}
					}
					break;
				}
				case 0b0100011: // Store
				{
					// sb -> m8(rs1+imms)←rs2[7:0], pc←pc+4
					// sh -> m16(rs1+imms)←rs2[15:0], pc←pc+4
					// sw -> m16(rs1+imms)←rs2[15:0], pc←pc+4
					uint32_t rs1 = REG((ir >> 15) & 0x1f);
					uint32_t rs2 = REG((ir >> 20) & 0x1f);
					uint32_t addy = ( ( ir >> 7 ) & 0x1f ) | ( ( ir & 0xfe000000 ) >> 20 );
					if( addy & 0x800 ) addy |= 0xfffff000;
					addy += rs1 - MINIRV32_RAM_IMAGE_OFFSET;

					// No destination register is being used
					rdid = 0;
					
					// Storing a value into MMIO reg
					if( addy >= MINI_RV32_RAM_SIZE-3 )
					{
						addy += MINIRV32_RAM_IMAGE_OFFSET;
						if( addy >= 0x10000000 && addy < 0x12000000 )
						{
							// The CLNT mtimecmp register is located at 0x11004000 to 0x11004004 
							// This register holds the compare value for the timer (timermatch)
							// Should be stuff like SYSCON, 8250, CLNT
							if( addy == 0x11004004 ) //CLNT
								CSR( timermatchh ) = rs2;
							else if( addy == 0x11004000 ) //CLNT
								CSR( timermatchl ) = rs2;
							// syscon (system configuration) are some MMIO regs used by LINUX to:
							// shutdown -> sw rs2, 0(rs1) -> where rs2 will be 3 and rs1 will 0x11100000
							// restart -> sw rs2, 0(rs1) -> where rs2 will be x7777 and rs1 will 0x11100000
							else if( addy == 0x11100000 )
							{
								SETCSR( pc, pc + 4 );
								return rs2; // NOTE: PC will be PC of Syscon.
							}

							// If the address is 0x10000005 or 0x10000000, then we store into UART MMIO reg by using UART store.
							// Remember this emulator we are simply displaying it on terminal
							else
								MINIRV32_HANDLE_MEM_STORE_CONTROL( addy, rs2 );
						}
						else
						{
							trap = trap_exit_code(STORE, ACCESS); // Store access fault.
							rval = addy;
						}
					}
					// However, if the address being accessed is within the valid range, the memory access is 
					// performed using the appropriate MINIRV32_STORE function based on the instruction type.
					else
					{
						uint64_t paddr = mmu_translate(state, addy, STORE, image, &trap);

						if (trap != 0){
							rval = paddr;
							break;
						}

						switch( ( ir >> 12 ) & 0x7 )
						{
							case 0b000: MINIRV32_STORE1( paddr, rs2 ); break;	//SB
							case 0b001: MINIRV32_STORE2( paddr, rs2 ); break;	//SH
							case 0b010: MINIRV32_STORE4( paddr, rs2 ); break;	//SW
							default: trap = trap_exit_code(ILLEGAL_INSTR, 0);								//Illegal instruction
						}
					}
					break;
				}
				// The R-type and the I-type instructions are very similar, so we do a fall through for the I case
				// and do all of the R-type and I-type calculations there. 
				case 0b0010011: // Op-immediate
				case 0b0110011: // Op
				{
					uint32_t imm = ir >> 20;
					imm = imm | (( imm & 0x800 )?0xfffff000:0);
					uint32_t rs1 = REG((ir >> 15) & 0x1f);
					// Whether it is an I type instruction or R type instruction is determined by the 6th bit of the instruction
					uint32_t is_reg = !!( ir & 0b100000 );
					// If the 6th bit is 1 then it is a R type instruction, if it is 0 it is an I type instruction
					uint32_t rs2 = is_reg ? REG(imm & 0x1f) : imm;

					//0x02000000 = RV32M. This is the RV32M extention to RV32I. 
					if( is_reg && ( ir & 0x02000000 ) )
					{
						switch( (ir>>12)&7 ) 
						{
							case 0b000: rval = rs1 * rs2; break; // MUL
							case 0b001: rval = ((int64_t)((int32_t)rs1) * (int64_t)((int32_t)rs2)) >> 32; break; // MULH
							case 0b010: rval = ((int64_t)((int32_t)rs1) * (uint64_t)rs2) >> 32; break; // MULHSU
							case 0b011: rval = ((uint64_t)rs1 * (uint64_t)rs2) >> 32; break; // MULHU
							case 0b100: if( rs2 == 0 ) rval = -1; else rval = ((int32_t)rs1 == INT32_MIN && (int32_t)rs2 == -1) ? rs1 : ((int32_t)rs1 / (int32_t)rs2); break; // DIV
							case 0b101: if( rs2 == 0 ) rval = 0xffffffff; else rval = rs1 / rs2; break; // DIVU
							case 0b110: if( rs2 == 0 ) rval = rs1; else rval = ((int32_t)rs1 == INT32_MIN && (int32_t)rs2 == -1) ? 0 : ((uint32_t)((int32_t)rs1 % (int32_t)rs2)); break; // REM
							case 0b111: if( rs2 == 0 ) rval = rs1; else rval = rs1 % rs2; break; // REMU
						}
					}
					else
					{
						switch( (ir>>12)&7 ) // These could be either op-immediate or op commands.  Be careful.
						{
							case 0b000: rval = (is_reg && (ir & 0x40000000) ) ? ( rs1 - rs2 ) : ( rs1 + rs2 ); break; 
							case 0b001: rval = rs1 << (rs2 & 0x1F); break;
							case 0b010: rval = (int32_t)rs1 < (int32_t)rs2; break;
							case 0b011: rval = rs1 < rs2; break;
							case 0b100: rval = rs1 ^ rs2; break;
							case 0b101: rval = (ir & 0x40000000 ) ? ( ((int32_t)rs1) >> (rs2 & 0x1F) ) : ( rs1 >> (rs2 & 0x1F) ); break;
							case 0b110: rval = rs1 | rs2; break;
							case 0b111: rval = rs1 & rs2; break;
						}
					}
					break;
				}
				// fencetype = (ir >> 12) & 0b111; We ignore fences in this impl.
				case 0b0001111:
					rdid = 0;   
					break;
				case 0b1110011: // Zifencei+Zicsr
				{
					uint32_t csrno = ir >> 20;
					// the microop decides between csrrw, csrrs, csrrc, csrrwi, csrrsi, csrrci
					int microop = ( ir >> 12 ) & 0b111;
					if( (microop & 3) ) // It's a Zicsr function.
					{
						// it is rs1 for csrrw, csrrs, csrrc and zimm for csrrwi, csrrsi, csrrci
						// csrrw -> rd←csr, csr←rs1, pc←pc+4
						// csrrs -> rd←csr, csr←csr ∨ rs1, pc←pc+4
						// csrrc -> rd←csr, csr←csr ∧ ∼rs1, pc←pc+4
						// csrrwi -> rd←csr, csr←zimm, pc←pc+4
						// csrrsi -> rd←csr, csr←csr ∨ zimm, pc←pc+4
						// csrrci -> rd←csr, csr←csr ∧ ∼zimm, pc←pc+4
						int rs1imm = (ir >> 15) & 0x1f;
						uint32_t rs1 = REG(rs1imm);
						uint32_t writeval = rs1;

						// https://raw.githubusercontent.com/riscv/virtual-memory/main/specs/663-Svpbmt.pdf
						// Generally, support for Zicsr
						switch( csrno )
						{
							case 0x340: rval = CSR( mscratch ); break;
							case 0x305: rval = CSR( mtvec ); break;
							case 0x304: rval = CSR( mie ); break;
							case 0xC00: rval = cycle; break;		
							case 0x344: rval = CSR( mip ); break;
							case 0x341: rval = CSR( mepc ); break;
							case 0x300: rval = CSR( mstatus ); break; 
							case 0x342: rval = CSR( mcause ); break;
							case 0x343: rval = CSR( mtval ); break;
							case 0x180: rval = CSR( satp ); break;
							case 0xf11: rval = 0xff0ff0ff; break; //machine vendor id -> https://five-embeddev.com/riscv-isa-manual/latest/machine.html#machine-vendor-id-register-mvendorid
							case 0x301: rval = 0x40401101; break; //machine ISA reg (XLEN=32, IMA+X) -> https://five-embeddev.com/riscv-isa-manual/latest/machine.html#sec:misa
							//case 0x3B0: rval = 0; break; //pmpaddr0
							//case 0x3a0: rval = 0; break; //pmpcfg0
							//case 0xf12: rval = 0x00000000; break; //marchid
							//case 0xf13: rval = 0x00000000; break; //mimpid
							//case 0xf14: rval = 0x00000000; break; //mhartid
							
							//CSRs 0x140 used by keyboard to read data from
							default:
								MINIRV32_OTHERCSR_READ( csrno, rval );
								break;
						}

						switch( microop )
						{
							case 0b001: writeval = rs1; break;  			//CSRRW
							case 0b010: writeval = rval | rs1; break;		//CSRRS
							case 0b011: writeval = rval & ~rs1; break;		//CSRRC
							case 0b101: writeval = rs1imm; break;			//CSRRWI
							case 0b110: writeval = rval | rs1imm; break;	//CSRRSI
							case 0b111: writeval = rval & ~rs1imm; break;	//CSRRCI
						}

						switch( csrno )
						{
							case 0x340: SETCSR( mscratch, writeval ); break;
							case 0x305: SETCSR( mtvec, writeval ); break;
							case 0x304: SETCSR( mie, writeval ); break;
							case 0x344: SETCSR( mip, writeval ); break;
							case 0x341: SETCSR( mepc, writeval ); break;
							case 0x300: SETCSR( mstatus, writeval ); break; //mstatus
							case 0x342: SETCSR( mcause, writeval ); break;
							case 0x343: SETCSR( mtval, writeval ); break;
							case 0x180: SETCSR( satp, writeval ); break;
							//case 0x3a0: break; //pmpcfg0
							//case 0x3B0: break; //pmpaddr0
							//case 0xf11: break; //mvendorid
							//case 0xf12: break; //marchid
							//case 0xf13: break; //mimpid
							//case 0xf14: break; //mhartid
							//case 0x301: break; //misa

							//CSRs 0x136-0x139 used by keyboard to write data to
							default:
								MINIRV32_OTHERCSR_WRITE( csrno, writeval );
								break;
						}
					}
					else if( microop == 0b000 ) // "SYSTEM" 
					{
						rdid = 0;
						
						// 0x105 is stvec (Supervisor trap handler base address)
						if( csrno == 0x105 ) //WFI (Wait for interrupts)
						{
							CSR( mstatus ) |= 8;    //Enable interrupts
							CSR( extraflags ) |= 4; //Inform environment we want to go to sleep. (remember bit 2 is for WFI)
							SETCSR( pc, pc + 4 );

							// sleep
							return 1;
						}

						else if( ( ( csrno & 0xff ) == 0x02 ) )  // MRET (return from traps, exceptions, interrupts)
						{
							//https://raw.githubusercontent.com/riscv/virtual-memory/main/specs/663-Svpbmt.pdf
							//Table 7.6. MRET then in mstatus/mstatush sets MPV (Machine Previous Virtualization Mode)=0, 
							// MPP(previous privilege mode)=0, MIE(machine interrupt enable)=MPIE, and MPIE(machine previous interrupt enable)=1.
							// Should also update extraflags to reflect correct mode (privelege mode -> MPP)
							uint32_t startmstatus = CSR( mstatus );
							uint32_t startextraflags = CSR( extraflags );
							SETCSR( mstatus , (( startmstatus & 0x80) >> 4) | ((startextraflags&3) << 11) | 0x80 );
							SETCSR( extraflags, (startextraflags & ~3) | ((startmstatus >> 11) & 3) );
							pc = CSR( mepc ) -4;
						}
						else
						{
							// Traps are software interrupts so we need to enable interrupts
							// We can use ecall for interupts and exceptions as well to enable interrupts. 
							// https://mullerlee.cyou/2020/07/09/riscv-exception-interrupt/#:~:text=We%20can%20enable%20interrupts%20in%20every%20mode%20by,and%20waiting%20the%20interrupt%20handler%20to%20reset%20it.
							switch( csrno )
							{
								case 0: trap = trap_exit_code(ENV_CALL, CSR( extraflags ) & 3); break; 	// ECALL (trap to debugger);
								case 1:	trap = trap_exit_code(BREAKPOINT, 0); break;  					// EBREAK 3 = "Trap to OS"
								default: trap = trap_exit_code(ILLEGAL_INSTR, 0); break; 				// Illegal opcode.
							}
						}
					}
					else
						trap = trap_exit_code(ILLEGAL_INSTR, 0); 	// Illegal instruction -> Note micrrop 0b100 == undefined.
					break;
				}
				case 0b0101111: // RV32A -> https://msyksphinz-self.github.io/riscv-isadoc/html/rva.html
				{
					uint32_t rs1 = REG((ir >> 15) & 0x1f);
					uint32_t rs2 = REG((ir >> 20) & 0x1f);
					uint32_t irmid = ( ir>>27 ) & 0x1f;

					rs1 -= MINIRV32_RAM_IMAGE_OFFSET;

					// We don't implement load/store from UART or CLNT with RV32A here.

					if( rs1 >= MINI_RV32_RAM_SIZE-3 )
					{
						trap = trap_exit_code(STORE, ACCESS); //Store/AMO access fault
						rval = rs1 + MINIRV32_RAM_IMAGE_OFFSET;
					}
					else
					{
						// struct MiniRV32IMAState * state, uint32_t ir, uint32_t vpn_addr, uint32_t access_type, uint8_t * image, uint32_t * trap
						uint64_t paddr = mmu_translate(state, rs1, STORE, image, &trap);

						if (trap != 0){
							rval = rs1;
							break;
						}

						rval = MINIRV32_LOAD4( paddr );

						// Referenced a little bit of https://github.com/franzflasch/riscv_em/blob/master/src/core/core.c
						uint32_t dowrite = 1;
						switch( irmid )
						{
							case 0b00010: 														//LR.W
								dowrite = 0;
								CSR( extraflags ) = (CSR( extraflags ) & 0b111) | (rs1<<3);
								break;
							case 0b00011:  														//SC.W (Make sure we have a slot, and, it's valid)
								// We check the lower 29 bits of rs1 and see if it is equal 
								// to rs1 (the mem addr we want to write into), If it is then
								// it is valid and we can write into it.
								rval = ( CSR( extraflags ) >> 3 != ( rs1 & 0x1fffffff ) ); 		// Validate that our reservation slot is OK.
								dowrite = !rval; 												// Only write if slot is valid.
								break;
							case 0b00001: break; 												//AMOSWAP.W
							case 0b00000: rs2 += rval; break; 									//AMOADD.W
							case 0b00100: rs2 ^= rval; break; 									//AMOXOR.W
							case 0b01100: rs2 &= rval; break; 									//AMOAND.W
							case 0b01000: rs2 |= rval; break; 									//AMOOR.W
							case 0b10000: rs2 = ((int32_t)rs2<(int32_t)rval)?rs2:rval; break; 	//AMOMIN.W
							case 0b10100: rs2 = ((int32_t)rs2>(int32_t)rval)?rs2:rval; break; 	//AMOMAX.W
							case 0b11000: rs2 = (rs2<rval)?rs2:rval; break; 					//AMOMINU.W
							case 0b11100: rs2 = (rs2>rval)?rs2:rval; break; 					//AMOMAXU.W
							default: trap = trap_exit_code(ILLEGAL_INSTR, 0); dowrite = 0; break; 							//Illegal instruction
						}
						if( dowrite ) MINIRV32_STORE4( rs1, rs2 );
					}
					break;
				}
				default: trap = trap_exit_code(ILLEGAL_INSTR, 0); // Illegal instruction
			}

			// If there was a trap, do NOT allow register writeback.
			if( trap )
				break;

			if( rdid )
			{
				REGSET( rdid, rval ); // Write back register.
			}
		}
		
		// fail_on_all_faults and don't handle traps and interrupts
		MINIRV32_POSTEXEC( pc, ir, trap );

		pc += 4;
	}

	// Handle traps and interrupts.
	if( trap )
	{	
		// If prefixed with 1 in MSB, it's an interrupt, not a trap.
		if( trap & 0x80000000 ) 
		{
			SETCSR( mcause, trap );
			SETCSR( mtval, 0 ); // we set it to 0 as mtval is only relevant for traps
			pc += 4; // PC needs to point to where the PC will return to.
		}
		// trap
		else
		{
			// The mcause register is used to identify the cause of the interrupt or exception, 
			// and mtval is used to store additional information about the interrupt, such as a 
			// faulting address or an error code. 
			SETCSR( mcause,  trap - 1 );

			// If trap is in the range 6 to 8, which correspond to Load access fault, Store/AMO address misaligned,
			// Store/AMO access fault, then mtval is set to rval. rval is the trap exception code.
			// Find more here: https://mi-v-ecosystem.github.io/SoftConsole-Documentation/SoftConsole-v2021.3/troubleshooting/riscv_trap.html

			// For all other traps, mtval is set to pc, which is the current program counter. This is because
			//  the cause of the trap is not related to a specific memory access, so there is no relevant memory
			//   address to store in mtval.
			SETCSR( mtval, (trap > 5 && trap <= 8)? rval : pc );
		}

		SETCSR( mepc, pc ); //TRICKY: The kernel advances mepc automatically.
		
		//CSR( mstatus ) & 8 = MIE, & 0x80 = MPIE
		// On an interrupt, the system moves current MIE into MPIE
		SETCSR( mstatus, (( CSR( mstatus ) & 0x08) << 4) | (( CSR( extraflags ) & 3 ) << 11) );
		pc = (CSR( mtvec ) - 4);

		// If trapping, always enter machine mode.
		CSR( extraflags ) |= 3;

		trap = 0;
		pc += 4;
	}

	if( CSR( cyclel ) > cycle ) CSR( cycleh )++;
	SETCSR( cyclel, cycle );
	SETCSR( pc, pc );
	return 0;
}

#endif

#endif


