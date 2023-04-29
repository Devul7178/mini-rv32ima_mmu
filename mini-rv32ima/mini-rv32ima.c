// Copyright 2022 Charles Lohr, you may use this file or any portions herein under any of the BSD, MIT, or CC0 licenses.

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "default64mbdtc2.h"

// Just default RAM amount is 64MB.
uint32_t ram_amt = 64*1024*1024;
int fail_on_all_faults = 0;

static int64_t SimpleReadNumberInt( const char * number, int64_t defaultNumber );
static uint64_t GetTimeMicroseconds();
static void ResetKeyboardInput();
static void CaptureKeyboardInput();
static uint32_t HandleException( uint32_t ir, uint32_t retval );
static uint32_t HandleControlStore( uint32_t addy, uint32_t val );
static uint32_t HandleControlLoad( uint32_t addy );
static void HandleOtherCSRWrite( uint8_t * image, uint16_t csrno, uint32_t value );
static int32_t HandleOtherCSRRead( uint8_t * image, uint16_t csrno );
static void MiniSleep();
static int IsKBHit();
static int ReadKBByte();

// This is the functionality we want to override in the emulator.
//  think of this as the way the emulator's processor is connected to the outside world.

// Use printf to output warning messages.
#define MINIRV32WARN( x... ) printf( x );

// Make all functions defined using this macro static.
#define MINIRV32_DECORATE  static

// Define the amount of RAM to use.
#define MINI_RV32_RAM_SIZE ram_amt

// Define that we want to implement the miniRV32 ISA.
#define MINIRV32_IMPLEMENTATION

// Define a post-execution routine that handles exceptions and checks for faults.
#define MINIRV32_POSTEXEC( pc, ir, retval ) { if( retval > 0 ) { if( fail_on_all_faults ) { printf( "FAULT\n" ); return 3; } else retval = HandleException( ir, retval ); } }

// Define a macro that calls the function that handles store operations to UART MMIO port. 
#define MINIRV32_HANDLE_MEM_STORE_CONTROL( addy, val ) if( HandleControlStore( addy, val ) ) return val;

// Define a macro that calls the function that handles load operations to UART MMIO port. 
#define MINIRV32_HANDLE_MEM_LOAD_CONTROL( addy, rval ) rval = HandleControlLoad( addy );

// Define a macro that calls the function that handles writes to custom control and status registers.
#define MINIRV32_OTHERCSR_WRITE( csrno, value ) HandleOtherCSRWrite( image, csrno, value );

// Define a macro that calls the function that handles reads from custom control and status registers.
#define MINIRV32_OTHERCSR_READ( csrno, value ) value = HandleOtherCSRRead( image, csrno );

#include "mini-rv32ima.h"

// The ram image is an image of the entire simulated memory space, including program memory. 
uint8_t * ram_image = 0;
struct MiniRV32IMAState * core;
const char * kernel_command_line = 0;

static void DumpState( struct MiniRV32IMAState * core, uint8_t * ram_image );

// argc is number of arguments passed to program (including program name)
// argv is array of strings that holds the actual args passed to prog. argv[0] is name of prog.
int main( int argc, char ** argv )
{
	int i;								// loop counter
	long long instct = -1;				// number of instructions that will be executed (-1 -> halt)
	int show_help = 0;					// flag set if user need help. Will print message and exit
	int time_divisor = 1;				// time_divisor = (computer clock rate) / (emulator clock rate) (higher = slower)
	int fixed_update = 0;				// indicates whether the emulator should run at a constant number of cycles per second.
	int do_sleep = 1;					// Should the program sleep between updates to display (can help prevent excessive CPU usage)
	int single_step = 0;				// if set, causes the prog to execute 1 instr. and wait for user to press a key before continuing. 
	int dtb_ptr = 0;					// keeps track of the location in memory where Device Tree Blob (DTB) will be loaded.
	const char * image_file_name = 0;	// name of file that contains MiniRV32IMA image
	const char * dtb_file_name = 0;		// name of file containing DTB
	const char * firmware_file_name = 0;		// name of file containing DTB
	for( i = 1; i < argc; i++ )
	{
		const char * param = argv[i];
		int param_continue = 0; // Can combine parameters, like -lpt (-1, -p, -t -> -1pt)
		
		// process command-line arguments
		do
		{
			if( param[0] == '-' || param_continue )
			{
				switch( param[1] )
				{
				// sets the amount of RAM available to the emulator. The next argument should be the size of the RAM in bytes.
				case 'm': if( ++i < argc ) ram_amt = SimpleReadNumberInt( argv[i], ram_amt ); break;

				// sets the number of instructions to execute before stopping. The next argument should be a number.
				case 'c': if( ++i < argc ) instct = SimpleReadNumberInt( argv[i], -1 ); break;

				// sets the kernel command line. The next argument should be a string.
				case 'k': if( ++i < argc ) kernel_command_line = argv[i]; break;

				//  sets the filename of the image to load into memory. The next argument should be a string.
				case 'f': image_file_name = (++i<argc)?argv[i]:0; break;

				//  sets the filename of the image to load into memory. The next argument should be a string.
				case 'x': firmware_file_name = (++i<argc)?argv[i]:0; break;

				// sets the filename of the DTB to load. The next argument should be a string.
				case 'b': dtb_file_name = (++i<argc)?argv[i]:0; break;

				// Updates to the display should happen at fixed intervals
				case 'l': param_continue = 1; fixed_update = 1; break;

				// Don't sleep between intervals
				case 'p': param_continue = 1; do_sleep = 0; break;

				// sets the emulator to single-step mode		
				case 's': param_continue = 1; single_step = 1; break;

				// sets the emulator to halt on all exceptions 
				case 'd': param_continue = 1; fail_on_all_faults = 1; break; 

				// sets the time divisor, which affects the speed of execution. The next argument should be a number.
				case 't': if( ++i < argc ) time_divisor = SimpleReadNumberInt( argv[i], 1 ); break;

				// If the second character of the argument is not recognized, or if the argument does not start with a hyphen,
				// show_help is set to 1 to indicate that the program should display usage information.
				default:
					if( param_continue )
						param_continue = 0;
					else
						show_help = 1;
					break;
				}
			}
			else
			{
				show_help = 1;
				break;
			}
			param++;
		} while( param_continue );
	}

	// help message
	if( show_help || image_file_name == 0 || time_divisor <= 0 )
	{
		fprintf( stderr, "./mini-rv32imaf [parameters]\n\t-m [ram amount]\n\t-f [running image]\n\t-k [kernel command line]\n\t-b [dtb file, or 'disable']\n\t-c instruction count\n\t-s single step with full processor state\n\t-t time divion base\n\t-l lock time base to instruction count\n\t-p disable sleep when wfi\n\t-d fail out immediately on all faults\n" );
		return 1;
	}

	// allocate memory for ram
	ram_image = malloc( ram_amt );
	if( !ram_image )
	{
		fprintf( stderr, "Error: could not allocate system image.\n" );
		return -4;
	}

// The restart label indicates that the emulator can restart if ret equals 0x7777, which is a special code for restarting the emulator).
restart:
	{
		// Open the image and firmware files in binary mode
		FILE *image_file = fopen(image_file_name, "rb");
		FILE *firmware_file = fopen(firmware_file_name, "rb");

		// Check if the files were opened successfully
		if(!image_file || ferror(image_file) || !firmware_file || ferror(firmware_file))
		{
			fprintf(stderr, "Error: \"%s\" or \"%s\" not found\n", image_file_name, firmware_file_name);
			return -5;
		}

		// Get the length of the files and check if they fit in RAM
		fseek(firmware_file, 0, SEEK_END);
		long firmware_len = ftell(firmware_file);
		fseek(firmware_file, 0, SEEK_SET);
		if(firmware_len > ram_amt)
		{
			fprintf(stderr, "Error: Could not fit firmware image (%ld bytes) into %d\n", firmware_len, ram_amt);
			return -6;
		}

		fseek(image_file, 0, SEEK_END);
		long image_len = ftell(image_file);
		fseek(image_file, 0, SEEK_SET);
		if(image_len + firmware_len > ram_amt)
		{
			fprintf(stderr, "Error: Could not fit RAM image (%ld bytes) into %d\n", image_len + firmware_len, ram_amt);
			return -6;
		}

		// Load the firmware into the beginning of the RAM image array
		memset(ram_image, 0, ram_amt);
		if(fread(ram_image, firmware_len, 1, firmware_file) != 1)
		{
			fprintf(stderr, "Error: Could not load firmware image.\n");
			return -7;
		}

		// Load the image after the firmware in the RAM image array
		if(fread(ram_image + firmware_len, image_len, 1, image_file) != 1)
		{
			fprintf(stderr, "Error: Could not load image.\n");
			return -7;
		}

		// Close the files
		fclose(image_file);
		fclose(firmware_file);


		// Checks if a Device Tree Blob (DTB) file name was passed as an argument to the program, 
		// if it was it attempts to open the file and read its contents into a specified location in memory.
		if( dtb_file_name )
		{
			if( strcmp( dtb_file_name, "disable" ) == 0 )
			{
				// No DTB reading.
			}
			else
			{
				FILE * f = fopen( dtb_file_name, "rb" );
				if( !f || ferror( f ) )
				{
					fprintf( stderr, "Error: \"%s\" not found\n", dtb_file_name );
					return -5;
				}
				fseek( f, 0, SEEK_END );
				long dtblen = ftell( f );
				fseek( f, 0, SEEK_SET );

				// location of DTB is at the end of memory (right before the core)
				dtb_ptr = ram_amt - dtblen - sizeof( struct MiniRV32IMAState );
				// 1 object read -> DTB
				if( fread( ram_image + dtb_ptr, dtblen, 1, f ) != 1 )
				{
					fprintf( stderr, "Error: Could not open dtb \"%s\"\n", dtb_file_name );
					return -9;
				}
				fclose( f );
			}
		}
		else
		{
			// Load a default dtb.
			int a = sizeof(default64mbdtb);
			int b = sizeof( struct MiniRV32IMAState );
			dtb_ptr = ram_amt - a - b;
			memcpy( ram_image + dtb_ptr, default64mbdtb, sizeof( default64mbdtb ) );
			if( kernel_command_line )
			{
				strncpy( (char*)( ram_image + dtb_ptr + 0xc0 ), kernel_command_line, 54 );
			}
		}
	}

	CaptureKeyboardInput();

	// The core lives at the end of RAM.
	core = (struct MiniRV32IMAState *)(ram_image + ram_amt - sizeof( struct MiniRV32IMAState ));

	// MiniRV32IMA is loaded into memory of my pc at MINIRV32_RAM_IMAGE_OFFSET. Start executing instructions from there. 
	core->pc = MINIRV32_RAM_IMAGE_OFFSET;

	// r10 and r11 are used to pass arguments
	core->regs[5] = 0x80000000; 
	core->regs[10] = 0x00; //hart ID (hardware thread id)
	core->regs[11] = dtb_ptr?(dtb_ptr+MINIRV32_RAM_IMAGE_OFFSET):0; //dtb_pa (Must be valid pointer) (Should be pointer to dtb)
	core->regs[12] = 0x1028;
	core->pmpaddr0 = -1;
	core->extraflags |= 3; // Machine-mode.

	// default ram is being used. 
	if( dtb_file_name == 0 )
	{
		// Update system ram size in DTB (but if and only if we're using the default DTB)
		// Warning - this will need to be updated if the skeleton DTB is ever modified.
		uint32_t * dtb = (uint32_t*)(ram_image + dtb_ptr);

		// updates only if the 52nd word is 0x00c0ff03
		if( dtb[0x130/4] == 0x00c0ff03 )
		{
			uint32_t validram = dtb_ptr;

			int validram_addr = (validram>>24) | ((( validram >> 16 ) & 0xff) << 8 ) | (((validram>>8) & 0xff ) << 16 ) | ( ( validram & 0xff) << 24 );

			// The value of validram is the starting address of the DTB in RAM. 
			// The code is converting this address into the format expected by the ndianness may be different)
			dtb[0x130/4] = validram_addr;
		}
	}

	// Image is loaded.
	uint64_t rt;

	// last time represents the time at which last instruction was executed
	// Because the emulator runs slower than my PC (and we want to emulate this)
	// we divide how much time has passed in the emulator world with 20 to get real world time. 
	uint64_t lastTime = (fixed_update)?0:(GetTimeMicroseconds()/time_divisor); 
	int instrs_per_flip = single_step?1:1024;

	// Executes a loop that runs instructions on a MiniRV32 processor core
	// runs either a fixed number of instructions or indefinitely
	// instct+1 || instct < 0
	for( rt = 0; rt < instct+1 || instct < 0; rt += instrs_per_flip )
	{
		// elapsedUs represents the amount of time (in microseconds) in the real world that has elapsed 
		// since the last iteration of the loop. 
		// To get elapsedUs we divide the current cycle count by 20 (because in the real world there would 
		// be 20 cycles executed for every cycle executed in the emulator world). We then subtract the time the last
		// instruction executed (last time). This gives us the time that elapsed in the real world. 
		uint64_t * this_ccount = ((uint64_t*)&core->cyclel);
		uint32_t elapsedUs = 0;
		if( fixed_update )
			elapsedUs = *this_ccount / time_divisor - lastTime;
		else
			elapsedUs = GetTimeMicroseconds()/time_divisor - lastTime;
		lastTime += elapsedUs;

		if( single_step) {
			printf("rt: %ld\n", rt);
			DumpState( core, ram_image);
		}

		// execute instructions
		int ret = MiniRV32IMAStep( core, ram_image, 0, elapsedUs, instrs_per_flip, rt ); // Execute upto 1024 cycles before breaking out.
		switch( ret )
		{	
			// continue executing instructions
			case 0: break;

			// sleeps for a short time before continuing
			case 1: if( do_sleep ) MiniSleep(); *this_ccount += instrs_per_flip; break;

			// exit immediately
			case 3: instct = 0; break;

			// syscon code for restart
			case 0x7777: goto restart;	

			// syscon code for power-off
			case 0x5555: printf( "POWEROFF@0x%08x%08x\n", core->cycleh, core->cyclel ); return 0; 
			default: printf( "Unknown failure\n" ); break;
		}
	}

	DumpState( core, ram_image);
}


//////////////////////////////////////////////////////////////////////////
// Platform-specific functionality
//////////////////////////////////////////////////////////////////////////


#if defined(WINDOWS) || defined(WIN32) || defined(_WIN32)

#include <windows.h>
#include <conio.h>

#define strtoll _strtoi64

static void CaptureKeyboardInput()
{
	system(""); // Poorly documented tick: Enable VT100 Windows mode.
}

static void ResetKeyboardInput()
{
}

static void MiniSleep()
{
	Sleep(1);
}

//  returns the current time in microseconds
static uint64_t GetTimeMicroseconds()
{
	static LARGE_INTEGER lpf;
	LARGE_INTEGER li;

	if( !lpf.QuadPart )
		QueryPerformanceFrequency( &lpf );

	QueryPerformanceCounter( &li );
	return ((uint64_t)li.QuadPart * 1000000LL) / (uint64_t)lpf.QuadPart;
}


static int IsKBHit()
{
	return _kbhit();
}

// This code reads a single character of input from the keyboard and returns its ASCII code.
static int ReadKBByte()
{
	// This code is kind of tricky, but used to convert windows arrow keys
	// to VT100 arrow keys.
	static int is_escape_sequence = 0;
	int r;
	if( is_escape_sequence == 1 )
	{
		is_escape_sequence++;
		return '[';
	}

	r = _getch();

	if( is_escape_sequence )
	{
		is_escape_sequence = 0;
		switch( r )
		{
			case 'H': return 'A'; // Up
			case 'P': return 'B'; // Down
			case 'K': return 'D'; // Left
			case 'M': return 'C'; // Right
			case 'G': return 'H'; // Home
			case 'O': return 'F'; // End
			default: return r; // Unknown code.
		}
	}
	else
	{
		switch( r )
		{
			case 13: return 10; //cr->lf
			case 224: is_escape_sequence = 1; return 27; // Escape arrow keys
			default: return r;
		}
	}
}

#else

#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <signal.h>
#include <sys/time.h>

static void CtrlC()
{
	DumpState( core, ram_image);
	exit( 0 );
}

// Override keyboard, so we can capture all keyboard input for the VM.
static void CaptureKeyboardInput()
{
	// Hook exit, because we want to re-enable keyboard.
	atexit(ResetKeyboardInput);
	signal(SIGINT, CtrlC);

	struct termios term;
	tcgetattr(0, &term);
	term.c_lflag &= ~(ICANON | ECHO); // Disable echo as well
	tcsetattr(0, TCSANOW, &term);
}

static void ResetKeyboardInput()
{
	// Re-enable echo, etc. on keyboard.
	struct termios term;
	tcgetattr(0, &term);
	term.c_lflag |= ICANON | ECHO;
	tcsetattr(0, TCSANOW, &term);
}

static void MiniSleep()
{
	usleep(500);
}

static uint64_t GetTimeMicroseconds()
{
	struct timeval tv;
	gettimeofday( &tv, 0 );
	return tv.tv_usec + ((uint64_t)(tv.tv_sec)) * 1000000LL;
}

static int is_eofd;

static int ReadKBByte()
{
	if( is_eofd ) return 0xffffffff;
	char rxchar = 0;
	int rread = read(fileno(stdin), (char*)&rxchar, 1);

	if( rread > 0 ) // Tricky: getchar can't be used with arrow keys.
		return rxchar;
	else
		return -1;
}

// This function checks if there is any input waiting in the standard input buffer. It returns 1 if there is
// input waiting to be read, and 0 otherwise.
static int IsKBHit()
{
	if( is_eofd ) return -1;
	int byteswaiting;
	// For example, in the function call ioctl(0, FIONREAD, &byteswaiting), the flag FIONREAD is
	// used to determine how many bytes are waiting to be read from the input buffer of file descriptor 0 (stdin)
	ioctl(0, FIONREAD, &byteswaiting);

	//If there are no bytes waiting and calling write() with the fileno(stdin) file 
	// descriptor and a null buffer does not return 0, it means that the end-of-file has
	// been reached and sets the is_eofd flag to 1 and returns -1.
	if( !byteswaiting && write( fileno(stdin), 0, 0 ) != 0 ) { is_eofd = 1; return -1; } // Is end-of-file for 
	
	// returns true if bytes waiting is not 0.
	return !!byteswaiting;
}


#endif


//////////////////////////////////////////////////////////////////////////
// Rest of functions functionality
//////////////////////////////////////////////////////////////////////////

static uint32_t HandleException( uint32_t ir, uint32_t code )
{
	// Weird opcode emitted by duktape on exit.
	if( code == 3 )
	{
		// Could handle other opcodes here.
	}
	return code;
}

/************************************************************************************************************/
/**
 * How does UART emulation work in the riscv32IMA emulator:
 * 1) The UART in the RISC-V32IMA system is emulated using two memory-mapped registers: 
 * the status register and the data buffer register.
 * 2) The status register indicates if there is any data waiting in the receive buffer, 
 * while the data buffer register holds the received data or the data to be transmitted.
 * 3) The HandleControlStore() function is responsible for outputting data
 * to the host system's console, and the HandleControlLoad() function is responsible for checking 
 * if there is data waiting in the UART's buffer and returning that data to the emulated system
 * 4) The IsKBHit() function is used to check if there is any data waiting in the standard input, 
 * and the ReadKBByte() function is used to read a byte from the standard input.
 * 5) When a user types a character on the keyboard, an interrupt is generated in 
 * the host's operating system, which sends the keyboard data to the emulator through the standard input.
 * 6) If there is data waiting in the standard input, HandleControlLoad() is called with address 0x10000005,
 * which checks if there is any data waiting in the receive buffer.
 * 7) If the lowest bit of the value returned by HandleControlLoad() is set, the emulator can read the data 
 * from the UART data buffer register by calling HandleControlLoad() with address 0x10000000.
 * 8) The emulator can use HandleControlStore() to send data to the UART data buffer register, which outputs 
 * the data to the standard output using printf() and fflush().
 **/
/************************************************************************************************************/

// handling the control store of the emulator.
// control store refers to a part of the processor that contains  
// firmware that controls the operation of the processor.
static uint32_t HandleControlStore( uint32_t addy, uint32_t val )
{
	// If the address matches, it prints the ASCII character value of val to the 
	// console using printf(), and then flushes the output stream using fflush() to
	//  ensure the output is immediately visible to the user.
	if( addy == 0x10000000 ) //UART 8250 / 16550 Data Buffer
	{
		printf( "%c", val );
		fflush( stdout );
	}
	return 0;
}


static uint32_t HandleControlLoad( uint32_t addy )
{
	// Emulating a 8250 / 16550 UART
	// 0x10000005 is the status register for UART
	// If there is data waiting, the HandleControlLoad() function returns a 
	// value with the eight bit set to 1 (i.e. 0x60). If there is no data waiting,
	//  the HandleControlLoad() function returns a value with the eight bit set to 0.
	if( addy == 0x10000005 )
		return 0x60 | IsKBHit();
	else if( addy == 0x10000000 && IsKBHit() )
		return ReadKBByte();
	return 0;
}

static void HandleOtherCSRWrite( uint8_t * image, uint16_t csrno, uint32_t value )
{	
	// csrno 0x136, 0x137, 0x138, 0x139, 0x140 are custom CSRs used for supervisor standard read/write
	
	// decimal integer
	if( csrno == 0x136 )
	{
		printf( "%d", value ); fflush( stdout );
	}

	// 8-digit hexadecimal number with leading zeros
	if( csrno == 0x137 )
	{
		printf( "%08x", value ); fflush( stdout );
	}

	// null terminated string in image
	else if( csrno == 0x138 )
	{
		//Print "string"
		uint32_t ptrstart = value - MINIRV32_RAM_IMAGE_OFFSET;
		uint32_t ptrend = ptrstart;
		if( ptrstart >= ram_amt )
			printf( "DEBUG PASSED INVALID PTR (%08x)\n", value );
		while( ptrend < ram_amt )
		{
			if( image[ptrend] == 0 ) break;
			ptrend++;
		}
		if( ptrend != ptrstart )
			fwrite( image + ptrstart, ptrend - ptrstart, 1, stdout );
	}

	// char
	else if( csrno == 0x139 )
	{
		putchar( value ); fflush( stdout );
	}
}

static int32_t HandleOtherCSRRead( uint8_t * image, uint16_t csrno )
{
	// returns keyboard input waiting to be read
	if( csrno == 0x140 )
	{
		if( !IsKBHit() ) return -1;
		return ReadKBByte();
	}
	return 0;
}

// The function attempts to convert the input string number to an integer using the specified base 
// (either 10, 16, 2, or 8). If the conversion is successful, the function returns the integer value.
//  If the conversion fails, the function returns the defaultNumber value.
static int64_t SimpleReadNumberInt( const char * number, int64_t defaultNumber )
{
	if( !number || !number[0] ) return defaultNumber;
	int radix = 10;
	if( number[0] == '0' )
	{
		char nc = number[1];
		number+=2;
		if( nc == 0 ) return 0;
		else if( nc == 'x' ) radix = 16;
		else if( nc == 'b' ) radix = 2;
		else { number--; radix = 8; }
	}
	char * endptr;
	uint64_t ret = strtoll( number, &endptr, radix );
	if( endptr == number )
	{
		return defaultNumber;
	}
	else
	{
		return ret;
	}
}

// This function is used to print out the current state of the CPU core and the contents of the registers. Used for debugging.
static void DumpState( struct MiniRV32IMAState * core, uint8_t * ram_image )
{
	uint32_t pc = core->pc;

	// memory address of the current instruction in the RAM image. 
	uint32_t pc_offset = pc - MINIRV32_RAM_IMAGE_OFFSET;
	uint32_t ir = 0;

	// printf( "PC: %08x ", pc );

	// if pc offset is within the range of ram_image, ir is set to to the instruction at the pc_offset.
	// Otherwise print xxxxxxxxxx.
	// if( pc_offset >= 0 && pc_offset < ram_amt - 3 )
	// {
	// 	ir = *((uint32_t*)(&((uint8_t*)ram_image)[pc_offset]));

	// 	// %08x means hexadecimal format, zero-padded with at least 8 digits and enclosed in square brackets and prefixed with '0x'
	// 	printf( "[0x%08x] ", ir ); 
	// }
	// else
	// 	printf( "[xxxxxxxxxx] " ); 

	/**
	 * prints the values of the CPU registers (32 regs). The registers are:
	 * 1) zero register (Z) - r0
	 * 2) return address register (ra) - r1
	 * 3) stack pointer (sp) - r2
	 * 4) global pointer (gp) - r3
	 * 5) thread pointer (tp) - r4
	 * 6) temporary registers (t0-t6) - r5 to r7 and r28 to r31
	 * 7) saved registers (s0-s11) - r8 to r9 and r18 to r27
	 * 8) function argument registers (a0-a7) - r10 to r17
	 */

	uint32_t * regs = core->regs;
	printf( "ra             0x%x	0x%x\nsp             0x%x	0x%x\ngp             0x%x	0x%x\ntp             0x%x	0x%x\nt0             0x%x	%d\nt1             0x%x	%d\nt2             0x%x	%d\nfp             0x%x	0x%x\ns1             0x%x	%d\na0             0x%x	%d\na1             0x%x	%d\na2             0x%x	%d\na3             0x%x	%d\na4             0x%x	%d\na5             0x%x	%d\n",
		regs[1], regs[1], regs[2], regs[2], regs[3], regs[3], regs[4], regs[4], regs[5], regs[5], regs[6], regs[6], regs[7], regs[7],
		regs[8], regs[8], regs[9], regs[9], regs[10], regs[10], regs[11], regs[11], regs[12], regs[12], regs[13], regs[13], regs[14], regs[14], regs[15], regs[15] );
	printf( "a6             0x%x	%d\na7             0x%x	%d\ns2             0x%x	%d\ns3             0x%x	%d\ns4             0x%x	%d\ns5             0x%x	%d\ns6             0x%x	%d\ns7             0x%x	%d\ns8             0x%x	%d\ns9             0x%x	%d\ns10            0x%x	%d\ns11            0x%x	%d\nt3             0x%x	%d\nt4             0x%x	%d\nt5             0x%x	%d\nt6             0x%x	%d\npc             0x%x	0x%x\n\n",
		regs[16], regs[16], regs[17], regs[17], regs[18], regs[18], regs[19], regs[19], regs[20], regs[20], regs[21], regs[21], regs[22], regs[22], regs[23],  regs[23],
		regs[24], regs[24], regs[25], regs[25], regs[26], regs[26], regs[27], regs[27], regs[28], regs[28], regs[29], regs[29], regs[30], regs[30], regs[31], regs[31], pc, pc );
}

