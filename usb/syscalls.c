#include <sys/stat.h>
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "usbd_cdc_vcp.h"

int __errno;

int _close(__attribute__ ((unused)) int file) {
	return 0;
}

int _fstat(__attribute__ ((unused)) int file, __attribute__((unused)) struct stat *st) {
	return 0;
}

int _isatty(__attribute__((unused)) int file) {
	return 1;
}

int _lseek(__attribute__((unused)) int file, __attribute__((unused)) int ptr, __attribute__((unused)) int dir) {
	return 0;
}

int _open(__attribute__((unused)) const char *name, __attribute__((unused)) int flags, __attribute__((unused)) int mode) {
	return -1;
}

int _read(int file, char *ptr, __attribute__((unused))int len) {
	if (file != 0) {
		return 0;
	}

	// Use USB CDC Port for stdin
	while(!VCP_get_char((uint8_t*)ptr)){};

	// Echo typed characters
	VCP_put_char((uint8_t)*ptr);

	return 1;
}

/* Register name faking - works in collusion with the linker.  */
register char * stack_ptr __asm__ ("sp");

caddr_t _sbrk_r (__attribute__((unused)) struct _reent *r, int incr) {
    extern char   end __asm__ ("end"); /* Defined by the linker.  */
	static char * heap_end;
	char *        prev_heap_end;

	if (heap_end == NULL)
        heap_end = & end;

	prev_heap_end = heap_end;

	if (heap_end + incr > stack_ptr) {
		//errno = ENOMEM;
		return (caddr_t) -1;
	}

	heap_end += incr;

	return (caddr_t) prev_heap_end;
}

int _write(__attribute__((unused)) int file, char *ptr, int len) {
	VCP_send_buffer((uint8_t*)ptr, len);
	return len;
}
