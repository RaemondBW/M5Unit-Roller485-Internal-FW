/**
 * Minimal _sbrk implementation backing newlib's malloc/heap on bare metal.
 * Restored to enable the GCC/CMake build (the Keil project did not need it).
 */
#include <errno.h>
#include <stdint.h>
#include <stddef.h>

extern uint8_t _end;     /* Symbol defined in the linker script */
extern uint8_t _estack;  /* Symbol defined in the linker script */
extern uint32_t _Min_Stack_Size; /* Symbol defined in the linker script */

void *_sbrk(ptrdiff_t incr)
{
  static uint8_t *__sbrk_heap_end = NULL;

  const uint8_t *max_heap = (uint8_t *)((uint32_t)&_estack - (uint32_t)&_Min_Stack_Size);
  uint8_t *prev_heap_end;

  if (__sbrk_heap_end == NULL) {
    __sbrk_heap_end = &_end;
  }

  if (__sbrk_heap_end + incr > max_heap) {
    errno = ENOMEM;
    return (void *)-1;
  }

  prev_heap_end = __sbrk_heap_end;
  __sbrk_heap_end += incr;

  return (void *)prev_heap_end;
}
