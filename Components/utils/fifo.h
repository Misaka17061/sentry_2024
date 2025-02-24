#ifndef __FIFO_H__
#define __FIFO_H__

#include "main.h"
#include <string.h>

/* FIFO Memory Model (Single Byte Mode) */
typedef struct
{
    char *p_start_addr; //!< FIFO Memory Pool Start Address
    char *p_end_addr;   //!< FIFO Memory Pool End Address
    int free_num;       //!< The remain capacity of FIFO
    int used_num;       //!< The number of elements in FIFO
    char *p_read_addr;  //!< FIFO Data Read Index Pointer
    char *p_write_addr; //!< FIFO Data Write Index Pointer
} fifo_s_t;

/* FIFO Memory Model */
typedef struct
{
    char *p_start_addr; //!< FIFO Memory Pool Start Address
    char *p_end_addr;   //!< FIFO Memory Pool End Address
    int free_num;       //!< The remain capacity of FIFO
    int used_num;       //!< The number of elements in FIFO
    int unit_size;      //!< FIFO Element Size(Unit: Byte)
    char *p_read_addr;  //!< FIFO Data Read Index Pointer
    char *p_write_addr; //!< FIFO Data Write Index Pointer
} fifo_t;

/* Control functions **********************************************************/
#ifdef USE_DYNAMIC_MEMORY
fifo_s_t *fifo_s_create(int uint_cnt);
void fifo_s_destroy(fifo_s_t *p_fifo);
#endif // USE_DYNAMIC_MEMORY
int fifo_s_init(fifo_s_t *p_fifo, void *p_base_addr, int uint_cnt);
int fifo_s_put(fifo_s_t *p_fifo, char element);
int fifo_s_puts(fifo_s_t *p_fifo, char *p_source, int len);
int fifo_s_puts_noprotect(fifo_s_t *p_fifo, char *p_source, int len);
char fifo_s_get(fifo_s_t *p_fifo);
int fifo_s_gets(fifo_s_t *p_fifo, char *p_dest, int len);
int fifo_s_gets_noprotect(fifo_s_t *p_fifo, char *p_dest, int len);
char fifo_s_preread(fifo_s_t *p_fifo, int offset);
int fifo_s_prereads(fifo_s_t *p_fifo, char *p_dest, int offset, int len);
char fifo_s_isempty(fifo_s_t *p_fifo);
char fifo_s_isfull(fifo_s_t *p_fifo);
int fifo_s_used(fifo_s_t *p_fifo);
int fifo_s_free(fifo_s_t *p_fifo);
void fifo_s_flush(fifo_s_t *p_fifo);
int fifo_s_discard(fifo_s_t *p_fifo, int len);

#ifdef USE_DYNAMIC_MEMORY
fifo_t *fifo_create(char unit_size, int unit_cnt);
void fifo_destory(fifo_t * p_fifo);
#endif // USE_DYNAMIC_MEMORY
int fifo_init(fifo_t * p_fifo, void *p_base_addr, int unit_size, int unit_cnt);
int fifo_put(fifo_t * p_fifo, void *p_element);
int fifo_put_noprotect(fifo_t * p_fifo, void *p_element);
int fifo_get(fifo_t * p_fifo, void *p_element);
int fifo_get_noprotect(fifo_t * p_fifo, void *p_element);
int fifo_pre_read(fifo_t * p_fifo, char offset, void *p_element);
int fifo_is_empty(fifo_t * p_fifo);
int fifo_is_full(fifo_t * p_fifo);
int fifo_used(fifo_t * p_fifo);
int fifo_free(fifo_t * p_fifo);
int fifo_flush(fifo_t * p_fifo);

#endif  // __FIFO_H__

