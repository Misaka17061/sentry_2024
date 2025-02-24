#ifndef BSP_DEF_H
#define BSP_DEF_H

/* 包含头文件 ----------------------------------------------------------------*/

/* 类型定义 ------------------------------------------------------------------*/
typedef enum
{
    BSP_OK       = 0U,
    BSP_ERROR    = 1U,
} BSP_Status_e;

typedef enum
{
    BSP_DISABLE = 0U,
    BSP_ENABLE = 1U
} BSP_FunctionalStatus_e;

/* 宏定义 --------------------------------------------------------------------*/
#define MASTER_INT_STATE_GET() __get_PRIMASK()
#define MASTER_INT_ENABLE()    do{__enable_irq();  }while(0)
#define MASTER_INT_DISABLE()   do{__disable_irq(); }while(0)
#define MASTER_INT_RESTORE(x)  do{__set_PRIMASK(x);}while(0)

#define CRITICAL_SETCION_ENTER()                      \
    do                                                \
    {                                                 \
    	uint32_t cpu_state = MASTER_INT_STATE_GET(); \
        MASTER_INT_DISABLE();

#define CRITICAL_SETCION_EXIT()        \
        MASTER_INT_RESTORE(cpu_state); \
    }                                  \
    while (0)

#define var_cpu_sr() register unsigned long cpu_sr

#define enter_critical()      \
  do                          \
  {                           \
    cpu_sr = __get_PRIMASK(); \
    __disable_irq();          \
  } while (0)

#define exit_critical()    \
  do                       \
  {                        \
    __set_PRIMASK(cpu_sr); \
  } while (0)

/* 扩展变量 ------------------------------------------------------------------*/

/* 函数声明 ------------------------------------------------------------------*/


#endif /* BSP_DEF_H */
