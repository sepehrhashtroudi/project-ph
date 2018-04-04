#line 1 "../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c"

















































































 

 
#line 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal.h"


































  

 







 
#line 1 "../Inc/stm32f4xx_hal_conf.h"
































  

 







#line 1 "../Inc/main.h"















































 

 



 
#line 1 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f4xx.h"











































 



 



 
    






   


 
  


 






 
#line 111 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f4xx.h"
   


 
#line 123 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f4xx.h"



 
#line 135 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f4xx.h"



 



 

#line 1 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"







































 



 



 
    









 



 








 
  


 




 
typedef enum
{
 
  NonMaskableInt_IRQn         = -14,     
  MemoryManagement_IRQn       = -12,     
  BusFault_IRQn               = -11,     
  UsageFault_IRQn             = -10,     
  SVCall_IRQn                 = -5,      
  DebugMonitor_IRQn           = -4,      
  PendSV_IRQn                 = -2,      
  SysTick_IRQn                = -1,      
 
  WWDG_IRQn                   = 0,       
  PVD_IRQn                    = 1,       
  TAMP_STAMP_IRQn             = 2,       
  RTC_WKUP_IRQn               = 3,       
  FLASH_IRQn                  = 4,       
  RCC_IRQn                    = 5,       
  EXTI0_IRQn                  = 6,       
  EXTI1_IRQn                  = 7,       
  EXTI2_IRQn                  = 8,       
  EXTI3_IRQn                  = 9,       
  EXTI4_IRQn                  = 10,      
  DMA1_Stream0_IRQn           = 11,      
  DMA1_Stream1_IRQn           = 12,      
  DMA1_Stream2_IRQn           = 13,      
  DMA1_Stream3_IRQn           = 14,      
  DMA1_Stream4_IRQn           = 15,      
  DMA1_Stream5_IRQn           = 16,      
  DMA1_Stream6_IRQn           = 17,      
  ADC_IRQn                    = 18,      
  CAN1_TX_IRQn                = 19,      
  CAN1_RX0_IRQn               = 20,      
  CAN1_RX1_IRQn               = 21,      
  CAN1_SCE_IRQn               = 22,      
  EXTI9_5_IRQn                = 23,      
  TIM1_BRK_TIM9_IRQn          = 24,      
  TIM1_UP_TIM10_IRQn          = 25,      
  TIM1_TRG_COM_TIM11_IRQn     = 26,      
  TIM1_CC_IRQn                = 27,      
  TIM2_IRQn                   = 28,      
  TIM3_IRQn                   = 29,      
  TIM4_IRQn                   = 30,      
  I2C1_EV_IRQn                = 31,      
  I2C1_ER_IRQn                = 32,      
  I2C2_EV_IRQn                = 33,      
  I2C2_ER_IRQn                = 34,      
  SPI1_IRQn                   = 35,      
  SPI2_IRQn                   = 36,      
  USART1_IRQn                 = 37,      
  USART2_IRQn                 = 38,      
  USART3_IRQn                 = 39,      
  EXTI15_10_IRQn              = 40,      
  RTC_Alarm_IRQn              = 41,      
  OTG_FS_WKUP_IRQn            = 42,      
  TIM8_BRK_TIM12_IRQn         = 43,      
  TIM8_UP_TIM13_IRQn          = 44,      
  TIM8_TRG_COM_TIM14_IRQn     = 45,      
  TIM8_CC_IRQn                = 46,      
  DMA1_Stream7_IRQn           = 47,      
  FSMC_IRQn                   = 48,      
  SDIO_IRQn                   = 49,      
  TIM5_IRQn                   = 50,      
  SPI3_IRQn                   = 51,      
  UART4_IRQn                  = 52,      
  UART5_IRQn                  = 53,      
  TIM6_DAC_IRQn               = 54,      
  TIM7_IRQn                   = 55,      
  DMA2_Stream0_IRQn           = 56,      
  DMA2_Stream1_IRQn           = 57,      
  DMA2_Stream2_IRQn           = 58,      
  DMA2_Stream3_IRQn           = 59,      
  DMA2_Stream4_IRQn           = 60,      
  ETH_IRQn                    = 61,      
  ETH_WKUP_IRQn               = 62,      
  CAN2_TX_IRQn                = 63,      
  CAN2_RX0_IRQn               = 64,      
  CAN2_RX1_IRQn               = 65,      
  CAN2_SCE_IRQn               = 66,      
  OTG_FS_IRQn                 = 67,      
  DMA2_Stream5_IRQn           = 68,      
  DMA2_Stream6_IRQn           = 69,      
  DMA2_Stream7_IRQn           = 70,      
  USART6_IRQn                 = 71,      
  I2C3_EV_IRQn                = 72,      
  I2C3_ER_IRQn                = 73,      
  OTG_HS_EP1_OUT_IRQn         = 74,      
  OTG_HS_EP1_IN_IRQn          = 75,      
  OTG_HS_WKUP_IRQn            = 76,      
  OTG_HS_IRQn                 = 77,      
  DCMI_IRQn                   = 78,      
  RNG_IRQn                    = 80,      
  FPU_IRQn                    = 81       
} IRQn_Type;
 




 

#line 1 "../Drivers/CMSIS/Include/core_cm4.h"
 




 

























 











#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
 
 





 









     
#line 27 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
     











#line 46 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"





 

     

     
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

     
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;

     

     
     
typedef   signed          char int_least8_t;
typedef   signed short     int int_least16_t;
typedef   signed           int int_least32_t;
typedef   signed       __int64 int_least64_t;

     
typedef unsigned          char uint_least8_t;
typedef unsigned short     int uint_least16_t;
typedef unsigned           int uint_least32_t;
typedef unsigned       __int64 uint_least64_t;

     

     
typedef   signed           int int_fast8_t;
typedef   signed           int int_fast16_t;
typedef   signed           int int_fast32_t;
typedef   signed       __int64 int_fast64_t;

     
typedef unsigned           int uint_fast8_t;
typedef unsigned           int uint_fast16_t;
typedef unsigned           int uint_fast32_t;
typedef unsigned       __int64 uint_fast64_t;

     




typedef   signed           int intptr_t;
typedef unsigned           int uintptr_t;


     
typedef   signed     long long intmax_t;
typedef unsigned     long long uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     






     






     






     

     


     


     


     

     
#line 216 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     



     






     
    
 



#line 241 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     







     










     











#line 305 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"






 
#line 45 "../Drivers/CMSIS/Include/core_cm4.h"

















 




 



 

 













#line 120 "../Drivers/CMSIS/Include/core_cm4.h"



 
#line 135 "../Drivers/CMSIS/Include/core_cm4.h"

#line 209 "../Drivers/CMSIS/Include/core_cm4.h"

#line 1 "../Drivers/CMSIS/Include/core_cmInstr.h"
 




 

























 












 



 

 
#line 1 "../Drivers/CMSIS/Include/cmsis_armcc.h"
 




 

























 










 



 

 
 





 
static __inline uint32_t __get_CONTROL(void)
{
  register uint32_t __regControl         __asm("control");
  return(__regControl);
}






 
static __inline void __set_CONTROL(uint32_t control)
{
  register uint32_t __regControl         __asm("control");
  __regControl = control;
}






 
static __inline uint32_t __get_IPSR(void)
{
  register uint32_t __regIPSR          __asm("ipsr");
  return(__regIPSR);
}






 
static __inline uint32_t __get_APSR(void)
{
  register uint32_t __regAPSR          __asm("apsr");
  return(__regAPSR);
}






 
static __inline uint32_t __get_xPSR(void)
{
  register uint32_t __regXPSR          __asm("xpsr");
  return(__regXPSR);
}






 
static __inline uint32_t __get_PSP(void)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  return(__regProcessStackPointer);
}






 
static __inline void __set_PSP(uint32_t topOfProcStack)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  __regProcessStackPointer = topOfProcStack;
}






 
static __inline uint32_t __get_MSP(void)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  return(__regMainStackPointer);
}






 
static __inline void __set_MSP(uint32_t topOfMainStack)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  __regMainStackPointer = topOfMainStack;
}






 
static __inline uint32_t __get_PRIMASK(void)
{
  register uint32_t __regPriMask         __asm("primask");
  return(__regPriMask);
}






 
static __inline void __set_PRIMASK(uint32_t priMask)
{
  register uint32_t __regPriMask         __asm("primask");
  __regPriMask = (priMask);
}








 







 







 
static __inline uint32_t  __get_BASEPRI(void)
{
  register uint32_t __regBasePri         __asm("basepri");
  return(__regBasePri);
}






 
static __inline void __set_BASEPRI(uint32_t basePri)
{
  register uint32_t __regBasePri         __asm("basepri");
  __regBasePri = (basePri & 0xFFU);
}







 
static __inline void __set_BASEPRI_MAX(uint32_t basePri)
{
  register uint32_t __regBasePriMax      __asm("basepri_max");
  __regBasePriMax = (basePri & 0xFFU);
}






 
static __inline uint32_t __get_FAULTMASK(void)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  return(__regFaultMask);
}






 
static __inline void __set_FAULTMASK(uint32_t faultMask)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  __regFaultMask = (faultMask & (uint32_t)1);
}










 
static __inline uint32_t __get_FPSCR(void)
{

  register uint32_t __regfpscr         __asm("fpscr");
  return(__regfpscr);



}






 
static __inline void __set_FPSCR(uint32_t fpscr)
{

  register uint32_t __regfpscr         __asm("fpscr");
  __regfpscr = (fpscr);

}





 


 



 




 






 







 






 








 










 










 











 








 

__attribute__((section(".rev16_text"))) static __inline __asm uint32_t __REV16(uint32_t value)
{
  rev16 r0, r0
  bx lr
}







 

__attribute__((section(".revsh_text"))) static __inline __asm int32_t __REVSH(int32_t value)
{
  revsh r0, r0
  bx lr
}









 









 








 
#line 455 "../Drivers/CMSIS/Include/cmsis_armcc.h"







 










 












 












 














 














 














 










 









 









 









 

__attribute__((section(".rrx_text"))) static __inline __asm uint32_t __RRX(uint32_t value)
{
  rrx r0, r0
  bx lr
}








 








 








 








 








 








 




   


 



 



#line 720 "../Drivers/CMSIS/Include/cmsis_armcc.h"











 


#line 54 "../Drivers/CMSIS/Include/core_cmInstr.h"

 
#line 84 "../Drivers/CMSIS/Include/core_cmInstr.h"

   

#line 211 "../Drivers/CMSIS/Include/core_cm4.h"
#line 1 "../Drivers/CMSIS/Include/core_cmFunc.h"
 




 

























 












 



 

 
#line 54 "../Drivers/CMSIS/Include/core_cmFunc.h"

 
#line 84 "../Drivers/CMSIS/Include/core_cmFunc.h"

 

#line 212 "../Drivers/CMSIS/Include/core_cm4.h"
#line 1 "../Drivers/CMSIS/Include/core_cmSimd.h"
 




 

























 
















 



 

 
#line 58 "../Drivers/CMSIS/Include/core_cmSimd.h"

 
#line 88 "../Drivers/CMSIS/Include/core_cmSimd.h"

 






#line 213 "../Drivers/CMSIS/Include/core_cm4.h"
















 
#line 256 "../Drivers/CMSIS/Include/core_cm4.h"

 






 
#line 272 "../Drivers/CMSIS/Include/core_cm4.h"

 




 













 



 






 



 
typedef union
{
  struct
  {
    uint32_t _reserved0:16;               
    uint32_t GE:4;                        
    uint32_t _reserved1:7;                
    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} APSR_Type;

 





















 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:23;               
  } b;                                    
  uint32_t w;                             
} IPSR_Type;

 






 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:7;                
    uint32_t GE:4;                        
    uint32_t _reserved1:4;                
    uint32_t T:1;                         
    uint32_t IT:2;                        
    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} xPSR_Type;

 






























 
typedef union
{
  struct
  {
    uint32_t nPRIV:1;                     
    uint32_t SPSEL:1;                     
    uint32_t FPCA:1;                      
    uint32_t _reserved0:29;               
  } b;                                    
  uint32_t w;                             
} CONTROL_Type;

 









 







 



 
typedef struct
{
  volatile uint32_t ISER[8U];                
        uint32_t RESERVED0[24U];
  volatile uint32_t ICER[8U];                
        uint32_t RSERVED1[24U];
  volatile uint32_t ISPR[8U];                
        uint32_t RESERVED2[24U];
  volatile uint32_t ICPR[8U];                
        uint32_t RESERVED3[24U];
  volatile uint32_t IABR[8U];                
        uint32_t RESERVED4[56U];
  volatile uint8_t  IP[240U];                
        uint32_t RESERVED5[644U];
  volatile  uint32_t STIR;                    
}  NVIC_Type;

 



 







 



 
typedef struct
{
  volatile const  uint32_t CPUID;                   
  volatile uint32_t ICSR;                    
  volatile uint32_t VTOR;                    
  volatile uint32_t AIRCR;                   
  volatile uint32_t SCR;                     
  volatile uint32_t CCR;                     
  volatile uint8_t  SHP[12U];                
  volatile uint32_t SHCSR;                   
  volatile uint32_t CFSR;                    
  volatile uint32_t HFSR;                    
  volatile uint32_t DFSR;                    
  volatile uint32_t MMFAR;                   
  volatile uint32_t BFAR;                    
  volatile uint32_t AFSR;                    
  volatile const  uint32_t PFR[2U];                 
  volatile const  uint32_t DFR;                     
  volatile const  uint32_t ADR;                     
  volatile const  uint32_t MMFR[4U];                
  volatile const  uint32_t ISAR[5U];                
        uint32_t RESERVED0[5U];
  volatile uint32_t CPACR;                   
} SCB_Type;

 















 






























 



 





















 









 


















 










































 









 









 















 







 



 
typedef struct
{
        uint32_t RESERVED0[1U];
  volatile const  uint32_t ICTR;                    
  volatile uint32_t ACTLR;                   
} SCnSCB_Type;

 



 















 







 



 
typedef struct
{
  volatile uint32_t CTRL;                    
  volatile uint32_t LOAD;                    
  volatile uint32_t VAL;                     
  volatile const  uint32_t CALIB;                   
} SysTick_Type;

 












 



 



 









 







 



 
typedef struct
{
  volatile  union
  {
    volatile  uint8_t    u8;                  
    volatile  uint16_t   u16;                 
    volatile  uint32_t   u32;                 
  }  PORT [32U];                          
        uint32_t RESERVED0[864U];
  volatile uint32_t TER;                     
        uint32_t RESERVED1[15U];
  volatile uint32_t TPR;                     
        uint32_t RESERVED2[15U];
  volatile uint32_t TCR;                     
        uint32_t RESERVED3[29U];
  volatile  uint32_t IWR;                     
  volatile const  uint32_t IRR;                     
  volatile uint32_t IMCR;                    
        uint32_t RESERVED4[43U];
  volatile  uint32_t LAR;                     
  volatile const  uint32_t LSR;                     
        uint32_t RESERVED5[6U];
  volatile const  uint32_t PID4;                    
  volatile const  uint32_t PID5;                    
  volatile const  uint32_t PID6;                    
  volatile const  uint32_t PID7;                    
  volatile const  uint32_t PID0;                    
  volatile const  uint32_t PID1;                    
  volatile const  uint32_t PID2;                    
  volatile const  uint32_t PID3;                    
  volatile const  uint32_t CID0;                    
  volatile const  uint32_t CID1;                    
  volatile const  uint32_t CID2;                    
  volatile const  uint32_t CID3;                    
} ITM_Type;

 



 



























 



 



 



 









   







 



 
typedef struct
{
  volatile uint32_t CTRL;                    
  volatile uint32_t CYCCNT;                  
  volatile uint32_t CPICNT;                  
  volatile uint32_t EXCCNT;                  
  volatile uint32_t SLEEPCNT;                
  volatile uint32_t LSUCNT;                  
  volatile uint32_t FOLDCNT;                 
  volatile const  uint32_t PCSR;                    
  volatile uint32_t COMP0;                   
  volatile uint32_t MASK0;                   
  volatile uint32_t FUNCTION0;               
        uint32_t RESERVED0[1U];
  volatile uint32_t COMP1;                   
  volatile uint32_t MASK1;                   
  volatile uint32_t FUNCTION1;               
        uint32_t RESERVED1[1U];
  volatile uint32_t COMP2;                   
  volatile uint32_t MASK2;                   
  volatile uint32_t FUNCTION2;               
        uint32_t RESERVED2[1U];
  volatile uint32_t COMP3;                   
  volatile uint32_t MASK3;                   
  volatile uint32_t FUNCTION3;               
} DWT_Type;

 






















































 



 



 



 



 



 



 



























   







 



 
typedef struct
{
  volatile uint32_t SSPSR;                   
  volatile uint32_t CSPSR;                   
        uint32_t RESERVED0[2U];
  volatile uint32_t ACPR;                    
        uint32_t RESERVED1[55U];
  volatile uint32_t SPPR;                    
        uint32_t RESERVED2[131U];
  volatile const  uint32_t FFSR;                    
  volatile uint32_t FFCR;                    
  volatile const  uint32_t FSCR;                    
        uint32_t RESERVED3[759U];
  volatile const  uint32_t TRIGGER;                 
  volatile const  uint32_t FIFO0;                   
  volatile const  uint32_t ITATBCTR2;               
        uint32_t RESERVED4[1U];
  volatile const  uint32_t ITATBCTR0;               
  volatile const  uint32_t FIFO1;                   
  volatile uint32_t ITCTRL;                  
        uint32_t RESERVED5[39U];
  volatile uint32_t CLAIMSET;                
  volatile uint32_t CLAIMCLR;                
        uint32_t RESERVED7[8U];
  volatile const  uint32_t DEVID;                   
  volatile const  uint32_t DEVTYPE;                 
} TPI_Type;

 



 



 












 






 



 





















 



 





















 



 



 


















 






   








 



 
typedef struct
{
  volatile const  uint32_t TYPE;                    
  volatile uint32_t CTRL;                    
  volatile uint32_t RNR;                     
  volatile uint32_t RBAR;                    
  volatile uint32_t RASR;                    
  volatile uint32_t RBAR_A1;                 
  volatile uint32_t RASR_A1;                 
  volatile uint32_t RBAR_A2;                 
  volatile uint32_t RASR_A2;                 
  volatile uint32_t RBAR_A3;                 
  volatile uint32_t RASR_A3;                 
} MPU_Type;

 









 









 



 









 






























 









 



 
typedef struct
{
        uint32_t RESERVED0[1U];
  volatile uint32_t FPCCR;                   
  volatile uint32_t FPCAR;                   
  volatile uint32_t FPDSCR;                  
  volatile const  uint32_t MVFR0;                   
  volatile const  uint32_t MVFR1;                   
} FPU_Type;

 



























 



 












 
























 












 








 



 
typedef struct
{
  volatile uint32_t DHCSR;                   
  volatile  uint32_t DCRSR;                   
  volatile uint32_t DCRDR;                   
  volatile uint32_t DEMCR;                   
} CoreDebug_Type;

 




































 






 







































 







 






 







 


 







 

 
#line 1541 "../Drivers/CMSIS/Include/core_cm4.h"

#line 1550 "../Drivers/CMSIS/Include/core_cm4.h"











 










 


 



 





 









 
static __inline void NVIC_SetPriorityGrouping(uint32_t PriorityGroup)
{
  uint32_t reg_value;
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);              

  reg_value  =  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR;                                                    
  reg_value &= ~((uint32_t)((0xFFFFUL << 16U) | (7UL << 8U)));  
  reg_value  =  (reg_value                                   |
                ((uint32_t)0x5FAUL << 16U) |
                (PriorityGroupTmp << 8U)                      );               
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR =  reg_value;
}






 
static __inline uint32_t NVIC_GetPriorityGrouping(void)
{
  return ((uint32_t)((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8U)) >> 8U));
}






 
static __inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[(((uint32_t)(int32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}






 
static __inline void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[(((uint32_t)(int32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}








 
static __inline uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[(((uint32_t)(int32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
}






 
static __inline void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[(((uint32_t)(int32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}






 
static __inline void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICPR[(((uint32_t)(int32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}








 
static __inline uint32_t NVIC_GetActive(IRQn_Type IRQn)
{
  return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IABR[(((uint32_t)(int32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
}








 
static __inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if ((int32_t)(IRQn) < 0)
  {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[(((uint32_t)(int32_t)IRQn) & 0xFUL)-4UL] = (uint8_t)((priority << (8U - 4U)) & (uint32_t)0xFFUL);
  }
  else
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[((uint32_t)(int32_t)IRQn)]               = (uint8_t)((priority << (8U - 4U)) & (uint32_t)0xFFUL);
  }
}










 
static __inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if ((int32_t)(IRQn) < 0)
  {
    return(((uint32_t)((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[(((uint32_t)(int32_t)IRQn) & 0xFUL)-4UL] >> (8U - 4U)));
  }
  else
  {
    return(((uint32_t)((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[((uint32_t)(int32_t)IRQn)]               >> (8U - 4U)));
  }
}












 
static __inline uint32_t NVIC_EncodePriority (uint32_t PriorityGroup, uint32_t PreemptPriority, uint32_t SubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);    
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7UL - PriorityGroupTmp) > (uint32_t)(4U)) ? (uint32_t)(4U) : (uint32_t)(7UL - PriorityGroupTmp);
  SubPriorityBits     = ((PriorityGroupTmp + (uint32_t)(4U)) < (uint32_t)7UL) ? (uint32_t)0UL : (uint32_t)((PriorityGroupTmp - 7UL) + (uint32_t)(4U));

  return (
           ((PreemptPriority & (uint32_t)((1UL << (PreemptPriorityBits)) - 1UL)) << SubPriorityBits) |
           ((SubPriority     & (uint32_t)((1UL << (SubPriorityBits    )) - 1UL)))
         );
}












 
static __inline void NVIC_DecodePriority (uint32_t Priority, uint32_t PriorityGroup, uint32_t* const pPreemptPriority, uint32_t* const pSubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);    
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7UL - PriorityGroupTmp) > (uint32_t)(4U)) ? (uint32_t)(4U) : (uint32_t)(7UL - PriorityGroupTmp);
  SubPriorityBits     = ((PriorityGroupTmp + (uint32_t)(4U)) < (uint32_t)7UL) ? (uint32_t)0UL : (uint32_t)((PriorityGroupTmp - 7UL) + (uint32_t)(4U));

  *pPreemptPriority = (Priority >> SubPriorityBits) & (uint32_t)((1UL << (PreemptPriorityBits)) - 1UL);
  *pSubPriority     = (Priority                   ) & (uint32_t)((1UL << (SubPriorityBits    )) - 1UL);
}





 
static __inline void NVIC_SystemReset(void)
{
  do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);                                                          
 
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR  = (uint32_t)((0x5FAUL << 16U)    |
                           (((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8U)) |
                            (1UL << 2U)    );          
  do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);                                                           

  for(;;)                                                            
  {
    __nop();
  }
}

 



 





 













 
static __inline uint32_t SysTick_Config(uint32_t ticks)
{
  if ((ticks - 1UL) > (0xFFFFFFUL ))
  {
    return (1UL);                                                    
  }

  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD  = (uint32_t)(ticks - 1UL);                          
  NVIC_SetPriority (SysTick_IRQn, (1UL << 4U) - 1UL);  
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL   = 0UL;                                              
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL  = (1UL << 2U) |
                   (1UL << 1U)   |
                   (1UL );                          
  return (0UL);                                                      
}



 



 





 

extern volatile int32_t ITM_RxBuffer;                     










 
static __inline uint32_t ITM_SendChar (uint32_t ch)
{
  if (((((ITM_Type *) (0xE0000000UL) )->TCR & (1UL )) != 0UL) &&       
      ((((ITM_Type *) (0xE0000000UL) )->TER & 1UL               ) != 0UL)   )      
  {
    while (((ITM_Type *) (0xE0000000UL) )->PORT[0U].u32 == 0UL)
    {
      __nop();
    }
    ((ITM_Type *) (0xE0000000UL) )->PORT[0U].u8 = (uint8_t)ch;
  }
  return (ch);
}







 
static __inline int32_t ITM_ReceiveChar (void)
{
  int32_t ch = -1;                            

  if (ITM_RxBuffer != 0x5AA55AA5U)
  {
    ch = ITM_RxBuffer;
    ITM_RxBuffer = 0x5AA55AA5U;        
  }

  return (ch);
}







 
static __inline int32_t ITM_CheckChar (void)
{

  if (ITM_RxBuffer == 0x5AA55AA5U)
  {
    return (0);                               
  }
  else
  {
    return (1);                               
  }
}

 










#line 184 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"
#line 1 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/system_stm32f4xx.h"

































  



 



   
  


 









 



 




 
  






 
extern uint32_t SystemCoreClock;           

extern const uint8_t  AHBPrescTable[16];     
extern const uint8_t  APBPrescTable[8];      



 



 



 



 



 



 
  
extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);


 









 
  


   
 
#line 185 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"
#line 186 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"



    



 

typedef struct
{
  volatile uint32_t SR;      
  volatile uint32_t CR1;     
  volatile uint32_t CR2;     
  volatile uint32_t SMPR1;   
  volatile uint32_t SMPR2;   
  volatile uint32_t JOFR1;   
  volatile uint32_t JOFR2;   
  volatile uint32_t JOFR3;   
  volatile uint32_t JOFR4;   
  volatile uint32_t HTR;     
  volatile uint32_t LTR;     
  volatile uint32_t SQR1;    
  volatile uint32_t SQR2;    
  volatile uint32_t SQR3;    
  volatile uint32_t JSQR;    
  volatile uint32_t JDR1;    
  volatile uint32_t JDR2;    
  volatile uint32_t JDR3;    
  volatile uint32_t JDR4;    
  volatile uint32_t DR;      
} ADC_TypeDef;

typedef struct
{
  volatile uint32_t CSR;     
  volatile uint32_t CCR;     
  volatile uint32_t CDR;    
 
} ADC_Common_TypeDef;




 

typedef struct
{
  volatile uint32_t TIR;   
  volatile uint32_t TDTR;  
  volatile uint32_t TDLR;  
  volatile uint32_t TDHR;  
} CAN_TxMailBox_TypeDef;



 
  
typedef struct
{
  volatile uint32_t RIR;   
  volatile uint32_t RDTR;  
  volatile uint32_t RDLR;  
  volatile uint32_t RDHR;  
} CAN_FIFOMailBox_TypeDef;



 
  
typedef struct
{
  volatile uint32_t FR1;  
  volatile uint32_t FR2;  
} CAN_FilterRegister_TypeDef;



 
  
typedef struct
{
  volatile uint32_t              MCR;                  
  volatile uint32_t              MSR;                  
  volatile uint32_t              TSR;                  
  volatile uint32_t              RF0R;                 
  volatile uint32_t              RF1R;                 
  volatile uint32_t              IER;                  
  volatile uint32_t              ESR;                  
  volatile uint32_t              BTR;                  
  uint32_t                   RESERVED0[88];        
  CAN_TxMailBox_TypeDef      sTxMailBox[3];        
  CAN_FIFOMailBox_TypeDef    sFIFOMailBox[2];      
  uint32_t                   RESERVED1[12];        
  volatile uint32_t              FMR;                  
  volatile uint32_t              FM1R;                 
  uint32_t                   RESERVED2;            
  volatile uint32_t              FS1R;                 
  uint32_t                   RESERVED3;            
  volatile uint32_t              FFA1R;                
  uint32_t                   RESERVED4;            
  volatile uint32_t              FA1R;                 
  uint32_t                   RESERVED5[8];          
  CAN_FilterRegister_TypeDef sFilterRegister[28];  
} CAN_TypeDef;



 

typedef struct
{
  volatile uint32_t DR;          
  volatile uint8_t  IDR;         
  uint8_t       RESERVED0;   
  uint16_t      RESERVED1;   
  volatile uint32_t CR;          
} CRC_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;        
  volatile uint32_t SWTRIGR;   
  volatile uint32_t DHR12R1;   
  volatile uint32_t DHR12L1;   
  volatile uint32_t DHR8R1;    
  volatile uint32_t DHR12R2;   
  volatile uint32_t DHR12L2;   
  volatile uint32_t DHR8R2;    
  volatile uint32_t DHR12RD;   
  volatile uint32_t DHR12LD;   
  volatile uint32_t DHR8RD;    
  volatile uint32_t DOR1;      
  volatile uint32_t DOR2;      
  volatile uint32_t SR;        
} DAC_TypeDef;



 

typedef struct
{
  volatile uint32_t IDCODE;   
  volatile uint32_t CR;       
  volatile uint32_t APB1FZ;   
  volatile uint32_t APB2FZ;   
}DBGMCU_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;        
  volatile uint32_t SR;        
  volatile uint32_t RISR;      
  volatile uint32_t IER;       
  volatile uint32_t MISR;      
  volatile uint32_t ICR;       
  volatile uint32_t ESCR;      
  volatile uint32_t ESUR;      
  volatile uint32_t CWSTRTR;   
  volatile uint32_t CWSIZER;   
  volatile uint32_t DR;        
} DCMI_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;      
  volatile uint32_t NDTR;    
  volatile uint32_t PAR;     
  volatile uint32_t M0AR;    
  volatile uint32_t M1AR;    
  volatile uint32_t FCR;     
} DMA_Stream_TypeDef;

typedef struct
{
  volatile uint32_t LISR;    
  volatile uint32_t HISR;    
  volatile uint32_t LIFCR;   
  volatile uint32_t HIFCR;   
} DMA_TypeDef;



 

typedef struct
{
  volatile uint32_t MACCR;
  volatile uint32_t MACFFR;
  volatile uint32_t MACHTHR;
  volatile uint32_t MACHTLR;
  volatile uint32_t MACMIIAR;
  volatile uint32_t MACMIIDR;
  volatile uint32_t MACFCR;
  volatile uint32_t MACVLANTR;              
  uint32_t      RESERVED0[2];
  volatile uint32_t MACRWUFFR;              
  volatile uint32_t MACPMTCSR;
  uint32_t      RESERVED1;
  volatile uint32_t MACDBGR;
  volatile uint32_t MACSR;                  
  volatile uint32_t MACIMR;
  volatile uint32_t MACA0HR;
  volatile uint32_t MACA0LR;
  volatile uint32_t MACA1HR;
  volatile uint32_t MACA1LR;
  volatile uint32_t MACA2HR;
  volatile uint32_t MACA2LR;
  volatile uint32_t MACA3HR;
  volatile uint32_t MACA3LR;                
  uint32_t      RESERVED2[40];
  volatile uint32_t MMCCR;                  
  volatile uint32_t MMCRIR;
  volatile uint32_t MMCTIR;
  volatile uint32_t MMCRIMR;
  volatile uint32_t MMCTIMR;                
  uint32_t      RESERVED3[14];
  volatile uint32_t MMCTGFSCCR;             
  volatile uint32_t MMCTGFMSCCR;
  uint32_t      RESERVED4[5];
  volatile uint32_t MMCTGFCR;
  uint32_t      RESERVED5[10];
  volatile uint32_t MMCRFCECR;
  volatile uint32_t MMCRFAECR;
  uint32_t      RESERVED6[10];
  volatile uint32_t MMCRGUFCR;
  uint32_t      RESERVED7[334];
  volatile uint32_t PTPTSCR;
  volatile uint32_t PTPSSIR;
  volatile uint32_t PTPTSHR;
  volatile uint32_t PTPTSLR;
  volatile uint32_t PTPTSHUR;
  volatile uint32_t PTPTSLUR;
  volatile uint32_t PTPTSAR;
  volatile uint32_t PTPTTHR;
  volatile uint32_t PTPTTLR;
  volatile uint32_t RESERVED8;
  volatile uint32_t PTPTSSR;
  uint32_t      RESERVED9[565];
  volatile uint32_t DMABMR;
  volatile uint32_t DMATPDR;
  volatile uint32_t DMARPDR;
  volatile uint32_t DMARDLAR;
  volatile uint32_t DMATDLAR;
  volatile uint32_t DMASR;
  volatile uint32_t DMAOMR;
  volatile uint32_t DMAIER;
  volatile uint32_t DMAMFBOCR;
  volatile uint32_t DMARSWTR;
  uint32_t      RESERVED10[8];
  volatile uint32_t DMACHTDR;
  volatile uint32_t DMACHRDR;
  volatile uint32_t DMACHTBAR;
  volatile uint32_t DMACHRBAR;
} ETH_TypeDef;



 

typedef struct
{
  volatile uint32_t IMR;     
  volatile uint32_t EMR;     
  volatile uint32_t RTSR;    
  volatile uint32_t FTSR;    
  volatile uint32_t SWIER;   
  volatile uint32_t PR;      
} EXTI_TypeDef;



 

typedef struct
{
  volatile uint32_t ACR;       
  volatile uint32_t KEYR;      
  volatile uint32_t OPTKEYR;   
  volatile uint32_t SR;        
  volatile uint32_t CR;        
  volatile uint32_t OPTCR;     
  volatile uint32_t OPTCR1;    
} FLASH_TypeDef;





 

typedef struct
{
  volatile uint32_t BTCR[8];        
} FSMC_Bank1_TypeDef;



 

typedef struct
{
  volatile uint32_t BWTR[7];     
} FSMC_Bank1E_TypeDef;



 
  
typedef struct
{
  volatile uint32_t PCR2;        
  volatile uint32_t SR2;         
  volatile uint32_t PMEM2;       
  volatile uint32_t PATT2;       
  uint32_t      RESERVED0;   
  volatile uint32_t ECCR2;       
  uint32_t      RESERVED1;   
  uint32_t      RESERVED2;   
  volatile uint32_t PCR3;        
  volatile uint32_t SR3;         
  volatile uint32_t PMEM3;       
  volatile uint32_t PATT3;       
  uint32_t      RESERVED3;   
  volatile uint32_t ECCR3;       
} FSMC_Bank2_3_TypeDef;



 

typedef struct
{
  volatile uint32_t PCR4;        
  volatile uint32_t SR4;         
  volatile uint32_t PMEM4;       
  volatile uint32_t PATT4;       
  volatile uint32_t PIO4;        
} FSMC_Bank4_TypeDef; 



 

typedef struct
{
  volatile uint32_t MODER;     
  volatile uint32_t OTYPER;    
  volatile uint32_t OSPEEDR;   
  volatile uint32_t PUPDR;     
  volatile uint32_t IDR;       
  volatile uint32_t ODR;       
  volatile uint32_t BSRR;      
  volatile uint32_t LCKR;      
  volatile uint32_t AFR[2];    
} GPIO_TypeDef;



 

typedef struct
{
  volatile uint32_t MEMRMP;        
  volatile uint32_t PMC;           
  volatile uint32_t EXTICR[4];     
  uint32_t      RESERVED[2];   
  volatile uint32_t CMPCR;         
} SYSCFG_TypeDef;



 

typedef struct
{
  volatile uint32_t CR1;         
  volatile uint32_t CR2;         
  volatile uint32_t OAR1;        
  volatile uint32_t OAR2;        
  volatile uint32_t DR;          
  volatile uint32_t SR1;         
  volatile uint32_t SR2;         
  volatile uint32_t CCR;         
  volatile uint32_t TRISE;       
} I2C_TypeDef;



 

typedef struct
{
  volatile uint32_t KR;    
  volatile uint32_t PR;    
  volatile uint32_t RLR;   
  volatile uint32_t SR;    
} IWDG_TypeDef;




 

typedef struct
{
  volatile uint32_t CR;    
  volatile uint32_t CSR;   
} PWR_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;             
  volatile uint32_t PLLCFGR;        
  volatile uint32_t CFGR;           
  volatile uint32_t CIR;            
  volatile uint32_t AHB1RSTR;       
  volatile uint32_t AHB2RSTR;       
  volatile uint32_t AHB3RSTR;       
  uint32_t      RESERVED0;      
  volatile uint32_t APB1RSTR;       
  volatile uint32_t APB2RSTR;       
  uint32_t      RESERVED1[2];   
  volatile uint32_t AHB1ENR;        
  volatile uint32_t AHB2ENR;        
  volatile uint32_t AHB3ENR;        
  uint32_t      RESERVED2;      
  volatile uint32_t APB1ENR;        
  volatile uint32_t APB2ENR;        
  uint32_t      RESERVED3[2];   
  volatile uint32_t AHB1LPENR;      
  volatile uint32_t AHB2LPENR;      
  volatile uint32_t AHB3LPENR;      
  uint32_t      RESERVED4;      
  volatile uint32_t APB1LPENR;      
  volatile uint32_t APB2LPENR;      
  uint32_t      RESERVED5[2];   
  volatile uint32_t BDCR;           
  volatile uint32_t CSR;            
  uint32_t      RESERVED6[2];   
  volatile uint32_t SSCGR;          
  volatile uint32_t PLLI2SCFGR;     
} RCC_TypeDef;



 

typedef struct
{
  volatile uint32_t TR;       
  volatile uint32_t DR;       
  volatile uint32_t CR;       
  volatile uint32_t ISR;      
  volatile uint32_t PRER;     
  volatile uint32_t WUTR;     
  volatile uint32_t CALIBR;   
  volatile uint32_t ALRMAR;   
  volatile uint32_t ALRMBR;   
  volatile uint32_t WPR;      
  volatile uint32_t SSR;      
  volatile uint32_t SHIFTR;   
  volatile uint32_t TSTR;     
  volatile uint32_t TSDR;     
  volatile uint32_t TSSSR;    
  volatile uint32_t CALR;     
  volatile uint32_t TAFCR;    
  volatile uint32_t ALRMASSR; 
  volatile uint32_t ALRMBSSR; 
  uint32_t RESERVED7;     
  volatile uint32_t BKP0R;    
  volatile uint32_t BKP1R;    
  volatile uint32_t BKP2R;    
  volatile uint32_t BKP3R;    
  volatile uint32_t BKP4R;    
  volatile uint32_t BKP5R;    
  volatile uint32_t BKP6R;    
  volatile uint32_t BKP7R;    
  volatile uint32_t BKP8R;    
  volatile uint32_t BKP9R;    
  volatile uint32_t BKP10R;   
  volatile uint32_t BKP11R;   
  volatile uint32_t BKP12R;   
  volatile uint32_t BKP13R;   
  volatile uint32_t BKP14R;   
  volatile uint32_t BKP15R;   
  volatile uint32_t BKP16R;   
  volatile uint32_t BKP17R;   
  volatile uint32_t BKP18R;   
  volatile uint32_t BKP19R;   
} RTC_TypeDef;



 

typedef struct
{
  volatile uint32_t POWER;                  
  volatile uint32_t CLKCR;                  
  volatile uint32_t ARG;                    
  volatile uint32_t CMD;                    
  volatile const uint32_t  RESPCMD;         
  volatile const uint32_t  RESP1;           
  volatile const uint32_t  RESP2;           
  volatile const uint32_t  RESP3;           
  volatile const uint32_t  RESP4;           
  volatile uint32_t DTIMER;                 
  volatile uint32_t DLEN;                   
  volatile uint32_t DCTRL;                  
  volatile const uint32_t  DCOUNT;          
  volatile const uint32_t  STA;             
  volatile uint32_t ICR;                    
  volatile uint32_t MASK;                   
  uint32_t      RESERVED0[2];           
  volatile const uint32_t  FIFOCNT;         
  uint32_t      RESERVED1[13];          
  volatile uint32_t FIFO;                   
} SDIO_TypeDef;



 

typedef struct
{
  volatile uint32_t CR1;         
  volatile uint32_t CR2;         
  volatile uint32_t SR;          
  volatile uint32_t DR;          
  volatile uint32_t CRCPR;       
  volatile uint32_t RXCRCR;      
  volatile uint32_t TXCRCR;      
  volatile uint32_t I2SCFGR;     
  volatile uint32_t I2SPR;       
} SPI_TypeDef;




 

typedef struct
{
  volatile uint32_t CR1;          
  volatile uint32_t CR2;          
  volatile uint32_t SMCR;         
  volatile uint32_t DIER;         
  volatile uint32_t SR;           
  volatile uint32_t EGR;          
  volatile uint32_t CCMR1;        
  volatile uint32_t CCMR2;        
  volatile uint32_t CCER;         
  volatile uint32_t CNT;          
  volatile uint32_t PSC;          
  volatile uint32_t ARR;          
  volatile uint32_t RCR;          
  volatile uint32_t CCR1;         
  volatile uint32_t CCR2;         
  volatile uint32_t CCR3;         
  volatile uint32_t CCR4;         
  volatile uint32_t BDTR;         
  volatile uint32_t DCR;          
  volatile uint32_t DMAR;         
  volatile uint32_t OR;           
} TIM_TypeDef;



 
 
typedef struct
{
  volatile uint32_t SR;          
  volatile uint32_t DR;          
  volatile uint32_t BRR;         
  volatile uint32_t CR1;         
  volatile uint32_t CR2;         
  volatile uint32_t CR3;         
  volatile uint32_t GTPR;        
} USART_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;    
  volatile uint32_t CFR;   
  volatile uint32_t SR;    
} WWDG_TypeDef;



 
  
typedef struct 
{
  volatile uint32_t CR;   
  volatile uint32_t SR;   
  volatile uint32_t DR;   
} RNG_TypeDef;



 
typedef struct
{
  volatile uint32_t GOTGCTL;               
  volatile uint32_t GOTGINT;               
  volatile uint32_t GAHBCFG;               
  volatile uint32_t GUSBCFG;               
  volatile uint32_t GRSTCTL;               
  volatile uint32_t GINTSTS;               
  volatile uint32_t GINTMSK;               
  volatile uint32_t GRXSTSR;               
  volatile uint32_t GRXSTSP;               
  volatile uint32_t GRXFSIZ;               
  volatile uint32_t DIEPTXF0_HNPTXFSIZ;    
  volatile uint32_t HNPTXSTS;              
  uint32_t Reserved30[2];              
  volatile uint32_t GCCFG;                 
  volatile uint32_t CID;                   
  uint32_t  Reserved40[48];            
  volatile uint32_t HPTXFSIZ;              
  volatile uint32_t DIEPTXF[0x0F];         
} USB_OTG_GlobalTypeDef;



 
typedef struct 
{
  volatile uint32_t DCFG;             
  volatile uint32_t DCTL;             
  volatile uint32_t DSTS;             
  uint32_t Reserved0C;            
  volatile uint32_t DIEPMSK;          
  volatile uint32_t DOEPMSK;          
  volatile uint32_t DAINT;            
  volatile uint32_t DAINTMSK;         
  uint32_t  Reserved20;           
  uint32_t Reserved9;             
  volatile uint32_t DVBUSDIS;         
  volatile uint32_t DVBUSPULSE;       
  volatile uint32_t DTHRCTL;          
  volatile uint32_t DIEPEMPMSK;       
  volatile uint32_t DEACHINT;         
  volatile uint32_t DEACHMSK;         
  uint32_t Reserved40;            
  volatile uint32_t DINEP1MSK;        
  uint32_t  Reserved44[15];       
  volatile uint32_t DOUTEP1MSK;       
} USB_OTG_DeviceTypeDef;



 
typedef struct 
{
  volatile uint32_t DIEPCTL;            
  uint32_t Reserved04;              
  volatile uint32_t DIEPINT;            
  uint32_t Reserved0C;              
  volatile uint32_t DIEPTSIZ;           
  volatile uint32_t DIEPDMA;            
  volatile uint32_t DTXFSTS;            
  uint32_t Reserved18;              
} USB_OTG_INEndpointTypeDef;



 
typedef struct 
{
  volatile uint32_t DOEPCTL;        
  uint32_t Reserved04;          
  volatile uint32_t DOEPINT;        
  uint32_t Reserved0C;          
  volatile uint32_t DOEPTSIZ;       
  volatile uint32_t DOEPDMA;        
  uint32_t Reserved18[2];       
} USB_OTG_OUTEndpointTypeDef;



 
typedef struct 
{
  volatile uint32_t HCFG;              
  volatile uint32_t HFIR;              
  volatile uint32_t HFNUM;             
  uint32_t Reserved40C;            
  volatile uint32_t HPTXSTS;           
  volatile uint32_t HAINT;             
  volatile uint32_t HAINTMSK;          
} USB_OTG_HostTypeDef;



 
typedef struct
{
  volatile uint32_t HCCHAR;            
  volatile uint32_t HCSPLT;            
  volatile uint32_t HCINT;             
  volatile uint32_t HCINTMSK;          
  volatile uint32_t HCTSIZ;            
  volatile uint32_t HCDMA;             
  uint32_t Reserved[2];            
} USB_OTG_HostChannelTypeDef;



 



 
#line 938 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 



 





 
#line 977 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 987 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"
 
#line 996 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 1033 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 



 






 

 



#line 1063 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"






 



   
#line 1110 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"
 
#line 1159 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"



 



 
  
  

 
    
 
 
 

 
 
 
 
 


 


 
#line 1205 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 1259 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"
  
 
#line 1309 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 1365 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 1427 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 




 




 




 




 




 
#line 1498 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 1548 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 1598 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 1637 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 




 




 




 
#line 1665 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 1721 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 
#line 1762 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 1770 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 



 
 
 
 
 
 
 
#line 1812 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"
 
#line 1840 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 1890 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 1903 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 1916 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 1930 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 1944 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 1989 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 2000 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 2007 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 2014 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 2043 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"


 
 
#line 2062 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 2073 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 2087 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 2101 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 2118 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 2129 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 2143 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 2157 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 2174 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

   
#line 2185 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 2199 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 2213 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 2227 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 2238 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 2252 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 2266 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 2280 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 2291 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 2305 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 2319 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
 
#line 2328 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 2417 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 2506 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 2595 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 2684 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"


 
#line 2783 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 2881 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 2979 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 3077 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 3175 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 3273 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 3371 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 3469 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 3567 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 3665 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 3763 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 3861 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 3959 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 4057 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 4155 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 4253 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 4351 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 4449 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 4547 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 4645 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 4743 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 4841 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 4939 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 5037 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 5135 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 5233 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 5331 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 5429 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
 
 
 
 
 





 





 




 
 
 
 
 


 

 
#line 5471 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 5478 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"







#line 5492 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 5508 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 5515 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"







#line 5529 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 5536 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 5544 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 




 




 




 




 




 
#line 5582 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 5590 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 5598 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 




 
#line 5616 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
 
 
 
 
 
#line 5657 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 5668 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 5685 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"
 
#line 5692 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 5709 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"
 


 
#line 5728 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 






 
#line 5752 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 


 
#line 5769 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 5783 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 5791 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 5799 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 5813 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
 
 
 
 
 
#line 5892 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 
#line 5918 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

  
#line 5937 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

  
#line 5999 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

  
#line 6061 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

  
#line 6123 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

  
#line 6185 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 




 





 
 
 
 
 
 
#line 6277 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 6305 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 6376 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 6401 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 6472 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 6543 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 6614 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 6685 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
 
 
 
 
 
#line 6703 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 6725 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 6748 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 6781 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 6789 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 6830 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"
                                             
 
#line 6847 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
 
 
 
 
 
#line 6860 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"













#line 6909 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 6917 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"













#line 6966 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 6974 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"













#line 7023 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 7031 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"













#line 7080 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 7089 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 7097 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 7109 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 7117 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 7125 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 7133 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"







 
#line 7148 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 7156 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 7168 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 7176 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 7184 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 7192 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"







 
#line 7207 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 7215 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 7227 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 7235 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 7243 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 7251 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"







 
#line 7266 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 7274 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 7286 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 7294 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 7302 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 7310 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"







 
#line 7325 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 7333 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 7345 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 7353 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"







 
#line 7368 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 7376 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 7388 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 7396 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"







 
#line 7411 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 7419 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 7431 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 7439 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"







 
#line 7454 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 7462 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 7474 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 7482 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"







 
#line 7499 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"











#line 7517 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 7525 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 7532 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 7543 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"











#line 7561 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 7569 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 7576 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 7587 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"











#line 7605 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 7613 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 7620 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 7643 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 7666 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 7689 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 7702 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 7714 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 7726 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 7738 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 7751 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 7763 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 7775 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 7787 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 7800 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 7812 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 7824 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 7836 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 7849 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 7861 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 7873 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 7885 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 7898 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 7910 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 7922 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 7934 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 7947 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 7959 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 7971 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 7983 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 7996 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 8008 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 8020 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 8032 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 




 
 
 
 
 
 
#line 8129 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 8211 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 8261 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 8279 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 8361 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 8411 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 8493 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 8543 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 8593 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 8611 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 8661 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"
 
#line 8678 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 8776 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 8810 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"
 
#line 8862 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"
 
#line 8919 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 8961 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 9019 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 9061 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 9111 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"


 
 
 
 
 
 
#line 9161 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 9172 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 9188 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 



#line 9223 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"





 
#line 9235 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 
#line 9284 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 9310 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 9321 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 





 
 
 
 
 
 




 
#line 9345 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 
#line 9358 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"



 
 
 
 
 
 
#line 9382 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 9389 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 9408 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 


 
#line 9434 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 


 
 
 
 
 
 
#line 9450 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 9459 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 9471 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 9490 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"


 


#line 9501 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 9512 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 9525 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"







#line 9539 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 9547 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"


 
 










 










 
#line 9580 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 9590 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 9598 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"







 
#line 9612 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"







 
#line 9628 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 










#line 9646 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 9653 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"







 
#line 9679 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 9701 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 9720 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"





 
#line 9768 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 9779 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"
 





 
#line 9855 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 9890 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 


 
#line 9955 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"
 


 


#line 9970 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 


 






 
#line 10051 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 10092 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 10142 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 10161 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 10172 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 
#line 10248 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 10289 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 10300 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"







#line 10313 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 10345 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"
 



 
#line 10362 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 10376 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 10383 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"


 
 
 
 
 
 
#line 10397 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 10414 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
 
 
 
 


 


 
#line 10467 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 10511 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 10581 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 


 
#line 10634 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 10642 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 
#line 10655 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 10725 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 10795 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 




 
#line 10813 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 10856 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 10886 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 
#line 10914 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 10962 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 


 
#line 10977 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 10989 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 




 



 
 
 
 
 
 






 
#line 11119 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"







#line 11132 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 










#line 11170 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 




 




 




 




 




 




 




 
#line 11224 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 11232 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 11245 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 
#line 11324 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 11365 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 11439 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 




 
 
 
 
 


 
#line 11467 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 11474 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 11505 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 11528 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 11557 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 




 




 




 






























#line 11615 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 11626 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
 
 
 
 
 





 



 


 
#line 11658 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"


 
#line 11670 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"



 
#line 11683 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"



 
#line 11696 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"



 
#line 11709 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 11723 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"



 
#line 11736 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"



 
#line 11749 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"



 
#line 11762 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"



 
#line 11775 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 11789 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"



 
#line 11802 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"



 
#line 11815 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"



 
#line 11828 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"



 
#line 11841 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 11855 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"



 
#line 11867 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"



 
#line 11879 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"



 
#line 11891 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"



 
#line 11903 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 11911 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
 
 
 
 
 
#line 11933 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

















 
#line 11960 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 11967 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 11992 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 12000 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 12007 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"





#line 12019 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"







#line 12032 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 12079 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 12117 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 12143 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 






#line 12157 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 12164 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"











#line 12181 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 12188 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"





 







#line 12208 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"







#line 12222 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 






#line 12236 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 12243 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"











#line 12260 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 12267 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"





 







#line 12287 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"







#line 12301 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 12348 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 




 




 




 




 




 




 




 
#line 12401 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"







#line 12426 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 12436 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 12445 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 






#line 12468 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"


 
 
 
 
 
 
#line 12506 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 
#line 12519 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 12566 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 12589 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"











 
#line 12637 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 12650 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"





 
 
 
 
 
 
#line 12671 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"
 
#line 12679 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"





 
#line 12695 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"
 
#line 12703 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"






 







 





 
 
 
 
 
 
#line 12735 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 12749 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"







 
#line 12808 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"
 


 
#line 12827 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
 
 
 
 
 
#line 12892 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 12936 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 




 
#line 12976 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 
#line 13014 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 13022 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

  




 








 

  
#line 13061 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 13141 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 13158 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 13166 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 




 
#line 13196 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 
#line 13221 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 
#line 13246 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 
 
 

 
#line 13275 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 13286 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 13297 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 13308 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 13319 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 




 




 




 




 




 
 
 

 
#line 13382 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 13401 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 




 
#line 13419 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 
#line 13432 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 




 




 
#line 13455 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
 
 

 
#line 13522 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 




 




 




 
#line 13556 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"
   
#line 13649 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 13699 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 13746 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 13760 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 




 




 




 
 
 
 
 
 
#line 13817 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 

#line 13828 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 

#line 13839 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 13850 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"













 
#line 13873 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 13893 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 13907 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 13929 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 
#line 13942 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




#line 13959 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 13981 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 

#line 14045 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 14062 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"


#line 14078 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 14104 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 14120 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 14132 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 
#line 14160 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 14240 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 14320 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 14328 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 
#line 14347 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 14355 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 




 
#line 14379 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 




#line 14401 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 14412 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 14420 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 14436 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 14452 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 
#line 14465 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 14485 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 14493 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 
#line 14527 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 14556 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 14565 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 14573 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"







 
#line 14614 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 14622 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 14636 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 14645 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 14671 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




#line 14690 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"













#line 14722 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 

#line 14735 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 14746 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 14758 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 14793 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 14828 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 14863 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 

#line 14875 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"
 
#line 14890 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 




 




 
#line 14913 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 

#line 14954 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 14974 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 

#line 14983 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"







 
#line 15000 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
 
#line 15013 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"







#line 15027 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 15035 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

#line 15043 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"


  



 



 

 








 


 


 


 


 
#line 15093 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 15104 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 


 




 


 


 


 



 





 
#line 15152 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 15166 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 15176 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 15184 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 15192 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 



 
#line 15204 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 15214 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 15222 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 15230 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 15238 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 15248 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 15258 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 



 
#line 15270 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 
#line 15333 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 15345 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 15353 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 15367 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 




 
#line 15380 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 15390 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 
#line 15398 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 



 
#line 15412 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"
 
#line 15419 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"
 



 





 
#line 15436 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"

 


 




 


 





 
#line 15461 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"


 



 



 


 


 


 







 



#line 15499 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f407xx.h"



















 
 
 
 
 
 
 
 


 




 



 



 









 
#line 150 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f4xx.h"
#line 193 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f4xx.h"



 



  
typedef enum 
{
  RESET = 0U, 
  SET = !RESET
} FlagStatus, ITStatus;

typedef enum 
{
  DISABLE = 0U, 
  ENABLE = !DISABLE
} FunctionalState;


typedef enum 
{
  ERROR = 0U, 
  SUCCESS = !ERROR
} ErrorStatus;



 




 



















 

#line 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal.h"


































  

 
#line 299 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal.h"

 
#line 251 "../Drivers/CMSIS/Device/ST/STM32F4xx/Include/stm32f4xx.h"









 



 
  



 
#line 56 "../Inc/main.h"
#line 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_system.h"













































 

 







 
#line 58 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_system.h"



 





 

 
 

 


 



 

 

 
 


 



 
#line 102 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_system.h"



 


 

 





 




#line 133 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_system.h"


  






 



 
#line 168 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_system.h"


 



 
#line 191 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_system.h"


 



 
#line 204 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_system.h"


 

#line 349 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_system.h"



 







 



 
#line 413 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_system.h"


 



 
#line 429 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_system.h"


 



 
#line 452 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_system.h"


 



 

 

 


 



 










 
static __inline void LL_SYSCFG_SetRemapMemory(uint32_t Memory)
{
  (((((SYSCFG_TypeDef *) ((0x40000000U + 0x00010000U) + 0x3800U))->MEMRMP)) = ((((((((SYSCFG_TypeDef *) ((0x40000000U + 0x00010000U) + 0x3800U))->MEMRMP))) & (~((0x3U << (0U))))) | (Memory))));
}










 
static __inline uint32_t LL_SYSCFG_GetRemapMemory(void)
{
  return (uint32_t)(((((SYSCFG_TypeDef *) ((0x40000000U + 0x00010000U) + 0x3800U))->MEMRMP) & ((0x3U << (0U)))));
}

#line 527 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_system.h"






 
static __inline void LL_SYSCFG_EnableCompensationCell(void)
{
  ((((SYSCFG_TypeDef *) ((0x40000000U + 0x00010000U) + 0x3800U))->CMPCR) |= ((0x1U << (0U))));
}







 
static __inline void LL_SYSCFG_DisableCompensationCell(void)
{
  ((((SYSCFG_TypeDef *) ((0x40000000U + 0x00010000U) + 0x3800U))->CMPCR) &= ~((0x1U << (0U))));
}





 
static __inline uint32_t LL_SYSCFG_IsActiveFlag_CMPCR(void)
{
  return (((((SYSCFG_TypeDef *) ((0x40000000U + 0x00010000U) + 0x3800U))->CMPCR) & ((0x1U << (8U)))) == ((0x1U << (8U))));
}









 
static __inline void LL_SYSCFG_SetPHYInterface(uint32_t Interface)
{
  (((((SYSCFG_TypeDef *) ((0x40000000U + 0x00010000U) + 0x3800U))->PMC)) = ((((((((SYSCFG_TypeDef *) ((0x40000000U + 0x00010000U) + 0x3800U))->PMC))) & (~((0x1U << (23U))))) | (Interface))));
}








 
static __inline uint32_t LL_SYSCFG_GetPHYInterface(void)
{
  return (uint32_t)(((((SYSCFG_TypeDef *) ((0x40000000U + 0x00010000U) + 0x3800U))->PMC) & ((0x1U << (23U)))));
}

 


#line 617 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_system.h"

#line 649 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_system.h"




































 
static __inline void LL_SYSCFG_SetEXTISource(uint32_t Port, uint32_t Line)
{
  (((((SYSCFG_TypeDef *) ((0x40000000U + 0x00010000U) + 0x3800U))->EXTICR[Line & 0xFF])) = ((((((((SYSCFG_TypeDef *) ((0x40000000U + 0x00010000U) + 0x3800U))->EXTICR[Line & 0xFF]))) & (~((Line >> 16)))) | (Port << (__clz(__rbit((Line >> 16))))))));
}


































 
static __inline uint32_t LL_SYSCFG_GetEXTISource(uint32_t Line)
{
  return (uint32_t)(((((SYSCFG_TypeDef *) ((0x40000000U + 0x00010000U) + 0x3800U))->EXTICR[Line & 0xFF]) & ((Line >> 16))) >> (__clz(__rbit(Line >> 16))));
}

#line 1242 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_system.h"


 




 















 
static __inline uint32_t LL_DBGMCU_GetDeviceID(void)
{
  return (uint32_t)(((((DBGMCU_TypeDef *) 0xE0042000U)->IDCODE) & ((0xFFFU << (0U)))));
}












 
static __inline uint32_t LL_DBGMCU_GetRevisionID(void)
{
  return (uint32_t)(((((DBGMCU_TypeDef *) 0xE0042000U)->IDCODE) & ((0xFFFFU << (16U)))) >> (16U));
}





 
static __inline void LL_DBGMCU_EnableDBGSleepMode(void)
{
  ((((DBGMCU_TypeDef *) 0xE0042000U)->CR) |= ((0x1U << (0U))));
}





 
static __inline void LL_DBGMCU_DisableDBGSleepMode(void)
{
  ((((DBGMCU_TypeDef *) 0xE0042000U)->CR) &= ~((0x1U << (0U))));
}





 
static __inline void LL_DBGMCU_EnableDBGStopMode(void)
{
  ((((DBGMCU_TypeDef *) 0xE0042000U)->CR) |= ((0x1U << (1U))));
}





 
static __inline void LL_DBGMCU_DisableDBGStopMode(void)
{
  ((((DBGMCU_TypeDef *) 0xE0042000U)->CR) &= ~((0x1U << (1U))));
}





 
static __inline void LL_DBGMCU_EnableDBGStandbyMode(void)
{
  ((((DBGMCU_TypeDef *) 0xE0042000U)->CR) |= ((0x1U << (2U))));
}





 
static __inline void LL_DBGMCU_DisableDBGStandbyMode(void)
{
  ((((DBGMCU_TypeDef *) 0xE0042000U)->CR) &= ~((0x1U << (2U))));
}












 
static __inline void LL_DBGMCU_SetTracePinAssignment(uint32_t PinAssignment)
{
  (((((DBGMCU_TypeDef *) 0xE0042000U)->CR)) = ((((((((DBGMCU_TypeDef *) 0xE0042000U)->CR))) & (~((0x1U << (5U)) | (0x3U << (6U))))) | (PinAssignment))));
}











 
static __inline uint32_t LL_DBGMCU_GetTracePinAssignment(void)
{
  return (uint32_t)(((((DBGMCU_TypeDef *) 0xE0042000U)->CR) & ((0x1U << (5U)) | (0x3U << (6U)))));
}















































 
static __inline void LL_DBGMCU_APB1_GRP1_FreezePeriph(uint32_t Periphs)
{
  ((((DBGMCU_TypeDef *) 0xE0042000U)->APB1FZ) |= (Periphs));
}















































 
static __inline void LL_DBGMCU_APB1_GRP1_UnFreezePeriph(uint32_t Periphs)
{
  ((((DBGMCU_TypeDef *) 0xE0042000U)->APB1FZ) &= ~(Periphs));
}

















 
static __inline void LL_DBGMCU_APB2_GRP1_FreezePeriph(uint32_t Periphs)
{
  ((((DBGMCU_TypeDef *) 0xE0042000U)->APB2FZ) |= (Periphs));
}

















 
static __inline void LL_DBGMCU_APB2_GRP1_UnFreezePeriph(uint32_t Periphs)
{
  ((((DBGMCU_TypeDef *) 0xE0042000U)->APB2FZ) &= ~(Periphs));
}


 



 






















 
static __inline void LL_FLASH_SetLatency(uint32_t Latency)
{
  (((((FLASH_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3C00U))->ACR)) = ((((((((FLASH_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3C00U))->ACR))) & (~((0xFU << (0U))))) | (Latency))));
}





















 
static __inline uint32_t LL_FLASH_GetLatency(void)
{
  return (uint32_t)(((((FLASH_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3C00U))->ACR) & ((0xFU << (0U)))));
}





 
static __inline void LL_FLASH_EnablePrefetch(void)
{
  ((((FLASH_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3C00U))->ACR) |= ((0x1U << (8U))));
}





 
static __inline void LL_FLASH_DisablePrefetch(void)
{
  ((((FLASH_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3C00U))->ACR) &= ~((0x1U << (8U))));
}





 
static __inline uint32_t LL_FLASH_IsPrefetchEnabled(void)
{
  return (((((FLASH_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3C00U))->ACR) & ((0x1U << (8U)))) == ((0x1U << (8U))));
}





 
static __inline void LL_FLASH_EnableInstCache(void)
{
  ((((FLASH_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3C00U))->ACR) |= ((0x1U << (9U))));
}





 
static __inline void LL_FLASH_DisableInstCache(void)
{
  ((((FLASH_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3C00U))->ACR) &= ~((0x1U << (9U))));
}





 
static __inline void LL_FLASH_EnableDataCache(void)
{
  ((((FLASH_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3C00U))->ACR) |= ((0x1U << (10U))));
}





 
static __inline void LL_FLASH_DisableDataCache(void)
{
  ((((FLASH_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3C00U))->ACR) &= ~((0x1U << (10U))));
}






 
static __inline void LL_FLASH_EnableInstCacheReset(void)
{
  ((((FLASH_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3C00U))->ACR) |= ((0x1U << (11U))));
}





 
static __inline void LL_FLASH_DisableInstCacheReset(void)
{
  ((((FLASH_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3C00U))->ACR) &= ~((0x1U << (11U))));
}






 
static __inline void LL_FLASH_EnableDataCacheReset(void)
{
  ((((FLASH_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3C00U))->ACR) |= ((0x1U << (12U))));
}





 
static __inline void LL_FLASH_DisableDataCacheReset(void)
{
  ((((FLASH_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3C00U))->ACR) &= ~((0x1U << (12U))));
}




 



 



 





 







 
#line 57 "../Inc/main.h"
#line 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_gpio.h"

































 

 







 
#line 46 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_gpio.h"



 





 

 
 
 
 



 



 


 



 



 
typedef struct
{
  uint32_t Pin;          
 

  uint32_t Mode;         


 

  uint32_t Speed;        


 

  uint32_t OutputType;   


 

  uint32_t Pull;         


 

  uint32_t Alternate;    


 
} LL_GPIO_InitTypeDef;



 


 


 



 
#line 146 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_gpio.h"


 



 






 



 




 



 






 



 





 



 
#line 210 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_gpio.h"


 



 

 


 



 







 







 



 



 

 


 



 






























 
static __inline void LL_GPIO_SetPinMode(GPIO_TypeDef *GPIOx, uint32_t Pin, uint32_t Mode)
{
  (((GPIOx->MODER)) = ((((((GPIOx->MODER))) & (~(((0x3U << (0U)) << ((__clz(__rbit(Pin))) * 2U))))) | ((Mode << ((__clz(__rbit(Pin))) * 2U))))));
}





























 
static __inline uint32_t LL_GPIO_GetPinMode(GPIO_TypeDef *GPIOx, uint32_t Pin)
{
  return (uint32_t)(((GPIOx->MODER) & (((0x3U << (0U)) << ((__clz(__rbit(Pin))) * 2U)))) >> ((__clz(__rbit(Pin))) * 2U));

}





























 
static __inline void LL_GPIO_SetPinOutputType(GPIO_TypeDef *GPIOx, uint32_t PinMask, uint32_t OutputType)
{
  (((GPIOx->OTYPER)) = ((((((GPIOx->OTYPER))) & (~(PinMask))) | ((PinMask * OutputType)))));
}





























 
static __inline uint32_t LL_GPIO_GetPinOutputType(GPIO_TypeDef *GPIOx, uint32_t Pin)
{
  return (uint32_t)(((GPIOx->OTYPER) & (Pin)) >> (__clz(__rbit(Pin))));
}
































 
static __inline void LL_GPIO_SetPinSpeed(GPIO_TypeDef *GPIOx, uint32_t Pin, uint32_t  Speed)
{
  (((GPIOx->OSPEEDR)) = ((((((GPIOx->OSPEEDR))) & (~(((0x3U << (0U)) << ((__clz(__rbit(Pin))) * 2U))))) | ((Speed << ((__clz(__rbit(Pin))) * 2U))))));

}































 
static __inline uint32_t LL_GPIO_GetPinSpeed(GPIO_TypeDef *GPIOx, uint32_t Pin)
{
  return (uint32_t)(((GPIOx->OSPEEDR) & (((0x3U << (0U)) << ((__clz(__rbit(Pin))) * 2U)))) >> ((__clz(__rbit(Pin))) * 2U));

}




























 
static __inline void LL_GPIO_SetPinPull(GPIO_TypeDef *GPIOx, uint32_t Pin, uint32_t Pull)
{
  (((GPIOx->PUPDR)) = ((((((GPIOx->PUPDR))) & (~(((0x3U << (0U)) << ((__clz(__rbit(Pin))) * 2U))))) | ((Pull << ((__clz(__rbit(Pin))) * 2U))))));
}



























 
static __inline uint32_t LL_GPIO_GetPinPull(GPIO_TypeDef *GPIOx, uint32_t Pin)
{
  return (uint32_t)(((GPIOx->PUPDR) & (((0x3U << (0U)) << ((__clz(__rbit(Pin))) * 2U)))) >> ((__clz(__rbit(Pin))) * 2U));

}


































 
static __inline void LL_GPIO_SetAFPin_0_7(GPIO_TypeDef *GPIOx, uint32_t Pin, uint32_t Alternate)
{
  (((GPIOx->AFR[0])) = ((((((GPIOx->AFR[0]))) & (~(((0xFU << (0U)) << ((__clz(__rbit(Pin))) * 4U))))) | ((Alternate << ((__clz(__rbit(Pin))) * 4U))))));

}































 
static __inline uint32_t LL_GPIO_GetAFPin_0_7(GPIO_TypeDef *GPIOx, uint32_t Pin)
{
  return (uint32_t)(((GPIOx->AFR[0]) & (((0xFU << (0U)) << ((__clz(__rbit(Pin))) * 4U)))) >> ((__clz(__rbit(Pin))) * 4U));

}


































 
static __inline void LL_GPIO_SetAFPin_8_15(GPIO_TypeDef *GPIOx, uint32_t Pin, uint32_t Alternate)
{
  (((GPIOx->AFR[1])) = ((((((GPIOx->AFR[1]))) & (~(((0xFU << (0U)) << ((__clz(__rbit(Pin >> 8U))) * 4U))))) | ((Alternate << ((__clz(__rbit(Pin >> 8U))) * 4U))))));

}
































 
static __inline uint32_t LL_GPIO_GetAFPin_8_15(GPIO_TypeDef *GPIOx, uint32_t Pin)
{
  return (uint32_t)(((GPIOx->AFR[1]) & (((0xFU << (0U)) << ((__clz(__rbit(Pin >> 8U))) * 4U)))) >> ((__clz(__rbit(Pin >> 8U))) * 4U));

}






























 
static __inline void LL_GPIO_LockPin(GPIO_TypeDef *GPIOx, uint32_t PinMask)
{
  volatile uint32_t temp;
  ((GPIOx->LCKR) = ((0x1U << (16U)) | PinMask));
  ((GPIOx->LCKR) = (PinMask));
  ((GPIOx->LCKR) = ((0x1U << (16U)) | PinMask));
  temp = ((GPIOx->LCKR));
  (void) temp;
}
























 
static __inline uint32_t LL_GPIO_IsPinLocked(GPIO_TypeDef *GPIOx, uint32_t PinMask)
{
  return (((GPIOx->LCKR) & (PinMask)) == (PinMask));
}






 
static __inline uint32_t LL_GPIO_IsAnyPinLocked(GPIO_TypeDef *GPIOx)
{
  return (((GPIOx->LCKR) & ((0x1U << (16U)))) == ((0x1U << (16U))));
}



 



 






 
static __inline uint32_t LL_GPIO_ReadInputPort(GPIO_TypeDef *GPIOx)
{
  return (uint32_t)(((GPIOx->IDR)));
}
























 
static __inline uint32_t LL_GPIO_IsInputPinSet(GPIO_TypeDef *GPIOx, uint32_t PinMask)
{
  return (((GPIOx->IDR) & (PinMask)) == (PinMask));
}







 
static __inline void LL_GPIO_WriteOutputPort(GPIO_TypeDef *GPIOx, uint32_t PortValue)
{
  ((GPIOx->ODR) = (PortValue));
}






 
static __inline uint32_t LL_GPIO_ReadOutputPort(GPIO_TypeDef *GPIOx)
{
  return (uint32_t)(((GPIOx->ODR)));
}
























 
static __inline uint32_t LL_GPIO_IsOutputPinSet(GPIO_TypeDef *GPIOx, uint32_t PinMask)
{
  return (((GPIOx->ODR) & (PinMask)) == (PinMask));
}
























 
static __inline void LL_GPIO_SetOutputPin(GPIO_TypeDef *GPIOx, uint32_t PinMask)
{
  ((GPIOx->BSRR) = (PinMask));
}
























 
static __inline void LL_GPIO_ResetOutputPin(GPIO_TypeDef *GPIOx, uint32_t PinMask)
{
  ((GPIOx->BSRR) = ((PinMask << 16)));
}
























 
static __inline void LL_GPIO_TogglePin(GPIO_TypeDef *GPIOx, uint32_t PinMask)
{
  ((GPIOx->ODR) = (((GPIOx->ODR)) ^ PinMask));
}



 




 

ErrorStatus LL_GPIO_DeInit(GPIO_TypeDef *GPIOx);
ErrorStatus LL_GPIO_Init(GPIO_TypeDef *GPIOx, LL_GPIO_InitTypeDef *GPIO_InitStruct);
void        LL_GPIO_StructInit(LL_GPIO_InitTypeDef *GPIO_InitStruct);



 




 



 




 







 
#line 58 "../Inc/main.h"
#line 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_exti.h"

































 

 







 
#line 46 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_exti.h"



 





 

 
 
 
 



 


 

 



 
typedef struct
{

  uint32_t Line_0_31;           
 

  FunctionalState LineCommand;  
 

  uint8_t Mode;                 
 

  uint8_t Trigger;              
 
} LL_EXTI_InitTypeDef;



 


 


 



 
#line 164 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_exti.h"










 




 





 



 







 







 

 


 



 






 






 



 




 



 


 


 




































 
static __inline void LL_EXTI_EnableIT_0_31(uint32_t ExtiLine)
{
  ((((EXTI_TypeDef *) ((0x40000000U + 0x00010000U) + 0x3C00U))->IMR) |= (ExtiLine));
}




































 
static __inline void LL_EXTI_DisableIT_0_31(uint32_t ExtiLine)
{
  ((((EXTI_TypeDef *) ((0x40000000U + 0x00010000U) + 0x3C00U))->IMR) &= ~(ExtiLine));
}





































 
static __inline uint32_t LL_EXTI_IsEnabledIT_0_31(uint32_t ExtiLine)
{
  return (((((EXTI_TypeDef *) ((0x40000000U + 0x00010000U) + 0x3C00U))->IMR) & (ExtiLine)) == (ExtiLine));
}




 



 

































 
static __inline void LL_EXTI_EnableEvent_0_31(uint32_t ExtiLine)
{
  ((((EXTI_TypeDef *) ((0x40000000U + 0x00010000U) + 0x3C00U))->EMR) |= (ExtiLine));

}


































 
static __inline void LL_EXTI_DisableEvent_0_31(uint32_t ExtiLine)
{
  ((((EXTI_TypeDef *) ((0x40000000U + 0x00010000U) + 0x3C00U))->EMR) &= ~(ExtiLine));
}


































 
static __inline uint32_t LL_EXTI_IsEnabledEvent_0_31(uint32_t ExtiLine)
{
  return (((((EXTI_TypeDef *) ((0x40000000U + 0x00010000U) + 0x3C00U))->EMR) & (ExtiLine)) == (ExtiLine));

}




 



 





































 
static __inline void LL_EXTI_EnableRisingTrig_0_31(uint32_t ExtiLine)
{
  ((((EXTI_TypeDef *) ((0x40000000U + 0x00010000U) + 0x3C00U))->RTSR) |= (ExtiLine));

}






































 
static __inline void LL_EXTI_DisableRisingTrig_0_31(uint32_t ExtiLine)
{
  ((((EXTI_TypeDef *) ((0x40000000U + 0x00010000U) + 0x3C00U))->RTSR) &= ~(ExtiLine));

}































 
static __inline uint32_t LL_EXTI_IsEnabledRisingTrig_0_31(uint32_t ExtiLine)
{
  return (((((EXTI_TypeDef *) ((0x40000000U + 0x00010000U) + 0x3C00U))->RTSR) & (ExtiLine)) == (ExtiLine));
}




 



 





































 
static __inline void LL_EXTI_EnableFallingTrig_0_31(uint32_t ExtiLine)
{
  ((((EXTI_TypeDef *) ((0x40000000U + 0x00010000U) + 0x3C00U))->FTSR) |= (ExtiLine));
}





































 
static __inline void LL_EXTI_DisableFallingTrig_0_31(uint32_t ExtiLine)
{
  ((((EXTI_TypeDef *) ((0x40000000U + 0x00010000U) + 0x3C00U))->FTSR) &= ~(ExtiLine));
}































 
static __inline uint32_t LL_EXTI_IsEnabledFallingTrig_0_31(uint32_t ExtiLine)
{
  return (((((EXTI_TypeDef *) ((0x40000000U + 0x00010000U) + 0x3C00U))->FTSR) & (ExtiLine)) == (ExtiLine));
}




 



 



































 
static __inline void LL_EXTI_GenerateSWI_0_31(uint32_t ExtiLine)
{
  ((((EXTI_TypeDef *) ((0x40000000U + 0x00010000U) + 0x3C00U))->SWIER) |= (ExtiLine));
}




 



 
































 
static __inline uint32_t LL_EXTI_IsActiveFlag_0_31(uint32_t ExtiLine)
{
  return (((((EXTI_TypeDef *) ((0x40000000U + 0x00010000U) + 0x3C00U))->PR) & (ExtiLine)) == (ExtiLine));
}

































 
static __inline uint32_t LL_EXTI_ReadFlag_0_31(uint32_t ExtiLine)
{
  return (uint32_t)(((((EXTI_TypeDef *) ((0x40000000U + 0x00010000U) + 0x3C00U))->PR) & (ExtiLine)));
}

































 
static __inline void LL_EXTI_ClearFlag_0_31(uint32_t ExtiLine)
{
  ((((EXTI_TypeDef *) ((0x40000000U + 0x00010000U) + 0x3C00U))->PR) = (ExtiLine));
}




 




 

uint32_t LL_EXTI_Init(LL_EXTI_InitTypeDef *EXTI_InitStruct);
uint32_t LL_EXTI_DeInit(void);
void LL_EXTI_StructInit(LL_EXTI_InitTypeDef *EXTI_InitStruct);




 




 



 





 







 
#line 59 "../Inc/main.h"
#line 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_bus.h"


















































 

 







 
#line 63 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_bus.h"



 





 

 
 
 
 
 
 


 



 
#line 148 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_bus.h"


 




 
#line 175 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_bus.h"


 





 
#line 194 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_bus.h"


 




 
#line 284 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_bus.h"


 



 
#line 354 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_bus.h"


 



 

 
 


 



 























































 
static __inline void LL_AHB1_GRP1_EnableClock(uint32_t Periphs)
{
  volatile uint32_t tmpreg;
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->AHB1ENR) |= (Periphs));
   
  tmpreg = ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->AHB1ENR) & (Periphs));
  (void)tmpreg;
}























































 
static __inline uint32_t LL_AHB1_GRP1_IsEnabledClock(uint32_t Periphs)
{
  return (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->AHB1ENR) & (Periphs)) == Periphs);
}























































 
static __inline void LL_AHB1_GRP1_DisableClock(uint32_t Periphs)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->AHB1ENR) &= ~(Periphs));
}












































 
static __inline void LL_AHB1_GRP1_ForceReset(uint32_t Periphs)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->AHB1RSTR) |= (Periphs));
}












































 
static __inline void LL_AHB1_GRP1_ReleaseReset(uint32_t Periphs)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->AHB1RSTR) &= ~(Periphs));
}






























































 
static __inline void LL_AHB1_GRP1_EnableClockLowPower(uint32_t Periphs)
{
  volatile uint32_t tmpreg;
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->AHB1LPENR) |= (Periphs));
   
  tmpreg = ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->AHB1LPENR) & (Periphs));
  (void)tmpreg;
}






























































 
static __inline void LL_AHB1_GRP1_DisableClockLowPower(uint32_t Periphs)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->AHB1LPENR) &= ~(Periphs));
}



 




 



















 
static __inline void LL_AHB2_GRP1_EnableClock(uint32_t Periphs)
{
  volatile uint32_t tmpreg;
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->AHB2ENR) |= (Periphs));
   
  tmpreg = ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->AHB2ENR) & (Periphs));
  (void)tmpreg;
}



















 
static __inline uint32_t LL_AHB2_GRP1_IsEnabledClock(uint32_t Periphs)
{
  return (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->AHB2ENR) & (Periphs)) == Periphs);
}



















 
static __inline void LL_AHB2_GRP1_DisableClock(uint32_t Periphs)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->AHB2ENR) &= ~(Periphs));
}




















 
static __inline void LL_AHB2_GRP1_ForceReset(uint32_t Periphs)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->AHB2RSTR) |= (Periphs));
}




















 
static __inline void LL_AHB2_GRP1_ReleaseReset(uint32_t Periphs)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->AHB2RSTR) &= ~(Periphs));
}



















 
static __inline void LL_AHB2_GRP1_EnableClockLowPower(uint32_t Periphs)
{
  volatile uint32_t tmpreg;
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->AHB2LPENR) |= (Periphs));
   
  tmpreg = ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->AHB2LPENR) & (Periphs));
  (void)tmpreg;
}



















 
static __inline void LL_AHB2_GRP1_DisableClockLowPower(uint32_t Periphs)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->AHB2LPENR) &= ~(Periphs));
}



 





 













 
static __inline void LL_AHB3_GRP1_EnableClock(uint32_t Periphs)
{
  volatile uint32_t tmpreg;
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->AHB3ENR) |= (Periphs));
   
  tmpreg = ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->AHB3ENR) & (Periphs));
  (void)tmpreg;
}













 
static __inline uint32_t LL_AHB3_GRP1_IsEnabledClock(uint32_t Periphs)
{
  return (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->AHB3ENR) & (Periphs)) == Periphs);
}













 
static __inline void LL_AHB3_GRP1_DisableClock(uint32_t Periphs)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->AHB3ENR) &= ~(Periphs));
}














 
static __inline void LL_AHB3_GRP1_ForceReset(uint32_t Periphs)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->AHB3RSTR) |= (Periphs));
}














 
static __inline void LL_AHB3_GRP1_ReleaseReset(uint32_t Periphs)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->AHB3RSTR) &= ~(Periphs));
}













 
static __inline void LL_AHB3_GRP1_EnableClockLowPower(uint32_t Periphs)
{
  volatile uint32_t tmpreg;
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->AHB3LPENR) |= (Periphs));
   
  tmpreg = ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->AHB3LPENR) & (Periphs));
  (void)tmpreg;
}













 
static __inline void LL_AHB3_GRP1_DisableClockLowPower(uint32_t Periphs)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->AHB3LPENR) &= ~(Periphs));
}



 




 





































































 
static __inline void LL_APB1_GRP1_EnableClock(uint32_t Periphs)
{
  volatile uint32_t tmpreg;
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->APB1ENR) |= (Periphs));
   
  tmpreg = ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->APB1ENR) & (Periphs));
  (void)tmpreg;
}





































































 
static __inline uint32_t LL_APB1_GRP1_IsEnabledClock(uint32_t Periphs)
{
  return (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->APB1ENR) & (Periphs)) == Periphs);
}





































































 
static __inline void LL_APB1_GRP1_DisableClock(uint32_t Periphs)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->APB1ENR) &= ~(Periphs));
}



































































 
static __inline void LL_APB1_GRP1_ForceReset(uint32_t Periphs)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->APB1RSTR) |= (Periphs));
}



































































 
static __inline void LL_APB1_GRP1_ReleaseReset(uint32_t Periphs)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->APB1RSTR) &= ~(Periphs));
}





































































 
static __inline void LL_APB1_GRP1_EnableClockLowPower(uint32_t Periphs)
{
  volatile uint32_t tmpreg;
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->APB1LPENR) |= (Periphs));
   
  tmpreg = ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->APB1LPENR) & (Periphs));
  (void)tmpreg;
}





































































 
static __inline void LL_APB1_GRP1_DisableClockLowPower(uint32_t Periphs)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->APB1LPENR) &= ~(Periphs));
}



 



 


























































 
static __inline void LL_APB2_GRP1_EnableClock(uint32_t Periphs)
{
  volatile uint32_t tmpreg;
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->APB2ENR) |= (Periphs));
   
  tmpreg = ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->APB2ENR) & (Periphs));
  (void)tmpreg;
}

























































 
static __inline uint32_t LL_APB2_GRP1_IsEnabledClock(uint32_t Periphs)
{
  return (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->APB2ENR) & (Periphs)) == Periphs);
}

























































 
static __inline void LL_APB2_GRP1_DisableClock(uint32_t Periphs)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->APB2ENR) &= ~(Periphs));
}




















































 
static __inline void LL_APB2_GRP1_ForceReset(uint32_t Periphs)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->APB2RSTR) |= (Periphs));
}





















































 
static __inline void LL_APB2_GRP1_ReleaseReset(uint32_t Periphs)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->APB2RSTR) &= ~(Periphs));
}


























































 
static __inline void LL_APB2_GRP1_EnableClockLowPower(uint32_t Periphs)
{
  volatile uint32_t tmpreg;
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->APB2LPENR) |= (Periphs));
   
  tmpreg = ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->APB2LPENR) & (Periphs));
  (void)tmpreg;
}


























































 
static __inline void LL_APB2_GRP1_DisableClockLowPower(uint32_t Periphs)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->APB2LPENR) &= ~(Periphs));
}



 



 



 





 







 
#line 60 "../Inc/main.h"
#line 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_cortex.h"

















































 

 







 
#line 62 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_cortex.h"



 



 

 
 

 

 

 
 


 



 




 



 





 





 






 



 
#line 127 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_cortex.h"


 



 
#line 162 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_cortex.h"


 



 
#line 175 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_cortex.h"


 



 






 



 




 



 




 



 




 



 




 



 

 

 


 



 






 
static __inline uint32_t LL_SYSTICK_IsActiveCounterFlag(void)
{
  return ((((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL & (1UL << 16U)) == ((1UL << 16U)));
}








 
static __inline void LL_SYSTICK_SetClkSource(uint32_t Source)
{
  if (Source == (1UL << 2U))
  {
    ((((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL) |= ((1UL << 2U)));
  }
  else
  {
    ((((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL) &= ~((1UL << 2U)));
  }
}







 
static __inline uint32_t LL_SYSTICK_GetClkSource(void)
{
  return ((((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL) & ((1UL << 2U)));
}





 
static __inline void LL_SYSTICK_EnableIT(void)
{
  ((((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL) |= ((1UL << 1U)));
}





 
static __inline void LL_SYSTICK_DisableIT(void)
{
  ((((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL) &= ~((1UL << 1U)));
}





 
static __inline uint32_t LL_SYSTICK_IsEnabledIT(void)
{
  return (((((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL) & ((1UL << 1U))) == ((1UL << 1U)));
}



 



 





 
static __inline void LL_LPM_EnableSleep(void)
{
   
  ((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SCR) &= ~(((uint32_t)(1UL << 2U))));
}





 
static __inline void LL_LPM_EnableDeepSleep(void)
{
   
  ((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SCR) |= (((uint32_t)(1UL << 2U))));
}







 
static __inline void LL_LPM_EnableSleepOnExit(void)
{
   
  ((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SCR) |= (((uint32_t)(1UL << 1U))));
}





 
static __inline void LL_LPM_DisableSleepOnExit(void)
{
   
  ((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SCR) &= ~(((uint32_t)(1UL << 1U))));
}






 
static __inline void LL_LPM_EnableEventOnPend(void)
{
   
  ((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SCR) |= (((uint32_t)(1UL << 4U))));
}






 
static __inline void LL_LPM_DisableEventOnPend(void)
{
   
  ((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SCR) &= ~(((uint32_t)(1UL << 4U))));
}



 



 









 
static __inline void LL_HANDLER_EnableFault(uint32_t Fault)
{
   
  ((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHCSR) |= (Fault));
}









 
static __inline void LL_HANDLER_DisableFault(uint32_t Fault)
{
   
  ((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHCSR) &= ~(Fault));
}



 



 





 
static __inline uint32_t LL_CPUID_GetImplementer(void)
{
  return (uint32_t)(((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->CPUID) & ((0xFFUL << 24U))) >> 24U);
}





 
static __inline uint32_t LL_CPUID_GetVariant(void)
{
  return (uint32_t)(((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->CPUID) & ((0xFUL << 20U))) >> 20U);
}





 
static __inline uint32_t LL_CPUID_GetConstant(void)
{
  return (uint32_t)(((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->CPUID) & ((0xFUL << 16U))) >> 16U);
}





 
static __inline uint32_t LL_CPUID_GetParNo(void)
{
  return (uint32_t)(((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->CPUID) & ((0xFFFUL << 4U))) >> 4U);
}





 
static __inline uint32_t LL_CPUID_GetRevision(void)
{
  return (uint32_t)(((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->CPUID) & ((0xFUL ))) >> 0U);
}



 




 










 
static __inline void LL_MPU_Enable(uint32_t Options)
{
   
  ((((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->CTRL) = (((1UL ) | Options)));
   
  do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);
   
  do { __schedule_barrier(); __isb(0xF); __schedule_barrier(); } while (0U);
}





 
static __inline void LL_MPU_Disable(void)
{
   
  do { __schedule_barrier(); __dmb(0xF); __schedule_barrier(); } while (0U);
   
  ((((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->CTRL) = (0U));
}





 
static __inline uint32_t LL_MPU_IsEnabled(void)
{
  return (((((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->CTRL) & ((1UL ))) == ((1UL )));
}














 
static __inline void LL_MPU_EnableRegion(uint32_t Region)
{
   
  ((((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RNR) = (Region));
   
  ((((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RASR) |= ((1UL )));
}






































 
static __inline void LL_MPU_ConfigRegion(uint32_t Region, uint32_t SubRegionDisable, uint32_t Address, uint32_t Attributes)
{
   
  ((((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RNR) = (Region));
   
  ((((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RBAR) = ((Address & 0xFFFFFFE0U)));
   
  ((((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RASR) = (((1UL ) | Attributes | SubRegionDisable << 8U)));
}















 
static __inline void LL_MPU_DisableRegion(uint32_t Region)
{
   
  ((((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RNR) = (Region));
   
  ((((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->RASR) &= ~((1UL )));
}



 




 



 



 







 
#line 61 "../Inc/main.h"
#line 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"

































 

 







 
#line 46 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"



 





 

 
 


 







 
 
 



 


 

 



 



 



 
typedef struct
{
  uint32_t SYSCLK_Frequency;         
  uint32_t HCLK_Frequency;           
  uint32_t PCLK1_Frequency;          
  uint32_t PCLK2_Frequency;          
} LL_RCC_ClocksTypeDef;



 



 


 


 






 





















 




 
#line 160 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"


 




 
#line 189 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"


 




 
#line 208 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"


 



 
#line 221 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"


 



 
#line 234 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"


 



 
#line 250 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"


 



 







 



 







 



 
#line 291 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"


 



 
#line 310 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"


 



 
#line 348 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"


 




 




 


#line 374 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"

#line 387 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"

#line 432 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"

#line 447 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"

#line 458 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"

#line 469 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"



 
#line 494 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"


 

#line 552 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"

#line 580 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"

#line 590 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"

#line 601 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"

#line 611 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"

#line 632 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"




 
#line 644 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"


 


#line 663 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"




 







 





 







 


#line 701 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"



 
#line 717 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"


 

#line 744 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"

#line 754 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"

#line 764 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"

#line 774 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"




 






 

#line 797 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"



 







 



 
#line 875 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"


 

#line 893 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"

#line 933 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"



 






 



 
#line 962 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"


 



 




 




 
#line 1106 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"


 

#line 1132 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"

#line 1173 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"

#line 1213 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"



 
#line 1223 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"


 

#line 1240 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"

#line 1479 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"


 

 


 



 






 






 



 



 














































































 



#line 1678 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"
























































































 



#line 1852 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"

#line 1935 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"

#line 2018 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"

#line 2142 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"

#line 2439 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"

#line 2613 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"

#line 2695 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"
















































































 



#line 2871 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"















 












 












 




 



 

 


 



 





 
static __inline void LL_RCC_HSE_EnableCSS(void)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CR) |= ((0x1U << (19U))));
}





 
static __inline void LL_RCC_HSE_EnableBypass(void)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CR) |= ((0x1U << (18U))));
}





 
static __inline void LL_RCC_HSE_DisableBypass(void)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CR) &= ~((0x1U << (18U))));
}





 
static __inline void LL_RCC_HSE_Enable(void)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CR) |= ((0x1U << (16U))));
}





 
static __inline void LL_RCC_HSE_Disable(void)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CR) &= ~((0x1U << (16U))));
}





 
static __inline uint32_t LL_RCC_HSE_IsReady(void)
{
  return (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CR) & ((0x1U << (17U)))) == ((0x1U << (17U))));
}



 



 





 
static __inline void LL_RCC_HSI_Enable(void)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CR) |= ((0x1U << (0U))));
}





 
static __inline void LL_RCC_HSI_Disable(void)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CR) &= ~((0x1U << (0U))));
}





 
static __inline uint32_t LL_RCC_HSI_IsReady(void)
{
  return (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CR) & ((0x1U << (1U)))) == ((0x1U << (1U))));
}







 
static __inline uint32_t LL_RCC_HSI_GetCalibration(void)
{
  return (uint32_t)(((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CR) & ((0xFFU << (8U)))) >> (8U));
}









 
static __inline void LL_RCC_HSI_SetCalibTrimming(uint32_t Value)
{
  (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CR)) = ((((((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CR))) & (~((0x1FU << (3U))))) | (Value << (3U)))));
}





 
static __inline uint32_t LL_RCC_HSI_GetCalibTrimming(void)
{
  return (uint32_t)(((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CR) & ((0x1FU << (3U)))) >> (3U));
}



 



 





 
static __inline void LL_RCC_LSE_Enable(void)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->BDCR) |= ((0x1U << (0U))));
}





 
static __inline void LL_RCC_LSE_Disable(void)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->BDCR) &= ~((0x1U << (0U))));
}





 
static __inline void LL_RCC_LSE_EnableBypass(void)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->BDCR) |= ((0x1U << (2U))));
}





 
static __inline void LL_RCC_LSE_DisableBypass(void)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->BDCR) &= ~((0x1U << (2U))));
}





 
static __inline uint32_t LL_RCC_LSE_IsReady(void)
{
  return (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->BDCR) & ((0x1U << (1U)))) == ((0x1U << (1U))));
}

#line 3147 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"



 



 





 
static __inline void LL_RCC_LSI_Enable(void)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CSR) |= ((0x1U << (0U))));
}





 
static __inline void LL_RCC_LSI_Disable(void)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CSR) &= ~((0x1U << (0U))));
}





 
static __inline uint32_t LL_RCC_LSI_IsReady(void)
{
  return (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CSR) & ((0x1U << (1U)))) == ((0x1U << (1U))));
}



 



 












 
static __inline void LL_RCC_SetSysClkSource(uint32_t Source)
{
  (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CFGR)) = ((((((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CFGR))) & (~((0x3U << (0U))))) | (Source))));
}











 
static __inline uint32_t LL_RCC_GetSysClkSource(void)
{
  return (uint32_t)(((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CFGR) & ((0x3U << (2U)))));
}















 
static __inline void LL_RCC_SetAHBPrescaler(uint32_t Prescaler)
{
  (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CFGR)) = ((((((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CFGR))) & (~((0xFU << (4U))))) | (Prescaler))));
}











 
static __inline void LL_RCC_SetAPB1Prescaler(uint32_t Prescaler)
{
  (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CFGR)) = ((((((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CFGR))) & (~((0x7U << (10U))))) | (Prescaler))));
}











 
static __inline void LL_RCC_SetAPB2Prescaler(uint32_t Prescaler)
{
  (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CFGR)) = ((((((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CFGR))) & (~((0x7U << (13U))))) | (Prescaler))));
}














 
static __inline uint32_t LL_RCC_GetAHBPrescaler(void)
{
  return (uint32_t)(((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CFGR) & ((0xFU << (4U)))));
}










 
static __inline uint32_t LL_RCC_GetAPB1Prescaler(void)
{
  return (uint32_t)(((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CFGR) & ((0x7U << (10U)))));
}










 
static __inline uint32_t LL_RCC_GetAPB2Prescaler(void)
{
  return (uint32_t)(((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CFGR) & ((0x7U << (13U)))));
}



 



 

#line 3357 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"

#line 3379 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"




























 
static __inline void LL_RCC_ConfigMCO(uint32_t MCOxSource, uint32_t MCOxPrescaler)
{
  (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CFGR)) = ((((((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CFGR))) & (~((MCOxSource & 0xFFFF0000U) | (MCOxPrescaler & 0xFFFF0000U)))) | ((MCOxSource << 16U) | (MCOxPrescaler << 16U)))));
}



 



 
#line 3435 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"

#line 3452 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"

#line 3488 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"

#line 3508 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"

#line 3577 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"

#line 3592 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"



















 
static __inline void LL_RCC_SetI2SClockSource(uint32_t Source)
{

  (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CFGR)) = ((((((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CFGR))) & (~((0x1U << (23U))))) | (Source))));



}

#line 3635 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"

#line 3672 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"

#line 3689 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"

#line 3706 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"

#line 3724 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"

#line 3766 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"

#line 3787 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"

#line 3859 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"

#line 3875 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"





















 
static __inline uint32_t LL_RCC_GetI2SClockSource(uint32_t I2Sx)
{

  return (uint32_t)(((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CFGR) & (I2Sx)));



}

#line 3946 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"

#line 3964 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"

#line 3980 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"



 



 













 
static __inline void LL_RCC_SetRTCClockSource(uint32_t Source)
{
  (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->BDCR)) = ((((((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->BDCR))) & (~((0x3U << (8U))))) | (Source))));
}









 
static __inline uint32_t LL_RCC_GetRTCClockSource(void)
{
  return (uint32_t)(((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->BDCR) & ((0x3U << (8U)))));
}





 
static __inline void LL_RCC_EnableRTC(void)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->BDCR) |= ((0x1U << (15U))));
}





 
static __inline void LL_RCC_DisableRTC(void)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->BDCR) &= ~((0x1U << (15U))));
}





 
static __inline uint32_t LL_RCC_IsEnabledRTC(void)
{
  return (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->BDCR) & ((0x1U << (15U)))) == ((0x1U << (15U))));
}





 
static __inline void LL_RCC_ForceBackupDomainReset(void)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->BDCR) |= ((0x1U << (16U))));
}





 
static __inline void LL_RCC_ReleaseBackupDomainReset(void)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->BDCR) &= ~((0x1U << (16U))));
}





































 
static __inline void LL_RCC_SetRTC_HSEPrescaler(uint32_t Prescaler)
{
  (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CFGR)) = ((((((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CFGR))) & (~((0x1FU << (16U))))) | (Prescaler))));
}




































 
static __inline uint32_t LL_RCC_GetRTC_HSEPrescaler(void)
{
  return (uint32_t)(((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CFGR) & ((0x1FU << (16U)))));
}



 

#line 4192 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"



 





 
static __inline void LL_RCC_PLL_Enable(void)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CR) |= ((0x1U << (24U))));
}






 
static __inline void LL_RCC_PLL_Disable(void)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CR) &= ~((0x1U << (24U))));
}





 
static __inline uint32_t LL_RCC_PLL_IsReady(void)
{
  return (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CR) & ((0x1U << (25U)))) == ((0x1U << (25U))));
}






























































































 
static __inline void LL_RCC_PLL_ConfigDomain_SYS(uint32_t Source, uint32_t PLLM, uint32_t PLLN, uint32_t PLLP_R)
{
  (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->PLLCFGR)) = ((((((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->PLLCFGR))) & (~((0x1U << (22U)) | (0x3FU << (0U)) | (0x1FFU << (6U))))) | (Source | PLLM | PLLN << (6U)))));

  (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->PLLCFGR)) = ((((((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->PLLCFGR))) & (~((0x3U << (16U))))) | (PLLP_R))));



}
































































































 
static __inline void LL_RCC_PLL_ConfigDomain_48M(uint32_t Source, uint32_t PLLM, uint32_t PLLN, uint32_t PLLQ)
{
  (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->PLLCFGR)) = ((((((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->PLLCFGR))) & (~((0x1U << (22U)) | (0x3FU << (0U)) | (0x1FFU << (6U)) | (0xFU << (24U))))) | (Source | PLLM | PLLN << (6U) | PLLQ))));

}

#line 4527 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"

#line 4621 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"

#line 4715 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"

#line 4853 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"








 
static __inline void LL_RCC_PLL_SetMainSource(uint32_t PLLSource)
{
  (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->PLLCFGR)) = ((((((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->PLLCFGR))) & (~((0x1U << (22U))))) | (PLLSource))));
}







 
static __inline uint32_t LL_RCC_PLL_GetMainSource(void)
{
  return (uint32_t)(((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->PLLCFGR) & ((0x1U << (22U)))));
}







 
static __inline uint32_t LL_RCC_PLL_GetN(void)
{
  return (uint32_t)(((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->PLLCFGR) & ((0x1FFU << (6U)))) >>  (6U));
}









 
static __inline uint32_t LL_RCC_PLL_GetP(void)
{
  return (uint32_t)(((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->PLLCFGR) & ((0x3U << (16U)))));
}




















 
static __inline uint32_t LL_RCC_PLL_GetQ(void)
{
  return (uint32_t)(((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->PLLCFGR) & ((0xFU << (24U)))));
}

#line 4948 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"

#line 4992 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"



































































 
static __inline uint32_t LL_RCC_PLL_GetDivider(void)
{
  return (uint32_t)(((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->PLLCFGR) & ((0x3FU << (0U)))));
}













 
static __inline void LL_RCC_PLL_ConfigSpreadSpectrum(uint32_t Mod, uint32_t Inc, uint32_t Sel)
{
  (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->SSCGR)) = ((((((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->SSCGR))) & (~((0x1FFFU << (0U)) | (0x7FFFU << (13U)) | (0x1U << (30U))))) | (Mod | (Inc << (13U)) | Sel))));
}





 
static __inline uint32_t LL_RCC_PLL_GetPeriodModulation(void)
{
  return (uint32_t)(((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->SSCGR) & ((0x1FFFU << (0U)))));
}






 
static __inline uint32_t LL_RCC_PLL_GetStepIncrementation(void)
{
  return (uint32_t)(((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->SSCGR) & ((0x7FFFU << (13U)))) >> (13U));
}








 
static __inline uint32_t LL_RCC_PLL_GetSpreadSelection(void)
{
  return (uint32_t)(((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->SSCGR) & ((0x1U << (30U)))));
}





 
static __inline void LL_RCC_PLL_SpreadSpectrum_Enable(void)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->SSCGR) |= ((0x1U << (31U))));
}





 
static __inline void LL_RCC_PLL_SpreadSpectrum_Disable(void)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->SSCGR) &= ~((0x1U << (31U))));
}



 




 





 
static __inline void LL_RCC_PLLI2S_Enable(void)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CR) |= ((0x1U << (26U))));
}





 
static __inline void LL_RCC_PLLI2S_Disable(void)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CR) &= ~((0x1U << (26U))));
}





 
static __inline uint32_t LL_RCC_PLLI2S_IsReady(void)
{
  return (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CR) & ((0x1U << (27U)))) == ((0x1U << (27U))));
}

#line 5374 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"

#line 5487 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"

#line 5585 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"





























































































 
static __inline void LL_RCC_PLLI2S_ConfigDomain_I2S(uint32_t Source, uint32_t PLLM, uint32_t PLLN, uint32_t PLLR)
{
  register uint32_t *pReg = (uint32_t *)((uint32_t)((uint32_t)(&((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->PLLCFGR) + (Source & 0x80U)));
  (((*pReg)) = ((((((*pReg))) & (~((0x1U << (22U))))) | ((Source & (~0x80U))))));



  (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->PLLCFGR)) = ((((((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->PLLCFGR))) & (~((0x3FU << (0U))))) | (PLLM))));

  (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->PLLI2SCFGR)) = ((((((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->PLLI2SCFGR))) & (~((0x1FFU << (6U)) | (0x7U << (28U))))) | (PLLN << (6U) | PLLR))));
}







 
static __inline uint32_t LL_RCC_PLLI2S_GetN(void)
{
  return (uint32_t)(((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->PLLI2SCFGR) & ((0x1FFU << (6U)))) >> (6U));
}

#line 5728 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"












 
static __inline uint32_t LL_RCC_PLLI2S_GetR(void)
{
  return (uint32_t)(((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->PLLI2SCFGR) & ((0x7U << (28U)))));
}

#line 5762 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"

#line 5807 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"

#line 5851 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"




































































 
static __inline uint32_t LL_RCC_PLLI2S_GetDivider(void)
{



  return (uint32_t)(((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->PLLCFGR) & ((0x3FU << (0U)))));

}











 
static __inline uint32_t LL_RCC_PLLI2S_GetMainSource(void)
{
#line 5948 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"
  return (uint32_t)(((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->PLLCFGR) & ((0x1U << (22U)))));

}



 


#line 6549 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"



 





 
static __inline void LL_RCC_ClearFlag_LSIRDY(void)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CIR) |= ((0x1U << (16U))));
}





 
static __inline void LL_RCC_ClearFlag_LSERDY(void)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CIR) |= ((0x1U << (17U))));
}





 
static __inline void LL_RCC_ClearFlag_HSIRDY(void)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CIR) |= ((0x1U << (18U))));
}





 
static __inline void LL_RCC_ClearFlag_HSERDY(void)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CIR) |= ((0x1U << (19U))));
}





 
static __inline void LL_RCC_ClearFlag_PLLRDY(void)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CIR) |= ((0x1U << (20U))));
}






 
static __inline void LL_RCC_ClearFlag_PLLI2SRDY(void)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CIR) |= ((0x1U << (21U))));
}



#line 6629 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"





 
static __inline void LL_RCC_ClearFlag_HSECSS(void)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CIR) |= ((0x1U << (23U))));
}





 
static __inline uint32_t LL_RCC_IsActiveFlag_LSIRDY(void)
{
  return (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CIR) & ((0x1U << (0U)))) == ((0x1U << (0U))));
}





 
static __inline uint32_t LL_RCC_IsActiveFlag_LSERDY(void)
{
  return (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CIR) & ((0x1U << (1U)))) == ((0x1U << (1U))));
}





 
static __inline uint32_t LL_RCC_IsActiveFlag_HSIRDY(void)
{
  return (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CIR) & ((0x1U << (2U)))) == ((0x1U << (2U))));
}





 
static __inline uint32_t LL_RCC_IsActiveFlag_HSERDY(void)
{
  return (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CIR) & ((0x1U << (3U)))) == ((0x1U << (3U))));
}





 
static __inline uint32_t LL_RCC_IsActiveFlag_PLLRDY(void)
{
  return (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CIR) & ((0x1U << (4U)))) == ((0x1U << (4U))));
}






 
static __inline uint32_t LL_RCC_IsActiveFlag_PLLI2SRDY(void)
{
  return (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CIR) & ((0x1U << (5U)))) == ((0x1U << (5U))));
}


#line 6713 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"





 
static __inline uint32_t LL_RCC_IsActiveFlag_HSECSS(void)
{
  return (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CIR) & ((0x1U << (7U)))) == ((0x1U << (7U))));
}





 
static __inline uint32_t LL_RCC_IsActiveFlag_IWDGRST(void)
{
  return (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CSR) & ((0x1U << (29U)))) == ((0x1U << (29U))));
}





 
static __inline uint32_t LL_RCC_IsActiveFlag_LPWRRST(void)
{
  return (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CSR) & ((0x1U << (31U)))) == ((0x1U << (31U))));
}





 
static __inline uint32_t LL_RCC_IsActiveFlag_PINRST(void)
{
  return (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CSR) & ((0x1U << (26U)))) == ((0x1U << (26U))));
}





 
static __inline uint32_t LL_RCC_IsActiveFlag_PORRST(void)
{
  return (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CSR) & ((0x1U << (27U)))) == ((0x1U << (27U))));
}





 
static __inline uint32_t LL_RCC_IsActiveFlag_SFTRST(void)
{
  return (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CSR) & ((0x1U << (28U)))) == ((0x1U << (28U))));
}





 
static __inline uint32_t LL_RCC_IsActiveFlag_WWDGRST(void)
{
  return (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CSR) & ((0x1U << (30U)))) == ((0x1U << (30U))));
}






 
static __inline uint32_t LL_RCC_IsActiveFlag_BORRST(void)
{
  return (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CSR) & ((0x1U << (25U)))) == ((0x1U << (25U))));
}






 
static __inline void LL_RCC_ClearResetFlags(void)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CSR) |= ((0x1U << (24U))));
}



 



 





 
static __inline void LL_RCC_EnableIT_LSIRDY(void)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CIR) |= ((0x1U << (8U))));
}





 
static __inline void LL_RCC_EnableIT_LSERDY(void)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CIR) |= ((0x1U << (9U))));
}





 
static __inline void LL_RCC_EnableIT_HSIRDY(void)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CIR) |= ((0x1U << (10U))));
}





 
static __inline void LL_RCC_EnableIT_HSERDY(void)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CIR) |= ((0x1U << (11U))));
}





 
static __inline void LL_RCC_EnableIT_PLLRDY(void)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CIR) |= ((0x1U << (12U))));
}






 
static __inline void LL_RCC_EnableIT_PLLI2SRDY(void)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CIR) |= ((0x1U << (13U))));
}


#line 6887 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"





 
static __inline void LL_RCC_DisableIT_LSIRDY(void)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CIR) &= ~((0x1U << (8U))));
}





 
static __inline void LL_RCC_DisableIT_LSERDY(void)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CIR) &= ~((0x1U << (9U))));
}





 
static __inline void LL_RCC_DisableIT_HSIRDY(void)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CIR) &= ~((0x1U << (10U))));
}





 
static __inline void LL_RCC_DisableIT_HSERDY(void)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CIR) &= ~((0x1U << (11U))));
}





 
static __inline void LL_RCC_DisableIT_PLLRDY(void)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CIR) &= ~((0x1U << (12U))));
}






 
static __inline void LL_RCC_DisableIT_PLLI2SRDY(void)
{
  ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CIR) &= ~((0x1U << (13U))));
}



#line 6962 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"





 
static __inline uint32_t LL_RCC_IsEnabledIT_LSIRDY(void)
{
  return (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CIR) & ((0x1U << (8U)))) == ((0x1U << (8U))));
}





 
static __inline uint32_t LL_RCC_IsEnabledIT_LSERDY(void)
{
  return (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CIR) & ((0x1U << (9U)))) == ((0x1U << (9U))));
}





 
static __inline uint32_t LL_RCC_IsEnabledIT_HSIRDY(void)
{
  return (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CIR) & ((0x1U << (10U)))) == ((0x1U << (10U))));
}





 
static __inline uint32_t LL_RCC_IsEnabledIT_HSERDY(void)
{
  return (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CIR) & ((0x1U << (11U)))) == ((0x1U << (11U))));
}





 
static __inline uint32_t LL_RCC_IsEnabledIT_PLLRDY(void)
{
  return (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CIR) & ((0x1U << (12U)))) == ((0x1U << (12U))));
}






 
static __inline uint32_t LL_RCC_IsEnabledIT_PLLI2SRDY(void)
{
  return (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CIR) & ((0x1U << (13U)))) == ((0x1U << (13U))));
}



#line 7037 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"



 




 
ErrorStatus LL_RCC_DeInit(void);


 



 
void        LL_RCC_GetSystemClocksFreq(LL_RCC_ClocksTypeDef *RCC_Clocks);
#line 7065 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"
uint32_t    LL_RCC_GetSDIOClockFreq(uint32_t SDIOxSource);


uint32_t    LL_RCC_GetRNGClockFreq(uint32_t RNGxSource);


uint32_t    LL_RCC_GetUSBClockFreq(uint32_t USBxSource);





uint32_t    LL_RCC_GetI2SClockFreq(uint32_t I2SxSource);
#line 7090 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_rcc.h"


 




 



 





 







 
#line 62 "../Inc/main.h"
#line 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_utils.h"













































 

 







 
#line 58 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_utils.h"



 



 

 
 

 


 

 




 




 




 




 

 


 


 
 


 


 
typedef struct
{
  uint32_t PLLM;   



 

  uint32_t PLLN;   




 

  uint32_t PLLP;   



 
} LL_UTILS_PLLInitTypeDef;



 
typedef struct
{
  uint32_t AHBCLKDivider;         



 

  uint32_t APB1CLKDivider;        



 

  uint32_t APB2CLKDivider;        



 

} LL_UTILS_ClkInitTypeDef;



 

 


 



 




 



 
#line 186 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_utils.h"


 



 

 

 


 



 




 
static __inline uint32_t LL_GetUID_Word0(void)
{
  return (uint32_t)(((*((uint32_t *)0x1FFF7A10U))));
}




 
static __inline uint32_t LL_GetUID_Word1(void)
{
  return (uint32_t)(((*((uint32_t *)(0x1FFF7A10U + 4U)))));
}




 
static __inline uint32_t LL_GetUID_Word2(void)
{
  return (uint32_t)(((*((uint32_t *)(0x1FFF7A10U + 8U)))));
}






 
static __inline uint32_t LL_GetFlashSize(void)
{
  return (uint16_t)(((*((uint32_t *)0x1FFF7A22U))));
}













 
static __inline uint32_t LL_GetPackageType(void)
{
  return (uint8_t)(((*((uint32_t *)0x1FFF7BF0U))) & 0x0700U);
}



 



 








 
static __inline void LL_InitTick(uint32_t HCLKFrequency, uint32_t Ticks)
{
   
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD  = (uint32_t)((HCLKFrequency / Ticks) - 1UL);   
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL   = 0UL;                                        
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL  = (1UL << 2U) |
                   (1UL );                    
}

void        LL_Init1msTick(uint32_t HCLKFrequency);
void        LL_mDelay(uint32_t Delay);



 



 

void        LL_SetSystemCoreClock(uint32_t HCLKFrequency);
ErrorStatus LL_PLL_ConfigSystemClock_HSI(LL_UTILS_PLLInitTypeDef *UTILS_PLLInitStruct,
                                         LL_UTILS_ClkInitTypeDef *UTILS_ClkInitStruct);
ErrorStatus LL_PLL_ConfigSystemClock_HSE(uint32_t HSEFrequency, uint32_t HSEBypass,
                                         LL_UTILS_PLLInitTypeDef *UTILS_PLLInitStruct, LL_UTILS_ClkInitTypeDef *UTILS_ClkInitStruct);



 



 



 



 







 
#line 63 "../Inc/main.h"
#line 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_pwr.h"

































 

 







 
#line 46 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_pwr.h"



 





 

 
 
 
 
 
 


 




 




 




 
#line 96 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_pwr.h"


 



 
#line 111 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_pwr.h"


 



 
#line 129 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_pwr.h"


 



 




 



 
#line 153 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_pwr.h"


 


 
#line 171 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_pwr.h"


 



 


 


 



 






 






 



 



 

 


 



 
#line 251 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_pwr.h"

#line 323 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_pwr.h"

#line 417 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_pwr.h"

#line 449 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_pwr.h"

#line 481 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_pwr.h"

#line 513 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_pwr.h"









 
static __inline void LL_PWR_SetRegulVoltageScaling(uint32_t VoltageScaling)
{
  (((((PWR_TypeDef *) (0x40000000U + 0x7000U))->CR)) = ((((((((PWR_TypeDef *) (0x40000000U + 0x7000U))->CR))) & (~((0x1U << (14U))))) | (VoltageScaling))));
}









 
static __inline uint32_t LL_PWR_GetRegulVoltageScaling(void)
{
  return (uint32_t)(((((PWR_TypeDef *) (0x40000000U + 0x7000U))->CR) & ((0x1U << (14U)))));
}




 
static __inline void LL_PWR_EnableFlashPowerDown(void)
{
  ((((PWR_TypeDef *) (0x40000000U + 0x7000U))->CR) |= ((0x1U << (9U))));
}





 
static __inline void LL_PWR_DisableFlashPowerDown(void)
{
  ((((PWR_TypeDef *) (0x40000000U + 0x7000U))->CR) &= ~((0x1U << (9U))));
}





 
static __inline uint32_t LL_PWR_IsEnabledFlashPowerDown(void)
{
  return (((((PWR_TypeDef *) (0x40000000U + 0x7000U))->CR) & ((0x1U << (9U)))) == ((0x1U << (9U))));
}





 
static __inline void LL_PWR_EnableBkUpAccess(void)
{
  ((((PWR_TypeDef *) (0x40000000U + 0x7000U))->CR) |= ((0x1U << (8U))));
}





 
static __inline void LL_PWR_DisableBkUpAccess(void)
{
  ((((PWR_TypeDef *) (0x40000000U + 0x7000U))->CR) &= ~((0x1U << (8U))));
}





 
static __inline uint32_t LL_PWR_IsEnabledBkUpAccess(void)
{
  return (((((PWR_TypeDef *) (0x40000000U + 0x7000U))->CR) & ((0x1U << (8U)))) == ((0x1U << (8U))));
}






 
static __inline void LL_PWR_EnableBkUpRegulator(void)
{
  ((((PWR_TypeDef *) (0x40000000U + 0x7000U))->CSR) |= ((0x1U << (9U))));
}







 
static __inline void LL_PWR_DisableBkUpRegulator(void)
{
  ((((PWR_TypeDef *) (0x40000000U + 0x7000U))->CSR) &= ~((0x1U << (9U))));
}





 
static __inline uint32_t LL_PWR_IsEnabledBkUpRegulator(void)
{
  return (((((PWR_TypeDef *) (0x40000000U + 0x7000U))->CSR) & ((0x1U << (9U)))) == ((0x1U << (9U))));
}








 
static __inline void LL_PWR_SetRegulModeDS(uint32_t RegulMode)
{
  (((((PWR_TypeDef *) (0x40000000U + 0x7000U))->CR)) = ((((((((PWR_TypeDef *) (0x40000000U + 0x7000U))->CR))) & (~((0x1U << (0U))))) | (RegulMode))));
}







 
static __inline uint32_t LL_PWR_GetRegulModeDS(void)
{
  return (uint32_t)(((((PWR_TypeDef *) (0x40000000U + 0x7000U))->CR) & ((0x1U << (0U)))));
}






















 
static __inline void LL_PWR_SetPowerMode(uint32_t PDMode)
{





  (((((PWR_TypeDef *) (0x40000000U + 0x7000U))->CR)) = ((((((((PWR_TypeDef *) (0x40000000U + 0x7000U))->CR))) & (~(((0x1U << (1U))| (0x1U << (0U)))))) | (PDMode))));

}





















 
static __inline uint32_t LL_PWR_GetPowerMode(void)
{





  return (uint32_t)(((((PWR_TypeDef *) (0x40000000U + 0x7000U))->CR) & (((0x1U << (1U))| (0x1U << (0U))))));

}














 
static __inline void LL_PWR_SetPVDLevel(uint32_t PVDLevel)
{
  (((((PWR_TypeDef *) (0x40000000U + 0x7000U))->CR)) = ((((((((PWR_TypeDef *) (0x40000000U + 0x7000U))->CR))) & (~((0x7U << (5U))))) | (PVDLevel))));
}













 
static __inline uint32_t LL_PWR_GetPVDLevel(void)
{
  return (uint32_t)(((((PWR_TypeDef *) (0x40000000U + 0x7000U))->CR) & ((0x7U << (5U)))));
}





 
static __inline void LL_PWR_EnablePVD(void)
{
  ((((PWR_TypeDef *) (0x40000000U + 0x7000U))->CR) |= ((0x1U << (4U))));
}





 
static __inline void LL_PWR_DisablePVD(void)
{
  ((((PWR_TypeDef *) (0x40000000U + 0x7000U))->CR) &= ~((0x1U << (4U))));
}





 
static __inline uint32_t LL_PWR_IsEnabledPVD(void)
{
  return (((((PWR_TypeDef *) (0x40000000U + 0x7000U))->CR) & ((0x1U << (4U)))) == ((0x1U << (4U))));
}














 
static __inline void LL_PWR_EnableWakeUpPin(uint32_t WakeUpPin)
{
  ((((PWR_TypeDef *) (0x40000000U + 0x7000U))->CSR) |= (WakeUpPin));
}














 
static __inline void LL_PWR_DisableWakeUpPin(uint32_t WakeUpPin)
{
  ((((PWR_TypeDef *) (0x40000000U + 0x7000U))->CSR) &= ~(WakeUpPin));
}














 
static __inline uint32_t LL_PWR_IsEnabledWakeUpPin(uint32_t WakeUpPin)
{
  return (((((PWR_TypeDef *) (0x40000000U + 0x7000U))->CSR) & (WakeUpPin)) == (WakeUpPin));
}




 



 





 
static __inline uint32_t LL_PWR_IsActiveFlag_WU(void)
{
  return (((((PWR_TypeDef *) (0x40000000U + 0x7000U))->CSR) & ((0x1U << (0U)))) == ((0x1U << (0U))));
}





 
static __inline uint32_t LL_PWR_IsActiveFlag_SB(void)
{
  return (((((PWR_TypeDef *) (0x40000000U + 0x7000U))->CSR) & ((0x1U << (1U)))) == ((0x1U << (1U))));
}





 
static __inline uint32_t LL_PWR_IsActiveFlag_BRR(void)
{
  return (((((PWR_TypeDef *) (0x40000000U + 0x7000U))->CSR) & ((0x1U << (3U)))) == ((0x1U << (3U))));
}




 
static __inline uint32_t LL_PWR_IsActiveFlag_PVDO(void)
{
  return (((((PWR_TypeDef *) (0x40000000U + 0x7000U))->CSR) & ((0x1U << (2U)))) == ((0x1U << (2U))));
}





 
static __inline uint32_t LL_PWR_IsActiveFlag_VOS(void)
{
  return (((((PWR_TypeDef *) (0x40000000U + 0x7000U))->CSR) & ((0x1U << (14U)))) == ((0x1U << (14U))));
}
#line 916 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_pwr.h"

#line 928 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_pwr.h"

#line 940 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_pwr.h"




 
static __inline void LL_PWR_ClearFlag_SB(void)
{
  ((((PWR_TypeDef *) (0x40000000U + 0x7000U))->CR) |= ((0x1U << (3U))));
}





 
static __inline void LL_PWR_ClearFlag_WU(void)
{
  ((((PWR_TypeDef *) (0x40000000U + 0x7000U))->CR) |= ((0x1U << (2U))));
}
#line 970 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_pwr.h"



 




 
ErrorStatus LL_PWR_DeInit(void);


 




 



 





 







 
#line 64 "../Inc/main.h"
#line 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_dma.h"

































 

 







 
#line 46 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_dma.h"



 





 

 
 


 
 
static const uint8_t STREAM_OFFSET_TAB[] =
{
  (uint8_t)((((0x40000000U + 0x00020000U) + 0x6000U) + 0x010U) - ((0x40000000U + 0x00020000U) + 0x6000U)),
  (uint8_t)((((0x40000000U + 0x00020000U) + 0x6000U) + 0x028U) - ((0x40000000U + 0x00020000U) + 0x6000U)),
  (uint8_t)((((0x40000000U + 0x00020000U) + 0x6000U) + 0x040U) - ((0x40000000U + 0x00020000U) + 0x6000U)),
  (uint8_t)((((0x40000000U + 0x00020000U) + 0x6000U) + 0x058U) - ((0x40000000U + 0x00020000U) + 0x6000U)),
  (uint8_t)((((0x40000000U + 0x00020000U) + 0x6000U) + 0x070U) - ((0x40000000U + 0x00020000U) + 0x6000U)),
  (uint8_t)((((0x40000000U + 0x00020000U) + 0x6000U) + 0x088U) - ((0x40000000U + 0x00020000U) + 0x6000U)),
  (uint8_t)((((0x40000000U + 0x00020000U) + 0x6000U) + 0x0A0U) - ((0x40000000U + 0x00020000U) + 0x6000U)),
  (uint8_t)((((0x40000000U + 0x00020000U) + 0x6000U) + 0x0B8U) - ((0x40000000U + 0x00020000U) + 0x6000U))
};



 

 


 


 


 
 



 
typedef struct
{
  uint32_t PeriphOrM2MSrcAddress;  


 

  uint32_t MemoryOrM2MDstAddress;  


 

  uint32_t Direction;              



 

  uint32_t Mode;                   




 

  uint32_t PeriphOrM2MSrcIncMode;  



 

  uint32_t MemoryOrM2MDstIncMode;  



 

  uint32_t PeriphOrM2MSrcDataSize; 



 

  uint32_t MemoryOrM2MDstDataSize; 



 

  uint32_t NbData;                 




 

  uint32_t Channel;                


 

  uint32_t Priority;               


 
                                        
  uint32_t FIFOMode;               




 

  uint32_t FIFOThreshold;          


 

  uint32_t MemBurst;               





 

  uint32_t PeriphBurst;            





 

} LL_DMA_InitTypeDef;


 

 


 



 
#line 210 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_dma.h"


 



 





 



 





 



 




 



 




 



 




 



 





 



 





 



 




 



 






 



 
#line 312 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_dma.h"


 



 






 



 






 
  


 




   



 
#line 356 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_dma.h"


 



 






 
    


 




 



 

 


 



 






 







 



 



 




 







 
#line 444 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_dma.h"






 
#line 468 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_dma.h"



 



 


 
 

 



 














 
static __inline void LL_DMA_EnableStream(DMA_TypeDef *DMAx, uint32_t Stream)
{
  ((((DMA_Stream_TypeDef *)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CR) |= ((0x1U << (0U))));
}















 
static __inline void LL_DMA_DisableStream(DMA_TypeDef *DMAx, uint32_t Stream)
{
  ((((DMA_Stream_TypeDef *)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CR) &= ~((0x1U << (0U))));
}















 
static __inline uint32_t LL_DMA_IsEnabledStream(DMA_TypeDef *DMAx, uint32_t Stream)
{
  return (((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CR) & ((0x1U << (0U)))) == ((0x1U << (0U))));
}






























 
static __inline void LL_DMA_ConfigTransfer(DMA_TypeDef *DMAx, uint32_t Stream, uint32_t Configuration)
{
  (((((DMA_Stream_TypeDef *)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CR)) = ((((((((DMA_Stream_TypeDef *)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CR))) & (~((0x3U << (6U)) | (0x1U << (8U)) | (0x1U << (9U)) | (0x1U << (10U)) | (0x3U << (11U)) | (0x3U << (13U)) | (0x3U << (16U)) | (0x1U << (5U))))) | (Configuration))));


}



















 
static __inline void LL_DMA_SetDataTransferDirection(DMA_TypeDef *DMAx, uint32_t Stream, uint32_t  Direction)
{
  (((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CR)) = ((((((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CR))) & (~((0x3U << (6U))))) | (Direction))));
}


















 
static __inline uint32_t LL_DMA_GetDataTransferDirection(DMA_TypeDef *DMAx, uint32_t Stream)
{
  return (((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CR) & ((0x3U << (6U)))));
}




















 
static __inline void LL_DMA_SetMode(DMA_TypeDef *DMAx, uint32_t Stream, uint32_t Mode)
{
  (((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CR)) = ((((((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CR))) & (~((0x1U << (8U)) | (0x1U << (5U))))) | (Mode))));
}



















 
static __inline uint32_t LL_DMA_GetMode(DMA_TypeDef *DMAx, uint32_t Stream)
{
  return (((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CR) & ((0x1U << (8U)) | (0x1U << (5U)))));
}


















 
static __inline void LL_DMA_SetPeriphIncMode(DMA_TypeDef *DMAx, uint32_t Stream, uint32_t IncrementMode)
{
  (((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CR)) = ((((((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CR))) & (~((0x1U << (9U))))) | (IncrementMode))));
}

















 
static __inline uint32_t LL_DMA_GetPeriphIncMode(DMA_TypeDef *DMAx, uint32_t Stream)
{
  return (((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CR) & ((0x1U << (9U)))));
}


















 
static __inline void LL_DMA_SetMemoryIncMode(DMA_TypeDef *DMAx, uint32_t Stream, uint32_t IncrementMode)
{
  (((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CR)) = ((((((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CR))) & (~((0x1U << (10U))))) | (IncrementMode))));
}

















 
static __inline uint32_t LL_DMA_GetMemoryIncMode(DMA_TypeDef *DMAx, uint32_t Stream)
{
  return (((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CR) & ((0x1U << (10U)))));
}



















 
static __inline void LL_DMA_SetPeriphSize(DMA_TypeDef *DMAx, uint32_t Stream, uint32_t  Size)
{
  (((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CR)) = ((((((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CR))) & (~((0x3U << (11U))))) | (Size))));
}


















 
static __inline uint32_t LL_DMA_GetPeriphSize(DMA_TypeDef *DMAx, uint32_t Stream)
{
  return (((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CR) & ((0x3U << (11U)))));
}



















 
static __inline void LL_DMA_SetMemorySize(DMA_TypeDef *DMAx, uint32_t Stream, uint32_t  Size)
{
  (((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CR)) = ((((((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CR))) & (~((0x3U << (13U))))) | (Size))));
}


















 
static __inline uint32_t LL_DMA_GetMemorySize(DMA_TypeDef *DMAx, uint32_t Stream)
{
  return (((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CR) & ((0x3U << (13U)))));
}


















 
static __inline void LL_DMA_SetIncOffsetSize(DMA_TypeDef *DMAx, uint32_t Stream, uint32_t OffsetSize)
{
  (((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CR)) = ((((((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CR))) & (~((0x1U << (15U))))) | (OffsetSize))));
}

















 
static __inline uint32_t LL_DMA_GetIncOffsetSize(DMA_TypeDef *DMAx, uint32_t Stream)
{
  return (((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CR) & ((0x1U << (15U)))));
}




















 
static __inline void LL_DMA_SetStreamPriorityLevel(DMA_TypeDef *DMAx, uint32_t Stream, uint32_t  Priority)
{
  (((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CR)) = ((((((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CR))) & (~((0x3U << (16U))))) | (Priority))));
}



















 
static __inline uint32_t LL_DMA_GetStreamPriorityLevel(DMA_TypeDef *DMAx, uint32_t Stream)
{
  return (((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CR) & ((0x3U << (16U)))));
}


















 
static __inline void LL_DMA_SetDataLength(DMA_TypeDef* DMAx, uint32_t Stream, uint32_t NbData)
{
  (((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->NDTR)) = ((((((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->NDTR))) & (~((0xFFFFU << (0U))))) | (NbData))));
}

















 
static __inline uint32_t LL_DMA_GetDataLength(DMA_TypeDef* DMAx, uint32_t Stream)
{
  return (((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->NDTR) & ((0xFFFFU << (0U)))));
}
























 
static __inline void LL_DMA_SetChannelSelection(DMA_TypeDef *DMAx, uint32_t Stream, uint32_t Channel)
{
  (((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CR)) = ((((((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CR))) & (~((0x7U << (25U))))) | (Channel))));
}























 
static __inline uint32_t LL_DMA_GetChannelSelection(DMA_TypeDef *DMAx, uint32_t Stream)
{
  return (((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CR) & ((0x7U << (25U)))));
}




















 
static __inline void LL_DMA_SetMemoryBurstxfer(DMA_TypeDef *DMAx, uint32_t Stream, uint32_t Mburst)
{
  (((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CR)) = ((((((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CR))) & (~((0x3U << (23U))))) | (Mburst))));
}



















 
static __inline uint32_t LL_DMA_GetMemoryBurstxfer(DMA_TypeDef *DMAx, uint32_t Stream)
{
  return (((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CR) & ((0x3U << (23U)))));
}




















 
static __inline void LL_DMA_SetPeriphBurstxfer(DMA_TypeDef *DMAx, uint32_t Stream, uint32_t Pburst)
{
  (((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CR)) = ((((((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CR))) & (~((0x3U << (21U))))) | (Pburst))));
}



















 
static __inline uint32_t LL_DMA_GetPeriphBurstxfer(DMA_TypeDef *DMAx, uint32_t Stream)
{
  return (((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CR) & ((0x3U << (21U)))));
}


















 
static __inline void LL_DMA_SetCurrentTargetMem(DMA_TypeDef *DMAx, uint32_t Stream, uint32_t CurrentMemory)
{
   (((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CR)) = ((((((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CR))) & (~((0x1U << (19U))))) | (CurrentMemory))));
}

















 
static __inline uint32_t LL_DMA_GetCurrentTargetMem(DMA_TypeDef *DMAx, uint32_t Stream)
{
  return (((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CR) & ((0x1U << (19U)))));
}















 
static __inline void LL_DMA_EnableDoubleBufferMode(DMA_TypeDef *DMAx, uint32_t Stream)
{
  ((((DMA_Stream_TypeDef *)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CR) |= ((0x1U << (18U))));
}















 
static __inline void LL_DMA_DisableDoubleBufferMode(DMA_TypeDef *DMAx, uint32_t Stream)
{
  ((((DMA_Stream_TypeDef *)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CR) &= ~((0x1U << (18U))));
}





















 
static __inline uint32_t LL_DMA_GetFIFOStatus(DMA_TypeDef *DMAx, uint32_t Stream)
{
  return (((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->FCR) & ((0x7U << (3U)))));
}















 
static __inline void LL_DMA_DisableFifoMode(DMA_TypeDef *DMAx, uint32_t Stream)
{
  ((((DMA_Stream_TypeDef *)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->FCR) &= ~((0x1U << (2U))));
}















 
static __inline void LL_DMA_EnableFifoMode(DMA_TypeDef *DMAx, uint32_t Stream)
{
  ((((DMA_Stream_TypeDef *)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->FCR) |= ((0x1U << (2U))));
}




















 
static __inline void LL_DMA_SetFIFOThreshold(DMA_TypeDef *DMAx, uint32_t Stream, uint32_t Threshold)
{
  (((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->FCR)) = ((((((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->FCR))) & (~((0x3U << (0U))))) | (Threshold))));
}



















 
static __inline uint32_t LL_DMA_GetFIFOThreshold(DMA_TypeDef *DMAx, uint32_t Stream)
{
  return (((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->FCR) & ((0x3U << (0U)))));
}
























 
static __inline void LL_DMA_ConfigFifo(DMA_TypeDef *DMAx, uint32_t Stream, uint32_t FifoMode, uint32_t FifoThreshold)
{
  (((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->FCR)) = ((((((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->FCR))) & (~((0x3U << (0U))|(0x1U << (2U))))) | (FifoMode|FifoThreshold))));
}























 
static __inline void LL_DMA_ConfigAddresses(DMA_TypeDef* DMAx, uint32_t Stream, uint32_t SrcAddress, uint32_t DstAddress, uint32_t Direction)
{
   
  if (Direction == (0x1U << (6U)))
  {
    ((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->M0AR) = (SrcAddress));
    ((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->PAR) = (DstAddress));
  }
   
  else
  {
    ((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->PAR) = (SrcAddress));
    ((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->M0AR) = (DstAddress));
  }
}


















 
static __inline void LL_DMA_SetMemoryAddress(DMA_TypeDef* DMAx, uint32_t Stream, uint32_t MemoryAddress)
{
  ((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->M0AR) = (MemoryAddress));
}


















 
static __inline void LL_DMA_SetPeriphAddress(DMA_TypeDef* DMAx, uint32_t Stream, uint32_t PeriphAddress)
{
  ((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->PAR) = (PeriphAddress));
}
















 
static __inline uint32_t LL_DMA_GetMemoryAddress(DMA_TypeDef* DMAx, uint32_t Stream)
{
  return (((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->M0AR)));
}
















 
static __inline uint32_t LL_DMA_GetPeriphAddress(DMA_TypeDef* DMAx, uint32_t Stream)
{
  return (((((DMA_Stream_TypeDef *)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->PAR)));
}


















 
static __inline void LL_DMA_SetM2MSrcAddress(DMA_TypeDef* DMAx, uint32_t Stream, uint32_t MemoryAddress)
{
  ((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->PAR) = (MemoryAddress));
}


















 
static __inline void LL_DMA_SetM2MDstAddress(DMA_TypeDef* DMAx, uint32_t Stream, uint32_t MemoryAddress)
  {
    ((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->M0AR) = (MemoryAddress));
  }
















 
static __inline uint32_t LL_DMA_GetM2MSrcAddress(DMA_TypeDef* DMAx, uint32_t Stream)
  {
   return (((((DMA_Stream_TypeDef *)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->PAR)));
  }
















 
static __inline uint32_t LL_DMA_GetM2MDstAddress(DMA_TypeDef* DMAx, uint32_t Stream)
{
 return (((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->M0AR)));
}
















 
static __inline void LL_DMA_SetMemory1Address(DMA_TypeDef *DMAx, uint32_t Stream, uint32_t Address)
{
  (((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->M1AR)) = ((((((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->M1AR))) & (~((0xFFFFFFFFU << (0U))))) | (Address))));
}















 
static __inline uint32_t LL_DMA_GetMemory1Address(DMA_TypeDef *DMAx, uint32_t Stream)
{
  return (((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->M1AR);
}



 



 






 
static __inline uint32_t LL_DMA_IsActiveFlag_HT0(DMA_TypeDef *DMAx)
{
  return (((DMAx->LISR) & ((0x1U << (4U))))==((0x1U << (4U))));
}






 
static __inline uint32_t LL_DMA_IsActiveFlag_HT1(DMA_TypeDef *DMAx)
{
  return (((DMAx->LISR) & ((0x1U << (10U))))==((0x1U << (10U))));
}






 
static __inline uint32_t LL_DMA_IsActiveFlag_HT2(DMA_TypeDef *DMAx)
{
  return (((DMAx->LISR) & ((0x1U << (20U))))==((0x1U << (20U))));
}






 
static __inline uint32_t LL_DMA_IsActiveFlag_HT3(DMA_TypeDef *DMAx)
{
  return (((DMAx->LISR) & ((0x1U << (26U))))==((0x1U << (26U))));
}






 
static __inline uint32_t LL_DMA_IsActiveFlag_HT4(DMA_TypeDef *DMAx)
{
  return (((DMAx->HISR) & ((0x1U << (4U))))==((0x1U << (4U))));
}






 
static __inline uint32_t LL_DMA_IsActiveFlag_HT5(DMA_TypeDef *DMAx)
{
  return (((DMAx->HISR) & ((0x1U << (10U))))==((0x1U << (10U))));
}






 
static __inline uint32_t LL_DMA_IsActiveFlag_HT6(DMA_TypeDef *DMAx)
{
  return (((DMAx->HISR) & ((0x1U << (20U))))==((0x1U << (20U))));
}






 
static __inline uint32_t LL_DMA_IsActiveFlag_HT7(DMA_TypeDef *DMAx)
{
  return (((DMAx->HISR) & ((0x1U << (26U))))==((0x1U << (26U))));
} 






 
static __inline uint32_t LL_DMA_IsActiveFlag_TC0(DMA_TypeDef *DMAx)
{
  return (((DMAx->LISR) & ((0x1U << (5U))))==((0x1U << (5U))));
}






 
static __inline uint32_t LL_DMA_IsActiveFlag_TC1(DMA_TypeDef *DMAx)
{
  return (((DMAx->LISR) & ((0x1U << (11U))))==((0x1U << (11U))));
}






 
static __inline uint32_t LL_DMA_IsActiveFlag_TC2(DMA_TypeDef *DMAx)
{
  return (((DMAx->LISR) & ((0x1U << (21U))))==((0x1U << (21U))));
}






 
static __inline uint32_t LL_DMA_IsActiveFlag_TC3(DMA_TypeDef *DMAx)
{
  return (((DMAx->LISR) & ((0x1U << (27U))))==((0x1U << (27U))));
}






 
static __inline uint32_t LL_DMA_IsActiveFlag_TC4(DMA_TypeDef *DMAx)
{
  return (((DMAx->HISR) & ((0x1U << (5U))))==((0x1U << (5U))));
}






 
static __inline uint32_t LL_DMA_IsActiveFlag_TC5(DMA_TypeDef *DMAx)
{
  return (((DMAx->HISR) & ((0x1U << (11U))))==((0x1U << (11U))));
}






 
static __inline uint32_t LL_DMA_IsActiveFlag_TC6(DMA_TypeDef *DMAx)
{
  return (((DMAx->HISR) & ((0x1U << (21U))))==((0x1U << (21U))));
}






 
static __inline uint32_t LL_DMA_IsActiveFlag_TC7(DMA_TypeDef *DMAx)
{
  return (((DMAx->HISR) & ((0x1U << (27U))))==((0x1U << (27U))));
} 






 
static __inline uint32_t LL_DMA_IsActiveFlag_TE0(DMA_TypeDef *DMAx)
{
  return (((DMAx->LISR) & ((0x1U << (3U))))==((0x1U << (3U))));
}






 
static __inline uint32_t LL_DMA_IsActiveFlag_TE1(DMA_TypeDef *DMAx)
{
  return (((DMAx->LISR) & ((0x1U << (9U))))==((0x1U << (9U))));
}






 
static __inline uint32_t LL_DMA_IsActiveFlag_TE2(DMA_TypeDef *DMAx)
{
  return (((DMAx->LISR) & ((0x1U << (19U))))==((0x1U << (19U))));
}






 
static __inline uint32_t LL_DMA_IsActiveFlag_TE3(DMA_TypeDef *DMAx)
{
  return (((DMAx->LISR) & ((0x1U << (25U))))==((0x1U << (25U))));
}






 
static __inline uint32_t LL_DMA_IsActiveFlag_TE4(DMA_TypeDef *DMAx)
{
  return (((DMAx->HISR) & ((0x1U << (3U))))==((0x1U << (3U))));
}






 
static __inline uint32_t LL_DMA_IsActiveFlag_TE5(DMA_TypeDef *DMAx)
{
  return (((DMAx->HISR) & ((0x1U << (9U))))==((0x1U << (9U))));
}






 
static __inline uint32_t LL_DMA_IsActiveFlag_TE6(DMA_TypeDef *DMAx)
{
  return (((DMAx->HISR) & ((0x1U << (19U))))==((0x1U << (19U))));
}






 
static __inline uint32_t LL_DMA_IsActiveFlag_TE7(DMA_TypeDef *DMAx)
{
  return (((DMAx->HISR) & ((0x1U << (25U))))==((0x1U << (25U))));
} 






 
static __inline uint32_t LL_DMA_IsActiveFlag_DME0(DMA_TypeDef *DMAx)
{
  return (((DMAx->LISR) & ((0x1U << (2U))))==((0x1U << (2U))));
}






 
static __inline uint32_t LL_DMA_IsActiveFlag_DME1(DMA_TypeDef *DMAx)
{
  return (((DMAx->LISR) & ((0x1U << (8U))))==((0x1U << (8U))));
}






 
static __inline uint32_t LL_DMA_IsActiveFlag_DME2(DMA_TypeDef *DMAx)
{
  return (((DMAx->LISR) & ((0x1U << (18U))))==((0x1U << (18U))));
}






 
static __inline uint32_t LL_DMA_IsActiveFlag_DME3(DMA_TypeDef *DMAx)
{
  return (((DMAx->LISR) & ((0x1U << (24U))))==((0x1U << (24U))));
}






 
static __inline uint32_t LL_DMA_IsActiveFlag_DME4(DMA_TypeDef *DMAx)
{
  return (((DMAx->HISR) & ((0x1U << (2U))))==((0x1U << (2U))));
}






 
static __inline uint32_t LL_DMA_IsActiveFlag_DME5(DMA_TypeDef *DMAx)
{
  return (((DMAx->HISR) & ((0x1U << (8U))))==((0x1U << (8U))));
}






 
static __inline uint32_t LL_DMA_IsActiveFlag_DME6(DMA_TypeDef *DMAx)
{
  return (((DMAx->HISR) & ((0x1U << (18U))))==((0x1U << (18U))));
}






 
static __inline uint32_t LL_DMA_IsActiveFlag_DME7(DMA_TypeDef *DMAx)
{
  return (((DMAx->HISR) & ((0x1U << (24U))))==((0x1U << (24U))));
}






 
static __inline uint32_t LL_DMA_IsActiveFlag_FE0(DMA_TypeDef *DMAx)
{
  return (((DMAx->LISR) & ((0x1U << (0U))))==((0x1U << (0U))));
}






 
static __inline uint32_t LL_DMA_IsActiveFlag_FE1(DMA_TypeDef *DMAx)
{
  return (((DMAx->LISR) & ((0x1U << (6U))))==((0x1U << (6U))));
}






 
static __inline uint32_t LL_DMA_IsActiveFlag_FE2(DMA_TypeDef *DMAx)
{
  return (((DMAx->LISR) & ((0x1U << (16U))))==((0x1U << (16U))));
}






 
static __inline uint32_t LL_DMA_IsActiveFlag_FE3(DMA_TypeDef *DMAx)
{
  return (((DMAx->LISR) & ((0x1U << (22U))))==((0x1U << (22U))));
}






 
static __inline uint32_t LL_DMA_IsActiveFlag_FE4(DMA_TypeDef *DMAx)
{
  return (((DMAx->HISR) & ((0x1U << (0U))))==((0x1U << (0U))));
}






 
static __inline uint32_t LL_DMA_IsActiveFlag_FE5(DMA_TypeDef *DMAx)
{
  return (((DMAx->HISR) & ((0x1U << (6U))))==((0x1U << (6U))));
}






 
static __inline uint32_t LL_DMA_IsActiveFlag_FE6(DMA_TypeDef *DMAx)
{
  return (((DMAx->HISR) & ((0x1U << (16U))))==((0x1U << (16U))));
}






 
static __inline uint32_t LL_DMA_IsActiveFlag_FE7(DMA_TypeDef *DMAx)
{
  return (((DMAx->HISR) & ((0x1U << (22U))))==((0x1U << (22U))));
}






 
static __inline void LL_DMA_ClearFlag_HT0(DMA_TypeDef *DMAx)
{
  ((DMAx->LIFCR) = ((0x1U << (4U))));
}






 
static __inline void LL_DMA_ClearFlag_HT1(DMA_TypeDef *DMAx)
{
  ((DMAx->LIFCR) = ((0x1U << (10U))));
}






 
static __inline void LL_DMA_ClearFlag_HT2(DMA_TypeDef *DMAx)
{
  ((DMAx->LIFCR) = ((0x1U << (20U))));
}






 
static __inline void LL_DMA_ClearFlag_HT3(DMA_TypeDef *DMAx)
{
  ((DMAx->LIFCR) = ((0x1U << (26U))));
}






 
static __inline void LL_DMA_ClearFlag_HT4(DMA_TypeDef *DMAx)
{
  ((DMAx->HIFCR) = ((0x1U << (4U))));
}






 
static __inline void LL_DMA_ClearFlag_HT5(DMA_TypeDef *DMAx)
{
  ((DMAx->HIFCR) = ((0x1U << (10U))));
}






 
static __inline void LL_DMA_ClearFlag_HT6(DMA_TypeDef *DMAx)
{
  ((DMAx->HIFCR) = ((0x1U << (20U))));
}






 
static __inline void LL_DMA_ClearFlag_HT7(DMA_TypeDef *DMAx)
{
  ((DMAx->HIFCR) = ((0x1U << (26U))));
}






 
static __inline void LL_DMA_ClearFlag_TC0(DMA_TypeDef *DMAx)
{
  ((DMAx->LIFCR) = ((0x1U << (5U))));
}






 
static __inline void LL_DMA_ClearFlag_TC1(DMA_TypeDef *DMAx)
{
  ((DMAx->LIFCR) = ((0x1U << (11U))));
}






 
static __inline void LL_DMA_ClearFlag_TC2(DMA_TypeDef *DMAx)
{
  ((DMAx->LIFCR) = ((0x1U << (21U))));
}






 
static __inline void LL_DMA_ClearFlag_TC3(DMA_TypeDef *DMAx)
{
  ((DMAx->LIFCR) = ((0x1U << (27U))));
}






 
static __inline void LL_DMA_ClearFlag_TC4(DMA_TypeDef *DMAx)
{
  ((DMAx->HIFCR) = ((0x1U << (5U))));
}






 
static __inline void LL_DMA_ClearFlag_TC5(DMA_TypeDef *DMAx)
{
  ((DMAx->HIFCR) = ((0x1U << (11U))));
}






 
static __inline void LL_DMA_ClearFlag_TC6(DMA_TypeDef *DMAx)
{
  ((DMAx->HIFCR) = ((0x1U << (21U))));
}






 
static __inline void LL_DMA_ClearFlag_TC7(DMA_TypeDef *DMAx)
{
  ((DMAx->HIFCR) = ((0x1U << (27U))));
}






 
static __inline void LL_DMA_ClearFlag_TE0(DMA_TypeDef *DMAx)
{
  ((DMAx->LIFCR) = ((0x1U << (3U))));
}






 
static __inline void LL_DMA_ClearFlag_TE1(DMA_TypeDef *DMAx)
{
  ((DMAx->LIFCR) = ((0x1U << (9U))));
}






 
static __inline void LL_DMA_ClearFlag_TE2(DMA_TypeDef *DMAx)
{
  ((DMAx->LIFCR) = ((0x1U << (19U))));
}






 
static __inline void LL_DMA_ClearFlag_TE3(DMA_TypeDef *DMAx)
{
  ((DMAx->LIFCR) = ((0x1U << (25U))));
}






 
static __inline void LL_DMA_ClearFlag_TE4(DMA_TypeDef *DMAx)
{
  ((DMAx->HIFCR) = ((0x1U << (3U))));
}






 
static __inline void LL_DMA_ClearFlag_TE5(DMA_TypeDef *DMAx)
{
  ((DMAx->HIFCR) = ((0x1U << (9U))));
}






 
static __inline void LL_DMA_ClearFlag_TE6(DMA_TypeDef *DMAx)
{
  ((DMAx->HIFCR) = ((0x1U << (19U))));
}






 
static __inline void LL_DMA_ClearFlag_TE7(DMA_TypeDef *DMAx)
{
  ((DMAx->HIFCR) = ((0x1U << (25U))));
}






 
static __inline void LL_DMA_ClearFlag_DME0(DMA_TypeDef *DMAx)
{
  ((DMAx->LIFCR) = ((0x1U << (2U))));
}






 
static __inline void LL_DMA_ClearFlag_DME1(DMA_TypeDef *DMAx)
{
  ((DMAx->LIFCR) = ((0x1U << (8U))));
}






 
static __inline void LL_DMA_ClearFlag_DME2(DMA_TypeDef *DMAx)
{
  ((DMAx->LIFCR) = ((0x1U << (18U))));
}






 
static __inline void LL_DMA_ClearFlag_DME3(DMA_TypeDef *DMAx)
{
  ((DMAx->LIFCR) = ((0x1U << (24U))));
}






 
static __inline void LL_DMA_ClearFlag_DME4(DMA_TypeDef *DMAx)
{
  ((DMAx->HIFCR) = ((0x1U << (2U))));
}






 
static __inline void LL_DMA_ClearFlag_DME5(DMA_TypeDef *DMAx)
{
  ((DMAx->HIFCR) = ((0x1U << (8U))));
}






 
static __inline void LL_DMA_ClearFlag_DME6(DMA_TypeDef *DMAx)
{
  ((DMAx->HIFCR) = ((0x1U << (18U))));
}






 
static __inline void LL_DMA_ClearFlag_DME7(DMA_TypeDef *DMAx)
{
  ((DMAx->HIFCR) = ((0x1U << (24U))));
}






 
static __inline void LL_DMA_ClearFlag_FE0(DMA_TypeDef *DMAx)
{
  ((DMAx->LIFCR) = ((0x1U << (0U))));
}






 
static __inline void LL_DMA_ClearFlag_FE1(DMA_TypeDef *DMAx)
{
  ((DMAx->LIFCR) = ((0x1U << (6U))));
}






 
static __inline void LL_DMA_ClearFlag_FE2(DMA_TypeDef *DMAx)
{
  ((DMAx->LIFCR) = ((0x1U << (16U))));
}






 
static __inline void LL_DMA_ClearFlag_FE3(DMA_TypeDef *DMAx)
{
  ((DMAx->LIFCR) = ((0x1U << (22U))));
}






 
static __inline void LL_DMA_ClearFlag_FE4(DMA_TypeDef *DMAx)
{
  ((DMAx->HIFCR) = ((0x1U << (0U))));
}






 
static __inline void LL_DMA_ClearFlag_FE5(DMA_TypeDef *DMAx)
{
  ((DMAx->HIFCR) = ((0x1U << (6U))));
}






 
static __inline void LL_DMA_ClearFlag_FE6(DMA_TypeDef *DMAx)
{
  ((DMAx->HIFCR) = ((0x1U << (16U))));
}






 
static __inline void LL_DMA_ClearFlag_FE7(DMA_TypeDef *DMAx)
{
  ((DMAx->HIFCR) = ((0x1U << (22U))));
}



 



 















 
static __inline void LL_DMA_EnableIT_HT(DMA_TypeDef *DMAx, uint32_t Stream)
{
  ((((DMA_Stream_TypeDef *)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CR) |= ((0x1U << (3U))));
}















 
static __inline void LL_DMA_EnableIT_TE(DMA_TypeDef *DMAx, uint32_t Stream)
{
  ((((DMA_Stream_TypeDef *)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CR) |= ((0x1U << (2U))));
}















 
static __inline void LL_DMA_EnableIT_TC(DMA_TypeDef *DMAx, uint32_t Stream)
{
  ((((DMA_Stream_TypeDef *)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CR) |= ((0x1U << (4U))));
}















 
static __inline void LL_DMA_EnableIT_DME(DMA_TypeDef *DMAx, uint32_t Stream)
{
  ((((DMA_Stream_TypeDef *)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CR) |= ((0x1U << (1U))));
}















 
static __inline void LL_DMA_EnableIT_FE(DMA_TypeDef *DMAx, uint32_t Stream)
{
  ((((DMA_Stream_TypeDef *)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->FCR) |= ((0x1U << (7U))));
}















 
static __inline void LL_DMA_DisableIT_HT(DMA_TypeDef *DMAx, uint32_t Stream)
{
  ((((DMA_Stream_TypeDef *)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CR) &= ~((0x1U << (3U))));
}















 
static __inline void LL_DMA_DisableIT_TE(DMA_TypeDef *DMAx, uint32_t Stream)
{
  ((((DMA_Stream_TypeDef *)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CR) &= ~((0x1U << (2U))));
}















 
static __inline void LL_DMA_DisableIT_TC(DMA_TypeDef *DMAx, uint32_t Stream)
{
  ((((DMA_Stream_TypeDef *)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CR) &= ~((0x1U << (4U))));
}















 
static __inline void LL_DMA_DisableIT_DME(DMA_TypeDef *DMAx, uint32_t Stream)
{
  ((((DMA_Stream_TypeDef *)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CR) &= ~((0x1U << (1U))));
}















 
static __inline void LL_DMA_DisableIT_FE(DMA_TypeDef *DMAx, uint32_t Stream)
{
  ((((DMA_Stream_TypeDef *)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->FCR) &= ~((0x1U << (7U))));
}















 
static __inline uint32_t LL_DMA_IsEnabledIT_HT(DMA_TypeDef *DMAx, uint32_t Stream)
{
  return (((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CR) & ((0x1U << (3U)))) == (0x1U << (3U)));
}















 
static __inline uint32_t LL_DMA_IsEnabledIT_TE(DMA_TypeDef *DMAx, uint32_t Stream)
{
  return (((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CR) & ((0x1U << (2U)))) == (0x1U << (2U)));
}















 
static __inline uint32_t LL_DMA_IsEnabledIT_TC(DMA_TypeDef *DMAx, uint32_t Stream)
{
  return (((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CR) & ((0x1U << (4U)))) == (0x1U << (4U)));
}















 
static __inline uint32_t LL_DMA_IsEnabledIT_DME(DMA_TypeDef *DMAx, uint32_t Stream)
{
  return (((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->CR) & ((0x1U << (1U)))) == (0x1U << (1U)));
}















 
static __inline uint32_t LL_DMA_IsEnabledIT_FE(DMA_TypeDef *DMAx, uint32_t Stream)
{
  return (((((DMA_Stream_TypeDef*)((uint32_t)((uint32_t)DMAx + STREAM_OFFSET_TAB[Stream])))->FCR) & ((0x1U << (7U)))) == (0x1U << (7U)));
}



 




 

uint32_t LL_DMA_Init(DMA_TypeDef *DMAx, uint32_t Stream, LL_DMA_InitTypeDef *DMA_InitStruct);
uint32_t LL_DMA_DeInit(DMA_TypeDef *DMAx, uint32_t Stream);
void LL_DMA_StructInit(LL_DMA_InitTypeDef *DMA_InitStruct);



 




 



 





 







 
#line 65 "../Inc/main.h"

 

 

 

#line 98 "../Inc/main.h"

 



 
 

 

 




void _Error_Handler(char *, int);








 
#line 44 "../Inc/stm32f4xx_hal_conf.h"
 
 

 


 


 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 


 
 
 
 
 
 
 
 
 
 
 
 
 
 
#line 100 "../Inc/stm32f4xx_hal_conf.h"

 




 












 






 







 












 





 

 


 
#line 165 "../Inc/stm32f4xx_hal_conf.h"

 



 
 

 

 

 
#line 184 "../Inc/stm32f4xx_hal_conf.h"

    





 

  

  

 





 



 
#line 218 "../Inc/stm32f4xx_hal_conf.h"




  
 





 




 



 


 

#line 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"

































 

 







 
#line 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_def.h"


































 

 







 
#line 47 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_def.h"
#line 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


































 

 







 
 
 



 








 



 
#line 105 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 113 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 



 





 



 
#line 147 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 214 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 



 



 



 






 



 

#line 250 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 



 
#line 272 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"









 



 

#line 358 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 



 

#line 376 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 




 
#line 395 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 



 





 



 






















#line 440 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 447 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"










 



 

#line 524 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"




 




 
#line 543 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 552 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 



 
#line 575 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 



 





 



 






 



 















 
 






 



 














 



 










 



 




























 



 






 



 

 
#line 720 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

 












 



 






























 




 















 




 
#line 807 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 



 











 



 



#line 860 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 870 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 889 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 



 




 



 

























 




 








 



 




 



 
#line 969 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 



 

#line 986 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 998 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 1029 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 



 











 

#line 1076 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 



 

 



 



 



 
#line 1104 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

 













 



 
#line 1139 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 



 
#line 1153 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

 

 



 






 

 



 
#line 1190 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 1198 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"






#line 1214 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 

 



 





 



 



 



 






 



 



 



 






 




 



 

 



 





 



 
#line 1307 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"









 




 
#line 1335 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 1356 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 1367 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 1376 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 1389 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 1398 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 



 







 



 
#line 1434 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 1449 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


#line 1475 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 



 
#line 1642 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



#line 1652 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 

#line 1666 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 







 



 

#line 1689 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 



 

#line 1717 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 



 






 



 














 




 




 




 







 




 
#line 1791 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 




 
#line 1835 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 1849 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 




 







#line 2122 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 2136 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 2379 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 2527 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

 



#line 2552 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 2573 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 2690 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 2707 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 2722 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"






#line 2751 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

















#line 2777 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"





#line 2804 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 2811 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 2820 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 2853 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 2871 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"












#line 2889 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 2910 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 2918 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"



 



 




 



 
#line 2941 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 2969 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 2984 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"






 



 




#line 3020 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"
 




#line 3050 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 3057 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 3068 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 



 

#line 3082 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"








 



 
#line 3103 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 



 







 



 













 




 











 



 












#line 3176 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 3185 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"

#line 3194 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"








 



 








#line 3227 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"




 



 

#line 3244 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"






 



 




 



 
#line 3278 "../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy/stm32_hal_legacy.h"


 



 







 



 



 







 

#line 48 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_def.h"
#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"
 
 
 





 






 







 




  
 








#line 47 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"


  



    typedef unsigned int size_t;    









 
 

 



    typedef struct __va_list __va_list;






   




 




typedef struct __fpos_t_struct {
    unsigned __int64 __pos;
    



 
    struct {
        unsigned int __state1, __state2;
    } __mbstate;
} fpos_t;
   


 


   

 

typedef struct __FILE FILE;
   






 

#line 136 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"


extern FILE __stdin, __stdout, __stderr;
extern FILE *__aeabi_stdin, *__aeabi_stdout, *__aeabi_stderr;

#line 166 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"
    

    

    





     



   


 


   


 

   



 

   


 




   


 





    


 






extern __declspec(__nothrow) int remove(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int rename(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) FILE *tmpfile(void);
   




 
extern __declspec(__nothrow) char *tmpnam(char *  );
   











 

extern __declspec(__nothrow) int fclose(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) int fflush(FILE *  );
   







 
extern __declspec(__nothrow) FILE *fopen(const char * __restrict  ,
                           const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   








































 
extern __declspec(__nothrow) FILE *freopen(const char * __restrict  ,
                    const char * __restrict  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(2,3)));
   








 
extern __declspec(__nothrow) void setbuf(FILE * __restrict  ,
                    char * __restrict  ) __attribute__((__nonnull__(1)));
   




 
extern __declspec(__nothrow) int setvbuf(FILE * __restrict  ,
                   char * __restrict  ,
                   int  , size_t  ) __attribute__((__nonnull__(1)));
   















 
#pragma __printf_args
extern __declspec(__nothrow) int fprintf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   


















 
#pragma __printf_args
extern __declspec(__nothrow) int _fprintf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   




 
#pragma __printf_args
extern __declspec(__nothrow) int _printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






 
#pragma __printf_args
extern __declspec(__nothrow) int _sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

#pragma __printf_args
extern __declspec(__nothrow) int __ARM_snprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(3)));


#pragma __printf_args
extern __declspec(__nothrow) int snprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   















 

#pragma __printf_args
extern __declspec(__nothrow) int _snprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int fscanf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






























 
#pragma __scanf_args
extern __declspec(__nothrow) int _fscanf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   






 
#pragma __scanf_args
extern __declspec(__nothrow) int _scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int sscanf(const char * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   








 
#pragma __scanf_args
extern __declspec(__nothrow) int _sscanf(const char * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

 
extern __declspec(__nothrow) int vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int _vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int _vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int _vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int __ARM_vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int _vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) int vfprintf(FILE * __restrict  ,
                    const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int vsprintf(char * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int __ARM_vsnprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));

extern __declspec(__nothrow) int vsnprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   







 

extern __declspec(__nothrow) int _vsprintf(char * __restrict  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vfprintf(FILE * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vsnprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   



 

#pragma __printf_args
extern __declspec(__nothrow) int asprintf(char **  , const char * __restrict  , ...) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) int vasprintf(char **  , const char * __restrict  , __va_list  ) __attribute__((__nonnull__(2)));

#pragma __printf_args
extern __declspec(__nothrow) int __ARM_asprintf(char **  , const char * __restrict  , ...) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) int __ARM_vasprintf(char **  , const char * __restrict  , __va_list  ) __attribute__((__nonnull__(2)));
   








 

extern __declspec(__nothrow) int fgetc(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) char *fgets(char * __restrict  , int  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   










 
extern __declspec(__nothrow) int fputc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   







 
extern __declspec(__nothrow) int fputs(const char * __restrict  , FILE * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int getc(FILE *  ) __attribute__((__nonnull__(1)));
   







 




    extern __declspec(__nothrow) int (getchar)(void);

   





 
extern __declspec(__nothrow) char *gets(char *  ) __attribute__((__nonnull__(1)));
   









 
extern __declspec(__nothrow) int putc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   





 




    extern __declspec(__nothrow) int (putchar)(int  );

   



 
extern __declspec(__nothrow) int puts(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int ungetc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   






















 

extern __declspec(__nothrow) size_t fread(void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   











 

extern __declspec(__nothrow) size_t __fread_bytes_avail(void * __restrict  ,
                    size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   











 

extern __declspec(__nothrow) size_t fwrite(const void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   







 

extern __declspec(__nothrow) int fgetpos(FILE * __restrict  , fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) int fseek(FILE *  , long int  , int  ) __attribute__((__nonnull__(1)));
   














 
extern __declspec(__nothrow) int fsetpos(FILE * __restrict  , const fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   










 
extern __declspec(__nothrow) long int ftell(FILE *  ) __attribute__((__nonnull__(1)));
   











 
extern __declspec(__nothrow) void rewind(FILE *  ) __attribute__((__nonnull__(1)));
   





 

extern __declspec(__nothrow) void clearerr(FILE *  ) __attribute__((__nonnull__(1)));
   




 

extern __declspec(__nothrow) int feof(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) int ferror(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) void perror(const char *  );
   









 

extern __declspec(__nothrow) int _fisatty(FILE *   ) __attribute__((__nonnull__(1)));
    
 

extern __declspec(__nothrow) void __use_no_semihosting_swi(void);
extern __declspec(__nothrow) void __use_no_semihosting(void);
    





 











#line 1021 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"



 

#line 49 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_def.h"

 



   
typedef enum 
{
  HAL_OK       = 0x00U,
  HAL_ERROR    = 0x01U,
  HAL_BUSY     = 0x02U,
  HAL_TIMEOUT  = 0x03U
} HAL_StatusTypeDef;



 
typedef enum 
{
  HAL_UNLOCKED = 0x00U,
  HAL_LOCKED   = 0x01U  
} HAL_LockTypeDef;

 




























 


#line 119 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_def.h"







#line 134 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_def.h"


 
#line 156 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_def.h"




  









 


#line 189 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_def.h"



  



 


#line 206 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_def.h"







 
#line 46 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"

 
 
#line 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

































  

 







 
#line 46 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"



 



  

 


 



 
typedef struct
{
  uint32_t PLLState;   
 

  uint32_t PLLSource;  
 

  uint32_t PLLM;       
 

  uint32_t PLLN;       

 

  uint32_t PLLP;       
 

  uint32_t PLLQ;       
 
#line 91 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
}RCC_PLLInitTypeDef;

#line 192 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

#line 218 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

#line 309 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

#line 394 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"





 
typedef struct
{




                                
  uint32_t PLLI2SN;    


 

  uint32_t PLLI2SR;    

 

}RCC_PLLI2SInitTypeDef;
 


 
typedef struct
{
  uint32_t PeriphClockSelection; 
 

  RCC_PLLI2SInitTypeDef PLLI2S;  
 

  uint32_t RTCClockSelection;      
 




}RCC_PeriphCLKInitTypeDef;



  

 


 



 
 
#line 470 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
 

 
#line 480 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
 

 
#line 497 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
 
    
 
#line 511 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
 

 
#line 523 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
 

 
#line 535 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
 


 






 




 





 
#line 564 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"


 



 
#line 578 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"


 



 
#line 591 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"


 

#line 616 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
      
#line 645 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

#line 738 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

#line 793 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

#line 872 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

#line 906 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

#line 923 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

#line 938 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"








 






 




#line 970 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"



 
     
 


 
 
#line 2023 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
 

 







 
#line 2115 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"


 
#line 2151 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"



 
#line 2165 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"


 
  






   
#line 2186 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

#line 2198 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"


 
#line 2208 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"


 
#line 2219 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"


 
  






 



                                        


#line 2244 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

#line 2255 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

#line 2274 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"


 








 











#line 2304 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"


   
  






 
#line 2323 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"


 







 




    
   






 
#line 2475 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"


 
 






  
#line 2502 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

#line 2519 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
  

 
  






  
#line 2572 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

#line 2579 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"


 







 
#line 2596 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
  
#line 2603 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"


 
    



 
#line 2619 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

#line 2628 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"


 




 








#line 2651 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
   







 




  







 




 
#line 2694 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

#line 2711 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"


 




 




                                          






 
                                        







 
#line 2756 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

#line 2773 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"


 








 











#line 2803 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"


 
                                        







 




 
                                        







 
#line 2845 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

#line 2862 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"


 
                                        







 
#line 2880 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

#line 2887 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"


 

 

 
#line 3272 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
 

 
#line 3542 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
 

 
#line 3918 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
 

 
#line 4722 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
 

 
#line 5716 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
 

 
#line 5762 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"




























 
#line 5797 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
 
                             
 








 



#line 5880 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"














 





#line 5925 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

#line 5948 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
 

 
#line 6038 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
 

 
#line 6057 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
                                 
#line 6076 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

#line 6088 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
 

 











 








 


                                 
#line 6147 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

#line 6312 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
      
#line 6361 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

#line 6595 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

#line 6650 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
      
#line 6674 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

 

#line 6701 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

#line 6714 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"



 

 


 



 
HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit);
void HAL_RCCEx_GetPeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit);

uint32_t HAL_RCCEx_GetPeriphCLKFreq(uint32_t PeriphClk);

#line 6739 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
HAL_StatusTypeDef HAL_RCCEx_EnablePLLI2S(RCC_PLLI2SInitTypeDef  *PLLI2SInit);
HAL_StatusTypeDef HAL_RCCEx_DisablePLLI2S(void);







  



 
 
 
 


 




 
   
#line 6773 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"






 






 
#line 6799 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

 





 


      



      
#line 6824 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

#line 6834 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"




 



 

 


 


 
#line 6861 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"
      



























      



      


#line 6915 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

#line 6923 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

#line 6943 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

#line 6995 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

#line 7016 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"

#line 7080 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc_ex.h"






      













 



 



  



   






 
#line 50 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"



 



 

 


 



 
typedef struct
{
  uint32_t OscillatorType;       
 

  uint32_t HSEState;             
 

  uint32_t LSEState;             
 

  uint32_t HSIState;             
 

  uint32_t HSICalibrationValue;  
 

  uint32_t LSIState;             
 

  RCC_PLLInitTypeDef PLL;         
}RCC_OscInitTypeDef;



 
typedef struct
{
  uint32_t ClockType;             
 

  uint32_t SYSCLKSource;          
 

  uint32_t AHBCLKDivider;         
 

  uint32_t APB1CLKDivider;        
 

  uint32_t APB2CLKDivider;        
 

}RCC_ClkInitTypeDef;



 

 


 



 







 



 





 



 





 



 






 



 




 



 





 



 






 



 




 



 






 





 






 





 






 



 
#line 252 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"


 



 







 



 
#line 305 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"


 



 




 



 






 



 







 



 
#line 351 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"


 









 
 





 


 
#line 382 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"


 



 

 


 







 
#line 444 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"

#line 451 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"


 







 
#line 468 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"

#line 475 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"


 







 
#line 535 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"

#line 543 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"


 







 
#line 561 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"

#line 569 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"


 







 
#line 636 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"

#line 645 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"


 







 
#line 664 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"

#line 673 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"


 




 
#line 688 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"

#line 696 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"


 




 
#line 712 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"

#line 721 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"


 




 
#line 738 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"

#line 748 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"


 








 
#line 766 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"

#line 773 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"


 








 
#line 792 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"

#line 800 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"


 








 
#line 820 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"

#line 829 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"


 



 















 









 




 



 








 




 



 





















 
#line 928 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"


 



 


















 
#line 971 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"


 



 



 
























 













 







 






 




 



 







 










 










 



 



 









 










 







 



 



 















 




















 




 




 











 












 













 













 




 



















 





 



 

 
 

 



 
 
HAL_StatusTypeDef HAL_RCC_DeInit(void);
HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef *RCC_OscInitStruct);
HAL_StatusTypeDef HAL_RCC_ClockConfig(RCC_ClkInitTypeDef *RCC_ClkInitStruct, uint32_t FLatency);


 



 
 
void     HAL_RCC_MCOConfig(uint32_t RCC_MCOx, uint32_t RCC_MCOSource, uint32_t RCC_MCODiv);
void     HAL_RCC_EnableCSS(void);
void     HAL_RCC_DisableCSS(void);
uint32_t HAL_RCC_GetSysClockFreq(void);
uint32_t HAL_RCC_GetHCLKFreq(void);
uint32_t HAL_RCC_GetPCLK1Freq(void);
uint32_t HAL_RCC_GetPCLK2Freq(void);
void     HAL_RCC_GetOscConfig(RCC_OscInitTypeDef *RCC_OscInitStruct);
void     HAL_RCC_GetClockConfig(RCC_ClkInitTypeDef *RCC_ClkInitStruct, uint32_t *pFLatency);

 
void HAL_RCC_NMI_IRQHandler(void);

 
void HAL_RCC_CSSCallback(void);



 



 

 
 
 


 




 

 
 



 


 



 
 



 



 
 




 


 


 


 












 



 

 


 



 






















#line 1429 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_rcc.h"































 



 



 



 







 
#line 245 "../Inc/stm32f4xx_hal_conf.h"


#line 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio.h"

































  

 







 
#line 46 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio.h"



 



  

 


 



  
typedef struct
{
  uint32_t Pin;       
 

  uint32_t Mode;      
 

  uint32_t Pull;      
 

  uint32_t Speed;     
 

  uint32_t Alternate;  
 
}GPIO_InitTypeDef;



 
typedef enum
{
  GPIO_PIN_RESET = 0,
  GPIO_PIN_SET
}GPIO_PinState;


 

 



  



 
#line 119 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio.h"




 










  







    



 





 




 






 

 


   





 
  


 

 


 






 







 







 







 







 



 

 
#line 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"

































  

 







 
#line 46 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"



 



  

 
 


 
  


 

 
#line 182 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"
 

 
#line 297 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"
 

 



  








  





  






  







  






  






  





  







  






  








  





  




  






  




  


 

 
#line 499 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"

 

 
#line 589 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"
 

 
#line 698 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"

 

 
#line 832 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"

 
#line 924 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"

 
#line 998 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"

 
#line 1118 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"
 

 
#line 1241 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"
 


  



 

 


 


 

 


 


 

 
 
 


 


 

 


 


 
#line 1293 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"

#line 1307 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"







#line 1321 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"

#line 1331 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"



 



   
 
#line 1365 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"
 

 
#line 1392 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"
 

 
#line 1414 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"


 

 
#line 1439 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"

 

 
#line 1459 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"
 
 




 
#line 1484 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"
 

 
#line 1516 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"
 

 
#line 1545 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio_ex.h"
 

 



 

 



 



  



 

 


 



 



  



  
  






 
#line 231 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio.h"

 


 



 
 
void  HAL_GPIO_Init(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init);
void  HAL_GPIO_DeInit(GPIO_TypeDef  *GPIOx, uint32_t GPIO_Pin);


 



 
 
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
void HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
HAL_StatusTypeDef HAL_GPIO_LockPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void HAL_GPIO_EXTI_IRQHandler(uint16_t GPIO_Pin);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);



  



  
 
 
 


 



 

 


 
#line 298 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_gpio.h"


 

 


 



 



  



 







 
#line 249 "../Inc/stm32f4xx_hal_conf.h"


#line 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dma.h"

































  

 







 
#line 46 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dma.h"



 



  

 




 
   


 
typedef struct
{
  uint32_t Channel;              
 

  uint32_t Direction;            

 

  uint32_t PeriphInc;            
 

  uint32_t MemInc;               
 

  uint32_t PeriphDataAlignment;  
 

  uint32_t MemDataAlignment;     
 

  uint32_t Mode;                 


 

  uint32_t Priority;             
 

  uint32_t FIFOMode;             


 

  uint32_t FIFOThreshold;        
 

  uint32_t MemBurst;             



 

  uint32_t PeriphBurst;          



 
}DMA_InitTypeDef;




 
typedef enum
{
  HAL_DMA_STATE_RESET             = 0x00U,   
  HAL_DMA_STATE_READY             = 0x01U,   
  HAL_DMA_STATE_BUSY              = 0x02U,   
  HAL_DMA_STATE_TIMEOUT           = 0x03U,   
  HAL_DMA_STATE_ERROR             = 0x04U,   
  HAL_DMA_STATE_ABORT             = 0x05U,   
}HAL_DMA_StateTypeDef;



 
typedef enum
{
  HAL_DMA_FULL_TRANSFER           = 0x00U,   
  HAL_DMA_HALF_TRANSFER           = 0x01U    
}HAL_DMA_LevelCompleteTypeDef;



 
typedef enum
{
  HAL_DMA_XFER_CPLT_CB_ID         = 0x00U,   
  HAL_DMA_XFER_HALFCPLT_CB_ID     = 0x01U,   
  HAL_DMA_XFER_M1CPLT_CB_ID       = 0x02U,   
  HAL_DMA_XFER_M1HALFCPLT_CB_ID   = 0x03U,   
  HAL_DMA_XFER_ERROR_CB_ID        = 0x04U,   
  HAL_DMA_XFER_ABORT_CB_ID        = 0x05U,   
  HAL_DMA_XFER_ALL_CB_ID          = 0x06U    
}HAL_DMA_CallbackIDTypeDef;



 
typedef struct __DMA_HandleTypeDef
{
  DMA_Stream_TypeDef         *Instance;                                                         

  DMA_InitTypeDef            Init;                                                               

  HAL_LockTypeDef            Lock;                                                                

  volatile HAL_DMA_StateTypeDef  State;                                                             

  void                       *Parent;                                                            

  void                       (* XferCpltCallback)( struct __DMA_HandleTypeDef * hdma);          

  void                       (* XferHalfCpltCallback)( struct __DMA_HandleTypeDef * hdma);      

  void                       (* XferM1CpltCallback)( struct __DMA_HandleTypeDef * hdma);        
  
  void                       (* XferM1HalfCpltCallback)( struct __DMA_HandleTypeDef * hdma);    
  
  void                       (* XferErrorCallback)( struct __DMA_HandleTypeDef * hdma);         
  
  void                       (* XferAbortCallback)( struct __DMA_HandleTypeDef * hdma);           

  volatile uint32_t              ErrorCode;                                                         
  
  uint32_t                   StreamBaseAddress;                                                 

  uint32_t                   StreamIndex;                                                       
 
}DMA_HandleTypeDef;



 

 




 




  
#line 210 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dma.h"


 




  
#line 236 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dma.h"


 




  





 
        



  




  




  




 




  





  




 





 




  





 




 






  




 




  




 






  




  






  




  






 




 







 




  
#line 399 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dma.h"


 



 
 
 




 













 






 






 


 





 
#line 464 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dma.h"





       
#line 484 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dma.h"





 
#line 504 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dma.h"





 
#line 524 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dma.h"





 
#line 544 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dma.h"













 

















 
















 














 














 




















 







 



 
#line 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dma_ex.h"

































 

 







 
#line 46 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dma_ex.h"



 



  

 



 
   


  
typedef enum
{
  MEMORY0      = 0x00U,     
  MEMORY1      = 0x01U      
}HAL_DMA_MemoryTypeDef;



 

 



 




 

 
HAL_StatusTypeDef HAL_DMAEx_MultiBufferStart(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t SecondMemAddress, uint32_t DataLength);
HAL_StatusTypeDef HAL_DMAEx_MultiBufferStart_IT(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t SecondMemAddress, uint32_t DataLength);
HAL_StatusTypeDef HAL_DMAEx_ChangeMemory(DMA_HandleTypeDef *hdma, uint32_t Address, HAL_DMA_MemoryTypeDef memory);



 


 
         
 



 


 



 



 







 
#line 657 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dma.h"

 




 




 
HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *hdma); 
HAL_StatusTypeDef HAL_DMA_DeInit(DMA_HandleTypeDef *hdma);


 




 
HAL_StatusTypeDef HAL_DMA_Start (DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
HAL_StatusTypeDef HAL_DMA_Start_IT(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_Abort_IT(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_PollForTransfer(DMA_HandleTypeDef *hdma, HAL_DMA_LevelCompleteTypeDef CompleteLevel, uint32_t Timeout);
void              HAL_DMA_IRQHandler(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_CleanCallbacks(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_RegisterCallback(DMA_HandleTypeDef *hdma, HAL_DMA_CallbackIDTypeDef CallbackID, void (* pCallback)(DMA_HandleTypeDef *_hdma));
HAL_StatusTypeDef HAL_DMA_UnRegisterCallback(DMA_HandleTypeDef *hdma, HAL_DMA_CallbackIDTypeDef CallbackID);



  




 
HAL_DMA_StateTypeDef HAL_DMA_GetState(DMA_HandleTypeDef *hdma);
uint32_t             HAL_DMA_GetError(DMA_HandleTypeDef *hdma);


  


  
 



 


  

 



 
#line 746 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_dma.h"

















































  

 



 


 



  



 







 
#line 253 "../Inc/stm32f4xx_hal_conf.h"

   
#line 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_cortex.h"

































  

 







 
#line 46 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_cortex.h"



 



  
 


 





 
typedef struct
{
  uint8_t                Enable;                
 
  uint8_t                Number;                
 
  uint32_t               BaseAddress;            
  uint8_t                Size;                  
 
  uint8_t                SubRegionDisable;      
          
  uint8_t                TypeExtField;          
                  
  uint8_t                AccessPermission;      
 
  uint8_t                DisableExec;           
 
  uint8_t                IsShareable;           
 
  uint8_t                IsCacheable;           
 
  uint8_t                IsBufferable;          
 
}MPU_Region_InitTypeDef;


 




 

 



 



 
#line 116 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_cortex.h"


 



 





 




 







 



 




 



 




 



 




 



 




 



 




 



 





 



 
#line 229 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_cortex.h"


 
   


 
#line 242 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_cortex.h"


 



 
#line 257 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_cortex.h"


 




 


 

 


 
  


 
 
void HAL_NVIC_SetPriorityGrouping(uint32_t PriorityGroup);
void HAL_NVIC_SetPriority(IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority);
void HAL_NVIC_EnableIRQ(IRQn_Type IRQn);
void HAL_NVIC_DisableIRQ(IRQn_Type IRQn);
void HAL_NVIC_SystemReset(void);
uint32_t HAL_SYSTICK_Config(uint32_t TicksNumb);


 



 
 
uint32_t HAL_NVIC_GetPriorityGrouping(void);
void HAL_NVIC_GetPriority(IRQn_Type IRQn, uint32_t PriorityGroup, uint32_t* pPreemptPriority, uint32_t* pSubPriority);
uint32_t HAL_NVIC_GetPendingIRQ(IRQn_Type IRQn);
void HAL_NVIC_SetPendingIRQ(IRQn_Type IRQn);
void HAL_NVIC_ClearPendingIRQ(IRQn_Type IRQn);
uint32_t HAL_NVIC_GetActive(IRQn_Type IRQn);
void HAL_SYSTICK_CLKSourceConfig(uint32_t CLKSource);
void HAL_SYSTICK_IRQHandler(void);
void HAL_SYSTICK_Callback(void);


void HAL_MPU_Enable(uint32_t MPU_Control);
void HAL_MPU_Disable(void);
void HAL_MPU_ConfigRegion(MPU_Region_InitTypeDef *MPU_Init);



 



 

 
 
 
 


 



































#line 363 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_cortex.h"

#line 372 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_cortex.h"

#line 401 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_cortex.h"






 

 



  



 
  





 

 
#line 257 "../Inc/stm32f4xx_hal_conf.h"


































#line 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash.h"

































  

 







 
#line 46 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash.h"



 



  

 


 
 


 
typedef enum 
{
  FLASH_PROC_NONE = 0U, 
  FLASH_PROC_SECTERASE,
  FLASH_PROC_MASSERASE,
  FLASH_PROC_PROGRAM
} FLASH_ProcedureTypeDef;



 
typedef struct
{
  volatile FLASH_ProcedureTypeDef ProcedureOnGoing;    
  
  volatile uint32_t               NbSectorsToErase;    
  
  volatile uint8_t                VoltageForErase;     
  
  volatile uint32_t               Sector;              
  
  volatile uint32_t               Bank;                
  
  volatile uint32_t               Address;             
  
  HAL_LockTypeDef             Lock;                

  volatile uint32_t               ErrorCode;           

}FLASH_ProcessTypeDef;



 

 


   



  
#line 113 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash.h"


 
  


  






 




  
#line 142 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash.h"


 
  



  




   



 







  



  







  



  
  
 


 





  






  





  





  





  





  





  





  






 








 










   









   
















 















 



 

 
#line 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"

































  

 







 
#line 46 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"



 



  

 


 



 
typedef struct
{
  uint32_t TypeErase;   
 

  uint32_t Banks;       
 

  uint32_t Sector;      
 

  uint32_t NbSectors;   
 

  uint32_t VoltageRange;
 

} FLASH_EraseInitTypeDef;



 
typedef struct
{
  uint32_t OptionType;   
 

  uint32_t WRPState;     
 

  uint32_t WRPSector;         
 

  uint32_t Banks;        
         

  uint32_t RDPLevel;     
 

  uint32_t BORLevel;     
 

  uint8_t  USERConfig;    

} FLASH_OBProgramInitTypeDef;



 
#line 149 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"


 

 



 



  




 
  


  






 
  


  




 
  


  






 
  


 






  
  


  




  
  


  




  




  




     



   






 

#line 265 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"



  






#line 282 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"


 



 
   
#line 309 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
 

  




     
#line 327 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
 



  
  



 
#line 343 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"

#line 352 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"


  
    


 





#line 372 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"


  



 
    
#line 407 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
 

    
#line 428 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
       

  
#line 446 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
 

  
#line 457 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
 

  
#line 467 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
 

 
#line 480 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
 



  



 
   
#line 518 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
 

  
#line 540 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
     
      
  
#line 559 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
 

 
#line 571 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
 
 
 
#line 582 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
 

 
#line 596 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
 


 
  


 
    
#line 633 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
 
      
 
#line 655 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
       

 
#line 667 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
 

 
#line 678 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
 

 
#line 693 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
 



 
  


 







 



 
#line 724 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"


 



  
  
 

 


 



 
 
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *pEraseInit, uint32_t *SectorError);
HAL_StatusTypeDef HAL_FLASHEx_Erase_IT(FLASH_EraseInitTypeDef *pEraseInit);
HAL_StatusTypeDef HAL_FLASHEx_OBProgram(FLASH_OBProgramInitTypeDef *pOBInit);
void              HAL_FLASHEx_OBGetConfig(FLASH_OBProgramInitTypeDef *pOBInit);

#line 760 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"







 



 
 
 
 


 
  




 




  





  




  




 






  






 

 


 



 



























#line 865 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"







#line 879 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
  
#line 899 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"

#line 914 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"







#line 929 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"
 
#line 944 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"

#line 955 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"

#line 965 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"













#line 984 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"





  
























   


























#line 1050 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ex.h"


 



 

 


 
void FLASH_Erase_Sector(uint32_t Sector, uint8_t VoltageRange);
void FLASH_FlushCaches(void);


  



  



 







 
#line 314 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash.h"
#line 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ramfunc.h"

































  

 



#line 91 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash_ramfunc.h"




 
#line 315 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_flash.h"

 


 


 
 
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t TypeProgram, uint32_t Address, uint64_t Data);
HAL_StatusTypeDef HAL_FLASH_Program_IT(uint32_t TypeProgram, uint32_t Address, uint64_t Data);
 
void HAL_FLASH_IRQHandler(void);
  
void HAL_FLASH_EndOfOperationCallback(uint32_t ReturnValue);
void HAL_FLASH_OperationErrorCallback(uint32_t ReturnValue);


 



 
 
HAL_StatusTypeDef HAL_FLASH_Unlock(void);
HAL_StatusTypeDef HAL_FLASH_Lock(void);
HAL_StatusTypeDef HAL_FLASH_OB_Unlock(void);
HAL_StatusTypeDef HAL_FLASH_OB_Lock(void);
 
HAL_StatusTypeDef HAL_FLASH_OB_Launch(void);


 



 
 
uint32_t HAL_FLASH_GetError(void);
HAL_StatusTypeDef FLASH_WaitForLastOperation(uint32_t Timeout);


 



  
 
 


 



 
 


 



  



  



  



  



  




 

 


 



 






 



 

 


 



 



  



 







 
#line 293 "../Inc/stm32f4xx_hal_conf.h"

 















  
























#line 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pwr.h"

































  

 







 
#line 46 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pwr.h"



 



  

 



 
   


 
typedef struct
{
  uint32_t PVDLevel;   
 

  uint32_t Mode;      
 
}PWR_PVDTypeDef;



 

 


 
  


 



 



  
#line 102 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pwr.h"


    
 


 
#line 116 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pwr.h"


 




 




 
    


 




 



 




 



 







 



  
  
 


 





















 







 





 





 





 





 





 





 





 






 






 








 







 





 





 




 

 
#line 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pwr_ex.h"

































  

 







 
#line 46 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pwr_ex.h"



 



  

  
 


 
#line 82 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pwr_ex.h"



 
#line 96 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pwr_ex.h"


 
#line 115 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pwr_ex.h"



  
  
 


 










 
#line 161 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pwr_ex.h"

#line 209 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pwr_ex.h"


 

 


 
 


 
void HAL_PWREx_EnableFlashPowerDown(void);
void HAL_PWREx_DisableFlashPowerDown(void); 
HAL_StatusTypeDef HAL_PWREx_EnableBkUpReg(void);
HAL_StatusTypeDef HAL_PWREx_DisableBkUpReg(void); 
uint32_t HAL_PWREx_GetVoltageRange(void);
HAL_StatusTypeDef HAL_PWREx_ControlVoltageScaling(uint32_t VoltageScaling);






#line 242 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pwr_ex.h"

#line 249 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pwr_ex.h"



 



 
 
 
 


 



 
 
 
 



 



 


    
 



 



 

 



   
 
 










 



 

 


 



 






#line 336 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pwr_ex.h"

#line 347 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pwr_ex.h"


 



 



  



 
  







 
#line 291 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pwr.h"

 


 
  


 
 
void HAL_PWR_DeInit(void);
void HAL_PWR_EnableBkUpAccess(void);
void HAL_PWR_DisableBkUpAccess(void);


 



 
 
 
void HAL_PWR_ConfigPVD(PWR_PVDTypeDef *sConfigPVD);
void HAL_PWR_EnablePVD(void);
void HAL_PWR_DisablePVD(void);

 
void HAL_PWR_EnableWakeUpPin(uint32_t WakeUpPinx);
void HAL_PWR_DisableWakeUpPin(uint32_t WakeUpPinx);

 
void HAL_PWR_EnterSTOPMode(uint32_t Regulator, uint8_t STOPEntry);
void HAL_PWR_EnterSLEEPMode(uint32_t Regulator, uint8_t SLEEPEntry);
void HAL_PWR_EnterSTANDBYMode(void);

 
void HAL_PWR_PVD_IRQHandler(void);
void HAL_PWR_PVDCallback(void);

 
void HAL_PWR_EnableSleepOnExit(void);
void HAL_PWR_DisableSleepOnExit(void);
void HAL_PWR_EnableSEVOnPend(void);
void HAL_PWR_DisableSEVOnPend(void);


 



 

 
 
 


 



 



 



 
 







 



 
 
 



 



 




 



 
 
 




 



 
 


 



 
#line 424 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_pwr.h"


 



 



  



 
  







 
#line 337 "../Inc/stm32f4xx_hal_conf.h"


























#line 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim.h"

































  

 







 
#line 46 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim.h"



 



 

 


 
  


 
typedef struct
{
  uint32_t Prescaler;         
 

  uint32_t CounterMode;       
 

  uint32_t Period;            

 

  uint32_t ClockDivision;     
 

  uint32_t RepetitionCounter;  






 
} TIM_Base_InitTypeDef;



 

typedef struct
{
  uint32_t OCMode;        
 

  uint32_t Pulse;         
 

  uint32_t OCPolarity;    
 

  uint32_t OCNPolarity;   

 
  
  uint32_t OCFastMode;   

 


  uint32_t OCIdleState;   

 

  uint32_t OCNIdleState;  

 
} TIM_OC_InitTypeDef;  



 
typedef struct
{
  uint32_t OCMode;        
 

  uint32_t Pulse;         
 

  uint32_t OCPolarity;    
 

  uint32_t OCNPolarity;   

 

  uint32_t OCIdleState;   

 

  uint32_t OCNIdleState;  

 

  uint32_t ICPolarity;    
 

  uint32_t ICSelection;   
 

  uint32_t ICFilter;      
   
} TIM_OnePulse_InitTypeDef;  




 

typedef struct
{
  uint32_t  ICPolarity;   
 

  uint32_t ICSelection;  
 

  uint32_t ICPrescaler;  
 

  uint32_t ICFilter;     
 
} TIM_IC_InitTypeDef;



 

typedef struct
{
  uint32_t EncoderMode;   
 
                                  
  uint32_t IC1Polarity;   
 

  uint32_t IC1Selection;  
 

  uint32_t IC1Prescaler;  
 

  uint32_t IC1Filter;     
 
                                  
  uint32_t IC2Polarity;   
 

  uint32_t IC2Selection;  
 

  uint32_t IC2Prescaler;  
 

  uint32_t IC2Filter;     
 
} TIM_Encoder_InitTypeDef;



  
typedef struct
{
  uint32_t ClockSource;     
  
  uint32_t ClockPolarity;   
 
  uint32_t ClockPrescaler;  
 
  uint32_t ClockFilter;    
 
}TIM_ClockConfigTypeDef;



  
typedef struct
{ 
  uint32_t ClearInputState;      
   
  uint32_t ClearInputSource;     
  
  uint32_t ClearInputPolarity;   
 
  uint32_t ClearInputPrescaler;  
 
  uint32_t ClearInputFilter;    
 
}TIM_ClearInputConfigTypeDef;



  
typedef struct {
  uint32_t  SlaveMode;         
  
  uint32_t  InputTrigger;      
 
  uint32_t  TriggerPolarity;   
 
  uint32_t  TriggerPrescaler;  
 
  uint32_t  TriggerFilter;     
   

}TIM_SlaveConfigTypeDef;



  
typedef enum
{
  HAL_TIM_STATE_RESET             = 0x00U,     
  HAL_TIM_STATE_READY             = 0x01U,     
  HAL_TIM_STATE_BUSY              = 0x02U,     
  HAL_TIM_STATE_TIMEOUT           = 0x03U,     
  HAL_TIM_STATE_ERROR             = 0x04U      
}HAL_TIM_StateTypeDef;



  
typedef enum
{
  HAL_TIM_ACTIVE_CHANNEL_1        = 0x01U,     
  HAL_TIM_ACTIVE_CHANNEL_2        = 0x02U,     
  HAL_TIM_ACTIVE_CHANNEL_3        = 0x04U,     
  HAL_TIM_ACTIVE_CHANNEL_4        = 0x08U,     
  HAL_TIM_ACTIVE_CHANNEL_CLEARED  = 0x00U      
}HAL_TIM_ActiveChannel;



  
typedef struct
{
  TIM_TypeDef                 *Instance;      
  TIM_Base_InitTypeDef        Init;           
  HAL_TIM_ActiveChannel       Channel;        
  DMA_HandleTypeDef           *hdma[7];      
 
  HAL_LockTypeDef             Lock;           
  volatile HAL_TIM_StateTypeDef   State;          
}TIM_HandleTypeDef;


 

 


 



 





 



 




 



 






 



 







 



 





 



 
#line 369 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim.h"



 



 




 



 




 



 




 



 




  



 




  



 





                                 


 



 





 



 








 



 






  



 




 



 



   


 



  
#line 497 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim.h"


 
  


   




 



 
#line 520 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim.h"


 



 
#line 535 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim.h"



 



 
#line 555 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim.h"


 



 
#line 572 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim.h"


 



 







 



 






 



 




 



 




 



 






 



   




 
  


 




 
  


 






   


 




 
  


 




 
  


 




   
  


   
#line 694 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim.h"


  
  


 







 



 




  
  


 
#line 731 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim.h"


   



 







 



 






 




 




  



 
#line 791 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim.h"


  



 
#line 816 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim.h"


 



 
#line 830 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim.h"


  



 






  



    
  
 


 



 






 






 







 
#line 889 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim.h"


 




 
#line 907 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim.h"






 















 
















 














 














 























 























 
















 















 








 







 






































 















 








 






 








 









 











 
#line 1183 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim.h"







 



















 




















 





    







 













 


















 







 

 
#line 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim_ex.h"

































  

 







 
#line 46 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim_ex.h"



 



  

  


 
  


 

typedef struct
{

  uint32_t IC1Polarity;            
 

  uint32_t IC1Prescaler;        
 

  uint32_t IC1Filter;           
 

  uint32_t Commutation_Delay;  
 
} TIM_HallSensor_InitTypeDef;



  
typedef struct {
  uint32_t  MasterOutputTrigger;   
 

  uint32_t  MasterSlaveMode;       
 
}TIM_MasterConfigTypeDef;



  
typedef struct
{
  uint32_t OffStateRunMode;            
 
  uint32_t OffStateIDLEMode;          
 
  uint32_t LockLevel;                     
                              
  uint32_t DeadTime;                     
 
  uint32_t BreakState;                   
 
  uint32_t BreakPolarity;                 
 
  uint32_t AutomaticOutput;               
            
}TIM_BreakDeadTimeConfigTypeDef;


 
  
 


 
  


 
#line 133 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim_ex.h"

#line 142 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim_ex.h"






 

#line 161 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim_ex.h"



  
 
 


 



 
 
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Init(TIM_HandleTypeDef* htim, TIM_HallSensor_InitTypeDef* sConfig);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_DeInit(TIM_HandleTypeDef* htim);

void HAL_TIMEx_HallSensor_MspInit(TIM_HandleTypeDef* htim);
void HAL_TIMEx_HallSensor_MspDeInit(TIM_HandleTypeDef* htim);

  
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Start(TIM_HandleTypeDef* htim);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Stop(TIM_HandleTypeDef* htim);
 
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Start_IT(TIM_HandleTypeDef* htim);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Stop_IT(TIM_HandleTypeDef* htim);
 
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Start_DMA(TIM_HandleTypeDef* htim, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Stop_DMA(TIM_HandleTypeDef* htim);


 



 
 
 
HAL_StatusTypeDef HAL_TIMEx_OCN_Start(TIM_HandleTypeDef* htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_OCN_Stop(TIM_HandleTypeDef* htim, uint32_t Channel);

 
HAL_StatusTypeDef HAL_TIMEx_OCN_Start_IT(TIM_HandleTypeDef* htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_OCN_Stop_IT(TIM_HandleTypeDef* htim, uint32_t Channel);

 
HAL_StatusTypeDef HAL_TIMEx_OCN_Start_DMA(TIM_HandleTypeDef* htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIMEx_OCN_Stop_DMA(TIM_HandleTypeDef* htim, uint32_t Channel);


 



 
 
 
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start(TIM_HandleTypeDef* htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop(TIM_HandleTypeDef* htim, uint32_t Channel);

 
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start_IT(TIM_HandleTypeDef* htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop_IT(TIM_HandleTypeDef* htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start_DMA(TIM_HandleTypeDef* htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop_DMA(TIM_HandleTypeDef* htim, uint32_t Channel);


 



 
 
 
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Start(TIM_HandleTypeDef* htim, uint32_t OutputChannel);
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Stop(TIM_HandleTypeDef* htim, uint32_t OutputChannel);

 
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Start_IT(TIM_HandleTypeDef* htim, uint32_t OutputChannel);
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Stop_IT(TIM_HandleTypeDef* htim, uint32_t OutputChannel);


 



 
 
HAL_StatusTypeDef HAL_TIMEx_ConfigCommutationEvent(TIM_HandleTypeDef* htim, uint32_t  InputTrigger, uint32_t  CommutationSource);
HAL_StatusTypeDef HAL_TIMEx_ConfigCommutationEvent_IT(TIM_HandleTypeDef* htim, uint32_t  InputTrigger, uint32_t  CommutationSource);
HAL_StatusTypeDef HAL_TIMEx_ConfigCommutationEvent_DMA(TIM_HandleTypeDef* htim, uint32_t  InputTrigger, uint32_t  CommutationSource);
HAL_StatusTypeDef HAL_TIMEx_MasterConfigSynchronization(TIM_HandleTypeDef* htim, TIM_MasterConfigTypeDef * sMasterConfig);
HAL_StatusTypeDef HAL_TIMEx_ConfigBreakDeadTime(TIM_HandleTypeDef* htim, TIM_BreakDeadTimeConfigTypeDef *sBreakDeadTimeConfig);
HAL_StatusTypeDef HAL_TIMEx_RemapConfig(TIM_HandleTypeDef* htim, uint32_t Remap);


 



  
 
void HAL_TIMEx_CommutationCallback(TIM_HandleTypeDef* htim);
void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef* htim);
void TIMEx_DMACommutationCplt(DMA_HandleTypeDef *hdma);


 



 
 
HAL_TIM_StateTypeDef HAL_TIMEx_HallSensor_GetState(TIM_HandleTypeDef* htim);


  



  

 
 
 
 


 
#line 332 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim_ex.h"

#line 339 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim_ex.h"




   
  
 


 
  


 



  



 
    






 
#line 1290 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim.h"

 


 



 

 
HAL_StatusTypeDef HAL_TIM_Base_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_Base_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_Base_Stop(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_Base_Stop_IT(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_Base_Start_DMA(TIM_HandleTypeDef *htim, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_Base_Stop_DMA(TIM_HandleTypeDef *htim);


 



 
 
HAL_StatusTypeDef HAL_TIM_OC_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_OC_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_OC_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_OC_MspDeInit(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_OC_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_OC_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_OC_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_OC_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_OC_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_OC_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);



 



 
 
HAL_StatusTypeDef HAL_TIM_PWM_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_PWM_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_PWM_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_PWM_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_PWM_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_PWM_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);



 



 
 
HAL_StatusTypeDef HAL_TIM_IC_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_IC_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_IC_MspDeInit(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_IC_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_IC_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_IC_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_IC_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_IC_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_IC_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);



 



 
 
HAL_StatusTypeDef HAL_TIM_OnePulse_Init(TIM_HandleTypeDef *htim, uint32_t OnePulseMode);
HAL_StatusTypeDef HAL_TIM_OnePulse_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_OnePulse_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_OnePulse_MspDeInit(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_OnePulse_Start(TIM_HandleTypeDef *htim, uint32_t OutputChannel);
HAL_StatusTypeDef HAL_TIM_OnePulse_Stop(TIM_HandleTypeDef *htim, uint32_t OutputChannel);

 
HAL_StatusTypeDef HAL_TIM_OnePulse_Start_IT(TIM_HandleTypeDef *htim, uint32_t OutputChannel);
HAL_StatusTypeDef HAL_TIM_OnePulse_Stop_IT(TIM_HandleTypeDef *htim, uint32_t OutputChannel);



 



 
 
HAL_StatusTypeDef HAL_TIM_Encoder_Init(TIM_HandleTypeDef *htim,  TIM_Encoder_InitTypeDef* sConfig);
HAL_StatusTypeDef HAL_TIM_Encoder_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_Encoder_MspDeInit(TIM_HandleTypeDef *htim);
  
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_Encoder_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_Encoder_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_Encoder_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_Encoder_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData1, uint32_t *pData2, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_Encoder_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);



 



 
 
void HAL_TIM_IRQHandler(TIM_HandleTypeDef *htim);



 



 
 
HAL_StatusTypeDef HAL_TIM_OC_ConfigChannel(TIM_HandleTypeDef *htim, TIM_OC_InitTypeDef* sConfig, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef *htim, TIM_OC_InitTypeDef* sConfig, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_IC_ConfigChannel(TIM_HandleTypeDef *htim, TIM_IC_InitTypeDef* sConfig, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_OnePulse_ConfigChannel(TIM_HandleTypeDef *htim, TIM_OnePulse_InitTypeDef* sConfig, uint32_t OutputChannel,  uint32_t InputChannel);
HAL_StatusTypeDef HAL_TIM_ConfigOCrefClear(TIM_HandleTypeDef *htim, TIM_ClearInputConfigTypeDef * sClearInputConfig, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_ConfigClockSource(TIM_HandleTypeDef *htim, TIM_ClockConfigTypeDef * sClockSourceConfig);    
HAL_StatusTypeDef HAL_TIM_ConfigTI1Input(TIM_HandleTypeDef *htim, uint32_t TI1_Selection);
HAL_StatusTypeDef HAL_TIM_SlaveConfigSynchronization(TIM_HandleTypeDef *htim, TIM_SlaveConfigTypeDef * sSlaveConfig);
HAL_StatusTypeDef HAL_TIM_SlaveConfigSynchronization_IT(TIM_HandleTypeDef *htim, TIM_SlaveConfigTypeDef * sSlaveConfig);
HAL_StatusTypeDef HAL_TIM_DMABurst_WriteStart(TIM_HandleTypeDef *htim, uint32_t BurstBaseAddress, uint32_t BurstRequestSrc,                                               uint32_t  *BurstBuffer, uint32_t  BurstLength);

HAL_StatusTypeDef HAL_TIM_DMABurst_WriteStop(TIM_HandleTypeDef *htim, uint32_t BurstRequestSrc);
HAL_StatusTypeDef HAL_TIM_DMABurst_ReadStart(TIM_HandleTypeDef *htim, uint32_t BurstBaseAddress, uint32_t BurstRequestSrc,                                               uint32_t  *BurstBuffer, uint32_t  BurstLength);

HAL_StatusTypeDef HAL_TIM_DMABurst_ReadStop(TIM_HandleTypeDef *htim, uint32_t BurstRequestSrc);
HAL_StatusTypeDef HAL_TIM_GenerateEvent(TIM_HandleTypeDef *htim, uint32_t EventSource);
uint32_t HAL_TIM_ReadCapturedValue(TIM_HandleTypeDef *htim, uint32_t Channel);



 



 
 
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_TriggerCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *htim);



 



 
 
HAL_TIM_StateTypeDef HAL_TIM_Base_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_OC_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_PWM_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_IC_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_OnePulse_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_Encoder_GetState(TIM_HandleTypeDef *htim);



 
  


 
  
 


 



 












                              
#line 1523 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim.h"





















































#line 1586 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim.h"















































#line 1641 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim.h"










#line 1659 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim.h"























#line 1702 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim.h"

#line 1721 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_tim.h"




  



   

 




 
  


 

 


 
void TIM_Base_SetConfig(TIM_TypeDef *TIMx, TIM_Base_InitTypeDef *Structure);
void TIM_TI1_SetConfig(TIM_TypeDef *TIMx, uint32_t TIM_ICPolarity, uint32_t TIM_ICSelection, uint32_t TIM_ICFilter);
void TIM_OC2_SetConfig(TIM_TypeDef *TIMx, TIM_OC_InitTypeDef *OC_Config);
void TIM_DMADelayPulseCplt(DMA_HandleTypeDef *hdma);
void TIM_DMAError(DMA_HandleTypeDef *hdma);
void TIM_DMACaptureCplt(DMA_HandleTypeDef *hdma);
void TIM_CCxChannelCmd(TIM_TypeDef* TIMx, uint32_t Channel, uint32_t ChannelState);  


  
      


  



  
  






 
#line 365 "../Inc/stm32f4xx_hal_conf.h"


#line 1 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_uart.h"

































  

 







 
#line 46 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_uart.h"



 



  

  


 



  
typedef struct
{
  uint32_t BaudRate;                  



 

  uint32_t WordLength;                
 

  uint32_t StopBits;                  
 

  uint32_t Parity;                    




 
 
  uint32_t Mode;                      
 

  uint32_t HwFlowCtl;                 

 
  
  uint32_t OverSampling;              
  
}UART_InitTypeDef;







































  
typedef enum
{
  HAL_UART_STATE_RESET             = 0x00U,    
 
  HAL_UART_STATE_READY             = 0x20U,    
 
  HAL_UART_STATE_BUSY              = 0x24U,    
 
  HAL_UART_STATE_BUSY_TX           = 0x21U,    
 
  HAL_UART_STATE_BUSY_RX           = 0x22U,    
 
  HAL_UART_STATE_BUSY_TX_RX        = 0x23U,    

 
  HAL_UART_STATE_TIMEOUT           = 0xA0U,    
 
  HAL_UART_STATE_ERROR             = 0xE0U     
 
}HAL_UART_StateTypeDef;



   
typedef struct
{
  USART_TypeDef                 *Instance;         
  
  UART_InitTypeDef              Init;              
  
  uint8_t                       *pTxBuffPtr;       
  
  uint16_t                      TxXferSize;        
  
  volatile uint16_t                 TxXferCount;       
  
  uint8_t                       *pRxBuffPtr;       
  
  uint16_t                      RxXferSize;        
  
  volatile uint16_t                 RxXferCount;         
  
  DMA_HandleTypeDef             *hdmatx;           
    
  DMA_HandleTypeDef             *hdmarx;           
  
  HAL_LockTypeDef               Lock;              

  volatile HAL_UART_StateTypeDef    gState;           

 
  
  volatile HAL_UART_StateTypeDef    RxState;          
 
  
  volatile uint32_t                 ErrorCode;         

}UART_HandleTypeDef;


 

 


 




  
#line 211 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_uart.h"


 



 




 



 




  



  





  



  






 



  





 
    
 

  




 



 




 



   




 
                                         


 




 





 
#line 315 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_uart.h"


 










  













 



 
  
 


 






 







 



















 


     





















 







 
#line 427 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_uart.h"
                                              





 







 







 







 

                                                 















 



















 


















 















 


















 


















 


















 









      





       





     





     



 
    
 


 
  


     
 
HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_HalfDuplex_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_LIN_Init(UART_HandleTypeDef *huart, uint32_t BreakDetectLength);
HAL_StatusTypeDef HAL_MultiProcessor_Init(UART_HandleTypeDef *huart, uint8_t Address, uint32_t WakeUpMethod);
HAL_StatusTypeDef HAL_UART_DeInit (UART_HandleTypeDef *huart);
void HAL_UART_MspInit(UART_HandleTypeDef *huart);
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart);


 



 
 
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_DMAPause(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_DMAResume(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *huart);
 
HAL_StatusTypeDef HAL_UART_Abort(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_Abort_IT(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_AbortTransmit_IT(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_AbortReceive_IT(UART_HandleTypeDef *huart);

void HAL_UART_IRQHandler(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
void HAL_UART_AbortCpltCallback (UART_HandleTypeDef *huart);
void HAL_UART_AbortTransmitCpltCallback (UART_HandleTypeDef *huart);
void HAL_UART_AbortReceiveCpltCallback (UART_HandleTypeDef *huart);


 



 
 
HAL_StatusTypeDef HAL_LIN_SendBreak(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_MultiProcessor_EnterMuteMode(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_MultiProcessor_ExitMuteMode(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_HalfDuplex_EnableTransmitter(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_HalfDuplex_EnableReceiver(UART_HandleTypeDef *huart);


 



 
 
HAL_UART_StateTypeDef HAL_UART_GetState(UART_HandleTypeDef *huart);
uint32_t              HAL_UART_GetError(UART_HandleTypeDef *huart);


  



  
 
 
 


 


  





 

 


 
#line 746 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal_uart.h"





 








 






 

 


 



 



  



 







 
#line 369 "../Inc/stm32f4xx_hal_conf.h"

























   



























   
 
#line 439 "../Inc/stm32f4xx_hal_conf.h"






 

 
#line 47 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal.h"



 



  

 
 



 



 
typedef enum
{
  HAL_TICK_FREQ_10HZ         = 100U,
  HAL_TICK_FREQ_100HZ        = 10U,
  HAL_TICK_FREQ_1KHZ         = 1U,
  HAL_TICK_FREQ_DEFAULT      = HAL_TICK_FREQ_1KHZ
} HAL_TickFreqTypeDef;


 



 
   
 


 


 
#line 110 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal.h"

#line 133 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal.h"


 



 





 






 





#line 172 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal.h"

#line 202 "../Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_hal.h"


 



 





 

 


 


 
 
HAL_StatusTypeDef HAL_Init(void);
HAL_StatusTypeDef HAL_DeInit(void);
void HAL_MspInit(void);
void HAL_MspDeInit(void);
HAL_StatusTypeDef HAL_InitTick (uint32_t TickPriority);


 



 
 
void HAL_IncTick(void);
void HAL_Delay(uint32_t Delay);
uint32_t HAL_GetTick(void);
uint32_t HAL_GetTickPrio(void);
HAL_StatusTypeDef HAL_SetTickFreq(HAL_TickFreqTypeDef Freq);
HAL_TickFreqTypeDef HAL_GetTickFreq(void);
void HAL_SuspendTick(void);
void HAL_ResumeTick(void);
uint32_t HAL_GetHalVersion(void);
uint32_t HAL_GetREVID(void);
uint32_t HAL_GetDEVID(void);
void HAL_DBGMCU_EnableDBGSleepMode(void);
void HAL_DBGMCU_DisableDBGSleepMode(void);
void HAL_DBGMCU_EnableDBGStopMode(void);
void HAL_DBGMCU_DisableDBGStopMode(void);
void HAL_DBGMCU_EnableDBGStandbyMode(void);
void HAL_DBGMCU_DisableDBGStandbyMode(void);
void HAL_EnableCompensationCell(void);
void HAL_DisableCompensationCell(void);
void HAL_GetUID(uint32_t *UID);







 



 
 
 


 


 
 


 


 
 
 


 



  
  






 
#line 86 "../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c"



 




 



 
 


 

 









 

 


 


 
 
 



 










































































 













 
__weak HAL_StatusTypeDef HAL_RCC_DeInit(void)
{
  return HAL_OK;
}














 
__weak HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef  *RCC_OscInitStruct)
{
  uint32_t tickstart;

   
  if(RCC_OscInitStruct == 0)
  {
    return HAL_ERROR;
  }

   
  ((void)0U);
   
  if(((RCC_OscInitStruct->OscillatorType) & 0x00000001U) == 0x00000001U)
  {
     
    ((void)0U);
     
    if(((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CFGR & (0x3U << (2U))) == 0x00000004U) ||      (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CFGR & (0x3U << (2U))) == 0x00000008U) && ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->PLLCFGR & (0x1U << (22U))) == (0x1U << (22U)))))

    {
      if(((((((((((uint8_t)0x31)) >> 5U) == 1U)? ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CR :((((((uint8_t)0x31)) >> 5U) == 2U) ? ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->BDCR :((((((uint8_t)0x31)) >> 5U) == 3U)? ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CSR :((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CIR))) & (1U << ((((uint8_t)0x31)) & ((uint8_t)0x1FU))))!= 0U)? 1U : 0U) != RESET) && (RCC_OscInitStruct->HSEState == 0x00000000U))
      {
        return HAL_ERROR;
      }
    }
    else
    {
       
      do { if ((RCC_OscInitStruct->HSEState) == (0x1U << (16U))) { ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CR) |= ((0x1U << (16U)))); } else if ((RCC_OscInitStruct->HSEState) == ((uint32_t)((0x1U << (18U)) | (0x1U << (16U))))) { ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CR) |= ((0x1U << (18U)))); ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CR) |= ((0x1U << (16U)))); } else { ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CR) &= ~((0x1U << (16U)))); ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CR) &= ~((0x1U << (18U)))); } } while(0U);

       
      if((RCC_OscInitStruct->HSEState) != 0x00000000U)
      {
         
        tickstart = HAL_GetTick();

         
        while((((((((((uint8_t)0x31)) >> 5U) == 1U)? ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CR :((((((uint8_t)0x31)) >> 5U) == 2U) ? ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->BDCR :((((((uint8_t)0x31)) >> 5U) == 3U)? ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CSR :((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CIR))) & (1U << ((((uint8_t)0x31)) & ((uint8_t)0x1FU))))!= 0U)? 1U : 0U) == RESET)
        {
          if((HAL_GetTick() - tickstart ) > ((uint32_t)100U))
          {
            return HAL_TIMEOUT;
          }
        }
      }
      else
      {
         
        tickstart = HAL_GetTick();

         
        while((((((((((uint8_t)0x31)) >> 5U) == 1U)? ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CR :((((((uint8_t)0x31)) >> 5U) == 2U) ? ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->BDCR :((((((uint8_t)0x31)) >> 5U) == 3U)? ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CSR :((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CIR))) & (1U << ((((uint8_t)0x31)) & ((uint8_t)0x1FU))))!= 0U)? 1U : 0U) != RESET)
        {
          if((HAL_GetTick() - tickstart ) > ((uint32_t)100U))
          {
            return HAL_TIMEOUT;
          }
        }
      }
    }
  }
   
  if(((RCC_OscInitStruct->OscillatorType) & 0x00000002U) == 0x00000002U)
  {
     
    ((void)0U);
    ((void)0U);

     
    if(((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CFGR & (0x3U << (2U))) == 0x00000000U) ||      (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CFGR & (0x3U << (2U))) == 0x00000008U) && ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->PLLCFGR & (0x1U << (22U))) == 0x00000000U)))

    {
       
      if(((((((((((uint8_t)0x21)) >> 5U) == 1U)? ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CR :((((((uint8_t)0x21)) >> 5U) == 2U) ? ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->BDCR :((((((uint8_t)0x21)) >> 5U) == 3U)? ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CSR :((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CIR))) & (1U << ((((uint8_t)0x21)) & ((uint8_t)0x1FU))))!= 0U)? 1U : 0U) != RESET) && (RCC_OscInitStruct->HSIState != ((uint8_t)0x01)))
      {
        return HAL_ERROR;
      }
       
      else
      {
         
        ((((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CR)) = ((((((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CR))) & (~((0x1FU << (3U))))) | ((uint32_t)(RCC_OscInitStruct->HSICalibrationValue) << (3U))))));
      }
    }
    else
    {
       
      if((RCC_OscInitStruct->HSIState)!= ((uint8_t)0x00))
      {
         
        (*(volatile uint32_t *) (0x42000000U + (((((0x40000000U + 0x00020000U) + 0x3800U) - 0x40000000U) + 0x00U) * 32U) + (0x00U * 4U)) = ENABLE);

         
        tickstart = HAL_GetTick();

         
        while((((((((((uint8_t)0x21)) >> 5U) == 1U)? ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CR :((((((uint8_t)0x21)) >> 5U) == 2U) ? ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->BDCR :((((((uint8_t)0x21)) >> 5U) == 3U)? ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CSR :((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CIR))) & (1U << ((((uint8_t)0x21)) & ((uint8_t)0x1FU))))!= 0U)? 1U : 0U) == RESET)
        {
          if((HAL_GetTick() - tickstart ) > 2U)
          {
            return HAL_TIMEOUT;
          }
        }

         
        ((((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CR)) = ((((((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CR))) & (~((0x1FU << (3U))))) | ((uint32_t)(RCC_OscInitStruct->HSICalibrationValue) << (3U))))));
      }
      else
      {
         
        (*(volatile uint32_t *) (0x42000000U + (((((0x40000000U + 0x00020000U) + 0x3800U) - 0x40000000U) + 0x00U) * 32U) + (0x00U * 4U)) = DISABLE);

         
        tickstart = HAL_GetTick();

         
        while((((((((((uint8_t)0x21)) >> 5U) == 1U)? ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CR :((((((uint8_t)0x21)) >> 5U) == 2U) ? ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->BDCR :((((((uint8_t)0x21)) >> 5U) == 3U)? ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CSR :((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CIR))) & (1U << ((((uint8_t)0x21)) & ((uint8_t)0x1FU))))!= 0U)? 1U : 0U) != RESET)
        {
          if((HAL_GetTick() - tickstart ) > 2U)
          {
            return HAL_TIMEOUT;
          }
        }
      }
    }
  }
   
  if(((RCC_OscInitStruct->OscillatorType) & 0x00000008U) == 0x00000008U)
  {
     
    ((void)0U);

     
    if((RCC_OscInitStruct->LSIState)!= ((uint8_t)0x00))
    {
       
      (*(volatile uint32_t *) (0x42000000U + (((((0x40000000U + 0x00020000U) + 0x3800U) - 0x40000000U) + 0x74U) * 32U) + (0x00U * 4U)) = ENABLE);

       
      tickstart = HAL_GetTick();

       
      while((((((((((uint8_t)0x61)) >> 5U) == 1U)? ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CR :((((((uint8_t)0x61)) >> 5U) == 2U) ? ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->BDCR :((((((uint8_t)0x61)) >> 5U) == 3U)? ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CSR :((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CIR))) & (1U << ((((uint8_t)0x61)) & ((uint8_t)0x1FU))))!= 0U)? 1U : 0U) == RESET)
      {
        if((HAL_GetTick() - tickstart ) > 2U)
        {
          return HAL_TIMEOUT;
        }
      }
    }
    else
    {
       
      (*(volatile uint32_t *) (0x42000000U + (((((0x40000000U + 0x00020000U) + 0x3800U) - 0x40000000U) + 0x74U) * 32U) + (0x00U * 4U)) = DISABLE);

       
      tickstart = HAL_GetTick();

       
      while((((((((((uint8_t)0x61)) >> 5U) == 1U)? ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CR :((((((uint8_t)0x61)) >> 5U) == 2U) ? ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->BDCR :((((((uint8_t)0x61)) >> 5U) == 3U)? ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CSR :((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CIR))) & (1U << ((((uint8_t)0x61)) & ((uint8_t)0x1FU))))!= 0U)? 1U : 0U) != RESET)
      {
        if((HAL_GetTick() - tickstart ) > 2U)
        {
          return HAL_TIMEOUT;
        }
      }
    }
  }
   
  if(((RCC_OscInitStruct->OscillatorType) & 0x00000004U) == 0x00000004U)
  {
    FlagStatus       pwrclkchanged = RESET;

     
    ((void)0U);

     
     
    if(((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->APB1ENR & ((0x1U << (28U)))) == RESET))
    {
      do { volatile uint32_t tmpreg = 0x00U; ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->APB1ENR) |= ((0x1U << (28U)))); tmpreg = ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->APB1ENR) & ((0x1U << (28U)))); (void)tmpreg; } while(0U);
      pwrclkchanged = SET;
    }

    if((((((PWR_TypeDef *) (0x40000000U + 0x7000U))->CR) & ((0x1U << (8U)))) == RESET))
    {
       
      ((((PWR_TypeDef *) (0x40000000U + 0x7000U))->CR) |= ((0x1U << (8U))));

       
      tickstart = HAL_GetTick();

      while((((((PWR_TypeDef *) (0x40000000U + 0x7000U))->CR) & ((0x1U << (8U)))) == RESET))
      {
        if((HAL_GetTick() - tickstart) > 2U)
        {
          return HAL_TIMEOUT;
        }
      }
    }

     
    do { if((RCC_OscInitStruct->LSEState) == (0x1U << (0U))) { ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->BDCR) |= ((0x1U << (0U)))); } else if((RCC_OscInitStruct->LSEState) == ((uint32_t)((0x1U << (2U)) | (0x1U << (0U))))) { ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->BDCR) |= ((0x1U << (2U)))); ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->BDCR) |= ((0x1U << (0U)))); } else { ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->BDCR) &= ~((0x1U << (0U)))); ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->BDCR) &= ~((0x1U << (2U)))); } } while(0U);
     
    if((RCC_OscInitStruct->LSEState) != 0x00000000U)
    {
       
      tickstart = HAL_GetTick();

       
      while((((((((((uint8_t)0x41)) >> 5U) == 1U)? ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CR :((((((uint8_t)0x41)) >> 5U) == 2U) ? ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->BDCR :((((((uint8_t)0x41)) >> 5U) == 3U)? ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CSR :((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CIR))) & (1U << ((((uint8_t)0x41)) & ((uint8_t)0x1FU))))!= 0U)? 1U : 0U) == RESET)
      {
        if((HAL_GetTick() - tickstart ) > ((uint32_t)5000U))
        {
          return HAL_TIMEOUT;
        }
      }
    }
    else
    {
       
      tickstart = HAL_GetTick();

       
      while((((((((((uint8_t)0x41)) >> 5U) == 1U)? ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CR :((((((uint8_t)0x41)) >> 5U) == 2U) ? ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->BDCR :((((((uint8_t)0x41)) >> 5U) == 3U)? ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CSR :((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CIR))) & (1U << ((((uint8_t)0x41)) & ((uint8_t)0x1FU))))!= 0U)? 1U : 0U) != RESET)
      {
        if((HAL_GetTick() - tickstart ) > ((uint32_t)5000U))
        {
          return HAL_TIMEOUT;
        }
      }
    }

     
    if(pwrclkchanged == SET)
    {
      (((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->APB1ENR &= ~((0x1U << (28U))));
    }
  }
   
   
  ((void)0U);
  if ((RCC_OscInitStruct->PLL.PLLState) != ((uint8_t)0x00))
  {
     
    if((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CFGR & (0x3U << (2U))) != 0x00000008U)
    {
      if((RCC_OscInitStruct->PLL.PLLState) == ((uint8_t)0x02))
      {
         
        ((void)0U);
        ((void)0U);
        ((void)0U);
        ((void)0U);
        ((void)0U);

         
        (*(volatile uint32_t *) (0x42000000U + (((((0x40000000U + 0x00020000U) + 0x3800U) - 0x40000000U) + 0x00U) * 32U) + (0x18U * 4U)) = DISABLE);

         
        tickstart = HAL_GetTick();

         
        while((((((((((uint8_t)0x39)) >> 5U) == 1U)? ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CR :((((((uint8_t)0x39)) >> 5U) == 2U) ? ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->BDCR :((((((uint8_t)0x39)) >> 5U) == 3U)? ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CSR :((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CIR))) & (1U << ((((uint8_t)0x39)) & ((uint8_t)0x1FU))))!= 0U)? 1U : 0U) != RESET)
        {
          if((HAL_GetTick() - tickstart ) > 2U)
          {
            return HAL_TIMEOUT;
          }
        }

         
        ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->PLLCFGR) = ((RCC_OscInitStruct->PLL . PLLSource | RCC_OscInitStruct->PLL . PLLM | (RCC_OscInitStruct->PLL . PLLN << (6U)) | (((RCC_OscInitStruct->PLL . PLLP >> 1U) - 1U) << (16U)) | (RCC_OscInitStruct->PLL . PLLQ << (24U)))));




         
        (*(volatile uint32_t *) (0x42000000U + (((((0x40000000U + 0x00020000U) + 0x3800U) - 0x40000000U) + 0x00U) * 32U) + (0x18U * 4U)) = ENABLE);

         
        tickstart = HAL_GetTick();

         
        while((((((((((uint8_t)0x39)) >> 5U) == 1U)? ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CR :((((((uint8_t)0x39)) >> 5U) == 2U) ? ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->BDCR :((((((uint8_t)0x39)) >> 5U) == 3U)? ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CSR :((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CIR))) & (1U << ((((uint8_t)0x39)) & ((uint8_t)0x1FU))))!= 0U)? 1U : 0U) == RESET)
        {
          if((HAL_GetTick() - tickstart ) > 2U)
          {
            return HAL_TIMEOUT;
          }
        }
      }
      else
      {
         
        (*(volatile uint32_t *) (0x42000000U + (((((0x40000000U + 0x00020000U) + 0x3800U) - 0x40000000U) + 0x00U) * 32U) + (0x18U * 4U)) = DISABLE);

         
        tickstart = HAL_GetTick();

         
        while((((((((((uint8_t)0x39)) >> 5U) == 1U)? ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CR :((((((uint8_t)0x39)) >> 5U) == 2U) ? ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->BDCR :((((((uint8_t)0x39)) >> 5U) == 3U)? ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CSR :((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CIR))) & (1U << ((((uint8_t)0x39)) & ((uint8_t)0x1FU))))!= 0U)? 1U : 0U) != RESET)
        {
          if((HAL_GetTick() - tickstart ) > 2U)
          {
            return HAL_TIMEOUT;
          }
        }
      }
    }
    else
    {
      return HAL_ERROR;
    }
  }
  return HAL_OK;
}

























 
HAL_StatusTypeDef HAL_RCC_ClockConfig(RCC_ClkInitTypeDef  *RCC_ClkInitStruct, uint32_t FLatency)
{
  uint32_t tickstart;

   
  if(RCC_ClkInitStruct == 0)
  {
    return HAL_ERROR;
  }

   
  ((void)0U);
  ((void)0U);

  

 

   
  if(FLatency > ((((((FLASH_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3C00U))->ACR)) & ((0xFU << (0U))))))
  {
     
    (*(volatile uint8_t *)0x40023C00U = (uint8_t)(FLatency));

    
 
    if(((((((FLASH_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3C00U))->ACR)) & ((0xFU << (0U))))) != FLatency)
    {
      return HAL_ERROR;
    }
  }

   
  if(((RCC_ClkInitStruct->ClockType) & 0x00000002U) == 0x00000002U)
  {
    
 
    if(((RCC_ClkInitStruct->ClockType) & 0x00000004U) == 0x00000004U)
    {
      (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CFGR)) = ((((((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CFGR))) & (~((0x7U << (10U))))) | (0x00001C00U))));
    }

    if(((RCC_ClkInitStruct->ClockType) & 0x00000008U) == 0x00000008U)
    {
      (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CFGR)) = ((((((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CFGR))) & (~((0x7U << (13U))))) | ((0x00001C00U << 3)))));
    }

    ((void)0U);
    (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CFGR)) = ((((((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CFGR))) & (~((0xFU << (4U))))) | (RCC_ClkInitStruct->AHBCLKDivider))));
  }

   
  if(((RCC_ClkInitStruct->ClockType) & 0x00000001U) == 0x00000001U)
  {
    ((void)0U);

     
    if(RCC_ClkInitStruct->SYSCLKSource == 0x00000001U)
    {
       
      if((((((((((uint8_t)0x31)) >> 5U) == 1U)? ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CR :((((((uint8_t)0x31)) >> 5U) == 2U) ? ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->BDCR :((((((uint8_t)0x31)) >> 5U) == 3U)? ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CSR :((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CIR))) & (1U << ((((uint8_t)0x31)) & ((uint8_t)0x1FU))))!= 0U)? 1U : 0U) == RESET)
      {
        return HAL_ERROR;
      }
    }
     
    else if((RCC_ClkInitStruct->SYSCLKSource == 0x00000002U)   ||
            (RCC_ClkInitStruct->SYSCLKSource == ((uint32_t)((0x1U << (0U)) | (0x2U << (0U))))))
    {
       
      if((((((((((uint8_t)0x39)) >> 5U) == 1U)? ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CR :((((((uint8_t)0x39)) >> 5U) == 2U) ? ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->BDCR :((((((uint8_t)0x39)) >> 5U) == 3U)? ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CSR :((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CIR))) & (1U << ((((uint8_t)0x39)) & ((uint8_t)0x1FU))))!= 0U)? 1U : 0U) == RESET)
      {
        return HAL_ERROR;
      }
    }
     
    else
    {
       
      if((((((((((uint8_t)0x21)) >> 5U) == 1U)? ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CR :((((((uint8_t)0x21)) >> 5U) == 2U) ? ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->BDCR :((((((uint8_t)0x21)) >> 5U) == 3U)? ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CSR :((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CIR))) & (1U << ((((uint8_t)0x21)) & ((uint8_t)0x1FU))))!= 0U)? 1U : 0U) == RESET)
      {
        return HAL_ERROR;
      }
    }

    (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CFGR)) = ((((((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CFGR))) & (~((0x3U << (0U))))) | ((RCC_ClkInitStruct->SYSCLKSource)))));

     
    tickstart = HAL_GetTick();

    while ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CFGR & (0x3U << (2U))) != (RCC_ClkInitStruct->SYSCLKSource << (2U)))
    {
      if ((HAL_GetTick() - tickstart) > 5000U)
      {
        return HAL_TIMEOUT;
      }
    }
  }

   
  if(FLatency < ((((((FLASH_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3C00U))->ACR)) & ((0xFU << (0U))))))
  {
      
    (*(volatile uint8_t *)0x40023C00U = (uint8_t)(FLatency));

    
 
    if(((((((FLASH_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3C00U))->ACR)) & ((0xFU << (0U))))) != FLatency)
    {
      return HAL_ERROR;
    }
  }

   
  if(((RCC_ClkInitStruct->ClockType) & 0x00000004U) == 0x00000004U)
  {
    ((void)0U);
    (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CFGR)) = ((((((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CFGR))) & (~((0x7U << (10U))))) | (RCC_ClkInitStruct->APB1CLKDivider))));
  }

   
  if(((RCC_ClkInitStruct->ClockType) & 0x00000008U) == 0x00000008U)
  {
    ((void)0U);
    (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CFGR)) = ((((((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CFGR))) & (~((0x7U << (13U))))) | (((RCC_ClkInitStruct->APB2CLKDivider) << 3U)))));
  }

   
  SystemCoreClock = HAL_RCC_GetSysClockFreq() >> AHBPrescTable[(((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CFGR & (0xFU << (4U)))>> (4U)];

   
  HAL_InitTick (((uint32_t)0U));

  return HAL_OK;
}



 














 





























 
void HAL_RCC_MCOConfig(uint32_t RCC_MCOx, uint32_t RCC_MCOSource, uint32_t RCC_MCODiv)
{
  GPIO_InitTypeDef GPIO_InitStruct;
   
  ((void)0U);
  ((void)0U);
   
  if(RCC_MCOx == 0x00000000U)
  {
    ((void)0U);

     
    do { volatile uint32_t tmpreg = 0x00U; ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->AHB1ENR) |= ((0x1U << (0U)))); tmpreg = ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->AHB1ENR) & ((0x1U << (0U)))); (void)tmpreg; } while(0U);

     
    GPIO_InitStruct.Pin = ((uint16_t)0x0100);
    GPIO_InitStruct.Mode = 0x00000002U;
    GPIO_InitStruct.Speed = 0x00000003U;
    GPIO_InitStruct.Pull = 0x00000000U;
    GPIO_InitStruct.Alternate = ((uint8_t)0x00);
    HAL_GPIO_Init(((GPIO_TypeDef *) ((0x40000000U + 0x00020000U) + 0x0000U)), &GPIO_InitStruct);

     
    (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CFGR)) = ((((((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CFGR))) & (~(((0x3U << (21U)) | (0x7U << (24U)))))) | ((RCC_MCOSource | RCC_MCODiv)))));

    



  }

  else
  {
    ((void)0U);

     
    do { volatile uint32_t tmpreg = 0x00U; ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->AHB1ENR) |= ((0x1U << (2U)))); tmpreg = ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->AHB1ENR) & ((0x1U << (2U)))); (void)tmpreg; } while(0U);

     
    GPIO_InitStruct.Pin = ((uint16_t)0x0200);
    GPIO_InitStruct.Mode = 0x00000002U;
    GPIO_InitStruct.Speed = 0x00000003U;
    GPIO_InitStruct.Pull = 0x00000000U;
    GPIO_InitStruct.Alternate = ((uint8_t)0x00);
    HAL_GPIO_Init(((GPIO_TypeDef *) ((0x40000000U + 0x00020000U) + 0x0800U)), &GPIO_InitStruct);

     
    (((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CFGR)) = ((((((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CFGR))) & (~(((0x3U << (30U)) | (0x7U << (27U)))))) | ((RCC_MCOSource | (RCC_MCODiv << 3U))))));

    



  }

}









 
void HAL_RCC_EnableCSS(void)
{
  *(volatile uint32_t *) (0x42000000U + (((((0x40000000U + 0x00020000U) + 0x3800U) - 0x40000000U) + 0x00U) * 32U) + (0x13U * 4U)) = (uint32_t)ENABLE;
}




 
void HAL_RCC_DisableCSS(void)
{
  *(volatile uint32_t *) (0x42000000U + (((((0x40000000U + 0x00020000U) + 0x3800U) - 0x40000000U) + 0x00U) * 32U) + (0x13U * 4U)) = (uint32_t)DISABLE;
}






























 
__weak uint32_t HAL_RCC_GetSysClockFreq(void)
{
  uint32_t pllm = 0U, pllvco = 0U, pllp = 0U;
  uint32_t sysclockfreq = 0U;

   
  switch (((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CFGR & (0x3U << (2U)))
  {
    case 0x00000000U:   
    {
      sysclockfreq = 16000000U;
       break;
    }
    case 0x00000004U:   
    {
      sysclockfreq = 25000000U;
      break;
    }
    case 0x00000008U:   
    {
      
 
      pllm = ((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->PLLCFGR & (0x3FU << (0U));
      if(((uint32_t)(((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->PLLCFGR & (0x1U << (22U)))) != 0x00000000U)
      {
         
        pllvco = (uint32_t) ((((uint64_t) 25000000U * ((uint64_t) ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->PLLCFGR & (0x1FFU << (6U))) >> (6U))))) / (uint64_t)pllm);
      }
      else
      {
         
        pllvco = (uint32_t) ((((uint64_t) 16000000U * ((uint64_t) ((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->PLLCFGR & (0x1FFU << (6U))) >> (6U))))) / (uint64_t)pllm);
      }
      pllp = ((((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->PLLCFGR & (0x3U << (16U))) >> (16U)) + 1U) *2U);

      sysclockfreq = pllvco/pllp;
      break;
    }
    default:
    {
      sysclockfreq = 16000000U;
      break;
    }
  }
  return sysclockfreq;
}









 
uint32_t HAL_RCC_GetHCLKFreq(void)
{
  return SystemCoreClock;
}






 
uint32_t HAL_RCC_GetPCLK1Freq(void)
{
   
  return (HAL_RCC_GetHCLKFreq() >> APBPrescTable[(((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CFGR & (0x7U << (10U)))>> (10U)]);
}






 
uint32_t HAL_RCC_GetPCLK2Freq(void)
{
   
  return (HAL_RCC_GetHCLKFreq()>> APBPrescTable[(((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CFGR & (0x7U << (13U)))>> (13U)]);
}







 
__weak void HAL_RCC_GetOscConfig(RCC_OscInitTypeDef  *RCC_OscInitStruct)
{
   
  RCC_OscInitStruct->OscillatorType = 0x00000001U | 0x00000002U | 0x00000004U | 0x00000008U;

   
  if((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CR &(0x1U << (18U))) == (0x1U << (18U)))
  {
    RCC_OscInitStruct->HSEState = ((uint32_t)((0x1U << (18U)) | (0x1U << (16U))));
  }
  else if((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CR &(0x1U << (16U))) == (0x1U << (16U)))
  {
    RCC_OscInitStruct->HSEState = (0x1U << (16U));
  }
  else
  {
    RCC_OscInitStruct->HSEState = 0x00000000U;
  }

   
  if((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CR &(0x1U << (0U))) == (0x1U << (0U)))
  {
    RCC_OscInitStruct->HSIState = ((uint8_t)0x01);
  }
  else
  {
    RCC_OscInitStruct->HSIState = ((uint8_t)0x00);
  }

  RCC_OscInitStruct->HSICalibrationValue = (uint32_t)((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CR &(0x1FU << (3U))) >> (3U));

   
  if((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->BDCR &(0x1U << (2U))) == (0x1U << (2U)))
  {
    RCC_OscInitStruct->LSEState = ((uint32_t)((0x1U << (2U)) | (0x1U << (0U))));
  }
  else if((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->BDCR &(0x1U << (0U))) == (0x1U << (0U)))
  {
    RCC_OscInitStruct->LSEState = (0x1U << (0U));
  }
  else
  {
    RCC_OscInitStruct->LSEState = 0x00000000U;
  }

   
  if((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CSR &(0x1U << (0U))) == (0x1U << (0U)))
  {
    RCC_OscInitStruct->LSIState = ((uint8_t)0x01);
  }
  else
  {
    RCC_OscInitStruct->LSIState = ((uint8_t)0x00);
  }

   
  if((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CR &(0x1U << (24U))) == (0x1U << (24U)))
  {
    RCC_OscInitStruct->PLL.PLLState = ((uint8_t)0x02);
  }
  else
  {
    RCC_OscInitStruct->PLL.PLLState = ((uint8_t)0x01);
  }
  RCC_OscInitStruct->PLL.PLLSource = (uint32_t)(((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->PLLCFGR & (0x1U << (22U)));
  RCC_OscInitStruct->PLL.PLLM = (uint32_t)(((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->PLLCFGR & (0x3FU << (0U)));
  RCC_OscInitStruct->PLL.PLLN = (uint32_t)((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->PLLCFGR & (0x1FFU << (6U))) >> (6U));
  RCC_OscInitStruct->PLL.PLLP = (uint32_t)((((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->PLLCFGR & (0x3U << (16U))) + (0x1U << (16U))) << 1U) >> (16U));
  RCC_OscInitStruct->PLL.PLLQ = (uint32_t)((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->PLLCFGR & (0xFU << (24U))) >> (24U));
}








 
void HAL_RCC_GetClockConfig(RCC_ClkInitTypeDef  *RCC_ClkInitStruct, uint32_t *pFLatency)
{
   
  RCC_ClkInitStruct->ClockType = 0x00000001U | 0x00000002U | 0x00000004U | 0x00000008U;

   
  RCC_ClkInitStruct->SYSCLKSource = (uint32_t)(((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CFGR & (0x3U << (0U)));

   
  RCC_ClkInitStruct->AHBCLKDivider = (uint32_t)(((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CFGR & (0xFU << (4U)));

   
  RCC_ClkInitStruct->APB1CLKDivider = (uint32_t)(((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CFGR & (0x7U << (10U)));

   
  RCC_ClkInitStruct->APB2CLKDivider = (uint32_t)((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CFGR & (0x7U << (13U))) >> 3U);

   
  *pFLatency = (uint32_t)(((FLASH_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3C00U))->ACR & (0xFU << (0U)));
}





 
void HAL_RCC_NMI_IRQHandler(void)
{
   
  if(((((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->CIR & (((uint8_t)0x80))) == (((uint8_t)0x80))))
  {
     
    HAL_RCC_CSSCallback();

     
    (*(volatile uint8_t *) ((uint32_t)(((0x40000000U + 0x00020000U) + 0x3800U) + 0x0CU + 0x02U)) = (((uint8_t)0x80)));
  }
}




 
__weak void HAL_RCC_CSSCallback(void)
{
  

 
}



 



 




 



 

 
