;/***********************************************************************************************************************
;* File Name    : startup_BAT32G137.s
;* Device(s)    : BAT32G137GH64FB
;* Tool-Chain   : ARMCC
;* Description  : This is start up file for ARMCC.
;* Creation Date: 2022/1/28
;***********************************************************************************************************************/

; <h> Stack Configuration
;   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Stack_Size      EQU     0x00000400

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp


; <h> Heap Configuration
;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Heap_Size       EQU     0x00000100

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit


                PRESERVE8
                THUMB


; Vector Table Mapped to Address 0 at Reset

                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors
                EXPORT  __Vectors_End
                EXPORT  __Vectors_Size

__Vectors       DCD     __initial_sp              ; Top of Stack
                DCD     Reset_Handler             ; Reset Handler
                DCD     NMI_Handler               ; NMI Handler
                DCD     HardFault_Handler         ; Hard Fault Handler
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     SVC_Handler               ; SVCall Handler
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     PendSV_Handler            ; PendSV Handler
                DCD     SysTick_Handler           ; SysTick Handler

; External Interrupts
; ToDo:  Add here the vectors for the device specific external interrupts handler
                DCD     IRQ00_Handler       	  ;  IRQ0 
                DCD     IRQ01_Handler       	  ;  IRQ1 
                DCD     IRQ02_Handler       	  ;  IRQ2 
                DCD     IRQ03_Handler       	  ;  IRQ3 
                DCD     IRQ04_Handler       	  ;  IRQ4 
                DCD     IRQ05_Handler       	  ;  IRQ5 
                DCD     IRQ06_Handler       	  ;  IRQ6 
                DCD     IRQ07_Handler       	  ;  IRQ7 
                DCD     IRQ08_Handler       	  ;  IRQ8 
                DCD     IRQ09_Handler       	  ;  IRQ9 
                DCD     IRQ10_Handler       	  ;  IRQ10
                DCD     IRQ11_Handler       	  ;  IRQ11
                DCD     IRQ12_Handler       	  ;  IRQ12
                DCD     IRQ13_Handler       	  ;  IRQ13
                DCD     IRQ14_Handler       	  ;  IRQ14
                DCD     IRQ15_Handler       	  ;  IRQ15
                DCD     IRQ16_Handler       	  ;  IRQ16
                DCD     IRQ17_Handler       	  ;  IRQ17
                DCD     IRQ18_Handler       	  ;  IRQ18
                DCD     IRQ19_Handler       	  ;  IRQ19
                DCD     IRQ20_Handler       	  ;  IRQ20
                DCD     IRQ21_Handler       	  ;  IRQ21
                DCD     IRQ22_Handler       	  ;  IRQ22
                DCD     IRQ23_Handler       	  ;  IRQ23
                DCD     IRQ24_Handler       	  ;  IRQ24
                DCD     IRQ25_Handler       	  ;  IRQ25
                DCD     IRQ26_Handler       	  ;  IRQ26
                DCD     IRQ27_Handler       	  ;  IRQ27
                DCD     IRQ28_Handler       	  ;  IRQ28
                DCD     IRQ29_Handler       	  ;  IRQ29
                DCD     IRQ30_Handler       	  ;  IRQ30
                DCD     IRQ31_Handler       	  ;  IRQ31
__Vectors_End

__Vectors_Size  EQU     __Vectors_End - __Vectors

                AREA    |.text|, CODE, READONLY


; Reset Handler

Reset_Handler   PROC
                EXPORT  Reset_Handler             [WEAK]
                IMPORT  SystemInit
                IMPORT  __main
                LDR     R0, =SystemInit
                BLX     R0
                LDR     R0, =__main
                BX      R0
                ENDP


; Dummy Exception Handlers (infinite loops which can be modified)

NMI_Handler\
                PROC
                EXPORT  NMI_Handler               [WEAK]
                B       .
                ENDP
HardFault_Handler\
                PROC
                EXPORT  HardFault_Handler         [WEAK]
                B       .
                ENDP
MemManage_Handler\
                PROC
                EXPORT  MemManage_Handler         [WEAK]
                B       .
                ENDP
BusFault_Handler\
                PROC
                EXPORT  BusFault_Handler          [WEAK]
                B       .
                ENDP
UsageFault_Handler\
                PROC
                EXPORT  UsageFault_Handler        [WEAK]
                B       .
                ENDP
SVC_Handler\
                PROC
                EXPORT  SVC_Handler               [WEAK]
                B       .
                ENDP
DebugMon_Handler\
                PROC
                EXPORT  DebugMon_Handler          [WEAK]
                B       .
                ENDP
PendSV_Handler\
                PROC
                EXPORT  PendSV_Handler            [WEAK]
                B       .
                ENDP
SysTick_Handler\
                PROC
                EXPORT  SysTick_Handler           [WEAK]
                B       .
                ENDP
IRQ00_Handler\
                PROC
                EXPORT  IRQ00_Handler           [WEAK]
                B       .
                ENDP
IRQ01_Handler\
                PROC
                EXPORT  IRQ01_Handler           [WEAK]
                B       .
                ENDP
IRQ02_Handler\
                PROC
                EXPORT  IRQ02_Handler           [WEAK]
                B       .
                ENDP
IRQ03_Handler\
                PROC
                EXPORT  IRQ03_Handler           [WEAK]
                B       .
                ENDP
IRQ04_Handler\
                PROC
                EXPORT  IRQ04_Handler           [WEAK]
                B       .
                ENDP
IRQ05_Handler\
                PROC
                EXPORT  IRQ05_Handler           [WEAK]
                B       .
                ENDP
IRQ06_Handler\
                PROC
                EXPORT  IRQ06_Handler           [WEAK]
                B       .
                ENDP
IRQ07_Handler\
                PROC
                EXPORT  IRQ07_Handler           [WEAK]
                B       .
                ENDP
IRQ08_Handler\
                PROC
                EXPORT  IRQ08_Handler           [WEAK]
                B       .
                ENDP
IRQ09_Handler\
                PROC
                EXPORT  IRQ09_Handler           [WEAK]
                B       .
                ENDP
IRQ10_Handler\
                PROC
                EXPORT  IRQ10_Handler           [WEAK]
                B       .
                ENDP
IRQ11_Handler\
                PROC
                EXPORT  IRQ11_Handler           [WEAK]
                B       .
                ENDP
IRQ12_Handler\
                PROC
                EXPORT  IRQ12_Handler           [WEAK]
                B       .
                ENDP
IRQ13_Handler\
                PROC
                EXPORT  IRQ13_Handler           [WEAK]
                B       .
                ENDP
IRQ14_Handler\
                PROC
                EXPORT  IRQ14_Handler           [WEAK]
                B       .
                ENDP
IRQ15_Handler\
                PROC
                EXPORT  IRQ15_Handler           [WEAK]
                B       .
                ENDP
IRQ16_Handler\
                PROC
                EXPORT  IRQ16_Handler           [WEAK]
                B       .
                ENDP
IRQ17_Handler\
                PROC
                EXPORT  IRQ17_Handler           [WEAK]
                B       .
                ENDP
IRQ18_Handler\
                PROC
                EXPORT  IRQ18_Handler           [WEAK]
                B       .
                ENDP
IRQ19_Handler\
                PROC
                EXPORT  IRQ19_Handler           [WEAK]
                B       .
                ENDP
IRQ20_Handler\
                PROC
                EXPORT  IRQ20_Handler           [WEAK]
                B       .
                ENDP
IRQ21_Handler\
                PROC
                EXPORT  IRQ21_Handler           [WEAK]
                B       .
                ENDP
IRQ22_Handler\
                PROC
                EXPORT  IRQ22_Handler           [WEAK]
                B       .
                ENDP
IRQ23_Handler\
                PROC
                EXPORT  IRQ23_Handler           [WEAK]
                B       .
                ENDP
IRQ24_Handler\
                PROC
                EXPORT  IRQ24_Handler           [WEAK]
                B       .
                ENDP
IRQ25_Handler\
                PROC
                EXPORT  IRQ25_Handler           [WEAK]
                B       .
                ENDP
IRQ26_Handler\
                PROC
                EXPORT  IRQ26_Handler           [WEAK]
                B       .
                ENDP
IRQ27_Handler\
                PROC
                EXPORT  IRQ27_Handler           [WEAK]
                B       .
                ENDP
IRQ28_Handler\
                PROC
                EXPORT  IRQ28_Handler           [WEAK]
                B       .
                ENDP
IRQ29_Handler\
                PROC
                EXPORT  IRQ29_Handler           [WEAK]
                B       .
                ENDP
IRQ30_Handler\
                PROC
                EXPORT  IRQ30_Handler           [WEAK]
                B       .
                ENDP
IRQ31_Handler\
                PROC
                EXPORT  IRQ31_Handler           [WEAK]
                B       .
                ENDP


                ALIGN


; User Initial Stack & Heap

                IF      :DEF:__MICROLIB

                EXPORT  __initial_sp
                EXPORT  __heap_base
                EXPORT  __heap_limit

                ELSE

                IMPORT  __use_two_region_memory
                EXPORT  __user_initial_stackheap

__user_initial_stackheap PROC
                LDR     R0, =  Heap_Mem
                LDR     R1, =(Stack_Mem + Stack_Size)
                LDR     R2, = (Heap_Mem +  Heap_Size)
                LDR     R3, = Stack_Mem
                BX      LR
                ENDP

                ALIGN

                ENDIF


                END

