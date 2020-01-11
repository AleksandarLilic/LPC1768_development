; generated by Component: ARM Compiler 5.06 update 5 (build 528) Tool: ArmCC [4d3621]
; commandline ArmCC [--c99 --split_sections --debug -c --asm -o.\objects\lpc1768_adc.o --depend=.\objects\lpc1768_adc.d --cpu=Cortex-M3 --apcs=interwork -O0 --diag_suppress=9931 -I.\Drivers -I.\RTE\Device\LPC1768 -I.\RTE\_Target_1 -IC:\Keil_v5\ARM\PACK\ARM\CMSIS\5.4.0\CMSIS\Core\Include -IC:\Keil_v5\ARM\PACK\Keil\LPC1700_DFP\2.5.0\Device\Include -D__UVISION_VERSION=524 -D_RTE_ -DLPC175x_6x --omf_browse=.\objects\lpc1768_adc.crf Drivers\lpc1768_adc.c]
        THUMB
        REQUIRE8
        PRESERVE8

        AREA ||i.ADC_D_I||, CODE, READONLY, ALIGN=2

ADC_D_I PROC
        LDR      r1,|L1.16|
        LDR      r1,[r1,#0xc]
        BIC      r1,r1,#1
        LDR      r2,|L1.16|
        STR      r1,[r2,#0xc]
        BX       lr
        ENDP

        DCW      0x0000
|L1.16|
        DCD      0x40034000

        AREA ||i.ADC_E_I||, CODE, READONLY, ALIGN=2

ADC_E_I PROC
        MOVS     r1,#1
        LDR      r2,|L2.8|
        STR      r1,[r2,#0xc]
        BX       lr
        ENDP

|L2.8|
        DCD      0x40034000

        AREA ||i.ADC_GetClock||, CODE, READONLY, ALIGN=2

ADC_GetClock PROC
        MOV      r1,r0
        LDR      r3,|L3.64|
        LDR      r3,[r3,#0]
        UBFX     r2,r3,#24,#2
        CBZ      r2,|L3.26|
        CMP      r2,#1
        BEQ      |L3.34|
        CMP      r2,#2
        BEQ      |L3.40|
        CMP      r2,#3
        BNE      |L3.56|
        B        |L3.48|
|L3.26|
        LDR      r3,|L3.68|
        LDR      r3,[r3,#0]  ; SystemCoreClock
        LSRS     r0,r3,#2
        B        |L3.56|
|L3.34|
        LDR      r3,|L3.68|
        LDR      r0,[r3,#0]  ; SystemCoreClock
        B        |L3.56|
|L3.40|
        LDR      r3,|L3.68|
        LDR      r3,[r3,#0]  ; SystemCoreClock
        LSRS     r0,r3,#1
        B        |L3.56|
|L3.48|
        LDR      r3,|L3.68|
        LDR      r3,[r3,#0]  ; SystemCoreClock
        LSRS     r0,r3,#3
        NOP      
|L3.56|
        NOP      
        STR      r0,[r1,#0xc]
        BX       lr
        ENDP

        DCW      0x0000
|L3.64|
        DCD      0x400fc1a8
|L3.68|
        DCD      SystemCoreClock

        AREA ||i.ADC_Init||, CODE, READONLY, ALIGN=1

ADC_Init PROC
        PUSH     {r4,lr}
        MOV      r4,r0
        MOV      r0,r4
        BL       ADC_Power_Enable
        MOV      r0,r4
        BL       ADC_GetClock
        MOV      r0,r4
        BL       ADC_SetSampleRate
        MOV      r0,r4
        BL       ADC_PinInit
        POP      {r4,pc}
        ENDP


        AREA ||i.ADC_PinInit||, CODE, READONLY, ALIGN=2

ADC_PinInit PROC
        PUSH     {r4,lr}
        MOV      r4,r0
        LDR      r0,|L5.40|
        LDR      r0,[r0,#0]
        ORR      r0,r0,#1
        LDR      r1,|L5.40|
        STR      r0,[r1,#0]
        MOVS     r2,#1
        MOVS     r1,#0x17
        MOVS     r0,#0
        BL       GPIO_PinFunction
        MOVS     r2,#0
        MOVS     r1,#0x17
        MOV      r0,r2
        BL       GPIO_PinDirection
        POP      {r4,pc}
        ENDP

        DCW      0x0000
|L5.40|
        DCD      0x40034000

        AREA ||i.ADC_Power_Disable||, CODE, READONLY, ALIGN=2

ADC_Power_Disable PROC
        LDR      r1,|L6.32|
        LDR      r1,[r1,#0]
        BIC      r1,r1,#0x200000
        LDR      r2,|L6.32|
        STR      r1,[r2,#0]
        LDR      r1,|L6.36|
        LDR      r1,[r1,#0]
        BIC      r1,r1,#0x1000
        LDR      r2,|L6.36|
        SUBS     r2,r2,#0xc4
        STR      r1,[r2,#0xc4]
        BX       lr
        ENDP

        DCW      0x0000
|L6.32|
        DCD      0x40034000
|L6.36|
        DCD      0x400fc0c4

        AREA ||i.ADC_Power_Enable||, CODE, READONLY, ALIGN=2

ADC_Power_Enable PROC
        LDR      r1,|L7.32|
        LDR      r1,[r1,#0]
        ORR      r1,r1,#0x1000
        LDR      r2,|L7.32|
        SUBS     r2,r2,#0xc4
        STR      r1,[r2,#0xc4]
        LDR      r1,|L7.36|
        LDR      r1,[r1,#0]
        ORR      r1,r1,#0x200000
        LDR      r2,|L7.36|
        STR      r1,[r2,#0]
        BX       lr
        ENDP

        DCW      0x0000
|L7.32|
        DCD      0x400fc0c4
|L7.36|
        DCD      0x40034000

        AREA ||i.ADC_ReadData||, CODE, READONLY, ALIGN=2

ADC_ReadData PROC
        LDR      r2,[r0,#0x60]
        CMP      r2,#0x17
        BLS      |L8.10|
        MOVS     r2,#0
        STR      r2,[r0,#0x60]
|L8.10|
        LDR      r2,|L8.40|
        LDR      r1,[r2,#0x10]
        AND      r2,r1,#0x80000000
        CBZ      r2,|L8.30|
        UBFX     r2,r1,#4,#12
        LDR      r3,[r0,#0x60]
        STR      r2,[r0,r3,LSL #2]
|L8.30|
        LDR      r2,[r0,#0x60]
        ADDS     r2,r2,#1
        STR      r2,[r0,#0x60]
        BX       lr
        ENDP

        DCW      0x0000
|L8.40|
        DCD      0x40034000

        AREA ||i.ADC_SetSampleRate||, CODE, READONLY, ALIGN=2

ADC_SetSampleRate PROC
        LDR      r1,|L9.16|
        LDR      r1,[r1,#0]
        ORR      r1,r1,#0x1000
        LDR      r2,|L9.16|
        STR      r1,[r2,#0]
        BX       lr
        ENDP

        DCW      0x0000
|L9.16|
        DCD      0x40034000

        AREA ||i.ADC_StartConversion||, CODE, READONLY, ALIGN=2

ADC_StartConversion PROC
        LDR      r1,|L10.16|
        LDR      r1,[r1,#0]
        ORR      r1,r1,#0x1000000
        LDR      r2,|L10.16|
        STR      r1,[r2,#0]
        BX       lr
        ENDP

        DCW      0x0000
|L10.16|
        DCD      0x40034000

        AREA ||.arm_vfe_header||, DATA, READONLY, NOALLOC, ALIGN=2

        DCD      0x00000000

        AREA ||.data||, DATA, ALIGN=0

ADC0_INT0
        DCB      0x00

;*** Start embedded assembler ***

#line 1 "Drivers\\lpc1768_adc.c"
	AREA ||.rev16_text||, CODE
	THUMB
	EXPORT |__asm___13_lpc1768_adc_c_ADC_E_I____REV16|
#line 463 "C:\\Keil_v5\\ARM\\PACK\\ARM\\CMSIS\\5.4.0\\CMSIS\\Core\\Include\\cmsis_armcc.h"
|__asm___13_lpc1768_adc_c_ADC_E_I____REV16| PROC
#line 464

 rev16 r0, r0
 bx lr
	ENDP
	AREA ||.revsh_text||, CODE
	THUMB
	EXPORT |__asm___13_lpc1768_adc_c_ADC_E_I____REVSH|
#line 478
|__asm___13_lpc1768_adc_c_ADC_E_I____REVSH| PROC
#line 479

 revsh r0, r0
 bx lr
	ENDP
	AREA ||.rrx_text||, CODE
	THUMB
	EXPORT |__asm___13_lpc1768_adc_c_ADC_E_I____RRX|
#line 665
|__asm___13_lpc1768_adc_c_ADC_E_I____RRX| PROC
#line 666

 rrx r0, r0
 bx lr
	ENDP

;*** End   embedded assembler ***

        EXPORT ADC_D_I [CODE]
        EXPORT ADC_E_I [CODE]
        EXPORT ADC_Init [CODE]
        EXPORT ADC_ReadData [CODE]
        EXPORT ADC_StartConversion [CODE]
        EXPORT ADC0_INT0 [DATA,SIZE=1]

        IMPORT ||Lib$$Request$$armlib|| [CODE,WEAK]
        IMPORT SystemCoreClock [DATA]
        IMPORT GPIO_PinFunction [CODE]
        IMPORT GPIO_PinDirection [CODE]

        KEEP ADC_GetClock
        KEEP ADC_PinInit
        KEEP ADC_Power_Disable
        KEEP ADC_Power_Enable
        KEEP ADC_SetSampleRate

        ATTR FILESCOPE
        ATTR SETVALUE Tag_ABI_PCS_wchar_t,2
        ATTR SETVALUE Tag_ABI_enum_size,1
        ATTR SETVALUE Tag_ABI_optimization_goals,6
        ATTR SETSTRING Tag_conformance,"2.09"
        ATTR SETVALUE AV,18,1

        ASSERT {ENDIAN} = "little"
        ASSERT {INTER} = {TRUE}
        ASSERT {ROPI} = {FALSE}
        ASSERT {RWPI} = {FALSE}
        ASSERT {IEEE_FULL} = {FALSE}
        ASSERT {IEEE_PART} = {FALSE}
        ASSERT {IEEE_JAVA} = {FALSE}
        END