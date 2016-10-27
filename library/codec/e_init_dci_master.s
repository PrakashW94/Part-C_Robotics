
;
; File Notes:
; 1. We will initialize the DCI module so as to allow it to initialize the
;    Codec in the codec initialization routine.
; 2. Some aspects of DCI functionality will be re-initialized after the Codec
;    setup is complete.
; 3. When this routine is executed, the DCI module will be set up for
;    transmitting and receiving only on TimeSlot1.
; 4. Each time slot will be 16-bits long.
;    A total of 8 time slots will be transmitted before a Frame Sync pulse is
;    generated.
; 5. Two of the four available buffer registers (TXBUF0/1) are set up for usage.



.include "p30f6014A.inc"
.include "e_common.inc"


.global  _e_init_dci_master

.section .text
_e_init_dci_master:

        push    w0                      ;Save w0


        clr     DCICON1                 ;Ensure DCI module is placed in known state
        clr     DCICON2
        clr     DCICON3
        clr     TSCON
        clr     RSCON

        bset    TRISB, #RB7             ;Make codec reset switch port pin an input for now.
                                        ;Ensure port pins are set correctly
                                        ;(Not really required since DCI takes
                                        ; control of it's pins)
        bclr    TRISG, #RG13            ;CSDO pin configured for output
        bset    TRISG, #RG12            ;CSDI pin configured for input
        bclr    TRISG, #RG15            ;COFS output to Si3000 in DCI Master mode
        bclr    TRISG, #RG14            ;CSCK output to Si3000

        bclr    DCICON1, #COFSM0        ;Frame sync mode (bits 1:0)
        bclr    DCICON1, #COFSM1        ;Set for multichannel frame sync mode
        bclr    DCICON1, #DJST          ;Data justification control
                                        ;Data txmit/rx begins 1 clock after Fs pulse
        bclr    DCICON1, #CSCKE         ;Sample clock edge control bit
                                        ;Data changes on rising, sampled on falling
        bset    DCICON1, #CSDOM         ;Tristate output CSDO pin when not txing
        bclr    DCICON1, #COFSD         ;Frame sync generated by dsPIC DCI
        bclr    DCICON1, #CSCKD         ;Clock is output from DCI


        mov     #0b0000000011101111, W0 ;Data word size is 16-bits (bits 3:0)
        mov     W0, DCICON2             ;Data frame is 8 words (bits 8:5)

        bclr    DCICON2, #BLEN1         ;Set buffer length control (2 data words
        bset    DCICON2, #BLEN0         ;Will be buffered between interrupts)


        mov     #BCG, W0                ;See common.inc file
        mov     W0, DCICON3             ;Initialize DCI bit clock generator


        bset    RSCON, #0               ;Set receive slot 1 enable
        bset    TSCON, #0               ;Set transmit slot 1 enable


        bclr    IFS2, #DCIIF            ;Ensure DCI interrupt flag is reset
        bclr    IEC2, #DCIIE            ;ISR processing is disabled for now
        bclr    DCICON1, #DCIEN         ;Ensure module is disabled for now

        pop     w0                      ;Restore w0

        return                          ;Return to calling routine


.end                                    ;EOF

