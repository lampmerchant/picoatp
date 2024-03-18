;;; 80 characters wide please ;;;;;;;;;;;;;;;;;;;;;;;;;; 8-space tabs please ;;;


;
;;;
;;;;;  PicoATP: Tiny AppleTalk Stack with ATP
;;;
;


;;; Connections ;;;

;;;                                                                         ;;;
;                                     .--------.                              ;
;                             Supply -|01 \/ 08|- Ground                      ;
;     UART (TashTalk) Rx ---> RX/RA5 -|02    07|- RA0    <--> GPIO 0          ;
;     UART (TashTalk) Tx <--- TX/RA4 -|03    06|- RA1    <--> GPIO 1          ;
;    UART (TashTalk) RTS --->    RA3 -|04    05|- RA2    ---> Soft UART Tx    ;
;                                     '--------'                              ;
;;;                                                                         ;;;


;;; Assembler Directives ;;;

	list		P=PIC12F1840, F=INHX32, ST=OFF, MM=OFF, R=DEC, X=ON
	#include	P12F1840.inc
	errorlevel	-302	;Suppress "register not in bank 0" messages
	errorlevel	-224	;Suppress TRIS instruction not recommended msgs
	__config	_CONFIG1, _FOSC_INTOSC & _WDTE_OFF & _PWRTE_ON & _MCLRE_OFF & _CP_OFF & _CPD_OFF & _BOREN_OFF & _CLKOUTEN_OFF & _IESO_OFF & _FCMEN_OFF
			;_FOSC_INTOSC	Internal oscillator, I/O on RA5
			;_WDTE_OFF	Watchdog timer disabled
			;_PWRTE_ON	Keep in reset for 64 ms on start
			;_MCLRE_OFF	RA3/!MCLR is RA3
			;_CP_OFF	Code protection off
			;_CPD_OFF	Data memory protection off
			;_BOREN_OFF	Brownout reset off
			;_CLKOUTEN_OFF	CLKOUT disabled, I/O on RA4
			;_IESO_OFF	Internal/External switch not needed
			;_FCMEN_OFF	Fail-safe clock monitor not needed
	__config	_CONFIG2, _WRT_OFF & _PLLEN_ON & _STVREN_ON & _LVP_OFF
			;_WRT_OFF	Write protection off
			;_PLLEN_ON	4x PLL on
			;_STVREN_ON	Stack over/underflow causes reset
			;_LVP_OFF	High-voltage on Vpp to program


;;; Macros ;;;

DELAY	macro	value		;Delay 3*W cycles, set W to 0
	movlw	value
	decfsz	WREG,F
	bra	$-1
	endm

DNOP	macro
	bra	$+1
	endm


;;; Pin Assignments ;;;

;RTS pin
RS_PORT	equ	PORTA
RS_IOCF	equ	IOCAF
RS_IOCN	equ	IOCAN
RS_PIN	equ	RA3


;;; Constants ;;;

;PicoATP configuration parameters
PA_VARH	equ	0x20	;Linear memory location where PicoATP's variables start
PA_VARL	equ	0x80	; "
PA_MBOX	equ	6	;Number of ATP mailboxes allocated for PicoATP
#define	PA_OBJ	"PIC12F1840"	;Object name for NTP
#define	PA_TYPE	"PicGpio"	;Object type for NTP

;PicoATP state variables in its linear memory
;INDFx		0x00	;Temporary variable (also used to hold received byte)
TMR2SEC	equ	0x01	;Countdown for how many Timer1 overflows equal 2 seconds
RTTMOUT	equ	0x02	;RTMP timeout counter
OURNODE	equ	0x03	;This node's node number
NETWRKH	equ	0x04	;This node's network number
NETWRKL	equ	0x05	; "
AROUTER	equ	0x06	;Node number of a router on this network
RCVSTAT	equ	0x07	;Receiver state (LSB clear indicates escape sequence)
RCVNODE	equ	0x08	;Node number of datagram source
RCVNETH	equ	0x09	;Network number of datagram source
RCVNETL	equ	0x0A	; "
RCVSOCK	equ	0x0B	;Datagram source socket
RCVTMP0	equ	0x0C	;Receiver temporary storage
RCVTMP1	equ	0x0D	; "
XMISTAT	equ	0x0E	;Transmitter state
XMITMP0	equ	0x0F	;Transmitter temporary storage

;PicoATP variables for ATP mailboxes
;INDFx		0x00	;Flags
MBSNETH	equ	0x01	;Sender's network number
MBSNETL	equ	0x02	; "
MBSNODE	equ	0x03	;Sender's node number
MBSSOCK	equ	0x04	;Sender's socket number
MBSTIDH	equ	0x05	;Request TID
MBSTIDL	equ	0x06	; "
MBTMOUT	equ	0x07	;Timeout (in two-second periods)
;		0x08-F	;Payload

;Mailbox Flags
MBINUSE	equ	7	;Set if mailbox is in use
MBRESP	equ	6	;Set if mailbox contains a TResp answered by mainline
MBXO	equ	5	;Set if TReq/TResp is XO and must be retained
MBXMITD	equ	4	;Clear if TResp should be sent
MBNBP	equ	3	;Set if this is an NBP response, not an ATP TReq/TResp
MBLEN2	equ	2	;Length of payload, minus one
MBLEN1	equ	1	; "
MBLEN0	equ	0	; "


;;; Variable Storage ;;;

	cblock	0x70	;Bank-common registers
	
	FLAGS		;You've got to have flags
	UQPUSH
	UQPOP
	X12
	X11
	X10
	X9
	X8
	X7
	X6
	X5
	X4
	X3
	X2
	X1
	X0
	
	endc


;;; Vectors ;;;

	org	0x0		;Reset vector
	goto	Init

	org	0x4		;Interrupt vector
	;fall through


;;; Interrupt Handler ;;;

Interrupt
	movlw	PA_VARH		;Load FSR0 and FSR1 with the location of
	movwf	FSR0H		; PicoATP's state
	movwf	FSR1H		; "
	movlw	PA_VARL		; "
	movwf	FSR0L		; "
	movwf	FSR1L		; "
	btfsc	INTCON,TMR0IF	;If Timer0 overflowed, handle it
	bra	IntTimer	; "
	movlb	0		;If a byte came in from TashTalk, handle it
	btfsc	PIR1,RCIF	; "
	bra	IntReceive	; "
	movlb	7		;If TashTalk flagged that it's ready to receive
	btfss	RS_IOCF,RS_PIN	; data again, enable the UART transmitter
	bra	Interr0		; interrupt and continue
	bcf	RS_IOCF,RS_PIN	; "
	movlb	1		; "
	bsf	PIE1,TXIE	; "
Interr0	movlb	0		;If the transmitter is ready to transmit a byte
	movlp	high IntMainline; and the transmitter interrupt is enabled,
	btfss	PIR1,TXIF	; handle it, else jump ahead to the mainline
	goto	IntMainline	; interrupt handler because this is some other
	movlb	1		; interrupt
	btfss	PIE1,TXIE	; "
	goto	IntMainline	; "
	;fall through

IntTransmit
	movlb	0		;If CTS is deasserted (high), disable the Tx
	movf	RS_PORT,W	; interrupt and return so we hold off sending
	btfss	WREG,RS_PIN	; data to TashTalk
	bra	IntTra0		; "
	movlb	1		; "
	bcf	PIE1,TXIE	; "
	retfie			; "
IntTra0	moviw	XMISTAT[FSR1]	;Get the pointer into the state machine
	movlb	3		;Assume that a write to the UART is coming
	movlp	high TransmitFsa;Jump into the transmitter state machine
	callw			; "
	movwi	XMISTAT[FSR1]	;Write the next state back into XMISTAT
	retfie			;Done

IntTimer
	bcf	INTCON,TMR0IF	;Clear the interrupt
	moviw	TMR2SEC[FSR1]	;Decrement the countdown of timer overflows that
	addlw	-1		; equal a two-second period
	movwi	TMR2SEC[FSR1]	; "
	btfss	STATUS,Z	;If it hasn't yet reached zero, we're done
	retfie			; "
	movlw	244		;Reset the countdown to 244 (this is the number
	movwi	TMR2SEC[FSR1]	; of timer overflows that equal two seconds)
	moviw	RTTMOUT[FSR1]	;Decrement the RTMP timeout
	addlw	-1		; "
	movwi	RTTMOUT[FSR1]	; "
	btfss	STATUS,Z	;If it reached zero, the router we knew has
	bra	IntTmr0		; apparently gone away, so blank out our network
	movlw	0		; number and the node address of the router
	movwi	NETWRKH[FSR1]	; "
	movwi	NETWRKL[FSR1]	; "
	movwi	AROUTER[FSR1]	; "
IntTmr0	movlw	PA_MBOX		;Use FSR1L as a temp variable to count the
	movwf	FSR1L		; mailboxes as we iterate through them
IntTmr1	addfsr	FSR0,0x10	;Move FSR0 forward to the next mailbox
	comf	INDF0,W		;Check if this mailbox is an XO TResp that has
	andlw	(1 << MBRESP) | (1 << MBXMITD) | (1 << MBXO); been transmitted
	btfss	STATUS,Z	;If not, don't bother with its timeout and skip
	bra	IntTmr2		; to the next
	moviw	MBTMOUT[FSR0]	;Decrement the timeout for this TResp
	addlw	-1		; "
	movwi	MBTMOUT[FSR0]	; "
	btfsc	STATUS,Z	;If it has hit zero, clear its flags so it
	clrf	INDF0		; becomes available again
IntTmr2	decfsz	FSR1L,F		;If there are more mailboxes left to maybe
	bra	IntTmr1		; decrement the timeout on, loop to handle the
	retfie			; next, else return

IntReceive
	movlb	3		;Grab the received byte
	movf	RCREG,W		; "
	btfss	STATUS,Z	;If the received byte was 0x00, the beginning of
	bra	IntRec0		; an escape sequence from TashTalk, clear the
	moviw	RCVSTAT[FSR1]	; LSB of the state pointer and return without
	andlw	B'11111110'	; entering the state machine
	movwi	RCVSTAT[FSR1]	; "
	retfie			; "
IntRec0	movwf	INDF0		;Move received byte to temp var for later use
	moviw	RCVSTAT[FSR1]	;If the received byte is not the second byte of
	btfsc	WREG,0		; an escape sequence, skip ahead to jump into
	bra	IntRec2		; the state machine
	incf	INDF0,W		;If the received byte is the second byte of an
	btfss	STATUS,Z	; 0x00 0xFF escape sequence, a literal zero,
	bra	IntRec1		; overstrike temp var with a zero and skip the
	movwf	INDF0		; first instruction when we jump into the
	moviw	RCVSTAT[FSR1]	; receiver state machine
	iorlw	B'00000001'	; "
	movlp	high ReceiveFsa	; "
	callw			; "
	movwi	RCVSTAT[FSR1]	;Write the next state back into RCVSTAT for next
	retfie			; time with the LSB set, done
IntRec1	moviw	RCVSTAT[FSR1]	;Re-get the pointer into the state machine
IntRec2	movlp	high ReceiveFsa	;Jump into the receiver state machine, skipping
	callw			; the first instruction if not an escape seq
	movwi	RCVSTAT[FSR1]	;Write the next state back into RCVSTAT for next
	retfie			; time with the LSB set, done


;;; Hardware Initialization ;;;

Init
	banksel	OSCCON		;32 MHz (w/PLL) high-freq internal oscillator
	movlw	B'11110000'
	movwf	OSCCON

	banksel	OSCSTAT		;Spin until PLL is ready and instruction clock
	btfss	OSCSTAT,PLLR	; gears up to 8 MHz
	bra	$-1

	banksel	RS_IOCN		;RTS pin interrupts on negative edge
	movlw	1 << RS_PIN
	movwf	RS_IOCN

	banksel	RCSTA		;UART async mode, 1 MHz, but receiver not
	movlw	B'01001000'	; enabled just yet
	movwf	BAUDCON
	clrf	SPBRGH
	movlw	7
	movwf	SPBRGL
	movlw	B'00100110'
	movwf	TXSTA
	movlw	B'10000000'
	movwf	RCSTA

	banksel	OPTION_REG	;Timer0 ticks 1:256 with instruction clock
	movlw	B'11010111'
	movwf	OPTION_REG

	banksel	APFCON		;Rx and Tx on RA5 and RA4
	movlw	B'10000100'
	movwf	APFCON

	banksel	ANSELA		;All pins digital, not analog
	clrf	ANSELA

	banksel	TRISA		;Tx output, Rx, RTS, RA2:0 inputs
	movlw	B'00101111'
	movwf	TRISA

	;fall through


;;; Software Initialization ;;;

	movlb	0		;Wait for 512 ms, this should give TashTalk
	clrf	TMR0		; plenty of time to come up
	movlw	64
	movwf	FSR0L
InitWLp	bcf	INTCON,TMR0IF
	btfss	INTCON,TMR0IF
	bra	$-1
	decfsz	FSR0L,F
	bra	InitWLp

	movlw	PA_VARH		;Initialize key globals
	movwf	FSR0H
	movwf	FSR1H
	movlw	PA_VARL
	movwf	FSR0L
	movwf	FSR1L
	movlw	0
	movwi	OURNODE[FSR1]
	movwi	RCVTMP1[FSR1]
	movlw	254
	movwi	RCVTMP0[FSR1]
	movlw	low LapDest | 1
	movwi	RCVSTAT[FSR1]
	movlw	low XDecide
	movwi	XMISTAT[FSR1]

	movlw	PA_MBOX		;Ensure all mailboxes are marked as empty
InitZLp	addfsr	FSR0,0x10
	clrf	INDF0
	decfsz	WREG,W
	bra	InitZLp

	banksel	PIE1		;UART Rx, Timer0, and IOC interrupts on,
	movlw	B'00100000'	; interrupt subsystem on
	movwf	PIE1
	movlw	B'11101000'
	movwf	INTCON

	banksel	RCSTA		;Enable receiver now that interrupt is on
	bsf	RCSTA,CREN

TashTI0	movlb	3		;Send a sequence of 1024 0x00 bytes to TashTalk
	movlw	4		; to ensure it's in a state to accept commands
	movwf	FSR0H		; "
	clrf	FSR0L		; "
TashTI1	call	TashTm0		; "
	decfsz	FSR0L,F		; "
	bra	TashTI1		; "
	decfsz	FSR0H,F		; "
	bra	TashTI1		; "
	movlw	0x02		;Send an 0x02 (Set Node IDs) command followed by
	call	TashTm1		; 32 0x00 bytes to ensure that the node IDs
	movlw	32		; bitmap is clear and TashTalk doesn't respond
	movwf	FSR0L		; automatically to any frames it receives
TashTI2	call	TashTm0		; "
	decfsz	FSR0L,F		; "
	bra	TashTI2		; "
	movlw	0x03		;Send an 0x03 (Set Features) command followed by
	call	TashTm1		; 0b11000000 to enable CRC calculation and
	movlw	B'11000000'	; checking
	call	TashTm1		; "
	movlb	0		;Reset Timer0 again so we can use it as a delay
	clrf	TMR0		; timer
	movlw	0		;Reset the attempts counter to 0
TashTI4	movwi	RCVTMP1[FSR1]	; "
	bcf	INTCON,TMR0IF	;Clear Timer0 interrupt
	movlw	0x01		;Send an 0x01 (Transmit Frame) command with an
	call	TashTm1		; ENQ (0x81) frame for the address we're trying
	moviw	RCVTMP0[FSR1]	; "
	call	TashTm1		; "
	call	TashTm1		; "
	movlw	0x81		; "
	call	TashTm1		; "
	call	TashTm0		; "
	call	TashTm0		; "
	btfss	INTCON,TMR0IF	;Wait until ~8 ms have elapsed
	bra	$-1		; "
	moviw	RCVTMP1[FSR1]	;Increment the attempts counter; if it hasn't
	addlw	1		; rolled over, loop around to send another ENQ
	btfss	STATUS,Z	; "
	bra	TashTI4		; "
	moviw	RCVTMP0[FSR1]	;Nobody's responded on our node address, so
	movwi	OURNODE[FSR1]	; claim it for ourselves
	movlw	0x02		;Send an 0x02 (Set Node IDs) command
	call	TashTm1		; "
	movwf	FSR0L		;Reckon the bit position of this node ID in the
	moviw	OURNODE[FSR1]	; node ID bitmap
	btfss	WREG,0		; "
	lsrf	FSR0L,F		; "
	btfsc	WREG,1		; "
	lslf	FSR0L,F		; "
	btfsc	WREG,1		; "
	lslf	FSR0L,F		; "
	btfsc	WREG,2		; "
	swapf	FSR0L,F		; "
	lsrf	WREG,F		;Reckon the byte position of this node ID (one-
	lsrf	WREG,F		; relative) in the node ID bitmap
	lsrf	WREG,F		; "
	addlw	1		; "
	movwf	FSR0H		; "
	movlw	32		;Load the node ID bitmap with the bit for this
	movwf	FSR1L		; node ID set
TashTI5	movf	FSR0L,W		; "
	decfsz	FSR0H,F		; "
	movlw	0		; "
	call	TashTm1		; "
	decfsz	FSR1L,F		; "
	bra	TashTI5		; "
	movlp	high Mainline	;Jump into mainline
	goto	Mainline	; "

TashTm0	movlw	0		;Put a 0 in W
TashTm1	movlb	3		;Transmit the byte in W
	movwf	TXREG		; "
	btfss	TXSTA,TRMT	;Wait for the byte to finish transmitting
	bra	$-1		; "
	return			;Done


;;; Receiver State Machine ;;;

XNbpLen	movlw	high NbpTbO | 0x80;Reckon the length of the NBP object and type
	movwf	FSR0H		; fields
	movlw	low NbpTbO	; "
	movwf	FSR0L		; "
	clrf	INDF1		; "
XmitX	incf	INDF1,F		; "
	moviw	FSR0++		; "
	btfss	STATUS,Z	; "
	bra	XmitX		; "
XmitY	incf	INDF1,F		; "
	moviw	FSR0++		; "
	btfss	STATUS,Z	; "
	bra	XmitY		; "
	movf	INDF1,W		;Return it in W
	return			;Done

	org	0x100

ReceiveFsa
WaitIgn	retlw	low LapDest | 1	;If an escape sequence, transition to LapDest
	retlw	low WaitIgn | 1	;Else, keep waiting for frame to end
LapDest	retlw	low LapDest | 1	;If an escape sequence, transition to self
	moviw	OURNODE[FSR1]	;If we haven't yet acquired a node address,
	btfsc	STATUS,Z	; skip ahead to handle incoming frames
	bra	Lap0		; differently
	xorwf	INDF1,W		;If this frame's destination matches our node
	btfsc	STATUS,Z	; address, transition to wait for the source
	retlw	low LapSrc | 1	; "
	incf	INDF1,W		;If this frame's destination is 0xFF, this is a
	btfsc	STATUS,Z	; broadcast frame that we should process,
	retlw	low LapSrc | 1	; transition to wait for the source
	retlw	low WaitIgn | 1	;Else, wait for the frame to be over
Lap0	moviw	RCVTMP0[FSR1]	;We don't have a node address yet, so check if
	xorwf	INDF1,W		; this frame is addressed to our tentative node
	btfss	STATUS,Z	;If it is not, just wait for the frame to end
	retlw	low WaitIgn | 1	; "
	movwi	RCVTMP1[FSR1]	;If it is, reset the ENQ attempts counter to
	moviw	RCVTMP0[FSR1]	; zero and decrement the tentative address,
	addlw	-1		; wrapping around so the range of addresses
	btfsc	STATUS,Z	; tested is 1-254
	movlw	254		; "
	movwi	RCVTMP0[FSR1]	; "
	retlw	low WaitIgn | 1	;Wait for the frame to end
LapSrc	retlw	low LapDest | 1	;If an escape sequence, transition to LapDest
	movf	INDF1,W		;Store sending address
	movwi	RCVNODE[FSR1]	; "
	retlw	low LapType | 1	;Transition to receive LLAP type
LapType	retlw	low LapDest | 1	;If an escape sequence, transition to LapDest
	movlw	0		;Clear sender network vars
	movwi	RCVNETH[FSR1]	; "
	movwi	RCVNETL[FSR1]	; "
	decf	INDF1,W		;If LLAP type is 1, transition to receive length
	btfsc	STATUS,Z	; for short LLAP header
	retlw	low DdpSHLH | 1	; "
	decf	WREG,W		;If LLAP type is 2, transition to receive length
	btfsc	STATUS,Z	; for extended LLAP header
	retlw	low DdpEHLH | 1	; "
	retlw	low WaitIgn | 1	;Else, wait for the frame to be over
	nop
DdpSHLH	retlw	low LapDest | 1	;If an escape sequence, transition to LapDest
	retlw	low DdpSHLL | 1	;Else, transition to wait for low length byte
DdpSHLL	retlw	low LapDest | 1	;If an escape sequence, transition to LapDest
	retlw	low DdpDSck | 1	;Else, transition to wait for destination socket
DdpEHLH	retlw	low LapDest | 1	;If an escape sequence, transition to LapDest
	retlw	low DdpEHLL | 1	;Else, transition to wait for low length byte
DdpEHLL	retlw	low LapDest | 1	;If an escape sequence, transition to LapDest
	retlw	low DdpChsH | 1	;Else, transition to wait for checksum
DdpChsH	retlw	low LapDest | 1	;If an escape sequence, transition to LapDest
	retlw	low DdpChsL | 1	;Else, transition to wait for checksum low byte
DdpChsL	retlw	low LapDest | 1	;If an escape sequence, transition to LapDest
	retlw	low DdpDNwH | 1	;Else, transition to wait for dest network
DdpDNwH	retlw	low LapDest | 1	;If an escape sequence, transition to LapDest
	retlw	low DdpDNwL | 1	;Else, transition to wait for dest network low
DdpDNwL	retlw	low LapDest | 1	;If an escape sequence, transition to LapDest
	retlw	low DdpSNwH | 1	;Else, transition to wait for src network
DdpSNwH	retlw	low LapDest | 1	;If an escape sequence, transition to LapDest
	movf	INDF1,W		;Save source network high byte
	movwi	RCVNETH[FSR1]	; "
	retlw	low DdpSNwL | 1	;Transition to wait for source network low byte
DdpSNwL	retlw	low LapDest | 1	;If an escape sequence, transition to LapDest
	movf	INDF1,W		;Save source network low byte
	movwi	RCVNETL[FSR1]	; "
	retlw	low DdpDNod | 1	;Transition to wait for destination node
DdpDNod	retlw	low LapDest | 1	;If an escape sequence, transition to LapDest
	retlw	low DdpSNod | 1	;Else, transition to wait for source node
DdpSNod	retlw	low LapDest | 1	;If an escape sequence, transition to LapDest
	movf	INDF1,W		;Save source node
	movwi	RCVNODE[FSR1]	; "
	retlw	low DdpDSck | 1	;Transition to wait for destination socket
DdpDSck	retlw	low LapDest | 1	;If an escape sequence, transition to LapDest
	movf	INDF1,W		;Save destination socket
	movwi	RCVTMP0[FSR1]	; "
	retlw	low DdpSSck | 1	;Transition to wait for source socket
DdpSSck	retlw	low LapDest | 1	;If an escape sequence, transition to LapDest
	movf	INDF1,W		;Save source socket
	movwi	RCVSOCK[FSR1]	; "
	retlw	low DdpType | 1	;Transition to wait for DDP type
DdpType	retlw	low LapDest | 1	;If an escape sequence, transition to LapDest
	decf	INDF1,W		;If DDP type is 1 (RTMP data), transition to
	btfsc	STATUS,Z	; wait for router network
	retlw	low RtmpNwH | 1	; "
	decf	WREG,W		;If DDP type is 2 (NBP), transition to wait for
	btfsc	STATUS,Z	; NBP function
	retlw	low NbpFunc | 1	; "
	decf	WREG,W		;If DDP type is 3 (ATP), transition to wait for
	btfsc	STATUS,Z	; ATP control byte
	retlw	low AtpCtrl | 1	; "
	retlw	low WaitIgn | 1	;Else, wait for the frame to be over
	nop
RtmpNwH	retlw	low LapDest | 1	;If an escape sequence, transition to LapDest
	movf	INDF1,W		;Save router network high byte
	movwi	RCVNETH[FSR1]	; "
	retlw	low RtmpNwL | 1	;Transition to wait for router network low byte
RtmpNwL	retlw	low LapDest | 1	;If an escape sequence, transition to LapDest
	movf	INDF1,W		;Save router network high byte
	movwi	RCVNETL[FSR1]	; "
	retlw	low RtmpNLn | 1	;Transition to wait for router network node len
RtmpNLn	retlw	low LapDest | 1	;If an escape sequence, transition to LapDest
	movf	INDF1,W		;If node length is 8 (bits), transition to wait
	xorlw	8		; for node number
	btfsc	STATUS,Z	; "
	retlw	low RtmpNod | 1	; "
	retlw	low WaitIgn | 1	;Else, wait for the frame to be over
RtmpNod	retlw	low LapDest | 1	;If an escape sequence, transition to LapDest
	movf	INDF1,W		;Save router node high byte
	movwi	RCVNODE[FSR1]	; "
	retlw	low RtmpFin | 1	;Transition to wait for rest of RTMP datagram
RtmpFin	bra	$+2		;If not an escape sequence, transition to self
	retlw	low RtmpFin | 1	; "
	movf	INDF1,W		;If an escape sequence other than frame-done,
	xorlw	0xFD		; transition to LapDest
	btfss	STATUS,Z	; "
	retlw	low LapDest | 1	; "
	moviw	RCVNETH[FSR1]	;Accept this network number as our own and the
	movwi	NETWRKH[FSR1]	; router as our router
	moviw	RCVNETL[FSR1]	; "
	movwi	NETWRKL[FSR1]	; "
	moviw	RCVNODE[FSR1]	; "
	movwi	AROUTER[FSR1]	; "
	movlw	15		;Set a timeout timer for 30 seconds so this info
	movwi	RTTMOUT[FSR1]	; ages out if the router goes away
	retlw	low LapDest | 1	;Transition to wait for beginning of next frame
	nop
NbpFunc	retlw	low LapDest | 1	;If an escape sequence, transition to LapDest
	movf	INDF1,W		;If the control byte is 0x21 (LkUp, one tuple),
	xorlw	0x21		; transition to wait for the NBP ID
	btfsc	STATUS,Z	; "
	retlw	low NbpId | 1	; "
	retlw	low WaitIgn | 1	;Else, wait for the frame to be over
NbpId	retlw	low LapDest | 1	;If an escape sequence, transition to LapDest
	movf	INDF1,W		;Save the NBP ID
	movwi	RCVTMP0[FSR1]	; "
	retlw	low NbpRNwH | 1	;Transition to wait for the requester's network
NbpRNwH	retlw	low LapDest | 1	;If an escape sequence, transition to LapDest
	movf	INDF1,W		;Save the high byte of the requester's network
	movwi	RCVNETH[FSR1]	; "
	retlw	low NbpRNwL | 1	;Transition to wait for the low byte
NbpRNwL	retlw	low LapDest | 1	;If an escape sequence, transition to LapDest
	movf	INDF1,W		;Save the low byte of the requester's network
	movwi	RCVNETL[FSR1]	; "
	retlw	low NbpRNod | 1	;Transition to wait for the node number
NbpRNod	retlw	low LapDest | 1	;If an escape sequence, transition to LapDest
	movf	INDF1,W		;Save the requester's node number
	movwi	RCVNODE[FSR1]	; "
	retlw	low NbpRSck | 1	;Transition to wait for the socket number
NbpRSck	retlw	low LapDest | 1	;If an escape sequence, transition to LapDest
	movf	INDF1,W		;Save the requester's socket number
	movwi	RCVSOCK[FSR1]	; "
	retlw	low NbpEnum | 1	;Transition to wait for the enumerator
NbpEnum	retlw	low LapDest | 1	;If an escape sequence, transition to LapDest
	retlw	low NbpObjL | 1	;Transition to wait for the object length
NbpObjL	retlw	low LapDest | 1	;If an escape sequence, transition to LapDest
	movf	INDF1,W		;Store the length of the LkUp's object string
	movwi	RCVTMP1[FSR1]	; "
	retlw	low NbpObj1 | 1	;Transition to wait for its first character
NbpObj1	retlw	low LapDest | 1	;If an escape sequence, transition to LapDest
	moviw	RCVTMP1[FSR1]	;If the length of the LkUp's object string is 1
	xorlw	1		; and its only character is '=', transition to
	btfss	STATUS,Z	; wait for the type (since = is a wildcard that
	bra	NbpObj + 1	; matches everything), else fall into NbpObj
	movf	INDF1,W		; "
	xorlw	'='		; "
	btfss	STATUS,Z	; "
	bra	NbpObj + 1	; "
	retlw	low NbpTypL | 1	; "
NbpObj	retlw	low LapDest | 1	;If an escape sequence, transition to LapDest
	movf	INDF1,W		;Capitalize the ASCII byte received
	call	UcaseW		; "
	movwf	INDF1		; "
	moviw	RCVTMP1[FSR1]	;Decrement the length of the LkUp's object
	addlw	-1		; string
	movwi	RCVTMP1[FSR1]	; "
	xorlw	B'11111111'	;Complement the decremented length and add it to
	addlw	low NbpTbOE	; the address of the terminating null of our
	movwf	FSR0L		; object string; this gives us the address of
	movlw	high NbpTbO | 0x80; what should be the corresponding byte in our
	movwf	FSR0H		; object string; store it in FSR0
	moviw	FSR0++		;Grab corresponding byte from object string,
	call	UcaseW		; advance the pointer, and capitalize the byte
	xorwf	INDF1,W		;Compare it to the capitalized received byte
	btfss	STATUS,Z	;If they don't match, transition to ignore the
	retlw	low WaitIgn | 1	; rest of the datagram
	moviw	RCVTMP1[FSR1]	;If the remaining length of the LkUp's object
	btfss	STATUS,Z	; string is nonzero, transition to self
	retlw	low NbpObj | 1	; "
	movf	INDF0,W		;If both strings ended together, transition to
	btfsc	STATUS,Z	; wait for the type length
	retlw	low NbpTypL | 1	; "
	retlw	low WaitIgn | 1	;Else, transition to wait for datagram to end
NbpTypL	retlw	low LapDest | 1	;If an escape sequence, transition to LapDest
	movf	INDF1,W		;Store the length of the LkUp's type string
	movwi	RCVTMP1[FSR1]	; "
	retlw	low NbpTyp1 | 1	;Transition to wait for its first character
NbpTyp1	retlw	low LapDest | 1	;If an escape sequence, transition to LapDest
	moviw	RCVTMP1[FSR1]	;If the length of the LkUp's type string is 1
	xorlw	1		; and its only character is '=', transition to
	btfss	STATUS,Z	; wait for the zone (since = is a wildcard that
	bra	NbpTyp + 1	; matches everything), else fall into NbpTyp
	movf	INDF1,W		; "
	xorlw	'='		; "
	btfss	STATUS,Z	; "
	bra	NbpTyp + 1	; "
	retlw	low NbpRest | 1	; "
NbpTyp	retlw	low LapDest | 1	;If an escape sequence, transition to LapDest
	movf	INDF1,W		;Capitalize the ASCII byte received
	call	UcaseW		; "
	movwf	INDF1		; "
	moviw	RCVTMP1[FSR1]	;Decrement the length of the LkUp's type string
	addlw	-1		; "
	movwi	RCVTMP1[FSR1]	; "
	xorlw	B'11111111'	;Complement the decremented length and add it to
	addlw	low NbpTbTE	; the address of the terminating null of our
	movwf	FSR0L		; type string; this gives us the address of
	movlw	high NbpTbT | 0x80; what should be the corresponding byte in our
	movwf	FSR0H		; type string; store it in FSR0
	moviw	FSR0++		;Grab corresponding byte from type string,
	call	UcaseW		; advance the pointer, and capitalize the byte
	xorwf	INDF1,W		;Compare it to the capitalized received byte
	btfss	STATUS,Z	;If they don't match, transition to ignore the
	retlw	low WaitIgn | 1	; rest of the datagram
	moviw	RCVTMP1[FSR1]	;If the remaining length of the LkUp's type
	btfss	STATUS,Z	; string is nonzero, transition to self
	retlw	low NbpTyp | 1	; "
	movf	INDF0,W		;If both strings ended together, transition to
	btfsc	STATUS,Z	; wait for the zone
	retlw	low NbpRest | 1	; "
	retlw	low WaitIgn | 1	;Else, transition to wait for datagram to end
NbpRest	bra	Nbp0		;If not an escape sequence, transition to self,
	retlw	low NbpRest | 1	; else skip ahead
AtpCtrl	retlw	low LapDest | 1	;If an escape sequence, transition to LapDest
	movf	INDF1,W		;Store the control byte
	movwi	RCVTMP0[FSR1]	; "
	retlw	low AtpBmap | 1	;Transition to await bitmap
AtpBmap	retlw	low LapDest | 1	;If an escape sequence, transition to LapDest
	retlw	low AtpTidH | 1	;Ignore bitmap, transition to wait for the TID
AtpTidH	retlw	low LapDest | 1	;If an escape sequence, transition to LapDest
	movf	INDF1,W		;Store the high byte of the TID
	movwi	RCVTMP1[FSR1]	; "
	retlw	low AtpTidL | 1	;Transition to await low byte
AtpTidL	retlw	low LapDest | 1	;If an escape sequence, transition to LapDest
	bra	Atp5		;Skip ahead
AtpReq1	retlw	low LapDest | 1	;If an escape sequence, transition to LapDest
	retlw	low AtpReq2 | 1	;Ignore first user byte, wait for second
AtpReq2	retlw	low LapDest | 1	;If an escape sequence, transition to LapDest
	retlw	low AtpReq3 | 1	;Ignore second user byte, wait for third
AtpReq3	retlw	low LapDest | 1	;If an escape sequence, transition to LapDest
	retlw	low AtpReq4 | 1	;Ignore third user byte, wait for fourth
AtpReq4	retlw	low LapDest | 1	;If an escape sequence, transition to LapDest
	retlw	low AtpReqF | 1	;Ignore fourth user byte, wait for payload
AtpRelF	bra	Atp2		;If not an escape sequence, transition to self,
	retlw	low AtpRelF | 1	; else jump ahead
AtpRtrF	bra	Atp0		;If not an escape sequence, transition to self,
	retlw	low AtpRtrF | 1	; else jump ahead
AtpReqF	bra	Atp1		;If an escape sequence, jump ahead
	movf	INDF1,W		;Store the incoming data byte, using RCVSTAT as
	movwi	RCVSTAT[FSR1]	; temporary storage
	call	AtpMbox		;Point FSR0 to mailbox we've found to be free
	moviw	RCVSTAT[FSR1]	;Restore the incoming data byte
	movwf	INDF1		; "
	btfsc	INDF0,MBXMITD	;If the transmitted flag is raised, the buffer
	retlw	low AtpReqF | 1	; count can't go any higher, discard this byte
	movf	INDF0,W		;Get the count of received bytes pre-increment
	incf	INDF0,F		;Increment the count of received bytes
	btfsc	WREG,MBNBP	;If there is not space in the buffer for this
	retlw	low AtpReqF | 1	; byte, discard it
	andlw	B'00000111'	;Store the incoming data byte in the buffer
	iorlw	B'00001000'	; "
	addwf	FSR0L,F		; "
	movf	INDF1,W		; "
	movwf	INDF0		; "
	retlw	low AtpReqF | 1	;Return to await the next byte
Atp0	movf	INDF1,W		;If an escape sequence other than frame-done,
	xorlw	0xFD		; transition to LapDest
	btfss	STATUS,Z	; "
	retlw	low LapDest | 1	; "
	call	AtpMbox		;Find the existing mailbox where the XO TReq is
	bcf	INDF0,MBXMITD	; located, mark it as needing transmit
	movlb	1		;If this is a TResp, enable the Tx interrupt so
	btfsc	INDF0,MBRESP	; it gets sent
	bsf	PIE1,TXIE	; "
	retlw	low LapDest | 1	;Transition to receive next frame
Atp1	movf	INDF1,W		;If an escape sequence other than frame-done,
	xorlw	0xFD		; transition to LapDest
	btfss	STATUS,Z	; "
	retlw	low LapDest | 1	; "
	call	AtpMbox		;Point FSR0 to mailbox we've found to be free
	bsf	INDF0,MBINUSE	;Set the in use flag so mainline knows
	movf	INDF0,W		;Get the count of received bytes (which maxes
	andlw	B'00011111'	; out at 16)
	addlw	-3		;Cut off the FCS bytes, adjust the count
	btfss	STATUS,C	;If we received no data payload and/or no FCS,
	bcf	INDF0,MBINUSE	; act like we didn't get the TReq at all
	btfsc	WREG,MBNBP	;If we got more than 8 bytes, say that we got
	movlw	B'00000111'	; 8 bytes - we didn't store past the 8th anyway
	bcf	INDF0,MBXMITD	;Clear the actual count of received bytes and
	bcf	INDF0,MBNBP	; replace it with the adjusted count and the
	bcf	INDF0,MBLEN2	; transmitted and NBP flags clear
	bcf	INDF0,MBLEN1	; "
	bcf	INDF0,MBLEN0	; "
	iorwf	INDF0,F		; "
	retlw	low LapDest | 1	;Return to await next datagram
Atp2	movf	INDF1,W		;If an escape sequence other than frame-done,
	xorlw	0xFD		; transition to LapDest
	btfss	STATUS,Z	; "
	retlw	low LapDest | 1	; "
	movlw	PA_MBOX		;Count down how many mailboxes are allocated,
Atp3	movwi	RCVSTAT[FSR1]	; using RCVSTAT as storage for it
	addfsr	FSR0,0x10	;Advance to the next mailbox
	btfss	INDF0,MBINUSE	;If the mailbox is in use, continue; else
	bra	Atp4		; proceed to the next mailbox if there is one
	btfsc	INDF0,MBNBP	;If the mailbox is used by ATP, continue; else
	bra	Atp4		; proceed to the next mailbox if there is one
	call	AtpComp		;If the TID, network, node, and socket match,
	btfss	STATUS,Z	; continue; else proceed to the next mailbox if
	bra	Atp4		; there is one
	clrf	INDF0		;Found the right mailbox, clear its flags
	retlw	low LapDest | 1	;Return to process next datagram
Atp4	moviw	RCVSTAT[FSR1]	;Decrement the mailbox counter and loop if there
	decfsz	WREG,W		; are more to check
	bra	Atp3		; "
	retlw	low LapDest | 1	;Ignore datagram, no such mailbox to release
Atp5	moviw	RCVTMP0[FSR1]	;Shuffle things around a bit so the TID is in
	movwi	RCVSTAT[FSR1]	; RCVTMP1:0 and the control byte, formerly in
	movf	INDF1,W		; RCVTMP0, is in INDF1
	movwi	RCVTMP0[FSR1]	; "
	moviw	RCVSTAT[FSR1]	; "
	movwf	INDF1		; "
	btfss	INDF1,6		;Function codes we can deal with are 01=TReq and
	retlw	low WaitIgn | 1	; 11=TRel, others are invalid
	btfsc	INDF1,7		;If it's a TRel, the rest of the header isn't
	retlw	low AtpRelF | 1	; important, so move on to await end of datagram
	movlw	PA_MBOX		;Count down how many mailboxes are allocated
	btfss	INDF1,5		;If this is an ALO TReq, don't bother checking
	bra	Atp8		; if we have an existing response for it
Atp6	movwi	RCVSTAT[FSR1]	;Use RCVSTAT temporarily to hold mailbox count
	addfsr	FSR0,0x10	;Advance to the next mailbox
	movf	INDF0,W		;Check some flags of this mailbox
	andlw	(1 << MBINUSE) | (1 << MBXO) | (1 << MBNBP)
	xorlw	(1 << MBINUSE) | (1 << MBXO)
	btfss	STATUS,Z	;If this mailbox contains an ATP XO TReq, go on
	bra	Atp7		;Else proceed to next mailbox if there is one
	call	AtpComp		;If the TID, network, node, and socket match,
	btfss	STATUS,Z	; continue; else proceed to the next mailbox if
	bra	Atp7		; there is one
	moviw	RCVSTAT[FSR1]	;If we found an existing XO TReq matching this
	movwi	RCVTMP1[FSR1]	; one, move its mailbox count into RCVTMP1 where
	retlw	low AtpRtrF | 1	; AtpMbox expects it and wait for datagram end
Atp7	moviw	RCVSTAT[FSR1]	;Decrement the mailbox counter and loop if there
	decfsz	WREG,W		; are more to check
	bra	Atp6		; "
	movf	FSR1H,W		;No existing XO TReq found, rewind to beginning
	movwf	FSR0H		; of vars
	movf	FSR1L,W		; "
	movwf	FSR0L		; "
	movlw	PA_MBOX		;Count down how many mailboxes are allocated
Atp8	addfsr	FSR0,0x10	;Advance to the next mailbox
	btfsc	INDF0,MBINUSE	;If it's in use, loop to the next
	bra	Atp9		; "
	movwi	RCVSTAT[FSR1]	;Make note that this is the mailbox we're using
	moviw	RCVTMP1[FSR1]	;Store the mailbox parameters
	movwi	MBSTIDH[FSR0]	; "
	moviw	RCVTMP0[FSR1]	; "
	movwi	MBSTIDL[FSR0]	; "
	moviw	RCVNETH[FSR1]	; "
	movwi	MBSNETH[FSR0]	; "
	moviw	RCVNETL[FSR1]	; "
	movwi	MBSNETL[FSR0]	; "
	moviw	RCVNODE[FSR1]	; "
	movwi	MBSNODE[FSR0]	; "
	moviw	RCVSOCK[FSR1]	; "
	movwi	MBSSOCK[FSR0]	; "
	moviw	RCVSTAT[FSR1]	;Move the mailbox countdown number to RCVTMP1
	movwi	RCVTMP1[FSR1]	; where AtpMbox expects to find it
	clrf	INDF0		;Zero out the mailbox's flags
	btfss	INDF1,5		;If this is an ALO transaction, move on to await
	retlw	low AtpReq1 | 1	; user bytes
	bsf	INDF0,MBXO	;This is an XO transaction, set the XO flag
	comf	INDF1,W		;Use this weird lookup table to calculate the
	andlw	B'00000111'	; TRel timeout in 2-second periods from the
	brw			; three low bits of the control byte
	addlw	1		;0b111 = 240 (8 minutes; should be invalid)
	addlw	1		;0b110 = 240 (8 minutes; should be invalid)
	addlw	1		;0b101 = 240 (8 minutes; should be invalid)
	addlw	121		;0b100 = 240 (8 minutes)
	addlw	61		;0b011 = 120 (4 minutes)
	addlw	31		;0b010 = 60 (2 minutes)
	addlw	16		;0b001 = 30 (1 minute)
	addlw	8		;0b000 = 15 (30 seconds)
	movwi	MBTMOUT[FSR0]	;Store it
	retlw	low AtpReq1 | 1	;Move on to await user bytes
Atp9	decfsz	WREG,W		;Decrement the mailbox counter and loop if there
	bra	Atp8		; are more to check
	retlw	low WaitIgn | 1	;Ignore datagram; no mailbox to keep it in
AtpMbox	movlw	PA_MBOX + 1	;Point FSR0 to the mailbox we've found to be
	movwf	INDF1		; free
	moviw	RCVTMP1[FSR1]	; "
	subwf	INDF1,W		; "
	addfsr	FSR0,0x10	;Advance FSR0 by one mailbox-length
	decfsz	WREG,W		;Decrement W, so 1 is first mailbox
	bra	$-2		;Loop until we've reached the desired mailbox
	return			;Done
AtpComp	moviw	RCVTMP0[FSR1]	;If the low byte of the TID matches, continue;
	movwf	INDF1		; else return with Z clear
	moviw	MBSTIDL[FSR0]	; "
	xorwf	INDF1,W		; "
	btfss	STATUS,Z	; "
	return			; "
	moviw	RCVTMP1[FSR1]	;If the high byte of the TID matches, continue;
	movwf	INDF1		; else return with Z clear
	moviw	MBSTIDH[FSR0]	; "
	xorwf	INDF1,W		; "
	btfss	STATUS,Z	; "
	return			; "
	moviw	RCVSOCK[FSR1]	;If the socket matches, continue; else return
	movwf	INDF1		; with Z clear
	moviw	MBSSOCK[FSR0]	; TODO do we check the socket?
	xorwf	INDF1,W		; "
	btfss	STATUS,Z	; "
	return			; "
	moviw	RCVNODE[FSR1]	;If the node number matches, continue; else
	movwf	INDF1		; return with Z clear
	moviw	MBSNODE[FSR0]	; "
	xorwf	INDF1,W		; "
	btfss	STATUS,Z	; "
	return			; "
	moviw	RCVNETL[FSR1]	;If the low byte of the network number matches,
	movwf	INDF1		; continue; else return with Z clear
	moviw	MBSNETL[FSR0]	; "
	xorwf	INDF1,W		; "
	btfss	STATUS,Z	; "
	return			; "
	moviw	RCVNETH[FSR1]	;If the high byte of the network number matches,
	movwf	INDF1		; return with Z set, else return with Z clear
	moviw	MBSNETH[FSR0]	; "
	xorwf	INDF1,W		; "
	return			; "
Nbp0	movf	INDF1,W		;If an escape sequence other than frame-done,
	xorlw	0xFD		; transition to LapDest
	btfss	STATUS,Z	; "
	retlw	low LapDest | 1	; "
	movlw	PA_MBOX		;Count down how many mailboxes are allocated
Nbp1	addfsr	FSR0,0x10	;Advance to the next mailbox
	btfsc	INDF0,MBINUSE	;If it's in use, loop to the next
	bra	Nbp2		; "
	moviw	RCVNETH[FSR1]	;Mailbox is free, so use it to contain a request
	movwi	MBSNETH[FSR0]	; to respond to an NBP LkUp
	moviw	RCVNETL[FSR1]	; "
	movwi	MBSNETL[FSR0]	; "
	moviw	RCVNODE[FSR1]	; "
	movwi	MBSNODE[FSR0]	; "
	moviw	RCVSOCK[FSR1]	; "
	movwi	MBSSOCK[FSR0]	; "
	moviw	RCVTMP0[FSR1]	; "
	movwi	MBSTIDH[FSR0]	; "
	movlw	(1 << MBINUSE) | (1 << MBNBP); "
	movwf	INDF0		; "
	movlb	1		;Enable the Tx interrupt so it gets sent
	bsf	PIE1,TXIE	; "
	retlw	low LapDest | 1	;Return to process next datagram
Nbp2	decfsz	WREG,W		;Decrement the mailbox counter and loop if there
	bra	Nbp1		; are more to check
	retlw	low WaitIgn | 1	;Ignore datagram; no mailbox to keep it in
UcaseW	addlw	-0x61		;If W is between 0x61 ('a') and 0x7A ('z'),
	addlw	-27		; clear C, else set it
	btfss	STATUS,C	;If C is clear, subtract 0x20 from W,
	addlw	-0x20		; capitalizing it
	addlw	0x61 + 27	;Restore W to its previous range
	return			;Done


;;; Data Tables ;;;

NbpTbO	dt	PA_OBJ
NbpTbOE	dt	0
NbpTbT	dt	PA_TYPE
NbpTbTE	dt	0


;;; Transmitter State Machine ;;;

XFindMb	moviw	XMITMP0[FSR1]	;Point FSR0 at the mailbox we're working on
	swapf	WREG,W		; "
	andlw	B'11110000'	; "
	movwf	FSR0L		; "
	moviw	XMITMP0[FSR1]	; "
	swapf	WREG,W		; "
	andlw	B'00001111'	; "
	iorlw	0x20		; "
	movwf	FSR0H		; "
	return			;Done

	org	0x300

TransmitFsa
XDecide	movlw	PA_MBOX		;Count down how many mailboxes are allocated
Xmit0	addfsr	FSR0,0x10	;Advance to the next mailbox
	btfsc	INDF0,MBINUSE	;If the mailbox is not in use or it doesn't need
	btfsc	INDF0,MBXMITD	; transmitting, skip to the next if there is one
	bra	Xmit1		; "
	movlw	0x01		;Send an 0x01 (Transmit Frame) command
	movwf	TXREG		; "
	bcf	FSR0H,5		;Store FSR0[11:4] (the offset to the mailbox,
	swapf	FSR0H,F		; assuming mailboxes are 16-bit aligned, which
	swapf	FSR0L,W		; they should be) in XMITMP0 for use by later
	iorwf	FSR0H,W		; states
	movwi	XMITMP0[FSR1]	; "
	retlw	low XDest	;Return to send the destination
Xmit1	decfsz	WREG,W		;Decrement the mailbox counter and loop if there
	bra	Xmit0		; are more to check
	movlb	1		;Disable the Tx interrupt, we have nothing to
	bcf	PIE1,TXIE	; transmit
	retlw	low XDecide	;Return to scan mailboxes again when reenabled
XDest	call	XFindMb		;Point FSR0 at the mailbox we're working on
	moviw	MBSNETH[FSR0]	;If destination network is zero, use short DDP
	movwf	INDF1		; header and send the frame directly to its
	moviw	MBSNETL[FSR0]	; destination
	iorwf	INDF1,W		; "
	btfsc	STATUS,Z	; "
	bra	Xmit2		; "
	moviw	NETWRKH[FSR1]	;Compare our network with the destination's; if
	xorwf	INDF1,W		; they're equal, use short DDP header and send
	btfss	STATUS,Z	; the frame directly to its destination
	bra	Xmit3		; "
	moviw	MBSNETL[FSR0]	; "
	movwf	INDF1		; "
	moviw	NETWRKL[FSR1]	; "
	xorwf	INDF1,W		; "
	btfss	STATUS,Z	; "
	bra	Xmit3		; "
Xmit2	moviw	MBSNODE[FSR0]	;If using the short DDP header, the destination
	movwf	TXREG		; node is the actual destination node number;
	retlw	low XShSrc	; transmit it and return to send the source next
Xmit3	moviw	AROUTER[FSR1]	;If using the long DDP header, the destination
	movwf	TXREG		; node is the last known router; transmit it and
	retlw	low XExSrc	; return to send the source next
XShSrc	moviw	OURNODE[FSR1]	;Source node is our node number
	movwf	TXREG		; "
	retlw	low XShLapT	;Return to send the LLAP type next
XExSrc	moviw	OURNODE[FSR1]	;Source node is our node number
	movwf	TXREG		; "
	retlw	low XExLapT	;Return to send the LLAP type next
XShLapT	movlw	0x01		;LLAP type is 0x01 for short DDP header
	movwf	TXREG		; "
	retlw	low XShLenH	;Return to send the length next
XExLapT	movlw	0x02		;LLAP type is 0x02 for extended DDP header
	movwf	TXREG		; "
	retlw	low XExLenH	;Return to send the length next
XShLenH	clrf	TXREG		;High byte of length is always zero
	retlw	low XShLenL	;Return to send the low byte next
XExLenH	clrf	TXREG		;High byte of length is always zero
	retlw	low XExLenL	;Return to send the low byte next
XShLenL	call	XFindMb		;Point FSR0 at the mailbox we're working on
	btfss	INDF0,MBNBP	;If this mailbox contains an ATP payload, skip
	bra	Xmit4		; ahead
	call	XNbpLen		;Mailbox contains an NBP payload, reckon length
	addlw	14		; of object and type fields and add 14 to this
	movwf	TXREG		;Transmit it
	retlw	low XDestSk	;Return to send destination socket next
Xmit4	incf	INDF0,W		;Mailbox contains an ATP payload, length is the
	andlw	B'00001111'	; payload length plus ATP header (4 bytes), user
	addlw	13		; bytes (4 bytes), and DDP header (5 bytes)
	movwf	TXREG		;Transmit it
	retlw	low XDestSk	;Return to send destination socket next
XExLenL	call	XFindMb		;Point FSR0 at the mailbox we're working on
	btfss	INDF0,MBNBP	;If this mailbox contains an ATP payload, skip
	bra	Xmit5		; ahead
	call	XNbpLen		;Mailbox contains an NBP payload, reckon length
	addlw	22		; of object and type fields and add 22 to this
	movwf	TXREG		;Transmit it
	retlw	low XChkSmH	;Return to send checksum next
Xmit5	incf	INDF0,W		;Mailbox contains an ATP payload, length is the
	andlw	B'00001111'	; payload length plus ATP header (4 bytes), user
	addlw	21		; bytes (4 bytes), and DDP header (13 bytes)
	movwf	TXREG		;Transmit it
	retlw	low XChkSmH	;Return to send checksum next
XChkSmH	clrf	TXREG		;Not calculating a DDP checksum
	retlw	low XChkSmL	;Return to send low byte next
XChkSmL	clrf	TXREG		;Not calculating a DDP checksum
	retlw	low XDestNH	;Return to send destination network next
XDestNH	call	XFindMb		;Point FSR0 at the mailbox we're working on
	moviw	MBSNETH[FSR0]	;Send high byte of network number
	movwf	TXREG		; "
	retlw	low XDestNL	;Return to send low byte next
XDestNL	call	XFindMb		;Point FSR0 at the mailbox we're working on
	moviw	MBSNETL[FSR0]	;Send low byte of network number
	movwf	TXREG		; "
	retlw	low XSrcNH	;Return to send source network next
XSrcNH	moviw	NETWRKH[FSR1]	;Send high byte of our network number
	movwf	TXREG		; "
	retlw	low XSrcNL	;Return to send low byte next
XSrcNL	moviw	NETWRKL[FSR1]	;Send low byte of our network number
	movwf	TXREG		; "
	retlw	low XDestNd	;Return to send destination node next
XDestNd	call	XFindMb		;Point FSR0 at the mailbox we're working on
	moviw	MBSNODE[FSR0]	;Send node number
	movwf	TXREG		; "
	retlw	low XSrcNd	;Return to send source node next
XSrcNd	moviw	OURNODE[FSR1]	;Send our node number
	movwf	TXREG		; "
	retlw	low XDestSk	;Return to send destination socket next
XDestSk	call	XFindMb		;Point FSR0 at the mailbox we're working on
	moviw	MBSSOCK[FSR0]	;Send socket number
	movwf	TXREG		; "
	retlw	low XSrcSk	;Return to send source socket next
XSrcSk	call	XFindMb		;Point FSR0 at the mailbox we're working on
	btfss	INDF0,MBNBP	;If this mailbox contains an ATP payload, skip
	bra	Xmit6		; ahead
	movlw	2		;Send the SAS for NBP
	movwf	TXREG		; "
	retlw	low XNbpTyp	;Return to send the DDP type next
Xmit6	movlw	254		;Send our 'dynamic' socket for ATP
	movwf	TXREG		; "
	retlw	low XAtpTyp	;Return to send the DDP type next
XNbpTyp	movlw	2		;Send the DDP type for NBP
	movwf	TXREG		; "
	retlw	low XNbpFnc	;Return to send the NBP function byte next
XNbpFnc	movlw	0x31		;Return 0x31 (LkUp-Reply, 1 tuple) for the
	movwf	TXREG		; function byte
	retlw	low XNbpId	;Return to send the NBP ID next
XNbpId	call	XFindMb		;Point FSR0 at the mailbox we're working on
	moviw	MBSTIDH[FSR0]	;Send the NBP ID of the request we're responding
	movwf	TXREG		; to
	bcf	INDF0,MBINUSE	;Mark the mailbox as no longer being in use
	retlw	low XNbpNwH	;Return to send our network next
XNbpNwH	moviw	NETWRKH[FSR1]	;Send the high byte of our network number
	movwf	TXREG		; "
	retlw	low XNbpNwL	;Return to send the low byte next
XNbpNwL	moviw	NETWRKL[FSR1]	;Send the low byte of our network number
	movwf	TXREG		; "
	retlw	low XNbpNod	;Return to send our node next
XNbpNod	moviw	OURNODE[FSR1]	;Send the low byte of our node number
	movwf	TXREG		; "
	retlw	low XNbpSck	;Return to send our socket next
XNbpSck	movlw	254		;Send our 'dynamic' socket for ATP
	movwf	TXREG		; "
	retlw	low XNbpEnu	;Return to send the enumerator next
XNbpEnu	clrf	TXREG		;Enumerator is always zero
	retlw	low XNbpDa1	;Return to send the object field length next
XNbpDa1	movlw	high NbpTbO | 0x80;Reckon the length of the NBP object field
	movwf	FSR0H		; "
	movlw	low NbpTbO	; "
	movwi	XMITMP0[FSR1]	; "
	movwf	FSR0L		; "
	clrf	INDF1		; "
Xmit7	incf	INDF1,F		; "
	moviw	FSR0++		; "
	btfss	STATUS,Z	; "
	bra	Xmit7		; "
	decf	INDF1,W		;Transmit it (not counting trailing null)
	movwf	TXREG		; "
	retlw	low XNbpDa2	;Return to send the object field itself next
XNbpDa2	movlw	high NbpTbO | 0x80;Prepare to get a byte from the object or type
	movwf	FSR0H		; fields
	moviw	XMITMP0[FSR1]	;Load the pointer
	movwf	FSR0L		; "
	movf	INDF0,W		;Dereference it; if the byte is not a null,
	btfss	STATUS,Z	; skip ahead to transmit it
	bra	Xmit9		; "
	movlw	low NbpTbT	;The byte was a null so we have to reckon the
	movwi	XMITMP0[FSR1]	; length of the type field now
	movwf	FSR0L		; "
	clrf	INDF1		; "
Xmit8	incf	INDF1,F		; "
	moviw	FSR0++		; "
	btfss	STATUS,Z	; "
	bra	Xmit8		; "
	decf	INDF1,W		;Transmit it (not counting trailing null)
	movwf	TXREG		; "
	retlw	low XNbpDa3	;Return to send the type field itself next
Xmit9	movwf	TXREG		;Transmit it
	incf	FSR0L,W		;Increment the pointer for next time
	movwi	XMITMP0[FSR1]	; "
	retlw	low XNbpDa2	;Return to self
XNbpDa3	movlw	high NbpTbO | 0x80;Prepare to get a byte from the type field
	movwf	FSR0H		; "
	moviw	XMITMP0[FSR1]	;Load the pointer
	movwf	FSR0L		; "
	movf	INDF0,W		;Dereference it; if the byte is not a null,
	btfss	STATUS,Z	; skip ahead to transmit it
	bra	XmitA		; "
	movlw	0x01		;Transmit the length of the zone (always 1, '*')
	movwf	TXREG		; "
	retlw	low XNbpDa4	;Return to send the zone field itself next
XmitA	movwf	TXREG		;Transmit it
	incf	FSR0L,W		;Increment the pointer for next time
	movwi	XMITMP0[FSR1]	; "
	retlw	low XNbpDa3	;Return to self
XNbpDa4	movlw	'*'		;Transmit the zone field, '*' since we're on a
	movwf	TXREG		; nonextended network
	retlw	low XFcs1	;Payload is over
XAtpTyp	movlw	3		;Send the DDP type for ATP
	movwf	TXREG		; "
	retlw	low XAtpCtl	;Return to send the ATP control byte next
XAtpCtl	call	XFindMb		;Point FSR0 at the mailbox we're working on
	movlw	0x90		;Return 0x90 for an ALO response (TResp, EOM)
	btfsc	INDF0,MBXO	; and 0xB0 (TResp, EOM, XO) for an XO response
	movlw	0xB0		; "
	movwf	TXREG		; "
	retlw	low XAtpBmp	;Return to send the bitmap next
XAtpBmp	clrf	TXREG		;Our reply is always sequence number 0
	retlw	low XAtpTdH	;Return to send the TID next
XAtpTdH	call	XFindMb		;Point FSR0 at the mailbox we're working on
	moviw	MBSTIDH[FSR0]	;Send the high byte of the TID
	movwf	TXREG		; "
	retlw	low XAtpTdL	;Return to send the low byte next
XAtpTdL	call	XFindMb		;Point FSR0 at the mailbox we're working on
	moviw	MBSTIDL[FSR0]	;Send the low byte of the TID
	movwf	TXREG		; "
	retlw	low XAtpUB0	;Return to send the first user byte next
XAtpUB0	clrf	TXREG		;First user byte is always zero
	retlw	low XAtpUB1	;Return to send the second user byte next
XAtpUB1	clrf	TXREG		;Second user byte is always zero
	retlw	low XAtpUB2	;Return to send the third user byte next
XAtpUB2	clrf	TXREG		;Third user byte is always zero
	retlw	low XAtpUB3	;Return to send the fourth user byte next
XAtpUB3	clrf	TXREG		;Fourth user byte is always zero
	retlw	low XAtpPl0	;Return to send the first byte of the payload
XAtpPl0	bra	XAtpPl7		;Skip ahead; this raft of labels is here so we
XAtpPl1	bra	XAtpPl7		; can use XMISTAT to store our current position
XAtpPl2	bra	XAtpPl7		; in the TResp payload
XAtpPl3	bra	XAtpPl7		; "
XAtpPl4	bra	XAtpPl7		; "
XAtpPl5	nop			; "
XAtpPl6	nop			; "
XAtpPl7	call	XFindMb		;Point FSR0 at the mailbox we're working on
	moviw	XMISTAT[FSR1]	;Figure out where we are in the payload based on
	addlw	8 - low XAtpPl0	; XMISTAT and point FSR0 to the next byte
	addwf	FSR0L,F		; "
	movf	INDF0,W		;Transmit the byte
	movwf	TXREG		; "
	movlw	B'11110000'	;Restore FSR0 to point at the beginning of the
	andwf	FSR0L,F		; mailbox
	moviw	XMISTAT[FSR1]	;Figure out where we are in the payload based on
	addlw	0 - low XAtpPl0	; XMISTAT again
	xorwf	INDF0,W		;Use it to check whether we've just transmitted
	andlw	B'00000111'	; the last byte
	btfsc	STATUS,Z	;If we haven't, proceed to the next state
	bra	XmitB		; "
	moviw	XMISTAT[FSR1]	; "
	addlw	1		; "
	return			; "
XmitB	bsf	INDF0,MBXMITD	;If we have, payload is over; mark that it's
	btfss	INDF0,MBXO	; been transmitted, free the mailbox if it's an
	bcf	INDF0,MBINUSE	; ALO transaction, and proceed to send the FCS
	retlw	low XFcs1	; "
XFcs1	clrf	TXREG		;Placeholder for first byte of FCS
	retlw	low XFcs2	;Return to send the second
XFcs2	clrf	TXREG		;Placeholder for second byte of FCS
	retlw	low XDecide	;End of transmission


;;; Mainline ;;;

Mainline
	banksel	T1CON		;Set up Timer1 regs to be just after CCP1 regs,
	clrf	TMR1H		; Timer1 ticks with instruction clock
	movlw	B'00000001'
	movwf	TMR1L
	movwf	T1CON

	banksel	CCP1CON		;CCP1 in compare mode, interrupt on match
	clrf	CCPR1H
	clrf	CCPR1L
	movlw	B'00001010'
	movwf	CCP1CON

	banksel	LATA		;Default state of pins is high
	movlw	B'00111111'
	movwf	LATA

	banksel	TRISA		;CCP1 pin is output
	bcf	TRISA,RA2

	clrf	UQPUSH		;Initialize key globals
	clrf	UQPOP

	;fall through

Main
	call	PollMailboxes	;Poll mailboxes for incoming activity
	call	PollCcp		;Poll whether the soft UART needs to be serviced
	bra	Main		;Loop

PollMailboxes
	movlw	PA_VARH		;Point FSR0 to the start of PicoATP's state
	movwf	FSR0H		; variables
	movlw	PA_VARL		; "
	movwf	FSR0L		; "
	movlw	PA_MBOX		;Count down the number of mailboxes
MboxLp	addfsr	FSR0,0x10	;Advance to the next mailbox
	btfss	INDF0,MBINUSE	;We're looking for a mailbox that is in use and
	bra	MboxNo		; contains an ATP TReq; if this isn't that, move
	btfsc	INDF0,MBRESP	; on to the next one (if there is one)
	bra	MboxNo		; "
	btfsc	INDF0,MBNBP	; "
	bra	MboxNo		; "
	movlw	0x20		;Point FSR1 to the soft UART queue's push point
	movwf	FSR1H		; "
	movf	UQPUSH,W	; "
	movwf	FSR1L		; "
	movf	INDF0,W		;Get the length of the incoming payload
	andlw	B'00000111'	; "
	btfsc	STATUS,Z	;If it's 1, skip ahead since it only contains
	bra	OnlyGio		; GPIO data
	movwf	X0		;Save the count of UART bytes as a countdown
	movlw	B'00001000'	;Point FSR0 to the first UART byte in the
	addwf	FSR0L,F		; payload
UartLp	moviw	++FSR0		;Put the next UART byte into the queue, wrapping
	movwi	FSR1++		; the pointer if necessary
	bcf	FSR1L,7		; "
	movf	FSR1L,W		;If we're about to overflow the queue, pretend
	xorwf	UQPOP,W		; we haven't received this TReq yet, we'll try
	btfsc	STATUS,Z	; again later
	return			; "
	decfsz	X0,F		;Loop to push the next UART byte, if there is
	bra	UartLp		; one
	movf	FSR1L,W		;Update the queue push pointer
	movwf	UQPUSH		; "
OnlyGio	movf	FSR0L,W		;Rewind FSR0 to point to the TReq payload's GPIO
	andlw	B'11110000'	; byte
	iorlw	B'00001000'	; "
	movwf	FSR0L		; "
	movf	INDF0,W		;Write output latches of PORTA with the two LSBs
	iorlw	B'00111100'	; "
	movlb	2		; "
	movwf	LATA		; "
	lsrf	INDF0,W		;Write TRISA with the next two bits up
	lsrf	WREG,W		; "
	andlw	B'00000011'	; "
	iorlw	B'00101000'	; "
	movlb	1		; "
	movwf	TRISA		; "
	movlw	B'11111100'	;Replace low two bits of the TReq payload with
	andwf	INDF0,F		; a current read of the GPIO pins
	movlb	0		; "
	movf	PORTA,W		; "
	andlw	B'00000011'	; "
	iorwf	INDF0,F		; "
	movlw	B'11110000'	;Set the flag that this mailbox now contains a
	andwf	FSR0L,F		; TResp to be sent
	bsf	INDF0,MBRESP	; "
	movlb	1		;Enable the Tx interrupt so the TResp can be
	bsf	PIE1,TXIE	; sent
	return			;Done
MboxNo	decfsz	WREG,W		;Decrement mailbox count and loop if there is
	bra	MboxLp		; another to check
	return			;Else, done

PollCcp
	movlb	0		;If there is no interrupt, return
	btfss	PIR1,CCP1IF	; "
	return			; "
	bcf	PIR1,CCP1IF	;Clear the interrupt
	movlb	5		;If the CCP output is on, skip ahead
	btfss	CCP1CON,CCP1M1	; "
	bra	PollCc0		; "
	movlw	0x03		;The CCP output is off, add 833 to CCPR1
	addwf	CCPR1H,F	; "
	movlw	0x41		; "
	addwf	CCPR1L,F	; "
	movlw	0		; "
	addwfc	CCPR1H,F	; "
	movf	UQPUSH,W	;If the queue is empty, return, we'll come back
	xorwf	UQPOP,W		; here in one bit time
	btfsc	STATUS,Z	; "
	return			; "
	movlw	B'10001001'	;Set CCP pin high (no change) and to go low on
	movwf	CCP1CON		; match, set the MSB as a flag that a new byte
	return			; is starting; return
PollCc0	movlw	0x20		;Point FSR0 to the queue pop point
	movwf	FSR0H		; "
	movf	UQPOP,W		; "
	movwf	FSR0L		; "
	btfss	CCP1CON,CCP1M0	;If the CCP output is set, skip ahead
	bra	PollCc2		; "
PollCc1	movlw	0x03		;The CCP output is clear, add 833 to CCPR1
	addwf	CCPR1H,F	; "
	movlw	0x41		; "
	addwf	CCPR1L,F	; "
	movlw	0		; "
	addwfc	CCPR1H,F	; "
	lslf	CCP1CON,W	;Set or clear carry depending on MSB of CCP1CON
	movlw	B'00001000'	;Set CCP to go high on match
	movwf	CCP1CON		; "
	rrf	INDF0,F		;Rotate carry into current byte
	btfsc	STATUS,C	;If we rotated a 1 bit out of the current byte,
	return			; return
	bra	PollCc1		;Else, loop
PollCc2	movlw	0x03		;The CCP output is set, add 833 to CCPR1
	addwf	CCPR1H,F	; "
	movlw	0x41		; "
	addwf	CCPR1L,F	; "
	movlw	0		; "
	addwfc	CCPR1H,F	; "
	lsrf	INDF0,F		;Shift the next bit out of the current byte
	btfsc	STATUS,Z	;If that was the last bit, skip ahead
	bra	PollCc3		; "
	btfss	STATUS,C	;If we shifted out a zero, skip ahead
	bra	PollCc4		; "
	bra	PollCc2		;Else, loop
PollCc3	incf	UQPOP,F		;Advance and loop the queue pop pointer
	bcf	UQPOP,7		; "
	movlw	B'00001010'	;Turn off the CCP output (no change) and set it
	movwf	CCP1CON		; to interrupt on match
	return			;Done
PollCc4	movlw	B'00001001'	;Set CCP to go low on match
	movwf	CCP1CON		; "
	return			;Done


;;; Mainline Interrupt ;;;

IntMainline
	bra	$


;;; End of Program ;;;
	end
