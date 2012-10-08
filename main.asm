; +-----------------------------------------------------------------------------------------------------------------------------+
; | Gestion de 24 servos avec un PIC18F452																						|
; | Copyright (C) 2012 Charles RINCHEVAL (contact@digitalspirit.org)															|
; | Ce programme est libre, vous pouvez le redistribuer et/ou le modifier selon les termes de la Licence Publique Générale GNU 	|
; | publiée par la Free Software Foundation (version 2 ou bien toute autre version ultérieure choisie par vous).				|
; | Ce programme est distribué car potentiellement utile, mais SANS AUCUNE GARANTIE, ni explicite ni implicite,					|
; | y compris les garanties de commercialisation ou d'adaptation dans un but spécifique.										|
; | Reportez-vous à la Licence Publique Générale GNU pour plus de détails.														|
; | Vous devez avoir reçu une copie de la Licence Publique Générale GNU en même temps que ce programme ; 						|
; | si ce n'est pas le cas,																										|
; | écrivez à la Free Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307, États-Unis. 				|
; +-----------------------------------------------------------------------------------------------------------------------------+
; 2004-11-19 : Lorsque la consigne est 0, le port reste à 1
; 2005-12-03 : Version finale débugguée en test sur Bleuette

	title           "Gestion de 24 servos !"
	processor       18F452

	#include		"p18F452.inc"
	#include		"macro.inc"

; Program Configuration Registers
;	__CONFIG    _CONFIG1H, _HSPLL_OSC_1H	; _OSCS_OFF_1H & _EC_OSC_1H & 
	__CONFIG    _CONFIG2L, _BOR_OFF_2L & _PWRT_OFF_2L
	__CONFIG    _CONFIG2H, _WDT_OFF_2H
;	__CONFIG    _CONFIG3H, _CCP2MX_OFF_3H
	__CONFIG    _CONFIG4L, _STVR_OFF_4L & _LVP_OFF_4L & _DEBUG_ON_4L
	__CONFIG    _CONFIG5L, _CP0_OFF_5L & _CP1_OFF_5L & _CP2_OFF_5L & _CP3_OFF_5L 
;	__CONFIG    _CONFIG5H, _CPB_OFF_5H & _CPD_OFF_5H
	__CONFIG    _CONFIG6L, _WRT0_OFF_6L & _WRT1_OFF_6L & _WRT2_OFF_6L & _WRT3_OFF_6L 
;	__CONFIG    _CONFIG6H, _WRTC_OFF_6H & _WRTB_OFF_6H & _WRTD_OFF_6H
	__CONFIG    _CONFIG7L, _EBTR0_OFF_7L & _EBTR1_OFF_7L & _EBTR2_OFF_7L & _EBTR3_OFF_7L
;	__CONFIG    _CONFIG7H, _EBTRB_OFF_7H


; +-----------------------------------------------+
; | Configuration du mode de réception des ordres |
; +-----------------------------------------------+
; 2 modes de fonctionnement :
; * Mode SERVO_MODE_CMD_UNIT (ServoCmdParServo) : On envoie une trame contenant 3 octets :
; 	- 0xFF : Début de la trame
;	- OxXX : Le numéro du servo qui concerne la trame
;	- 0xXX : La position DU servo
; Le mode SERVO_MODE_CMD_UNIT permet de garder une compatibilité pour les autres systèmes
;
; * Mode SERVO_MODE_CMD_MASK (ServoCmdMask) : On envoie un mask de bit
;	- 0xFF : Début de la trame
;	- 0xXX : Masque de bit des servos 16 à 23
;	- 0xXX : Masque de bit des servos 8 à 15
;	- 0xXX : Masque de bit des servos 0 à 7
;	- 0xZZ : La position DES servos
; L'avantage du mode SERVO_MODE_CMD_MASK est qu'il permet de donner une position à plusieurs
; servos en même temps et permet d'être parfaitement synhcrone très utile dans le cas de pilotage
; de patte pour des robots hexapodes...
;
; * Mode SERVO_MODE_CMD_MASK_3BLOCK
;	- 0xFF : Début de la trame
;	- 0xXX : Masque de bit des servos 16 à 23
;	- 0xXX : Masque de bit des servos 8 à 15
;	- 0xXX : Masque de bit des servos 0 à 7
;	- 0xZZ : La position des servos 16 à 23
;	- 0xZZ : La position des servos 8 à 15
;	- 0xZZ : La position des servos 0 à 7
; Ce mode permet de donner des positions différentes à différents servos en même temps
;


;#DEFINE	SERVO_MODE_CMD_UNIT
;#DEFINE	SERVO_MODE_CMD_MASK
#DEFINE	SERVO_MODE_CMD_MASK_3BLOCK



; +--------------------------+
; | Définition de constantes |
; +--------------------------+

#DEFINE		NBR_SERVO	.24

#DEFINE		PHASE_1		0x00
#DEFINE		PHASE_2		0x01
#DEFINE		PHASE_3		0x02
#DEFINE		PHASE_4		0x03


	CBLOCK	0x00	; zone access ram de la banque 0
		servo_cons0 	: 1
		servo_cons1 	: 1
		servo_cons2 	: 1
		servo_cons3 	: 1
		servo_cons4 	: 1
		servo_cons5 	: 1
		servo_cons6 	: 1
		servo_cons7 	: 1
		servo_cons8 	: 1
		servo_cons9		: 1
		servo_cons10	: 1
		servo_cons11	: 1
		servo_cons12	: 1
		servo_cons13	: 1
		servo_cons14	: 1
		servo_cons15	: 1
		servo_cons16	: 1
		servo_cons17	: 1
		servo_cons18	: 1
		servo_cons19	: 1
		servo_cons20	: 1
		servo_cons21	: 1
		servo_cons22	: 1
		servo_cons23	: 1

		servo_sorted	: 24

		cmpt_pass		: 1		; Le compteur de passage
		current_cons	: 1		; La consigne courante
		current_servo	: 1 	; Offset du servo courant
		current_qnt		: 1		; Quantité de servo courante

		total_qnt		: 1		; Quantité totale de servos qui y sont passés
		cons_prec		: 1		; Valeur de la consigne précédente

		buf_PORTA		: 1		; Les buffers des ports
		buf_PORTB		: 1
		buf_PORTC		: 1
		buf_PORTD		: 1
		buf_PORTE		: 1

		phase			: 1		; La phase courante
		phase_cmpt		: 1		; Phase de comptage


		; Variables spécifiques pour le tri
		var_min 		: 1
		var_max 		: 1

		mask 			: 1
		mask_tmp 		: 1

		buf_tab0 		: 1
		buf_tab1 		: 1
		buf_tab2 		: 1

		cmpt_bcl 		: 1

		tmp_FSR0L 		: 1
;		tmp_FSR0H 		: 1

		tmp_FSR1L 		: 1
		tmp_FSR1H 		: 1

		tmp_FSR2L 		: 1
;		tmp_FSR2H 		: 1

		cmpt 			: 1
		bcl				: 1

		; Masque de bit
		mask_recept0	: 1		; Octets contenant les masques de bits
		mask_recept1	: 1
		mask_recept2	: 1

		pos_recept		: 1		; La position voulue
;#IFDEF	SERVO_MODE_CMD_MASK_3BLOCK
		pos_recept1		: 1
		pos_recept2		: 1
;#ENDIF

		cmpt_word		: 1		; Compteur d'octets

;		cmpt_maj		: 1

;#IFDEF	SERVO_MODE_CMD_UNIT
		csaving_IN_FSR2L	: 1		; Context saving
		csaving_IN_FSR2H	: 1

		csaving_OUT_FSR2L	: 1		; Context saving
		csaving_OUT_FSR2H	: 1
;#ENDIF
	ENDC

	CBLOCK 0x100	; Zone Bank 1
		servo_nbr00	: 1
		servo_nbr01	: 1
		servo_nbr02	: 1
		servo_nbr03	: 1
		servo_nbr04	: 1
		servo_nbr05	: 1
		servo_nbr06	: 1
		servo_nbr07	: 1
		servo_nbr08	: 1
		servo_nbr09	: 1
		servo_nbr10	: 1
		servo_nbr11	: 1
		servo_nbr12	: 1
		servo_nbr13	: 1
		servo_nbr14	: 1
		servo_nbr15	: 1
		servo_nbr16	: 1
		servo_nbr17	: 1
		servo_nbr18	: 1
		servo_nbr19	: 1
		servo_nbr20	: 1
		servo_nbr21	: 1
		servo_nbr22	: 1
		servo_nbr23	: 1

		servo_val00 : 1	; 24 * 3 zones contenants les états des ports à chaque itération
		servo_val01 : 1
		servo_val02 : 1
		servo_val03 : 1
		servo_val04 : 1
		servo_val05 : 1
		servo_val06 : 1
		servo_val07 : 1
		servo_val08 : 1
		servo_val09 : 1
		servo_val10 : 1
		servo_val11 : 1
		servo_val12 : 1
		servo_val13 : 1
		servo_val14 : 1
		servo_val15 : 1
		servo_val16 : 1
		servo_val17 : 1
		servo_val18 : 1
		servo_val19 : 1
		servo_val20 : 1
		servo_val21 : 1
		servo_val22 : 1
		servo_val23 : 1
		servo_val24 : 1
		servo_val25 : 1
		servo_val26 : 1
		servo_val27 : 1
		servo_val28 : 1
		servo_val29 : 1
		servo_val30 : 1
		servo_val31 : 1
		servo_val32 : 1
		servo_val33 : 1
		servo_val34 : 1
		servo_val35 : 1
		servo_val36 : 1
		servo_val37 : 1
		servo_val38 : 1
		servo_val39 : 1
		servo_val40 : 1
		servo_val41 : 1
		servo_val42 : 1
		servo_val43 : 1
		servo_val44 : 1
		servo_val45 : 1
		servo_val46 : 1
		servo_val47 : 1
		servo_val48 : 1
		servo_val49 : 1
		servo_val50 : 1
		servo_val51 : 1
		servo_val52 : 1
		servo_val53 : 1
		servo_val54 : 1
		servo_val55 : 1
		servo_val56 : 1
		servo_val57 : 1
		servo_val58 : 1
		servo_val59 : 1
		servo_val60 : 1
		servo_val61 : 1
		servo_val62 : 1
		servo_val63 : 1
		servo_val64 : 1
		servo_val65 : 1
		servo_val66 : 1
		servo_val67 : 1
		servo_val68 : 1
		servo_val69 : 1
		servo_val70 : 1
		servo_val71 : 1
	ENDC

	; Reset Vector
	org		0
	nop
	bra		_init


	; High Priority Vector
	org		0x08
	nop
	bra		inth

	; Low Priority Vector
	org		0x18
	nop
	bra		intl



; High Priority Vector
inth

	; "Switch" des interruptions
	btfsc   INTCON, TMR0IF
	goto	_inth_Timer

	bcf		T1CON, TMR1ON
	bsf		T0CON, TMR0ON
	btfsc   PIR1, TMR1IF
	goto	_inth_Timer

	retfie	FAST


; Low Priority Vector
intl
	btfsc   PIR1, RCIF
	goto	_intl_usart

	retfie


; Interruption venant de l'USART
_intl_usart

	btfsc	cmpt_word, 0
	bra		recept_phase0
	btfsc	cmpt_word, 1
	bra		recept_phase1
	btfsc	cmpt_word, 2
	bra		recept_phase2

#IFDEF	SERVO_MODE_CMD_MASK
	btfsc	cmpt_word, 3
	bra		recept_phase3
	btfsc	cmpt_word, 4
	bra		recept_phase4
#ENDIF

#IFDEF	SERVO_MODE_CMD_MASK_3BLOCK
	btfsc	cmpt_word, 3
	bra		recept_phase3
	btfsc	cmpt_word, 4
	bra		recept_phase4
	btfsc	cmpt_word, 5
	bra		recept_phase5
	btfsc	cmpt_word, 6
	bra		recept_phase6
#ENDIF

; Erreur !
pouf	bra		pouf


recept_phase0
	; Le premier octet reçu doit être un 0xFF
	movf	RCREG, W
	sublw	0xFF
	btfss	STATUS, Z	; Si le caractère lu est 0xFF, on continu
	bra		_intl_usart_end2

	bra		_intl_usart_end

	; Ici, on vient charger les octets reçus un par un
	; dans les buffer correspondant mask_receptX
recept_phase1
	movf	RCREG, W
	movwf	mask_recept0
	bra		_intl_usart_end

recept_phase2
	movf	RCREG, W

#IFDEF	SERVO_MODE_CMD_UNIT 
	movwf	pos_recept
	bra		_intl_usart_end
#ENDIF


#IFDEF	SERVO_MODE_CMD_MASK
	movwf	mask_recept1
	bra		_intl_usart_end

recept_phase3
	movf	RCREG, W
	movwf	mask_recept2
	bra		_intl_usart_end

recept_phase4
	movf	RCREG, W
	movwf	pos_recept
#ENDIF


#IFDEF	SERVO_MODE_CMD_MASK_3BLOCK
	movwf	mask_recept1
	bra		_intl_usart_end

recept_phase3
	movf	RCREG, W
	movwf	mask_recept2
	bra		_intl_usart_end

recept_phase4
	movf	RCREG, W
	movwf	pos_recept
	bra		_intl_usart_end

recept_phase5
	movf	RCREG, W
	movwf	pos_recept1
	bra		_intl_usart_end

recept_phase6
	movf	RCREG, W
	movwf	pos_recept2
#ENDIF


_intl_usart_end
	rlncf	cmpt_word

_intl_usart_end2
	bcf		PIR1, RCIF
	retfie




; Interruption du Timer
_inth_Timer

#IFDEF	SERVO_MODE_CMD_UNIT
	movff	FSR2L, csaving_IN_FSR2L
	movff	FSR2H, csaving_IN_FSR2H

	movff	csaving_OUT_FSR2L, FSR2L
	movff	csaving_OUT_FSR2H, FSR2H
#ENDIF

	; Aiguillage vers la bonne phase
	btfsc	phase_cmpt, .1
	goto	phase3


; ###########
; # PHASE 1 #
; ###########

	call	sort

	movlw	0x48				; Ajout hugo, le 10/02/2005
	movwf	TMR1H				; Pour être sûr que la période soit de 20ms
	movlw	0xA0
	movwf	TMR1L
	bsf		T1CON, TMR1ON		; Pour revenir dans 20ms

	call	setAllPort			; Allumage des ports !


	; Tempo de 400us, oui, c'est dommage ...
	; ... on pourrait se servir de ce temps pour autre chose ...
	; ... il faut donc refaire la routine de tri !
	movlw	0xFF
	movwf	cmpt_bcl
bcl_toto
	bra		$ + 2
	bra		$ + 2
	bra		$ + 2
	bra		$ + 2
	bra		$ + 2
	bra		$ + 2
	decfsz	cmpt_bcl
	bra		bcl_toto

	fill(bra	$ + 2), 64	; 64 -> Passage en repos à l'état bas


; ###########
; # PHASE 2 #
; ###########
	
	lfsr	FSR0, servo_sorted	; Pointeur de consignes
	lfsr	FSR1, servo_val00	; Pointeur des ports actifs
	lfsr	FSR2, servo_nbr00	; Pointeur de la quantité courante

	clrf	total_qnt
	clrf	cons_prec

phase2
	movf	INDF0, W
	movwf	current_cons		; La consigne courante

	movf	cons_prec, W
	subwf	current_cons, F		; On ne compte que le décalage par rapport au précédent

	bz		phase4				; Consigne courante à 0 ?

	movf	INDF0, W
	movwf	cons_prec

	movlw	.1
	movwf	cmpt_pass


	setf	phase_cmpt			; Phase de comptage


	nop							; Très important pour la précision !
	nop							; Ne pas toucher !!

	nop		; Ajout pour passage en mode repos niveau haut
	nop		; "
	nop		; "

#IFDEF	SERVO_MODE_CMD_MASK
	nop
	nop
	nop
	nop
#ENDIF

#IFDEF	SERVO_MODE_CMD_MASK_3BLOCK
	nop
	nop
	nop
	nop
#ENDIF


; ###########
; # PHASE 3 #
; ###########
phase3
	movlw	0xFF
	movwf	TMR0H

#IFDEF	SERVO_MODE_CMD_UNIT
	nop
	nop
	nop
	nop
	movlw	0xE7
#ELSE	; SERVO_MODE_CMD_MASK	SERVO_MODE_CMD_MASK_3BLOCK
	movlw	0xE1
#ENDIF


	movwf	TMR0L

	nop

	incf	cmpt_pass			; Incrémente le compteur

	movf	current_cons, W
	cpfsgt	cmpt_pass			; Si cmpt est supérieur à la consigne courante, on skip
	bra		inth_end

	clrf	phase_cmpt			; Sinon, on a fini de compter


; ###########
; # PHASE 4 #
; ###########
	nop
phase4
	call	clrPort				; Extinction du ou des servos courants

	movf	POSTINC2, W			; Addition de la quantité courante ...
	addwf	total_qnt, F		; ... et de la quantité de servos déjà passés

	incf	FSR0L
	nop

	movlw	NBR_SERVO - 1
	cpfsgt	total_qnt			; Si le total de servo déjà passé est égal au nombre de servo, on skip !
	bra		phase2


; ###########
; # PHASE 5 #
; ###########

;	movlw	0x61				; Supprimé hugo, le 10/02/2005
;	movwf	TMR1H				; Pour être sûr que la période soit de 20ms
;	movlw	0xFF				; ... 0xFF pour 20ms pile poil !
;	movwf	TMR1L

	bcf		T0CON, TMR0ON
;	bsf		T1CON, TMR1ON		; Supprimé hugo, le 10/02/2005

; On sort de l'interruption
inth_end
	bcf		PIR1, TMR1IF
	bcf		INTCON, TMR0IF

#IFDEF	SERVO_MODE_CMD_UNIT
	movff	FSR2L, csaving_OUT_FSR2L
	movff	FSR2H, csaving_OUT_FSR2H

	movff	csaving_IN_FSR2H, FSR2H
	movff	csaving_IN_FSR2L, FSR2L
#ENDIF

	retfie	FAST






	#include "init.inc"
	#include "init2.inc"


	SEND	'\n'
	SEND	'O'
	SEND	'O'
	SEND	'K'
	SEND	'K'
	SEND	'\n'

	movlw	0
	call	setValue

; Routine principale
	bsf		INTCON, GIEH	; On active les interruptions
	bsf		INTCON, GIEL	; On active les interruptions

main


#IFDEF	SERVO_MODE_CMD_UNIT

	btfss	cmpt_word, 3	; Est-ce que 3 octets ont été reçus ?
	bra		main			; non, on boucle !

	movlw	.1				; oui
	movwf	cmpt_word

	lfsr	FSR2, servo_cons0

	movf	mask_recept0, W
	addwf	FSR2L

	movf	pos_recept, W
	movwf	INDF2

#ENDIF


#IFDEF	SERVO_MODE_CMD_MASK
	
	btfss	cmpt_word, 5	; Est-ce que 5 octets ont été reçus ?
	bra		main			; non, on boucle !

	movlw	.1				; oui
	movwf	cmpt_word


	movlw	0xFF			; Si la consigne demandée est égale ...
	cpfseq	pos_recept		; ... à 0xFF ...
	bra		_main_next

	decf	pos_recept, f	; ... on la décrémente car 0xFF est interdit !

_main_next
	movf	pos_recept, W


	; Théoriquement, on va plus vite à faire comme ça ...
	; ... que de faire une boucle qui parcourt les bits !

	btfsc	mask_recept0, 0	; Servo 16 à 23
	movwf	servo_cons16
	btfsc	mask_recept0, 1
	movwf	servo_cons17
	btfsc	mask_recept0, 2
	movwf	servo_cons18
	btfsc	mask_recept0, 3
	movwf	servo_cons19
	btfsc	mask_recept0, 4
	movwf	servo_cons20
	btfsc	mask_recept0, 5
	movwf	servo_cons21
	btfsc	mask_recept0, 6
	movwf	servo_cons22
	btfsc	mask_recept0, 7
	movwf	servo_cons23

	btfsc	mask_recept1, 0	; Servo 8 à 15
	movwf	servo_cons8
	btfsc	mask_recept1, 1
	movwf	servo_cons9
	btfsc	mask_recept1, 2
	movwf	servo_cons10
	btfsc	mask_recept1, 3
	movwf	servo_cons11
	btfsc	mask_recept1, 4
	movwf	servo_cons12
	btfsc	mask_recept1, 5
	movwf	servo_cons13
	btfsc	mask_recept1, 6
	movwf	servo_cons14
	btfsc	mask_recept1, 7
	movwf	servo_cons15

	btfsc	mask_recept2, 0	; Servo 0 à 7
	movwf	servo_cons0
	btfsc	mask_recept2, 1
	movwf	servo_cons1
	btfsc	mask_recept2, 2
	movwf	servo_cons2
	btfsc	mask_recept2, 3
	movwf	servo_cons3
	btfsc	mask_recept2, 4
	movwf	servo_cons4
	btfsc	mask_recept2, 5
	movwf	servo_cons5
	btfsc	mask_recept2, 6
	movwf	servo_cons6
	btfsc	mask_recept2, 7
	movwf	servo_cons7

#ENDIF


#IFDEF	SERVO_MODE_CMD_MASK_3BLOCK
	
	btfss	cmpt_word, 7	; Est-ce que 7 octets ont été reçus ?
	bra		main			; non, on boucle !

	movlw	.1				; oui
	movwf	cmpt_word


	movlw	0xFF			; Si la consigne demandée est égale ...
	cpfseq	pos_recept		; ... à 0xFF ...
	bra		$ + 4
	decf	pos_recept, f	; ... on la décrémente car 0xFF est interdit !

	movlw	0xFF			; Si la consigne demandée est égale ...
	cpfseq	pos_recept1		; ... à 0xFF ...
	bra		$ + 4
	decf	pos_recept1, f	; ... on la décrémente car 0xFF est interdit !

	movlw	0xFF			; Si la consigne demandée est égale ...
	cpfseq	pos_recept2		; ... à 0xFF ...
	bra		$ + 4
	decf	pos_recept2, f	; ... on la décrémente car 0xFF est interdit !

_main_next

	; Théoriquement, on va plus vite à faire comme ça ...
	; ... que de faire une boucle qui parcourt les bits !
	movf	pos_recept, W

	btfsc	mask_recept0, 0	; Servo 16 à 23
	movwf	servo_cons16
	btfsc	mask_recept0, 1
	movwf	servo_cons17
	btfsc	mask_recept0, 2
	movwf	servo_cons18
	btfsc	mask_recept0, 3
	movwf	servo_cons19
	btfsc	mask_recept0, 4
	movwf	servo_cons20
	btfsc	mask_recept0, 5
	movwf	servo_cons21
	btfsc	mask_recept0, 6
	movwf	servo_cons22
	btfsc	mask_recept0, 7
	movwf	servo_cons23


	movf	pos_recept1, W

	btfsc	mask_recept1, 0	; Servo 8 à 15
	movwf	servo_cons8
	btfsc	mask_recept1, 1
	movwf	servo_cons9
	btfsc	mask_recept1, 2
	movwf	servo_cons10
	btfsc	mask_recept1, 3
	movwf	servo_cons11
	btfsc	mask_recept1, 4
	movwf	servo_cons12
	btfsc	mask_recept1, 5
	movwf	servo_cons13
	btfsc	mask_recept1, 6
	movwf	servo_cons14
	btfsc	mask_recept1, 7
	movwf	servo_cons15


	movf	pos_recept2, W

	btfsc	mask_recept2, 0	; Servo 0 à 7
	movwf	servo_cons0
	btfsc	mask_recept2, 1
	movwf	servo_cons1
	btfsc	mask_recept2, 2
	movwf	servo_cons2
	btfsc	mask_recept2, 3
	movwf	servo_cons3
	btfsc	mask_recept2, 4
	movwf	servo_cons4
	btfsc	mask_recept2, 5
	movwf	servo_cons5
	btfsc	mask_recept2, 6
	movwf	servo_cons6
	btfsc	mask_recept2, 7
	movwf	servo_cons7

#ENDIF


	bra		main



; Met toutes les consignes à la valeur voulues (dans WREG)
setValue

	movwf	servo_cons0
	movwf	servo_cons1
	movwf	servo_cons2
	movwf	servo_cons3
	movwf	servo_cons4
	movwf	servo_cons5
	movwf	servo_cons6
	movwf	servo_cons7

	movwf	servo_cons8
	movwf	servo_cons9
	movwf	servo_cons10
	movwf	servo_cons11
	movwf	servo_cons12
	movwf	servo_cons13
	movwf	servo_cons14
	movwf	servo_cons15

	movwf	servo_cons16
	movwf	servo_cons17
	movwf	servo_cons18
	movwf	servo_cons19
	movwf	servo_cons20
	movwf	servo_cons21
	movwf	servo_cons22
	movwf	servo_cons23

	return



; Tous les ports des servos à 1 ...
; ... sauf ceux qui ont une consigne à 0
setAllPort
;	movlw	b'00011111'
;	iorwf	LATA
;	movlw	b'00000111'
;	iorwf	LATC
;	movlw	b'00111111'
;	iorwf	LATB
;	movlw	b'00111111'
;	iorwf	LATE
;	setf	LATD
;	return

; On reprend le principe suivant :
; RA0, RA1, RA2, RA3, RA4, RC0, RC1, RC2	-> Servo 0 à 7
; RB0, RB1, RB2, RB3, RB4, RB5, RE0, RE1	-> Servo 8 à 15 
; RD0, RD1, RD2, RD3, RD4, RD5, RD6, RD7	-> Servo 16 à 23
; Inutile de faire une boucle pour gérer le code ci-dessous !

	clrf	buf_PORTA
	clrf	buf_PORTC
	clrf	buf_PORTB
	clrf	buf_PORTE
	clrf	buf_PORTD



;	movf	servo_cons0, W
;	bz		toto
;	movlw	b'00000001'
;	bra		$ + 6
;toto
;	andwf	b'00000001'
;	iorwf	buf_PORTA

	

	clrf	WREG
	tstfsz	servo_cons0
	movlw	b'00000001'
	iorwf	buf_PORTA

	clrf	WREG
	tstfsz	servo_cons1
	movlw	b'00000010'
	iorwf	buf_PORTA

	clrf	WREG
	tstfsz	servo_cons2
	movlw	b'00000100'
	iorwf	buf_PORTA

	clrf	WREG
	tstfsz	servo_cons3
	movlw	b'00001000'
	iorwf	buf_PORTA

	clrf	WREG
	tstfsz	servo_cons4
	movlw	b'00010000'
	iorwf	buf_PORTA

	clrf	WREG
	tstfsz	servo_cons5
	movlw	b'00000001'
	iorwf	buf_PORTC

	clrf	WREG
	tstfsz	servo_cons6
	movlw	b'00000010'
	iorwf	buf_PORTC

	clrf	WREG
	tstfsz	servo_cons7
	movlw	b'00000100'
	iorwf	buf_PORTC



	clrf	WREG
	tstfsz	servo_cons8
	movlw	b'00000001'
	iorwf	buf_PORTB

	clrf	WREG
	tstfsz	servo_cons9
	movlw	b'00000010'
	iorwf	buf_PORTB

	clrf	WREG
	tstfsz	servo_cons10
	movlw	b'00000100'
	iorwf	buf_PORTB

	clrf	WREG
	tstfsz	servo_cons11
	movlw	b'00001000'
	iorwf	buf_PORTB

	clrf	WREG
	tstfsz	servo_cons12
	movlw	b'00010000'
	iorwf	buf_PORTB

	clrf	WREG
	tstfsz	servo_cons13
	movlw	b'00100000'
	iorwf	buf_PORTB

	clrf	WREG
	tstfsz	servo_cons14
	movlw	b'00000001'
	iorwf	buf_PORTE

	clrf	WREG
	tstfsz	servo_cons15
	movlw	b'00000010'
	iorwf	buf_PORTE



	clrf	WREG
	tstfsz	servo_cons16
	movlw	b'00000001'
	iorwf	buf_PORTD

	clrf	WREG
	tstfsz	servo_cons17
	movlw	b'00000010'
	iorwf	buf_PORTD

	clrf	WREG
	tstfsz	servo_cons18
	movlw	b'00000100'
	iorwf	buf_PORTD

	clrf	WREG
	tstfsz	servo_cons19
	movlw	b'00001000'
	iorwf	buf_PORTD

	clrf	WREG
	tstfsz	servo_cons20
	movlw	b'00010000'
	iorwf	buf_PORTD

	clrf	WREG
	tstfsz	servo_cons21
	movlw	b'00100000'
	iorwf	buf_PORTD

	clrf	WREG
	tstfsz	servo_cons22
	movlw	b'01000000'
	iorwf	buf_PORTD

	clrf	WREG
	tstfsz	servo_cons23
	movlw	b'10000000'
	iorwf	buf_PORTD


	movf	buf_PORTA, W
	movwf	LATA
	movf	buf_PORTC, W
	movwf	LATC
	movf	buf_PORTB, W
	movwf	LATB
	movf	buf_PORTE, W
	movwf	LATE
	movf	buf_PORTD, W
	movwf	LATD

	return




; Eteint le port ou les ports spécifiés par le contenu de 3 octets,
; l'adresse du premier des 3 octets étant spécifié dans W
; Les ports utilisés sont A, B, C, D, E, le port C servant
; à recevoir les ordres pour piloter les servos
; x [0;7]	PORTA [0-4], PORTC [0-2]
; x [8;15]	PORTB [0-5], PORTE [1-2]
; x [16;23]	PORTD [0-7]
; RA0, RA1, RA2, RA3, RA4, RC0, RC1, RC2	-> Servo 0 à 7
; RB0, RB1, RB2, RB3, RB4, RB5, RE0, RE1	-> Servo 8 à 15 
; RD0, RD1, RD2, RD3, RD4, RD5, RD6, RD7	-> Servo 16 à 23
clrPort

	; Si la consigne courante est 0
	; Le ou les servo(s) restent comme il sont, à 1
	movf	current_cons, W
	bz		_clrPort

	; Le premier octet concerne
	; les port A et C
	movf	INDF1, W
	andlw	b'000111111'
	xorwf	LATA, W		; Pour A
	movwf	buf_PORTA

	movf	INDF1, W
	rlncf	WREG
	rlncf	WREG
	rlncf	WREG
	andlw	b'00000111'
	xorwf	LATC, W		; Pour C
	movwf	buf_PORTC

	; Le deuxième octet concerne
	; les port B et E
	movf	PREINC1, W
	andlw	b'00111111'
	xorwf	LATB, W		; Pour B
	movwf	buf_PORTB

	movf	POSTINC1, W
	rlncf	WREG
	rlncf	WREG
	andlw	b'0000011'
	xorwf	LATE, W		; Pour E
	movwf	buf_PORTE

	; Le dernier octet concerne le port D
	movf	POSTINC1, W
	xorwf	LATD, W		; Pour D
	movwf	buf_PORTD

	; Maintenant, on place tout le contenu des buffers ...
	; ... dans les ports qui vont bien, ce qui garanti une synchro idéale
	movf	buf_PORTA, W
	movwf	LATA
	movf	buf_PORTC, W
	movwf	LATC
	movf	buf_PORTB, W
	movwf	LATB
	movf	buf_PORTE, W
	movwf	LATE
	movf	buf_PORTD, W
	movwf	LATD

	return

; Rattrapage du temps en cas de consigne nul
; Ne surtout pas modifier
_clrPort

	incf	FSR1L
	incf	FSR1L
	incf	FSR1L
	nop
	nop
	nop
	nop
	nop
	nop
	nop

	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop

	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop

	nop
	nop
	nop

	return



; Tri !
; TODO: refaire tout le code de tri ci dessous !
; il doit y avoir moyen d'économiser beaucoup de temps CPU...
sort

	lfsr	FSR0, servo_sorted	; Pointeur des servos triés
	lfsr	FSR1, servo_val00	; Pointeur des ports actifs
	lfsr	FSR2, servo_nbr00	; Pointeur de la quantité courante

	movff	FSR0L, tmp_FSR0L
	movff	FSR1H, tmp_FSR1H
	movff	FSR1L, tmp_FSR1L
	movff	FSR2L, tmp_FSR2L

	clrf	var_max

	movlw	NBR_SERVO
	movwf	cmpt_bcl

sort_

	; Initialisation des variables, pointeurs...
	lfsr	FSR0, servo_cons0	; Pointeur de consignes
	lfsr	FSR1, buf_tab0		; Buffer de données

	setf	var_min

	clrf	buf_tab0
	clrf	buf_tab1
	clrf	buf_tab2

	movlw	.1
	movwf	cmpt
	movwf	mask_tmp

	clrf	bcl

; C'est parti !
_sort
	incf	bcl
	movlw	NBR_SERVO + 1
	cpfslt	bcl					; Si toutes les consignes y sont passées...
	bra		_sort_quit			; ... on quitte !

	movlw	NBR_SERVO
	subwf	cmpt_bcl, W
	bz		_sort_last

	movf	INDF0, W			; Met la consigne courante dans W ...
	cpfslt	var_max				; ... et test si elle est supérieur à ...
	bra		_sort_paf			; ... var_max, on poursuit, sinon, on retourne au début !

_sort_last
	movf	INDF0, W			; Met la consigne courante dans W ...
	cpfseq	var_min				; Si la consigne courante est égale à var_min ...
	bra		_sort_next

	movf	mask_tmp, W			; ... on va là !
	iorwf	INDF1

	incf	cmpt
	bra		_sort_paf

_sort_next
	cpfslt	var_min				; Si la consigne courante est inférieur à var_min ...
	bra		_sort_inf

	bra		_sort_paf

_sort_inf
	movf	INDF0, W			; ... on file ici !
	movwf	var_min

	clrf	buf_tab0
	clrf	buf_tab1
	clrf	buf_tab2
	movlw	.1
	movwf	cmpt

	movf	mask_tmp, W
	iorwf	INDF1

_sort_paf
	bcf		STATUS, C
	rlcf	mask_tmp
	btfss	STATUS, C
	bra		_sort_pif

	rlcf	mask_tmp
	incf	FSR1L

_sort_pif
	incf	FSR0L
	bra		_sort


_sort_quit

	movf	var_min, W
	movwf	var_max

	movff	tmp_FSR0L, FSR0L	; servo_sorted	->	Pointeur de consignes
	movff	tmp_FSR1H, FSR1H	; servo_val00	->	Pointeur des ports actifs
	movff	tmp_FSR1L, FSR1L
	movff	tmp_FSR2L, FSR2L	; servo_nbr00	->	Pointeur de la quantité courante

	movff	var_min, POSTINC0

	movff	buf_tab0, POSTINC1
	movff	buf_tab1, POSTINC1
	movff	buf_tab2, POSTINC1

	movff	cmpt, POSTINC2

	movff	FSR0L, tmp_FSR0L	; Sauvegarde des pointeurs
	movff	FSR1H, tmp_FSR1H
	movff	FSR1L, tmp_FSR1L
	movff	FSR2L, tmp_FSR2L

	decfsz	cmpt_bcl
	bra		sort_

	return



	END
