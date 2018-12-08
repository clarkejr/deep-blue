;
; Copyright (C) 2018 Don Clarke -- see LICENSE.TXT
;
stage1 equ 0x7C00

USE16				; The assembler is to create 16 bit real mode code.

org stage1			; The assembler is to assume that the binary will be loaded to this memmory address.

Start:

; Halt if the code reaches this point as there is a problem.

Halt:
	mov si, msg_Error
	call print_string
.repeat:
	hlt
	jmp short .repeat

;------------------------------------------------------------------------------

; 16-bit function to print a character in #al to the screen
print_char:
    pusha			; save all registers onto the stack
    mov bx, 0x07	; text console page number
    mov ah, 0x0e	; teletype function
    int 0x10		; call interupt 0x10
    popa			; restore all registers onto the stack
    ret

; 16-bit function to print CR and LF to the screen
print_CR_LF:
    pusha			; save all registers onto the stack
	mov al, 0x0d	; carriage return = sends the cursor back to the start of the line
	call print_char
	mov al, 0x0a	; line feed = sends the cursor to the next line
	call print_char
    popa			; restore all registers onto the stack
	ret

; 16-bit function to print a string to the screen
; IN:	SI - Address of start of string
print_string:			; Output string in SI to screen
    pusha			; save all registers onto the stack
.repeat:
	lodsb			; Get char from string
	cmp al, 0
	je .done		; If char is zero, end of string
	call print_char
	jmp short .repeat
.done:
    popa			; restore all registers onto the stack
	ret

;------------------------------------------------------------------------------

msg_Error db "Error!", 0x0d, 0x0a, 0x00

; To zerofill up to the MBR signature at the end of the boot code.
times 510 - ($ - $$) db 0

sign dw 0xAA55
