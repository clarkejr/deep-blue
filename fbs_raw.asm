;
; Copyright (C) 2018 Don Clarke -- see LICENSE.TXT
;
stage1 equ 0x7C00
stage2 equ stage1 + 0x0200
e820map equ 0x1000

USE16				; The assembler is to create 16 bit real mode code.

org stage1			; The assembler is to assume that the binary will be loaded to this memmory address.

Start:
	cli				; Disable interrupts.
	cld				; Clear direction flag.
	xor ax, ax		; Zero the Accumulator Register.
	mov ss, ax 		; Zero the SS Stack Segment.
	mov ds, ax		; Zero the DS Data Segment.
	mov es, ax		; Zero the ES Extra Segment.
	mov sp, stage1	; Set the Stack Pointer to the base of this boot code.
	sti				; Enable interrupts.

	mov [DriveNumber], dl		; BIOS passes drive number in DL

PrintOSGreeting:
	mov si, msg_DeepBlue
	call print_string
	call print_CR_LF

TestLowMemory:
	; Detect available conventional memory
	clc	; Clear carry flag
    int 0x12	; call BIOS (request low memory size)
    ; The carry flag is set if it failed
    jc Halt
    ; AX = amount of continuous memory in KB starting from 0.
	mov [LowMemory], ax	; store the size of conventional memory

	; TODO: test to see if there is sufficient memory!
	mov bx, 0x200	; 512 kb in decimal
	cmp ax, bx		; compare (ax - bx)
	jbe Halt		; jump if unsigned number is below or equal
	; Only print memory if there is sufficient memory.
	mov si, msg_LowMemory
	call print_string
	mov bx, [LowMemory]
	call print_hex_dw
	call print_CR_LF

; Load stage 2 boot sectors from the drive.
ResetFloppy:
	clc							; clear carry flag
	mov		ah, 0x00			; reset floppy disk function
	mov		dl, [DriveNumber]	; drive 0 is floppy drive
	int		0x13				; call BIOS
	jc		ResetFloppy			; if Carry Flag (CF) is set, there was an error. Try resetting again

; Print loading stage 2 message.
	mov si, msg_LoadStage2
	call print_string

; MBR is the first sector on a disk -- cylinder 0, head 0, sector 1

LoadStage2:
	; Setup ES:BX with the address of the memory buffer
	xor		bx, bx					; BX = zero
	mov		es, bx					; ES = zero
	mov		bx, stage2				; we are going to read sector two into address 0x0:stage2

	mov		dl, [DriveNumber]		; Load drive number into DL
	mov		dh, 0x00				; Load head number into DH
	mov		ch, 0x00				; Load cylinder number into CH
	mov		cl, 0x02				; Load sector number into CL
	mov		al, 0x02				; Load number of sectors to be read into AL

	mov		ah, 0x02				; function 2
	int		0x13					; call BIOS - Read the sector
	jc		LoadStage2				; Error, so try again

; Print running stage 2 message.
	mov si, msg_RunStage2
	call print_string

; jump to execute the sector!
	jmp ContinueStage2

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

; 16-bit function to print a hex byte to the screen
; IN: BH = unsigned byte to print
print_hex_db:
	pusha
	mov cx, 2		; this will need to loop twice to handle the high and low nibbles
.lp:
	mov al, bh		; copy input byte to al for processing
	shr al, 4		; shift high nibble to low nibble for processing
	cmp al, 0xA
	jb .below_0xA	; handle hex numbers greater than 0-9 differently
	add al, 'A' - 0xA - '0' ; the nibble is greater than 9
.below_0xA:
	add al, '0'		; convert nibble to an ASCII character
	call print_char
	shl bh, 4		; get next nibble to the right (by shifting left)
	loop .lp
	popa
	ret

; 16-bit function to print a hex word to the screen
; IN: BX = integer word to print
print_hex_dw:
    pusha				; save all registers onto the stack
    mov cx, 2			; prepare to loop for 2 bytes (8) in a word (16)
.lp:
    mov ax, bx			; work with a copy of bx
    shr ax, 8			; get the low byte
    call print_hex_db
    shl bx, 8			; get the high byte
    loop .lp
    popa				; restore all registers onto the stack
    ret

; 16-bit function to print a hex double word to the screen
; IN: EBX = integer word to print
print_hex_dd:
    pusha				; save all registers onto the stack
    call print_hex_dw
    shr ebx, 16			; get the high word
    call print_hex_dw
    popa				; restore all registers onto the stack
    ret

;------------------------------------------------------------------------------

DriveNumber db 0x0
LowMemory dw 0x0000
SMAP_Count db 0x0
msg_DeepBlue db "Deep Blue OS", 0x00
msg_LowMemory db "Mem: ", 0x00
msg_LoadStage2 db "Loading stage 2...", 0x0d, 0x0a, 0x00
msg_RunStage2  db "Running stage 2...", 0x0d, 0x0a, 0x00
msg_Error db "Error!", 0x0d, 0x0a, 0x00

; To zerofill up to the MBR signature at the end of the boot code.
times 510 - ($ - $$) db 0

sign dw 0xAA55

;------------------------------------------------------------------------------
; Start of stage 2
;------------------------------------------------------------------------------

ContinueStage2:

GetMemoryMap:
	; TODO: build E820 memory map
	mov di, e820map			; ES:DI should point to 0x0:0x1000
	call do_e820
	jmp Halt

; 16-bit function to Query System Address Map
do_e820:
	xor ebx, ebx			; ebx must be 0 to start
	xor bp, bp				; keep an entry count in bp
	mov edx, 0x0534D4150	; Place "SMAP" into edx
	mov eax, 0xe820
	mov [es:di + 20], dword 1	; force a valid ACPI 3.X entry
	mov ecx, 24				; ask for 24 bytes
	int 0x15
	jc short .failed		; carry set on first call means "unsupported function"
	mov edx, 0x0534D4150	; Some BIOSes apparently trash this register?
	cmp eax, edx			; on success, eax must have been reset to "SMAP"
	jne short .failed
	test ebx, ebx			; ebx = 0 implies list is only 1 entry long (worthless)
	je short .failed
	jmp short .jmpin
.e820lp:
	mov eax, 0xe820		; eax, ecx get trashed on every int 0x15 call
	mov [es:di + 20], dword 1	; force a valid ACPI 3.X entry
	mov ecx, 24		; ask for 24 bytes again
	int 0x15
	jc short .e820f		; carry set means "end of list already reached"
	mov edx, 0x0534D4150	; repair potentially trashed register
.jmpin:
	jcxz .skipent		; skip any 0 length entries
	cmp cl, 20		; got a 24 byte ACPI 3.X response?
	jbe short .notext
	test byte [es:di + 20], 1	; if so: is the "ignore this data" bit clear?
	je short .skipent
.notext:
	mov ecx, [es:di + 8]	; get lower uint32_t of memory region length
	or ecx, [es:di + 12]	; "or" it with upper uint32_t to test for zero
	jz .skipent		; if length uint64_t is 0, skip entry
	call print_SMAP_entry
	inc bp			; got a good entry: ++count, move to next storage spot
	add di, 24
.skipent:
	test ebx, ebx		; if ebx resets to 0, list is complete
	jne short .e820lp
.e820f:
	mov [SMAP_Count], bp	; store the entry count
	clc			; there is "jc" on end of list to this point, so the carry must be cleared
	ret
.failed:
	stc			; "function unsupported" error exit
	ret

; 16-bit function to print the current SMAP entry pointed to by [es:di]
print_SMAP_entry:
	pusha
	mov ebx, [es:di]
	call print_hex_dd
	mov ebx, [es:di + 4]
	call print_hex_dd
	mov al, ':'
	call print_char
	mov ebx, [es:di + 8]
	call print_hex_dd
	mov ebx, [es:di + 12]
	call print_hex_dd
	mov al, '|'
	call print_char
	mov ebx, [es:di + 16]
	call print_hex_dd
	mov al, '|'
	call print_char
	mov ebx, [es:di + 20]
	call print_hex_dd
	call print_CR_LF
	popa
	ret

; To zerofill up to the size of a 3.5" HD floppy disk
; 512 bytes per sector, 18 sectors per track, 80 tracks per side and two sides.
; 512 * 18 * 80 * 2 = 1,474,560 bytes
times 1474560 - ($ - $$) db 0
