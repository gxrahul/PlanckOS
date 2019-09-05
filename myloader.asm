[BITS 16]
[ORG 0]

; setup segments
cli
mov ax, 0x07C0
mov ds, ax
mov es, ax

; create stack
mov ax, 0x07C0
mov ss, ax
mov sp, 0xFFFF
sti

; display startup message
mov si, welcome_msg
call displayMsg

; read kernel image from 2nd sector of hard disk into memory at 0x07F0:0000
; give 5 retries to read the image
mov si, read_msg
call displayMsg

readSector:
mov ax, 0x07F0
mov es, ax
mov bx, 0x0000
mov ah, 0x02
mov al, [COPY_NUM_SECTORS]
mov ch, 0x00  		; low eight bits of cylinder no
mov cl, [COPY_SECTORS_START]	; bits 0-5 sector no and bits 6-7 high two bits of cylinder no
mov dh, 0x00			; head no
mov dl, 0x80			; drive no, bit 7 set for hard disk
int 0x13
jnc jumpToKernel
xor ax, ax	; BIOS reset
int 0x13	; disk
dec di
jnz readSector

Failure:
mov si, fail_msg
call displayMsg
jmp $	; if read fails, reboot

jumpToKernel:
; mov bx, 0x0098
; mov ax, 0x0000
; mov es, ax
; mov [es:bx], dword 0x7CE00000
mov si, kernel_msg
call displayMsg
; int 26h
push WORD 0x07F0
push WORD 0x0000
retf
; jmp 0x7F00


displayMsg:
mov al, [si]	; load next char of msg
inc si
or al, al	; test for NULL byte, i.e. end of string
jz Done
mov ah, 0x0E	;
mov bh, 0x00	;
# bootloader to load the OS in memory
mov bl, 0x07
int 0x10
jmp displayMsg

Done:
ret

MAX_NUM_TRIES		DB	0x05	; max no of tries
COPY_NUM_SECTORS	DB	12	; no of sectors to copy
COPY_SECTORS_START	DB	0X02	; where to start copying

welcome_msg 	DB 	0x0D, 0x0A, "Welcome To My BootLoader", 0x0D, 0x0A, 0x00
read_msg 	DB 	0x0D, 0x0A, "Reading Kernel Image...", 0x0D, 0x0A, 0x00
fail_msg 	DB 	0x0D, 0x0A, "Read Failed :(", 0x0D, 0x0A, 0x00
kernel_msg 	DB 	0x0D, 0x0A, "Loading Kernel...", 0x0D, 0x0A, 0x00

TIMES 446-($-$$) DB 0
; DW 0xAA55
