[BITS 16]
[ORG 0]

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; To provide an interface to the user, provides some built-in commands
;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

%macro Function_Prologue 0
  pusha
	push bp
	mov bp, sp
%endmacro

%macro Function_Epilogue 0
	mov sp, bp
	pop bp
	popa
%endmacro

mov word[__PROC_LOAD_SEGMENT_NUM__], ds
mov ax, ds
mov ss, ax
mov sp, 0x0fff
call userPrompt


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; userPrompt: give a command prompt to the user
;;
userPrompt:
; 	call setPageColor
; 	setVideoMode:
; 		mov ah, 0x00
; 		mov al, 0x03
; 		int 0x10

	showPrompt:
		call setCursor
		mov si, PROMPT_STRING
		xor cx, cx
		mov cl, byte[PROMPT_STRING_SIZE]
		call printString

		; save current prompt row no
		mov ah, [CURR_CURSOR_ROW]
		mov [CURR_PROMPT_ROW], ah

	;; wait for keypress in a loop
	waitForKeyPress:
		xor ax, ax
		mov ah, 0x10
		int 0x16
		mov byte[scancode], ah
		mov byte[asciicode], al
		; asciicode for the pressed key is returned in AL
		; and scancode is returned in AH.
		; If some special key was pressed call its handler

		case1:
			cmp ah, byte[RET_KEY]
			jz execFileOrCommand
			
		case2:
			cmp ah, byte[BACKSPACE_KEY]
			jz eraseChar

		case3:
			cmp ah, byte[UP_KEY]
			jz ignoreKey

		case4:
			cmp ah, byte[DOWN_KEY]
			jz ignoreKey

		case5:
			cmp ah, byte[LEFT_KEY]
			jz ignoreKey

		case6:
			cmp ah, byte[RIGHT_KEY]
			jz ignoreKey

		case7:
			cmp ah, byte[DEL_KEY]
			jz ignoreKey

		case8:
			cmp byte[asciicode], 32
			jl ignoreKey
			cmp byte[asciicode], 126
			jg ignoreKey

		defaultcase:
			mov di, word[CMDSTR_COUNT]
			cmp di, word[MAX_CMDSTR_SIZE]
			jz ignoreKey
			mov si, word asciicode
			xor cx, cx
			mov cl, 1
			call printString
			mov bx, word CMDSTR
			mov al, byte[si]
			mov byte[ds:bx+di], al
			inc word[CMDSTR_COUNT]

	jmp waitForKeyPress
	ret

	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	;; Function Data
	;;
	loop_counter		DB	0x00
	scancode		DB	0x00
	asciicode		DB	0x00
	sp_key_data_size	DB	0x00
	sp_key_string_size	DB	0x00
	;;
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;;
setCursor:
	pusha
	xor ax, ax
	mov ah, 0x01
	mov ch, 00000110b
	mov cl, 00000001b
	int 0x10
	popa
	ret
; ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;




;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;eraseChar: erase last char and move cursor 1 char back
;;
eraseChar:
	pusha
	mov ax, word[__PROC_LOAD_SEGMENT_NUM__]
	mov es, ax
	
	;; if we are trying to erase prompt, ignore keypress
	mov al, byte[CURR_CURSOR_ROW]
	cmp al, byte[CURR_PROMPT_ROW]
	jnz movCursorLeft
	mov al, byte[CURR_CURSOR_COL]
	cmp al, byte[PROMPT_STRING_SIZE]
	jle finish
	movCursorLeft:
		mov ax, ds
		mov es, ax
		push 0x0000		; null byte
		mov bp, sp
		mov cx, 0x01
		mov ah, 0x13
		mov al, 0x00
		mov bl, 01000000b
		mov bh, 0x00
		mov dh, [CURR_CURSOR_ROW]
		mov dl, [CURR_CURSOR_COL]
		cmp dl, 0x00
		jz col_n_row
		dec dl
		int 0x10
		jmp saveCurrCursorPosn

		col_n_row:
			cmp dh, 0x00
			jz ignoreEraseReq
			mov dl, 79
			dec dh
			int 0x10
			jmp saveCurrCursorPosn
			
			ignoreEraseReq:
				jmp finish

	saveCurrCursorPosn:		; get current cursor position and save it
		;; but first move the cursor 1 left
		mov ah, 0x02
		mov bh, 0x00
		int 0x10

		mov ah, 0x03
		mov bh, 0x00
		int 0x10
		mov [CURR_CURSOR_ROW], dh
		mov [CURR_CURSOR_COL], dl
		
finish:
	cmp word[CMDSTR_COUNT], 0
	jz exit_func
	dec word[CMDSTR_COUNT]

exit_func:
	add sp, 2
	popa
	jmp waitForKeyPress

	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	;; local function data
	;;
		null_byte	DB	0x00
		nextOperation	DW	0x0000
	;;
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;




;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; ignoreKey:
;;
ignoreKey:
	jmp waitForKeyPress
;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;




;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; execFileOrCommand:
;;
execFileOrCommand:
	pusha
	;print newline
	push 0x0A0D
	mov si, sp
	mov cx, 2
	call printString
	add sp, 2

	;; search a file or command matching the CMDSTRING
	COMP_STRINGS:
		mov ax, word[ed_cmd_size]
		cmp ax, word[CMDSTR_COUNT]
		jz MATCH_CHARS
		mov ax, word[mkxfs_cmd_size]
		cmp ax, word[CMDSTR_COUNT]
		jz MATCH_CHARS
		jmp CMD_NOT_FOUND_MSG

	MATCH_CHARS:
		CMD1:
		mov di, 0
		.loop:
			mov bl, byte[ds:CMDSTR+di]
			mov cl, byte[ds:ed_cmd+di]
			xor bl, cl
			cmp bl, 0
			jnz CMD2
			inc di
			cmp di, word[CMDSTR_COUNT]
			jnz .loop
		call ed
		jmp _finish
		CMD2:
		mov di, 0
		.loop1:
			mov bl, byte[ds:CMDSTR+di]
			mov cl, byte[ds:mkxfs_cmd+di]
			xor bl, cl
			cmp bl, 0
			jnz CMD_NOT_FOUND_MSG
			inc di
			cmp di, word[CMDSTR_COUNT]
			jnz .loop1
		call mkxfs
		jmp _finish


	CMD_NOT_FOUND_MSG:
		cmp word[CMDSTR_COUNT], 0
		jz _finish
		mov si, word CMDSTR
		xor cx, cx
		mov cx, word[CMDSTR_COUNT]
		call printString

		mov si, word cmd_not_found_str
		xor cx, cx
		mov cx, word[cmd_not_found_str_size]
		call printString
		jmp _finish

	_finish:
		mov word[CMDSTR_COUNT], 0
		popa
		jmp showPrompt


	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	;;
	;;
		ed_cmd				DB	"ed"
		ed_cmd_size			DW	($-ed_cmd)
		mkxfs_cmd			DB	"mkxfs"
		mkxfs_cmd_size			DW	($-mkxfs_cmd)
		cmd_not_found_str		DB	": command not found!", 0x0D, 0x0A
		cmd_not_found_str_size	DW	($-cmd_not_found_str)
	;;
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;




;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; mkxfs: create a X Filesystem on a primary partition
;;

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; macros definition ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
%macro	getData 4

%4:
	push word %2
	push word ds
	push word[ds:%3]
	mov bx, ss
	mov es, bx
	mov bx, sp
	mov ax, 4
	int 0x26
	add sp, 6

	mov word[ds:%1], 0
	mov word[ds:%1+2], 0
	.start:
		xor ax, ax
		mov ah, 0x10
		int 0x16
		mov byte[sc_code], ah
		mov byte[char_code], al

		cmp byte[sc_code], 0x1C			; look for RET (scancode = 0x1C) keypress
		jz .end					; data input complete jmp to next location
	
		;; check if character entered is a digit, else ignore
		cmp byte[char_code], 48
		jl .start
		cmp byte[char_code], 57
		jg .start
	
		;; print the entered digit on terminal
		push word char_code
		push word ds
		push word 1
		mov bx, ds
		mov es, bx
		mov bx, sp
		mov ax, 4
		int 0x26
		add sp, 6

		;; convert to decimal and save in %1
		mov ax, word[ds:%1]
		mov bx, 10
		mul bx
		push ax
		push dx
		mov ax, word[ds:%1+2]
		mul bx
		pop dx
		add dx, ax
		pop ax
		sub byte[char_code], 48		; get the digit
		xor bx, bx
		mov bl, byte[char_code]
		add ax, bx		; add to the number
		mov word[ds:%1], ax
		mov word[ds:%1+2], dx
		jmp .start

	.SIZE_OVERFLOW:
		push word size_overflow_msg
		push word ds
		push word size_overflow_msg_size
		mov bx, ss
		mov es, bx
		mov bx, sp
		mov ax, 4
		int 0x26
		add sp, 6

	.end:
		;; print a newline
		push word newline_str
		push word ds
		push word[newline_str_size]
		mov bx, ds
		mov es, bx
		mov bx, sp
		mov ax, 4
		int 0x26
		add sp, 6
		nop
%endmacro

%macro invalidValue 3

	mov si, word invalid_value_str
	mov cx, word[invalid_value_str_size]
	call printString

	mov si, word %1
	mov cx, word[%2]
	sub cx, 3
	call printString

	mov si, word newline_str
	mov cx, word[newline_str_size]
	call printString

	jmp %3
	
%endmacro
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

mkxfs:
	pusha

	getData START_SECTOR, ask_start_sector_str, ask_start_sector_str_size, GET_START_SECTOR
	mov ax, word[ds:START_SECTOR]
	or ax, word[ds:START_SECTOR+2]
	cmp ax, 0
	jnz GET_PART_SIZE
	invalidValue ask_start_sector_str, ask_start_sector_str_size, GET_START_SECTOR


	getData PART_SIZE, ask_part_size_str, ask_part_size_str_size, GET_PART_SIZE
	mov ax, word[ds:PART_SIZE]
	or ax, word[ds:PART_SIZE+2]
	cmp ax, 0
	jnz GET_CHS
	invalidValue ask_part_size_str, ask_part_size_str_size, GET_PART_SIZE



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;  File System Description  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; 
; 					block size = 1 sector
; 					FAT size = 1 block
; 					
; 					+-----------+----------+--------------------------------------------------------------+
; 					| BLOCK NUM | BYTE NUM |                             DATA                             |
; 					+-----------+----------+--------------------------------------------------------------+
; 					|           | 0 - 1    | 0x66BB (Filesystem Signature)                                |
; 					|           +----------+--------------------------------------------------------------+
; 					|           | 2 - 63   | author ID string followed by at least 1 null byte,           |
; 					|           |          | remaining space must be filled with null bytes               |
; 					|           +----------+--------------------------------------------------------------+
; 					|           | 64 - 66  | 0x3+START_SECTOR (first FAT's block number) (fixed)          |
; 					|     1     +----------+--------------------------------------------------------------+
; 					|           | 67 - 69  | 0x5+START_SECTOR (topmost dir's 1st block num) (fixed)       |
; 					|           +----------+--------------------------------------------------------------+
; 					|           | 70 - 72  | 0x7+START_SECTOR (first block of free blocks list)           |
; 					|           +----------+--------------------------------------------------------------+
; 					|           | 73 - 75  | (PART_SIZE-1)+START_SECTOR (last block of free blocks list)  |
; 					|           +----------+--------------------------------------------------------------+
; 					|           | 76 - 511 | reserved                                                     |
; 					+-----------+----------+--------------------------------------------------------------+
; 					|     2     | 0 - 511  | reserved                                                     |
; 					+-----------+----------+--------------------------------------------------------------+
; 					|           |          | first FAT (all FATs are 1 block) (all entries 3 bytes).      |
; 					|           | 0 - 511  | Since blocks 0 - 3 are used for FS info, so file entries     |
; 					|           |          | start from the 5th entry in the first FAT                    |
; 					|           +----------+--------------------------------------------------------------+
; 					|           | 0 - 2    | 0x0 (EOF) (1st entry in FAT)                                 |
; 					|           |          | (corresponds to 1st block of partition)                      |
; 					|           +----------+--------------------------------------------------------------+
; 					|           | 3 - 5    | 0x0 (EOF) (2nd entry in FAT)                                 |
; 					|           +----------+--------------------------------------------------------------+
; 					|           | 6 - 8    | 0x0 (EOF) (3rd entry in FAT)                                 |
; 					|     3     +----------+--------------------------------------------------------------+
; 					|           | 9 - 11   | 0x0 (EOF) (4th entry in FAT)                                 |
; 					|           +----------+--------------------------------------------------------------+
; 					|           | 12 - 14  | 0x6 (topmost dir's next block) (5th FAT entry)               |
; 					|           +----------+--------------------------------------------------------------+
; 					|           | 15 - 17  | 0x0 (EOF) (topmost dir's 2nd n last block) (6th FAT entry)   |
; 					|           +----------+--------------------------------------------------------------+
; 					|           | 18 - 20  | 0x08 (first block of free blocks list)                       |
; 					|           +----------+--------------------------------------------------------------+
; 					|           | 21 - 507 | initialised to next block number,                            |
; 					|           |          | part of the free blocks list                                 |
; 					|           +----------+--------------------------------------------------------------+
; 					|           | 508 - 511| block number of next FAT                                     |
; 					+-----------+----------+--------------------------------------------------------------+
; 					|     4     | 0 - 511  | reserved                                                     |
; 					+-----------+----------+--------------------------------------------------------------+
; 					|           |          | topmost dir's block 1                                        |
; 					|     5     | 0 - 511  | (initially only 5th and 6th blocks are allocated)            |
; 					|           |          | (more blocks are allocated as needed)                        |
; 					+-----------+----------+--------------------------------------------------------------+
; 					|     6     | 0 - 511  | topmost dir's block 2                                        |
; 					+-----------+----------+--------------------------------------------------------------+
;
;
; 		Directory Entry
; 		
; 			+--------------+----------------+---------------------+--------------------+-------------------------+-------------------+
; 			|  16 bytes    |     1 byte     |       3 bytes       |       3 bytes      |      6 bytes            |      6 bytes      |
; 			+--------------+----------------+---------------------+--------------------+-------------------------+-------------------+
; 			| File Name    | drwx----       | First Block of File | File Size in bytes | Modification Time       | Last Access Time  |
; 			|              |                |                     |                    | Year         [12] [MSB] |                   |
; 			| a-Z, 0-9,    | (File Attrbts) |                     |                    | Month        [4]        |                   |
; 			| 'dot',       |                |                     |                    | Day          [5]        |                   |
; 			| 'underscore' |                |                     |                    | Secs in Day  [20] [LSB] |                   |
; 			+------------- +----------------+---------------------+--------------------+-------------------------+-------------------+
;




	;; sign the partition's first sector with XFS Signature and next an optional string 
	;; about fs creator program, all this within first 64 bytes.
	mov ax, word[ds:XFS_SIGNATURE]
	mov word[ds:MKFS_BUFFER], ax
	mov di, 2
	.cploop:
		mov al, byte[ds:XFS_string+di-2]
		mov byte[ds:MKFS_BUFFER+di], al
		inc di
		mov ax, word[ds:XFS_string_size]
		add ax, 2
		cmp di, ax
		jnz .cploop

	;; fill remaining bytes before byte64 with 0x00
	.loop:
		cmp di, 64
		jz .loop_end
		mov byte[ds:MKFS_BUFFER+di], 0x00
		inc di
		jmp .loop
	.loop_end:

	;; block num of first FAT at byte 64
	mov ax, word[ds:START_SECTOR]
	mov dx, word[ds:START_SECTOR+2]
	add ax, 0x0003
	adc dx, 0
	mov word[ds:MKFS_BUFFER+di], ax
	mov byte[ds:MKFS_BUFFER+di+2], dl
	add di, 3

	;; topmost dir's 1st block number
	mov ax, word[ds:START_SECTOR]
	mov dx, word[ds:START_SECTOR+2]
	add ax, 0x0005
	adc dx, 0
	mov word[ds:MKFS_BUFFER+di], ax
	mov byte[ds:MKFS_BUFFER+di+2], dl
	add di, 3

	;; first block of free blocks list
	mov ax, word[ds:START_SECTOR]
	mov dx, word[ds:START_SECTOR+2]
	add ax, 0x0007
	adc dx, 0
	mov word[ds:MKFS_BUFFER+di], ax
	mov byte[ds:MKFS_BUFFER+di+2], dl
	add di, 3

	;; last block of free blocks list
	mov bx, word[ds:PART_SIZE]
	mov cx, word[ds:PART_SIZE+2]
	sub bx, 1
	sbb cx, 0
	mov ax, word[ds:START_SECTOR]
	mov dx, word[ds:START_SECTOR+2]
	add ax, bx
	adc dx, cx
	mov word[ds:MKFS_BUFFER+di], ax
	mov byte[ds:MKFS_BUFFER+di+2], dl
	add di, 3

	;; fill remaining bytes before byte512 with 0x00
	.loop:
		cmp di, 512
		jz .loop_end
		mov byte[ds:MKFS_BUFFER+di], 0x00
		inc di
		jmp .loop
	.loop_end:

	; 1st sector completed, write to disk
	mov word[ds:WRITE_OFFSET], 0x0000
	mov word[ds:WRITE_OFFSET+2], 0x0000
	call WRITE_BUF_TO_SECTOR


	;; 2nd sector is reserved, so fill it with zeros
	mov di ,0
	.loop:
		cmp di, 512
		jz .loop_end
		mov byte[ds:MKFS_BUFFER+di], 0x00
		inc di
		jmp .loop
	.loop_end:

	; 2nd sector completed, write to disk
	mov word[ds:WRITE_OFFSET], 0x0001
	mov word[ds:WRITE_OFFSET+2], 0x0000
	call WRITE_BUF_TO_SECTOR


	;; 3rd sector is the first FAT
	mov di, 0
	mov word[ds:MKFS_BUFFER], 0x0000
	mov word[ds:MKFS_BUFFER+2], 0x0000
	mov word[ds:MKFS_BUFFER+4], 0x0000
	mov word[ds:MKFS_BUFFER+6], 0x0000
	mov word[ds:MKFS_BUFFER+8], 0x0000
	mov word[ds:MKFS_BUFFER+10], 0x0000
	add di, 12
	mov word[ds:MKFS_BUFFER+di], 0x0006
	mov byte[ds:MKFS_BUFFER+di+2], 0x00
	add di, 3
	mov word[ds:MKFS_BUFFER+di], 0x0000
	mov byte[ds:MKFS_BUFFER+di+2], 0x00
	add di, 3
	; rest of initialisation of the FAT will be done by INIT_FREE_LIST
	; 3rd sector completed, write to disk
	mov word[ds:WRITE_OFFSET], 0x0002
	mov word[ds:WRITE_OFFSET+2], 0x0000
	call WRITE_BUF_TO_SECTOR


	;; 4th sector reserved, fill with zeros
	mov di ,0
	.loop:
		cmp di, 512
		jz .loop_end
		mov byte[ds:MKFS_BUFFER+di], 0x00
		inc di
		jmp .loop
	.loop_end:
	; 4th sector completed, write to disk
	mov word[ds:WRITE_OFFSET], 0x0003
	mov word[ds:WRITE_OFFSET+2], 0x0000
	call WRITE_BUF_TO_SECTOR


	; 5th sector, topmost dir
	; for toplevel dir ".." should point to the same dir, like "."

	;; first make entry for the doubledot dir file
	mov di, 0
	.loop:
		mov al, byte[ds:ddot_str+di]
		mov byte[ds:MKFS_BUFFER+di], al
		inc di
		cmp di, 16
		jnz .loop
	
	mov byte[ds:MKFS_BUFFER+di], 11100000b		; drwx0000
	inc di

	; it points to the topmost dir itself, since there is no dir above it
	; store first block of the dir file
	mov ax, word[ds:START_SECTOR]
	mov dx, word[ds:START_SECTOR+2]
	add ax, 0x0005
	adc dx, 0
	mov word[ds:MKFS_BUFFER+di], ax
	mov byte[ds:MKFS_BUFFER+di+2], dl
	add di, 3
	; store file size in bytes
	mov word[ds:MKFS_BUFFER+di], 0x0400
	mov byte[ds:MKFS_BUFFER+di+2], 0x00
	add di, 3
	; ignore mod time and access time for now, to be implemented later
	.loop:
		mov byte[ds:MKFS_BUFFER+di], 0x00
		inc di
		cmp di, 35
		jnz .loop

	;; 2nd entry is for the dot dir file
	mov di, 0
	.loop:
		mov al, byte[ds:dot_str+di]
		mov byte[ds:MKFS_BUFFER+35+di], al
		inc di
		cmp di, 16
		jnz .loop
	
	mov byte[ds:MKFS_BUFFER+35+di], 11100000b		; drwx0000
	inc di

	mov ax, word[ds:START_SECTOR]
	mov dx, word[ds:START_SECTOR+2]
	add ax, 0x0005
	adc dx, 0
	mov word[ds:MKFS_BUFFER+35+di], ax
	mov byte[ds:MKFS_BUFFER+35+di+2], dl
	add di, 3

	mov word[ds:MKFS_BUFFER+35+di], 0x0400
	mov byte[ds:MKFS_BUFFER+35+di+2], 0x00
	add di, 3

	.loop:
		mov byte[ds:MKFS_BUFFER+35+di], 0x00	; right now we leave the modify and access times
		inc di
		cmp di, 35
		jnz .loop

	;; now we must make an entry to indicate last entry in the dir, for this we use the fact
	;; that filename can't start with null bytes. So we need to check only the first two bytes
	;; to verify the END OF ENTRIES marker. Its not necessary to fill null bytes after the 
	;; marker but we still do it
	mov di, 70		; since we have made two entries till now
	.loop:
		mov byte[ds:MKFS_BUFFER+di], 0x00	; right now we leave the modify and access times
		inc di
		cmp di, 512
		jnz .loop

	;; finally write the 5th block to disk
	mov word[ds:WRITE_OFFSET], 0x0004
	mov word[ds:WRITE_OFFSET+2], 0x0000
	call WRITE_BUF_TO_SECTOR

	;; Although not necessary, we fill the topmost dir file's 2nd block with nulls just to ensure cleanliness
	mov di, 0
	.loop:
		mov byte[ds:MKFS_BUFFER+di], 0x00	; right now we leave the modify and access times
		inc di
		cmp di, 512
		jnz .loop

	;; finally write the 6th block to disk
	mov word[ds:WRITE_OFFSET], 0x0005
	mov word[ds:WRITE_OFFSET+2], 0x0000
	call WRITE_BUF_TO_SECTOR


	AllocateFATs:
		mov ax, word[ds:PART_SIZE]
		sub ax, 170
		push ax
		mov ax, word[ds:PART_SIZE+2]
		sbb ax, 0
		push ax
		.loop:
			pop ax
			cmp ax, 0
			jz .check_low_bits
			.check_low_bits:

	InitFreeBlocks:
		;; Now its time to initialise the free blocks list
		mov ax, word[ds:START_SECTOR]
		mov dx, word[ds:START_SECTOR+2]
		add ax, 0x0007
		adc dx, 0
		mov di, 0



	WRITE_BUF_TO_SECTOR:
		; write sector to disk
		mov ax, word[ds:START_SECTOR]
		mov dx, word[ds:START_SECTOR+2]
		add ax, word[ds:WRITE_OFFSET]
		adc dx, word[ds:WRITE_OFFSET+2]
		call GET_CHS

		push word MKFS_BUFFER
		push word ds
		push word[PART_START_SECTOR]
		push word[PART_START_HEAD]
		push word[PART_START_CYL]
		push word 0
		push word[MKFS_BUFFER_SIZE]
		mov bx, ss
		mov es, bx
		mov bx, sp
		mov ax, 1
		int 0x26
		add sp, 14
		ret


	GET_CHS:
		; get CHS of Start Sector, dx:ax contains the sector number
		sub ax, 1
		sbb dx, 0
		mov bx, 16128
		div bx
		mov word[ds:PART_START_CYL], ax
		mov ax, dx
		mov dx, 0
		mov bx, 63
		div bx
		mov word[ds:PART_START_HEAD], ax
		inc dx
		mov word[ds:PART_START_SECTOR], dx

	popa
	ret


	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	;;
	;;
		ask_start_sector_str		DB	"first sector of the partition to be formatted with XFS : "
		ask_start_sector_str_size	DW	($-ask_start_sector_str)
		newline_str			DB	0x0D, 0x0A
		newline_str_size		DW	($-newline_str)
		ask_part_size_str		DB	"size of the partition : "
		ask_part_size_str_size		DW	($-ask_part_size_str)
		invalid_value_str		DB	"invalid value for "
		invalid_value_str_size		DW	($-invalid_value_str)
		allowed_range_str		DB	0x0D, 0x0A, "allowed range "
		allowed_range_str_size		DW	($-allowed_range_str)
		size_overflow_msg		DB	"Size Overflow.", 0x0D, 0x0A
		size_overflow_msg_size		DW	($-size_overflow_msg)
		dot_str				DB	".",0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
		ddot_str			DB	"..",0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00

		sc_code				DB	0x00
		char_code			DB	0x00

		START_SECTOR			DD	0x00000000
		PART_SIZE			DD	0x00000000
		WRITE_OFFSET			DD	0x00000000

		PART_START_CYL			DW	0x0000
		PART_START_HEAD			DW	0x0000
		PART_START_SECTOR		DW	0x0000

		XFS_SIGNATURE			DW	0x66BB
		XFS_string			DB	"XFS FILESYSTEM CREATED BY MKXFS V0.1",0x0D, 0x0A
		XFS_string_size			DW	($-XFS_string)

		MKFS_BUFFER		TIMES		512		DB		0x00
		MKFS_BUFFER_SIZE		DW	512
	;;
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;




;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; ed: a simple text editor
;;
ed:
	pusha
	getChar:
		xor ax, ax
		mov ah, 0x10
		int 0x16
		mov byte[scode], ah
		mov byte[char], al

	cmp byte[scode], 0x01			; look for ESC keypress
	jz WRITE_BUFFER_TO_DISK

	cmp byte[char], 32
	jl getChar
	cmp byte[char], 126
	jg getChar

	mov si, word char
	xor cx, cx
	mov cl, 1
	call printString

	mov di, word[BUFFER_OFFSET]
	mov byte[ds:ED_BUFFER+di], al
	inc word[BUFFER_OFFSET]
	inc di
	cmp di, word[BUFFER_SIZE]
	jz WRITE_BUFFER_TO_DISK
	jmp getChar

	WRITE_BUFFER_TO_DISK:
		mov si, word write_msg
		xor cx, cx
		mov cl, byte[write_msg_size]
		call printString

		push word ED_BUFFER
		push word ds
		push word 20
		push word 0
		push word 0
		push word[BYTES_WRITTEN_TO_DISK]
		push word[BUFFER_OFFSET]
		mov bx, ss
		mov es, bx
		mov bx, sp
		mov ax, 1
		int 0x26
		add sp, 14

		mov si, word write_done
		xor cx, cx
		mov cl, byte[write_done_size]
		call printString

		cmp byte[scode], 0x01
		jz BYE
		mov ax, word[BUFFER_OFFSET]
		add word[BYTES_WRITTEN_TO_DISK], ax
		mov word[BUFFER_OFFSET], 0
		jmp getChar

	BYE:
		mov si, word bye_msg
		xor cx, cx
		mov cl, byte[bye_msg_size]
		call printString

		popa
		ret

	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	;;
		write_msg		DB	0x0d, 0x0a, "going to write buffer to disk..."
		write_msg_size		DB	($-write_msg)
		write_done		DB	0x0d, 0x0a, "write successful."
		write_done_size		DB	($-write_done)
		bye_msg			DB	0x0d, 0x0a, "Leaving editor....", 0x0d, 0x0a
		bye_msg_size		DB	($-bye_msg)

		scode	DB	0x00
		char	DB	0x00

		ED_BUFFER		TIMES	512	DB	0x00

		BUFFER_SIZE		DW	512
		BUFFER_OFFSET		DW	0x0000
		BYTES_WRITTEN_TO_DISK	DW	0x0000
	;;
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; printString: PRINT MESSAGES ON THE TERMINAL
;;
printString:
	pusha
	push si
	push word ds
	push cx
	mov bx, ds
	mov es, bx
	mov bx, sp
	mov ax, 4
	int 0x26
	add sp, 6

	;; save current cursor position
	mov ah, 0x03
	mov bh, 0x00
	int 0x10
	mov [CURR_CURSOR_ROW], dh
	mov [CURR_CURSOR_COL], dl

	popa
	ret
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Data Section
;;

	__PROC_LOAD_SEGMENT_NUM__	DW	0x0000
	

	CURR_CURSOR_ROW		DB	0X0000
	CURR_CURSOR_COL		DB	0X0000

	SPACE_STRING		DB	"             "
	SCAN_STRING		DB	"SCANCODE:0x"
	SCAN_STRING_SIZE	DB	11
	ASCII_STRING		DB	0xD, 0xA,"ASCII:"
	ASCII_STRING_SIZE	DB	8
	PROMPT_STRING		DB	"[user@Ripper]$ "
	PROMPT_STRING_SIZE	DB	15
	CURR_PROMPT_ROW		DB	0x00
	HEX_CHARS		DB	"0","1","2","3","4","5","6","7","8","9","A","B","C","D","E","F"

	RET_KEY			DB	0x1C ;, RET_KEY_HANDLER
	BACKSPACE_KEY		DB	0x0E;, BACKSPACE_KEY_HANDLER,
	UP_KEY			DB	0x48;, UP_ARROW_KEY_HANDLER, 
	DOWN_KEY		DB	0x50;, DOWN_ARROW_KEY_HANDLER, 
	LEFT_KEY		DB	0x4B;, LEFT_ARROW_KEY_HANDLER, 
	RIGHT_KEY		DB	0x4D;, RIGHT_ARROW_KEY_HANDLER, 
	DEL_KEY			DB	0x53;, DEL_KEY_HANDLER, \
; 				0x01, ESC_KEY_HANDLER, 
; 					0x3B, F1_KEY_HANDLER, 
; 					0x3C, F2_KEY_HANDLER, 
; 					0x3D, F3_KEY_HANDLER, 
; 					0x3E, F4_KEY_HANDLER, 
; 					0x3F, F5_KEY_HANDLER, 
; 					0x40, F6_KEY_HANDLER, 
; 					0x41, F7_KEY_HANDLER, 
; 					0x42, F8_KEY_HANDLER, 
; 					0x43, F9_KEY_HANDLER, 
; 					0x44, F10_KEY_HANDLER,
; 					0x85, F11_KEY_HANDLER,
; 					0x86, F12_KEY_HANDLER

	SP_KEY_TABLE_ROW_SIZE	DB	0x03
	SP_KEY_TABLE_SIZE	DB	0x14

	MAX_CMDSTR_SIZE		DW	200
	CMDSTR_COUNT		DW	0x0000
	CMDSTR		TIMES	40	DB	0x00
;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

