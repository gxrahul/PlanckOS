[BITS 16]
[ORG 0]

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;; Date: 4th July 2009
;; Author: Rahul Ranjan
;; version: 0.1
;; A monolithic 16 bit kernel for x86 PCs
;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

%define __KERNEL_LOAD_ADDRESS__   0x07F0
%define __STACK_START_ADDRESS__   0x07F0
%define __STACK_START_OFFSET__    0x5000


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; This routine calls up different initialization routines, it is called by the bootloader
;;
initKernel:
;   cli	; disable interrupts

	;; first of all initialize segments
	jmp initSegments
	INITKERNEL_RET_ADDR_INITSEG:
	;; ready to setup system-calls
	setUpSysCalls:
		call initSysCalls

	;; hardware interrupt handler setup.
	;; we skip this for now. Without it
	;; I/O will not be parallelized.
	;; Also we can't implement process
	;; switching without it

	;; initialise process management
	call init_PM

	;; initialise memory-management
	call init_MM

	;; initialize I_O_Manager
	call init_I_O_Manager

	;; start the scheduler
	call init_Scheduler

	mov byte[KERNEL_INITIALIZED], 0x01

 call setPageColor

	;; start the init process
	;; init is the first user process
	;; all other processes are either
	;; started by init or its children
	call start_Init


; 	sti
;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Some code for testing
;;
; 	call setPageColor
; 	jmp theShell
; 	mov si, welcome_msg
; 	xor cx, cx
; 	mov cl, [welcome_msg_size]
; 	call printString
; 
; 	int 0x26
; 
; 	call [SYS_CALL_TABLE]
; 	call [SYS_CALL_TABLE+4]
; 	call [SYS_CALL_TABLE+8]
; 	call [SYS_CALL_TABLE+12]
; 
; 	JMP $
;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; This routine sets up various segments, i.e. DS, ES, SS and also the stack pointers BP and SP
;; Note that we don't set CS since it is already set by the bootloader to the point in memory 
;; where kernel code is loaded. This routine should run whenever kernel gets to run.
;;
initSegments:
	xor eax, eax
	xor ebx, ebx
	xor ecx, ecx
	xor edx, edx
	mov ax, __KERNEL_LOAD_ADDRESS__
	mov ds, ax
	mov es, ax
	mov ax, __STACK_START_ADDRESS__
	mov ss, ax
	mov bp, __STACK_START_OFFSET__
	mov sp, bp
	cmp byte[KERNEL_INITIALIZED], 0x01
	jnz INITKERNEL_RET_ADDR_INITSEG
	jmp HANDLER_RET_ADDR_INITSEG
;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Kernel Initialization Routine for
;; setting up SYS_CALL_TABLE
;; This should run only once, when the kernel is loaded at boot
;;
initSysCalls:
	;; setup the system-call table
	%macro setupSysCallTable 2
		mov eax, __KERNEL_LOAD_ADDRESS__
		shl eax, 4
		shl eax, 4
		shl eax, 4
		shl eax, 4
		add eax, %1
		mov dword [SYS_CALL_TABLE+4*%2], eax
	%endmacro

	setupSysCallTable read_SysCall, 0
	setupSysCallTable write_SysCall, 1
	setupSysCallTable fork_SysCall, 2
	setupSysCallTable execve_SysCall, 3
	setupSysCallTable write_To_Terminal, 4

	;; setup the interrupt vector table, so that 'int 26h' points to the system-call handler
	mov bx, 0x0098
	mov ax, 0x0000
	mov es, ax
	mov ecx, __KERNEL_LOAD_ADDRESS__
	shl ecx, 4
	shl ecx, 4
	shl ecx, 4
	shl ecx, 4
	mov eax, sysCallHandler
	add eax, ecx
	mov [es:bx], dword eax

	ret
;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;





;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; init_PM: initialise process management data structures and routines
;;
init_PM:
	pusha

	mov ax, word FREE_PCB_LIST_HEAD
	mov bx, word[FREE_PCB_LIST_HEAD]
	mov di, 0
	.loop:
		mov word[ds:bx+PCB.prev], ax
		mov ax, bx
		add ax, word[PCB_SIZE]
		mov word[ds:bx+PCB.next], ax
		mov ax, bx
		mov cx, word[ds:bx+PCB.next]
		mov bx, cx
		inc di
		cmp di, word[MAX_NUM_PCB]
		jnz .loop
	popa
	ret
;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;





;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; init_MM:
;;
init_MM:
	mov word[NUM_FREE_MEM_BLOCKS], 240		;; considering init is loaded at absolute Physical Add 0xCF10 and total available mem in real mode is 1MB

	mov word[FREE_MEM_BLOCKS_LIST_HEAD_SEG_NUM], ds
	xor ax, ax
	add ax, MEM_BLOCK_STRUC_SPACE
	mov word[FREE_MEM_BLOCKS_LIST_HEAD_OFF], ax

	mov ax, word[FREE_MEM_BLOCKS_LIST_HEAD_SEG_NUM]
	mov es, ax
	mov bx, word[FREE_MEM_BLOCKS_LIST_HEAD_OFF]
	mov di, 0					; loop counter
	.loop:
		mov ax, word[USR_PROC_ADD_SPACE_START_SEGMENT_NUM]
		mov cx, di
		shl cx, 4
		shl cx, 4						; now cx=di*256=di*(4096/16)
		add ax, cx						; next segment no
		mov word[es:bx+MEM_BLOCK.mem_block_seg_num], ax
		mov ax, word[USR_PROC_ADD_SPACE_START_OFFSET]		; offset is always at 0x0000
		mov word[es:bx+MEM_BLOCK.mem_block_off], ax
		inc di
		cmp di, 239
		jz .loop_exit
		mov word[es:bx+MEM_BLOCK.next_block_seg_num], ds
		xor ax, ax
		add ax, MEM_BLOCK_STRUC_SPACE
		mov cx, di
		shl cx, 3
		add ax, cx
		mov word[es:bx+MEM_BLOCK.next_block_off], ax		; size of one memory block struc is 8 byte

		mov ax, word[es:bx+MEM_BLOCK.next_block_seg_num]
		mov cx, word[es:bx+MEM_BLOCK.next_block_off]
		mov es, ax
		mov bx, cx
		jmp .loop

		.loop_exit:
			mov word[es:bx+MEM_BLOCK.next_block_seg_num], 0		; last block
			mov word[es:bx+MEM_BLOCK.next_block_off], 0		; last block
	ret
;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;





;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; init_I_O_Manager:
;;
init_I_O_Manager:
	pusha

	mov di, 0
	.loop:
		mov ax, di
		mov bl, 120
		mul bl
		add ax, word FDT_MEM_SPACE
		mov bx, word FREE_FDT_LIST
		mov word[ds:bx+di], ax
		inc di
		cmp di, word[MAX_NUM_OF_FDTS]
		jnz .loop

	popa
;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;





;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; init_Scheduler:
;;
init_Scheduler:
	ret
;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;




;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; start_Init:
;;
start_Init:
	;; create PCB for the new process and set current context to it
; 	mov word[FREE_PCB_LIST_HEAD], PCB_MEM_SPACE
	call pcb_Alloc
	mov word[ds:bx+PCB.pid], 1
	mov word[ds:bx+PCB.ppid], 0
	mov byte[ds:bx+PCB.state], 1		;; state=1 means ready to run, state=0 implies blocked
	mov word[CURRENT_PROC_PCB], bx

	;; request the memory manager (mem_Alloc routine) for allocation of INIT_PROC_MEM_SIZE bytes
	push word 0x0000				;; this is where mem_Alloc will return offset of pointer to the allocated memory
	push word 0x0000				;; this is where mem_Alloc will return segment num of pointer to the allocated memory
	push word 0x1000				;; no of bytes requested for allocation
	mov word[SYS_CALL_ARGS_ADD_SEG], ss
	mov word[SYS_CALL_ARGS_ADD_OFF], sp
	call mem_Alloc

	pop ax
	mov ax, 0x1000
	mov word[ds:bx+PCB.mem_size], ax
	pop ax					;; segment no of allocated memory
	mov word[ds:bx+PCB.start_mem_seg], ax
	mov word[ds:bx+PCB.start_mem_off], 0	;; we always keep offset as 0

	;; finally load the init program code in the allocated memory and jump to it, ideally scheduler should be run
	push word[ds:bx+PCB.start_mem_off]
	push word[ds:bx+PCB.start_mem_seg]
	push word initProgCode
	push ds
	push INIT_PROG_SIZE
	mov ax, ss
	mov es, ax
	mov bx, sp
	call memcpy

	push WORD 0x0CF1
	push WORD 0x0000
	retf


ret
	

	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	;; Local Routine Data
	;;
; 		INIT_PROC_ADD_SEGMENT_NUM	DW	0X17F0
; 		INIT_PROC_ADD_OFFSET		DW	0X0000
		INIT_PROC_MEM_SIZE		DW	0X1000
		cpcount				DB	0
		INIT_PROG_SIZE			DW	300
	;;
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


;;#################################################################################################################################################################;;
;;##																				 ##;;
;;##  					INITIALISATION DONE, NOW THE REAL KERNEL ROUTINES START									 ##;;
;;##																				 ##;;
;;#################################################################################################################################################################;;




;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; scheduler: The process scheduler, right now it does nothing
;;
scheduler:
	pusha
	nop
	nop
	popa
	ret

;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;




;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; allocate PCB to a new process:
;;
pcb_Alloc:
	pusha
	cmp word[NUM_FREE_PCB], 0x00
	jz ALLOC_FAIL
	jmp ALLOC_SCCS

	ALLOC_SCCS:
		xor bx, bx
		xor di, di
		mov bx, word[FREE_PCB_LIST_HEAD]

		mov di,word[ds:bx+PCB.next]
		mov word[FREE_PCB_LIST_HEAD], di
		cmp di, 0
		jz UPDATE_ALLOCATED_PCB_LIST
		mov word[ds:di+PCB.prev], FREE_PCB_LIST_HEAD

		UPDATE_ALLOCATED_PCB_LIST:
			mov di, word[ALLOCATED_PCB_LIST_HEAD]
			mov word[ds:bx+PCB.next], di
			mov word[ds:bx+PCB.prev], ALLOCATED_PCB_LIST_HEAD
			mov word[ALLOCATED_PCB_LIST_HEAD], bx
			cmp di, 0
			jz INITIALIZE_PCB
			mov word[ds:di+PCB.prev], bx

		INITIALIZE_PCB:
			mov word[ds:bx+PCB.pid], 0			;; to be filled by calling routine
			mov word[ds:bx+PCB.ppid], 0			;; to be filled by calling routine
			mov byte[ds:bx+PCB.state], 0			;; state=1 means ready to run, state=0 implies blocked
			mov word[ds:bx+PCB.start_mem_seg], 0		;; to be filled by calling routine
			mov word[ds:bx+PCB.start_mem_off], 0		;; to be filled by calling routine
			mov word[ds:bx+PCB.mem_size], 0			;; to be filled by calling routine
		xor ax, ax
		mov ax, 1	; return status of the routine, bx contains the new PCB address offset(wrt DS)
		jmp pcb_Alloc.func_ret

	ALLOC_FAIL:
		xor ax, ax
		jmp pcb_Alloc.func_ret

	pcb_Alloc.func_ret:
		mov word[ret_status], ax
		mov word[allocated_pcb_off], bx
		popa
		mov ax, word[ret_status]
		mov bx, word[allocated_pcb_off]
		ret

	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	;;
	;;
		allocated_pcb_off	DW	0x0000
		ret_status		DW	0x0000
	;;
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;





;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; memory_Manager:
;;
mem_Alloc:
	pusha
	xor cx, cx
	mov bx, word[SYS_CALL_ARGS_ADD_SEG]
	mov es, bx
	mov bx, word[SYS_CALL_ARGS_ADD_OFF]
	mov ax, word[es:bx]			;; amount of memory requested in bytes, RIGHT NOW ITS ALWAYS 4KB


	CHECK_AVAILABILITY:
		cmp word[NUM_FREE_MEM_BLOCKS],0
		jz DENY_ALLOC
		jmp ACCEPT_ALLOC

	ACCEPT_ALLOC:
		dec word[NUM_FREE_MEM_BLOCKS]					;; decrement number of free memory blocks

		mov bx, word[FREE_MEM_BLOCKS_LIST_HEAD_SEG_NUM]
		mov es, bx
		mov bx, word[FREE_MEM_BLOCKS_LIST_HEAD_OFF]
		mov ax, word[es:bx+MEM_BLOCK.mem_block_seg_num]
		mov cx, word[es:bx+MEM_BLOCK.mem_block_off]

		mov bx, word[SYS_CALL_ARGS_ADD_SEG]
		mov es, bx					;; load segment no of pointer to return parameters
		mov bx, word[SYS_CALL_ARGS_ADD_OFF]		;; load offset of pointer to return parameters
		mov word[es:bx], 0x1000				;; return number of bytes allocated = 4KB
		mov word[es:bx+2], ax				;; return segment number of pointer to allocated memory
		mov word[es:bx+4], cx				;; return offset of pointer to allocated memory

		mov bx, word[FREE_MEM_BLOCKS_LIST_HEAD_SEG_NUM]
		mov es, bx
		mov bx, word[FREE_MEM_BLOCKS_LIST_HEAD_OFF]
		mov ax, word[es:bx+MEM_BLOCK.next_block_seg_num]
		mov cx, word[es:bx+MEM_BLOCK.next_block_off]
		mov word[FREE_MEM_BLOCKS_LIST_HEAD_SEG_NUM], ax
		mov word[FREE_MEM_BLOCKS_LIST_HEAD_OFF], cx
		jmp mem_Alloc.func_ret

	DENY_ALLOC:
		xor ax, ax
		mov ax, 0x00					;; value 0x00 in return status indicates to the calling process that request has been denied
		mov es, [SYS_CALL_ARGS_ADD_SEG]
		mov bx, [SYS_CALL_ARGS_ADD_OFF]
		mov word[es:bx], 0x0000				;; amount of memory allocated = 0
		jmp mem_Alloc.func_ret

	mem_Alloc.func_ret:
		popa
		ret

;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;






;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Various system-calls
;;
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	;; syscall_1: read_SysCall
	;;
	read_SysCall:
		call read_Disk		; we should first check the file descriptor table and then decide which routine to call
		ret
	;;
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	

	
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	;; syscall_2: write_SysCall
	;;
	write_SysCall:
		call write2Disk
		ret
	;;
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;



	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	;;
	;;
	fork_SysCall:
		pusha

		;; STEP1 => setup PCB for the new process
		call pcb_Alloc
		cmp ax, 1
		jnz fork_fail

		mov word[NEW_PROC_PCB], bx		; save offset of the new PCB in a local var

		;; copy the whole PCB Data of the parent process(CURRENT_PROC_PCB) to the child PCB(NEW_PROC_PCB)
		push word[NEW_PROC_PCB]		; DST_ADD_OFF
		push ds				; DST_ADD_SEG_NUM
		push word[CURRENT_PROC_PCB]	; SRC_ADD_OFF
		push ds				; SRC_ADD_SEG_NUM
		push word[PCB_SIZE]		; NO_OF_BYTES_TO_COPY
		mov ax, ss
		mov es, ax
		mov bx, sp
		call memcpy

		add sp, 10			; clean stack

		mov bx, word[NEW_PROC_PCB]
		mov di, word[CURRENT_PROC_PCB]
		mov cx, word[ds:di+PCB.pid]
		mov word[ds:bx+PCB.ppid], cx
		mov byte[ds:bx+PCB.state], 1		;; state=1 means ready to run, state=0 implies blocked


		mov ax, word[PROCESS_PID_COUNT]
		mov word[ds:bx+PCB.pid], ax

		cmp word[PROCESS_PID_COUNT], MAX_PID
		jz WRAP_PID

		inc word[PROCESS_PID_COUNT]
		jmp PID_DONE

		WRAP_PID:
			mov word[PROCESS_PID_COUNT], 2

		PID_DONE:
			nop

		;; request the memory manager (mem_Alloc routine) for allocation of INIT_PROC_MEM_SIZE bytes
		push 0x0000				;; this is where mem_Alloc will return offset of the pointer to the allocated memory
		push 0x0000				
		push word[ds:di+PCB.mem_size]		;; no of bytes requested for allocation is equal to memory size of the parent process,
							;; since we are creating a copy

		mov word[SYS_CALL_ARGS_ADD_SEG], ss
		mov word[SYS_CALL_ARGS_ADD_OFF], sp
		call mem_Alloc
		pop ax					;; no of bytes allocated
		mov word[ds:bx+PCB.mem_size], ax
		pop ax					;; segment no of allocated memory
		mov word[ds:bx+PCB.start_mem_seg], ax
		pop ax
		mov word[ds:bx+PCB.start_mem_off], 0	;; we always keep offset as 0

		push word[ds:bx+PCB.start_mem_off]
		push word[ds:bx+PCB.start_mem_seg]
		push word[ds:di+PCB.start_mem_off]
		push word[ds:di+PCB.start_mem_seg]
		push word[ds:bx+PCB.mem_size]
		mov ax, ss
		mov es, ax
		mov bx, sp
		call memcpy
		add sp, 10			; clean stack


		mov bx, word[NEW_PROC_PCB]
		mov word[ds:bx+PCB.syscall_ret_value], 0	;; '0' is returned to the child process
		mov ax, word[ds:bx+PCB.pid]
		mov word[ds:di+PCB.syscall_ret_value], ax	;; child's PID is returned to the parent

		;; now change the child's segment registers, return address's segment value, SS saved on PCB and DS pushed on the stack
		mov ax, word[ds:bx+PCB.start_mem_seg]		; this is the segment value for the child's memory space
		mov es, ax
		mov cx, word[ds:bx+PCB.stack_sp]
		mov bx, cx

		mov word[es:bx], ax				; change DS of child to the child's memory segment

		mov word[es:bx+20], ax				; change segment number of return address
		mov bx, word[NEW_PROC_PCB]
		mov word[ds:bx+PCB.stack_ss], ax		; finally change the SS on PCB


		;; STEP2 => Let's setup the FDT for the new process
		mov di, 0
		.loop:
			mov ax, word[ds:FREE_FDT_LIST+di*2]
			cmp ax, 0
			jnz .exit_loop
			inc di
			cmp di, word[MAX_NUM_OF_FDTS]
			jnz .loop

		.exit_loop:
			cmp ax, 0
			jz fork_fail

		mov bx, word[NEW_PROC_PCB]
		mov word[ds:bx+PCB.pointer_to_FDT], ax

		;; make the child's FDT, a copy of the parent's FDT
		;; our policy is that whatever is open for the parent process
		;; is also available to the child process in the same state
		mov di, word[CURRENT_PROC_PCB]
		mov bx, word[NEW_PROC_PCB]
		push word[ds:bx+PCB.pointer_to_FDT]
		push ds
		push word[ds:di+PCB.pointer_to_FDT]
		push ds
		push word[FDT_SIZE]
		mov ax, ss
		mov es, ax
		mov bx, sp
		call memcpy
		add sp, 10			; clean stack

		;; here we are scheduling the child just created for execution, 
		;; it's temporary hack for scheduling, remove after we have a real working scheduler
		mov word[CURRENT_PROC_PCB], bx
		jmp _ret


		fork_fail:
			popa
			mov ax, 0
			ret
		_ret:
			popa
			ret


		;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
		;; Local Routine Data
		;;
			MAX_PID				DW	65535
			PROCESS_PID_COUNT		DW	2
			NEW_PROC_PCB			DW	0
		;;
		;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	;;
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;




	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	;; not a syscall, helper routine, move somewhere else
	;;
	memcpy:
		;; args to this routine are passed through a pointer to a mem location in the register pair ES:BX
		pusha
		mov ax, word[es:bx]
		mov word[NO_OF_BYTES_TO_COPY], ax
		mov ax, word[es:bx+2]
		mov word[SRC_ADD_SEG_NUM], ax
		mov ax, word[es:bx+4]
		mov word[SRC_ADD_OFF], ax
		mov ax, word[es:bx+6]
		mov word[DST_ADD_SEG_NUM], ax
		mov ax, word[es:bx+8]
		mov word[DST_ADD_OFF], ax

		xor bx, bx
		xor ax, ax
		xor di, di
		_cploop:
			mov bx, word[SRC_ADD_SEG_NUM]
			mov es, bx
			mov bx, word[SRC_ADD_OFF]
			xor ax, ax
			mov al, byte[es:bx+di]

			mov bx, word[DST_ADD_SEG_NUM]
			mov es, bx
			mov bx, word[DST_ADD_OFF]
			mov byte[es:bx+di], al
			inc di
			cmp di, word[NO_OF_BYTES_TO_COPY]
			jnz _cploop

		popa
		ret
		;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
		;;
		;;
			NO_OF_BYTES_TO_COPY	DW	0
			SRC_ADD_SEG_NUM		DW	0
			SRC_ADD_OFF		DW	0
			DST_ADD_SEG_NUM		DW	0
			DST_ADD_OFF		DW	0
		;;
		;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	;;
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;




	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	;; syscall_4: execve_SysCall
	;;
	execve_SysCall:
		pusha

		;; Get the syscall parameters first
		mov ax, word[SYS_CALL_ARGS_ADD_SEG]
		mov es, ax
		mov bx, word[SYS_CALL_ARGS_ADD_OFF]
		mov ax, word[es:bx]
		mov word[FILE_START_CYL_NUM], ax	; Cyllinder no of the start of the binary file to load
		mov ax, word[es:bx+2]
		mov word[FILE_START_HEAD_NUM], ax	; Head no of the start of the binary file to load
		mov ax, word[es:bx+4]
		mov word[FILE_START_SECT_NUM], ax	; Sector no of the start of the binary file to load
		mov ax, word[es:bx+6]
		mov word[FILE_SIZE], ax			; Size of the binary file in number of sectors


		;; now call read_Disk routine (device driver for writing to disk)
		;; to load the binary file overwriting calling proc's memory
		mov bx, word[CURRENT_PROC_PCB]
		push word[ds:bx+PCB.start_mem_off]
		push word[ds:bx+PCB.start_mem_seg]
		push word[FILE_START_SECT_NUM]
		push word[FILE_START_HEAD_NUM]
		push word[FILE_START_CYL_NUM]
		push word[FILE_SIZE]
		mov word[SYS_CALL_ARGS_ADD_SEG], ss
		mov word[SYS_CALL_ARGS_ADD_OFF], sp
		call read_Disk
		add sp, 12				; cleanup the stack

		;; examine the parent's file descriptor's status_flags to decide which ones to preserve
		mov bx, word[CURRENT_PROC_PCB]
		mov ax, word[ds:bx+PCB.pointer_to_FDT]
		mov bx, ax
		mov di, 0				; counter for file descriptor
		.loop:
			mov al, byte[ds:bx+di+FD.status_flags]
			shl al, 4
			shl al, 3				; lowest bit is the flag for CLOSE_ON_EXEC, if set clear this file descriptor
			shr al, 3
			shr al, 4				; bring the CLOSE_ON_EXEC flag bit to lowest position, so as to examine it
			cmp al, 1				; check whether CLOSE_ON_EXEC flag is set
			jnz _loop_it				; CLOSE_ON_EXEC flag not set, don't close this file
			; CLOSE_ON_EXEC flag is set,
			; clear this file descriptor
			xor ax, ax
			mov word[ds:bx+di], ax
			mov word[ds:bx+di+2], ax
			mov word[ds:bx+di+4], ax
			mov word[ds:bx+di+6], ax
			mov word[ds:bx+di+8], ax
			mov word[ds:bx+di+10], ax
			mov byte[ds:bx+di+12], al
		_loop_it:
			inc di
			cmp di, word[MAX_NUM_OF_FDS]
			jnz .loop

		;; now set the calling proc's return address's offset part to zero, optionally
		;; we may also clear the general registers saved on the stack
		mov bx, word[CURRENT_PROC_PCB]
		mov ax, word[ds:bx+PCB.start_mem_seg]
		mov es, ax
		mov word[ds:bx+PCB.stack_sp], 0x0FFF	; set the stack pointer so that it points to 0x0fff
		sub word[ds:bx+PCB.stack_sp], 24	; after everything has been popped off
		mov word[ds:bx+PCB.stack_bp], 0x0FFF	; also set the BP
		mov ax, word[ds:bx+PCB.stack_sp]
		mov bx, ax

		mov word[es:bx+20], es			; segment of return address, this will be popped into CS
		mov word[es:bx+18], 0x0000		; offset of return address, this will be popped into IP
		mov word[es:bx+16], 0x0000		; rest are just general purpose registers
		mov word[es:bx+14], 0x0000
		mov word[es:bx+12], 0x0000
		mov word[es:bx+10], 0x0000
		mov word[es:bx+8], 0x0000
		mov word[es:bx+6], 0x0000
		mov word[es:bx+4], 0x0000
		mov word[es:bx+2], 0x0000
		mov word[es:bx], es			; this is where DS register is stored on the stack

		popa
		ret

		;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
		;;
		;;
			FILE_START_CYL_NUM	DW	0x0000
			FILE_START_HEAD_NUM	DW	0x00
			FILE_START_SECT_NUM	DW	0x00
			FILE_SIZE		DW	0x0000
		;;
		;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

	;;
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	;; syscall_5: exit_SysCall
	;;
	exit_SysCall:

		ret
	;;
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	;;syscall_6: malloc_SysCall
	;;
	malloc_SysCall:
		
	;;
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;




;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; The system call handler, this is the starting point whenever
;; a system call is made by some user process
sysCallHandler:

	; first of all save the calling process general purpose registers
	pusha
	push ds
	; disable interrupts
	cli
	; set kernel's data segment context
	mov cx, __KERNEL_LOAD_ADDRESS__
	mov ds, cx

	;; save the syscall arguments
	mov word[ds:SYS_CALL_NO], ax
	mov word[ds:SYS_CALL_ARGS_ADD_OFF], bx
	mov word[ds:SYS_CALL_ARGS_ADD_SEG], es


	;; save the caller's stack parameters on the PCB
	mov bx, word[CURRENT_PROC_PCB]
	mov word[ds:bx+PCB.stack_ss], ss
	mov word[ds:bx+PCB.stack_sp], sp
	mov word[ds:bx+PCB.stack_bp], bp

	;; now we call the segments initialization routine
	jmp initSegments
	HANDLER_RET_ADDR_INITSEG:		;; return address for initSegments

	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	;; JUST FOR TESTING
	;;
; 		call setPageColor
; 		mov si, sys_call_handler_msg
; 		xor cx, cx
; 		mov cx, word[sys_call_handler_msg_size]
; 		call printString
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;



	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	;; Now examine the system call no to decide which routine to call
	;; AL register contains the sys-call no, we use it to index into the sys-call table
	;; and jmp to the address stored there, since it contains the apropriate routine
	;;
		xor ax, ax
		mov ax, word[ds:SYS_CALL_NO]
		call word[ds:SYS_CALL_TABLE + 4 * eax]
	;;
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;



	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	;; Call the sheduler now
	;;
; 		call scheduler
	;;
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;



	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	;; jmp to process runner, to run the scheduled process
	;;
		jmp run_Proc
	;;
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;



;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;





;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; run_Proc: run the scheduled process
;;
run_Proc:
	
	;; Finally before returning from the interrupt, Restore context of the caller
	mov bx, word[CURRENT_PROC_PCB]
	mov ss, word[ds:bx+PCB.stack_ss]
	mov sp, word[ds:bx+PCB.stack_sp]
	mov bp, word[ds:bx+PCB.stack_bp]


	; pass the syscall return value into ax register
	mov bx, word[CURRENT_PROC_PCB]
	mov ax, word[ds:bx+PCB.syscall_ret_value]
	mov bx, ss
	mov es, bx
	mov bx, sp
	mov word[es:bx+16], ax
	sti

	; restore ds register
	pop ds
	; restore all the general purpose registers
	popa


	iret

;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;




;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; read_Disk: device driver for reading data from a disk
;;
read_Disk:
	pusha

	;; read the system-call arguments
	xor ax, ax
	mov ax, word[SYS_CALL_ARGS_ADD_SEG]
	mov es, ax
	mov bx, word[SYS_CALL_ARGS_ADD_OFF]
	mov ax, word[es:bx]
	mov word[NUM_SECTORS_TO_READ], ax			;; number of sectors to read
	mov ax, word[es:bx+2]
	mov word[READ_START_AT_CYL_NUM], ax			;; cylinder no can be of 10 bits in the 'int 0x13 BIOS routine'
	mov ax, word[es:bx+4]
	mov byte[READ_START_AT_HEAD_NUM], al
	mov ax, word[es:bx+6]
	mov byte[READ_START_AT_SECTOR_NUM], al			;; only 63 sectors are possible, so low 6 bits only are used
	mov ax, word[es:bx+8]
	mov word[POINTER_TO_BUFFER_SEG_NUM], ax			;; segment no of the address of memory where data has to be stored
	mov ax, word[es:bx+10]
	mov word[POINTER_TO_BUFFER_OFF], ax			;; offset of the address of memory where data has to be stored

	mov ax, word[NUM_SECTORS_TO_READ]
	mov word[REMAINING_SECTORS_TO_READ], ax

	mov di, 4						;; set number of retries to 4

	READ_SECTORS:
		mov ax, word[POINTER_TO_BUFFER_SEG_NUM]
		mov es, ax
		mov bx, word[POINTER_TO_BUFFER_OFF]
		
		xor ax, ax
		mov al, byte[DISK_PARAM_MAX_SECTORS]
		sub al, byte[READ_START_AT_SECTOR_NUM]	;; number of sectors to read in the current track
		cmp ax, word[REMAINING_SECTORS_TO_READ]
		jle READ_CHUNK

		xor ax, ax
		mov al, byte[REMAINING_SECTORS_TO_READ]	; whole track not to be read

		READ_CHUNK:
			mov byte[NUM_SECTORS_TRYING_TO_READ], al
			mov ah, 0x02
			mov ch, byte[READ_START_AT_CYL_NUM]	; low eight bits of cylinder no
			mov cl, byte[READ_START_AT_CYL_NUM+1]
			shl cl, 4
			shl cl, 2
			add cl, byte[READ_START_AT_SECTOR_NUM]	; bits 0-5 sector no and bits 6-7 high two bits of cylinder no
			mov dh, byte[READ_START_AT_HEAD_NUM]	; head no
			mov dl, 0x80				; drive no, bit 7 set for hard disk

; 			pusha
; 			call hexPrint
; 			mov bx, es
; 			call hexPrint
; 			mov bx, cx
; 			call hexPrint
; 			xor bx, bx
; 			mov bl, dh
; 			call hexPrint
; 			popa


			int 0x13
			jnc READ_NEXT_CHUNK			; IF CARRY FLAG IS SET. IT MEANS READ WAS UNSUCCESSFUL
			jmp RETRY

		RETRY:
			xor ax, ax				
			int 0x13				; Disk reset
			dec di					; decrease number of retries left
			jnz READ_SECTORS
			jmp READ_FAILURE

		READ_NEXT_CHUNK:
			xor ax, ax
			mov al, byte[NUM_SECTORS_TRYING_TO_READ]
			sub word[REMAINING_SECTORS_TO_READ], ax
			cmp word[REMAINING_SECTORS_TO_READ], 0
			jz READ_COMPLETE
			mov byte[READ_START_AT_SECTOR_NUM], 0
			inc byte[READ_START_AT_HEAD_NUM]
			xor ax, ax
			mov al, byte[DISK_PARAM_MAX_HEADS]
			cmp byte[READ_START_AT_HEAD_NUM], al
			jg RESET_HEAD
			jmp READ_SECTORS

			RESET_HEAD:
				mov byte[READ_START_AT_HEAD_NUM], 0
				inc word[READ_START_AT_CYL_NUM]
				xor ax, ax
				mov ax, word[DISK_PARAM_MAX_CYLLINDERS]
				cmp word[READ_START_AT_CYL_NUM], ax
				jg READ_FAILURE
		jmp READ_SECTORS
			

	READ_COMPLETE:
		popa
		mov eax, 1		;; 1 in eax means system call success
		ret

	READ_FAILURE:
		popa
		mov eax, 0		;; 0 in eax means failure of system call
		ret

	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	;; Local Routine Data
	;;
		READ_START_AT_CYL_NUM		DW	0x0000
		READ_START_AT_HEAD_NUM		DB	0x00
		READ_START_AT_SECTOR_NUM	DB	0x00
		NUM_SECTORS_TO_READ		DW	0x0000
		REMAINING_SECTORS_TO_READ	DW	0x0000
		NUM_SECTORS_TRYING_TO_READ	DB	0x00
		POINTER_TO_BUFFER_SEG_NUM	DW	0x0000
		POINTER_TO_BUFFER_OFF		DW	0x0000
	
		;; Disk Parameters, Ideally These Should be obtained by using BIOS INT13h func 8h,
		;; but right now we just take max possible values
		DISK_PARAM_MAX_SECTORS		DB	63
		DISK_PARAM_MAX_HEADS		DB	255
		DISK_PARAM_MAX_CYLLINDERS	DW	1023
	;;
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;





;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; write2Disk: Driver for writing to a Disk
;;
write2Disk:
	pusha

	;; read the system-call arguments
	xor ax, ax
	mov ax, word[SYS_CALL_ARGS_ADD_SEG]
	mov es, ax
	mov bx, word[SYS_CALL_ARGS_ADD_OFF]
	mov ax, word[es:bx]
	mov word[NUM_BYTES_TO_WRITE], ax			;; number of sectors to read
	mov ax, word[es:bx+2]
	mov word[WRITE_START_OFFSET_IN_BLOCK], ax		;; offset in bytes to start writing within a block(we are taking 1 block = 1 sector)
	mov ax, word[es:bx+4]
	mov word[WRITE_START_AT_CYL_NUM], ax			;; cylinder no can be of 10 bits in the 'int 0x13 BIOS routine'
	mov ax, word[es:bx+6]
	mov byte[WRITE_START_AT_HEAD_NUM], al
	mov ax, word[es:bx+8]
	mov byte[WRITE_START_AT_SECTOR_NUM], al			;; only 63 sectors are possible, so low 6 bits only are used
	mov ax, word[es:bx+10]
	mov word[POINTER_TO_BUFFER_SEG_NUM], ax			;; segment no of the address of memory where data has to be stored
	mov ax, word[es:bx+12]
	mov word[POINTER_TO_BUFFER_OFF], ax			;; offset of the address of memory where data has to be stored
; 
; 	pusha
; 	mov bx, word[WRITE_START_AT_CYL_NUM]
; 	call hexPrint
; 	popa
; 
; 	pusha
; 	xor bx, bx
; 	mov bl, byte[WRITE_START_AT_HEAD_NUM]
; 	call hexPrint
; 	popa
; 
; 	pusha
; 	xor bx, bx
; 	mov bl, byte[WRITE_START_AT_SECTOR_NUM]
; 	call hexPrint
; 	popa

	;; find new base CHS
	mov ax, word[WRITE_START_OFFSET_IN_BLOCK]
	mov dx, 0
	mov bx, 512
	div bx							;; dx has remainder and ax has quotient
	mov word[WRITE_START_OFFSET_IN_BLOCK], dx		;; remainder contains the offset bytes into a sector
	xor bx, bx
	mov bl, byte[WRITE_START_AT_SECTOR_NUM]
	add ax, bx
	cmp ax, 63
	jg INC_HEAD
	mov byte[WRITE_START_AT_SECTOR_NUM], al
	jmp CHS_DONE

	INC_HEAD:
		mov bl, 63
		div bl
		mov byte[WRITE_START_AT_SECTOR_NUM], ah		;; remainder ah contains offset sectors into a head
		mov ah, 0
		xor bx, bx
		mov bl, byte[WRITE_START_AT_HEAD_NUM]
		add ax, bx
		cmp ax, 254
		jg INC_CYL
		jmp CHS_DONE

	INC_CYL:
		mov bx, 255
		mov dx, 0
		div bx
		mov byte[WRITE_START_AT_HEAD_NUM], dl		;; remainder contains offset heads into a cyllinder
		add word[WRITE_START_AT_CYL_NUM], ax
		cmp word[WRITE_START_AT_CYL_NUM], 1023
		jg DISK_ACCESS_LIMIT_REACHED

	CHS_DONE:
		nop


	mov di, 0						;; counter for number of sectors written
	mov ax, word[NUM_BYTES_TO_WRITE]
	mov word[REMAINING_BYTES_TO_WRITE], ax
	mov word[NUM_BYTES_TO_WRITE_CURRENT_BLOCK], ax
	SELECT_MODE_OF_WRITING:
		mov ax, word[WRITE_START_OFFSET_IN_BLOCK]
		add ax, word[REMAINING_BYTES_TO_WRITE]
		cmp ax, 512
		jle .one_block_or_less_to_write
		jg .more_than_one_block_to_write

		.one_block_or_less_to_write:
			cmp word[REMAINING_BYTES_TO_WRITE], 512
			jz WRITE_ONE_BLOCK
			jmp READ_ONE_BLOCK_INTO_TMP_BUF
		.more_than_one_block_to_write:
			mov ax, 512
			sub ax, word[WRITE_START_OFFSET_IN_BLOCK]
			mov word[NUM_BYTES_TO_WRITE_CURRENT_BLOCK], ax
			cmp ax, 512
			jz WRITE_ONE_BLOCK
			jmp READ_ONE_BLOCK_INTO_TMP_BUF

	WRITE_ONE_BLOCK:
		mov ah, 0					; Disk reset function
		mov dl, 0x80					; Drive no starts at 0x80
		int 0x13					; make disk reset call

		mov ah, 0x03					; function number of INT13h for writing to disk
		mov al, 1					; we write 1 sector at a time
		mov ch, byte[WRITE_START_AT_CYL_NUM]		; low 8 bits of cyllinder no

	pusha
	xor bx, bx
	mov bl, ch
	call hexPrint
	popa

		mov cl, byte[ds:WRITE_START_AT_CYL_NUM+1]	; cyl no high two bits
		shl cl, 4					; high two bits of cylinder no at 6,7 bits of cl
		shl cl, 2					; high two bits of cylinder no at 6,7 bits of cl
		add cl, byte[WRITE_START_AT_SECTOR_NUM]		; sector no in bits 0-5 of cl

	pusha
	xor bx, bx
	mov bl, cl
	call hexPrint
	popa

		mov dh, byte[WRITE_START_AT_HEAD_NUM]		; head no in DH

	pusha
	xor bx, bx
	mov bl, dh
	call hexPrint
	popa

		mov dl, 0x80					; drive no, first=80h and so on

	pusha
	xor bx, bx
	mov bl, dl
	call hexPrint
	popa

		mov bx, word[POINTER_TO_BUFFER_SEG_NUM]
		mov es, bx
		mov bx, word[POINTER_TO_BUFFER_OFF+di]		; pass buffer pointer in ES:BX
; pusha
; mov si, bx
; mov cx, 38
; call printString
; popa
		int 0x13					; make the call

		jc WRITE_DISK_ERROR
		cmp al, 0
		jz WRITE_DISK_ERROR

		add di, 512					; increment di
		sub word[REMAINING_BYTES_TO_WRITE], 512		; now decrement num of bytes to be written

		jz FINISH_WRITING

		_inc_sector_count:
			cmp byte[WRITE_START_AT_SECTOR_NUM], 63
			jz _inc_head_count
			inc byte[WRITE_START_AT_SECTOR_NUM]
			jmp SELECT_MODE_OF_WRITING

		_inc_head_count:
			mov byte[WRITE_START_AT_SECTOR_NUM], 0
			cmp byte[WRITE_START_AT_HEAD_NUM], 255
			jz _inc_cyl_count
			inc byte[WRITE_START_AT_HEAD_NUM]
			jmp SELECT_MODE_OF_WRITING

		_inc_cyl_count:
			mov byte[WRITE_START_AT_HEAD_NUM], 0
			cmp word[WRITE_START_AT_CYL_NUM], 1024
			jz DISK_ACCESS_LIMIT_REACHED
			inc word[WRITE_START_AT_CYL_NUM]
			jmp SELECT_MODE_OF_WRITING


	READ_ONE_BLOCK_INTO_TMP_BUF:
		mov ah, 0x02					; function no of read
		mov al, 1					; just one sector to read
		mov ch, byte[WRITE_START_AT_CYL_NUM]		; first get the block that we want to write into at some offset
		mov cl, byte[ds:WRITE_START_AT_CYL_NUM+1]	; high two bits of cyl no
		shl cl, 4					; high two bits of cyl no should be at bits 6-7 of cl
		shl cl, 2					; high two bits of cyl no should be at bits 6-7 of cl
		add cl, byte[WRITE_START_AT_SECTOR_NUM]		; sector no at bits 0-5 of cl
		mov dh, byte[WRITE_START_AT_HEAD_NUM]		; head no in dh
		mov dl, 0x80					; disk no for first disk
		mov bx, ds					; to move segment no of temp buffer in es first move to bx
		mov es, bx					; the copy to es, since direct copy not allowed
		mov bx, word TMP_WRITE_BUFFER			; offset to temp buffer in bx
		int 0x13					; make the read call
		jc READ_DISK_ERROR
		; Now copy bytes from WRITE_BUFFER TO TMP_WRITE_BUFFER at the specified offset
		mov bx, word[POINTER_TO_BUFFER_SEG_NUM]
		mov es, bx
		mov bx, word[POINTER_TO_BUFFER_OFF+di]
		push di
		mov di, 0
		.copy_loop:
			mov al, byte[es:bx+di]
			add di, word[WRITE_START_OFFSET_IN_BLOCK]
			mov byte[ds:TMP_WRITE_BUFFER+di], al
			sub di, word[WRITE_START_OFFSET_IN_BLOCK]
			inc di
			cmp di, word[NUM_BYTES_TO_WRITE_CURRENT_BLOCK]
			jnz .copy_loop
		pop di

		mov ah, 0x03					; function number of INT13h for writing to disk
		mov al, 1					; we write 1 sector at a time
		mov ch, byte[WRITE_START_AT_CYL_NUM]		; low 8 bits of cyllinder no
		mov cl, byte[ds:WRITE_START_AT_CYL_NUM+1]	; cyl no high two bits
		shl cl, 4					; high two bits of cylinder no at 6,7 bits of cl
		shl cl, 2					; high two bits of cyl no should be at bits 6-7 of cl
		add cl, byte[WRITE_START_AT_SECTOR_NUM]		; sector no in bits 0-5 of cl
		mov dh, byte[WRITE_START_AT_HEAD_NUM]		; head no in DH
		mov dl, 0x80					; drive no, first=80h and so on
		mov bx, ds
		mov es, bx
		mov bx, word TMP_WRITE_BUFFER			; pass buffer pointer in ES:BX
		int 0x13					; make the call
		jc WRITE_DISK_ERROR_FROM_PARTIAL

		mov word[WRITE_START_OFFSET_IN_BLOCK], 0
		mov ax, word[NUM_BYTES_TO_WRITE_CURRENT_BLOCK]
		sub word[REMAINING_BYTES_TO_WRITE], ax
		add di, ax
		cmp word[REMAINING_BYTES_TO_WRITE], 0
		jz FINISH_WRITING
		cmp word[REMAINING_BYTES_TO_WRITE], 512
		jle .last_block_remaining
		mov word[NUM_BYTES_TO_WRITE_CURRENT_BLOCK], 512
		jmp SELECT_MODE_OF_WRITING
		.last_block_remaining:
			mov ax, word[REMAINING_BYTES_TO_WRITE]
			mov word[NUM_BYTES_TO_WRITE_CURRENT_BLOCK], ax
			jmp SELECT_MODE_OF_WRITING

	READ_DISK_ERROR:
		cmp byte[NUM_OF_TRIES], 0
		jz EXIT_ON_ERROR
		dec byte[NUM_OF_TRIES]
		mov ah, 0					; Disk reset function
		mov dl, 0x80					; Drive no starts at 0x80
		int 0x13					; make disk reset call
		jmp READ_ONE_BLOCK_INTO_TMP_BUF

	WRITE_DISK_ERROR:

	pusha
	mov si, word sys_call_handler_msg
	mov cx, word[sys_call_handler_msg_size]
	call printString
	popa

		cmp byte[NUM_OF_TRIES], 0
		jz EXIT_ON_ERROR
		dec byte[NUM_OF_TRIES]
		mov ah, 0					; Disk reset function
		mov dl, 0x80					; Drive no starts at 0x80
		int 0x13					; make disk reset call
		jmp WRITE_ONE_BLOCK

	WRITE_DISK_ERROR_FROM_PARTIAL:
		cmp byte[NUM_OF_TRIES], 0
		jz EXIT_ON_ERROR
		dec byte[NUM_OF_TRIES]
		mov ah, 0					; Disk reset function
		mov dl, 0x80					; Drive no starts at 0x80
		int 0x13					; make disk reset call
		jmp READ_ONE_BLOCK_INTO_TMP_BUF

	DISK_ACCESS_LIMIT_REACHED:
		jmp EXIT_ON_ERROR

	EXIT_ON_ERROR:
		popa
		mov ax, 0					; write successful
		ret

	FINISH_WRITING:
		popa
		mov ax, 1					; write successful
		ret

	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	;; Local Routine Data
	;;
		WRITE_START_AT_CYL_NUM			DW	0x0000
		WRITE_START_AT_HEAD_NUM			DB	0x00
		WRITE_START_AT_SECTOR_NUM		DB	0x00
		WRITE_START_OFFSET_IN_BLOCK		DW	0x0000
		NUM_BYTES_TO_WRITE			DW	0x0000
		REMAINING_BYTES_TO_WRITE		DW	0x0000
		NUM_SECTORS_TRYING_TO_WRITE		DB	0x00

		WRITE_OFFSET_CURRENT_BLOCK		DW	0x0000
		NUM_BYTES_TO_WRITE_CURRENT_BLOCK	DW	0x00
		NUM_OF_TRIES				DB	0x02

		; data is first copied to this buffer then after some processing written to the disk, since data is
		; written block size at once, so size of this buffer is 1 block = 1 sector
		TMP_WRITE_BUFFER			TIMES	(512)	DB	0x00

		;; Disk Parameters have been set in read_Disk
	;;
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; I_O_Manager: handles I/O requests open, read, write, seek, close
;;
I_O_Manager:
	pusha

	cmp word[SYS_CALL_NO], 0
	jz openFileOrDevice

	cmp word[SYS_CALL_NO], 1
	jz readFileOrDevice

	cmp word[SYS_CALL_NO], 2
	jz writeFileOrDevice

	cmp word[SYS_CALL_NO], 3
	jz seekFileOrDevice

	cmp word[SYS_CALL_NO], 4
	jz closeFileOrDevice


	openFileOrDevice:
		mov ax, word[SYS_CALL_ARGS_ADD_SEG]
		mov es, ax
		mov bx, word[SYS_CALL_ARGS_ADD_OFF]
		mov ax, word[es:bx]
		mov word[FILE_OR_DEVICE_TO_OPEN], ax	; path of file to open or device descriptor num if IS_DEVICE flag is set
		mov ax, word[es:bx+2]
		mov word[OPEN_MODE], ax			; mode to open file or device
		mov ax, word[es:bx+4]
		mov word[OPEN_FLAGS], ax		; flags

		xor ax, ax
		mov al, byte[OPEN_FLAGS]
		shl al, 5
		shr al, 6				; 2nd least significant bit is IS_DEVICE flag
		cmp al, 1
		jz openDevice				; jump to openDevice if IS_DEVICE flag is set, otherwise it's a normal file
		jmp openFile

	openFile:
		; locate file on the filesystem tree
		; right now we are not implementing this

		; find the lowest descriptor available
		call find_Free_FD_Entry
		cmp di, word[MAX_NUM_OF_FDS]
		jz OPEN_FAILURE					; di=MAX_NUM_OF_FDS indicates FDT for the current process is full, so open can't continue

		; otherwise we have the lowest available FD in di, next we populate the FDT
		mov bx, word[CURRENT_PROC_PCB]
		mov ax, word[ds:bx+PCB.pointer_to_FDT]
		mov bx, ax
		; all files are right now on the root device, so set the device descriptor to 0, root device's device descriptor
		mov word[ds:bx+13*di+FDT.device_desc], 0

		push bx
		push di
		mov bx, word[DEV_TABLE]
		mov di, ax
		mov ax, word[ds:bx+di*4+DDT.handler_module]
		pop di
		pop bx
		mov word[ds:bx+13*di+FDT.pointer_to_handler_module], ax

		mov ax, 0
		mov word[ds:bx+13*di+FDT.offset_into_file], ax

		; find first block of file
		; @@@@@@@@@@@@@ to be implemented @@@@@@@@@@@@@@@@
		mov word[ds:bx+13*di+FDT.pointer_to_first_block], ax

		; get pointer to dir containing the file
		mov word[ds:bx+13*di+FDT.pointer_to_dir], ax

		; get file size
		mov word[ds:bx+13*di+FDT.file_size], ax

		mov ax, word[OPEN_MODE]
		mov word[ds:bx+13*di+FDT.attr], ax

		mov ax, word[OPEN_FLAGS]
		mov word[ds:bx+13*di+FDT.status_flags], ax

		jmp OPEN_SCCS

	openDevice:
		; find the lowest descriptor available
		call find_Free_FD_Entry
		cmp di, word[MAX_NUM_OF_FDS]
		jz OPEN_FAILURE					; di=MAX_NUM_OF_FDS indicates FDT for the current process is full, so open can't continue

		; otherwise we have the lowest available FD in di, next we populate the FDT
		mov bx, word[CURRENT_PROC_PCB]
		mov ax, word[ds:bx+PCB.pointer_to_FDT]
		mov bx, ax
		mov ax, word[FILE_OR_DEVICE_TO_OPEN]
		mov word[ds:bx+13*di+FDT.device_desc], ax

		; get handler module address from the device descriptor table
		push bx
		push di
		mov bx, word[DEV_TABLE]
		mov di, ax
		mov ax, word[ds:bx+di*4+DDT.handler_module]
		pop di
		pop bx
		mov word[ds:bx+13*di+FDT.pointer_to_handler_module], ax

		; since the file is just opened, it should be zero
		mov ax, 0
		mov word[ds:bx+13*di+FDT.offset_into_file], ax

		mov word[ds:bx+13*di+FDT.pointer_to_first_block], ax

		mov ax, word[OPEN_MODE]
		or al, 10000000b					; the 8th bit should be on if the file is a device
		mov word[ds:bx+13*di+FDT.attr], ax

		mov ax, word[OPEN_FLAGS]
		mov word[ds:bx+13*di+FDT.status_flags], ax

		jmp OPEN_SCCS						; its done

	find_Free_FD_Entry:
		mov bx, word[CURRENT_PROC_PCB]
		mov ax, word[ds:bx+PCB.pointer_to_FDT]
		mov bx, ax
		mov di, 0
		.loop:
			cmp word[ds:bx+13*di], 0x0000
			jnz .next_it
			cmp word[ds:bx+13*di+2], 0x0000
			jnz .next_it
			cmp word[ds:bx+13*di+4], 0x0000
			jnz .next_it
			cmp word[ds:bx+13*di+6], 0x0000
			jnz .next_it
			cmp word[ds:bx+13*di+8], 0x0000
			jnz .next_it
			cmp word[ds:bx+13*di+10], 0x0000
			jnz .next_it
			cmp byte[ds:bx+13*di+12], 0x00
			jnz .next_it
			ret					; found a free FDT entry, so return, value of di < MAX_NUM_OF_FDS or >= 0
			.next_it:
				inc di
				cmp di, word[MAX_NUM_OF_FDS]
				jnz .loop
				ret				; didn't find a free FDT entry, return with a value of di=MAX_NUM_OF_FDS


	OPEN_FAILURE:
		mov bx, word[CURRENT_PROC_PCB]
		mov word[ds:bx+PCB.syscall_ret_value], -1
		popa
		ret

	OPEN_SCCS:
		mov bx, word[CURRENT_PROC_PCB]
		mov word[ds:bx+PCB.syscall_ret_value], di
		popa
		ret


	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	;;
	;;
		FDT	TIMES	200	DB	0x00
	;;
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; tty_driver:
;;
tty_driver:



	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	;; tty_read: Device Driver for writing to terminal device
	;;
	tty_read:

	;;
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	;; write_To_Terminal: Device Driver for writing to terminal device
	;;
	tty_write:
		pusha
	
		;; Get the syscall parameters first
		xor ax, ax
		mov ax, word[SYS_CALL_ARGS_ADD_SEG]
		mov es, ax
		mov bx, word[SYS_CALL_ARGS_ADD_OFF]
		xor cx, cx
		mov cx, word[es:bx]			;; no of characters in string
		mov ax, word[es:bx+2]			;; segment no of address of string, will be passed in ES
		mov si, word[es:bx+4]			;; offset part of the address of the string, will be passed in BP
	
		mov es, ax				;; string address to be passed in ES:BP
	
		mov word[SAVE_BP], bp			;; save bp
		mov bp, si				;; pass string add offset in BP
		mov ah, 0x13				;; function no for the BIOS 10h INT for writing string to video memory
		mov al, 0x01				;; bit 0: update cursor after writing
							;; bit 1: string contains alternating characters and attributes
							;; bits 2-7: reserved (0)
		mov bl, 01000000b			;; high four bits for background color, low four bits for foreground color,
							;; current setting black chars on green background
		mov bh, 0x00				;; video page no
		mov dh, byte[CURR_CURSOR_ROW]		;; get cursor position after last write
		mov dl, byte[CURR_CURSOR_COL]		;;
		int 0x10				;; finally make the call
	
		mov bp,	word[SAVE_BP]			;; restore bp
	
		;get current cursor position and save it
		mov ah, 0x03				;; function no for getting cursor position
		mov bh, 0x00				;; video page no
		int 0x10
		mov byte[CURR_CURSOR_ROW], dh
		mov byte[CURR_CURSOR_COL], dl
	
		popa
	
		ret
	
	
	;;
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;




;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;








;##################################################################################################################################################################;
;##																				 ##;
;##					 set of routines for managing a File Allocation Blocks(FABS) FileSystem							 ##;
;##																				 ##;
;##################################################################################################################################################################;
;;
;; Services Provided by the FABS Driver:
;;
;;	1. QUERY(PATH_TO_FILE,QUERY_CMD,QUERY_PARAMS). Returns the answer to a particular query or returns -1 on error.
;;
;;	2. READ(FIRST_BLOCK_OF_FILE, OFFSET_IN_BYTES_WHERE_TO_START_READ, NUMBER_OF_BYTES_TO_READ, BUFFER_POINTER). Returns -1 if failed to read,
;;	   else return no of bytes read.
;;
;;	3. WRITE(FILE_DESCRIPTOR, OFFSET_IN_BYTES_TO_START_WRITE, NUMBER_OF_BYTES_TO_WRITE, BUFFER_POINTER), return no of bytes written.
;;
;;	4. CREATE(PATH_OF_THE_FILE_TO_CREATE, SPACE_IN_BYTES_TO_BE_ALLOCATED_INITIALLY), returns size of the file if successful else 0 is returned.
;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;






;###################################################################################################################################################################




;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; hexPrint: print 16 bit data as hex strings, pass the data to be printed in the BX register
;;
hexPrint:
	pusha

	mov si, word hex_prefix
	xor cx, cx
	mov cx, 2
	call printString
; 	mov bx, 0xabcd
	mov di, 1		; counter for num characters left to print
	mov ax, bx		; we need two copies
	xor ebx, ebx		; just cleaning
	.mainloop:
		mov bx, ax
		mov cx, di
		.shift_left_loop:
			shl ax, 4
			dec cx
			jnz .shift_left_loop

		mov cx, di
		.shift_right_loop:
			shr ax, 4
			dec cx
			jnz .shift_right_loop

		sub bx, ax	; now bx contains only the char to be printed but at a high psition, we need to bring it down
		mov cx, 4
		sub cx, di
		jz .printHexChar
		.bring_down_loop:
			shr bx, 4
			dec cx
			jnz .bring_down_loop

		.printHexChar:
			lea si, [HEX_CHARS+bx]
			xor cx, cx
			mov cx, 1
			call printString

		inc di
		cmp di, 5
		jnz .mainloop

	mov si, word newline
	xor cx, cx
	mov cx, 2
	call printString

	popa
	ret

	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	;; local routine data
	;;
		hex_prefix	DB	"0x"
		newline		DB	0x0D, 0x0A
		HEX_CHARS	DB	"0","1","2","3","4","5","6","7","8","9","A","B","C","D","E","F"
	;;
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;




;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; printString: PRINT MESSAGES ON THE TERMINAL
;;
printString:
	pusha
	mov ax, __KERNEL_LOAD_ADDRESS__
	mov es, ax
	mov word[SAVE_BP], bp	;; save bp
	mov bp, si
	mov ah, 0x13
	mov al, 0x01
	mov bl, 01000000b
	mov bh, 0x00
	mov dh, [CURR_CURSOR_ROW]
	mov dl, [CURR_CURSOR_COL]
	int 0x10
	; get current cursor position and save it
	mov ah, 0x03
	mov bh, 0x00
	int 0x10
	mov [CURR_CURSOR_ROW], dh
	mov [CURR_CURSOR_COL], dl
	mov bp, word[SAVE_BP]	;; restore bp
	popa
	ret
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;setPageColor:
;;
setPageColor:
	pusha
	xor eax, eax
	mov ax, __KERNEL_LOAD_ADDRESS__
	mov es, ax
	mov word[SAVE_BP], bp	;; save bp
	mov bp, blank_char
	mov cx, 1
	mov ah, 0x13
	mov al, 0x01
	mov bl, 01000000b
	mov bh, 0x00
	mov dh, 0x00
	mov dl, 0x00
	loop:
	int 0x10
	inc dl
	cmp dl, 81
	jnz loop

	mov dl, 0x00
	inc dh
	cmp dh, 26
	jnz loop
	mov dh, 0x00
	mov dl, 0x00
	mov bp, word[SAVE_BP]	;; restore bp
	popa
	ret

;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;





;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; DATA SECTION
;;

	blank_char			DB	" "
	welcome_msg 			DB 	"Welcome To My Kernel.", 0x0D, 0x0A
	welcome_msg_size		DW	($-welcome_msg)
	sys_call_handler_msg		DB	"sysCallHandler", 0x0D, 0x0A
	sys_call_handler_msg_size	DW	($-sys_call_handler_msg)
	read_msg 			DB 	"read_SysCall", 0x0D, 0x0A
	read_msg_size			DB	14
	write_msg 			DB 	"write_SysCall", 0x0D, 0x0A
	write_msg_size			DB	15
	fork_msg 			DB 	"fork_SysCall", 0x0D, 0x0A
	fork_msg_size			DW	14
	execve_msg 			DB 	"execve_SysCall", 0x0D, 0x0A
	execve_msg_size			DB	16
	
	return_address			DW	0x0000
	ret_add_seg			DW	0x0000
	ret_add_off			DW	0x0000
	
	KERNEL_INITIALIZED		DB	0X00
	SYS_CALL_NO			DW	0X0000
	SYS_CALL_ARGS_ADD_SEG		DW	0x0000
	SYS_CALL_ARGS_ADD_OFF		DW	0x0000
	CURR_CURSOR_ROW			DB	0X00
	CURR_CURSOR_COL			DB	0X00

	SAVE_BP				DW	0x0000

	counter_0			DB	0x00
	
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	;; The system-call table, which defines which handler to use for a particular call
	;;
		SYS_CALL_TABLE		TIMES		5	DD 	0x00000000
	;;
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;



	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	;; Memory Management Data
	;;
		struc MEM_BLOCK
			.mem_block_seg_num		RESW	1
			.mem_block_off			RESW	1
			.next_block_seg_num		RESW	1
			.next_block_off			RESW	1
		endstruc

		USR_PROC_ADD_SPACE_START_SEGMENT_NUM		DW	0x0CF1
		USR_PROC_ADD_SPACE_START_OFFSET			DW	0x0000

		FREE_MEM_BLOCKS_LIST_HEAD_SEG_NUM		DW	0x0000
		FREE_MEM_BLOCKS_LIST_HEAD_OFF			DW	0x0000
		NUM_FREE_MEM_BLOCKS				DW	0x0000
		MEM_BLOCK_SIZE					DW	0x1000

		MEM_BLOCK_STRUC_SPACE:	\
					TIMES	(240*8)		DB	0x00
	;;
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;




	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	;; PCB Data
	;;
		CURRENT_PROC_PCB		DW	0x0000
		ALLOCATED_PCB_LIST_HEAD		DW	0x0000
		PCB_SIZE			DW	0x0017
		MAX_NUM_PCB			DW	0x000A
		FREE_PCB_LIST_HEAD		DW	PCB_MEM_SPACE
		NUM_FREE_PCB			DW	0x000A
	
		struc PCB
			.pid			RESW	1
			.ppid			RESW	1
			.state			RESB	1
			.next			RESW	1
			.prev			RESW	1
			.start_mem_seg		RESW	1
			.start_mem_off		RESW	1
			.mem_size		RESW	1
			.stack_ss		RESW	1
			.stack_sp		RESW	1
			.stack_bp		RESW	1
			.syscall_ret_value	RESW	1
			.pointer_to_FDT		RESW	1
; 			.pwd
		endstruc

		PCB_MEM_SPACE:	\
				TIMES		230	DB	0x00

	;;
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;




	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	;;
	;;
		struc FD
			.device_desc			RESB	1
			.pointer_to_handler_module	RESB	2		; handler module is the raw mode device driver not the
										; higher level handler like a file system module
			.offset_into_file		RESB	4
			.pointer_to_first_block		RESB	4
			.pointer_to_dir			RESB	4
			.file_size			RESB	4
			.attr				RESB	2
			.status_flags			RESB	1
		endstruc

		FDT_ENTRY_SIZE		DB	13
		MAX_NUM_OF_FDS		DW	10
		FDT_SIZE		DW	130				; 10 files per process can be simultaneously opened
		MAX_NUM_OF_FDTS		DW	10

		FREE_FDT_LIST		DW	10				; an array of free FDTs, if an FDT is used up we store
										; zeros in its entry otherwise the entry contains address of the FDT

		FDT_MEM_SPACE:	\
				TIMES		(130*10)	DB	0x00	; a process is allowed to open at most 10 files at once,
										; and our system allows 10 procs at a time, so we need 1200 bytes.

	;;
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;




	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
	;; DDT(Device Descriptor Table) contains information about Registered Devices
	;;
		struc DDT
			.type			RESB	1
			.id			RESB	1
			.handler_module		RESB	2
		endstruc

		DD_SIZE		DW	4


		DEV_TABLE:	\

				; device descriptor 0, it is a special device descriptor,
				; since we always register the root device (device containing
				; the root partition) here

				DB	0x00			; device type for hard disk is 0
				DB	0x80			; device identifier for first hard disk
				DW	GENERIC_DISK_DRIVER


				; device descriptor 1
				DB	0x01			; device type for physical terminal is 1
				DB	0x00			; device identifier for physical terminal
				DW	TTY_DIRECT_ACCESS_DRIVER


				; device descriptor 2
				DB	0x02			; device type for virtual terminal is 2
				DB	0x01			; device identifier for first virtual terminal
				DW	PSUDO_TTY_DRIVER


				; device descriptor 3
				DB	0x02			; device type for virtual terminal is 2
				DB	0x02			; device identifier for 2nd virtual terminal
				DW	PSUDO_TTY_DRIVER

				; device descriptor 4
				DB	0x02			; device type for virtual terminal is 2
				DB	0x03			; device identifier for 3rd virtual terminal
				DW	PSUDO_TTY_DRIVER


				; device descriptor 5
				DB	0x02			; device type for virtual terminal is 2
				DB	0x04			; device identifier for 4th virtual terminal
				DW	PSUDO_TTY_DRIVER


; 		DDT_MEM_SPACE:	\
; 				TIMES		230	DB	0x00

	;;
	;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;



;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;




;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; init binary is attached here
;;
	initProgCode:	\
			INCBIN	"init.bin"

;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
