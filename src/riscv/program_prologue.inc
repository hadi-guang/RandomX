;#	save callee-saved registers fs0-fs11 s1-s3
	addi	sp,	sp,	-176
	fsd	fs0,	0x00(sp)
	fsd	fs1,	0x08(sp)
	fsd	fs2,	0x10(sp)
	fsd	fs3,	0x18(sp)
	fsd	fs4,	0x20(sp)
	fsd	fs5,	0x28(sp)
	fsd	fs6,	0x30(sp)
	fsd	fs7,	0x38(sp)
#	fsd	fs8,	0x40(sp)
#	fsd	fs9,	0x48(sp)
#	fsd	fs10,	0x50(sp)
#	fsd	fs11,	0x58(sp)
	sd	s1,	0x60(sp)
	sd	s2,	0x68(sp)
	sd	s3,	0x70(sp)
	sd	s4,	0x78(sp)
	sd	s5,	0x80(sp)
	
	sd	s6,	0x88(sp)
	sd	s7,	0x90(sp)
	sd	s8,	0x98(sp)
	sd	s9,	0xA0(sp)
	sd	s10,0xA8(sp)

;#	function arguments
	mv	t0,	a0	;# regfile
	ld	t1,	0x00(a1)	;# memory registers "ma" (high 32 bits), "mx" (low 32 bits)
	ld	t6,	0x08(a1)	;# dataset pointer
	mv	t2,	a2	;# scratchpad pointer
	mv	t3,	a3	;# loop counter

;#	init
	mv	t4, t1	;# registers "spAddr1" "spAddr0" init use "ma" "mx"
	li	t5, (RANDOMX_SCRATCHPAD_MASK << 32) + RANDOMX_SCRATCHPAD_MASK

	li	s1,	RANDOMX_SCRATCHPAD_L1;
	li	s2,	RANDOMX_SCRATCHPAD_L2;
	li	s3,	RANDOMX_SCRATCHPAD_L3;
	

;#	zero integer registers
	mv	a0,	zero
	mv	a1,	zero
	mv	a2,	zero
	mv	a3,	zero
	mv	a4,	zero
	mv	a5,	zero
	mv	a6,	zero
	mv	a7,	zero

;#	load constant registers
#	ld	a0,	0x00(t0)
#	ld	a1,	0x08(t0)
#	ld	a2,	0x10(t0)
#	ld	a3,	0x18(t0)
#	ld	a4,	0x20(t0)
#	ld	a5,	0x28(t0)
#	ld	a6,	0x30(t0)
#	ld	a7,	0x38(t0)
	
#	fld	fa0,	0x40(t0)
#	fld	fa4,	0x48(t0)
#	fld	fa1,	0x50(t0)
#	fld	fa5,	0x58(t0)
#	fld	fa2,	0x60(t0)
#	fld	fa6,	0x68(t0)
#	fld	fa3,	0x70(t0)
#	fld	fa7,	0x78(t0)

#	fld 	ft0,	0x80(t0)
#	fld 	ft4,	0x88(t0)
#	fld 	ft1,	0x90(t0)
#	fld 	ft5,	0x98(t0)
#	fld 	ft2,	0xA0(t0)
#	fld 	ft6,	0xA8(t0)
#	fld 	ft3,	0xB0(t0)
#	fld 	ft7,	0xB8(t0)
	#load a0-a3
	fld 	fs0,	0xC0(t0)	;# a0 - a3
	fld 	fs4,	0xC8(t0)
	fld 	fs1,	0xD0(t0)
	fld 	fs5,	0xD8(t0)
	fld 	fs2,	0xE0(t0)
	fld 	fs6,	0xE8(t0)
	fld 	fs3,	0xF0(t0)
	fld 	fs7,	0xF8(t0)