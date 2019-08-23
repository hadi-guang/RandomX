.section .data
#save_fs:
#.dword 0
#.dword 0
#.dword 0
#.dword 0
#.dword 0
#.dword 0
#.dword 0
#.dword 0
#.dword 0
#.dword 0
#.dword 0
#.dword 0
#
#save_s:
#.dword 0
#.dword 0
#.dword 0

floatarray:
.double 1.0
.double 2.0
.double 3.0
.double 4.0
.double 5.0
.double 6.0
.double 7.0
.double 8.0


.section .text
.global funa
funa:
#	for save fs0-fs11 s1-s3
	addi	sp,		sp,		-120

#	save fs regs
	fsd		fs0,	0x00(sp)
	fsd		fs1,	0x08(sp)
	fsd		fs2,	0x10(sp)
	fsd		fs3,	0x18(sp)
	fsd		fs4,	0x20(sp)
	fsd		fs5,	0x28(sp)
	fsd		fs6,	0x30(sp)
	fsd		fs7,	0x38(sp)
	fsd		fs8,	0x40(sp)
	fsd		fs9,	0x48(sp)
	fsd		fs10,	0x50(sp)
	fsd		fs11,	0x58(sp)
#	save s regs
	sd		s1,		0x60(sp)
	sd		s2,		0x68(sp)
	sd		s3,		0x70(sp)
######################################################################
#	save	args
	mv		t0,		a0
	mv		t1,		a1
	mv		t2,		a2
	mv		t3,		a3

######################################################################
#	load 	regs
	ld		a0,		0x00(t0)
	ld		a1,		0x08(t0)
	ld		a2,		0x10(t0)
	ld		a3,		0x18(t0)
	ld		a4,		0x20(t0)
	ld		a5,		0x28(t0)
	ld		a6,		0x30(t0)
	ld		a7,		0x38(t0)
	
	fld		fa0,	0x40(t0)
	fld		fa4,	0x48(t0)
	fld		fa1,	0x50(t0)
	fld		fa5,	0x58(t0)
	fld		fa2,	0x60(t0)
	fld		fa6,	0x68(t0)
	fld		fa3,	0x70(t0)
	fld		fa7,	0x78(t0)

	fld 	ft0,	0x80(t0)
	fld 	ft4,	0x88(t0)
	fld 	ft1,	0x90(t0)
	fld 	ft5,	0x98(t0)
	fld 	ft2,	0xA0(t0)
	fld 	ft6,	0xA8(t0)
	fld 	ft3,	0xB0(t0)
	fld 	ft7,	0xB8(t0)
	
	fld 	fs0,	0xC0(t0)
	fld 	fs4,	0xC8(t0)
	fld 	fs1,	0xD0(t0)
	fld 	fs5,	0xD8(t0)
	fld 	fs2,	0xE0(t0)
	fld 	fs6,	0xE8(t0)
	fld 	fs3,	0xF0(t0)
	fld 	fs7,	0xF8(t0)

######################################################################
#	run program
	li	a0,8
	li	a1,1
	li	a2,2
	li	a3,3
	li	a4,4
	li	a5,5
	li	a6,6
	li	a7,7

	lui t4,       %hi(floatarray)       # load msg(hi)
	addi t4, t4,  %lo(floatarray)
	
	fld		fa0,	0x00(t4)
	fld		fa4,	0x08(t4)
	fld		fa1,	0x10(t4)
	fld		fa5,	0x18(t4)
	fld		fa2,	0x20(t4)
	fld		fa6,	0x28(t4)
	fld		fa3,	0x30(t4)
	fld		fa7,	0x38(t4)

	fld 	ft0,	0x00(t4)
	fld 	ft4,	0x08(t4)
	fld 	ft1,	0x10(t4)
	fld 	ft5,	0x18(t4)
	fld 	ft2,	0x20(t4)
	fld 	ft6,	0x28(t4)
	fld 	ft3,	0x30(t4)
	fld 	ft7,	0x38(t4)
	
	fld 	fs0,	0x00(t4)
	fld 	fs4,	0x08(t4)
	fld 	fs1,	0x10(t4)
	fld 	fs5,	0x18(t4)
	fld 	fs2,	0x20(t4)
	fld 	fs6,	0x28(t4)
	fld 	fs3,	0x30(t4)
	fld 	fs7,	0x38(t4)


######################################################################
#	save 	regs
	sd		a0,		0x00(t0)
	sd		a1,		0x08(t0)
	sd		a2,		0x10(t0)
	sd		a3,		0x18(t0)
	sd		a4,		0x20(t0)
	sd		a5,		0x28(t0)
	sd		a6,		0x30(t0)
	sd		a7,		0x38(t0)
	
	fsd		fa0,	0x40(t0)
	fsd		fa4,	0x48(t0)
	fsd		fa1,	0x50(t0)
	fsd		fa5,	0x58(t0)
	fsd		fa2,	0x60(t0)
	fsd		fa6,	0x68(t0)
	fsd		fa3,	0x70(t0)
	fsd		fa7,	0x78(t0)

	fsd 	ft0,	0x80(t0)
	fsd 	ft4,	0x88(t0)
	fsd 	ft1,	0x90(t0)
	fsd 	ft5,	0x98(t0)
	fsd 	ft2,	0xA0(t0)
	fsd 	ft6,	0xA8(t0)
	fsd 	ft3,	0xB0(t0)
	fsd 	ft7,	0xB8(t0)

	fsd 	fs0,	0xC0(t0)
	fsd 	fs4,	0xC8(t0)
	fsd 	fs1,	0xD0(t0)
	fsd 	fs5,	0xD8(t0)
	fsd 	fs2,	0xE0(t0)
	fsd 	fs6,	0xE8(t0)
	fsd 	fs3,	0xF0(t0)
	fsd 	fs7,	0xF8(t0)


######################################################################
#	load fs regs
	fld		fs0,	0x00(sp)
	fld		fs1,	0x08(sp)
	fld		fs2,	0x10(sp)
	fld		fs3,	0x18(sp)
	fld		fs4,	0x20(sp)
	fld		fs5,	0x28(sp)
	fld		fs6,	0x30(sp)
	fld		fs7,	0x38(sp)
	fld		fs8,	0x40(sp)
	fld		fs9,	0x48(sp)
	fld		fs10,	0x50(sp)
	fld		fs11,	0x58(sp)
#	save s regs
	ld		s1,		0x60(sp)
	ld		s2,		0x68(sp)
	ld		s3,		0x70(sp)
#	
	addi	sp,		sp,		120
	ret
