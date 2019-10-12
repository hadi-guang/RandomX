##	load reg from scratchpad

	
# get spAddr0 -> s5
	addw	s5,zero,t4
	add		s4,	t2,s5

## load randomx r0-r7 from scratchpad
	ld		s5,	0(s4)
	xor		a0,a0,s5
	ld		s5,	8(s4)
	xor		a1,a1,s5
	ld		s5,	16(s4)
	xor		a2,a2,s5
	ld		s5,	24(s4)
	xor		a3,a3,s5
	ld		s5,	32(s4)
	xor		a4,a4,s5
	ld		s5,	40(s4)
	xor		a5,a5,s5
	ld		s5,	48(s4)
	xor		a6,a6,s5
	ld		s5,	56(s4)
	xor		a7,a7,s5

# get spAddr1 -> s5
	srli	s5,	t4,32
	add		s4,	t2,s5

## load randomx f0-f3
# fl0 fh0
	ld		s5,	0(s4)
	fcvt.d.w	fa0,s5		# randomx fl0
	srai		s5,s5,32
	fcvt.d.w	fa4,s5		# randomx fh0
# fl1 fh1
	ld		s5,	8(s4)
	fcvt.d.w	fa1,s5		# randomx fl1
	srai		s5,s5,32
	fcvt.d.w	fa5,s5		# randomx fh1
# fl2 fh2
	ld		s5,	16(s4)
	fcvt.d.w	fa2,s5		# randomx fl2
	srai		s5,s5,32
	fcvt.d.w	fa6,s5		# randomx fh2
# fl3 fh3
	ld		s5,	24(s4)
	fcvt.d.w	fa3,s5		# randomx fl3
	srai		s5,s5,32
	fcvt.d.w	fa7,s5		# randomx fh3
	
## load randomx e0-e3
# el0 eh0
	ld		s5,	32(s4)
	fcvt.d.w	ft0,s5		# randomx el0
	srai		s5,s5,32
	fcvt.d.w	ft4,s5		# randomx eh0
# el1 eh1
	ld		s5,	40(s4)
	fcvt.d.w	ft1,s5		# randomx el1
	srai		s5,s5,32
	fcvt.d.w	ft5,s5		# randomx eh1
# el2 eh2
	ld		s5,	48(s4)
	fcvt.d.w	ft2,s5		# randomx el2
	srai		s5,s5,32
	fcvt.d.w	ft6,s5		# randomx eh2
# el3 eh3
	ld		s5,	56(s4)
	fcvt.d.w	ft3,s5		# randomx el3
	srai		s5,s5,32
	fcvt.d.w	ft7,s5		# randomx eh3
## mask e0-e3
# el0 eh0
	fmv.x.d		s5,ft0
	and			s5,s5,s8
	or			s5,s5,s9
	fmv.d.x		ft0,s5
	fmv.x.d		s5,ft4
	and			s5,s5,s8
	or			s5,s5,s10
	fmv.d.x		ft4,s5
# el1 eh1
	fmv.x.d		s5,ft1
	and			s5,s5,s8
	or			s5,s5,s9
	fmv.d.x		ft1,s5
	fmv.x.d		s5,ft5
	and			s5,s5,s8
	or			s5,s5,s10
	fmv.d.x		ft5,s5
# el2 eh2
	fmv.x.d		s5,ft2
	and			s5,s5,s8
	or			s5,s5,s9
	fmv.d.x		ft2,s5
	fmv.x.d		s5,ft6
	and			s5,s5,s8
	or			s5,s5,s10
	fmv.d.x		ft6,s5
# el3 eh3
	fmv.x.d		s5,ft3
	and			s5,s5,s8
	or			s5,s5,s9
	fmv.d.x		ft3,s5
	fmv.x.d		s5,ft7
	and			s5,s5,s8
	or			s5,s5,s10
	fmv.d.x		ft7,s5

