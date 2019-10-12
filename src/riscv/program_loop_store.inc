# get spAddr1 -> s5
	srli	s5,	s7,32
	add		s4,	t2,s5

## store randomx r0-r7 to scratchpad
	sd		a0,	0(s4)
	sd		a1,	8(s4)
	sd		a2,	16(s4)
	sd		a3,	24(s4)
	sd		a4,	32(s4)
	sd		a5,	40(s4)
	sd		a6,	48(s4)
	sd		a7,	56(s4)

# get spAddr0 -> s5
	addw	s5,zero,s7
	add		s4,	t2,s5

# reg f = f ^ e
# store randomx f0-f3 to scratchpad
	fmv.x.d	s5,fa0		# randomx fl0
	fmv.x.d	s0,ft0		# randomx el0
	xor			s5,s5,s0
	sd			s5,0(s4)
#	fmv.d.x	fa0,s5		# randomx fl0

	fmv.x.d	s5,fa4		# randomx fh0
	fmv.x.d	s0,ft4		# randomx eh0
	xor			s5,s5,s0
	sd			s5,8(s4)
#	fmv.d.x	fa4,s5		# randomx fh0

	fmv.x.d	s5,fa1		# randomx fl1
	fmv.x.d	s0,ft1		# randomx el1
	xor			s5,s5,s0
	sd			s5,16(s4)
#	fmv.d.x	fa1,s5		# randomx fl1

	fmv.x.d	s5,fa5		# randomx fh1
	fmv.x.d	s0,ft5		# randomx eh1
	xor			s5,s5,s0
	sd			s5,24(s4)
#	fmv.d.x	fa5,s5		# randomx fh1

	fmv.x.d	s5,fa2		# randomx fl2
	fmv.x.d	s0,ft2		# randomx el2
	xor			s5,s5,s0
	sd			s5,32(s4)
#	fmv.d.x	fa2,s5		# randomx fl2	

	fmv.x.d	s5,fa6		# randomx fh2
	fmv.x.d	s0,ft6		# randomx eh2
	xor			s5,s5,s0
	sd			s5,40(s4)
#	fmv.d.x	fa6,s5		# randomx fh2

	fmv.x.d	s5,fa3		# randomx fl3
	fmv.x.d	s0,ft3		# randomx el3
	xor			s5,s5,s0
	sd			s5,48(s4)
#	fmv.d.x	fa3,s5		# randomx fl3

	fmv.x.d	s5,fa7		# randomx fh3
	fmv.x.d	s0,ft7		# randomx eh3
	xor			s5,s5,s0
	sd			s5,56(s4)
#	fmv.d.x	fa7,s5		# randomx fh3
