/*
Copyright (c) 2018-2019, tevador <tevador@gmail.com>

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
	* Redistributions of source code must retain the above copyright
	  notice, this list of conditions and the following disclaimer.
	* Redistributions in binary form must reproduce the above copyright
	  notice, this list of conditions and the following disclaimer in the
	  documentation and/or other materials provided with the distribution.
	* Neither the name of the copyright holder nor the
	  names of its contributors may be used to endorse or promote products
	  derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#include <cstdint>
#include <cstring>
#include <vector>
#include "common.hpp"

namespace randomx {

	/*

	REGISTER ALLOCATION:
	; ra  -> return address
	; sp  -> stack pointer
	
	; t0  -> regfile
	; t1  -> memory registers "ma" (high 32 bits), "mx" (low 32 bits)
	; t2  -> scratchpad pointer
	; t3  -> program_iterations
	; t4  -> tmp1
	; t5  -> tmp2
	; t6  -> n64
	; s1  -> L1M		need save
	; s2  -> L2M		need save
	; s3  -> L3M		need save
	; s4  ->
	; s5  ->
	; s6  ->
	; s7  ->
	; s8  ->
	; s9  ->
	; s10 ->
	; s11 ->
	
	; a0  -> "r0"
	; a1  -> "r1"
	; a2  -> "r2"
	; a3  -> "r3"
	; a4  -> "r4"
	; a5  -> "r5"
	; a6  -> "r6"
	; a7  -> "r7"


	; fa0 -> "fl0"
	; fa1 -> "fl1"
	; fa2 -> "fl2"
	; fa3 -> "fl3"
	; fa4 -> "fh0"
	; fa5 -> "fh1"
	; fa6 -> "fh2"
	; fa7 -> "fh3"
	
	; ft0 -> "el0"
	; ft1 -> "el1"
	; ft2 -> "el2"
	; ft3 -> "el3"
	; ft4 -> "eh0"
	; ft5 -> "eh1"
	; ft6 -> "eh2"
	; ft7 -> "eh3"
	
	; fs0 -> "al0"		need save
	; fs1 -> "al1"		need save
	; fs2 -> "al2"		need save
	; fs3 -> "al3"		need save
	; fs4 -> "ah0"		need save
	; fs5 -> "ah1"		need save
	; fs6 -> "ah2"		need save
	; fs7 -> "ah3"		need save
	
	; ft8  -> temporary
	; ft9  -> E 'and' mask = 0x00ffffffffffffff00ffffffffffffff
	; ft10 -> E 'or' mask  = 0x3*00000000******3*00000000******
	; ft11 -> scale mask   = 0x81f000000000000081f0000000000000
	; fs8  -> temporary												need save
	; fs9  -> E 'and' mask = 0x00ffffffffffffff00ffffffffffffff		need save
	; fs10 -> E 'or' mask  = 0x3*00000000******3*00000000******		need save
	; fs11 -> scale mask   = 0x81f000000000000081f0000000000000		need save
	/////////////////////////////////////////////////////////////////////////////////////
	// x86
	; rax -> temporary
	; rbx -> iteration counter "ic"
	; rcx -> temporary
	; rdx -> temporary
	; rsi -> scratchpad pointer
	; rdi -> dataset pointer
	; rbp -> memory registers "ma" (high 32 bits), "mx" (low 32 bits)
	; rsp -> stack pointer
	; r8  -> "r0"
	; r9  -> "r1"
	; r10 -> "r2"
	; r11 -> "r3"
	; r12 -> "r4"
	; r13 -> "r5"
	; r14 -> "r6"
	; r15 -> "r7"
	; xmm0 -> "f0"
	; xmm1 -> "f1"
	; xmm2 -> "f2"
	; xmm3 -> "f3"
	; xmm4 -> "e0"
	; xmm5 -> "e1"
	; xmm6 -> "e2"
	; xmm7 -> "e3"
	; xmm8 -> "a0"
	; xmm9 -> "a1"
	; xmm10 -> "a2"
	; xmm11 -> "a3"
	; xmm12 -> temporary
	; xmm13 -> E 'and' mask = 0x00ffffffffffffff00ffffffffffffff
	; xmm14 -> E 'or' mask  = 0x3*00000000******3*00000000******
	; xmm15 -> scale mask   = 0x81f000000000000081f0000000000000

	*/
#define RISCV_R_X0		(0b00000)
#define RISCV_R_ZERO	(0)
#define	RISCV_R_RA		(1)		//return address
#define	RISCV_R_SP		(2)		//stack pointer
#define	RISCV_R_GP		(3)
#define	RISCV_R_TP		(4)
#define	RISCV_R_T0		(5)		//	regfile
#define	RISCV_R_T1		(6)		//memory registers "ma" (high 32 bits), "mx" (low 32 bits)
#define	RISCV_R_T2		(7)		//scratchpad pointer
#define	RISCV_R_S0		(8)		
#define	RISCV_R_S1		(9)		//L1M
#define	RISCV_R_A0		(10)	//r0 - r7
#define	RISCV_R_A1		(11)	//r0 - r7
#define	RISCV_R_A2		(12)	//r0 - r7
#define	RISCV_R_A3		(13)	//r0 - r7
#define	RISCV_R_A4		(14)	//r0 - r7
#define	RISCV_R_A5		(15)	//r0 - r7
#define	RISCV_R_A6		(16)	//r0 - r7
#define	RISCV_R_A7		(17)	//r0 - r7
#define	RISCV_R_S2		(18)	//L2M
#define	RISCV_R_S3		(19)	//L3M
#define	RISCV_R_S4		(20)
#define	RISCV_R_S5		(21)
#define	RISCV_R_S6		(22)
#define	RISCV_R_S7		(23)
#define	RISCV_R_S8		(24)
#define	RISCV_R_S9		(25)
#define	RISCV_R_S10		(26)
#define	RISCV_R_S11		(27)
#define	RISCV_R_T3		(28)	//program_iterations
#define	RISCV_R_T4		(29)	//tmp1
#define	RISCV_R_T5		(30)	//tmp2
#define	RISCV_R_T6		(31)	//n64
	
	
	typedef enum
	{
		RISCVOP_LOAD	= 0b0000011,
		RISCVOP_LOADFP	= 0b0000111,
		RISCVOP_CUSTOM0 = 0b0001011,
		RISCVOP_MISCMEM = 0b0001111,
		RISCVOP_IMM 	= 0b0010011,
		RISCVOP_AUIPC	= 0b0010111,
		RISCVOP_IMM32	= 0b0011011,
		RISCVOP_STORE	= 0b0100011,
		RISCVOP_STOREFP = 0b0100111,
		RISCVOP_CUSTOM1 = 0b0101011,
		RISCVOP_AMO 	= 0b0101111,
		RISCVOP_OP		= 0b0110011,
		RISCVOP_LUI 	= 0b0110111,
		RISCVOP_OP32	= 0b0111011,
		RISCVOP_MADD	= 0b1000011,
		RISCVOP_MSUB	= 0b1000111,
		RISCVOP_NMSUB	= 0b1001011,
		RISCVOP_NMADD	= 0b1001111,
		RISCVOP_FP		= 0b1010011,
		RISCVOP_RESERVED1= 0b1010111,
		RISCVOP_CUSTOM2 = 0b1011011,	
		RISCVOP_BRANCH	= 0b1100011,
		RISCVOP_JALR	= 0b1100111,
		RISCVOP_RESERVED2= 0b1101011,
		RISCVOP_JAL 	= 0b1101111,
		RISCVOP_SYSTEM	= 0b1110011,
		RISCVOP_RESERVED3= 0b1110111,
		RISCVOP_CUSTOM3 = 0b1111011,	
	}RISCVOP;
	typedef enum
	{
		RISCVFUNC3_OP_XOR	=0b100,
	}RISVFUNC3;


	uint32_t mk_J(RISCVOP op_code,uint8_t rd,uint32_t imm_21);
	
	class Program;
	class ProgramConfiguration;
	class SuperscalarProgram;
	class JitCompilerRiscv;
	class Instruction;

	typedef void(JitCompilerRiscv::*InstructionGeneratorRiscv)(Instruction&, int);

	constexpr uint32_t CodeSize = 64 * 1024;

	class JitCompilerRiscv {
	public:
		JitCompilerRiscv();
		~JitCompilerRiscv();
		void generateProgram(Program&, ProgramConfiguration&);
		void generateProgramLight(Program&, ProgramConfiguration&, uint32_t);
		template<size_t N>
		void generateSuperscalarHash(SuperscalarProgram (&programs)[N], std::vector<uint64_t> &);
		void generateDatasetInitCode();
		ProgramFunc* getProgramFunc() {
			return (ProgramFunc*)code;
		}
		DatasetInitFunc* getDatasetInitFunc() {
			return (DatasetInitFunc*)code;
		}
		uint8_t* getCode() {
			return code;
		}
		size_t getCodeSize();
	private:
		static InstructionGeneratorRiscv engine[256];
		std::vector<int32_t> instructionOffsets;
		int registerUsage[RegistersCount];
		uint8_t* code;
		int32_t codePos;

		void generateProgramPrologue(Program&, ProgramConfiguration&);
		void generateProgramEpilogue(Program&);
		void genAddressReg(Instruction&, bool);
		void genAddressRegDst(Instruction&);
		void genAddressImm(Instruction&);
		void genSIB(int scale, int index, int base);

		void generateCode(Instruction&, int);
		void generateSuperscalarCode(Instruction &, std::vector<uint64_t> &);

		void emitByte(uint8_t val) {
			code[codePos] = val;
			codePos++;
		}

		void emit32(uint32_t val) {
			memcpy(code + codePos, &val, sizeof val);
			codePos += sizeof val;
		}

		void emit64(uint64_t val) {
			memcpy(code + codePos, &val, sizeof val);
			codePos += sizeof val;
		}

		template<size_t N>
		void emit(const uint8_t (&src)[N]) {
			emit(src, N);
		}

		void emit(const uint8_t* src, size_t count) {
			memcpy(code + codePos, src, count);
			codePos += count;
		}

		void h_IADD_RS(Instruction&, int);
		void h_IADD_M(Instruction&, int);
		void h_ISUB_R(Instruction&, int);
		void h_ISUB_M(Instruction&, int);
		void h_IMUL_R(Instruction&, int);
		void h_IMUL_M(Instruction&, int);
		void h_IMULH_R(Instruction&, int);
		void h_IMULH_M(Instruction&, int);
		void h_ISMULH_R(Instruction&, int);
		void h_ISMULH_M(Instruction&, int);
		void h_IMUL_RCP(Instruction&, int);
		void h_INEG_R(Instruction&, int);
		void h_IXOR_R(Instruction&, int);
		void h_IXOR_M(Instruction&, int);
		void h_IROR_R(Instruction&, int);
		void h_IROL_R(Instruction&, int);
		void h_ISWAP_R(Instruction&, int);
		void h_FSWAP_R(Instruction&, int);
		void h_FADD_R(Instruction&, int);
		void h_FADD_M(Instruction&, int);
		void h_FSUB_R(Instruction&, int);
		void h_FSUB_M(Instruction&, int);
		void h_FSCAL_R(Instruction&, int);
		void h_FMUL_R(Instruction&, int);
		void h_FDIV_M(Instruction&, int);
		void h_FSQRT_R(Instruction&, int);
		void h_CBRANCH(Instruction&, int);
		void h_CFROUND(Instruction&, int);
		void h_ISTORE(Instruction&, int);
		void h_NOP(Instruction&, int);
	};
}
