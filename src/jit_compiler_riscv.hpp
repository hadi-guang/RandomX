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
	; zero-> 
	; ra  -> return address
	; sp  -> stack pointer
	; gp
	; tp
	
	; t0  -> regfile
	; t1  -> memory registers "ma" (high 32 bits), "mx" (low 32 bits)
	; t2  -> scratchpad pointer
	; t3  -> program_iterations
	; t4  -> registers	"spAddr1" (high 32 bits), "spAddr0" (low 32 bits)
	; t5  -> RANDOMX_SCRATCHPAD_MASK << 32 + RANDOMX_SCRATCHPAD_MASK
	; t6  -> dataset pointer

	; s0	TMP0									need save
	; s1  -> L1M									need save
	; s2  -> L2M									need save
	; s3  -> L3M									need save
	; s4  -> TMP1									need save
	; s5  -> TMP2									need save
	; s6  -> 	need save
	; s7  -> 	need save
	; s8  -> E 'and' mask  =	0x00ffffffffffffff	need save
	; s9  -> E 'or' mask low=	0x3*00000000******	need save
	; s10 -> E 'or' mask high=	0x3*00000000******	need save
	; s11 -> scale mask = 0x81f0000000000000		need save
	
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
	
	; fs2 -> "al0"		need save
	; fs3 -> "al1"		need save
	; fs4 -> "al2"		need save
	; fs5 -> "al3"		need save
	; fs6 -> "ah0"		need save
	; fs7 -> "ah1"		need save
	; fs8 -> "ah2"		need save
	; fs9 -> "ah3"		need save
	
	; ft8  ->
	; ft9  ->
	; ft10 ->
	; ft11 ->
	; fs8  ->			need save
	; fs9  ->			need save
	; fs10 ->			need save
	; fs11 ->			need save
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
#define	RISCV_R_S0		(8)		// TMP0
#define	RISCV_R_S1		(9)		// L1M
#define	RISCV_R_A0		(10)	// r0 - r7
#define	RISCV_R_A1		(11)	// r0 - r7
#define	RISCV_R_A2		(12)	// r0 - r7
#define	RISCV_R_A3		(13)	// r0 - r7
#define	RISCV_R_A4		(14)	// r0 - r7
#define	RISCV_R_A5		(15)	// r0 - r7
#define	RISCV_R_A6		(16)	// r0 - r7
#define	RISCV_R_A7		(17)	// r0 - r7
#define	RISCV_R_S2		(18)	// L2M
#define	RISCV_R_S3		(19)	// L3M
#define	RISCV_R_S4		(20)	// TMP1
#define	RISCV_R_S5		(21)	// TMP2
#define	RISCV_R_S6		(22)
#define	RISCV_R_S7		(23)
#define	RISCV_R_S8		(24)	// E 'and' mask
#define	RISCV_R_S9		(25)	// E 'or' mask low
#define	RISCV_R_S10		(26)	// E 'or' mask high
#define	RISCV_R_S11		(27)	// scale mask
#define	RISCV_R_T3		(28)	//program_iterations
#define	RISCV_R_T4		(29)	
#define	RISCV_R_T5		(30)	
#define	RISCV_R_T6		(31)	

#define RISCV_EL0		(0)		//"el0"
#define RISCV_EL1		(1)		//"el1"
#define RISCV_EL2		(2)		//"el2"
#define RISCV_EL3		(3)		//"el3"
#define RISCV_EH0		(4)		//"eh0"
#define RISCV_EH1		(5)		//"eh1"
#define RISCV_EH2		(6)		//"eh2"
#define RISCV_EH3		(7)		//"eh3"
#define RISCV_FL0		(10)	//"fl0"
#define RISCV_FL1		(11)	//"fl1"
#define RISCV_FL2		(12)	//"fl2"
#define RISCV_FL3		(13)	//"fl3"
#define RISCV_FH0		(14)	//"fh0"
#define RISCV_FH1		(15)	//"fh1"
#define RISCV_FH2		(16)	//"fh2"
#define RISCV_FH3		(17)	//"fh3"
#define RISCV_AL0		(18)	//"al0"
#define RISCV_AL1		(19)	//"al1"
#define RISCV_AL2		(20)	//"al2"
#define RISCV_AL3		(21)	//"al3"
#define RISCV_AH0		(22)	//"ah0"
#define RISCV_AH1		(23)	//"ah1"
#define RISCV_AH2		(24)	//"ah2"
#define RISCV_AH3		(25)	//"ah3"

#define RISCV_FT0		(0)		//"el0"
#define RISCV_FT1		(1)		//"el1"
#define RISCV_FT2		(2)		//"el2"
#define RISCV_FT3		(3)		//"el3"
#define RISCV_FT4		(4)		//"eh0"
#define RISCV_FT5		(5)		//"eh1"
#define RISCV_FT6		(6)		//"eh2"
#define RISCV_FT7		(7)		//"eh3"
#define RISCV_FS0		(8)
#define RISCV_FS1		(9)
#define RISCV_FA0		(10)	//"fl0"
#define RISCV_FA1		(11)	//"fl1"
#define RISCV_FA2		(12)	//"fl2"
#define RISCV_FA3		(13)	//"fl3"
#define RISCV_FA4		(14)	//"fh0"
#define RISCV_FA5		(15)	//"fh1"
#define RISCV_FA6		(16)	//"fh2"
#define RISCV_FA7		(17)	//"fh3"
#define RISCV_FS2		(18)	//"al0"
#define RISCV_FS3		(19)	//"al1"
#define RISCV_FS4		(20)	//"al2"
#define RISCV_FS5		(21)	//"al3"
#define RISCV_FS6		(22)	//"ah0"
#define RISCV_FS7		(23)	//"ah1"
#define RISCV_FS8		(24)	//"ah2"
#define RISCV_FS9		(25)	//"ah3"
#define RISCV_FS10		(26)
#define RISCV_FS11		(27)
#define RISCV_FT8		(28)
#define RISCV_FT9		(29)
#define RISCV_FT10		(30)
#define RISCV_FT11		(31)

	
	typedef enum
	{
#if 1//RV32I RV64I
		RISCVOP_LUI_U 		= 0b0110111,// U
		RISCVOP_AUIPC_U		= 0b0010111,// U
		RISCVOP_JAL_J 		= 0b1101111,// J
		RISCVOP_JALR_I		= 0b1100111,// I
		RISCVOP_BRANCH_B	= 0b1100011,// B
		RISCVOP_LOAD_I		= 0b0000011,// I
		RISCVOP_STORE_I		= 0b0100011,// S
		RISCVOP_IMM_I 		= 0b0010011,// I
		RISCVOP_OP_R		= 0b0110011,// R
		RISCVOP_MISCMEM_I 	= 0b0001111,// I
		RISCVOP_SYSTEM_I	= 0b1110011,// I
#endif
#if 1// RV64I
		RISCVOP_IMM32_I		= 0b0011011,// I
		RISCVOP_OP32_R		= 0b0111011,// R
#endif
#if 1//RV32M RV64M
		//RISCVOP_OP_R		= 0b0110011,
#endif
#if 1//RV64M
		//RISCVOP_OP32_R	= 0b0111011,
#endif
#if 1//RV32A RV64A
		RISCVOP_AMO_R 		= 0b0101111,// R
#endif
#if 1//RV64A
		//RISCVOP_AMO_R 	= 0b0101111,
#endif
#if 1//RV32F RV64F
		RISCVOP_LOADFP_I	= 0b0000111,// I
		RISCVOP_STOREFP_S 	= 0b0100111,// S
		RISCVOP_MADD_R4		= 0b1000011,// R4
		RISCVOP_MSUB_R4		= 0b1000111,// R4
		RISCVOP_FP_R		= 0b1010011,// R
#endif
#if 1//RV64F
		//RISCVOP_FP_R		= 0b1010011,
#endif
#if 1//RV32D RV64D
		//RISCVOP_LOADFP_I	= 0b0000111,
		//RISCVOP_STOREFP_S = 0b0100111,
		//RISCVOP_MADD_R4	= 0b1000011,
		//RISCVOP_MSUB_R4	= 0b1000111,
		RISCVOP_NMSUB_R4	= 0b1001011,// R4
		RISCVOP_NMADD_R4	= 0b1001111,// R4
		//RISCVOP_FP_R		= 0b1010011,
#endif
#if 1//RV64D
		//RISCVOP_FP_R		= 0b1010011,
#endif
#if 1//RV32Q RV64Q
		//RISCVOP_LOADFP_I	= 0b0000111,
		//RISCVOP_STOREFP_S = 0b0100111,
		//RISCVOP_MADD_R4	= 0b1000011,
		//RISCVOP_MSUB_R4	= 0b1000111,
		//RISCVOP_NMSUB_R4	= 0b1001011,
		//RISCVOP_NMADD_R4	= 0b1001111,
		//RISCVOP_FP_R		= 0b1010011,
#endif
#if 1//RV64Q
		//RISCVOP_FP_R		= 0b1010011,
#endif
		RISCVOP_CUSTOM0 	= 0b0001011,		
		RISCVOP_CUSTOM1 	= 0b0101011,
		RISCVOP_CUSTOM2 	= 0b1011011,
		RISCVOP_CUSTOM3 	= 0b1111011,		
		RISCVOP_RESERVED1	= 0b1010111,
		RISCVOP_RESERVED2	= 0b1101011,
		RISCVOP_RESERVED3	= 0b1110111,
	}RISCVOP;
	typedef enum
	{
#if 1 //RV32I RV64I
		RISCVF3_JALR_JALR	= 0b000,

		RISCVF3_BRANCH_BEQ	= 0b000,
		RISCVF3_BRANCH_BNE	= 0b001,
		RISCVF3_BRANCH_BLT	= 0b100,
		RISCVF3_BRANCH_BGE	= 0b101,
		RISCVF3_BRANCH_BLTU= 0b110,
		RISCVF3_BRANCH_BGEU= 0b111,

		RISCVF3_LOAD_LB	= 0b000,
		RISCVF3_LOAD_LH	= 0b001,
		RISCVF3_LOAD_LW	= 0b010,
		RISCVF3_LOAD_LBU	= 0b100,
		RISCVF3_LOAD_LHU	= 0b101,

		RISCVF3_STORE_SB	= 0b000,
		RISCVF3_STORE_SH	= 0b001,
		RISCVF3_STORE_SW	= 0b010,

		RISCVF3_IMM_ADDI	= 0b000,
		RISCVF3_IMM_SLTI	= 0b010,
		RISCVF3_IMM_SLTIU	= 0b011,
		RISCVF3_IMM_XORI	= 0b100,
		RISCVF3_IMM_ORI		= 0b110,
		RISCVF3_IMM_ANDI	= 0b111,
		RISCVF3_IMM_SLLI_7	= 0b001,// need func7
		RISCVF3_IMM_SRLI_7	= 0b101,// need func7
		RISCVF3_IMM_SRAI_7	= 0b101,// need func7

		RISCVF3_OP_ADD		= 0b000,
		RISCVF3_OP_SUB		= 0b000,
		RISCVF3_OP_SLL		= 0b001,
		RISCVF3_OP_SLT		= 0b010,
		RISCVF3_OP_SLTU		= 0b011,
		RISCVF3_OP_XOR		= 0b100,
		RISCVF3_OP_SRL		= 0b101,
		RISCVF3_OP_SRA		= 0b101,
		RISCVF3_OP_OR		= 0b110,
		RISCVF3_OP_AND		= 0b111,

		RISCVF3_MISCMEM_FENCE= 0b000,
		RISCVF3_SYSTEM_ECALL= 0b000,
		RISCVF3_SYSTEM_EBREAK= 0b000,
#endif
#if 1//RV64I
		RISCVF3_LOAD_LWU	= 0b110,
		RISCVF3_LOAD_LD	= 0b011,
//		RISCVF3_LOAD_SD
//		
#endif
#if 1//RV32M RV64M
		RISCVF3_OP_MUL		= 0b000,
		RISCVF3_OP_MULH	= 0b001,
		RISCVF3_OP_MULHSU	= 0b010,
		RISCVF3_OP_MULHU	= 0b011,
		RISCVF3_OP_DIV		= 0b100,
		RISCVF3_OP_DIVU	= 0b101,
		RISCVF3_OP_REM		= 0b110,
		RISCVF3_OP_REMU	= 0b111,
#endif
#if 1//RV32D RV64D
		RISCVF3_LOADFP_FLD	= 0b011,
		RISCVF3_STOREFP_FSD= 0b011,
//		RISCVF3_MADD_FMADD_RM=
//		RISCVF3_FP_FADD_RM	= 0,
		RISCVF3_FP_FSGNJD_7	=0b000,
		RISCVF3_FP_FSGNJND_7=0b001,
		RISCVF3_FP_FSGNJXD_7=0b010,
		RISCVF3_FP_FMIND	=0b000,
		RISCVF3_FP_FMAXD	=0b001,

		RISCVF3_RM_RNE		= 0b000,// round to nearst,tie to even
		RISCVF3_RM_RTZ		= 0b001,// round towards zero
		RISCVF3_RM_RDN		= 0b010,// round down
		RISCVF3_RM_RUP		= 0b011,// round up
		RISCVF3_RM_RMM		= 0b100,// round to nearst,tie to max magnitude
		RISCVF3_RM_DYN		= 0b111,// synamic
#endif
#if 1//RV64D
		RISCVF3_FP_FMVXD_27	= 0b000,
		RISCVF3_FP_FMVDX_27	= 0b000,
#endif

	}RISCVF3;
	typedef enum
	{
		RISCVE7_OP_ADD		= 0b0000000,
		RISCVE7_OP_SUB		= 0b0100000,
		RISCVE7_OP_SLL		= 0b0000000,
		RISCVE7_OP_SLT		= 0b0000000,
		RISCVE7_OP_SLTU		= 0b0000000,
		RISCVE7_OP_XOR		= 0b0000000,
		RISCVE7_OP_SRL		= 0b0000000,
		RISCVE7_OP_SRA		= 0b0100000,
		RISCVE7_OP_OR		= 0b0000000,
		RISCVE7_OP_AND		= 0b0000000,

		RISCVE7_OP_MUL		= 0b0000001,
		RISCVE7_OP_MULH		= 0b0000001,
		RISCVE7_OP_MULHSU	= 0b0000001,
		RISCVE7_OP_MULHU	= 0b0000001,
		RISCVE7_OP_DIV		= 0b0000001,
		RISCVE7_OP_DIVU		= 0b0000001,
		RISCVE7_OP_REM		= 0b0000001,
		RISCVE7_OP_REMU		= 0b0000001,

		RISCVE7_IMM_SLLI	= 0b0000000,
		RISCVE7_IMM_SRLI	= 0b0000000,
		RISCVE7_IMM_SRAI	= 0b0100000,
#if 1//RV32D
		RISCVE7_FP_FSGNJD_7		= 0b0010001,
		RISCVE7_FP_FSGNJND_7	= 0b0010001,
		RISCVE7_FP_FSGNJXD_7	= 0b0010001,

		RISCVE7_FP_FADDD		= 0b0000001,
		RISCVE7_FP_FSUBD		= 0b0000101,
		RISCVE7_FP_FMULD		= 0b0001001,
		RISCVE7_FP_FDIVD		= 0b0001101,
#endif
#if 1//RV64D
		RISCVE2_FP_FMVXD_27		= 0b00000,
		RISCVE7_FP_FMVXD_27		= 0b1110001,
		RISCVE2_FP_FMVDX_27		= 0b00000,
		RISCVE7_FP_FMVDX_27		= 0b1111001,
#endif
	}RISCVE;


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

		uint32_t i32;
		uint32_t imm32;

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
