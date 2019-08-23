#include "stdio.h"
#define RegistersCount  8
#define RegisterCountFlt  (RegistersCount / 2)

typedef uint32_t addr_t;
//using addr_t = uint32_t;

typedef uint64_t int_reg_t;

//using int_reg_t = uint64_t;

typedef struct fpu_reg_t {
	double lo;				//8BYTE
	double hi;				//8BYTE
}fpu_reg_t;


typedef struct MemoryRegisters {
	addr_t mx, ma;			//4+4=8BTYE
	uint8_t* memory;		//8BYTE
}MemoryRegisters;

//register file in little-endian byte order
typedef struct RegisterFile {//256BYTE
	int_reg_t r[RegistersCount];	//8*8    =64BYTE
	fpu_reg_t f[RegisterCountFlt];	//(8+8)*4=64BYTE
	fpu_reg_t e[RegisterCountFlt];	//(8+8)*8=64BYTE
	fpu_reg_t a[RegisterCountFlt];	//(8+8)*8=64BYTE
	float     t;
}RegisterFile;
void funa(RegisterFile *reg, MemoryRegisters *memreg, uint8_t* scratchpad/* scratchpad */, uint64_t program_iterations);
void zProgramFunc(RegisterFile *reg, MemoryRegisters *memreg, uint8_t* scratchpad/* scratchpad */, uint64_t program_iterations)
{
	uint64_t m;
	uint8_t	n;
	reg->r[0] = program_iterations;
	reg->f[0].lo = 1.0;
	reg->f[0].hi = 2.0;
	memreg->ma = 1;
	memreg->mx = 2;
	scratchpad[0] = 2;
	m = program_iterations;
	n = 255;
	reg->t = 3.0;
}
void funa();
int main()
{
	struct RegisterFile reg;
	uint8_t scratchpad[20];
	struct MemoryRegisters memreg;
	uint64_t program_iterations = 2048;
	funa(&reg,&memreg,&scratchpad,program_iterations);

	printf("reg:\n");
	printf("r:%d %d %d %d %d %d %d %d\n",reg.r[0],reg.r[1],reg.r[2],reg.r[3],reg.r[4],reg.r[5],reg.r[6],reg.r[7]);
	printf("f:%f %f %f %f %f %f %f %f\n",reg.f[0].lo,reg.f[0].hi,reg.f[1].lo,reg.f[1].hi,reg.f[2].lo,reg.f[2].hi,reg.f[3].lo,reg.f[3].hi);
	printf("e:%f %f %f %f %f %f %f %f\n",reg.e[0].lo,reg.e[0].hi,reg.e[1].lo,reg.e[1].hi,reg.e[2].lo,reg.e[2].hi,reg.e[3].lo,reg.e[3].hi);
	printf("a:%f %f %f %f %f %f %f %f\n",reg.a[0].lo,reg.a[0].hi,reg.a[1].lo,reg.a[1].hi,reg.a[2].lo,reg.a[2].hi,reg.a[3].lo,reg.a[3].hi);
	printf("\n");

	printf("memreg:\n");
	printf("ma:%d mx:%d\n",memreg.ma,memreg.mx);
	printf("\n");
	
	printf("scratchpad:\n");
	printf("%d %d %d %d %d %d %d %d\n",scratchpad[0],scratchpad[1],scratchpad[2],scratchpad[3],scratchpad[4],scratchpad[5],scratchpad[6],scratchpad[7]);
	printf("%d %d %d %d %d %d %d %d\n",scratchpad[8],scratchpad[9],scratchpad[10],scratchpad[11],scratchpad[12],scratchpad[13],scratchpad[14],scratchpad[15]);
	printf("%d %d %d %d %d\n",scratchpad[16],scratchpad[17],scratchpad[18],scratchpad[19],scratchpad[20]);
//	zProgramFunc(&reg,&memreg,&scratchpad,program_iterations);
	
//	printf("hello\n");
//	a();
//	printf("hello1\n");
	return 0;
}
