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

#include "vm_compiled.hpp"
#include "common.hpp"
#include "zlog.h"
namespace randomx {

	static_assert(sizeof(MemoryRegisters) == 2 * sizeof(addr_t) + sizeof(uintptr_t), "Invalid alignment of struct randomx::MemoryRegisters");
	static_assert(sizeof(RegisterFile) == 256, "Invalid alignment of struct randomx::RegisterFile");

	template<class Allocator, bool softAes>
	void CompiledVm<Allocator, softAes>::setDataset(randomx_dataset* dataset) {
		datasetPtr = dataset;
	}

	template<class Allocator, bool softAes>
	void CompiledVm<Allocator, softAes>::run(void* seed) {
		VmBase<Allocator, softAes>::generateProgram(seed);
		randomx_vm::initialize();
		compiler.generateProgram(program, config);
		mem.memory = datasetPtr->memory + datasetOffset;
		execute();
	}

	template<class Allocator, bool softAes>
	void CompiledVm<Allocator, softAes>::execute() {
		printf("ma:%d mx:%d\n",mem.ma,mem.mx);
		printf("RANDOMX_PROGRAM_ITERATIONS:%d\n",RANDOMX_PROGRAM_ITERATIONS);
		compiler.getProgramFunc()(reg, mem, scratchpad, RANDOMX_PROGRAM_ITERATIONS);
		printf("scratchpad:%p\n",scratchpad);
		p_hex8("reg\n",&reg,sizeof(reg));
		printf("f:%f %f %f %f %f %f %f %f \n"
			,reg.f[0].lo,reg.f[0].hi
			,reg.f[1].lo,reg.f[1].hi
			,reg.f[2].lo,reg.f[2].hi
			,reg.f[3].lo,reg.f[3].hi);
		printf("e:%f %f %f %f %f %f %f %f \n"
			,reg.e[0].lo,reg.e[0].hi
			,reg.e[1].lo,reg.e[1].hi
			,reg.e[2].lo,reg.e[2].hi
			,reg.e[3].lo,reg.e[3].hi);
	}

	template class CompiledVm<AlignedAllocator<CacheLineSize>, false>;
	template class CompiledVm<AlignedAllocator<CacheLineSize>, true>;
#if LINUX_MMAP
	template class CompiledVm<LargePageAllocator, false>;
	template class CompiledVm<LargePageAllocator, true>;
#endif
}