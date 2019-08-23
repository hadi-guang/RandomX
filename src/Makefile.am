AUTOMAKE_OPTIONS = subdir-objects

bin_PROGRAMS = ztest benchmark randomx-generator randomx-tests

ztest_SOURCES = ztest.c

benchmark_SOURCES = tests/benchmark.cpp
#tests/affinity.cpp
benchmark_LDADD = librandomx.la
#benchmark_LDFLAGS = -lpthread

randomx_generator_SOURCES = tests/code-generator.cpp
randomx_generator_LDADD = librandomx.la

randomx_tests_SOURCES = tests/tests.cpp
randomx_tests_LDADD	= librandomx.la

lib_LTLIBRARIES = librandomx.la
librandomx_la_SOURCES = aes_hash.cpp argon2_ref.c dataset.cpp soft_aes.cpp virtual_memory.cpp vm_interpreted.cpp \
	allocator.cpp assembly_generator_x86.cpp instruction.cpp randomx.cpp superscalar.cpp vm_compiled.cpp \
	vm_interpreted_light.cpp argon2_core.c blake2_generator.cpp instructions_portable.cpp reciprocal.c \
	virtual_machine.cpp vm_compiled_light.cpp blake2/blake2b.c bytecode_machine.cpp \
	jit_compiler_riscv.cpp jit_compiler_riscv_static.S
#	jit_compiler_x86.cpp jit_compiler_x86_static.S
	
#librandomx_la_CXXFLAGS = -maes