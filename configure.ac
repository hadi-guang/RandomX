AC_INIT([randomx], [1.0], [zdhzdh102417@163.com])

#foreign	no gnu standards check
#gnu		Automake will check-as much as possible-for compliance to the GNU standards for packages
#gnits		Automake will check for compliance to the as-yet-unwritten Gnits standards
AM_INIT_AUTOMAKE([-Wall -Werror foreign])

AC_PROG_CC
AC_PROG_CXX
AM_PROG_AS
AM_PROG_AR
AC_CONFIG_HEADERS([config.h])

AC_PROG_LIBTOOL

AC_ARG_ENABLE([riscv],
[  --enable-riscv Turn on riscv],
[case "${enableval}" in
yes) riscv=true ;AC_MSG_NOTICE([select yes]);;
no) riscv=false ;AC_MSG_NOTICE([select no]);;
*) AC_MSG_ERROR([bad value ${enableval} for --enable-riscv]) ;;
esac],[riscv=false])
AM_CONDITIONAL([COND_RISCV], [test x$riscv = xtrue])

# Automake will require the sources fle declared with AC_LIBSOURCE
# in the directory specifed by this macro
# AC_CONFIG_LIBOBJ_DIR([])

# *.AM -> *.in -> Makefile
AC_CONFIG_FILES([
	Makefile
	src/Makefile
	])
AC_OUTPUT
