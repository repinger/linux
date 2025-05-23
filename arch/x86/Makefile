# SPDX-License-Identifier: GPL-2.0
# Unified Makefile for i386 and x86_64

# select defconfig based on actual architecture
ifeq ($(ARCH),x86)
  ifeq ($(shell uname -m | sed -e 's/i.86/i386/'),i386)
        KBUILD_DEFCONFIG := i386_defconfig
  else
        KBUILD_DEFCONFIG := x86_64_defconfig
  endif
else
        KBUILD_DEFCONFIG := $(ARCH)_defconfig
endif

ifdef CONFIG_CC_IS_GCC
RETPOLINE_CFLAGS	:= $(call cc-option,-mindirect-branch=thunk-extern -mindirect-branch-register)
RETPOLINE_VDSO_CFLAGS	:= $(call cc-option,-mindirect-branch=thunk-inline -mindirect-branch-register)
endif
ifdef CONFIG_CC_IS_CLANG
RETPOLINE_CFLAGS	:= -mretpoline-external-thunk
RETPOLINE_VDSO_CFLAGS	:= -mretpoline
endif
RETPOLINE_CFLAGS	+= $(call cc-option,-mindirect-branch-cs-prefix)

ifdef CONFIG_MITIGATION_RETHUNK
RETHUNK_CFLAGS		:= -mfunction-return=thunk-extern
RETHUNK_RUSTFLAGS	:= -Zfunction-return=thunk-extern
RETPOLINE_CFLAGS	+= $(RETHUNK_CFLAGS)
RETPOLINE_RUSTFLAGS	+= $(RETHUNK_RUSTFLAGS)
endif

export RETHUNK_CFLAGS
export RETHUNK_RUSTFLAGS
export RETPOLINE_CFLAGS
export RETPOLINE_RUSTFLAGS
export RETPOLINE_VDSO_CFLAGS

# For gcc stack alignment is specified with -mpreferred-stack-boundary,
# clang has the option -mstack-alignment for that purpose.
ifneq ($(call cc-option, -mpreferred-stack-boundary=4),)
      cc_stack_align4 := -mpreferred-stack-boundary=2
      cc_stack_align8 := -mpreferred-stack-boundary=3
else ifneq ($(call cc-option, -mstack-alignment=16),)
      cc_stack_align4 := -mstack-alignment=4
      cc_stack_align8 := -mstack-alignment=8
endif

# How to compile the 16-bit code.  Note we always compile for -march=i386;
# that way we can complain to the user if the CPU is insufficient.
REALMODE_CFLAGS	:= -std=gnu11 -m16 -g -Os -DDISABLE_BRANCH_PROFILING -D__DISABLE_EXPORTS \
		   -Wall -Wstrict-prototypes -march=i386 -mregparm=3 \
		   -fno-strict-aliasing -fomit-frame-pointer -fno-pic \
		   -mno-mmx -mno-sse $(call cc-option,-fcf-protection=none)

REALMODE_CFLAGS += -ffreestanding
REALMODE_CFLAGS += -fno-stack-protector
REALMODE_CFLAGS += -Wno-address-of-packed-member
REALMODE_CFLAGS += $(cc_stack_align4)
REALMODE_CFLAGS += $(CLANG_FLAGS)
ifdef CONFIG_CC_IS_CLANG
REALMODE_CFLAGS += -Wno-gnu
endif
export REALMODE_CFLAGS

# BITS is used as extension for files which are available in a 32 bit
# and a 64 bit version to simplify shared Makefiles.
# e.g.: obj-y += foo_$(BITS).o
export BITS

#
# Prevent GCC from generating any FP code by mistake.
#
# This must happen before we try the -mpreferred-stack-boundary, see:
#
#    https://gcc.gnu.org/bugzilla/show_bug.cgi?id=53383
#
KBUILD_CFLAGS += -mno-sse -mno-mmx -mno-sse2 -mno-3dnow -mno-avx
KBUILD_RUSTFLAGS += --target=$(objtree)/scripts/target.json
KBUILD_RUSTFLAGS += -Ctarget-feature=-sse,-sse2,-sse3,-ssse3,-sse4.1,-sse4.2,-avx,-avx2

#
# CFLAGS for compiling floating point code inside the kernel.
#
CC_FLAGS_FPU := -msse -msse2
ifdef CONFIG_CC_IS_GCC
# Stack alignment mismatch, proceed with caution.
# GCC < 7.1 cannot compile code using `double` and -mpreferred-stack-boundary=3
# (8B stack alignment).
# See https://gcc.gnu.org/bugzilla/show_bug.cgi?id=53383
#
# The "-msse" in the first argument is there so that the
# -mpreferred-stack-boundary=3 build error:
#
#  -mpreferred-stack-boundary=3 is not between 4 and 12
#
# can be triggered. Otherwise gcc doesn't complain.
CC_FLAGS_FPU += -mhard-float
CC_FLAGS_FPU += $(call cc-option,-msse -mpreferred-stack-boundary=3,-mpreferred-stack-boundary=4)
endif

ifeq ($(CONFIG_X86_KERNEL_IBT),y)
#
# Kernel IBT has S_CET.NOTRACK_EN=0, as such the compilers must not generate
# NOTRACK prefixes. Current generation compilers unconditionally employ NOTRACK
# for jump-tables, as such, disable jump-tables for now.
#
# (jump-tables are implicitly disabled by RETPOLINE)
#
#   https://gcc.gnu.org/bugzilla/show_bug.cgi?id=104816
#
KBUILD_CFLAGS += $(call cc-option,-fcf-protection=branch -fno-jump-tables)
KBUILD_RUSTFLAGS += -Zcf-protection=branch -Zno-jump-tables
else
KBUILD_CFLAGS += $(call cc-option,-fcf-protection=none)
endif

ifeq ($(CONFIG_X86_32),y)
        BITS := 32
        UTS_MACHINE := i386
        CHECKFLAGS += -D__i386__

        KBUILD_AFLAGS += -m32
        KBUILD_CFLAGS += -m32

        KBUILD_CFLAGS += -msoft-float -mregparm=3 -freg-struct-return

        # Never want PIC in a 32-bit kernel, prevent breakage with GCC built
        # with nonstandard options
        KBUILD_CFLAGS += -fno-pic

        # Align the stack to the register width instead of using the default
        # alignment of 16 bytes. This reduces stack usage and the number of
        # alignment instructions.
        KBUILD_CFLAGS += $(cc_stack_align4)

        # CPU-specific tuning. Anything which can be shared with UML should go here.
        include $(srctree)/arch/x86/Makefile_32.cpu
        KBUILD_CFLAGS += $(cflags-y)

        # temporary until string.h is fixed
        KBUILD_CFLAGS += -ffreestanding

    ifeq ($(CONFIG_STACKPROTECTOR),y)
        ifeq ($(CONFIG_SMP),y)
            KBUILD_CFLAGS += -mstack-protector-guard-reg=fs \
                             -mstack-protector-guard-symbol=__ref_stack_chk_guard
        else
            KBUILD_CFLAGS += -mstack-protector-guard=global
        endif
    endif
else
        BITS := 64
        UTS_MACHINE := x86_64
        CHECKFLAGS += -D__x86_64__

        KBUILD_AFLAGS += -m64
        KBUILD_CFLAGS += -m64

        # Align jump targets to 1 byte, not the default 16 bytes:
        KBUILD_CFLAGS += $(call cc-option,-falign-jumps=1)

        # Pack loops tightly as well:
        KBUILD_CFLAGS += $(call cc-option,-falign-loops=1)

        # Don't autogenerate traditional x87 instructions
        KBUILD_CFLAGS += -mno-80387
        KBUILD_CFLAGS += $(call cc-option,-mno-fp-ret-in-387)

        # By default gcc and clang use a stack alignment of 16 bytes for x86.
        # However the standard kernel entry on x86-64 leaves the stack on an
        # 8-byte boundary. If the compiler isn't informed about the actual
        # alignment it will generate extra alignment instructions for the
        # default alignment which keep the stack *mis*aligned.
        # Furthermore an alignment to the register width reduces stack usage
        # and the number of alignment instructions.
        KBUILD_CFLAGS += $(cc_stack_align8)

	# Use -mskip-rax-setup if supported.
	KBUILD_CFLAGS += $(call cc-option,-mskip-rax-setup)

        # FIXME - should be integrated in Makefile.cpu (Makefile_32.cpu)
        cflags-$(CONFIG_MK8)		+= -march=k8
        cflags-$(CONFIG_MPSC)		+= -march=nocona
        cflags-$(CONFIG_MCORE2)		+= -march=core2
        cflags-$(CONFIG_MATOM)		+= -march=bonnell
        ifeq ($(CONFIG_X86_64_VERSION),1)
          cflags-$(CONFIG_GENERIC_CPU)		+= -mtune=generic
          rustflags-$(CONFIG_GENERIC_CPU)	+= -Ztune-cpu=generic
        else
          cflags-$(CONFIG_GENERIC_CPU)		+= -march=x86-64-v$(CONFIG_X86_64_VERSION)
          rustflags-$(CONFIG_GENERIC_CPU)	+= -Ctarget-cpu=x86-64-v$(CONFIG_X86_64_VERSION)
        endif
        cflags-$(CONFIG_MK8SSE3)	+= -march=k8-sse3
        cflags-$(CONFIG_MK10) 		+= -march=amdfam10
        cflags-$(CONFIG_MBARCELONA) 	+= -march=barcelona
        cflags-$(CONFIG_MBOBCAT) 	+= -march=btver1
        cflags-$(CONFIG_MJAGUAR) 	+= -march=btver2
        cflags-$(CONFIG_MBULLDOZER) 	+= -march=bdver1
        cflags-$(CONFIG_MPILEDRIVER)	+= -march=bdver2 -mno-tbm
        cflags-$(CONFIG_MSTEAMROLLER) 	+= -march=bdver3 -mno-tbm
        cflags-$(CONFIG_MEXCAVATOR) 	+= -march=bdver4 -mno-tbm
        cflags-$(CONFIG_MZEN) 		+= -march=znver1
        cflags-$(CONFIG_MZEN2) 	+= -march=znver2
        cflags-$(CONFIG_MZEN3) 	+= -march=znver3
        cflags-$(CONFIG_MZEN4) 	+= -march=znver4
        cflags-$(CONFIG_MZEN5) 	+= -march=znver5
        cflags-$(CONFIG_MNATIVE_INTEL) += -march=native
        cflags-$(CONFIG_MNATIVE_AMD) 	+= -march=native -mno-tbm
        cflags-$(CONFIG_MNEHALEM) 	+= -march=nehalem
        cflags-$(CONFIG_MWESTMERE) 	+= -march=westmere
        cflags-$(CONFIG_MSILVERMONT) 	+= -march=silvermont
        cflags-$(CONFIG_MGOLDMONT) 	+= -march=goldmont
        cflags-$(CONFIG_MGOLDMONTPLUS) += -march=goldmont-plus
        cflags-$(CONFIG_MSANDYBRIDGE) 	+= -march=sandybridge
        cflags-$(CONFIG_MIVYBRIDGE) 	+= -march=ivybridge
        cflags-$(CONFIG_MHASWELL) 	+= -march=haswell
        cflags-$(CONFIG_MBROADWELL) 	+= -march=broadwell
        cflags-$(CONFIG_MSKYLAKE) 	+= -march=skylake
        cflags-$(CONFIG_MSKYLAKEX) 	+= -march=skylake-avx512
        cflags-$(CONFIG_MCANNONLAKE) 	+= -march=cannonlake
        cflags-$(CONFIG_MICELAKE) 	+= -march=icelake-client
        cflags-$(CONFIG_MCASCADELAKE) 	+= -march=cascadelake
        cflags-$(CONFIG_MCOOPERLAKE) 	+= -march=cooperlake
        cflags-$(CONFIG_MTIGERLAKE) 	+= -march=tigerlake
        cflags-$(CONFIG_MSAPPHIRERAPIDS) += -march=sapphirerapids
        cflags-$(CONFIG_MROCKETLAKE) 	+= -march=rocketlake
        cflags-$(CONFIG_MALDERLAKE) 	+= -march=alderlake
        cflags-$(CONFIG_MRAPTORLAKE) 	+= -march=raptorlake
        cflags-$(CONFIG_MMETEORLAKE) 	+= -march=meteorlake
        cflags-$(CONFIG_MEMERALDRAPIDS)	+= -march=emeraldrapids
        KBUILD_CFLAGS += $(cflags-y)

        rustflags-$(CONFIG_MK8)		+= -Ctarget-cpu=k8
        rustflags-$(CONFIG_MPSC)	+= -Ctarget-cpu=nocona
        rustflags-$(CONFIG_MCORE2)	+= -Ctarget-cpu=core2
        rustflags-$(CONFIG_MATOM)	+= -Ctarget-cpu=atom
        rustflags-$(CONFIG_MK8SSE3)	+= -Ctarget-cpu=k8-sse3
        rustflags-$(CONFIG_MK10) 		+= -Ctarget-cpu=amdfam10
        rustflags-$(CONFIG_MBARCELONA) 	+= -Ctarget-cpu=barcelona
        rustflags-$(CONFIG_MBOBCAT) 	+= -Ctarget-cpu=btver1
        rustflags-$(CONFIG_MJAGUAR) 	+= -Ctarget-cpu=btver2
        rustflags-$(CONFIG_MBULLDOZER) 	+= -Ctarget-cpu=bdver1
        rustflags-$(CONFIG_MPILEDRIVER)	+= -Ctarget-cpu=bdver2
        rustflags-$(CONFIG_MSTEAMROLLER) 	+= -Ctarget-cpu=bdver3
        rustflags-$(CONFIG_MEXCAVATOR) 	+= -Ctarget-cpu=bdver4
        rustflags-$(CONFIG_MZEN) 		+= -Ctarget-cpu=znver1
        rustflags-$(CONFIG_MZEN2) 	+= -Ctarget-cpu=znver2
        rustflags-$(CONFIG_MZEN3) 	+= -Ctarget-cpu=znver3
        rustflags-$(CONFIG_MZEN4) 	+= -Ctarget-cpu=znver4
        rustflags-$(CONFIG_MZEN5) 	+= -Ctarget-cpu=znver5
        rustflags-$(CONFIG_MNATIVE_INTEL) += -Ctarget-cpu=native
        rustflags-$(CONFIG_MNATIVE_AMD) 	+= -Ctarget-cpu=native
        rustflags-$(CONFIG_MNEHALEM) 	+= -Ctarget-cpu=nehalem
        rustflags-$(CONFIG_MWESTMERE) 	+= -Ctarget-cpu=westmere
        rustflags-$(CONFIG_MSILVERMONT) 	+= -Ctarget-cpu=silvermont
        rustflags-$(CONFIG_MGOLDMONT) 	+= -Ctarget-cpu=goldmont
        rustflags-$(CONFIG_MGOLDMONTPLUS) += -Ctarget-cpu=goldmont-plus
        rustflags-$(CONFIG_MSANDYBRIDGE) 	+= -Ctarget-cpu=sandybridge
        rustflags-$(CONFIG_MIVYBRIDGE) 	+= -Ctarget-cpu=ivybridge
        rustflags-$(CONFIG_MHASWELL) 	+= -Ctarget-cpu=haswell
        rustflags-$(CONFIG_MBROADWELL) 	+= -Ctarget-cpu=broadwell
        rustflags-$(CONFIG_MSKYLAKE) 	+= -Ctarget-cpu=skylake
        rustflags-$(CONFIG_MSKYLAKEX) 	+= -Ctarget-cpu=skylake-avx512
        rustflags-$(CONFIG_MCANNONLAKE) 	+= -Ctarget-cpu=cannonlake
        rustflags-$(CONFIG_MICELAKE) 	+= -Ctarget-cpu=icelake-client
        rustflags-$(CONFIG_MCASCADELAKE) 	+= -Ctarget-cpu=cascadelake
        rustflags-$(CONFIG_MCOOPERLAKE) 	+= -Ctarget-cpu=cooperlake
        rustflags-$(CONFIG_MTIGERLAKE) 	+= -Ctarget-cpu=tigerlake
        rustflags-$(CONFIG_MSAPPHIRERAPIDS) += -Ctarget-cpu=sapphirerapids
        rustflags-$(CONFIG_MROCKETLAKE) 	+= -Ctarget-cpu=rocketlake
        rustflags-$(CONFIG_MALDERLAKE) 	+= -Ctarget-cpu=alderlake
        rustflags-$(CONFIG_MRAPTORLAKE) 	+= -Ctarget-cpu=raptorlake
        rustflags-$(CONFIG_MMETEORLAKE) 	+= -Ctarget-cpu=meteorlake
        rustflags-$(CONFIG_MEMERALDRAPIDS)	+= -Ctarget-cpu=emeraldrapids
        KBUILD_RUSTFLAGS += $(rustflags-y)

        KBUILD_CFLAGS += -mno-red-zone
        KBUILD_CFLAGS += -mcmodel=kernel
        KBUILD_RUSTFLAGS += -Cno-redzone=y
        KBUILD_RUSTFLAGS += -Ccode-model=kernel
endif

#
# If the function graph tracer is used with mcount instead of fentry,
# '-maccumulate-outgoing-args' is needed to prevent a GCC bug
# (https://gcc.gnu.org/bugzilla/show_bug.cgi?id=42109)
#
ifdef CONFIG_FUNCTION_GRAPH_TRACER
  ifndef CONFIG_HAVE_FENTRY
	ACCUMULATE_OUTGOING_ARGS := 1
  endif
endif

ifeq ($(ACCUMULATE_OUTGOING_ARGS), 1)
	# This compiler flag is not supported by Clang:
	KBUILD_CFLAGS += $(call cc-option,-maccumulate-outgoing-args,)
endif

# Workaround for a gcc prelease that unfortunately was shipped in a suse release
KBUILD_CFLAGS += -Wno-sign-compare
#
KBUILD_CFLAGS += -fno-asynchronous-unwind-tables

# Avoid indirect branches in kernel to deal with Spectre
ifdef CONFIG_MITIGATION_RETPOLINE
  KBUILD_CFLAGS += $(RETPOLINE_CFLAGS)
  KBUILD_RUSTFLAGS += $(RETPOLINE_RUSTFLAGS)
  # Additionally, avoid generating expensive indirect jumps which
  # are subject to retpolines for small number of switch cases.
  # LLVM turns off jump table generation by default when under
  # retpoline builds, however, gcc does not for x86. This has
  # only been fixed starting from gcc stable version 8.4.0 and
  # onwards, but not for older ones. See gcc bug #86952.
  ifndef CONFIG_CC_IS_CLANG
    KBUILD_CFLAGS += -fno-jump-tables
  endif
endif

ifdef CONFIG_MITIGATION_SLS
  KBUILD_CFLAGS += -mharden-sls=all
endif

ifdef CONFIG_CALL_PADDING
PADDING_CFLAGS := -fpatchable-function-entry=$(CONFIG_FUNCTION_PADDING_BYTES),$(CONFIG_FUNCTION_PADDING_BYTES)
KBUILD_CFLAGS += $(PADDING_CFLAGS)
export PADDING_CFLAGS

PADDING_RUSTFLAGS := -Zpatchable-function-entry=$(CONFIG_FUNCTION_PADDING_BYTES),$(CONFIG_FUNCTION_PADDING_BYTES)
KBUILD_RUSTFLAGS += $(PADDING_RUSTFLAGS)
export PADDING_RUSTFLAGS
endif

KBUILD_LDFLAGS += -m elf_$(UTS_MACHINE)

ifdef CONFIG_X86_NEED_RELOCS
LDFLAGS_vmlinux := --emit-relocs --discard-none
else
LDFLAGS_vmlinux :=
endif

#
# The 64-bit kernel must be aligned to 2MB.  Pass -z max-page-size=0x200000 to
# the linker to force 2MB page size regardless of the default page size used
# by the linker.
#
ifdef CONFIG_X86_64
LDFLAGS_vmlinux += -z max-page-size=0x200000
endif


archscripts: scripts_basic
	$(Q)$(MAKE) $(build)=arch/x86/tools relocs

###
# Syscall table generation

archheaders:
	$(Q)$(MAKE) $(build)=arch/x86/entry/syscalls all

###
# Kernel objects

libs-y  += arch/x86/lib/

# drivers-y are linked after core-y
drivers-$(CONFIG_MATH_EMULATION) += arch/x86/math-emu/
drivers-$(CONFIG_PCI)            += arch/x86/pci/

# suspend and hibernation support
drivers-$(CONFIG_PM) += arch/x86/power/

drivers-$(CONFIG_VIDEO) += arch/x86/video/

####
# boot loader support. Several targets are kept for legacy purposes

boot := arch/x86/boot

BOOT_TARGETS = bzdisk fdimage fdimage144 fdimage288 hdimage isoimage

PHONY += bzImage $(BOOT_TARGETS)

# Default kernel to build
all: bzImage

# KBUILD_IMAGE specify target image being built
KBUILD_IMAGE := $(boot)/bzImage

bzImage: vmlinux
ifeq ($(CONFIG_X86_DECODER_SELFTEST),y)
	$(Q)$(MAKE) $(build)=arch/x86/tools posttest
endif
	$(Q)$(MAKE) $(build)=$(boot) $(KBUILD_IMAGE)
	$(Q)mkdir -p $(objtree)/arch/$(UTS_MACHINE)/boot
	$(Q)ln -fsn ../../x86/boot/bzImage $(objtree)/arch/$(UTS_MACHINE)/boot/$@

$(BOOT_TARGETS): vmlinux
	$(Q)$(MAKE) $(build)=$(boot) $@

PHONY += install
install:
	$(call cmd,install)

vdso-install-$(CONFIG_X86_64)		+= arch/x86/entry/vdso/vdso64.so.dbg
vdso-install-$(CONFIG_X86_X32_ABI)	+= arch/x86/entry/vdso/vdsox32.so.dbg
vdso-install-$(CONFIG_COMPAT_32)	+= arch/x86/entry/vdso/vdso32.so.dbg

archprepare: checkbin
checkbin:
ifdef CONFIG_MITIGATION_RETPOLINE
ifeq ($(RETPOLINE_CFLAGS),)
	@echo "You are building kernel with non-retpoline compiler." >&2
	@echo "Please update your compiler." >&2
	@false
endif
endif

ifdef CONFIG_UNWINDER_ORC
orc_hash_h := arch/$(SRCARCH)/include/generated/asm/orc_hash.h
orc_hash_sh := $(srctree)/scripts/orc_hash.sh
targets += $(orc_hash_h)
quiet_cmd_orc_hash = GEN     $@
      cmd_orc_hash = mkdir -p $(dir $@); \
		     $(CONFIG_SHELL) $(orc_hash_sh) < $< > $@
$(orc_hash_h): $(srctree)/arch/x86/include/asm/orc_types.h $(orc_hash_sh) FORCE
	$(call if_changed,orc_hash)
archprepare: $(orc_hash_h)
endif

archclean:
	$(Q)rm -rf $(objtree)/arch/i386
	$(Q)rm -rf $(objtree)/arch/x86_64

define archhelp
  echo  '* bzImage		- Compressed kernel image (arch/x86/boot/bzImage)'
  echo  '  install		- Install kernel using (your) ~/bin/$(INSTALLKERNEL) or'
  echo  '			  (distribution) /sbin/$(INSTALLKERNEL) or install to '
  echo  '			  $$(INSTALL_PATH) and run lilo'
  echo  ''
  echo  '  fdimage		- Create 1.4MB boot floppy image (arch/x86/boot/fdimage)'
  echo  '  fdimage144		- Create 1.4MB boot floppy image (arch/x86/boot/fdimage)'
  echo  '  fdimage288		- Create 2.8MB boot floppy image (arch/x86/boot/fdimage)'
  echo  '  hdimage		- Create a BIOS/EFI hard disk image (arch/x86/boot/hdimage)'
  echo  '  isoimage		- Create a boot CD-ROM image (arch/x86/boot/image.iso)'
  echo  '			  bzdisk/fdimage*/hdimage/isoimage also accept:'
  echo  '			  FDARGS="..."  arguments for the booted kernel'
  echo  '			  FDINITRD=file initrd for the booted kernel'

endef
