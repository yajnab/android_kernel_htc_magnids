# MSM7x01A
   zreladdr-$(CONFIG_ARCH_MSM7X01A)	:= 0x10008000
params_phys-$(CONFIG_ARCH_MSM7X01A)	:= 0x10000100
initrd_phys-$(CONFIG_ARCH_MSM7X01A)	:= 0x10800000

# MSM7x25
   zreladdr-$(CONFIG_ARCH_MSM7X25)	:= 0x00208000
params_phys-$(CONFIG_ARCH_MSM7X25)	:= 0x00200100
initrd_phys-$(CONFIG_ARCH_MSM7X25)	:= 0x0A000000

# MSM7x27
   zreladdr-$(CONFIG_ARCH_MSM7X27)	:= 0x00208000
params_phys-$(CONFIG_ARCH_MSM7X27)	:= 0x00200100
initrd_phys-$(CONFIG_ARCH_MSM7X27)	:= 0x0A000000

# MSM7x27A
   zreladdr-$(CONFIG_ARCH_MSM7X27A)	:= 0x00208000
params_phys-$(CONFIG_ARCH_MSM7X27A)	:= 0x00200100

# MSM8625
   zreladdr-$(CONFIG_ARCH_MSM8625)	:= 0x00208000
params_phys-$(CONFIG_ARCH_MSM8625)	:= 0x00200100

   zreladdr-$(CONFIG_MACH_PROTOU)	:= 0x03208000
params_phys-$(CONFIG_MACH_PROTOU)	:= 0x03200100
initrd_phys-$(CONFIG_MACH_PROTOU)	:= 0x04200000

   zreladdr-$(CONFIG_MACH_PROTODUG)	:= 0x03208000
params_phys-$(CONFIG_MACH_PROTODUG)	:= 0x03200100
initrd_phys-$(CONFIG_MACH_PROTODUG)	:= 0x04200000

   zreladdr-$(CONFIG_MACH_PROTODCG)	:= 0x03208000
params_phys-$(CONFIG_MACH_PROTODCG)	:= 0x03200100
initrd_phys-$(CONFIG_MACH_PROTODCG)	:= 0x04200000

   zreladdr-$(CONFIG_MACH_MAGNIDS)	:= 0x03208000
params_phys-$(CONFIG_MACH_MAGNIDS)	:= 0x03200100
initrd_phys-$(CONFIG_MACH_MAGNIDS)	:= 0x04200000

# MSM7x30
   zreladdr-$(CONFIG_ARCH_MSM7X30)	:= 0x00208000
params_phys-$(CONFIG_ARCH_MSM7X30)	:= 0x00200100
initrd_phys-$(CONFIG_ARCH_MSM7X30)	:= 0x01200000

ifeq ($(CONFIG_MSM_SOC_REV_A),y)
# QSD8x50
   zreladdr-$(CONFIG_ARCH_QSD8X50)	:= 0x20008000
params_phys-$(CONFIG_ARCH_QSD8X50)	:= 0x20000100
initrd_phys-$(CONFIG_ARCH_QSD8X50)	:= 0x24000000
endif

# MSM8x60
   zreladdr-$(CONFIG_ARCH_MSM8X60)	:= 0x40208000

# MSM8960
   zreladdr-$(CONFIG_ARCH_MSM8960)	:= 0x80208000

# MSM8930
   zreladdr-$(CONFIG_ARCH_MSM8930)	:= 0x80208000

# APQ8064
   zreladdr-$(CONFIG_ARCH_APQ8064)	:= 0x80208000

# MSMCOPPER
   zreladdr-$(CONFIG_ARCH_MSMCOPPER)	:= 0x20208000

# MSM9615
   zreladdr-$(CONFIG_ARCH_MSM9615)	:= 0x40808000

# FSM9XXX
   zreladdr-$(CONFIG_ARCH_FSM9XXX)	:= 0x10008000
params_phys-$(CONFIG_ARCH_FSM9XXX)	:= 0x10000100
initrd_phys-$(CONFIG_ARCH_FSM9XXX)	:= 0x12000000
