savedcmd_controller_kernel.mod := printf '%s\n'   controller_kernel.o | awk '!x[$$0]++ { print("./"$$0) }' > controller_kernel.mod
