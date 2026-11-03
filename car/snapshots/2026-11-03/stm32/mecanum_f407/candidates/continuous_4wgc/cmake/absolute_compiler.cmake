# The legacy toolchain resets compiler names before its nested project().
# Resolve those names here without editing the managed legacy toolchain.
find_program(CONTINUOUS_ARM_CC arm-none-eabi-gcc REQUIRED NO_CMAKE_FIND_ROOT_PATH)
set(CMAKE_C_COMPILER ${CONTINUOUS_ARM_CC} CACHE FILEPATH "Isolated candidate ARM compiler" FORCE)
set(CMAKE_ASM_COMPILER ${CONTINUOUS_ARM_CC} CACHE FILEPATH "Isolated candidate ASM compiler" FORCE)
