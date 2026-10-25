# 在 PC 上编译并运行 common/ 算法层的自测用例。
# common/ 不依赖任何芯片寄存器，所以可以直接用主机 gcc 验证算法正确性，
# 这比每次烧进单片机看 OLED 快得多。
#
#   make run
CC      := gcc
CFLAGS  := -std=c99 -Wall -Wextra -I../common
LDFLAGS := -lm
SRC     := selftest.c \
           ../common/pid_controller.c \
           ../common/encoder_math.c \
           ../common/line_sensor.c \
           ../common/line_follow.c \
           ../common/link_protocol.c

selftest: $(SRC)
	$(CC) $(CFLAGS) -o $@ $(SRC) $(LDFLAGS)

run: selftest
	./selftest

clean:
	rm -f selftest

.PHONY: run clean
