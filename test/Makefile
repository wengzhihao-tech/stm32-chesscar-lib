# 在 PC 上编译并运行 common/ 算法层的自测用例。
# common/ 不依赖任何芯片寄存器，所以可以直接用主机 gcc 验证算法正确性，
# 这比每次烧进单片机看 OLED 快得多。
#
#   make run        运行全部自测（基础模块 + 赛题专用模块）
#   make run-base   只跑基础模块（PID/编码器/探头/协议）
#   make run-match  只跑赛题专用模块（位姿/决策/导航/状态机/视觉）
CC      := gcc
CFLAGS  := -std=c99 -Wall -Wextra -I../common
LDFLAGS := -lm

BASE_SRC := ../common/pid_controller.c \
            ../common/encoder_math.c \
            ../common/line_sensor.c \
            ../common/line_follow.c \
            ../common/link_protocol.c

MATCH_SRC := ../common/pose_estimator.c \
             ../common/chess_target.c \
             ../common/navigator.c \
             ../common/match_fsm.c \
             ../common/vision_chess.c \
             ../common/link_protocol.c

all: selftest selftest_match

selftest: selftest.c $(BASE_SRC)
	$(CC) $(CFLAGS) -o $@ selftest.c $(BASE_SRC) $(LDFLAGS)

selftest_match: selftest_match.c $(MATCH_SRC)
	$(CC) $(CFLAGS) -o $@ selftest_match.c $(MATCH_SRC) $(LDFLAGS)

run: run-base run-match

run-base: selftest
	./selftest

run-match: selftest_match
	./selftest_match

clean:
	rm -f selftest selftest_match

.PHONY: all run run-base run-match clean
