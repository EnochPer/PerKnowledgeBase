#!/usr/bin/env python3
"""学习状态专用入口；参数与 mvg_pdf.py 的 status/set 保持一致。"""
import sys

sys.dont_write_bytecode = True
from mvg_pdf import parser

if __name__ == "__main__":
    if len(sys.argv) < 2 or sys.argv[1] not in ("status", "set"):
        sys.exit("用法：mvg_state.py status | set 原书页码 [--topic 已整理主题] [--mastered 经用户确认的主题]")
    arguments = parser().parse_args()
    arguments.func(arguments)
